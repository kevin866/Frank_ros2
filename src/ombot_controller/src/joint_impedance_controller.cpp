#include <algorithm>
#include <numeric>
#include <sstream>

#include "joint_impedance_controller/joint_impedance_controller.hpp"  // <-- adjust to your path
#include "pluginlib/class_list_macros.hpp"

#include "kdl_parser/kdl_parser.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace ombot_controller
{

JointImpedanceController::JointImpedanceController() = default;

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) conf.names.push_back(j + "/effort");
  return conf;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const
{
  // Require position & velocity state interfaces (recommended).
  // If your hardware does not expose velocity, make a variant that only requests positions
  // and computes dq numerically (see comments).
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) conf.names.push_back(j + "/position");
  for (const auto & j : joint_names_) conf.names.push_back(j + "/velocity");
  return conf;
}

std::vector<hardware_interface::CommandInterface>
JointImpedanceController::on_export_reference_interfaces()
{
  const size_t N = joint_names_.size();
  reference_interfaces_.assign(2 * N, 0.0);

  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve(2 * N);

  // const std::string ctrl = "joint_impedance_controller";
  const std::string ctrl = get_node()->get_name();  // ✅ instance-safe


  for (size_t i = 0; i < N; ++i) {
    const auto &joint = joint_names_[i];

    // Swap: resource=controller_name, interface=joint/signal
    refs.emplace_back(ctrl, joint + "/position", &reference_interfaces_[i]);
    refs.emplace_back(ctrl, joint + "/velocity", &reference_interfaces_[N + i]);
  }

  // Optional: log for sanity
  for (auto &ci : refs) {
    RCLCPP_INFO(get_node()->get_logger(), "Exported ref: %s/%s",
                ci.get_name().c_str(), ci.get_interface_name().c_str());
  }
  return refs;
}


bool JointImpedanceController::on_set_chained_mode(bool chained)
{
  // We always command hardware efforts; record whether we should read upstream refs.
  chained_mode_ = chained;
  RCLCPP_INFO(get_node()->get_logger(), "Chained mode %s", chained ? "enabled" : "disabled");
  return true;
}


controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("Kp", {});
    auto_declare<std::vector<double>>("Kd", {});
    auto_declare<std::vector<double>>("effort_limits", {});
    auto_declare<bool>("use_gravity", true);
    auto_declare<std::vector<double>>("gravity_xyz", {0.0, 0.0, -9.81});
    auto_declare<std::string>("base_link", "");
    auto_declare<std::string>("tip_link", "");
    auto_declare<std::string>("robot_description", "");
    auto_declare<double>("vel_lpf_alpha", 1.0);  // 1.0 = no filtering
    auto_declare<std::string>("upstream_controller", "resolved_rate_controller"); // set to actual instance name

    auto_declare<bool>("publish_ee_pose", true);              // enable/disable
    auto_declare<std::string>("ee_pose_topic", "/ee_pose");   // topic name

    auto_declare<double>("ref_step_limit", 0.01);
    auto_declare<double>("ref_qdot_limit", 2.0);
    auto_declare<double>("ref_vel_alpha",  0.6);


  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointImpedanceController::on_configure(const rclcpp_lifecycle::State &)
{
  // ---- read params ----
  joint_names_   = get_node()->get_parameter("joints").as_string_array();
  kp_            = get_node()->get_parameter("Kp").as_double_array();
  kd_            = get_node()->get_parameter("Kd").as_double_array();
  effort_limits_ = get_node()->get_parameter("effort_limits").as_double_array();
  use_gravity_   = get_node()->get_parameter("use_gravity").as_bool();
  vel_lpf_alpha_ = std::clamp(get_node()->get_parameter("vel_lpf_alpha").as_double(), 0.0, 1.0);

  bool publish_ee_pose_param = true;
  std::string ee_pose_topic_param = "/ee_pose";
  (void)get_node()->get_parameter("publish_ee_pose", publish_ee_pose_param);
  (void)get_node()->get_parameter("ee_pose_topic",   ee_pose_topic_param);
  publish_ee_pose_ = publish_ee_pose_param;
  ee_pose_topic_   = ee_pose_topic_param;

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Param 'joints' is required and must be non-empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t N = joint_names_.size();

  if (kp_.size() != N || kd_.size() != N) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Kp (%zu) and Kd (%zu) must match joints size (%zu).", kp_.size(), kd_.size(), N);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!effort_limits_.empty() && effort_limits_.size() != N) {
    RCLCPP_WARN(get_node()->get_logger(),
      "effort_limits size (%zu) != joints (%zu). Ignoring limits.", effort_limits_.size(), N);
    effort_limits_.clear();
  }

  // gravity vector
  {
    auto g = get_node()->get_parameter("gravity_xyz").as_double_array();
    if (g.size() != 3) g = {0.0, 0.0, -9.81};
    gravity_vec_ = KDL::Vector(g[0], g[1], g[2]);
  }

  // ---- Optional KDL for gravity feedforward ----

  const bool need_chain = use_gravity_ || publish_ee_pose_;                        // NEW
  if (need_chain) {
    base_link_ = get_node()->get_parameter("base_link").as_string();
    tip_link_  = get_node()->get_parameter("tip_link").as_string();

    if (base_link_.empty() || tip_link_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "With 'use_gravity'=true you must set 'base_link' and 'tip_link'.");
      return controller_interface::CallbackReturn::ERROR;
    }

    std::string urdf_xml = get_node()->get_parameter("robot_description").as_string();
    if (urdf_xml.empty()) {
      rclcpp::Parameter p;
      if (get_node()->get_parameter("robot_description", p) &&
          p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        urdf_xml = p.as_string();
      }
    }
    if (urdf_xml.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "URDF not provided. Set 'robot_description' on this node.");
      return controller_interface::CallbackReturn::ERROR;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF into KDL tree.");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!tree.getChain(base_link_, tip_link_, chain_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Failed to extract KDL chain from '%s' to '%s'.", base_link_.c_str(), tip_link_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (chain_.getNrOfJoints() != N) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "KDL chain DOF (%u) != joints param size (%zu).", chain_.getNrOfJoints(), N);
      return controller_interface::CallbackReturn::ERROR;
    }

    dyn_    = std::make_unique<KDL::ChainDynParam>(chain_, gravity_vec_);
    q_kdl_  = KDL::JntArray(N);
    g_kdl_  = KDL::JntArray(N);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    ee_pub_ = std::make_shared<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
          ee_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()));  

    
  }

  // ---- runtime buffers ----
  q_.assign(N, 0.0);
  dq_.assign(N, 0.0);
  dq_filt_.assign(N, 0.0);
  last_q_.assign(N, 0.0);
  tau_cmd_.assign(N, 0.0);

  // Desired RT buffer initial state: hold position at current q (will be set on activate)
  desired_shadow_ = Desired{};
  desired_shadow_.qd.assign(N, 0.0);
  desired_shadow_.dqd.assign(N, 0.0);
  desired_shadow_.tau_ff.assign(N, 0.0);
  desired_shadow_.has_qd = false;     // becomes true after first activate/update
  desired_shadow_.has_dqd = false;
  desired_shadow_.has_tau_ff = false;
  desired_rt_.writeFromNonRT(desired_shadow_);

  // ---- subscriber ----
  sub_cmd_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    std::bind(&JointImpedanceController::command_cb, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(),
    "JointImpedanceController configured: N=%zu, use_gravity=%s, vel_lpf_alpha=%.2f",
    N, use_gravity_ ? "true" : "false", vel_lpf_alpha_);

    // on_configure(...)
  sub_traj_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/trajectory", rclcpp::SystemDefaultsQoS(),
    std::bind(&JointImpedanceController::traj_cb, this, std::placeholders::_1));


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointImpedanceController::on_activate(const rclcpp_lifecycle::State &)
{
  // Initialize command to zero
  for (auto & ci : command_interfaces_) ci.set_value(0.0);

  const size_t N = joint_names_.size();
  
  bool have_refs = (reference_interfaces_.size() == 2*N);
  if (have_refs) {
    RCLCPP_INFO(get_node()->get_logger(),
      "Impedance: found %zu reference interfaces (position_ref + velocity_ref).", 2*N);
  } else {
    RCLCPP_INFO(get_node()->get_logger(),
      "Impedance: no reference interfaces found, will use ~/command or ~/trajectory.");
  }

  // Read initial q & dq (if velocity interfaces provided they come after positions)
  if (state_interfaces_.size() == N * 2) {
    have_velocity_state_ = true;
    for (size_t i = 0; i < N; ++i) {
      q_[i]  = state_interfaces_[i].get_value();
      dq_[i] = state_interfaces_[N + i].get_value();
    }
  } else {
    have_velocity_state_ = false;
    for (size_t i = 0; i < N; ++i) {
      q_[i]  = state_interfaces_[i].get_value();
      dq_[i] = 0.0;
    }
  }

  qd_int_.resize(N);
  dqd_cmd_.assign(N, 0.0);

  // Start the integrator at the measured joint angles
  for (size_t i = 0; i < N; ++i) {
    qd_int_[i] = q_[i];  // your measured joint positions
  }
  last_chained_ = this->chained_mode_; // or your flag

  dq_filt_ = dq_;
  last_q_  = q_;

  // Set hold-position desired to current state
  desired_shadow_ = Desired{};
  desired_shadow_.qd     = q_;
  desired_shadow_.dqd.assign(N, 0.0);
  desired_shadow_.tau_ff.assign(N, 0.0);
  desired_shadow_.has_qd = true;
  desired_shadow_.has_dqd = true;
  desired_shadow_.has_tau_ff = false;
  desired_rt_.writeFromNonRT(desired_shadow_);

  // NEW: Pre-fill chained references with current state so we don't jump to zeros
  if (reference_interfaces_.size() == 2 * N) {
    for (size_t i = 0; i < N; ++i) {
      reference_interfaces_[i]      = q_[i];   // desired position = current
      reference_interfaces_[N + i]  = 0.0;     // desired velocity = 0
    }
  }

  if (publish_ee_pose_ && fk_solver_) {
    // 1) set frame_id once
    if (ee_pub_ && ee_pub_->trylock()) {
      ee_pub_->msg_.header.frame_id = base_link_;
      ee_pub_->unlock();
    }

    // 2) compute FK once from current q_ and publish an initial sample
    for (size_t i = 0; i < N; ++i) q_kdl_(i) = q_[i];
    KDL::Frame T;
    if (fk_solver_->JntToCart(q_kdl_, T) >= 0) {
      if (ee_pub_ && ee_pub_->trylock()) {
        auto & msg = ee_pub_->msg_;
        // on_activate is non-RT; using node clock is fine here
        msg.header.stamp = get_node()->get_clock()->now();
        msg.header.frame_id = base_link_;
        msg.pose.position.x = T.p.x();
        msg.pose.position.y = T.p.y();
        msg.pose.position.z = T.p.z();
        double x,y,z,w; T.M.GetQuaternion(x,y,z,w);
        msg.pose.orientation.x = x;
        msg.pose.orientation.y = y;
        msg.pose.orientation.z = z;
        msg.pose.orientation.w = w;
        ee_pub_->unlockAndPublish();
      }
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointImpedanceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) ci.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

// --- helper: command callback ---
void JointImpedanceController::command_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const size_t N = joint_names_.size();
  Desired d;

  // Map message into desired vectors. If names[] is empty, use configured joint order.
  if (!msg->name.empty()) {
    if (msg->name.size() != N) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Received command name size (%zu) != joints (%zu). Ignoring.", msg->name.size(), N);
      return;
    }
    // Build a name->index for configured order
    std::unordered_map<std::string, size_t> index;
    index.reserve(N);
    for (size_t i = 0; i < N; ++i) index[joint_names_[i]] = i;

    auto copy_field = [&](const std::vector<double>& src, std::vector<double>& dst, bool &flag){
      if (src.empty()) { flag = false; return; }
      dst.assign(N, 0.0);
      for (size_t k = 0; k < N; ++k) {
        auto it = index.find(msg->name[k]);
        if (it == index.end()) {
          RCLCPP_WARN(get_node()->get_logger(), "Unknown joint in command: %s", msg->name[k].c_str());
          return;
        }
        if (k < src.size()) dst[it->second] = src[k];
      }
      flag = true;
    };

    copy_field(msg->position, d.qd, d.has_qd);
    copy_field(msg->velocity, d.dqd, d.has_dqd);
    copy_field(msg->effort,   d.tau_ff, d.has_tau_ff);
  } else {
    // Assume exact joint order
    auto copy_field = [&](const std::vector<double>& src, std::vector<double>& dst, bool &flag){
      if (src.empty()) { flag = false; return; }
      if (src.size() != N) {
        RCLCPP_WARN(get_node()->get_logger(),
          "Command field size (%zu) != joints (%zu). Ignoring field.", src.size(), N);
        flag = false; return;
      }
      dst = src; flag = true;
    };
    copy_field(msg->position, d.qd, d.has_qd);
    copy_field(msg->velocity, d.dqd, d.has_dqd);
    copy_field(msg->effort,   d.tau_ff, d.has_tau_ff);
  }

  // Ensure vectors are allocated even if flags are false (keeps update code branchless)
  if (!d.has_qd)   d.qd.assign(N, 0.0);
  if (!d.has_dqd)  d.dqd.assign(N, 0.0);
  if (!d.has_tau_ff) d.tau_ff.assign(N, 0.0);

  desired_rt_.writeFromNonRT(d);
}

// non-RT callback: convert JointTrajectory -> Poly5 segments, cache coeffs, publish to RT
void JointImpedanceController::traj_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  const size_t N = joint_names_.size();
  if (msg->points.empty()) return;

  // Build remap index from msg->joint_names to controller joint order
  std::vector<int> idx(N,-1);
  if (!msg->joint_names.empty()) {
    for (size_t i=0;i<N;i++){
      auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_names_[i]);
      if (it==msg->joint_names.end()){
        RCLCPP_WARN(get_node()->get_logger(),"Trajectory missing joint %s", joint_names_[i].c_str());
        return;
      }
      idx[i] = int(std::distance(msg->joint_names.begin(), it));
    }
  } else {
    // assume same order/size
    if (msg->points.front().positions.size()!=N){
      RCLCPP_WARN(get_node()->get_logger(),"Trajectory has no names but size mismatch");
      return;
    }
    for (size_t i=0;i<N;i++) idx[i]=int(i);
  }

  // Build segments with boundary conditions (use zeros if not provided)
  TrajRT plan;
  plan.segs.clear();
  auto get = [&](auto &vec, size_t /*j*/, size_t k, double def)->double{
    const auto &v = vec; if (v.empty()) return def;
    if (k>=v.size()) return def; // ROS <humble> sanity
    return v[k];
  };

  // Use first point as start; prepend current state if needed
  const auto &P0 = msg->points.front();
  Poly5Seg seg0;
  seg0.t0 = 0.0; seg0.T = std::max(1e-3, P0.time_from_start.sec + P0.time_from_start.nanosec*1e-9);
  seg0.q0.resize(N); seg0.v0.assign(N,0.0); seg0.a0.assign(N,0.0);
  seg0.q1.resize(N); seg0.v1.resize(N);     seg0.a1.resize(N);
  for (size_t i=0;i<N;i++){
    const size_t k = size_t(idx[i]);
    seg0.q0[i] = q_[i];               // start at current q
    seg0.q1[i] = get(P0.positions,  k, 0, q_[i]);
    seg0.v1[i] = get(P0.velocities, k, 0, 0.0);
    seg0.a1[i] = get(P0.accelerations,k,0,0.0);
  }
  plan.segs.push_back(seg0);

  // Remaining segments between successive points
  for (size_t s=1; s<msg->points.size(); ++s){
    const auto &Pa = msg->points[s-1];
    const auto &Pb = msg->points[s];
    Poly5Seg seg;
    const double ta = Pa.time_from_start.sec + Pa.time_from_start.nanosec*1e-9;
    const double tb = Pb.time_from_start.sec + Pb.time_from_start.nanosec*1e-9;
    seg.t0 = ta; seg.T = std::max(1e-3, tb - ta);
    seg.q0.resize(N); seg.v0.resize(N); seg.a0.resize(N);
    seg.q1.resize(N); seg.v1.resize(N); seg.a1.resize(N);
    for (size_t i=0;i<N;i++){
      const size_t k = size_t(idx[i]);
      seg.q0[i] = get(Pa.positions,     k, 0, seg0.q1[i]);
      seg.v0[i] = get(Pa.velocities,    k, 0, 0.0);
      seg.a0[i] = get(Pa.accelerations, k, 0, 0.0);
      seg.q1[i] = get(Pb.positions,     k, 0, seg.q0[i]);
      seg.v1[i] = get(Pb.velocities,    k, 0, 0.0);
      seg.a1[i] = get(Pb.accelerations, k, 0, 0.0);
    }
    plan.segs.push_back(seg);
  }

  // Precompute coefficients
  for (auto &s : plan.segs){
    s.coeffs.resize(N);
    for (size_t i=0;i<N;i++){
      s.coeffs[i] = quintic_coeff(s.q0[i], s.v0[i], s.a0[i],
                                  s.q1[i], s.v1[i], s.a1[i], s.T);
    }
  }
  plan.t_plan0 = get_node()->get_clock()->now().seconds(); // non-RT time base
  plan.valid = true;
  traj_rt_.writeFromNonRT(plan);
  RCLCPP_INFO(get_node()->get_logger(),"Accepted trajectory: %zu segments", plan.segs.size());
}

// helper (inside class)
void JointImpedanceController::eval_traj(double t_ctrl,
               std::vector<double>& qd,
               std::vector<double>& dqd,
               std::vector<double>& dqdd)
{
  // Check trajectory validity FIRST (before initializing vectors)
  auto plan = traj_rt_.readFromRT();
  if (!plan || !plan->valid) {
    // No valid trajectory → return EMPTY vectors as signal
    qd.clear();
    dqd.clear();
    dqdd.clear();
    return;
  }

  const size_t N = q_.size();
  qd.resize(N);
  dqd.resize(N);
  dqdd.resize(N);

  const double t = t_ctrl - plan->t_plan0; // seconds since plan start
  if (t <= 0.0) {
    const auto &s = plan->segs.front();
    for (size_t i=0;i<q_.size();++i){ qd[i]=s.q0[i]; }
    return;
  }

  const Poly5Seg *seg_ptr = nullptr;
  double tau = 0.0; // local time in segment
  for (const auto &s : plan->segs){
    if (t < s.t0 + s.T){ seg_ptr=&s; tau = t - s.t0; break; }
  }
  if (!seg_ptr){ // past the end -> hold final
    const auto &s = plan->segs.back();
    for (size_t i=0;i<q_.size();++i){ qd[i]=s.q1[i]; }
    return;
  }

  const auto &s = *seg_ptr;
  const double tt=tau, tt2=tt*tt, tt3=tt2*tt, tt4=tt3*tt, tt5=tt4*tt;
  for (size_t i=0;i<q_.size();++i){
    const auto &c = s.coeffs[i]; // c0..c5
    qd[i]   = c[0] + c[1]*tt + c[2]*tt2 + c[3]*tt3 + c[4]*tt4 + c[5]*tt5;
    dqd[i]  =      c[1]     + 2*c[2]*tt + 3*c[3]*tt2 + 4*c[4]*tt3 + 5*c[5]*tt4;
    dqdd[i] =                2*c[2]     + 6*c[3]*tt  +12*c[4]*tt2 +20*c[5]*tt3;
  }
}

// --- helper: gravity computation ---
void JointImpedanceController::compute_gravity(const std::vector<double>& q, std::vector<double>& tau_g)
{
  if (!use_gravity_ || !dyn_) { tau_g.assign(q.size(), 0.0); return; }

  for (size_t i = 0; i < q.size(); ++i) q_kdl_(i) = q[i];
  const int rc = dyn_->JntToGravity(q_kdl_, g_kdl_);
  if (rc != 0) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "KDL::JntToGravity rc=%d", rc);
    tau_g.assign(q.size(), 0.0);
    return;
  }
  tau_g.resize(q.size());
  for (size_t i = 0; i < q.size(); ++i) tau_g[i] = g_kdl_(i);
}

// --- helper: clamp ---
void JointImpedanceController::clamp_effort(std::vector<double>& tau) const
{
  if (effort_limits_.empty()) return;
  for (size_t i = 0; i < tau.size(); ++i) {
    const double lim = std::abs(effort_limits_[i]);
    tau[i] = std::clamp(tau[i], -lim, lim);
  }
}

// --- helper: optional velocity filtering ---
void JointImpedanceController::maybe_filter_velocity(double /*dt*/)
{
  if (vel_lpf_alpha_ >= 0.9999) {
    dq_filt_ = dq_;
    return;
  }
  for (size_t i = 0; i < dq_.size(); ++i) {
    dq_filt_[i] = vel_lpf_alpha_ * dq_[i] + (1.0 - vel_lpf_alpha_) * dq_filt_[i];
  }
}

controller_interface::return_type
JointImpedanceController::update_reference_from_subscribers()
{
  const size_t N = joint_names_.size();

  // 1) If you have a trajectory generator, try it first
  std::vector<double> qd_ref, dqd_ref, qdd_ref;
  const double t_ctrl = this->get_node()->get_clock()->now().seconds();
  eval_traj(t_ctrl, qd_ref, dqd_ref, qdd_ref);

  // 2) Fallback: your existing Desired RT buffer (~/command)
  std::vector<double> tau_ff(N, 0.0);
  if (qd_ref.empty()) {
    Desired d = *(desired_rt_.readFromRT());
    if (!d.has_qd)    { d.qd   = q_;              d.has_qd   = true; }
    if (!d.has_dqd)   { d.dqd.assign(N, 0.0);     d.has_dqd  = true; }
    if (!d.has_tau_ff){ d.tau_ff.assign(N, 0.0);  d.has_tau_ff = true; }
    qd_ref  = d.qd;
    dqd_ref = d.dqd;
    tau_ff  = d.tau_ff;  // keep if you use feedforward in stage B (store in a member if needed)
  }
  // 3) If you have external reference interfaces, override the references
  if (reference_interfaces_.size() == 2 * N) {
    for (size_t i = 0; i < N; ++i) {
      qd_ref[i]  = reference_interfaces_[i];
      dqd_ref[i] = reference_interfaces_[N + i];
    }
  }

  // If you want tau_ff in stage B, cache it in a member (e.g., tau_ff_cache_)
  tau_ff_cache_ = std::move(tau_ff);

  return controller_interface::return_type::OK;
}



controller_interface::return_type
JointImpedanceController::update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const size_t N = joint_names_.size();
  const double dt = period.seconds();
  // dt_ = period.seconds();   // store for integration

  // Validate interfaces
  if (state_interfaces_.size() != N * 2 || command_interfaces_.size() != N) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Interface size mismatch: state=%zu (expected %zu), cmd=%zu (expected %zu)",
      state_interfaces_.size(), N * 2, command_interfaces_.size(), N);
    return controller_interface::return_type::ERROR;
  }

  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //   "JIC dt=%.6f s (%.1f Hz)", dt, dt > 1e-6 ? 1.0/dt : 0.0);


  // Read state
  for (size_t i = 0; i < N; ++i) {
    q_[i]  = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[N + i].get_value();
  }
  maybe_filter_velocity(dt);  // fills dq_filt_

  // --- references: chain refs > trajectory > ~/command > hold ---
  std::vector<double> qd_ref, dqd_ref, qdd_ref;
  std::vector<double> tau_ff(N, 0.0);

  if (chained_mode_ && reference_interfaces_.size() == 2 * N) {
    last_chained_ = false;  // so we rebase next time we re-enter chained mode
    const size_t N = joint_names_.size();
    qd_ref.resize(N);
    dqd_ref.resize(N);

    // Detect chained-mode edge and rebase integrator once
    if (!last_chained_) {
      for (size_t i = 0; i < N; ++i) qd_int_[i] = q_[i];
    }
    last_chained_ = true;

    // 1) Read upstream refs: prefer velocity; position is optional
    for (size_t i = 0; i < N; ++i) {
      // const double qdot_in  = reference_interfaces_[N + i];   // <upstream>/<joint>/velocity
      // Optional: if you also want to honor upstream position when present:
      const double qpos_in = reference_interfaces_[i];
      const double qdot_in  = reference_interfaces_[N + i];
      const double qdot_c  = std::clamp(qdot_in, -ref_qdot_limit_, ref_qdot_limit_);
      qd_ref[i]  = qpos_in;         // track upstream position directly
      dqd_ref[i] = qdot_c;          // use upstream velocity for damping
    }

    last_chained_ = true;           // (no internal integrator state to manage)

  } else {
    last_chained_ = false;  // so we rebase next time we re-enter chained mode
    // Try trajectory
    eval_traj(time.seconds(), qd_ref, dqd_ref, qdd_ref);

    // Fallback to ~/command (Desired RT buffer)
    if (qd_ref.empty()) {
      Desired d = *(desired_rt_.readFromRT());
      if (!d.has_qd)     { d.qd   = q_; }
      if (!d.has_dqd)    { d.dqd.assign(N, 0.0); }
      if (!d.has_tau_ff) { d.tau_ff.assign(N, 0.0); }
      qd_ref  = std::move(d.qd);
      dqd_ref = std::move(d.dqd);
      tau_ff  = std::move(d.tau_ff);
    }

    // Final safe fallback: hold
    if (qd_ref.empty()) {
      qd_ref = q_;
      dqd_ref.assign(N, 0.0);
    }
  }

  // Gravity feedforward
  std::vector<double> tau_g;
  if (use_gravity_) compute_gravity(q_, tau_g);

  // PD impedance + ff (+ gravity)
  for (size_t i = 0; i < N; ++i) {
    const double e_pos = qd_ref[i]  - q_[i];
    const double e_vel = dqd_ref[i] - dq_filt_[i];
    double tau = kp_[i] * e_pos + kd_[i] * e_vel + tau_ff[i];
    if (use_gravity_ && tau_g.size() == N) tau += tau_g[i];
    tau_cmd_[i] = tau;
  }

  // Clamp & write
  clamp_effort(tau_cmd_);
  for (size_t i = 0; i < N; ++i) {
    command_interfaces_[i].set_value(tau_cmd_[i]);
  }
  // Debug: log torques every ~1s
  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //   "Cmd torques: [%.3f %.3f %.3f %.3f %.3f %.3f]",
  //   tau_cmd_[0], tau_cmd_[1], tau_cmd_[2],
  //   tau_cmd_[3], tau_cmd_[4], tau_cmd_[5]);

  // (Optional) EE pose publisher
  if (publish_ee_pose_ && fk_solver_) {
    for (size_t i = 0; i < N; ++i) q_kdl_(i) = q_[i];
    KDL::Frame T;
    if (fk_solver_->JntToCart(q_kdl_, T) >= 0) {
      if (ee_pub_ && ee_pub_->trylock()) {
        auto & msg = ee_pub_->msg_;
        msg.header.stamp = time;
        msg.header.frame_id = base_link_;
        msg.pose.position.x = T.p.x();
        msg.pose.position.y = T.p.y();
        msg.pose.position.z = T.p.z();
        double x, y, z, w; T.M.GetQuaternion(x, y, z, w);
        msg.pose.orientation.x = x; msg.pose.orientation.y = y;
        msg.pose.orientation.z = z; msg.pose.orientation.w = w;
        ee_pub_->unlockAndPublish();
      }
    }
  }

  return controller_interface::return_type::OK;
}
// ...existing code...

// helper: build one quintic between (q0,v0,a0) and (q1,v1,a1) over T
std::array<double,6> JointImpedanceController::quintic_coeff(
    double q0, double v0, double a0,
    double q1, double v1, double a1,
    double T)
{
  const double T2=T*T, T3=T2*T, T4=T3*T, T5=T4*T;
  const double c0=q0, c1=v0, c2=0.5*a0;
  const double A = q1 - (c0 + c1*T + c2*T2);
  const double B = v1 - (c1 + 2*c2*T);
  const double C = a1 - (2*c2);
  const double c3 = (10*A - 4*B*T - 0.5*C*T2) / T3;
  const double c4 = (-15*A + 7*B*T +     C*T2) / T4;
  const double c5 = ( 6*A - 3*B*T - 0.5*C*T2) / T5;
  return {c0,c1,c2,c3,c4,c5};
}



} // namespace ombot_controller

PLUGINLIB_EXPORT_CLASS(
  ombot_controller::JointImpedanceController,
  controller_interface::ChainableControllerInterface)
