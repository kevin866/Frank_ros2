#include "joint_friction_calib_controller/joint_friction_calib_controller.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

#include "pluginlib/class_list_macros.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace ombot_controller
{

// ----- small 3x3 solver (Gauss-Jordan) -----
bool JointFrictionCalibController::LS3::solve(double x_out[3]) const
{
  // Copy into augmented matrix [A|b]
  double M[3][4];
  for (int i=0;i<3;i++){
    for (int j=0;j<3;j++) M[i][j]=A[i][j];
    M[i][3]=b[i];
  }

  // Add tiny regularization to diagonal to avoid singularity in low-data cases
  const double lam = 1e-9;
  M[0][0] += lam; M[1][1] += lam; M[2][2] += lam;

  // Gauss-Jordan
  for (int col=0; col<3; ++col) {
    // pivot
    int piv = col;
    double best = std::abs(M[col][col]);
    for (int r=col+1; r<3; ++r) {
      double v = std::abs(M[r][col]);
      if (v > best) { best = v; piv = r; }
    }
    if (best < 1e-12) return false;

    if (piv != col) {
      for (int c=col; c<4; ++c) std::swap(M[col][c], M[piv][c]);
    }

    // normalize
    const double div = M[col][col];
    for (int c=col; c<4; ++c) M[col][c] /= div;

    // eliminate
    for (int r=0; r<3; ++r) {
      if (r == col) continue;
      const double f = M[r][col];
      for (int c=col; c<4; ++c) M[r][c] -= f * M[col][c];
    }
  }

  x_out[0] = M[0][3];
  x_out[1] = M[1][3];
  x_out[2] = M[2][3];
  return true;
}

// ---------------- controller interface ----------------

controller_interface::InterfaceConfiguration
JointFrictionCalibController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) conf.names.push_back(j + "/velocity");
  return conf;
}

controller_interface::InterfaceConfiguration
JointFrictionCalibController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // IMPORTANT: ensure your hardware exports measured effort to HW_IF_EFFORT state
  for (const auto & j : joint_names_) {
    conf.names.push_back(j + "/position");
    conf.names.push_back(j + "/velocity");
    conf.names.push_back(j + "/effort");
  }
  return conf;
}

controller_interface::CallbackReturn JointFrictionCalibController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_link", "");
    auto_declare<std::string>("tip_link", "");
    auto_declare<std::string>("robot_description", "");

    auto_declare<std::vector<double>>("gravity_xyz", {0.0, 0.0, -9.81});
    auto_declare<double>("g_scale", 1.0);
    auto_declare<double>("alpha", 1.0);
    auto_declare<std::vector<double>>("effort_limits", {});

    auto_declare<std::string>("active_joint", "");  // name
    auto_declare<int>("active_joint_index", -1);

    auto_declare<double>("v_amp", 0.2);
    auto_declare<double>("v_freq", 0.2);
    auto_declare<double>("settle_time", 2.0);
    auto_declare<double>("run_time", 20.0);

    auto_declare<double>("Kv", 2.0);
    auto_declare<double>("Kd", 0.0);
    auto_declare<double>("max_servo_tau", 3.0);
    auto_declare<std::vector<double>>("q_min", {});
    auto_declare<std::vector<double>>("q_max", {});
    auto_declare<double>("q_soft_margin", 0.2);   // rad
    auto_declare<double>("k_limit", 15.0);        // Nm/rad
    auto_declare<double>("d_limit", 1.0);         // Nm/(rad/s)


    auto_declare<double>("fit_min_speed", 0.03);
    auto_declare<double>("v_sgn_eps", 0.01);

    auto_declare<bool>("auto_advance", true);
    auto_declare<bool>("print_each_joint", true);
    auto_declare<bool>("apply_friction_ff_after_fit", false);
    auto_declare<std::vector<double>>("Kv_joints", {});
    auto_declare<std::vector<double>>("Kd_joints", {});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointFrictionCalibController::on_configure(const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  base_link_   = get_node()->get_parameter("base_link").as_string();
  tip_link_    = get_node()->get_parameter("tip_link").as_string();
  Kv_joints_ = get_node()->get_parameter("Kv_joints").as_double_array();
  Kd_joints_ = get_node()->get_parameter("Kd_joints").as_double_array();
    
  const size_t N = joint_names_.size();


  if (!Kv_joints_.empty() && Kv_joints_.size() != N) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Kv_joints must have size %zu (got %zu)", N, Kv_joints_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!Kd_joints_.empty() && Kd_joints_.size() != N) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Kd_joints must have size %zu (got %zu)", N, Kd_joints_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
    


  q_min_ = get_node()->get_parameter("q_min").as_double_array();
  q_max_ = get_node()->get_parameter("q_max").as_double_array();

  if (q_min_.size() != N || q_max_.size() != N) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "q_min/q_max must have size %zu (got %zu / %zu)",
      N, q_min_.size(), q_max_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  q_soft_margin_ = get_node()->get_parameter("q_soft_margin").as_double();
  k_limit_ = get_node()->get_parameter("k_limit").as_double();
  d_limit_ = get_node()->get_parameter("d_limit").as_double();


  g_scale_ = get_node()->get_parameter("g_scale").as_double();
  alpha_   = std::clamp(get_node()->get_parameter("alpha").as_double(), 0.0, 1.0);

  active_joint_name_  = get_node()->get_parameter("active_joint").as_string();
  active_joint_index_ = get_node()->get_parameter("active_joint_index").as_int();

  v_amp_       = get_node()->get_parameter("v_amp").as_double();
  v_freq_      = get_node()->get_parameter("v_freq").as_double();
  settle_time_ = get_node()->get_parameter("settle_time").as_double();
  run_time_    = get_node()->get_parameter("run_time").as_double();

  Kv_ = get_node()->get_parameter("Kv").as_double();
  Kd_ = get_node()->get_parameter("Kd").as_double();
  max_servo_tau_ = std::abs(get_node()->get_parameter("max_servo_tau").as_double());

  fit_min_speed_ = get_node()->get_parameter("fit_min_speed").as_double();
  v_sgn_eps_     = get_node()->get_parameter("v_sgn_eps").as_double();

  auto_advance_ = get_node()->get_parameter("auto_advance").as_bool();
  print_each_joint_ = get_node()->get_parameter("print_each_joint").as_bool();
  apply_friction_ff_after_fit_ = get_node()->get_parameter("apply_friction_ff_after_fit").as_bool();

  if (joint_names_.empty() || base_link_.empty() || tip_link_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Params 'joints', 'base_link', and 'tip_link' are required.");
    return controller_interface::CallbackReturn::ERROR;
  }
  // const size_t N = joint_names_.size();

  // gravity vec
  {
    auto g = get_node()->get_parameter("gravity_xyz").as_double_array();
    if (g.size() != 3) g = {0.0, 0.0, -9.81};
    gravity_vec_ = KDL::Vector(g[0], g[1], g[2]);
  }

  // limits (optional)
  effort_limits_ = get_node()->get_parameter("effort_limits").as_double_array();
  if (!effort_limits_.empty() && effort_limits_.size() != N) {
    RCLCPP_WARN(get_node()->get_logger(),
                "effort_limits size (%zu) != joints (%zu). Ignoring limits.",
                effort_limits_.size(), N);
    effort_limits_.clear();
  }

  // ---- Build KDL chain from URDF ----




  
  std::string urdf_xml = get_node()->get_parameter("robot_description").as_string();
  if (urdf_xml.empty()) {
    rclcpp::Parameter p;
    if (get_node()->get_parameter("robot_description", p) &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
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
                 "Failed to extract KDL chain from '%s' to '%s'.",
                 base_link_.c_str(), tip_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (chain_.getNrOfJoints() != N) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "KDL chain DOF (%u) != joints param size (%zu).",
                 chain_.getNrOfJoints(), N);
    return controller_interface::CallbackReturn::ERROR;
  }

  dyn_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity_vec_);
  q_ = KDL::JntArray(N);
  g_ = KDL::JntArray(N);
  last_g_cmd_.assign(N, 0.0);

  dq_prev_.assign(N, 0.0);


  // Fit buffers
  fit_.assign(N, LS3{});
  Fc_.assign(N, 0.0);
  B_.assign(N, 0.0);
  tau_bias_.assign(N, 0.0);
  joint_done_.assign(N, false);

  recompute_active_joint_();

  // q_ = KDL::JntArray(N);
  // g_ = KDL::JntArray(N);

  dq_kdl_ = KDL::JntArray(N);
  c_kdl_  = KDL::JntArray(N);

  RCLCPP_INFO(get_node()->get_logger(),
              "JointFrictionCalibController configured: N=%zu active_joint_index=%d auto_advance=%d "
              "v_amp=%.3f v_freq=%.3f Kv=%.3f Kd=%.3f",
              N, active_joint_index_, (int)auto_advance_, v_amp_, v_freq_, Kv_, Kd_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointFrictionCalibController::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) ci.set_value(0.0);
  active_ = true;
  phase_ = Phase::SETTLE;
  t0_ = get_node()->now();

  // reset fits
  for (auto & f : fit_) f.reset();
  std::fill(joint_done_.begin(), joint_done_.end(), false);

  if (active_joint_index_ < 0 && auto_advance_) {
    active_joint_index_ = 0;
  }
  if (active_joint_index_ >= 0) {
    start_joint_(active_joint_index_, t0_);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointFrictionCalibController::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  for (auto & ci : command_interfaces_) ci.set_value(0.0);

  // final print
  print_yaml_results_();
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------- helpers ----------------

int JointFrictionCalibController::find_joint_index_(const std::string & name) const
{
  for (size_t i=0;i<joint_names_.size();++i) if (joint_names_[i]==name) return (int)i;
  return -1;
}

void JointFrictionCalibController::recompute_active_joint_()
{
  if (!active_joint_name_.empty()) {
    const int idx = find_joint_index_(active_joint_name_);
    if (idx >= 0) active_joint_index_ = idx;
  }
}

double JointFrictionCalibController::smooth_sign_(double v) const
{
  const double eps = std::max(1e-6, v_sgn_eps_);
  return std::tanh(v / eps);
}

void JointFrictionCalibController::compute_gravity_(const std::vector<double> & q,
                                                    std::vector<double> & tau_g)
{
  const size_t N = joint_names_.size();
  for (size_t i=0;i<N;++i) q_(i) = q[i];

  const int rc = dyn_->JntToGravity(q_, g_);
  if (rc != 0) {
    // if gravity fails, just output zeros to be safe
    std::fill(tau_g.begin(), tau_g.end(), 0.0);
    return;
  }

  for (size_t i=0;i<N;++i) {
    const double raw = g_scale_ * g_(i);
    const double filt = alpha_ * raw + (1.0 - alpha_) * last_g_cmd_[i];
    last_g_cmd_[i] = filt;
    tau_g[i] = filt;
  }
}

void JointFrictionCalibController::clamp_effort_(std::vector<double> & tau) const
{
  if (effort_limits_.empty()) return;
  for (size_t i=0;i<tau.size();++i) {
    const double lim = std::abs(effort_limits_[i]);
    tau[i] = std::clamp(tau[i], -lim, lim);
  }
}

void JointFrictionCalibController::write_effort_cmd_(const std::vector<double> & tau_cmd)
{
  const size_t N = joint_names_.size();
  for (size_t i=0;i<N;++i) command_interfaces_[i].set_value(tau_cmd[i]);
}

void JointFrictionCalibController::write_velocity_cmd_(const std::vector<double> & dq_cmd)
{
  const size_t N = joint_names_.size();
  for (size_t i=0;i<N;++i) command_interfaces_[i].set_value(dq_cmd[i]);
}

void JointFrictionCalibController::start_joint_(int j, const rclcpp::Time & now)
{
  phase_ = Phase::SETTLE;
  t0_ = now;

  if (j >= 0 && j < (int)fit_.size()) {
    fit_[j].reset();
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Starting friction calibration for joint[%d]=%s (settle=%.2fs run=%.2fs)",
              j, joint_names_[j].c_str(), settle_time_, run_time_);
}

void JointFrictionCalibController::finish_joint_(int j)
{
  if (j < 0 || j >= (int)fit_.size()) return;

  double x[3] = {0,0,0};
  const bool ok = fit_[j].solve(x);
  if (ok) {
    Fc_[j] = x[0];
    B_[j]  = x[1];
    tau_bias_[j] = x[2];
  }

  joint_done_[j] = true;

  // if (print_each_joint_) {
  //   RCLCPP_INFO(get_node()->get_logger(),
  //     "Fit joint[%d]=%s: n=%d  Fc=%.5f  B=%.5f  bias=%.5f  (ok=%d)",
  //     j, joint_names_[j].c_str(), fit_[j].n, Fc_[j], B_[j], tau_bias_[j], (int)ok);
  // }
}

void JointFrictionCalibController::maybe_advance_joint_(const rclcpp::Time & now)
{
  if (!auto_advance_) return;

  const size_t N = joint_names_.size();
  // find next not-done joint
  int next = -1;
  for (size_t i=0;i<N;++i) {
    if (!joint_done_[i]) { next = (int)i; break; }
  }

  if (next < 0) {
    phase_ = Phase::DONE;
    print_yaml_results_();
    RCLCPP_INFO(get_node()->get_logger(), "All joints done. Calibration complete.");
    return;
  }

  if (next != active_joint_index_) {
    active_joint_index_ = next;
    start_joint_(active_joint_index_, now);
  }
}

void JointFrictionCalibController::print_yaml_results_() const
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss << std::setprecision(5);

  auto print_vec = [&](const std::string & name, const std::vector<double> & v) {
    oss << "  " << name << ": [";
    for (size_t i=0;i<v.size();++i) {
      oss << v[i];
      if (i + 1 < v.size()) oss << ", ";
    }
    oss << "]\n";
  };

  oss << "friction:\n";
  print_vec("Fc", Fc_);
  print_vec("B", B_);
  print_vec("tau_bias", tau_bias_);

  // Keep your existing defaults (you can also fit these later)
  oss << "  v_deadband: [";
  for (size_t i=0;i<Fc_.size();++i) { oss << 0.02 << (i+1<Fc_.size()? ", ":""); }
  oss << "]\n";

  oss << "  v_sgn_eps: [";
  for (size_t i=0;i<Fc_.size();++i) { oss << v_sgn_eps_ << (i+1<Fc_.size()? ", ":""); }
  oss << "]\n";

  RCLCPP_INFO(get_node()->get_logger(), "\n%s", oss.str().c_str());
}

// ---------------- update loop ----------------

controller_interface::return_type
JointFrictionCalibController::update(const rclcpp::Time & time,
                                     const rclcpp::Duration & period)
{
  if (!active_) return controller_interface::return_type::OK;

  const size_t N = joint_names_.size();
  const size_t expected_cmd = N;
  const size_t expected_state = 3 * N;
  const double dt = std::max(1e-4, period.seconds());


  if (command_interfaces_.size() != expected_cmd || state_interfaces_.size() != expected_state) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Interface size mismatch: cmd=%zu (exp %zu), state=%zu (exp %zu)",
      command_interfaces_.size(), expected_cmd, state_interfaces_.size(), expected_state);
    return controller_interface::return_type::ERROR;
  }

  // Read state arrays
  std::vector<double> q(N), dq(N), tau_meas(N);
  for (size_t i=0;i<N;++i) {
    q[i] = state_interfaces_[3*i + 0].get_value();
    dq[i]= state_interfaces_[3*i + 1].get_value();
    tau_meas[i] = state_interfaces_[3*i + 2].get_value();
  }

  // Gravity torques
  std::vector<double> tau_g(N, 0.0);
  compute_gravity_(q, tau_g);

  std::ostringstream oss;
  oss << "tau_g = [";
  for (size_t i = 0; i < tau_g.size(); ++i) {
    oss << std::fixed << std::setprecision(4) << tau_g[i];
    if (i < tau_g.size() - 1)
      oss << ", ";
  }
  oss << "]";

  RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "%s",
      oss.str().c_str());

  // Coriolis torques
  for (size_t i = 0; i < N; ++i) dq_kdl_(i) = dq[i];

  int rcC = dyn_->JntToCoriolis(q_, dq_kdl_, c_kdl_);
  if (rcC != 0) {
    // be safe: if Coriolis fails, treat as zero
    for (size_t i = 0; i < N; ++i) c_kdl_(i) = 0.0;
  }

  // Default command = gravity only (holds posture)
  // std::vector<double> tau_cmd = tau_g;

  // if (phase_ == Phase::DONE) {
  //   clamp_effort_(tau_cmd);
  //   write_effort_cmd_(tau_cmd);
  //   return controller_interface::return_type::OK;
  // }

  // if (active_joint_index_ < 0 || active_joint_index_ >= (int)N) {
  //   // no active joint selected; just gravity
  //   clamp_effort_(tau_cmd);
  //   write_effort_cmd_(tau_cmd);
  //   return controller_interface::return_type::OK;
  // }

  // Default command = zero velocity (hold)
  std::vector<double> dq_cmd(N, 0.0);

  if (phase_ == Phase::DONE) {
    write_velocity_cmd_(dq_cmd);
    return controller_interface::return_type::OK;
  }

  if (active_joint_index_ < 0 || active_joint_index_ >= (int)N) {
    write_velocity_cmd_(dq_cmd);
    return controller_interface::return_type::OK;
  }

  const int j = active_joint_index_;
  const double t = (time - t0_).seconds();

  // Phase switching
  // if (phase_ == Phase::SETTLE && t >= settle_time_) {
  //   phase_ = Phase::RUN;
  //   RCLCPP_INFO(get_node()->get_logger(), "Joint[%d] entering RUN phase.", j);
  // }

  // if (phase_ == Phase::RUN && t >= (settle_time_ + run_time_)) {
  //   finish_joint_(j);
  //   maybe_advance_joint_(time);
  //   // after maybe_advance_joint_, we might be in SETTLE for next joint or DONE
  //   clamp_effort_(tau_cmd);
  //   write_effort_cmd_(tau_cmd);
  //   return controller_interface::return_type::OK;
  // }

  // Phase switching
  if (phase_ == Phase::SETTLE && t >= settle_time_) {
    phase_ = Phase::RUN;
    RCLCPP_INFO(get_node()->get_logger(), "Joint[%d] entering RUN phase.", j);
  }

  if (phase_ == Phase::RUN && t >= (settle_time_ + run_time_)) {
    finish_joint_(j);
    maybe_advance_joint_(time);
    // after maybe_advance_joint_, we might be in SETTLE for next joint or DONE

    // velocity-mode: stop motion for settle / next joint
    std::vector<double> dq_cmd(N, 0.0);
    write_velocity_cmd_(dq_cmd);
    return controller_interface::return_type::OK;
  }



  // Build velocity reference for active joint
  // const double w = 2.0 * M_PI * v_freq_;
  // const double dq_ref = v_amp_ * std::sin(w * std::max(0.0, t - settle_time_));
  const double Tseg = 2.0;                      // seconds per segment
  const double t_run = std::max(0.0, t - settle_time_);
  const int seg = static_cast<int>(t_run / Tseg) % 2;

  double dq_ref = 0.0;
  switch (seg) {
    case 0: dq_ref = +0.1; break;
    case 1: dq_ref = -0.1; break;
  }
  // const double T = 1.0 / v_freq_;
  // const double tt = std::fmod(std::max(0.0, t - settle_time_), T);
  // const double dq_ref = (tt < 0.5*T) ? +v_amp_ : -v_amp_;


  // Velocity servo torque
//   const double e_v = dq_ref - dq[j];
//   const double Kvj = Kv_joints_.empty() ? Kv_ : Kv_joints_[j];
//   const double Kdj = Kd_joints_.empty() ? Kd_ : Kd_joints_[j];
//   double tau_servo = Kvj * e_v - Kdj * dq[j];
//   tau_servo = std::clamp(tau_servo, -max_servo_tau_, max_servo_tau_);


//   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
//   "dq_ref=%.3f dq=%.3f tau_servo=%.3f (sat=%d) n=%d",
//   dq_ref, dq[j], tau_servo,
//   (std::abs(tau_servo) >= 0.99*max_servo_tau_), fit_[j].n);

//   // Optional friction feedforward after fit
//   double tau_ff = 0.0;
//   if (apply_friction_ff_after_fit_ && joint_done_[j]) {
//     const double s = smooth_sign_(dq[j]);
//     tau_ff = Fc_[j] * s + B_[j] * dq[j] + tau_bias_[j];
//   }

//   const double qj   = q_(j);      // if q_ is KDL::JntArray
// // OR: const double qj = q[j];   // if you store positions in std::vector<double>

//   const double dqj  = dq[j];

//   const double qmin = q_min_[j];
//   const double qmax = q_max_[j];
//   const double m    = q_soft_margin_;

//   double tau_limit = 0.0;

//   if (qj < qmin + m) {
//     tau_limit += k_limit_ * ((qmin + m) - qj) - d_limit_ * dqj;
//   }
//   if (qj > qmax - m) {
//     tau_limit -= k_limit_ * (qj - (qmax - m)) + d_limit_ * dqj;
//   }

//   // Command = gravity + servo (+ optional FF)
//   // tau_cmd[j] = tau_g[j] + tau_servo + tau_ff;
//   tau_cmd[j] = tau_g[j] + tau_servo + tau_limit;

  dq_cmd[j] = dq_ref;

  
  // ----- Only collect during RUN -----
  if (phase_ == Phase::RUN)
  {
    bool accept = true;

    const double v = dq[j];
    const double dt_safe = std::max(1e-4, dt);

    // --- Acceleration gate ---
    const double ddq = (v - dq_prev_[j]) / dt_safe;
    dq_prev_[j] = v;

    const double ddq_max = 0.15;  // tune 0.3~1.0
    if (std::abs(ddq) > ddq_max)
      accept = false;
    if (std::abs(dq_ref) < 0.7 * v_amp_) accept = false;
    const double t_in_seg = std::fmod(t_run, Tseg);
    if (t_in_seg < 0.5) accept = false;                 // wait for settling
    if (std::abs(dq[j] - dq_ref) > 0.03) accept = false; // not tracking well
    if (std::abs(dq[j]) < 0.02) accept = false;

    // --- Reject saturated torque samples ---
    // const bool sat = (std::abs(tau_servo) > 0.98 * max_servo_tau_);
    // if (sat)
    //   accept = false;

    // --- Reject near-zero velocity ---
    if (std::abs(v) < fit_min_speed_)
      accept = false;

    if (accept)
    {
      const double s = smooth_sign_(v);

      // Use command-based friction during debugging
      // const double y = tau_meas[j] - tau_g[j];
      const double y = tau_meas[j] - tau_g[j] - c_kdl_(j);

      fit_[j].add(s, v, y);
    }
  }

  


  

  // clamp_effort_(tau_cmd);
  // write_effort_cmd_(tau_cmd);
  write_velocity_cmd_(dq_cmd);

  // Light debug
  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //   "j=%d phase=%d dq_ref=%.3f dq=%.3f tau_g=%.3f tau_servo=%.3f tau_meas=%.3f n=%d",
  //   j, (int)phase_, dq_ref, dq[j], tau_g[j], tau_servo, tau_meas[j], fit_[j].n);

  return controller_interface::return_type::OK;
}

} // namespace ombot_controller

PLUGINLIB_EXPORT_CLASS(
  ombot_controller::JointFrictionCalibController,
  controller_interface::ControllerInterface)
