#include <algorithm>
#include <cmath>

#include "cartesian_impedance_controller/cartesian_impedance_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace ombot_controller
{

CartesianImpedanceController::CartesianImpedanceController() = default;

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) conf.names.push_back(j + "/effort");
  return conf;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) conf.names.push_back(j + "/position");
  for (const auto & j : joint_names_) conf.names.push_back(j + "/velocity");
  return conf;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("effort_limits", {});
    auto_declare<bool>("use_gravity", true);
    auto_declare<std::vector<double>>("gravity_xyz", {0.0, 0.0, -9.81});
    auto_declare<std::string>("base_link", "");
    auto_declare<std::string>("tip_link", "");
    auto_declare<std::string>("robot_description", "");
    auto_declare<double>("vel_lpf_alpha", 1.0);

    auto_declare<std::vector<double>>("Kp_xyz", {200,200,200});
    auto_declare<std::vector<double>>("Kd_xyz", {30,30,30});
    auto_declare<std::vector<double>>("Kr_xyz", {20,20,20});
    auto_declare<std::vector<double>>("Dr_xyz", {2,2,2});
    auto_declare<double>("wrench_force_limit", 80.0);
    auto_declare<double>("wrench_torque_limit", 10.0);

    auto_declare<bool>("publish_ee_pose", true);
    auto_declare<std::string>("ee_pose_topic", "/ee_pose");

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

static std::array<double,3> vec3_from_param(const std::vector<double>& v, const std::array<double,3>& def)
{
  if (v.size() != 3) return def;
  return {v[0], v[1], v[2]};
}

controller_interface::CallbackReturn
CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State &)
{
  joint_names_   = get_node()->get_parameter("joints").as_string_array();
  effort_limits_ = get_node()->get_parameter("effort_limits").as_double_array();
  use_gravity_   = get_node()->get_parameter("use_gravity").as_bool();
  vel_lpf_alpha_ = std::clamp(get_node()->get_parameter("vel_lpf_alpha").as_double(), 0.0, 1.0);

  publish_ee_pose_ = get_node()->get_parameter("publish_ee_pose").as_bool();
  ee_pose_topic_   = get_node()->get_parameter("ee_pose_topic").as_string();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Param 'joints' must be non-empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t N = joint_names_.size();

  if (!effort_limits_.empty() && effort_limits_.size() != N) {
    RCLCPP_WARN(get_node()->get_logger(),
      "effort_limits size (%zu) != joints (%zu). Ignoring limits.", effort_limits_.size(), N);
    effort_limits_.clear();
  }

  kp_xyz_ = vec3_from_param(get_node()->get_parameter("Kp_xyz").as_double_array(), kp_xyz_);
  kd_xyz_ = vec3_from_param(get_node()->get_parameter("Kd_xyz").as_double_array(), kd_xyz_);
  kr_xyz_ = vec3_from_param(get_node()->get_parameter("Kr_xyz").as_double_array(), kr_xyz_);
  dr_xyz_ = vec3_from_param(get_node()->get_parameter("Dr_xyz").as_double_array(), dr_xyz_);

  wrench_force_limit_  = std::abs(get_node()->get_parameter("wrench_force_limit").as_double());
  wrench_torque_limit_ = std::abs(get_node()->get_parameter("wrench_torque_limit").as_double());

  // gravity vector
  {
    auto g = get_node()->get_parameter("gravity_xyz").as_double_array();
    if (g.size() != 3) g = {0.0, 0.0, -9.81};
    gravity_vec_ = KDL::Vector(g[0], g[1], g[2]);
  }

  base_link_ = get_node()->get_parameter("base_link").as_string();
  tip_link_  = get_node()->get_parameter("tip_link").as_string();
  robot_description_ = get_node()->get_parameter("robot_description").as_string();

  if (base_link_.empty() || tip_link_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Set 'base_link' and 'tip_link'.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (robot_description_.empty()) {
    rclcpp::Parameter p;
    if (get_node()->get_parameter("robot_description", p) &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      robot_description_ = p.as_string();
    }
  }
  if (robot_description_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "URDF not provided. Set 'robot_description'.");
    return controller_interface::CallbackReturn::ERROR;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_description_, tree)) {
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
      "KDL chain DOF (%u) != joints size (%zu).", chain_.getNrOfJoints(), N);
    return controller_interface::CallbackReturn::ERROR;
  }

  fk_solver_  = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  J_kdl_      = KDL::Jacobian(N);

  if (use_gravity_) {
    dyn_   = std::make_unique<KDL::ChainDynParam>(chain_, gravity_vec_);
    q_kdl_ = KDL::JntArray(N);
    g_kdl_ = KDL::JntArray(N);
  } else {
    q_kdl_ = KDL::JntArray(N);
  }

  // Runtime buffers
  q_.assign(N, 0.0);
  dq_.assign(N, 0.0);
  dq_filt_.assign(N, 0.0);
  tau_cmd_.assign(N, 0.0);

  // Desired buffer init (invalid until activate)
  des_shadow_ = CartDesired{};
  des_shadow_.has_pose = false;
  des_shadow_.has_twist = false;
  des_shadow_.has_wrench_ff = false;
  des_rt_.writeFromNonRT(des_shadow_);

  // Subs
  sub_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/command_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianImpedanceController::pose_cb, this, std::placeholders::_1));

  sub_twist_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/command_twist", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianImpedanceController::twist_cb, this, std::placeholders::_1));

  sub_wrench_ff_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "~/wrench_ff", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianImpedanceController::wrench_ff_cb, this, std::placeholders::_1));

  // EE pose publisher optional
  if (publish_ee_pose_) {
    ee_pub_ = std::make_shared<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
          ee_pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()));
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "CartesianImpedanceController configured: N=%zu, base=%s tip=%s",
    N, base_link_.c_str(), tip_link_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) ci.set_value(0.0);

  const size_t N = joint_names_.size();
  if (state_interfaces_.size() != N * 2 || command_interfaces_.size() != N) {
    RCLCPP_ERROR(get_node()->get_logger(), "Interface size mismatch on activate.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // read initial q/dq
  for (size_t i = 0; i < N; ++i) {
    q_[i]  = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[N + i].get_value();
  }
  dq_filt_ = dq_;

  // Set desired pose to current EE pose (no jump on startup)
  for (size_t i = 0; i < N; ++i) q_kdl_(i) = q_[i];
  KDL::Frame T;
  fk_solver_->JntToCart(q_kdl_, T);

  if (reference_interfaces_.size() == 12) {
    reference_interfaces_[0] = T.p.x();
    reference_interfaces_[1] = T.p.y();
    reference_interfaces_[2] = T.p.z();

    reference_interfaces_[3] = 0.0;
    reference_interfaces_[4] = 0.0;
    reference_interfaces_[5] = 0.0;

    for (int i = 6; i < 12; ++i) reference_interfaces_[i] = 0.0;
  }

  

  CartDesired d = *(des_rt_.readFromRT());
  d.pd = T.p;
  d.Rd = T.M;
  d.vd = KDL::Vector::Zero();
  d.wd = KDL::Vector::Zero();
  d.f_ff = KDL::Vector::Zero();
  d.tau_ff = KDL::Vector::Zero();
  d.has_pose = true;
  d.has_twist = true;
  d.has_wrench_ff = false;
  des_rt_.writeFromNonRT(d);

  // set frame_id for ee pose pub
  if (publish_ee_pose_ && ee_pub_ && ee_pub_->trylock()) {
    ee_pub_->msg_.header.frame_id = base_link_;
    ee_pub_->unlock();
  }


  

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) ci.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Assumption: msg is already in base_link (keep it simple)
  CartDesired d = *(des_rt_.readFromRT());
  d.pd = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  d.Rd = KDL::Rotation::Quaternion(
    msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w);
  d.has_pose = true;
  des_rt_.writeFromNonRT(d);
}

void CartesianImpedanceController::twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  CartDesired d = *(des_rt_.readFromRT());
  d.vd = KDL::Vector(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  d.wd = KDL::Vector(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
  d.has_twist = true;
  des_rt_.writeFromNonRT(d);
}

void CartesianImpedanceController::wrench_ff_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  CartDesired d = *(des_rt_.readFromRT());
  d.f_ff   = KDL::Vector(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
  d.tau_ff = KDL::Vector(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
  d.has_wrench_ff = true;
  des_rt_.writeFromNonRT(d);
}

controller_interface::return_type
CartesianImpedanceController::update_reference_from_subscribers()
{
  // No extra staging needed; callbacks write directly to RT buffer.
  return controller_interface::return_type::OK;
}

void CartesianImpedanceController::maybe_filter_velocity()
{
  if (vel_lpf_alpha_ >= 0.9999) {
    dq_filt_ = dq_;
    return;
  }
  for (size_t i = 0; i < dq_.size(); ++i) {
    dq_filt_[i] = vel_lpf_alpha_ * dq_[i] + (1.0 - vel_lpf_alpha_) * dq_filt_[i];
  }
}

void CartesianImpedanceController::compute_gravity(const std::vector<double>& q, std::vector<double>& tau_g)
{
  if (!use_gravity_ || !dyn_) { tau_g.assign(q.size(), 0.0); return; }
  for (size_t i = 0; i < q.size(); ++i) q_kdl_(i) = q[i];
  const int rc = dyn_->JntToGravity(q_kdl_, g_kdl_);
  if (rc != 0) {
    tau_g.assign(q.size(), 0.0);
    return;
  }
  tau_g.resize(q.size());
  for (size_t i = 0; i < q.size(); ++i) tau_g[i] = g_kdl_(i);
}

// bool CartesianImpedanceController::on_set_chained_mode(bool chained)
// {
//   chained_mode_ = chained;
//   RCLCPP_INFO(get_node()->get_logger(), "Chained mode %s", chained ? "enabled" : "disabled");
//   return true;
// }

bool CartesianImpedanceController::on_set_chained_mode(bool /*chained*/)
{
  return true;
}



std::vector<hardware_interface::CommandInterface>
CartesianImpedanceController::on_export_reference_interfaces()
{
  const size_t Nref = 12;
  reference_interfaces_.assign(Nref, 0.0);

  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve(Nref);

  const std::string ctrl = get_node()->get_name();  // instance-safe

  // pose (6): x y z rx ry rz  (rotation vector)
  refs.emplace_back(ctrl, "ee_ref/x",  &reference_interfaces_[0]);
  refs.emplace_back(ctrl, "ee_ref/y",  &reference_interfaces_[1]);
  refs.emplace_back(ctrl, "ee_ref/z",  &reference_interfaces_[2]);
  refs.emplace_back(ctrl, "ee_ref/rx", &reference_interfaces_[3]);
  refs.emplace_back(ctrl, "ee_ref/ry", &reference_interfaces_[4]);
  refs.emplace_back(ctrl, "ee_ref/rz", &reference_interfaces_[5]);

  // twist (6): vx vy vz wx wy wz
  refs.emplace_back(ctrl, "ee_ref/vx", &reference_interfaces_[6]);
  refs.emplace_back(ctrl, "ee_ref/vy", &reference_interfaces_[7]);
  refs.emplace_back(ctrl, "ee_ref/vz", &reference_interfaces_[8]);
  refs.emplace_back(ctrl, "ee_ref/wx", &reference_interfaces_[9]);
  refs.emplace_back(ctrl, "ee_ref/wy", &reference_interfaces_[10]);
  refs.emplace_back(ctrl, "ee_ref/wz", &reference_interfaces_[11]);

  return refs;
}


void CartesianImpedanceController::clamp_effort(std::vector<double>& tau) const
{
  if (effort_limits_.empty()) return;
  for (size_t i = 0; i < tau.size(); ++i) {
    const double lim = std::abs(effort_limits_[i]);
    tau[i] = std::clamp(tau[i], -lim, lim);
  }
}

controller_interface::return_type
CartesianImpedanceController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const size_t N = joint_names_.size();
  if (state_interfaces_.size() != N * 2 || command_interfaces_.size() != N) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Interface size mismatch: state=%zu (expected %zu), cmd=%zu (expected %zu)",
      state_interfaces_.size(), N * 2, command_interfaces_.size(), N);
    return controller_interface::return_type::ERROR;
  }

  // Read state
  for (size_t i = 0; i < N; ++i) {
    q_[i]  = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[N + i].get_value();
  }
  maybe_filter_velocity();

  // FK + J
  for (size_t i = 0; i < N; ++i) q_kdl_(i) = q_[i];

  KDL::Frame T;
  fk_solver_->JntToCart(q_kdl_, T);
  jac_solver_->JntToJac(q_kdl_, J_kdl_);

  const KDL::Vector p = T.p;
  const KDL::Rotation R = T.M;


  // --- desired from exported refs (12) ---
  // Position target (absolute)
  KDL::Vector pd(reference_interfaces_[0],
                reference_interfaces_[1],
                reference_interfaces_[2]);

  // Orientation target (delta rotvec applied to current orientation)
  KDL::Rotation Rcur = R;

  const double rx = reference_interfaces_[3];
  const double ry = reference_interfaces_[4];
  const double rz = reference_interfaces_[5];
  const double ang = std::sqrt(rx*rx + ry*ry + rz*rz);

  KDL::Rotation Rd;
  if (ang < 1e-9) {
    Rd = Rcur;  // zero rotvec = hold current orientation
  } else {
    const KDL::Vector axis(rx/ang, ry/ang, rz/ang);
    Rd = KDL::Rotation::Rot(axis, ang) * Rcur;
  }

  // Twist targets
  KDL::Vector vd(reference_interfaces_[6],
                reference_interfaces_[7],
                reference_interfaces_[8]);

  KDL::Vector wd(reference_interfaces_[9],
                reference_interfaces_[10],
                reference_interfaces_[11]);


  // Current twist xdot = J * dq
  double vx=0, vy=0, vz=0, wx=0, wy=0, wz=0;
  for (size_t j = 0; j < N; ++j) {
    const double dqj = dq_filt_[j];
    vx += J_kdl_(0,j) * dqj;  vy += J_kdl_(1,j) * dqj;  vz += J_kdl_(2,j) * dqj;
    wx += J_kdl_(3,j) * dqj;  wy += J_kdl_(4,j) * dqj;  wz += J_kdl_(5,j) * dqj;
  }

  // Desired
  // CartDesired des = *(des_rt_.readFromRT());
  // if (!des.has_pose) {
  //   des.pd = p;
  //   des.Rd = R;
  // }
  // if (!des.has_twist) {
  //   des.vd = KDL::Vector::Zero();
  //   des.wd = KDL::Vector::Zero();
  // }
  // if (!des.has_wrench_ff) {
  //   des.f_ff = KDL::Vector::Zero();
  //   des.tau_ff = KDL::Vector::Zero();
  // }

  // Errors
  KDL::Vector e_p = pd - p;

  KDL::Rotation R_err = Rd * R.Inverse();
  KDL::Vector axis_err;
  double angle_err = 0.0;
  R_err.GetRotAngle(axis_err, angle_err);
  KDL::Vector e_R = axis_err * angle_err;

  KDL::Vector e_v = vd - KDL::Vector(vx,vy,vz);
  KDL::Vector e_w = wd - KDL::Vector(wx,wy,wz);


  // Wrench (diagonal gains)
  auto clamp = [](double x, double lim) {
    return std::clamp(x, -lim, lim);
  };

  const double fx = clamp(
    kp_xyz_[0] * e_p.x() + kd_xyz_[0] * e_v.x(),
    wrench_force_limit_
  );

  const double fy = clamp(
    kp_xyz_[1] * e_p.y() + kd_xyz_[1] * e_v.y(),
    wrench_force_limit_
  );

  const double fz = clamp(
    kp_xyz_[2] * e_p.z() + kd_xyz_[2] * e_v.z(),
    wrench_force_limit_
  );

  const double tx = clamp(
    kr_xyz_[0] * e_R.x() + dr_xyz_[0] * e_w.x(),
    wrench_torque_limit_
  );

  const double ty = clamp(
    kr_xyz_[1] * e_R.y() + dr_xyz_[1] * e_w.y(),
    wrench_torque_limit_
  );

  const double tz = clamp(
    kr_xyz_[2] * e_R.z() + dr_xyz_[2] * e_w.z(),
    wrench_torque_limit_
  );


  // tau = J^T * F
  for (size_t j = 0; j < N; ++j) {
    double tau = 0.0;
    tau += J_kdl_(0,j) * fx + J_kdl_(1,j) * fy + J_kdl_(2,j) * fz;
    tau += J_kdl_(3,j) * tx + J_kdl_(4,j) * ty + J_kdl_(5,j) * tz;
    tau_cmd_[j] = tau;
  }

  // Gravity
  if (use_gravity_) {
    std::vector<double> tau_g;
    compute_gravity(q_, tau_g);
    if (tau_g.size() == N) {
      for (size_t j = 0; j < N; ++j) tau_cmd_[j] += tau_g[j];
    }
  }

  // Clamp torques and write
  clamp_effort(tau_cmd_);
  for (size_t i = 0; i < N; ++i) command_interfaces_[i].set_value(tau_cmd_[i]);

  // Optional EE pose pub (same as your joint controller)
  if (publish_ee_pose_ && ee_pub_ && ee_pub_->trylock()) {
    auto & msg = ee_pub_->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = base_link_;
    msg.pose.position.x = p.x();
    msg.pose.position.y = p.y();
    msg.pose.position.z = p.z();
    double qx,qy,qz,qw; R.GetQuaternion(qx,qy,qz,qw);
    msg.pose.orientation.x = qx;
    msg.pose.orientation.y = qy;
    msg.pose.orientation.z = qz;
    msg.pose.orientation.w = qw;
    ee_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

} // namespace ombot_controller

PLUGINLIB_EXPORT_CLASS(
  ombot_controller::CartesianImpedanceController,
  controller_interface::ChainableControllerInterface)
