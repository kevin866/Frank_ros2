#include "ee_twist_velocity_controller/ee_twist_velocity_controller.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace ombot_controller
{

controller_interface::InterfaceConfiguration
EeTwistVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) conf.names.push_back(j + "/velocity");
  return conf;
}

controller_interface::InterfaceConfiguration
EeTwistVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) {
    conf.names.push_back(j + "/position");
    conf.names.push_back(j + "/velocity");
  }
  return conf;
}

controller_interface::CallbackReturn EeTwistVelocityController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_link", "");
    auto_declare<std::string>("tip_link", "");
    auto_declare<std::string>("robot_description", "");

    auto_declare<std::string>("twist_topic", "/ee_twist_cmd");
    auto_declare<double>("cmd_timeout", 0.2);

    auto_declare<double>("max_qdot", 1.5);
    auto_declare<double>("max_task_lin", 0.3);
    auto_declare<double>("max_task_ang", 1.0);

    auto_declare<double>("lambda", 0.02);
    auto_declare<double>("cmd_alpha", 1.0);

    auto_declare<bool>("use_posture_nullspace", false);
    auto_declare<double>("null_kp", 0.0);
    auto_declare<std::vector<double>>("q_nom", {});

    auto_declare<bool>("use_soft_limits", false);
    auto_declare<std::vector<double>>("q_min", {});
    auto_declare<std::vector<double>>("q_max", {});
    auto_declare<double>("q_soft_margin", 0.2);
    auto_declare<double>("k_limit", 2.0);
    auto_declare<double>("d_limit", 0.0);

    auto_declare<std::string>("e1_topic", "/debug/e1");
    auto_declare<double>("e1_scale", 1.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EeTwistVelocityController::on_configure(const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  base_link_   = get_node()->get_parameter("base_link").as_string();
  tip_link_    = get_node()->get_parameter("tip_link").as_string();

  twist_topic_ = get_node()->get_parameter("twist_topic").as_string();
  cmd_timeout_ = get_node()->get_parameter("cmd_timeout").as_double();

  max_qdot_      = get_node()->get_parameter("max_qdot").as_double();
  max_task_lin_  = get_node()->get_parameter("max_task_lin").as_double();
  max_task_ang_  = get_node()->get_parameter("max_task_ang").as_double();
  lambda_        = get_node()->get_parameter("lambda").as_double();
  cmd_alpha_     = std::clamp(get_node()->get_parameter("cmd_alpha").as_double(), 0.0, 1.0);

  use_posture_nullspace_ = get_node()->get_parameter("use_posture_nullspace").as_bool();
  null_kp_ = get_node()->get_parameter("null_kp").as_double();
  q_nom_   = get_node()->get_parameter("q_nom").as_double_array();

  use_soft_limits_ = get_node()->get_parameter("use_soft_limits").as_bool();
  q_min_ = get_node()->get_parameter("q_min").as_double_array();
  q_max_ = get_node()->get_parameter("q_max").as_double_array();
  q_soft_margin_ = get_node()->get_parameter("q_soft_margin").as_double();
  k_limit_ = get_node()->get_parameter("k_limit").as_double();
  d_limit_ = get_node()->get_parameter("d_limit").as_double();

  e1_topic_ = get_node()->get_parameter("e1_topic").as_string();
  e1_scale_ = get_node()->get_parameter("e1_scale").as_double();

  if (joint_names_.empty() || base_link_.empty() || tip_link_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Params 'joints', 'base_link', 'tip_link' are required.");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t N = joint_names_.size();

  if (use_posture_nullspace_) {
    if (q_nom_.size() != N) {
      RCLCPP_ERROR(get_node()->get_logger(), "q_nom must be size N when use_posture_nullspace=true");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  if (use_soft_limits_) {
    if (q_min_.size() != N || q_max_.size() != N) {
      RCLCPP_ERROR(get_node()->get_logger(), "q_min/q_max must be size N when use_soft_limits=true");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // robot_description
  robot_description_ = get_node()->get_parameter("robot_description").as_string();
  if (robot_description_.empty()) {
    rclcpp::Parameter p;
    if (get_node()->get_parameter("robot_description", p) &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      robot_description_ = p.as_string();
    }
  }
  if (robot_description_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "URDF not provided. Set 'robot_description' on this node.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Build KDL chain
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_description_, tree)) {
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
                 "KDL chain DOF (%u) != joints size (%zu).",
                 chain_.getNrOfJoints(), N);
    return controller_interface::CallbackReturn::ERROR;
  }

  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  q_kdl_ = KDL::JntArray(N);
  dq_kdl_ = KDL::JntArray(N);
  J_kdl_ = KDL::Jacobian(N);

  dq_cmd_prev_.assign(N, 0.0);

  // subscriber
  twist_sub_ = get_node()->create_subscription<TwistMsg>(
    twist_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&EeTwistVelocityController::twist_cb_, this, std::placeholders::_1));

  rt_twist_.writeFromNonRT(std::shared_ptr<TwistMsg>(nullptr));

  RCLCPP_INFO(get_node()->get_logger(),
              "EeTwistVelocityController configured. N=%zu topic=%s lambda=%.4f timeout=%.3f",
              N, twist_topic_.c_str(), lambda_, cmd_timeout_);

  e1_sub_ = get_node()->create_subscription<Vector3Msg>(
    e1_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const Vector3Msg::SharedPtr msg)
    {
      const double x = msg->vector.x;
      const double y = msg->vector.y;
      const double z = msg->vector.z;
      const double n = std::sqrt(x*x + y*y + z*z);

      std::lock_guard<std::mutex> lk(e1_mtx_);
      e1_norm_ = n;
      e1_stamp_ = msg->header.stamp;
      have_e1_ = true;
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EeTwistVelocityController::on_activate(const rclcpp_lifecycle::State &)
{
  active_ = true;
  last_cmd_time_ = get_node()->now();

  // zero commands
  for (auto & ci : command_interfaces_) ci.set_value(0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
EeTwistVelocityController::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  for (auto & ci : command_interfaces_) ci.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

void EeTwistVelocityController::twist_cb_(const TwistMsg::SharedPtr msg)
{
  rt_twist_.writeFromNonRT(msg);
}

void EeTwistVelocityController::write_velocity_cmd_(const std::vector<double> & dq_cmd)
{
  for (size_t i = 0; i < dq_cmd.size(); ++i) {
    command_interfaces_[i].set_value(dq_cmd[i]);
  }
}

void EeTwistVelocityController::clamp_task_(Eigen::Matrix<double,6,1> & v) const
{
  // linear
  const double vx = std::clamp(v(0), -max_task_lin_, max_task_lin_);
  const double vy = std::clamp(v(1), -max_task_lin_, max_task_lin_);
  const double vz = std::clamp(v(2), -max_task_lin_, max_task_lin_);
  // angular
  const double wx = std::clamp(v(3), -max_task_ang_, max_task_ang_);
  const double wy = std::clamp(v(4), -max_task_ang_, max_task_ang_);
  const double wz = std::clamp(v(5), -max_task_ang_, max_task_ang_);
  v << vx, vy, vz, wx, wy, wz;
}

void EeTwistVelocityController::apply_soft_limits_(const std::vector<double>& q,
                                                  const std::vector<double>& dq,
                                                  std::vector<double>& dq_cmd) const
{
  if (!use_soft_limits_) return;

  const size_t N = q.size();
  const double m = q_soft_margin_;

  for (size_t i = 0; i < N; ++i) {
    double push = 0.0;

    if (q[i] < q_min_[i] + m) {
      push += k_limit_ * ((q_min_[i] + m) - q[i]) - d_limit_ * dq[i];
    }
    if (q[i] > q_max_[i] - m) {
      push -= k_limit_ * (q[i] - (q_max_[i] - m)) + d_limit_ * dq[i];
    }

    // push is a velocity correction term (rad/s)
    dq_cmd[i] += push;
  }
}

controller_interface::return_type
EeTwistVelocityController::update(const rclcpp::Time & time,
                                  const rclcpp::Duration & period)
{
  if (!active_) return controller_interface::return_type::OK;

  const size_t N = joint_names_.size();
  const size_t expected_cmd = N;
  const size_t expected_state = 2 * N;
  const double dt = std::max(1e-4, period.seconds());

  if (command_interfaces_.size() != expected_cmd || state_interfaces_.size() != expected_state) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Interface size mismatch: cmd=%zu (exp %zu), state=%zu (exp %zu)",
      command_interfaces_.size(), expected_cmd, state_interfaces_.size(), expected_state);
    return controller_interface::return_type::ERROR;
  }

  // Read state
  std::vector<double> q(N), dq(N);
  for (size_t i = 0; i < N; ++i) {
    q[i]  = state_interfaces_[2*i + 0].get_value();
    dq[i] = state_interfaces_[2*i + 1].get_value();
    q_kdl_(i) = q[i];
    dq_kdl_(i) = dq[i];
  }

  // Fetch latest twist cmd
  auto msg = rt_twist_.readFromRT();
  Eigen::Matrix<double,6,1> v_task;
  v_task.setZero();

  const rclcpp::Time now = time;

  bool have_cmd = false;
  if (msg && *msg) {
    // NOTE: we treat msg->header.stamp as optional; we use arrival-time watchdog
    v_task(0) = (*msg)->twist.linear.x;
    v_task(1) = (*msg)->twist.linear.y;
    v_task(2) = (*msg)->twist.linear.z;
    v_task(3) = (*msg)->twist.angular.x;
    v_task(4) = (*msg)->twist.angular.y;
    v_task(5) = (*msg)->twist.angular.z;
    have_cmd = true;
    last_cmd_time_ = get_node()->now();
  }

  // watchdog: if no cmd recently, stop
  if (!have_cmd) {
    const double age = (get_node()->now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_) {
      std::vector<double> dq_cmd(N, 0.0);
      write_velocity_cmd_(dq_cmd);
      return controller_interface::return_type::OK;
    }
  }

  clamp_task_(v_task);

  // Compute Jacobian
  if (jac_solver_->JntToJac(q_kdl_, J_kdl_) != 0) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "JntToJac failed; commanding zero velocity.");
    std::vector<double> dq_cmd(N, 0.0);
    write_velocity_cmd_(dq_cmd);
    return controller_interface::return_type::OK;
  }

  // Convert KDL Jacobian -> Eigen (6xN)
  Eigen::MatrixXd J(6, (int)N);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < (int)N; ++c) {
      J(r, c) = J_kdl_(r, c);
    }
  }

  // Damped least squares:
  // dq = J^T (J J^T + lambda^2 I)^-1 v
  Eigen::Matrix<double,6,6> A = J * J.transpose();
  A.diagonal().array() += (lambda_ * lambda_);
  Eigen::Matrix<double,6,1> x = A.ldlt().solve(v_task);
  Eigen::VectorXd dq_cmd_e = J.transpose() * x;

  // Optional nullspace posture: dq += (I - J# J) * ( -kp (q - q_nom) )
  if (use_posture_nullspace_ && null_kp_ > 0.0) {
    double e1_mag = 0.0;
    rclcpp::Time e1_stamp;
    bool have_e1 = false;

    {
      std::lock_guard<std::mutex> lk(e1_mtx_);
      e1_mag = e1_norm_;
      e1_stamp = e1_stamp_;
      have_e1 = have_e1_;
    }

    if (!have_e1) {
      e1_mag = e1_scale_;
    }

    const double denom = std::max(1e-6, e1_scale_);
    const double e1_mag_n = std::clamp(e1_mag / denom, 0.0, 1.0);
    const double w = std::pow(1.0 - e1_mag_n, 3.0);
    const double null_scale_adapt = w;

    // RCLCPP_INFO_THROTTLE(
    //   get_node()->get_logger(),
    //   *get_node()->get_clock(),
    //   100,
    //   "E1: ||e1||=%.4f norm=%.4f w=%.4f null_scale=%.4f have_e1=%d",
    //   e1_mag,
    //   e1_mag_n,
    //   w,
    //   null_scale_adapt,
    //   static_cast<int>(have_e1)
    // );

    Eigen::MatrixXd Jsharp = J.transpose() * A.inverse();
    Eigen::MatrixXd Nproj =
      Eigen::MatrixXd::Identity((int)N, (int)N) - (Jsharp * J);

    Eigen::VectorXd qerr((int)N);
    for (int i = 0; i < (int)N; ++i) {
      qerr(i) = q[i] - q_nom_[(size_t)i];
    }

    Eigen::VectorXd dq_null = (-null_kp_) * qerr;
    dq_null *= null_scale_adapt;
    dq_cmd_e += dq_null;
  }

  // Clamp joint velocities
  std::vector<double> dq_cmd(N, 0.0);
  for (size_t i = 0; i < N; ++i) {
    dq_cmd[i] = std::clamp((double)dq_cmd_e((int)i), -max_qdot_, max_qdot_);
  }

  // Soft limits push-back
  apply_soft_limits_(q, dq, dq_cmd);

  // Optional smoothing (1st order IIR on dq command)
  if (cmd_alpha_ < 1.0) {
    const double a = cmd_alpha_;
    for (size_t i = 0; i < N; ++i) {
      dq_cmd[i] = a * dq_cmd[i] + (1.0 - a) * dq_cmd_prev_[i];
      dq_cmd_prev_[i] = dq_cmd[i];
    }
  }

  write_velocity_cmd_(dq_cmd);
  return controller_interface::return_type::OK;
}

} // namespace ombot_controller

PLUGINLIB_EXPORT_CLASS(
  ombot_controller::EeTwistVelocityController,
  controller_interface::ControllerInterface)