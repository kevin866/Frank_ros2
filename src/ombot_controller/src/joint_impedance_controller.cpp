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
  if (use_gravity_) {
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

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointImpedanceController::on_activate(const rclcpp_lifecycle::State &)
{
  // Initialize command to zero
  for (auto & ci : command_interfaces_) ci.set_value(0.0);

  const size_t N = joint_names_.size();
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
JointImpedanceController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const size_t N = joint_names_.size();
  const double dt = period.seconds();

  // Validate interfaces
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
  maybe_filter_velocity(dt);  // dq_filt_ updated

  // Desired command (real-time safe)
  Desired d = *(desired_rt_.readFromRT());
  // If first run and no command yet, hold position at current q
  if (!d.has_qd) {
    d.qd   = q_;
    d.has_qd = true;
  }
  if (!d.has_dqd) {
    d.dqd.assign(N, 0.0);
    d.has_dqd = true;
  }
  if (!d.has_tau_ff) {
    d.tau_ff.assign(N, 0.0);
    d.has_tau_ff = true;
  }

  // Gravity feedforward
  std::vector<double> tau_g;
  compute_gravity(q_, tau_g);

  // Impedance law
  for (size_t i = 0; i < N; ++i) {
    const double e_pos = d.qd[i]  - q_[i];
    const double e_vel = d.dqd[i] - dq_filt_[i];
    double tau = kp_[i] * e_pos + kd_[i] * e_vel + d.tau_ff[i];
    if (use_gravity_) tau += tau_g[i];
    tau_cmd_[i] = tau;
  }

  // Clamp & write
  clamp_effort(tau_cmd_);
  for (size_t i = 0; i < N; ++i){
    command_interfaces_[i].set_value(tau_cmd_[i]);
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
      "Joint " << joint_names_[i] << " command = " << tau_cmd_[i]);
  } 



  return controller_interface::return_type::OK;
}

} // namespace ombot_controller

PLUGINLIB_EXPORT_CLASS(
  ombot_controller::JointImpedanceController,
  controller_interface::ControllerInterface)
