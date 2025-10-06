#include <algorithm>
#include <sstream>

#include "gravity_cancel_controller/gravity_cancel_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

// URDF/KDL parsing
#include "kdl_parser/kdl_parser.hpp"

namespace ombot_controller
{

GravityCancelController::GravityCancelController() = default;

controller_interface::InterfaceConfiguration
GravityCancelController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) conf.names.push_back(j + "/effort");
  return conf;
}

controller_interface::InterfaceConfiguration
GravityCancelController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) conf.names.push_back(j + "/position");
  return conf;
}

controller_interface::CallbackReturn GravityCancelController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_link", "");
    auto_declare<std::string>("tip_link", "");
    auto_declare<std::string>("robot_description", "");      // optional: pass inline
    auto_declare<std::vector<double>>("gravity_xyz", {0.0, 0.0, -9.81});
    auto_declare<double>("scale", 1.0);
    auto_declare<std::vector<double>>("effort_limits", {});  // optional
    auto_declare<double>("alpha", 1.0);                      // 1.0 = no filter
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCancelController::on_configure(const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  base_link_   = get_node()->get_parameter("base_link").as_string();
  tip_link_    = get_node()->get_parameter("tip_link").as_string();
  scale_       = get_node()->get_parameter("scale").as_double();
  alpha_       = std::clamp(get_node()->get_parameter("alpha").as_double(), 0.0, 1.0);

  if (joint_names_.empty() || base_link_.empty() || tip_link_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Params 'joints', 'base_link', and 'tip_link' are required.");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t N = joint_names_.size();

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
    // try local node param named "robot_description"
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

  // urdf::Model urdf_model;
  // if (!urdf_model.initString(urdf_xml)) {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF.");
  //   return controller_interface::CallbackReturn::ERROR;
  // }

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
  last_cmd_.assign(N, 0.0);

  RCLCPP_INFO(get_node()->get_logger(),
              "GravityCancelController configured: base='%s', tip='%s', N=%zu, scale=%.3f, alpha=%.2f",
              base_link_.c_str(), tip_link_.c_str(), N, scale_, alpha_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCancelController::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) ci.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityCancelController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) ci.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GravityCancelController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const size_t N = joint_names_.size();

  if (command_interfaces_.size() != N || state_interfaces_.size() != N) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Interface size mismatch: cmd=%zu, state=%zu, expected=%zu",
      command_interfaces_.size(), state_interfaces_.size(), N);
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < N; ++i) q_(i) = state_interfaces_[i].get_value();

  const int rc = dyn_->JntToGravity(q_, g_);
  if (rc != 0) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "KDL::JntToGravity rc=%d", rc);
    return controller_interface::return_type::OK;
  }

  for (size_t i = 0; i < N; ++i) {
    const double raw_tau = scale_ * g_(i);
    const double filtered = alpha_ * raw_tau + (1.0 - alpha_) * last_cmd_[i];

    double out = filtered;
    if (!effort_limits_.empty()) {
      const double lim = std::abs(effort_limits_[i]);
      out = std::clamp(out, -lim, lim);
    }

    last_cmd_[i] = out;
    command_interfaces_[i].set_value(out);

  }

  return controller_interface::return_type::OK;
}

} // namespace ombot_controller

PLUGINLIB_EXPORT_CLASS(
  ombot_controller::GravityCancelController,
  controller_interface::ControllerInterface)
