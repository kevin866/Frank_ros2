#pragma once

#include <string>
#include <vector>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ombot_controller
{

class EffortController : public controller_interface::ControllerInterface
{
public:
  EffortController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // One effort command interface per joint
    for (const auto & j : joint_names_) {
      conf.names.push_back(j + "/effort");
    }
    return conf;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    // We don't require any state interfaces, but you could request position/velocity/effort here.
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::NONE;
    return conf;
  }

  controller_interface::CallbackReturn on_init() override
  {
    try {
      auto_declare<std::vector<std::string>>("joints", {});
      auto_declare<std::string>("topic_name", "effort_cmd");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &) override
  {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
      return controller_interface::CallbackReturn::ERROR;
    }

    topic_name_ = get_node()->get_parameter("topic_name").as_string();
    if (topic_name_.empty()) topic_name_ = "effort_cmd";

    // Prepare RT buffer with zero torques
    latest_cmd_.writeFromNonRT(std::vector<double>(joint_names_.size(), 0.0));

    // Subscriber (non-RT) -> RT buffer
    cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_name_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        if (msg->data.size() != joint_names_.size()) {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "Received %zu efforts, but controller expects %zu. Ignoring.",
            msg->data.size(), joint_names_.size());
          return;
        }
        latest_cmd_.writeFromNonRT(msg->data);
      });

    RCLCPP_INFO(get_node()->get_logger(),
      "EffortController configured. Listening on '%s' for %zu torques.",
      topic_name_.c_str(), joint_names_.size());
    std::vector<double> default_effort(6, 0.0);   // <-- make sure this is in scope

    if (get_node()->has_parameter("fixed_effort")) {
      // already declared (from YAML or a prior configure)
      get_node()->get_parameter("fixed_effort", fixed_cmd_);
    } else {
      fixed_cmd_ =
        get_node()->declare_parameter<std::vector<double>>("fixed_effort", default_effort);
    }


    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &) override
  {
    // Reset commands to zero on activate
    latest_cmd_.writeFromNonRT(std::vector<double>(joint_names_.size(), 0.0));
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &) override
  {
    // Zero out commands when deactivating
    for (auto & ci : command_interfaces_) {
      ci.set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
      const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const auto * cmd = latest_cmd_.readFromRT();
    if (!cmd) return controller_interface::return_type::OK;

    // Safety: size check
    const size_t N = joint_names_.size();
    if (command_interfaces_.size() != N) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Command interface size mismatch: have %zu, expected %zu.",
        command_interfaces_.size(), N);
      return controller_interface::return_type::ERROR;
    }

    std::ostringstream oss;
    for (const auto &val : fixed_cmd_)
    {
      oss << val << " ";
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("effort_controller"),
                      "fixed_cmd_: " << oss.str());



    // Write torques to effort command interfaces (Nm)
    for (size_t i = 0; i < N; ++i) {
      // command_interfaces_[i].set_value((*cmd)[i]);
      command_interfaces_[i].set_value(fixed_cmd_[i]); // Nm

    }
    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> joint_names_;
  std::string topic_name_;
  std::vector<double> fixed_cmd_;  // <-- add this
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  realtime_tools::RealtimeBuffer<std::vector<double>> latest_cmd_;
};

}  // namespace ombot_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ombot_controller::EffortController, controller_interface::ControllerInterface)
