#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
// #include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


namespace ombot_controller
{

class JointExcitationController : public controller_interface::ControllerInterface
{
public:
  JointExcitationController() = default;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Config
  std::vector<std::string> joint_names_;
  std::vector<double> effort_limits_;   // optional per-joint clamp
  std::vector<double> amplitudes_;      // per-joint excitation amplitude

  std::string mode_{"prbs"};           // "prbs", "sine", "chirp"
  double prbs_dwell_{0.02};            // s per bit
  double sine_freq_{0.5};              // Hz
  double chirp_f_start_{0.1};          // Hz
  double chirp_f_end_{5.0};            // Hz
  double test_duration_{0.0};          // 0 => infinite
  

  // Runtime
  std::vector<double> tau_cmd_;
  rclcpp::Time start_time_;
  rclcpp::Time next_switch_time_;      // for PRBS
  std::vector<int> prbs_state_;        // ±1 per joint
  std::vector<uint32_t> lfsr_state_;  // one LFSR per joint




std::shared_ptr<
realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> tau_pub_;

  bool active_{false};

  // helpers
  void clamp_effort(std::vector<double> & tau) const;
  void update_prbs(const rclcpp::Time & now);
  void compute_sine(double t);
  void compute_chirp(double t);
  void init_prbs();

};

}  // namespace ombot_controller
