#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ombot_controller
{

class JointVelocityExcitationController : public controller_interface::ControllerInterface
{
public:
  JointVelocityExcitationController() = default;

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
  // -----------------------------
  // Config (params)
  // -----------------------------
  std::vector<std::string> joint_names_;

  // If commanding velocity: per-joint clamp in rad/s (optional).
  // Keep the name velocity_limits_ even if your param is called "effort_limits" before.
  std::vector<double> velocity_limits_;

  // Per-joint excitation amplitude (rad/s if velocity mode).
  std::vector<double> amplitudes_;

  // Excitation mode selection
  std::string mode_{"prbs"};   // "prbs", "sine", "chirp"
  double prbs_dwell_{0.02};    // seconds per PRBS update
  double sine_freq_{0.5};      // Hz
  double chirp_f_start_{0.1};  // Hz
  double chirp_f_end_{5.0};    // Hz
  double test_duration_{0.0};  // seconds, 0 => run forever

  // -----------------------------
  // Runtime
  // -----------------------------
  std::vector<double> qdot_cmd_;       // commanded joint velocities [rad/s]
  rclcpp::Time start_time_;
  rclcpp::Time next_switch_time_;      // for PRBS dwell timing

  std::vector<int> prbs_state_;        // ±1 per joint
  std::vector<uint32_t> lfsr_state_;   // one LFSR per joint for PRBS

  std::shared_ptr<
    realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> qdot_pub_;

  bool active_{false};

  // -----------------------------
  // Helpers
  // -----------------------------
  void clamp_velocity(std::vector<double> & qdot) const;
  void init_prbs();
  void update_prbs(const rclcpp::Time & now);
  void compute_sine(double t);
  void compute_chirp(double t);
};

}  // namespace ombot_controller
