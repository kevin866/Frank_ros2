#pragma once
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ombot_hardware
{

// Minimal stub to abstract Roboteq I/O. Replace with real serial/CAN.
class RoboteqIface
{
public:
  bool open(const std::string &port, int baud) { (void)port; (void)baud; return true; }
  void close() {}
  bool write_speed(int channel, double speed_native) { (void)channel; (void)speed_native; return true; }
  bool read_encoder(int channel, int64_t &counts) { (void)channel; counts = 0; return true; }
  bool read_speed(int channel, double &speed_native) { (void)channel; speed_native = 0.0; return true; }
};

class OMBotBaseSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OMBotBaseSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // Params
  std::string ctrl1_port_, ctrl2_port_;
  int ctrl_baud_{115200};
  double wheel_radius_{0.0762}; // m
  double encoder_cpr_{2048.0};
  double gear_ratio_{1.0};
  double max_wheel_rad_s_{20.0};

  // Joint order: fl, fr, rl, rr
  std::vector<std::string> joint_names_;
  std::vector<double> pos_rad_;   // from encoders
  std::vector<double> vel_rad_s_; // from controller speed
  std::vector<double> cmd_rad_s_; // desired velocity

  // Map joint index -> (controller, channel)
  struct Chan { int ctrl; int ch; }; // ctrl: 1 or 2, ch: 1 or 2
  std::vector<Chan> map_;

  // Two controllers
  RoboteqIface ctrl1_, ctrl2_;

  // Helpers
  double nativeSpeedFromRadPerSec(double rad_s) const { return rad_s; } // TODO: scale
  double radPerSecFromNative(double native) const { return native; }    // TODO: scale

  double radFromCounts(int64_t counts) const
  {
    const double rev = (static_cast<double>(counts) / encoder_cpr_) / gear_ratio_;
    return rev * 2.0 * M_PI;
  }
};

} // namespace ombot_hardware
