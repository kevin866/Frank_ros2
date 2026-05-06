#pragma once

#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace ombot_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ── XL330 / X-series register map ──────────────────────────────────────────
// (Different from P-series used by the arm)
constexpr uint16_t GRIPPER_ADDR_OPERATING_MODE   = 11;   // 1B
constexpr uint16_t GRIPPER_ADDR_CURRENT_LIMIT     = 38;   // 2B
constexpr uint16_t GRIPPER_ADDR_PWM_LIMIT         = 36;   // 2B
constexpr uint16_t GRIPPER_ADDR_TORQUE_ENABLE     = 64;   // 1B
constexpr uint16_t GRIPPER_ADDR_GOAL_PWM          = 100;  // 2B  (mode 16)
constexpr uint16_t GRIPPER_ADDR_GOAL_CURRENT      = 102;  // 2B  (mode 5, not supported on XL330)
constexpr uint16_t GRIPPER_ADDR_GOAL_POSITION     = 116;  // 4B  (mode 3)
constexpr uint16_t GRIPPER_ADDR_PRESENT_PWM       = 124;  // 2B
constexpr uint16_t GRIPPER_ADDR_PRESENT_CURRENT   = 126;  // 2B  signed, unit = ~1mA
constexpr uint16_t GRIPPER_ADDR_PRESENT_VELOCITY  = 128;  // 4B  unit = 0.229 rpm
constexpr uint16_t GRIPPER_ADDR_PRESENT_POSITION  = 132;  // 4B  unit = 1 pulse (4096 per rev)

// XL330 constants
constexpr int     GRIPPER_PULSES_PER_REV = 4096;
constexpr double  GRIPPER_VEL_UNIT_RPM   = 0.229;   // rpm per unit
constexpr int16_t GRIPPER_PWM_MAX        = 885;      // ±885 = ±100% PWM

// Operating modes
constexpr uint8_t GRIPPER_OP_MODE_POSITION = 3;
constexpr uint8_t GRIPPER_OP_MODE_PWM      = 16;

// ── Conversion helpers (XL330-specific) ────────────────────────────────────

inline int32_t gripper_rad_to_pulses(double rad)
{
  // XL330 zero is at 2048 (centre of 0-4095 range)
  return static_cast<int32_t>(std::llround(rad * GRIPPER_PULSES_PER_REV / (2.0 * M_PI))) + 2048;
}

inline double gripper_pulses_to_rad(int32_t pulses)
{
  return static_cast<double>(pulses - 2048) * (2.0 * M_PI) / GRIPPER_PULSES_PER_REV;
}

inline double gripper_vel_units_to_rad_s(int32_t units)
{
  // units * 0.229 rpm -> rad/s
  return static_cast<double>(units) * GRIPPER_VEL_UNIT_RPM * (2.0 * M_PI) / 60.0;
}

// ── Hardware interface class ────────────────────────────────────────────────

class OMBotGripperHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── helpers ──
  bool set_operating_mode(uint8_t mode);
  bool set_torque(bool enable);

  // ── SDK ──
  dynamixel::PortHandler   * port_{nullptr};
  dynamixel::PacketHandler * ph_{nullptr};

  // ── params (from URDF <ros2_control>) ──
  std::string device_{"/dev/gripper_u2d2"};
  int         baudrate_{1000000};
  uint8_t     dxl_id_{12};
  int32_t     open_pulse_{1754};   // raw pulse for fully open
  int32_t     close_pulse_{594};   // raw pulse for fully closed
  int16_t     current_limit_{300}; // mA soft cap in position mode
  bool        simulate_{false};

  // ── mode ──
  enum class GripperMode { Position, PWM };
  GripperMode command_mode_{GripperMode::Position};
  GripperMode requested_mode_{GripperMode::Position};

  // ── state buffers (exported as StateInterfaces) ──
  double joint_position_{0.0};   // rad
  double joint_velocity_{0.0};   // rad/s
  double joint_effort_{0.0};     // mA (raw current, no k_t for XL330 micro)

  // ── command buffers (exported as CommandInterfaces) ──
  double position_cmd_{0.0};     // rad  (used in Position mode)
  double pwm_cmd_{0.0};          // -1.0 … +1.0 normalised (used in PWM mode)

  bool hw_connected_{false};
  bool is_ready_{false};

  rclcpp::Clock::SharedPtr log_clock_{std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
};

}  // namespace ombot_hardware