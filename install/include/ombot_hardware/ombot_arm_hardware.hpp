// r6bot_hardware.hpp
#pragma once

#include <vector>
#include <string>
#include <algorithm>  // for std::clamp
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"  // or <dynamixel_sdk/dynamixel_sdk.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
using hardware_interface::return_type;


// ---- Conversion helpers ----
// ------------------- Position -------------------
// Pulses (raw encoder counts) <-> Radians
inline int32_t rad_to_pulses(double rad, int pulses_per_rev) {
  return static_cast<int32_t>(std::llround(rad * pulses_per_rev / (2.0 * M_PI)));
}

inline double pulses_to_rad(int32_t pulses, int pulses_per_rev) {
  return static_cast<double>(pulses) * (2.0 * M_PI) / static_cast<double>(pulses_per_rev);
}

// ------------------- Velocity -------------------
// Units are 0.01 rev/min
inline int32_t rad_s_to_vel_units(double rad_s) {
  double rev_per_min = rad_s * 60.0 / (2.0 * M_PI);
  return static_cast<int32_t>(std::llround(rev_per_min / 0.01));
}

inline double vel_units_to_rad_s(int32_t units) {
  double rev_per_min = static_cast<double>(units) * 0.01;
  return rev_per_min * (2.0 * M_PI) / 60.0;
}

// ------------------- Effort -------------------
// Torque (Nm) <-> Current (mA)
// k_t is motor torque constant in Nm/A
inline int16_t torque_to_current_mA(double tau_Nm, double k_t_Nm_per_A) {
  double amps = tau_Nm / k_t_Nm_per_A;
  return static_cast<int16_t>(std::llround(amps * 1000.0)); // A -> mA
}


inline double current_mA_to_torque(int16_t mA, double k_t_Nm_per_A) {
  double amps = static_cast<double>(mA) / 1000.0;
  return amps * k_t_Nm_per_A;
}



namespace ombot_hardware
{

enum class CommandMode { Position, Velocity, Effort };

// P-series (PH/PM) addresses (from e-Manual)
constexpr uint16_t ADDR_TORQUE_ENABLE     = 512;
constexpr uint16_t ADDR_OPERATING_MODE    = 11;

constexpr uint16_t ADDR_GOAL_CURRENT      = 550; // 2B, mA
constexpr uint16_t ADDR_GOAL_VELOCITY     = 552; // 4B, 0.01 rev/min
constexpr uint16_t ADDR_GOAL_POSITION     = 564; // 4B, pulses

constexpr uint16_t ADDR_PRESENT_CURRENT   = 574; // 2B, mA
constexpr uint16_t ADDR_PRESENT_VELOCITY  = 576; // 4B, 0.01 rev/min
constexpr uint16_t ADDR_PRESENT_POSITION  = 580; // 4B, pulses

constexpr uint16_t ADDR_HW_ERR = 518;   // P-series
constexpr uint8_t  LEN_HW_ERR  = 1;


bool is_ready_{false};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OMBotArmSystem : public hardware_interface::SystemInterface
{
public:
  // Called once when the hardware is initialized with the parsed <ros2_control> info
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override; 

  // Provide state handles (position, velocity, sensor data)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Provide command handles (position commands)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read the current state from hardware (or simulate it)
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Write the latest command to hardware (or clamp/update buffers)
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;



private:
  static uint8_t dxlOpModeFor(CommandMode m) {
    switch (m) {
      case CommandMode::Effort:   return 0; // current
      case CommandMode::Velocity: return 1; // velocity
      case CommandMode::Position: default:  return 3; // position
    }
  }
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)};

  dynamixel::PortHandler   *portHandler_{nullptr};
  dynamixel::PacketHandler *packetHandler_{nullptr};

  std::unique_ptr<dynamixel::GroupSyncRead>  sync_read_pos_;
  std::unique_ptr<dynamixel::GroupSyncRead>  sync_read_vel_;
  std::unique_ptr<dynamixel::GroupSyncRead>  sync_read_cur_;

  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_goal_pos_;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_goal_vel_;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_goal_cur_;

  std::unique_ptr<dynamixel::GroupSyncRead> gsr_hw_err_;
  std::unique_ptr<dynamixel::GroupSyncRead> sync_read_all_;


  std::string device_name_{"/dev/ttyUSB0"};
  int baudrate_{1000000};
  bool hw_connected_{false};

  // Names of the joints in the same order as the arrays
  std::vector<std::string> joint_names_;

  // Robot mapping
  std::vector<uint8_t> ids_;                // size = num joints
  std::vector<int>     pulses_per_rev_;     // per joint PPR
  std::vector<double>  kt_Nm_per_A_;        // per joint torque constants

  // Limits
  std::vector<double> pos_min_, pos_max_, vel_max_, eff_max_;

  // Command modes
  std::vector<CommandMode> command_mode_;
  bool any_position_mode_{false}, any_velocity_mode_{false}, any_effort_mode_{false};

  // Command buffers (if you use them)
  std::vector<double> joint_position_cmd_, joint_velocity_cmd_, joint_effort_cmd_;

  // State buffers (you already had position/velocity; add effort)
  std::vector<double> joint_position_, joint_velocity_, joint_effort_;

  // Force/torque sensor states (fx, fy, fz, tx, ty, tz)
  std::array<double, 6> ft_states_{};

  bool simulate_{true};
  bool serial_ok_{false};

};

}  // namespace ombot_hardware

