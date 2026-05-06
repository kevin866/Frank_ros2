#include "ombot_hardware/ombot_gripper_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace ombot_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

// ── on_init ─────────────────────────────────────────────────────────────────
CallbackReturn OMBotGripperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // ── Read parameters from URDF <ros2_control> block ──
  auto param = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return (it != info_.hardware_parameters.end()) ? it->second : def;
  };

  device_       = param("port",          "/dev/gripper_u2d2");
  baudrate_     = std::stoi(param("baudrate",     "1000000"));
  dxl_id_       = static_cast<uint8_t>(std::stoi(param("id", "12")));
  open_pulse_   = std::stoi(param("open_pulse",   "1754"));
  close_pulse_  = std::stoi(param("close_pulse",  "594"));
  current_limit_= static_cast<int16_t>(std::stoi(param("current_limit_mA", "300")));
  simulate_     = (param("simulate", "false") == "true" || param("simulate", "false") == "1");

  // ── Validate: expect exactly one joint ──
  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotGripperHardware"),
                 "Expected 1 joint, got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // ── Seed command to open position ──
  position_cmd_ = gripper_pulses_to_rad(open_pulse_);
  pwm_cmd_      = 0.0;

  // ── Prepare SDK handles (no I/O yet) ──
  port_ = dynamixel::PortHandler::getPortHandler(device_.c_str());
  ph_   = dynamixel::PacketHandler::getPacketHandler(2.0);

  RCLCPP_INFO(rclcpp::get_logger("OMBotGripperHardware"),
              "on_init OK  port=%s  id=%u  simulate=%d",
              device_.c_str(), dxl_id_, (int)simulate_);

  return CallbackReturn::SUCCESS;
}

// ── on_activate ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn OMBotGripperHardware::on_activate(const rclcpp_lifecycle::State &)
{
  // // Must call base class first
  // if (SystemInterface::on_activate(previous_state) != CallbackReturn::SUCCESS)
  //   return CallbackReturn::ERROR;

  is_ready_ = false;

  if (simulate_) {
    hw_connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("OMBotGripperHardware"), "Simulation mode — no hardware.");
    is_ready_ = true;
    return CallbackReturn::SUCCESS;
  }
  // Open port (close first in case of re-activation)
  port_->closePort();

  // Open port
  // if (!port_->openPort() || !port_->setBaudRate(baudrate_)) {
  //   RCLCPP_ERROR(rclcpp::get_logger("OMBotGripperHardware"),
  //                "Failed to open %s @ %d", device_.c_str(), baudrate_);
  //   return CallbackReturn::FAILURE;
  // }
  if (!port_->openPort() || !port_->setBaudRate(baudrate_)) {
    RCLCPP_WARN(rclcpp::get_logger("OMBotGripperHardware"),
                "Failed to open %s @ %d — falling back to simulate",
                device_.c_str(), baudrate_);
    hw_connected_ = false;
    is_ready_ = true;
    return CallbackReturn::SUCCESS;
  }

  // Reboot to clear latched hardware errors
  uint8_t dxl_error = 0;
  ph_->reboot(port_, dxl_id_, &dxl_error);
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  // Clear hardware error register explicitly
  ph_->write1ByteTxRx(port_, dxl_id_, 63, 0, &dxl_error);
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  // Ping to confirm servo is alive
  uint16_t model = 0;
  int rc = ph_->ping(port_, dxl_id_, &model, &dxl_error);
  if (rc != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotGripperHardware"),
                 "Ping failed id=%u rc=%d (%s) — falling back to simulate",
                 dxl_id_, rc, ph_->getTxRxResult(rc));
    port_->closePort();
    hw_connected_ = false;
    is_ready_ = true;
    return CallbackReturn::SUCCESS;
  }
  RCLCPP_INFO(rclcpp::get_logger("OMBotGripperHardware"),
              "Gripper servo found: id=%u model=%u", dxl_id_, model);

  hw_connected_ = true;

  // Set current limit
  ph_->write2ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_CURRENT_LIMIT, current_limit_, &dxl_error);

  // Disable torque before changing mode (required by Dynamixel)
  ph_->write1ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_TORQUE_ENABLE, 0, &dxl_error);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Set operating mode
  ph_->write1ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_OPERATING_MODE, GRIPPER_OP_MODE_POSITION, &dxl_error);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Enable torque
  ph_->write1ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_TORQUE_ENABLE, 1, &dxl_error);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Verify torque is enabled
  uint8_t torque_val = 0;
  ph_->read1ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_TORQUE_ENABLE, &torque_val, &dxl_error);
  RCLCPP_INFO(rclcpp::get_logger("OMBotGripperHardware"),
              "Torque enable verify: %u", torque_val);

  // Seed state from actual hardware position
  uint32_t raw = 0;
  ph_->read4ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_PRESENT_POSITION, &raw, &dxl_error);
  joint_position_ = gripper_pulses_to_rad(static_cast<int32_t>(raw));
  position_cmd_   = joint_position_;   // don't jump on first write

  RCLCPP_INFO(rclcpp::get_logger("OMBotGripperHardware"),
              "Activated. Initial position: %.4f rad (%u pulses)", joint_position_, raw);

  is_ready_ = true;
  return CallbackReturn::SUCCESS;
}

// ── on_deactivate ───────────────────────────────────────────────────────────
hardware_interface::CallbackReturn OMBotGripperHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  is_ready_ = false;
  if (hw_connected_) {
    set_torque(false);
    port_->closePort();
    hw_connected_ = false;
  }
  return CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ─────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
OMBotGripperHardware::export_state_interfaces()
{
  const std::string & jname = info_.joints[0].name;
  std::vector<hardware_interface::StateInterface> out;
  out.emplace_back(jname, hardware_interface::HW_IF_POSITION, &joint_position_);
  out.emplace_back(jname, hardware_interface::HW_IF_VELOCITY, &joint_velocity_);
  out.emplace_back(jname, hardware_interface::HW_IF_EFFORT,   &joint_effort_);
  return out;
}

// ── export_command_interfaces ───────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
OMBotGripperHardware::export_command_interfaces()
{
  const std::string & jname = info_.joints[0].name;
  std::vector<hardware_interface::CommandInterface> out;
  out.emplace_back(jname, hardware_interface::HW_IF_POSITION, &position_cmd_);
  out.emplace_back(jname, hardware_interface::HW_IF_EFFORT,   &pwm_cmd_);  // normalised PWM
  return out;
}

// ── prepare_command_mode_switch ─────────────────────────────────────────────
hardware_interface::return_type
OMBotGripperHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  requested_mode_ = command_mode_;  // default: keep current

  for (const auto & key : start_interfaces) {
    const auto slash = key.find('/');
    if (slash == std::string::npos) continue;
    const std::string ifname = key.substr(slash + 1);

    if (ifname == hardware_interface::HW_IF_POSITION)
      requested_mode_ = GripperMode::Position;
    else if (ifname == hardware_interface::HW_IF_EFFORT)
      requested_mode_ = GripperMode::PWM;
  }

  return hardware_interface::return_type::OK;
}

// ── perform_command_mode_switch ──────────────────────────────────────────────
hardware_interface::return_type
OMBotGripperHardware::perform_command_mode_switch(
  const std::vector<std::string> & /*start*/,
  const std::vector<std::string> & /*stop*/)
{
  if (requested_mode_ == command_mode_) return hardware_interface::return_type::OK;

  const uint8_t dxl_mode = (requested_mode_ == GripperMode::PWM)
                           ? GRIPPER_OP_MODE_PWM
                           : GRIPPER_OP_MODE_POSITION;

  if (hw_connected_) {
    set_torque(false);
    set_operating_mode(dxl_mode);
    set_torque(true);
  }

  command_mode_ = requested_mode_;

  RCLCPP_INFO(rclcpp::get_logger("OMBotGripperHardware"),
              "Mode switched to %s",
              command_mode_ == GripperMode::PWM ? "PWM" : "Position");

  return hardware_interface::return_type::OK;
}

// ── read ─────────────────────────────────────────────────────────────────────
hardware_interface::return_type
OMBotGripperHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!is_ready_) return return_type::OK;

  if (!hw_connected_) {
    // Simulate smooth motion toward command
    const double dt  = period.seconds();
    const double err = position_cmd_ - joint_position_;
    const double v   = std::clamp(err * 10.0, -2.0, 2.0);
    joint_velocity_  = v;
    joint_position_ += v * dt;
    joint_effort_    = 0.0;
    return return_type::OK;
  }

  uint8_t dxl_error = 0;

  // Present position
  uint32_t raw_pos = 0;
  ph_->read4ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_PRESENT_POSITION, &raw_pos, &dxl_error);
  joint_position_ = gripper_pulses_to_rad(static_cast<int32_t>(raw_pos));

  // Present velocity
  uint32_t raw_vel = 0;
  ph_->read4ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_PRESENT_VELOCITY, &raw_vel, &dxl_error);
  joint_velocity_ = gripper_vel_units_to_rad_s(static_cast<int32_t>(raw_vel));

  // Present current (effort) — signed 16-bit, units ≈ 1 mA
  uint16_t raw_cur = 0;
  ph_->read2ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_PRESENT_CURRENT, &raw_cur, &dxl_error);
  joint_effort_ = static_cast<double>(static_cast<int16_t>(raw_cur));  // mA

  return return_type::OK;
}

// ── write ─────────────────────────────────────────────────────────────────────
hardware_interface::return_type
OMBotGripperHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!is_ready_ || !hw_connected_) return return_type::OK;

  uint8_t dxl_error = 0;
  int rc = COMM_SUCCESS;

  if (command_mode_ == GripperMode::Position) {
    // Clamp to safe pulse range
    const double rad_min = gripper_pulses_to_rad(
      std::min(open_pulse_, close_pulse_));
    const double rad_max = gripper_pulses_to_rad(
      std::max(open_pulse_, close_pulse_));
    const double clamped = std::clamp(position_cmd_, rad_min, rad_max);

    const int32_t pulses = gripper_rad_to_pulses(clamped);
    rc = ph_->write4ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_GOAL_POSITION,
                              static_cast<uint32_t>(pulses), &dxl_error);
  } else {
    // PWM mode: pwm_cmd_ is normalised -1.0 … +1.0
    const double  clamped = std::clamp(pwm_cmd_, -1.0, 1.0);
    const int16_t pwm_raw = static_cast<int16_t>(
      std::llround(clamped * GRIPPER_PWM_MAX));
    rc = ph_->write2ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_GOAL_PWM,
                              static_cast<uint16_t>(pwm_raw), &dxl_error);
  }

  // if (rc != COMM_SUCCESS || dxl_error != 0) {
    // Fixed — alert bit alone does not block writes
  if (rc != COMM_SUCCESS || (dxl_error & 0x7F) != 0)
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OMBotGripperHardware"),
                         *log_clock_, 2000,
                         "Write failed rc=%d err=%u (%s)",
                         rc, dxl_error, ph_->getTxRxResult(rc));
    return return_type::ERROR;
  }

  return return_type::OK;
}

// ── private helpers ──────────────────────────────────────────────────────────
bool OMBotGripperHardware::set_operating_mode(uint8_t mode)
{
  uint8_t err = 0;
  int rc = ph_->write1ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_OPERATING_MODE, mode, &err);
  if (rc != COMM_SUCCESS || err != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotGripperHardware"),
                 "set_operating_mode(%u) failed rc=%d err=%u", mode, rc, err);
    return false;
  }
  return true;
}

bool OMBotGripperHardware::set_torque(bool enable)
{
  uint8_t err = 0;
  int rc = ph_->write1ByteTxRx(port_, dxl_id_, GRIPPER_ADDR_TORQUE_ENABLE,
                                enable ? 1 : 0, &err);
  if (rc != COMM_SUCCESS || err != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotGripperHardware"),
                 "set_torque(%d) failed rc=%d err=%u", (int)enable, rc, err);
    return false;
  }
  return true;
}

}  // namespace ombot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ombot_hardware::OMBotGripperHardware,
  hardware_interface::SystemInterface)