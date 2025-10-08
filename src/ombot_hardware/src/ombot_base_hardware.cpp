#include "ombot_hardware/ombot_base_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace ombot_hardware
{

hardware_interface::CallbackReturn
OMBotBaseSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(
      rclcpp::get_logger("OMBotBaseSystem"),
      "start initialization");
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // Read params from <hardware> block in ros2_control xacro
  auto getp = [&](const std::string &name, const std::string &def="")->std::string{
    auto it = info_.hardware_parameters.find(name);
    return (it!=info_.hardware_parameters.end()) ? it->second : def;
  };

  ctrl1_port_ = getp("controller_1_port", "/dev/ttyACM0");
  ctrl2_port_ = getp("controller_2_port", "/dev/ttyACM1");
  if (auto s = getp("controller_baud", "115200"); !s.empty()) ctrl_baud_ = std::stoi(s);

  if (auto s = getp("wheel_radius", "0.0762"); !s.empty()) wheel_radius_ = std::stod(s);
  if (auto s = getp("encoder_cpr", "2048"); !s.empty()) encoder_cpr_ = std::stod(s);
  if (auto s = getp("gear_ratio", "1.0"); !s.empty()) gear_ratio_ = std::stod(s);
  if (auto s = getp("max_wheel_rad_s", "20.0"); !s.empty()) max_wheel_rad_s_ = std::stod(s);

  // Expect 4 joints
  joint_names_.clear();
  // for (const auto & ji : info_.joints) joint_names_.push_back(ji.name);
  joint_names_.clear();
  for (const auto & ji : info_.joints)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("OMBotBaseSystem"),
      "Parsed joint from URDF: '%s'", ji.name.c_str());
    joint_names_.push_back(ji.name);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("OMBotBaseSystem"),
    "Total joints parsed: %zu", joint_names_.size());
    
  if (joint_names_.size() != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotBaseSystem"), "Expected 4 wheel joints, got %zu", joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Mapping params: fl_map, fr_map, rl_map, rr_map with values like "ctrl1:A", "ctrl2:B"
  auto parse_map = [&](const std::string &v)->Chan{
    Chan c{1,1};
    if (v.find("ctrl2") != std::string::npos) c.ctrl = 2;
    if (v.find(":B") != std::string::npos || v.find(":b") != std::string::npos) c.ch = 2;
    return c;
  };
  std::vector<std::string> map_keys = {"fl_map","fr_map","rl_map","rr_map"};
  map_.resize(4);
  for (size_t i=0;i<4;i++) {
    std::string v = getp(map_keys[i], i<2 ? "ctrl1:A" : "ctrl2:A");
    map_[i] = parse_map(v);
  }

  pos_rad_.assign(4, 0.0);
  vel_rad_s_.assign(4, 0.0);
  cmd_rad_s_.assign(4, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OMBotBaseSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i=0;i<joint_names_.size();++i) {
    RCLCPP_INFO(rclcpp::get_logger("ombot_hw"), "joint='%s'", joint_names_[i].c_str());

    si.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_rad_[i]));
    si.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_rad_s_[i]));
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
OMBotBaseSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i=0;i<joint_names_.size();++i) {
    ci.emplace_back(hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_rad_s_[i]));
  }
  return ci;
}

hardware_interface::CallbackReturn
OMBotBaseSystem::on_configure(const rclcpp_lifecycle::State &)
{
  // Open ports
  if (!ctrl1_.open(ctrl1_port_, ctrl_baud_) || !ctrl2_.open(ctrl2_port_, ctrl_baud_)) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotBaseSystem"), "Failed to open Roboteq ports");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "Configured: %s and %s",
              ctrl1_port_.c_str(), ctrl2_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OMBotBaseSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  ctrl1_.close();
  ctrl2_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OMBotBaseSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Wake channels with zero speed (RPM) so the driver is “alive”
  for (auto &c : map_) {
    RoboteqIface &dev = (c.ctrl==1) ? ctrl1_ : ctrl2_;
    (void)dev.write_speed(c.ch, 0.0);
  }
  std::fill(cmd_rad_s_.begin(), cmd_rad_s_.end(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "Roboteq assumed preconfigured (Closed Loop Speed).");
  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn
OMBotBaseSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Stop motors
  for (size_t i=0;i<4;i++) cmd_rad_s_[i] = 0.0;
  write(rclcpp::Time{}, rclcpp::Duration{0,0});
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
OMBotBaseSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i=0;i<4;i++) {
    const Chan &c = map_[i];
    RoboteqIface &dev = (c.ctrl==1) ? ctrl1_ : ctrl2_;
    int64_t counts = 0;
    double motor_rpm = 0.0;
    (void)dev.read_encoder(c.ch, counts);   // motor shaft counts
    (void)dev.read_speed(c.ch, motor_rpm);  // motor RPM
    pos_rad_[i]   = wheelRad_from_encoderCounts(counts);
    vel_rad_s_[i] = wheelRadPerSec_from_motorRPM(motor_rpm);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OMBotBaseSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Send commands to Roboteq controllers, matching Python logic
  for (size_t i=0;i<2;i++) {
    std::string payload_front, payload_back;
    if (deadman_) {
      payload_front = "!G " + std::to_string(i+1) + " " + std::to_string(-static_cast<int>(cmd_rad_s_[i])) + "_";
      payload_back  = "!G " + std::to_string(i+1) + " " + std::to_string(-static_cast<int>(cmd_rad_s_[i+2])) + "_";
    } else {
      payload_front = "!MS " + std::to_string(i+1) + "_";
      payload_back  = "!MS " + std::to_string(i+1) + "_";
    }
    RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "FRONT: %s", payload_front.c_str());
    RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "BACK: %s", payload_back.c_str());
    ctrl1_.write_raw(payload_front);
    ctrl2_.write_raw(payload_back);
  }

  // Optional: one-line summary
  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("OMBotBaseSystem"), ros_clock_, 1000,
    "cmd wheel(rad/s): [%.2f %.2f %.2f %.2f]",
    cmd_rad_s_[0], cmd_rad_s_[1], cmd_rad_s_[2], cmd_rad_s_[3]
  );

  return hardware_interface::return_type::OK;
}


} // namespace ombot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ombot_hardware::OMBotBaseSystem, hardware_interface::SystemInterface)
