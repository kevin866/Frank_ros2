#include "ombot_hardware/ombot_base_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace ombot_hardware
{

hardware_interface::CallbackReturn
OMBotBaseSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "start initialization");
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // Helper to read params from <hardware> block
  auto getp = [&](const std::string &name, const std::string &def="")->std::string{
    auto it = info_.hardware_parameters.find(name);
    return (it!=info_.hardware_parameters.end()) ? it->second : def;
  };

  // --- ports & tuning ---
  ctrl1_port_ = getp("controller_1_port", "/dev/ttyACM0");
  ctrl2_port_ = getp("controller_2_port", "/dev/ttyACM1");
  if (auto s = getp("controller_baud", "115200"); !s.empty()) ctrl_baud_ = std::stoi(s);

  if (auto s = getp("wheel_radius", "0.0762"); !s.empty()) wheel_radius_ = std::stod(s);
  if (auto s = getp("encoder_cpr", "2048");   !s.empty()) encoder_cpr_ = std::stod(s);
  if (auto s = getp("gear_ratio", "1.0");     !s.empty()) gear_ratio_   = std::stod(s);
  if (auto s = getp("max_wheel_rad_s", "20.0"); !s.empty()) max_wheel_rad_s_ = std::stod(s);

  // --- which joints belong to THIS hardware ---
  // Either grab from params (optional) or use known names
  std::array<std::string,4> desired_wheels = {
    getp("front_left_joint",  "front_left_wheel_joint"),
    getp("front_right_joint", "front_right_wheel_joint"),
    getp("rear_left_joint",   "rear_left_wheel_joint"),
    getp("rear_right_joint",  "rear_right_wheel_joint")
  };
  std::unordered_set<std::string> wheel_set(desired_wheels.begin(), desired_wheels.end());

  joint_names_.clear();
  for (const auto & ji : info_.joints) {
    if (wheel_set.count(ji.name)) {
      RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "Claiming wheel joint: '%s'", ji.name.c_str());
      joint_names_.push_back(ji.name);
    } else {
      // Just FYI for debugging; these will belong to other hardware (e.g., arm)
      RCLCPP_DEBUG(rclcpp::get_logger("OMBotBaseSystem"), "Ignoring non-wheel joint: '%s'", ji.name.c_str());
    }
  }

  if (joint_names_.size() != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotBaseSystem"),
      "Expected 4 wheel joints, found %zu. Check names in URDF/YAML.", joint_names_.size());
    for (auto &n : joint_names_) RCLCPP_ERROR(rclcpp::get_logger("OMBotBaseSystem"), "  found: %s", n.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Mapping params: fl_map, fr_map, rl_map, rr_map with values like "ctrl1:A", "ctrl2:B"
  auto parse_map = [&](const std::string &v)->Chan{
    Chan c{1,1};
    if (v.find("ctrl2") != std::string::npos) c.ctrl = 2;
    if (v.find(":B")   != std::string::npos || v.find(":b") != std::string::npos) c.ch = 2;
    return c;
  };
  std::vector<std::string> map_keys = {"fl_map","fr_map","rl_map","rr_map"};
  map_.resize(4);
  for (size_t i=0;i<4;i++) {
    std::string v = getp(map_keys[i], i<2 ? "ctrl1:A" : "ctrl2:A");
    map_[i] = parse_map(v);
  }

  // Size buffers EXACTLY to the number of joints this hardware owns (4)
  pos_rad_.assign(joint_names_.size(), 0.0);
  vel_rad_s_.assign(joint_names_.size(), 0.0);
  cmd_rad_s_.assign(joint_names_.size(), 0.0);
  prev_pos_rad_.assign(joint_names_.size(), 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "Initialization OK; wheels=[%s,%s,%s,%s]",
              joint_names_[0].c_str(), joint_names_[1].c_str(),
              joint_names_[2].c_str(), joint_names_[3].c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OMBotBaseSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(joint_names_.size()*2);
  for (size_t i=0;i<joint_names_.size();++i) {
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_rad_[i]);
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_rad_s_[i]);
  }
  RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "Exported %zu state interfaces", si.size());
  return si;
}

std::vector<hardware_interface::CommandInterface>
OMBotBaseSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(joint_names_.size());
  for (size_t i=0;i<joint_names_.size();++i) {
    ci.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_rad_s_[i]);
  }
  RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "Exported %zu command interfaces (velocity)", ci.size());
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



// hardware_interface::return_type
// OMBotBaseSystem::read(const rclcpp::Time & /*stamp*/, const rclcpp::Duration &period)
// {
//   const double dt = period.seconds();
//   for (size_t i = 0; i < 4; ++i) {
//     const Chan &c = map_[i];
//     RoboteqIface &dev = (c.ctrl == 1) ? ctrl1_ : ctrl2_;

//     int64_t counts = 0;
//     double motor_rpm = std::numeric_limits<double>::quiet_NaN();

//     bool ok_enc  = dev.read_encoder(c.ch, counts);     // absolute motor shaft counts
//     bool ok_rpm  = dev.read_speed(c.ch, motor_rpm);    // motor RPM (shaft)
//     if (!ok_enc && !ok_rpm) {
//       RCLCPP_WARN_THROTTLE(
//         rclcpp::get_logger("OMBotBaseSystem"), ros_clock_, 2000,
//         "Wheel %zu: neither encoder nor speed available", i);
//       continue; // keep last states
//     }

//     // Velocity first (if available)
//     if (ok_rpm && std::isfinite(motor_rpm)) {
//       vel_rad_s_[i] = wheelRadPerSec_from_motorRPM(motor_rpm); // includes gear ratio
//     } // else keep last vel_rad_s_[i]

//     // Position from encoder if available; otherwise integrate velocity
//     if (ok_enc) {
//       pos_rad_[i] = wheelRad_from_encoderCounts(counts); // includes counts/rev & gear ratio
//     } else if (std::isfinite(vel_rad_s_[i]) && dt > 0.0) {
//       pos_rad_[i] += vel_rad_s_[i] * dt; // fallback so RViz animates
//     }

//     // Optional sanity clamp for absurd jumps (e.g., serial glitch)
//     const double max_jump = 50.0; // rad between cycles (tune)
//     if (std::abs(pos_rad_[i] - prev_pos_rad_[i]) > max_jump) {
//       RCLCPP_WARN_THROTTLE(
//         rclcpp::get_logger("OMBotBaseSystem"), ros_clock_, 2000,
//         "Wheel %zu: position jump %.2f rad -> clamped", i,
//         pos_rad_[i] - prev_pos_rad_[i]);
//       pos_rad_[i] = prev_pos_rad_[i];
//     }
//     prev_pos_rad_[i] = pos_rad_[i];
//   }
//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type
// OMBotBaseSystem::read(const rclcpp::Time & /*stamp*/, const rclcpp::Duration &period)
// {
//   const double dt = period.seconds();
  
//   // For now, just integrate velocity to keep the system running
//   // TODO: Implement actual hardware reads once serial communication is stable
//   for (size_t i = 0; i < 4; ++i) {
//     // Keep last velocity or zero
//     // Integrate position from velocity for visualization
//     if (std::isfinite(vel_rad_s_[i]) && dt > 0.0) {
//       pos_rad_[i] += vel_rad_s_[i] * dt;
//     }
//   }
  
//   return hardware_interface::return_type::OK;
// }


// hardware_interface::return_type
// OMBotBaseSystem::read(const rclcpp::Time & /*stamp*/, const rclcpp::Duration &period)
// {
//   const double dt = period.seconds();
//   // const double rpm_to_rad_s = 2.0 * M_PI / 60.0; // conversion constant

//   for (size_t i = 0; i < 4; ++i)
//   {
//     const Chan &c = map_[i];
//     RoboteqIface &dev = (c.ctrl == 1) ? ctrl1_ : ctrl2_;

//     // --- Query Roboteq for encoder and speed ---
//     std::string q_speed = "?S " + std::to_string(c.ch) + "_";
//     std::string q_pos   = "?C " + std::to_string(c.ch) + "_";

//     std::string resp_speed = dev.query_raw(q_speed);
//     std::string resp_pos   = dev.query_raw(q_pos);

//     bool ok_rpm = false;
//     bool ok_enc = false;
//     double motor_rpm = 0.0;
//     int64_t counts = 0;

//     // --- Parse responses ---
//     try {
//       if (!resp_speed.empty()) {
//         motor_rpm = std::stod(resp_speed);
//         ok_rpm = true;
//       }
//     } catch (...) {
//       RCLCPP_WARN_THROTTLE(
//         rclcpp::get_logger("OMBotBaseSystem"), ros_clock_, 2000,
//         "Wheel %zu: failed to parse speed response '%s'", i, resp_speed.c_str());
//     }

//     try {
//       if (!resp_pos.empty()) {
//         counts = static_cast<int64_t>(std::stoll(resp_pos));
//         ok_enc = true;
//       }
//     } catch (...) {
//       RCLCPP_WARN_THROTTLE(
//         rclcpp::get_logger("OMBotBaseSystem"), ros_clock_, 2000,
//         "Wheel %zu: failed to parse encoder response '%s'", i, resp_pos.c_str());
//     }

//     if (!ok_enc && !ok_rpm) {
//       RCLCPP_WARN_THROTTLE(
//         rclcpp::get_logger("OMBotBaseSystem"), ros_clock_, 2000,
//         "Wheel %zu: neither encoder nor speed available", i);
//       continue; // keep last states
//     }

//     // --- Convert RPM → rad/s ---
//     if (ok_rpm && std::isfinite(motor_rpm)) {
//       vel_rad_s_[i] = wheelRadPerSec_from_motorRPM(motor_rpm); // or simply: motor_rpm * rpm_to_rad_s * gear_ratio
//     }

//     // --- Position update ---
//     if (ok_enc) {
//       pos_rad_[i] = wheelRad_from_encoderCounts(counts); // convert counts → radians (includes CPR & gear ratio)
//     } else if (std::isfinite(vel_rad_s_[i]) && dt > 0.0) {
//       pos_rad_[i] += vel_rad_s_[i] * dt; // integrate when encoder not available
//     }

//     // --- Sanity check for sudden jumps ---
//     const double max_jump = 50.0;
//     if (std::abs(pos_rad_[i] - prev_pos_rad_[i]) > max_jump) {
//       RCLCPP_WARN_THROTTLE(
//         rclcpp::get_logger("OMBotBaseSystem"), ros_clock_, 2000,
//         "Wheel %zu: position jump %.2f rad -> clamped", i,
//         pos_rad_[i] - prev_pos_rad_[i]);
//       pos_rad_[i] = prev_pos_rad_[i];
//     }

//     prev_pos_rad_[i] = pos_rad_[i];
//   }

//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type
// OMBotBaseSystem::read(const rclcpp::Time&, const rclcpp::Duration& period)
// {
//   const double dt = period.seconds();
//   for (size_t i = 0; i < 4; ++i) {
//     const Chan &c = map_[i];
//     RoboteqIface &dev = (c.ctrl == 1) ? ctrl1_ : ctrl2_;

//     // Query ONLY speed (cheap). Position can be added later.
//     std::string s = dev.query_raw("?S " + std::to_string(c.ch) + "_");

//     bool ok_rpm = false;
//     double motor_rpm = 0.0;
//     if (!s.empty()) {
//       try { motor_rpm = std::stod(s); ok_rpm = true; }
//       catch (...) { /* keep last */ }
//     }

//     if (ok_rpm) {
//       vel_rad_s_[i] = wheelRadPerSec_from_motorRPM(motor_rpm);
//     }
//     // Integrate to keep RViz animated if needed
//     if (std::isfinite(vel_rad_s_[i]) && dt > 0.0) {
//       pos_rad_[i] += vel_rad_s_[i] * dt;
//     }
//   }
//   return hardware_interface::return_type::OK;
// }

hardware_interface::return_type
OMBotBaseSystem::read(const rclcpp::Time &, const rclcpp::Duration &period)
{
  const double dt = period.seconds();

  for (size_t i = 0; i < 4; ++i) {
    const Chan &c = map_[i];
    RoboteqIface &dev = (c.ctrl == 1) ? ctrl1_ : ctrl2_;

    // --- Always get speed ---
    std::string s = dev.query_raw("?S " + std::to_string(c.ch) + "_");

    bool ok_rpm = false;
    double motor_rpm = 0.0;
    if (!s.empty()) {
      try { motor_rpm = std::stod(s); ok_rpm = true; } catch (...) {}
    }
    if (ok_rpm) {
      vel_rad_s_[i] = wheelRadPerSec_from_motorRPM(motor_rpm);  // includes gear ratio inside helper
    }

    // --- Staggered true position refresh ---
    // Each wheel refreshes on a different cycle so only 1 wheel (typically) does ?C this tick.
    const bool do_pos_refresh = (pos_update_stride_ > 0) &&
                                ((cycle_counter_ + i) % pos_update_stride_ == 0);

    if (do_pos_refresh) {
      std::string p = dev.query_raw("?C " + std::to_string(c.ch) + "_");
      if (!p.empty()) {
        try {
          const int64_t counts = std::stoll(p);
          pos_rad_[i] = wheelRad_from_encoderCounts(counts);     // counts → radians (CPR & gear ratio handled inside)
        } catch (...) {
          // fallback if parse fails
          if (std::isfinite(vel_rad_s_[i]) && dt > 0.0) pos_rad_[i] += vel_rad_s_[i] * dt;
        }
      } else {
        // no reply; integrate
        if (std::isfinite(vel_rad_s_[i]) && dt > 0.0) pos_rad_[i] += vel_rad_s_[i] * dt;
      }
    } else {
      // No refresh this cycle → integrate velocity
      if (std::isfinite(vel_rad_s_[i]) && dt > 0.0) pos_rad_[i] += vel_rad_s_[i] * dt;
    }

    // --- Optional sanity clamp for serial glitches ---
    const double max_jump = 100.0;  // rad
    if (std::abs(pos_rad_[i] - prev_pos_rad_[i]) > max_jump) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("OMBotBaseSystem"), ros_clock_, 2000,
        "Wheel %zu: position jump %.2f rad -> clamped", i, pos_rad_[i] - prev_pos_rad_[i]);
      pos_rad_[i] = prev_pos_rad_[i];
    }
    prev_pos_rad_[i] = pos_rad_[i];
  }

  ++cycle_counter_;
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
    // RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "FRONT: %s", payload_front.c_str());
    // RCLCPP_INFO(rclcpp::get_logger("OMBotBaseSystem"), "BACK: %s", payload_back.c_str());
    ctrl1_.write_raw(payload_front);
    ctrl2_.write_raw(payload_back);
  }


  return hardware_interface::return_type::OK;
}


} // namespace ombot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ombot_hardware::OMBotBaseSystem, hardware_interface::SystemInterface)
