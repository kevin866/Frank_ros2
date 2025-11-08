#include "ombot_hardware/ombot_arm_hardware.hpp"
#include <string>
#include <vector>
#include <chrono>


namespace ombot_hardware
{

using hardware_interface::HardwareInfo;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
static ombot_hardware::CommandMode parse_mode(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  if (s=="vel" || s=="velocity") return ombot_hardware::CommandMode::Velocity;
  if (s=="eff" || s=="effort"   || s=="current" || s=="torque")
    return ombot_hardware::CommandMode::Effort;
  return ombot_hardware::CommandMode::Position; // default
}


CallbackReturn OMBotArmSystem::on_init(const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;


  // Read params
  if (auto it = info_.hardware_parameters.find("simulate"); it != info_.hardware_parameters.end()) {
    simulate_ = (it->second == "true" || it->second == "1");
  }


  // ---- Params (just read; don't open hardware here) ----
  if (auto it = info_.hardware_parameters.find("port"); it != info_.hardware_parameters.end())
    device_name_ = it->second;
  if (auto it = info_.hardware_parameters.find("baudrate"); it != info_.hardware_parameters.end())
    baudrate_ = std::stoi(it->second);

  // ---- Software state (sizes, names, limits, modes) ----
  const size_t N = info_.joints.size();

  joint_names_.clear(); joint_names_.reserve(N);
  for (const auto& j : info_.joints) joint_names_.push_back(j.name);

  joint_position_.assign(N, 0.0);
  joint_velocity_.assign(N, 0.0);
  joint_effort_.assign(N, 0.0);          // if you publish torque
  joint_position_cmd_.assign(N, 0.0);
  joint_velocity_cmd_.assign(N, 0.0);
  joint_effort_cmd_.assign(N, 0.0);

  pos_min_.assign(N, -1e9); pos_max_.assign(N, 1e9);
  vel_max_.assign(N, 1e9);  eff_max_.assign(N, 1e9);

  // IDs / PPR / k_t should come from parameters or a YAML you parse here
  ids_ = {1,2,3,4,5,6};
  pulses_per_rev_ = {1003846,1003846,1003846,1003846,607500,607500};
  // kt_Nm_per_A_    = {...};
  kt_Nm_per_A_.assign(N, 1.0);           
  kt_Nm_per_A_[0] = 6.2;  // PH54-200-S500-R
  kt_Nm_per_A_[1] = 6.2;  // another PH54
  kt_Nm_per_A_[2] = 5.2;  // PH54-100-S500-R
  kt_Nm_per_A_[3] = 5.2;  // PH54-100-S500-R
  kt_Nm_per_A_[4] = 4.5;  // PH42-020-S300-R
  kt_Nm_per_A_[5] = 4.5;  // PH42-020-S300-R


  // Modes
  command_mode_.assign(N, CommandMode::Effort);
  if (auto it = info_.hardware_parameters.find("default_mode"); it != info_.hardware_parameters.end()) {
    std::fill(command_mode_.begin(), command_mode_.end(), parse_mode(it->second));
  }
  if (auto it = info_.hardware_parameters.find("modes"); it != info_.hardware_parameters.end()) {
    std::stringstream ss(it->second); std::string tok; size_t i=0;
    while (std::getline(ss, tok, ',') && i<N) command_mode_[i++] = parse_mode(tok);
  }
  any_position_mode_ = std::any_of(command_mode_.begin(), command_mode_.end(),
                                   [](auto m){return m==CommandMode::Position;});
  any_velocity_mode_ = std::any_of(command_mode_.begin(), command_mode_.end(),
                                   [](auto m){return m==CommandMode::Velocity;});
  any_effort_mode_   = std::any_of(command_mode_.begin(), command_mode_.end(),
                                   [](auto m){return m==CommandMode::Effort;});

  // Prepare SDK pointers only (no I/O yet)
  portHandler_   = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  log_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OMBotArmSystem::on_activate(const rclcpp_lifecycle::State&) {
  is_ready_ = false;

  if (simulate_) {
    RCLCPP_INFO(rclcpp::get_logger("OMBotArmSystem"), "Running in simulation mode.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Open and configure the serial bus
  if (!portHandler_->openPort() || !portHandler_->setBaudRate(baudrate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotArmSystem"),
                 "Failed to open %s @ %d", device_name_.c_str(), baudrate_);
    hw_connected_ = false;
    return CallbackReturn::SUCCESS;  // fall back to fake mode in read()
  }
  hw_connected_ = true;

  // 2) Ping and filter IDs
  std::vector<uint8_t> alive;
  for (auto id : ids_) {
    uint16_t model = 0; uint8_t err = 0;
    const int rc = packetHandler_->ping(portHandler_, id, &model, &err);
    if (rc == COMM_SUCCESS && err == 0) {
      alive.push_back(id);
      RCLCPP_INFO(rclcpp::get_logger("OMBotArmSystem"), "DXL ID %u OK, model=%u", id, model);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("OMBotArmSystem"), "DXL ID %u not responding (rc=%d err=%u)", id, rc, err);
      RCLCPP_WARN(rclcpp::get_logger("OMBotArmSystem"),
        "ID %u not responding (rc=%d:%s err=%u)",
        id, rc, packetHandler_->getTxRxResult(rc), err);

    }
  }
  
  ids_ = alive;
  if (ids_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotArmSystem"), "No responding Dynamixels; abort activate.");
    return hardware_interface::CallbackReturn::FAILURE;  // or ::ERROR if you hit an exceptional state
  }

  // Set operating mode & torque enable per joint
  for (size_t i=0; i<ids_.size(); ++i) {
    uint8_t dxl_error = 0;
    const uint8_t mode = dxlOpModeFor(command_mode_[i]); // 0=current,1=vel,3=pos
    packetHandler_->write1ByteTxRx(portHandler_, ids_[i], ADDR_OPERATING_MODE, mode, &dxl_error);
    packetHandler_->write1ByteTxRx(portHandler_, ids_[i], ADDR_TORQUE_ENABLE, 1,    &dxl_error);
  }

  // Create writers based on modes
  if (any_position_mode_)
    sync_write_goal_pos_ = std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GOAL_POSITION, 4);
  if (any_velocity_mode_)
    sync_write_goal_vel_ = std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GOAL_VELOCITY, 4);
  if (any_effort_mode_)
    sync_write_goal_cur_ = std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GOAL_CURRENT, 2);
  // (Optional but recommended) Seed initial state to avoid a big first-tick jump:
  // Do a one-shot read of PRESENT_POSITION/VELOCITY/CURRENT and fill joint_*_.
  // Or initialize joint_position_cmd_ = joint_position_ so controllers start from current pose.
  sync_read_cur_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, ADDR_PRESENT_CURRENT, 2);
  sync_read_vel_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, ADDR_PRESENT_VELOCITY, 4);
  sync_read_pos_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, ADDR_PRESENT_POSITION, 4);
  gsr_hw_err_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, ADDR_HW_ERR, LEN_HW_ERR);
  sync_read_all_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, 574, 10);

  if (!sync_read_pos_ || !sync_read_vel_ || !sync_read_cur_) {
    RCLCPP_ERROR(rclcpp::get_logger("OMBotArmSystem"), "Failed to allocate GroupSyncRead objects");
    return hardware_interface::CallbackReturn::FAILURE;  // or ::ERROR if you hit an exceptional state
  }

  for (auto id : ids_) {
    if (!sync_read_pos_->addParam(id)) RCLCPP_ERROR(rclcpp::get_logger("OMBotArmSystem"), "addParam pos id=%u failed", id);
    if (!sync_read_vel_->addParam(id)) RCLCPP_ERROR(rclcpp::get_logger("OMBotArmSystem"), "addParam vel id=%u failed", id);
    if (!sync_read_cur_->addParam(id)) RCLCPP_ERROR(rclcpp::get_logger("OMBotArmSystem"), "addParam cur id=%u failed", id);
    gsr_hw_err_->addParam(id);
    sync_read_all_->addParam(id);
  }


  RCLCPP_INFO(rclcpp::get_logger("OMBotArmSystem"), "Activate OK: %zu servos alive", ids_.size());
  is_ready_ = true;

  std::fill(joint_position_cmd_.begin(), joint_position_cmd_.end(), 0.0);
  std::fill(joint_effort_cmd_.begin(),   joint_effort_cmd_.end(),   0.0);


  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OMBotArmSystem::on_deactivate(const rclcpp_lifecycle::State&) {
  if (hw_connected_) {
    for (auto id : ids_) {
      uint8_t err=0;
      packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0, &err);
    }
    portHandler_->closePort();
  }
  hw_connected_ = false;
  sync_write_goal_pos_.reset();
  sync_write_goal_vel_.reset();
  sync_write_goal_cur_.reset();
  return CallbackReturn::SUCCESS;
}



std::vector<StateInterface> OMBotArmSystem::export_state_interfaces() {
  std::vector<StateInterface> out;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_effort_cmd_[i]);
  }
  // sensor states
  out.emplace_back("tcp_fts_sensor","force.x",&ft_states_[0]);
  out.emplace_back("tcp_fts_sensor","force.y",&ft_states_[1]);
  out.emplace_back("tcp_fts_sensor","force.z",&ft_states_[2]);
  out.emplace_back("tcp_fts_sensor","torque.x",&ft_states_[3]);
  out.emplace_back("tcp_fts_sensor","torque.y",&ft_states_[4]);
  out.emplace_back("tcp_fts_sensor","torque.z",&ft_states_[5]);
  return out;
}

std::vector<CommandInterface> OMBotArmSystem::export_command_interfaces() {
  std::vector<CommandInterface> out;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_cmd_[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_effort_cmd_[i]);

  }
  return out;
}

return_type OMBotArmSystem::read(const rclcpp::Time&, const rclcpp::Duration& period) {

  // auto t_start = std::chrono::steady_clock::now();
  
  const double dt = period.seconds();

  if (!hw_connected_) {
    // ---- FAKE MODE (your original smooth integrator) ----
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const double err = joint_position_cmd_[i] - joint_position_[i];
      const double v   = std::clamp(err * 10.0, -3.0, 3.0);
      joint_velocity_[i] = v;
      joint_position_[i] += v * dt;
      // optional: decay effort reading
      joint_effort_[i] *= 0.95;
    }
    return return_type::OK;
  }

  // ---- HARDWARE MODE ----
  // Use one bulk/sync read for all:
  // We expect PRESENT_POSITION (4B), PRESENT_VELOCITY (4B), PRESENT_CURRENT (2B)
  // If you created 3 separate GroupSyncRead, you’ll call each and parse.
  // int dxl_comm_result{};
  
  if (!is_ready_) return hardware_interface::return_type::OK;

  
  bool read_ok = true;


// // 2) Parse per ID and convert → SI


  const int rc = sync_read_all_->txRxPacket();   // <-- ONCE
  if (rc != COMM_SUCCESS) { /* warn once/throttle */ }

  for (size_t i = 0; i < ids_.size(); ++i) {
    const uint8_t id = ids_[i];
    if (sync_read_all_->isAvailable(id, 574, 10)) {
      const int16_t raw_cur = (int16_t)sync_read_all_->getData(id, 574, 2);
      const int32_t raw_vel = (int32_t)sync_read_all_->getData(id, 576, 4);
      const int32_t raw_pos = (int32_t)sync_read_all_->getData(id, 580, 4);
      joint_position_[i] = pulses_to_rad(raw_pos, pulses_per_rev_[i]);
      joint_velocity_[i] = vel_units_to_rad_s(raw_vel);
      joint_effort_[i]   = current_mA_to_torque(raw_cur, kt_Nm_per_A_[i]);
    }
  }
  
  // for (size_t i = 0; i < ids_.size(); ++i) {
  //   const uint8_t id = ids_[i];

  //   int32_t raw_pos = 0;
  //   int32_t raw_vel = 0;
  //   int16_t raw_cur = 0;

  //   int rc = sync_read_all_->txRxPacket();
  //   if (rc != COMM_SUCCESS) {
  //     RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OMBotArmSystem"), *clock_, 2000, "sync read rc=%d", rc);
  //   }
  //   // per-ID:
  //   if (sync_read_all_->isAvailable(id, 574, 10)) {
  //     int16_t raw_cur = static_cast<int16_t>(sync_read_all_->getData(id, 574, 2));
  //     int32_t raw_vel    = static_cast<int32_t>(sync_read_all_->getData(id, 576, 4));
  //     int32_t raw_pos    = static_cast<int32_t>(sync_read_all_->getData(id, 580, 4));
  //     joint_position_[i] = pulses_to_rad(raw_pos, pulses_per_rev_[i]);
  //     joint_velocity_[i] = vel_units_to_rad_s(raw_vel);
  //     joint_effort_[i] = current_mA_to_torque(raw_cur, kt_Nm_per_A_[i]);
  //   }

  //   // Defaults in case some field is missing this cycle

  //   bool have_pos = false, have_vel = false, have_cur = false;

  //   // ---- Convert → SI using your helpers ----
  //   // Position: ticks → rad
  //   if (have_pos) {
  //     const double rad = pulses_to_rad(raw_pos, pulses_per_rev_[i]);
  //     joint_position_[i] = std::isfinite(rad) ? rad : joint_position_[i]; // keep last good on NaN/Inf
  //   }

  //   // Velocity: device units → rad/s
  //   if (have_vel) {
  //     const double rads = vel_units_to_rad_s(raw_vel);
  //     joint_velocity_[i] = std::isfinite(rads) ? rads : joint_velocity_[i];
  //   }

  //   // Effort: current (mA or device units) → Nm using per-joint k_t
  //   if (have_cur) {
  //     const double tau = current_mA_to_torque(raw_cur, kt_Nm_per_A_[i]);
  //     joint_effort_[i] = std::isfinite(tau) ? tau : joint_effort_[i];
  //   }

  // }

  if (!read_ok) {
    // Preserve last values; optionally set hw_connected_=false to failover to fake mode.
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OMBotArmSystem"), *clock_, 5000,
                         "Read not fully available; keeping last state.");
  }

  // auto t_end = std::chrono::steady_clock::now();
  // double read_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("OMBotHW"), *clock_, 1000,
  //     "HW read took %.2f ms", read_ms);



  return return_type::OK;
}


return_type OMBotArmSystem::write(const rclcpp::Time&, const rclcpp::Duration&) {
  // auto t0 = std::chrono::steady_clock::now();

  if (!hw_connected_) {
    // Nothing to push; fake mode just updates state in read()
    // Still clamp commands to limits to keep simulation sane
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_position_cmd_[i] = std::clamp(joint_position_cmd_[i], pos_min_[i], pos_max_[i]);
    }
    return return_type::OK;
  }

  // ---- HARDWARE MODE ----
  // 1) Clamp SI commands against limits
  for (size_t i=0; i<ids_.size(); ++i) {
  switch (command_mode_[i]) {
    case CommandMode::Position:
      joint_position_cmd_[i] = std::clamp(joint_position_cmd_[i], pos_min_[i], pos_max_[i]);
      // convert to pulses + add to sync_write_goal_pos_
      break;
    case CommandMode::Velocity:
      joint_velocity_cmd_[i] = std::clamp(joint_velocity_cmd_[i], -vel_max_[i], vel_max_[i]);
      // convert to vel units + add to sync_write_goal_vel_
      break;
    case CommandMode::Effort:
      joint_effort_cmd_[i] = std::clamp(joint_effort_cmd_[i], -eff_max_[i], eff_max_[i]);
      // convert to mA + add to sync_write_goal_cur_
      break;
  }
}


  // 2) Build payloads and GroupSyncWrite once per mode
  bool ok = true;

  // Position mode
  if (any_position_mode_) {
    sync_write_goal_pos_->clearParam();
    for (size_t i = 0; i < ids_.size(); ++i) {
      if (command_mode_[i] != CommandMode::Position) continue;
      const int32_t ticks = rad_to_pulses(joint_position_cmd_[i], pulses_per_rev_[i]);
      uint8_t param[4] = {
        static_cast<uint8_t>(ticks & 0xFF),
        static_cast<uint8_t>((ticks >> 8) & 0xFF),
        static_cast<uint8_t>((ticks >> 16) & 0xFF),
        static_cast<uint8_t>((ticks >> 24) & 0xFF)
      };
      if (!sync_write_goal_pos_->addParam(ids_[i], param)) ok = false;
    }
    if (ok) {
      // const int res = sync_write_goal_pos_->txPacket();
      // if (res != COMM_SUCCESS) {
      //   RCLCPP_WARN(rclcpp::get_logger("OMBotArmSystem"),
      //               "sync write goal position failed: %s", packetHandler_->getTxRxResult(res));
      //   ok = false;
      // }
    }
  }

  // Velocity mode
  if (any_velocity_mode_) {
    sync_write_goal_vel_->clearParam();
    for (size_t i = 0; i < ids_.size(); ++i) {
      if (command_mode_[i] != CommandMode::Velocity) continue;
      const int32_t vu = rad_s_to_vel_units(joint_velocity_cmd_[i]);
      uint8_t param[4] = {
        static_cast<uint8_t>(vu & 0xFF),
        static_cast<uint8_t>((vu >> 8) & 0xFF),
        static_cast<uint8_t>((vu >> 16) & 0xFF),
        static_cast<uint8_t>((vu >> 24) & 0xFF)
      };
      if (!sync_write_goal_vel_->addParam(ids_[i], param)) ok = false;
    }
    if (ok) {
      // const int res = sync_write_goal_vel_->txPacket();
      // if (res != COMM_SUCCESS) {
      //   RCLCPP_WARN(rclcpp::get_logger("OMBotArmSystem"),
      //               "sync write goal velocity failed: %s", packetHandler_->getTxRxResult(res));
      //   ok = false;
      // }
    }
  }
  // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("OMBotArmSystem"), *log_clock_, 1000,
  // "any_effort_mode=%d  modes=[%d %d %d %d %d %d]",
  // (int)any_effort_mode_, (int)command_mode_[0], (int)command_mode_[1],
  // (int)command_mode_[2], (int)command_mode_[3], (int)command_mode_[4],
  // (int)command_mode_[5]);


  // Effort (current) mode
  if (any_effort_mode_) {
    sync_write_goal_cur_->clearParam();

    // std::ostringstream oss;
    // oss.setf(std::ios::fixed); oss.precision(3);
    // oss << "Currents mA: [";
    // bool first = true;


    for (size_t i = 0; i < ids_.size(); ++i) {
      if (command_mode_[i] != CommandMode::Effort) continue;
      // Nm -> A -> units
      // const double amps = joint_effort_cmd_[i] / kt_Nm_per_A_[i];
      const int16_t cu  = torque_to_current_mA(joint_effort_cmd_[i], kt_Nm_per_A_[i]);
      // if (!first) oss << " ";
      //     first = false;
      //     oss << cu;

      uint8_t param[2] = {
        static_cast<uint8_t>(cu & 0xFF),
        static_cast<uint8_t>((cu >> 8) & 0xFF)
      };
      if (!sync_write_goal_cur_->addParam(ids_[i], param)) ok = false;
    }
    // oss << "]";
    // RCLCPP_INFO_THROTTLE(
    //   rclcpp::get_logger("OMBotArmSystem"),
    //   *log_clock_, 500,
    //   "%s", oss.str().c_str()
    // );


    if (ok) {
      const int res = sync_write_goal_cur_->txPacket();
      if (res != COMM_SUCCESS) {
        RCLCPP_WARN(rclcpp::get_logger("OMBotArmSystem"),
                    "sync write goal current failed: %s", packetHandler_->getTxRxResult(res));
        ok = false;
      }
    }
  }
  // auto t1 = std::chrono::steady_clock::now();
  // double write_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("OMBotHW"), *clock_, 1000,
  //     "HW write took %.2f ms", write_ms);
  // double loop_ms = std::chrono::duration<double, std::milli>(t1 - t_prev_).count();
  // t_prev_ = t1;
  // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("OMBotHW"), *clock_, 1000,
  //     "Full HW loop = %.2f ms", loop_ms);
  return ok ? return_type::OK : return_type::ERROR;
}

}  // namespace ombot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ombot_hardware::OMBotArmSystem,
  hardware_interface::SystemInterface)
