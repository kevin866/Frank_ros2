#pragma once
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>
#include <io_context/io_context.hpp>

#include <io_context/io_context.hpp>
using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::StopBits;
using drivers::common::IoContext;
namespace ombot_hardware
{

// Minimal stub to abstract Roboteq I/O. Replace with real serial/CAN.
class RoboteqIface {
public:
  bool open(const std::string & device, int baud)
  {
    try {
      io_ctx_ = std::make_shared<IoContext>(1);
      driver_ = std::make_unique<SerialDriver>(*io_ctx_);

      SerialPortConfig cfg(
        static_cast<uint32_t>(baud),
        FlowControl::NONE,
        Parity::NONE,
        StopBits::ONE
      );

      driver_->init_port(device, cfg);
      driver_->port()->open();
      return true;
    } catch (...) { return false; }
  }

  void close() {
    if (driver_ && driver_->port() && driver_->port()->is_open()) {
      driver_->port()->close();
    }
  }

  // --- raw I/O helpers ---
  bool write_raw(std::vector<uint8_t> bytes) {
    if (!driver_ || !driver_->port() || !driver_->port()->is_open()) return false;
    return driver_->port()->send(bytes) == bytes.size();
  }
  bool write_raw(const std::string & s) {
    return write_raw(std::vector<uint8_t>(s.begin(), s.end()));
  }

  bool read_raw(std::string & out, size_t max_bytes = 256 /*unused*/, int /*timeout_ms*/ = 0) {
    (void)max_bytes;
    if (!driver_ || !driver_->port() || !driver_->port()->is_open()) return false;
    std::vector<uint8_t> buf(256);
    const size_t n = driver_->port()->receive(buf);   // <-- no timeout arg
    if (n == 0) return false;                          // no data available
    out.assign(buf.begin(), buf.begin() + n);
    return true;
  }

  std::string query_raw(const std::string &cmd)
  {
    if (!driver_ || !driver_->port() || !driver_->port()->is_open()) return "";

    // Send (add CR only if neither '_' nor CR present)
    std::string tx = cmd;
    if (!tx.empty() && tx.back() != '_' && tx.back() != '\r') tx.push_back('\r');
    std::vector<uint8_t> tx_bytes(tx.begin(), tx.end());
    driver_->port()->send(tx_bytes);  // ignore partial send; we’re in a tight loop

    // Read with a short time budget (~12ms)
    std::string out;
    std::vector<uint8_t> buf(64);
    auto t0 = std::chrono::steady_clock::now();
    auto budget = std::chrono::milliseconds(12);

    while (std::chrono::steady_clock::now() - t0 < budget) {
      size_t n = driver_->port()->receive(buf);
      if (n == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      for (size_t i = 0; i < n; ++i) {
        char c = static_cast<char>(buf[i]);
        if (c == '_' || c == '\r' || c == '\n') {
          // trim
          auto s = out.find_first_not_of(" \r\n");
          auto e = out.find_last_not_of(" \r\n");
          return (s == std::string::npos) ? std::string() : out.substr(s, e - s + 1);
        }
        out.push_back(c);
      }
    }
    // timeout: return whatever collected (or "")
    return out;
  }

// std::string query_raw(const std::string &cmd)
// {
//   if (!driver_ || !driver_->port() || !driver_->port()->is_open()) {
//     return "";
//   }

//   // 1) Flush any stale bytes in RX buffer (echoes from previous !S, etc.)
//   {
//     std::vector<uint8_t> flush_buf(64);
//     while (true) {
//       size_t n = driver_->port()->receive(flush_buf);
//       if (n == 0) break;  // nothing left
//     }
//   }

//   // 2) Build TX (ensure terminator)
//   std::string tx = cmd;
//   if (!tx.empty() && tx.back() != '_' && tx.back() != '\r') {
//     tx.push_back('\r');
//   }
//   std::vector<uint8_t> tx_bytes(tx.begin(), tx.end());
//   driver_->port()->send(tx_bytes);

//   // 3) Read lines until we get a "data" line or timeout
//   std::string out;
//   std::vector<uint8_t> buf(64);
//   auto t0 = std::chrono::steady_clock::now();
//   auto budget = std::chrono::milliseconds(20);  // a bit more generous

//   auto trim = [](std::string &s) {
//     auto start = s.find_first_not_of(" \r\n");
//     auto end   = s.find_last_not_of(" \r\n");
//     if (start == std::string::npos) { s.clear(); return; }
//     s = s.substr(start, end - start + 1);
//   };

//   // command prefix without terminator (for detecting echo)
//   std::string cmd_no_term = cmd;
//   while (!cmd_no_term.empty() && (cmd_no_term.back() == '_' || cmd_no_term.back() == '\r')) {
//     cmd_no_term.pop_back();
//   }

//   while (std::chrono::steady_clock::now() - t0 < budget) {
//     size_t n = driver_->port()->receive(buf);
//     if (n == 0) {
//       std::this_thread::sleep_for(std::chrono::milliseconds(1));
//       continue;
//     }

//     for (size_t i = 0; i < n; ++i) {
//       char c = static_cast<char>(buf[i]);
//       if (c == '_' || c == '\r' || c == '\n') {
//         // We have a line in 'out'
//         trim(out);
//         if (!out.empty()) {
//           // Skip echo lines: those that start with the command or start with '!'/'?'
//           bool looks_like_echo = false;
//           if (!cmd_no_term.empty() && out.rfind(cmd_no_term, 0) == 0) {
//             looks_like_echo = true;
//           } else if (!out.empty() && (out[0] == '!' || out[0] == '?')) {
//             looks_like_echo = true;
//           }

//           if (!looks_like_echo) {
//             return out;  // return the first non-echo line
//           }
//         }
//         // Reset buffer for next line
//         out.clear();
//       } else {
//         out.push_back(c);
//       }
//     }
//   }

//   // timeout: no good data line, return empty
//   return "";
// }


  // --- convenience commands (ASCII) ---
  // Send a speed command (example: "!G <ch> <value>\r")
  bool write_speed(int ch, double rpm) {
    // TODO: map rpm to your controller's expected units
    const int cmd_val = static_cast<int>(rpm);
    std::string cmd = "!G " + std::to_string(ch) + " " + std::to_string(cmd_val) + "\r";
    return write_raw(cmd);
  }


  bool read_speed(int ch, double & rpm_out) {
    std::string q = "?S " + std::to_string(ch) + "_";
    std::string resp = query_raw(q);
    if (resp.empty()) return false;
    
    try {
      rpm_out = std::stod(resp);
      return true;
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("RoboteqIface"),
                  "Failed to parse speed response '%s' for ch %d", resp.c_str(), ch);
      return false;
    }
  }


  bool read_encoder(int ch, int64_t & counts_out) {
    std::string q = "?C " + std::to_string(ch) + "_";
    std::string resp = query_raw(q);
    if (resp.empty()) return false;
    
    try {
      counts_out = static_cast<int64_t>(std::stoll(resp));
      return true;
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("RoboteqIface"),
                  "Failed to parse encoder response '%s' for ch %d", resp.c_str(), ch);
      return false;
    }
  }


private:
  std::shared_ptr<IoContext> io_ctx_;
  std::unique_ptr<SerialDriver> driver_;
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
  double max_wheel_rad_s_{200.0};
  rclcpp::Clock ros_clock_{RCL_STEADY_TIME};  // <-- add this
  // In OMBotBaseSystem (private:)
  size_t cycle_counter_{0};
  size_t pos_update_stride_{10};  // refresh true encoder pos ~every 10 cycles per wheel



  // Joint order: fl, fr, rl, rr
  std::vector<std::string> joint_names_;
  std::vector<double> pos_rad_;   // from encoders
  std::vector<double> vel_rad_s_; // from controller speed
  std::vector<double> cmd_rad_s_; // desired velocity
  std::vector<double> prev_pos_rad_; // for jump detection

  // Map joint index -> (controller, channel)
  struct Chan { int ctrl; int ch; }; // ctrl: 1 or 2, ch: 1 or 2
  std::vector<Chan> map_;

  // Two controllers
  RoboteqIface ctrl1_, ctrl2_;

  // Deadman flag for motor safety
  bool deadman_ = true;

  // Helpers
  double nativeSpeedFromRadPerSec(double rad_s) const { return rad_s; } // TODO: scale
  double radPerSecFromNative(double native) const { return native; }    // TODO: scale

  double radFromCounts(int64_t counts) const
  {
    const double rev = (static_cast<double>(counts) / encoder_cpr_) / gear_ratio_;
    return rev * 2.0 * M_PI;
  }



  // Helpers you can put in your class (private:)
  inline double wheelRadPerSec_from_motorRPM(double motor_rpm) const {
    // motor rpm -> motor rad/s -> wheel rad/s
    const double motor_rad_s = motor_rpm * (2.0 * M_PI / 60.0);
    return motor_rad_s / gear_ratio_;
  }

  inline double motorRPM_from_wheelRadPerSec(double wheel_rad_s) const {
    // wheel rad/s -> motor rad/s -> motor rpm
    const double motor_rad_s = wheel_rad_s * gear_ratio_;
    return motor_rad_s * (60.0 / (2.0 * M_PI));
  }

  inline double wheelRad_from_encoderCounts(int64_t counts) const {
    // counts -> motor rev -> wheel rad
    const double motor_rev = static_cast<double>(counts) / encoder_cpr_;
    const double wheel_rev = motor_rev / gear_ratio_;
    return wheel_rev * (2.0 * M_PI);
  }
};

} // namespace ombot_hardware
