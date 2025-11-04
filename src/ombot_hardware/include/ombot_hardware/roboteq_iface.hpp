// roboteq_iface.hpp
#pragma once
#include <string>
#include <cstdint>

class RoboteqIface {
public:
  explicit RoboteqIface(const std::string& dev, int baud = 115200);
  ~RoboteqIface();

  bool is_ok() const { return fd_ >= 0; }

  // ch is 1 or 2 (SDC2160 has only ch=1; ignore others gracefully)
  bool read_encoder(int ch, int64_t& counts);
  bool read_speed(int ch, double& rpm);

  // Optional: reset encoder counter
  bool reset_encoder(int ch);

private:
  int fd_{-1};
  bool write_cmd(const std::string& cmd);
  bool query(const std::string& cmd, std::string& resp, int timeout_ms = 50);
  static bool parse_pair(const std::string& line,
                         const std::string& key,
                         int ch, int64_t& out); // supports "KEY=a" or "KEY=a:b"

  // one-time controller setup
  void initial_config();
};
