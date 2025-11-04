// roboteq_iface.cpp
#include "roboteq_iface.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#include <regex>

static bool set_interface_attribs(int fd, int speed) {
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) return false;

  cfmakeraw(&tty);
  tty.c_cflag = CLOCAL | CREAD | CS8;  // 8N1
  tty.c_iflag = IGNPAR;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cc[VTIME] = 0; // read timeout handled via select()
  tty.c_cc[VMIN]  = 0;

  return tcsetattr(fd, TCSANOW, &tty) == 0;
}

RoboteqIface::RoboteqIface(const std::string& dev, int baud) {
  (void)baud; // fixed 115200 here; adjust if needed
  fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ >= 0) {
    if (!set_interface_attribs(fd_, B115200)) { ::close(fd_); fd_ = -1; return; }
    initial_config();
  }
}

RoboteqIface::~RoboteqIface() {
  if (fd_ >= 0) ::close(fd_);
}

void RoboteqIface::initial_config() {
  // Turn off command echo so replies are clean
  // NOTE: Roboteq expects CR '\r' line endings.
  std::string tmp;
  query("^ECHOF 1\r", tmp, 100);   // disable echo (no error if unknown)
  query("?FID\r", tmp, 100);       // touch the port; ignore result
}

bool RoboteqIface::write_cmd(const std::string& cmd) {
  if (fd_ < 0) return false;
  ssize_t n = ::write(fd_, cmd.data(), cmd.size());
  return n == (ssize_t)cmd.size();
}

bool RoboteqIface::query(const std::string& cmd, std::string& resp, int timeout_ms) {
  if (!write_cmd(cmd)) return false;

  // Read until CR or timeout
  resp.clear();
  std::string buf;
  fd_set readfds;
  timeval tv{ timeout_ms/1000, (timeout_ms%1000)*1000 };

  while (true) {
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);
    auto r = select(fd_+1, &readfds, nullptr, nullptr, &tv);
    if (r < 0) return false;
    if (r == 0) break; // timeout

    char tmp[256];
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0) {
      buf.append(tmp, tmp + n);
      // Roboteq replies end with '\r'
      auto pos = buf.find('\r');
      if (pos != std::string::npos) {
        resp = buf.substr(0, pos);
        break;
      }
    } else {
      // no data; try again until timeout
    }
  }
  return !resp.empty();
}

// Parse "KEY=a" or "KEY=a:b". Choose channel (1 or 2) value.
bool RoboteqIface::parse_pair(const std::string& line, const std::string& key,
                              int ch, int64_t& out) {
  // example lines: "C=12345", "C=12345:6789", "S=87:90"
  auto eq = line.find('=');
  if (eq == std::string::npos) return false;
  if (line.substr(0, eq) != key) return false;
  std::string vals = line.substr(eq+1); // "12345" or "12345:6789"

  size_t colon = vals.find(':');
  std::string pick;
  if (colon == std::string::npos) {
    pick = vals;
  } else {
    // ch=1 -> left of colon, ch=2 -> right
    pick = (ch == 2) ? vals.substr(colon+1) : vals.substr(0, colon);
  }
  try {
    out = std::stoll(pick);
    return true;
  } catch (...) { return false; }
}

bool RoboteqIface::read_encoder(int ch, int64_t& counts) {
  if (fd_ < 0) return false;
  if (ch != 1 && ch != 2) ch = 1;           // SDC2160 is single channel
  std::string line;
  if (!query("?C " + std::to_string(ch) + "\r", line, 50)) return false;
  int64_t v = 0;
  if (!parse_pair(line, "C", ch, v)) return false;
  counts = v;
  return true;
}

bool RoboteqIface::read_speed(int ch, double& rpm) {
  if (fd_ < 0) return false;
  if (ch != 1 && ch != 2) ch = 1;
  std::string line;
  if (!query("?S " + std::to_string(ch) + "\r", line, 50)) return false;
  int64_t v = 0;
  if (!parse_pair(line, "S", ch, v)) return false;
  rpm = static_cast<double>(v);   // Roboteq returns RPM
  return true;
}

bool RoboteqIface::reset_encoder(int ch) {
  if (fd_ < 0) return false;
  if (ch != 1 && ch != 2) ch = 1;
  std::string resp;
  // "C" (no question mark) sets counter. "C 1=0" sets ch1 to zero.
  return query("C " + std::to_string(ch) + "=0\r", resp, 50);
}
