#ifndef NOIR_FIRMWARE__SERIAL_PORT_HPP_
#define NOIR_FIRMWARE__SERIAL_PORT_HPP_

#include <string>

namespace noir_firmware
{

// Minimal blocking-read-with-timeout serial port (POSIX termios).
// No external dependencies. This is the "transport" layer only: it
// knows nothing about ROS or your wire protocol.
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Opens the device (e.g. "/dev/ttyUSB0") at the given baud rate
  // (must be one of the termios B-constants' values: 9600, 115200, ...).
  // Returns false and leaves the port closed on failure.
  bool open(const std::string & device, int baud);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // Sends `line` followed by '\n'. Returns false on a write error.
  bool write_line(const std::string & line);

  // Reads one '\n'-terminated line, waiting up to timeout_ms.
  // Returns false (leaving `line` untouched) on timeout or error.
  bool read_line(std::string & line, int timeout_ms = 50);

private:
  int fd_{-1};
};

}  // namespace noir_firmware

#endif  // NOIR_FIRMWARE__SERIAL_PORT_HPP_