#include "noirbot_firmware/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace noirbot_firmware
{

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baud)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }

  speed_t speed;
  switch (baud) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    default:
      close();
      return false;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, no flow control, raw mode.
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_lflag = 0;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write_line(const std::string & line)
{
  if (fd_ < 0) {
    return false;
  }
  const std::string out = line + "\n";
  ssize_t written = ::write(fd_, out.c_str(), out.size());
  return written == static_cast<ssize_t>(out.size());
}

bool SerialPort::read_line(std::string & line, int timeout_ms)
{
  if (fd_ < 0) {
    return false;
  }

  std::string buffer;
  pollfd pfd{fd_, POLLIN, 0};

  while (true) {
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) {
      return false;  // timeout or error
    }

    char c;
    ssize_t n = ::read(fd_, &c, 1);
    if (n <= 0) {
      return false;
    }
    if (c == '\n') {
      line = buffer;
      return true;
    }
    if (c != '\r') {
      buffer += c;
    }
  }
}

}  // namespace noirbot_firmware