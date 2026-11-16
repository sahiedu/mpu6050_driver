// Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).

#include "mpu6050_driver/i2c_bus.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>

extern "C" {
#include <i2c/smbus.h>
}

namespace mpu6050_driver
{

namespace
{
std::string hex(unsigned value)
{
  std::ostringstream os;
  os << "0x" << std::hex << std::setw(2) << std::setfill('0') << value;
  return os.str();
}
}  // namespace

LinuxI2CBus::LinuxI2CBus(const std::string & device, std::uint8_t address)
: device_(device), address_(address)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    const int err = errno;
    std::string hint;
    if (err == ENOENT) {
      hint = " I2C is not enabled: add 'dtparam=i2c_arm=on' to /boot/firmware/config.txt "
        "and reboot (or check the bus number with 'ls /dev/i2c-*').";
    } else if (err == EACCES) {
      hint = " Your user cannot access the bus: run 'sudo usermod -aG i2c $USER', "
        "then log out and back in.";
    }
    throw I2CError("Cannot open " + device + ": " + std::strerror(err) + "." + hint);
  }

  if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
    const int err = errno;
    ::close(fd_);
    fd_ = -1;
    throw I2CError(describeError("select address", err));
  }
}

LinuxI2CBus::~LinuxI2CBus()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void LinuxI2CBus::readRegisters(std::uint8_t reg, std::uint8_t * data, std::size_t len)
{
  if (len == 0 || len > I2C_SMBUS_BLOCK_MAX) {
    throw I2CError("Invalid I2C read length " + std::to_string(len));
  }
  const int n = i2c_smbus_read_i2c_block_data(fd_, reg, static_cast<std::uint8_t>(len), data);
  if (n < 0) {
    throw I2CError(describeError("read register " + hex(reg), errno));
  }
  if (static_cast<std::size_t>(n) != len) {
    throw I2CError(
            "Short I2C read from register " + hex(reg) + ": got " + std::to_string(n) +
            " of " + std::to_string(len) + " bytes");
  }
}

void LinuxI2CBus::writeRegister(std::uint8_t reg, std::uint8_t value)
{
  if (i2c_smbus_write_byte_data(fd_, reg, value) < 0) {
    throw I2CError(describeError("write register " + hex(reg), errno));
  }
}

std::string LinuxI2CBus::describeError(const std::string & action, int err) const
{
  std::string msg = "I2C " + action + " failed on " + device_ + " (address " + hex(address_) +
    "): " + std::strerror(err) + ".";
  if (err == EREMOTEIO || err == ENXIO || err == EIO) {
    msg += " The sensor is not answering: check the wiring (SDA, SCL, 3.3V, GND) and the "
      "address with 'i2cdetect -y 1' (0x68 if AD0 is low, 0x69 if high).";
  }
  return msg;
}

}  // namespace mpu6050_driver
