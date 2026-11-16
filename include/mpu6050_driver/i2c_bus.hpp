// Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).
//
// Register-level access to an I2C device.
//
// RegisterInterface is an abstract class so that the MPU6050 driver can be
// tested without hardware (see test/fake_mpu6050.hpp). LinuxI2CBus is the real
// implementation on top of the Linux i2c-dev interface (/dev/i2c-N).

#ifndef MPU6050_DRIVER__I2C_BUS_HPP_
#define MPU6050_DRIVER__I2C_BUS_HPP_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace mpu6050_driver
{

/// Thrown on any I2C communication failure. The message explains the cause.
class I2CError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Minimal register read/write interface used by the driver.
class RegisterInterface
{
public:
  virtual ~RegisterInterface() = default;

  /// Read `len` consecutive registers starting at `reg` in ONE transaction.
  /// Throws I2CError on failure.
  virtual void readRegisters(std::uint8_t reg, std::uint8_t * data, std::size_t len) = 0;

  /// Write a single register. Throws I2CError on failure.
  virtual void writeRegister(std::uint8_t reg, std::uint8_t value) = 0;
};

/// Real I2C bus on Linux (e.g. /dev/i2c-1 on a Raspberry Pi).
class LinuxI2CBus : public RegisterInterface
{
public:
  /// Opens `device` and selects the slave `address`. Throws I2CError with a
  /// hint about the usual fix (permissions, I2C not enabled, wrong address).
  LinuxI2CBus(const std::string & device, std::uint8_t address);
  ~LinuxI2CBus() override;

  LinuxI2CBus(const LinuxI2CBus &) = delete;
  LinuxI2CBus & operator=(const LinuxI2CBus &) = delete;

  void readRegisters(std::uint8_t reg, std::uint8_t * data, std::size_t len) override;
  void writeRegister(std::uint8_t reg, std::uint8_t value) override;

private:
  std::string describeError(const std::string & action, int err) const;

  int fd_{-1};
  std::string device_;
  std::uint8_t address_;
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER__I2C_BUS_HPP_
