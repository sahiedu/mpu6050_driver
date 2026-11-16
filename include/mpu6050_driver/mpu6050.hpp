// Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).
// Register map and scale factors follow the InvenSense MPU-6000/MPU-6050
// Register Map and Descriptions (RM-MPU-6000A-00, rev 4.2). The original
// layered design comes from kimsniper/ros2_mpu6050 (BSD-3-Clause,
// Copyright (c) 2022, Mezael Docoy).
//
// Hardware driver for the MPU6050, with no ROS dependency.

#ifndef MPU6050_DRIVER__MPU6050_HPP_
#define MPU6050_DRIVER__MPU6050_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "mpu6050_driver/i2c_bus.hpp"

namespace mpu6050_driver
{

/// Standard gravity [m/s^2].
constexpr double kGravity = 9.80665;

/// Register addresses (subset used by this driver).
namespace reg
{
constexpr std::uint8_t SMPLRT_DIV = 0x19;
constexpr std::uint8_t CONFIG = 0x1A;
constexpr std::uint8_t GYRO_CONFIG = 0x1B;
constexpr std::uint8_t ACCEL_CONFIG = 0x1C;
constexpr std::uint8_t ACCEL_XOUT_H = 0x3B;  // start of the 14-byte data block
constexpr std::uint8_t SIGNAL_PATH_RESET = 0x68;
constexpr std::uint8_t PWR_MGMT_1 = 0x6B;
constexpr std::uint8_t PWR_MGMT_2 = 0x6C;
constexpr std::uint8_t WHO_AM_I = 0x75;
}  // namespace reg

/// Gyroscope full scale (FS_SEL).
enum class GyroRange : std::uint8_t { DPS_250 = 0, DPS_500 = 1, DPS_1000 = 2, DPS_2000 = 3 };

/// Accelerometer full scale (AFS_SEL).
enum class AccelRange : std::uint8_t { G_2 = 0, G_4 = 1, G_8 = 2, G_16 = 3 };

/// Sensor configuration applied by Mpu6050::initialize().
struct Config
{
  GyroRange gyro_range{GyroRange::DPS_250};
  AccelRange accel_range{AccelRange::G_2};
  /// DLPF_CFG 0..6. 0 = 260 Hz (noisy) ... 3 = 44/42 Hz ... 6 = 5 Hz.
  std::uint8_t dlpf{3};
  /// CLKSEL 0..3. 0 = internal 8 MHz oscillator, 1..3 = PLL with gyro X/Y/Z
  /// reference (recommended by the datasheet for better stability).
  std::uint8_t clock_source{1};
  /// Sample rate = gyro output rate / (1 + divider). Gyro output rate is
  /// 1 kHz when dlpf is 1..6 and 8 kHz when dlpf is 0.
  std::uint8_t sample_rate_divider{0};
};

/// One measurement, converted to SI units, in the sensor's own axes
/// (the X/Y arrows printed on the breakout board, Z up out of the chip).
struct Sample
{
  std::array<double, 3> accel{};  ///< [m/s^2], gravity included
  std::array<double, 3> gyro{};   ///< [rad/s]
  double temperature{0.0};        ///< [deg C]
  bool saturated{false};          ///< true if any axis hit the end of its range
};

/// Result of reading WHO_AM_I.
struct DeviceInfo
{
  std::uint8_t who_am_i{0};
  bool genuine{false};     ///< true for a real MPU6050 (0x68)
  std::string description;
};

class Mpu6050
{
public:
  /// The bus must outlive this object.
  explicit Mpu6050(RegisterInterface & bus);

  /// Read WHO_AM_I. Throws I2CError if nothing answers (0x00 / 0xFF).
  /// Register-compatible clones (e.g. MPU6500 = 0x70) are accepted with
  /// genuine = false so the caller can print a warning.
  DeviceInfo probe();

  /// Reset the chip, wake it up and apply `config`. Every configuration
  /// register is read back and checked. Takes about 250 ms.
  /// Throws std::invalid_argument for bad values and I2CError on bus errors.
  void initialize(const Config & config);

  /// Read accelerometer, temperature and gyroscope in ONE 14-byte burst, so
  /// all axes belong to the same sample. Throws I2CError on failure.
  Sample read();

  const Config & config() const {return config_;}

  /// LSB per deg/s for each gyro range (131, 65.5, 32.8, 16.4).
  static double gyroSensitivity(GyroRange range);
  /// LSB per g for each accelerometer range (16384, 8192, 4096, 2048).
  static double accelSensitivity(AccelRange range);
  /// Convert the raw 14-byte block (big endian, starting at ACCEL_XOUT_H).
  static Sample convert(
    const std::array<std::uint8_t, 14> & raw, GyroRange gyro_range,
    AccelRange accel_range);
  /// Throws std::invalid_argument if a field is out of range.
  static void validate(const Config & config);

private:
  void writeVerified(std::uint8_t reg, std::uint8_t value);

  RegisterInterface & bus_;
  Config config_{};
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER__MPU6050_HPP_
