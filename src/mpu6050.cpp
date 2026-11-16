// Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).

#include "mpu6050_driver/mpu6050.hpp"

#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace mpu6050_driver
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

std::string hex(unsigned value)
{
  std::ostringstream os;
  os << "0x" << std::hex << std::setw(2) << std::setfill('0') << value;
  return os.str();
}

std::int16_t bigEndian16(const std::uint8_t * p)
{
  return static_cast<std::int16_t>(static_cast<std::uint16_t>((p[0] << 8) | p[1]));
}

bool atLimit(std::int16_t v)
{
  return v == std::numeric_limits<std::int16_t>::max() ||
         v == std::numeric_limits<std::int16_t>::min();
}
}  // namespace

Mpu6050::Mpu6050(RegisterInterface & bus)
: bus_(bus) {}

double Mpu6050::gyroSensitivity(GyroRange range)
{
  switch (range) {
    case GyroRange::DPS_250: return 131.0;
    case GyroRange::DPS_500: return 65.5;
    case GyroRange::DPS_1000: return 32.8;
    case GyroRange::DPS_2000: return 16.4;
  }
  throw std::invalid_argument("invalid gyro range");
}

double Mpu6050::accelSensitivity(AccelRange range)
{
  switch (range) {
    case AccelRange::G_2: return 16384.0;
    case AccelRange::G_4: return 8192.0;
    case AccelRange::G_8: return 4096.0;
    case AccelRange::G_16: return 2048.0;
  }
  throw std::invalid_argument("invalid accel range");
}

void Mpu6050::validate(const Config & c)
{
  if (static_cast<unsigned>(c.gyro_range) > 3) {
    throw std::invalid_argument("gyro_range must be 0..3");
  }
  if (static_cast<unsigned>(c.accel_range) > 3) {
    throw std::invalid_argument("accel_range must be 0..3");
  }
  if (c.dlpf > 6) {
    throw std::invalid_argument("dlpf must be 0..6");
  }
  if (c.clock_source > 3) {
    throw std::invalid_argument("clock_source must be 0..3");
  }
}

DeviceInfo Mpu6050::probe()
{
  DeviceInfo info;
  bus_.readRegisters(reg::WHO_AM_I, &info.who_am_i, 1);

  switch (info.who_am_i) {
    case 0x68:
      info.genuine = true;
      info.description = "MPU-6050";
      break;
    case 0x70:
      info.description = "MPU-6500 (often sold as MPU-6050)";
      break;
    case 0x71:
      info.description = "MPU-9250";
      break;
    case 0x72:
    case 0x98:
      info.description = "MPU-6050 clone";
      break;
    case 0x73:
      info.description = "MPU-9255";
      break;
    case 0x00:
    case 0xFF:
      throw I2CError(
              "WHO_AM_I returned " + hex(info.who_am_i) +
              ": no MPU6050 is answering at this address");
    default:
      info.description = "unknown device";
      break;
  }
  return info;
}

void Mpu6050::initialize(const Config & config)
{
  using namespace std::chrono_literals;
  validate(config);

  // Full device reset: all registers go back to defaults and the chip sleeps.
  bus_.writeRegister(reg::PWR_MGMT_1, 0x80);
  std::this_thread::sleep_for(100ms);
  // Reset the gyro/accel/temp signal paths (bits self-clear, not verified).
  bus_.writeRegister(reg::SIGNAL_PATH_RESET, 0x07);
  std::this_thread::sleep_for(100ms);

  // Wake up (SLEEP = 0) and select the clock source.
  writeVerified(reg::PWR_MGMT_1, config.clock_source);
  writeVerified(reg::PWR_MGMT_2, 0x00);  // all axes enabled
  writeVerified(reg::SMPLRT_DIV, config.sample_rate_divider);
  writeVerified(reg::CONFIG, config.dlpf);  // EXT_SYNC off
  writeVerified(reg::GYRO_CONFIG, static_cast<std::uint8_t>(config.gyro_range) << 3);
  writeVerified(reg::ACCEL_CONFIG, static_cast<std::uint8_t>(config.accel_range) << 3);

  config_ = config;
  // Let the PLL lock and the digital filters fill up before the first read.
  std::this_thread::sleep_for(50ms);
}

void Mpu6050::writeVerified(std::uint8_t reg, std::uint8_t value)
{
  bus_.writeRegister(reg, value);
  std::uint8_t read_back = 0;
  bus_.readRegisters(reg, &read_back, 1);
  if (read_back != value) {
    throw I2CError(
            "Register " + hex(reg) + " verification failed: wrote " + hex(value) +
            ", read back " + hex(read_back));
  }
}

Sample Mpu6050::read()
{
  std::array<std::uint8_t, 14> raw{};
  bus_.readRegisters(reg::ACCEL_XOUT_H, raw.data(), raw.size());
  return convert(raw, config_.gyro_range, config_.accel_range);
}

Sample Mpu6050::convert(
  const std::array<std::uint8_t, 14> & raw, GyroRange gyro_range,
  AccelRange accel_range)
{
  // Layout: AX AY AZ TEMP GX GY GZ, each a big-endian int16.
  const double accel_scale = kGravity / accelSensitivity(accel_range);
  const double gyro_scale = kDegToRad / gyroSensitivity(gyro_range);

  Sample s;
  for (std::size_t i = 0; i < 3; ++i) {
    const std::int16_t a = bigEndian16(&raw[2 * i]);
    const std::int16_t g = bigEndian16(&raw[8 + 2 * i]);
    s.accel[i] = a * accel_scale;
    s.gyro[i] = g * gyro_scale;
    s.saturated = s.saturated || atLimit(a) || atLimit(g);
  }
  s.temperature = bigEndian16(&raw[6]) / 340.0 + 36.53;
  return s;
}

}  // namespace mpu6050_driver
