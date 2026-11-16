// Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).
//
// Offline calibration tool. Measures the gyro and accelerometer biases while
// the sensor lies flat and still, and prints lines to paste into the YAML file.
//
// Unlike the original package, the accelerometer bias KEEPS gravity: after
// calibration a flat sensor reads about +9.81 m/s^2 on its vertical axis, which
// is what ROS (REP-145) expects.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "mpu6050_driver/i2c_bus.hpp"
#include "mpu6050_driver/mpu6050.hpp"
#include "mpu6050_driver/running_stats.hpp"

using mpu6050_driver::kGravity;

namespace
{

struct Options
{
  std::string bus{"/dev/i2c-1"};
  unsigned address{0x68};
  double duration{10.0};
  unsigned gyro_range{0};
  unsigned accel_range{0};
  unsigned dlpf{3};
};

void printUsage()
{
  std::cout <<
    "Usage: ros2 run mpu6050_driver mpu6050_calibrate [options]\n"
    "\n"
    "Place the sensor FLAT (Z axis up) and do not touch it during the measurement.\n"
    "Power it for a few minutes first so its temperature is stable.\n"
    "\n"
    "Options:\n"
    "  --bus PATH          I2C device (default /dev/i2c-1)\n"
    "  --address N         I2C address, decimal or hex (default 0x68)\n"
    "  --duration S        seconds of data to average (default 10)\n"
    "  --gyro-range N      0..3, use the same value as in your YAML (default 0)\n"
    "  --accel-range N     0..3, use the same value as in your YAML (default 0)\n"
    "  --dlpf N            0..6, use the same value as in your YAML (default 3)\n"
    "  -h, --help          show this help\n";
}

Options parseArgs(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      printUsage();
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("Missing value for " + arg);
    }
    const std::string value = argv[++i];
    if (arg == "--bus") {
      o.bus = value;
    } else if (arg == "--address") {
      o.address = static_cast<unsigned>(std::stoul(value, nullptr, 0));
    } else if (arg == "--duration") {
      o.duration = std::stod(value);
    } else if (arg == "--gyro-range") {
      o.gyro_range = static_cast<unsigned>(std::stoul(value));
    } else if (arg == "--accel-range") {
      o.accel_range = static_cast<unsigned>(std::stoul(value));
    } else if (arg == "--dlpf") {
      o.dlpf = static_cast<unsigned>(std::stoul(value));
    } else {
      throw std::invalid_argument("Unknown option " + arg + " (see --help)");
    }
  }
  if (o.duration < 1.0) {
    throw std::invalid_argument("--duration must be at least 1 second");
  }
  if (o.address > 0x77) {
    throw std::invalid_argument("--address must be a 7-bit I2C address");
  }
  return o;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options opt = parseArgs(argc, argv);

    mpu6050_driver::LinuxI2CBus bus(opt.bus, static_cast<std::uint8_t>(opt.address));
    mpu6050_driver::Mpu6050 imu(bus);

    const auto info = imu.probe();
    std::printf("Found %s (WHO_AM_I = 0x%02X) on %s at 0x%02X\n", info.description.c_str(),
      info.who_am_i, opt.bus.c_str(), opt.address);
    if (!info.genuine) {
      std::printf("WARNING: not a genuine MPU-6050; results may differ from the datasheet.\n");
    }

    mpu6050_driver::Config config;
    config.gyro_range = static_cast<mpu6050_driver::GyroRange>(opt.gyro_range);
    config.accel_range = static_cast<mpu6050_driver::AccelRange>(opt.accel_range);
    config.dlpf = static_cast<std::uint8_t>(opt.dlpf);
    imu.initialize(config);  // validates the values too

    using clock = std::chrono::steady_clock;
    constexpr auto kPeriod = std::chrono::milliseconds(5);  // 200 Hz

    std::printf("Warming up for 1 s...\n");
    const auto warmup_end = clock::now() + std::chrono::seconds(1);
    while (clock::now() < warmup_end) {
      imu.read();
      std::this_thread::sleep_for(kPeriod);
    }

    std::printf("Measuring for %.0f s. Do not touch the sensor...\n", opt.duration);
    mpu6050_driver::RunningStats3 gyro_stats;
    mpu6050_driver::RunningStats3 accel_stats;
    double temperature = 0.0;
    const auto end = clock::now() + std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(opt.duration));
    auto next = clock::now();
    while (clock::now() < end) {
      const auto s = imu.read();
      gyro_stats.add(s.gyro);
      accel_stats.add(s.accel);
      temperature = s.temperature;
      next += kPeriod;
      std::this_thread::sleep_until(next);
    }

    const auto gyro_mean = gyro_stats.mean();
    const auto gyro_std = gyro_stats.stddev();
    const auto accel_mean = accel_stats.mean();
    const auto accel_std = accel_stats.stddev();

    // Gravity is on the axis with the largest mean acceleration. Keep it in
    // the reading: the bias on that axis is the difference from +/-g.
    std::size_t up = 0;
    for (std::size_t i = 1; i < 3; ++i) {
      if (std::fabs(accel_mean[i]) > std::fabs(accel_mean[up])) {
        up = i;
      }
    }
    std::array<double, 3> accel_bias = accel_mean;
    accel_bias[up] = accel_mean[up] - std::copysign(kGravity, accel_mean[up]);

    const double norm = std::sqrt(
      accel_mean[0] * accel_mean[0] + accel_mean[1] * accel_mean[1] +
      accel_mean[2] * accel_mean[2]);
    const double tilt_deg = std::acos(std::min(1.0, std::fabs(accel_mean[up]) / norm)) *
      180.0 / 3.14159265358979323846;

    std::printf("\nSamples: %zu   Temperature: %.1f C\n", gyro_stats.count(), temperature);
    std::printf("Gyro  mean [rad/s]: [% .6f, % .6f, % .6f]  noise: [%.5f, %.5f, %.5f]\n",
      gyro_mean[0], gyro_mean[1], gyro_mean[2], gyro_std[0], gyro_std[1], gyro_std[2]);
    std::printf("Accel mean [m/s^2]: [% .4f, % .4f, % .4f]  noise: [%.4f, %.4f, %.4f]\n",
      accel_mean[0], accel_mean[1], accel_mean[2], accel_std[0], accel_std[1], accel_std[2]);
    std::printf("Gravity measured on axis %c (|a| = %.3f m/s^2, tilt %.1f deg)\n",
      "XYZ"[up], norm, tilt_deg);

    if (gyro_stats.maxStddev() > 0.02) {
      std::printf("\nERROR: the sensor moved during the measurement (gyro noise too high).\n"
        "Fix it in place and run the tool again.\n");
      return 2;
    }
    if (up != 2 || accel_mean[2] < 0.0) {
      std::printf("\nWARNING: gravity is not on +Z. The sensor is not lying flat with Z up.\n"
        "That's fine if it is mounted that way on the robot, but the static TF\n"
        "base_link -> imu_link must describe that rotation.\n");
    }
    if (tilt_deg > 2.0) {
      std::printf("\nWARNING: the sensor is tilted %.1f deg. accel_bias on the horizontal axes\n"
        "includes that tilt. Level the board, or keep accel_bias at [0, 0, 0] if you\n"
        "only fuse the gyro (recommended for wheel odometry).\n", tilt_deg);
    }

    // Measured white noise underestimates real-world error (vibration, bias
    // drift), so suggest at least 3x the measurement with a sensible floor.
    const double gyro_sd = std::max(0.005, 3.0 * gyro_stats.maxStddev());
    const double accel_sd = std::max(0.05, 3.0 * accel_stats.maxStddev());

    std::printf(
      "\nPaste these lines into config/mpu6050.yaml (under ros__parameters):\n\n"
      "    gyro_bias: [%.6f, %.6f, %.6f]    # rad/s\n"
      "    accel_bias: [%.4f, %.4f, %.4f]    # m/s^2, gravity kept\n"
      "    angular_velocity_stddev: %.4f       # rad/s\n"
      "    linear_acceleration_stddev: %.4f    # m/s^2\n\n"
      "Note: with calibrate_gyro_on_startup: true the node re-measures the gyro bias\n"
      "every time it starts; gyro_bias above is only used if that calibration fails.\n",
      gyro_mean[0], gyro_mean[1], gyro_mean[2], accel_bias[0], accel_bias[1], accel_bias[2],
      gyro_sd, accel_sd);
    return 0;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }
}
