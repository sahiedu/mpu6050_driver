// Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).
//
// ROS 2 node that publishes MPU6050 data as sensor_msgs/Imu.

#ifndef MPU6050_DRIVER__MPU6050_NODE_HPP_
#define MPU6050_DRIVER__MPU6050_NODE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mpu6050_driver/i2c_bus.hpp"
#include "mpu6050_driver/mpu6050.hpp"
#include "mpu6050_driver/running_stats.hpp"

namespace mpu6050_driver
{

class Mpu6050Node : public rclcpp::Node
{
public:
  /// If `bus` is null the node opens the I2C bus given by the parameters.
  /// Passing a bus is meant for tests (a simulated sensor).
  /// Throws std::exception with a readable message if the sensor cannot be
  /// configured.
  explicit Mpu6050Node(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<RegisterInterface> bus = nullptr);

private:
  enum class State { kCalibrating, kRunning };

  void loadParameters();
  void onTimer();
  void startCalibration();
  void finishCalibration();
  void handleReadError(const std::exception & error);
  void tryReinitialize();
  void publish(const Sample & sample, const rclcpp::Time & stamp);
  void onCalibrateRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Parameters
  std::string i2c_bus_;
  int i2c_address_{0x68};
  std::string frame_id_;
  double publish_rate_{100.0};
  Config sensor_config_;
  bool calibrate_on_startup_{true};
  double calibration_duration_{2.0};
  double calibration_max_gyro_stddev_{0.02};
  int calibration_max_attempts_{5};
  std::array<double, 3> gyro_bias_{};   // [rad/s], subtracted from the gyro
  std::array<double, 3> accel_bias_{};  // [m/s^2], subtracted from the accel
  double angular_velocity_variance_{0.0};
  double linear_acceleration_variance_{0.0};
  bool publish_temperature_{true};
  int max_consecutive_errors_{10};

  // Hardware (bus_ must be declared before imu_: imu_ holds a reference)
  std::unique_ptr<RegisterInterface> bus_;
  std::unique_ptr<Mpu6050> imu_;

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Runtime state
  State state_{State::kRunning};
  bool has_bias_{false};  // no IMU message is published until this is true
  RunningStats3 calibration_stats_;
  std::size_t calibration_samples_needed_{0};
  int calibration_attempt_{0};
  int consecutive_errors_{0};
  bool needs_reinit_{false};
  std::chrono::steady_clock::time_point last_reinit_attempt_{};
  std::size_t temperature_divider_{1};
  std::size_t sample_counter_{0};
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER__MPU6050_NODE_HPP_
