// Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).

#include "mpu6050_driver/mpu6050_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mpu6050_driver
{

namespace
{
std::array<double, 3> toArray3(const std::vector<double> & v, const std::string & name)
{
  if (v.size() != 3) {
    throw std::invalid_argument("Parameter '" + name + "' must have exactly 3 values [x, y, z]");
  }
  return {v[0], v[1], v[2]};
}

int intInRange(rclcpp::Node & node, const std::string & name, int lo, int hi)
{
  const int value = static_cast<int>(node.get_parameter(name).as_int());
  if (value < lo || value > hi) {
    throw std::invalid_argument(
            "Parameter '" + name + "' = " + std::to_string(value) + " is out of range [" +
            std::to_string(lo) + ", " + std::to_string(hi) + "]");
  }
  return value;
}
}  // namespace

Mpu6050Node::Mpu6050Node(
  const rclcpp::NodeOptions & options,
  std::unique_ptr<RegisterInterface> bus)
: rclcpp::Node("mpu6050", options), bus_(std::move(bus))
{
  loadParameters();

  // --- Hardware ---------------------------------------------------------
  if (!bus_) {
    bus_ = std::make_unique<LinuxI2CBus>(i2c_bus_, static_cast<std::uint8_t>(i2c_address_));
  }
  imu_ = std::make_unique<Mpu6050>(*bus_);

  const DeviceInfo info = imu_->probe();
  if (info.genuine) {
    RCLCPP_INFO(get_logger(), "Found %s (WHO_AM_I = 0x%02X)", info.description.c_str(),
      info.who_am_i);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "WHO_AM_I = 0x%02X (%s), not a genuine MPU-6050 (0x68). Continuing: most clones are "
      "register compatible, but check the data before trusting it.",
      info.who_am_i, info.description.c_str());
  }

  imu_->initialize(sensor_config_);
  RCLCPP_INFO(
    get_logger(), "Configured: gyro range %d, accel range %d, DLPF %d, clock source %d, %.0f Hz",
    static_cast<int>(sensor_config_.gyro_range), static_cast<int>(sensor_config_.accel_range),
    sensor_config_.dlpf, sensor_config_.clock_source, publish_rate_);

  // --- ROS interfaces ---------------------------------------------------
  // Reliable QoS: compatible with both reliable and best-effort subscribers.
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(10));
  if (publish_temperature_) {
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      "imu/temperature", rclcpp::QoS(10));
    // Temperature changes slowly: publish it at about 1 Hz.
    temperature_divider_ = std::max<std::size_t>(1, static_cast<std::size_t>(publish_rate_));
  }
  calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/calibrate_gyro",
    std::bind(
      &Mpu6050Node::onCalibrateRequest, this, std::placeholders::_1,
      std::placeholders::_2));

  if (calibrate_on_startup_) {
    startCalibration();
  } else {
    has_bias_ = true;
    RCLCPP_INFO(
      get_logger(), "Startup calibration disabled, using gyro_bias = [%.5f, %.5f, %.5f] rad/s",
      gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));
  timer_ = create_wall_timer(period, std::bind(&Mpu6050Node::onTimer, this));
}

void Mpu6050Node::loadParameters()
{
  i2c_bus_ = declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
  declare_parameter<int>("i2c_address", 0x68);
  frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
  publish_rate_ = declare_parameter<double>("publish_rate", 100.0);

  declare_parameter<int>("gyro_range", 0);
  declare_parameter<int>("accel_range", 0);
  declare_parameter<int>("dlpf", 3);
  declare_parameter<int>("clock_source", 1);

  calibrate_on_startup_ = declare_parameter<bool>("calibrate_gyro_on_startup", true);
  calibration_duration_ = declare_parameter<double>("calibration_duration", 2.0);
  calibration_max_gyro_stddev_ = declare_parameter<double>("calibration_max_gyro_stddev", 0.02);
  declare_parameter<int>("calibration_max_attempts", 5);

  declare_parameter<std::vector<double>>("gyro_bias", {0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("accel_bias", {0.0, 0.0, 0.0});
  const double gyro_stddev = declare_parameter<double>("angular_velocity_stddev", 0.01);
  const double accel_stddev = declare_parameter<double>("linear_acceleration_stddev", 0.1);

  publish_temperature_ = declare_parameter<bool>("publish_temperature", true);
  declare_parameter<int>("max_consecutive_errors", 10);

  // --- Validation -------------------------------------------------------
  i2c_address_ = intInRange(*this, "i2c_address", 0x03, 0x77);
  sensor_config_.gyro_range = static_cast<GyroRange>(intInRange(*this, "gyro_range", 0, 3));
  sensor_config_.accel_range = static_cast<AccelRange>(intInRange(*this, "accel_range", 0, 3));
  sensor_config_.dlpf = static_cast<std::uint8_t>(intInRange(*this, "dlpf", 0, 6));
  sensor_config_.clock_source = static_cast<std::uint8_t>(intInRange(*this, "clock_source", 0, 3));
  calibration_max_attempts_ = intInRange(*this, "calibration_max_attempts", 1, 100);
  max_consecutive_errors_ = intInRange(*this, "max_consecutive_errors", 1, 10000);

  if (!(publish_rate_ > 0.0 && publish_rate_ <= 1000.0)) {
    throw std::invalid_argument("Parameter 'publish_rate' must be in (0, 1000] Hz");
  }
  if (!(calibration_duration_ > 0.0)) {
    throw std::invalid_argument("Parameter 'calibration_duration' must be > 0");
  }
  if (gyro_stddev <= 0.0 || accel_stddev <= 0.0) {
    throw std::invalid_argument(
            "Parameters 'angular_velocity_stddev' and 'linear_acceleration_stddev' "
            "must be > 0 (a zero covariance tells the EKF the sensor is perfect)");
  }
  if (frame_id_.empty()) {
    throw std::invalid_argument("Parameter 'frame_id' must not be empty");
  }

  gyro_bias_ = toArray3(get_parameter("gyro_bias").as_double_array(), "gyro_bias");
  accel_bias_ = toArray3(get_parameter("accel_bias").as_double_array(), "accel_bias");
  angular_velocity_variance_ = gyro_stddev * gyro_stddev;
  linear_acceleration_variance_ = accel_stddev * accel_stddev;
}

void Mpu6050Node::onTimer()
{
  if (needs_reinit_) {
    tryReinitialize();
    return;
  }

  const rclcpp::Time stamp = now();  // taken just before the burst read

  Sample sample;
  try {
    sample = imu_->read();
  } catch (const I2CError & e) {
    handleReadError(e);
    return;  // never publish a sample that was not read correctly
  }
  consecutive_errors_ = 0;

  if (sample.saturated) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Sensor saturated (a reading hit the end of its range). Increase gyro_range or "
      "accel_range if this happens during normal driving.");
  }

  if (state_ == State::kCalibrating) {
    calibration_stats_.add(sample.gyro);
    if (calibration_stats_.count() >= calibration_samples_needed_) {
      finishCalibration();
    }
  }

  if (has_bias_) {
    publish(sample, stamp);
  }
}

void Mpu6050Node::startCalibration()
{
  calibration_stats_.reset();
  calibration_attempt_ = 0;
  calibration_samples_needed_ = std::max<std::size_t>(
    10, static_cast<std::size_t>(std::lround(calibration_duration_ * publish_rate_)));
  state_ = State::kCalibrating;
  RCLCPP_INFO(
    get_logger(), "Calibrating gyro bias for %.1f s: keep the robot completely still...",
    calibration_duration_);
}

void Mpu6050Node::finishCalibration()
{
  const auto mean = calibration_stats_.mean();
  const double noise = calibration_stats_.maxStddev();

  if (noise <= calibration_max_gyro_stddev_) {
    gyro_bias_ = mean;
    has_bias_ = true;
    state_ = State::kRunning;
    RCLCPP_INFO(
      get_logger(),
      "Gyro calibration done: bias = [%.5f, %.5f, %.5f] rad/s, noise = %.5f rad/s",
      mean[0], mean[1], mean[2], noise);
    return;
  }

  ++calibration_attempt_;
  if (calibration_attempt_ < calibration_max_attempts_) {
    RCLCPP_WARN(
      get_logger(),
      "Robot moved during calibration (gyro stddev %.4f > %.4f rad/s). Keep it still. "
      "Retrying (%d/%d)...", noise, calibration_max_gyro_stddev_, calibration_attempt_ + 1,
      calibration_max_attempts_);
    calibration_stats_.reset();
    return;
  }

  // Give up: keep the current bias (the gyro_bias parameter at startup, or the
  // previous calibration when this was requested through the service).
  has_bias_ = true;
  state_ = State::kRunning;
  RCLCPP_WARN(
    get_logger(),
    "Gyro calibration failed after %d attempts. Using bias [%.5f, %.5f, %.5f] rad/s. "
    "Call the ~/calibrate_gyro service when the robot is still.",
    calibration_max_attempts_, gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
}

void Mpu6050Node::handleReadError(const std::exception & error)
{
  ++consecutive_errors_;
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000, "Read failed (%d in a row): %s", consecutive_errors_,
    error.what());

  if (consecutive_errors_ < max_consecutive_errors_) {
    return;
  }
  consecutive_errors_ = 0;
  needs_reinit_ = true;
  RCLCPP_ERROR(get_logger(), "Too many consecutive read errors, re-initializing the sensor");
}

void Mpu6050Node::tryReinitialize()
{
  // A reset leaves the chip asleep, so after a failed attempt we must not read
  // again until a full initialization succeeds. Retry at most once per second.
  const auto t = std::chrono::steady_clock::now();
  if (t - last_reinit_attempt_ < std::chrono::seconds(1)) {
    return;
  }
  last_reinit_attempt_ = t;
  try {
    imu_->initialize(sensor_config_);
    needs_reinit_ = false;
    RCLCPP_INFO(get_logger(), "Sensor re-initialized, publishing again");
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Re-initialization failed (will retry): %s", e.what());
  }
}

void Mpu6050Node::publish(const Sample & sample, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;

  // The MPU6050 does not provide orientation: mark it as unavailable (REP-145).
  msg.orientation.w = 1.0;
  msg.orientation_covariance[0] = -1.0;

  msg.angular_velocity.x = sample.gyro[0] - gyro_bias_[0];
  msg.angular_velocity.y = sample.gyro[1] - gyro_bias_[1];
  msg.angular_velocity.z = sample.gyro[2] - gyro_bias_[2];

  msg.linear_acceleration.x = sample.accel[0] - accel_bias_[0];
  msg.linear_acceleration.y = sample.accel[1] - accel_bias_[1];
  msg.linear_acceleration.z = sample.accel[2] - accel_bias_[2];

  for (std::size_t i : {0u, 4u, 8u}) {  // diagonal of the 3x3 matrices
    msg.angular_velocity_covariance[i] = angular_velocity_variance_;
    msg.linear_acceleration_covariance[i] = linear_acceleration_variance_;
  }
  imu_pub_->publish(msg);

  if (temp_pub_ && (sample_counter_++ % temperature_divider_) == 0) {
    sensor_msgs::msg::Temperature temp;
    temp.header = msg.header;
    temp.temperature = sample.temperature;
    temp.variance = 0.0;  // unknown
    temp_pub_->publish(temp);
  }
}

void Mpu6050Node::onCalibrateRequest(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (state_ == State::kCalibrating) {
    response->success = false;
    response->message = "A calibration is already running";
    return;
  }
  startCalibration();
  response->success = true;
  response->message = "Gyro calibration started: keep the robot still for " +
    std::to_string(calibration_duration_) + " s";
}

}  // namespace mpu6050_driver
