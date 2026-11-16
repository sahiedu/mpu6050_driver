// Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).

#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mpu6050_driver/mpu6050_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<mpu6050_driver::Mpu6050Node>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mpu6050"), "%s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
