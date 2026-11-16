cmake_minimum_required(VERSION 3.16)
project(mpu6050_driver LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# libi2c: give a clear message instead of "i2c/smbus.h: No such file".
find_path(I2C_INCLUDE_DIR i2c/smbus.h)
find_library(I2C_LIBRARY NAMES i2c)
if(NOT I2C_INCLUDE_DIR OR NOT I2C_LIBRARY)
  message(FATAL_ERROR
    "libi2c not found. Install it with:  sudo apt install libi2c-dev")
endif()

# ---- Hardware driver (no ROS dependency) -----------------------------------
add_library(mpu6050_core STATIC
  src/i2c_bus.cpp
  src/mpu6050.cpp
)
set_target_properties(mpu6050_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mpu6050_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${I2C_INCLUDE_DIR}
)
target_link_libraries(mpu6050_core PUBLIC ${I2C_LIBRARY})

# ---- ROS node ---------------------------------------------------------------
add_library(mpu6050_node_lib STATIC src/mpu6050_node.cpp)
set_target_properties(mpu6050_node_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(mpu6050_node_lib PUBLIC
  mpu6050_core
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

add_executable(mpu6050_node src/mpu6050_node_main.cpp)
target_link_libraries(mpu6050_node PRIVATE mpu6050_node_lib)

# ---- Calibration tool (no ROS needed at runtime) ---------------------------
add_executable(mpu6050_calibrate src/calibrate_main.cpp)
target_link_libraries(mpu6050_calibrate PRIVATE mpu6050_core)

# ---- Install ----------------------------------------------------------------
install(TARGETS mpu6050_node mpu6050_calibrate
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()
