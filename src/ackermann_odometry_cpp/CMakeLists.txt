cmake_minimum_required(VERSION 3.8)
project(ackermann_odometry_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(ackermann_odometry src/ackermann_odometry_node.cpp)
target_compile_features(ackermann_odometry PUBLIC cxx_std_17)
ament_target_dependencies(ackermann_odometry
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros)
install(TARGETS ackermann_odometry DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(odometry_integration test/test_odometry.py
    TIMEOUT 60
    ENV ODOMETRY_EXECUTABLE=$<TARGET_FILE:ackermann_odometry>)
endif()

ament_package()
