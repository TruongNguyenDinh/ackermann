cmake_minimum_required(VERSION 3.8)
project(ackermann_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ackerman_kinematic src/ackermann_kinematics_node.cpp)
target_compile_features(ackerman_kinematic PUBLIC cxx_std_17)
ament_target_dependencies(ackerman_kinematic
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS ackerman_kinematic
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
