cmake_minimum_required(VERSION 3.8)
project(ur5e_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the executable
add_executable(move_arm src/move_arm.cpp)

# Specify dependencies for the target
ament_target_dependencies(move_arm
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
)

# Install the target
install(TARGETS
  move_arm
  DESTINATION lib/${PROJECT_NAME})

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)  # Skip cpplint if desired
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
