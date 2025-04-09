cmake_minimum_required(VERSION 3.8)
project(robot_chase)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable
add_executable(robot_chase src/robot_chase.cpp)
ament_target_dependencies(robot_chase rclcpp geometry_msgs tf2 tf2_ros)

# Install executable
install(TARGETS
  robot_chase
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()