cmake_minimum_required(VERSION 3.5)
project(phytium_car_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(uart_bridge_cpp src/uart_bridge_cpp.cpp)
add_executable(d6_lidar_cpp src/d6_lidar_cpp.cpp)
add_executable(safety_guard_cpp src/safety_guard_cpp.cpp)
add_executable(cmd_vel_mux_cpp src/cmd_vel_mux_cpp.cpp)

ament_target_dependencies(uart_bridge_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(d6_lidar_cpp
  rclcpp
  sensor_msgs
)

install(TARGETS
  cmd_vel_mux_cpp
  safety_guard_cpp
  uart_bridge_cpp
  d6_lidar_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

ament_target_dependencies(safety_guard_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)


ament_target_dependencies(cmd_vel_mux_cpp
  rclcpp
  geometry_msgs
  std_msgs
)

