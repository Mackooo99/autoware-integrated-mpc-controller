cmake_minimum_required(VERSION 3.14)
project(autoware_integrated_mpc_trajectory_follower)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(integrated_mpc_node
  src/integrated_mpc_node.cpp
)

ament_target_dependencies(integrated_mpc_node
  rclcpp
  autoware_adapi_v1_msgs
  autoware_control_msgs
  autoware_planning_msgs
  autoware_vehicle_msgs
  geometry_msgs
  nav_msgs
)

install(
  TARGETS integrated_mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
