cmake_minimum_required(VERSION 3.14)

project(autoware_integrated_mpc_trajectory_follower)

# Match stock Autoware's default build type so timing comparisons are valid
# (stock packages are built -O3 -DNDEBUG; this package previously had no
# CMAKE_BUILD_TYPE and therefore no optimisation flags at all).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(autoware_osqp_interface REQUIRED)
find_package(autoware_internal_debug_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# ============================================================
# Integrated MPC vehicle model
# ============================================================

add_library(
  integrated_mpc_model
  SHARED
  src/integrated_kinematics_bicycle_model.cpp
)

target_compile_features(
  integrated_mpc_model
  PUBLIC
  cxx_std_17
)

target_include_directories(
  integrated_mpc_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  integrated_mpc_model
  Eigen3::Eigen
)


# ============================================================
# Extended prediction matrices
# ============================================================

add_library(
  integrated_mpc_prediction
  SHARED
  src/prediction_matrix_builder.cpp
)

target_compile_features(
  integrated_mpc_prediction
  PUBLIC
  cxx_std_17
)

target_include_directories(
  integrated_mpc_prediction
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  integrated_mpc_prediction
  Eigen3::Eigen
)

# ============================================================
# Tracking state / trajectory geometry
# ============================================================

add_library(
  integrated_mpc_tracking
  SHARED
  src/tracking_state_builder.cpp
)

target_compile_features(
  integrated_mpc_tracking
  PUBLIC
  cxx_std_17
)

target_include_directories(
  integrated_mpc_tracking
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  integrated_mpc_tracking
  autoware_planning_msgs
  autoware_vehicle_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(
  integrated_mpc_tracking
  integrated_mpc_model
)

# ============================================================
# Prediction reference horizon
# ============================================================

add_library(
  integrated_mpc_reference
  SHARED
  src/prediction_reference_builder.cpp
)

target_compile_features(
  integrated_mpc_reference
  PUBLIC
  cxx_std_17
)

target_include_directories(
  integrated_mpc_reference
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  integrated_mpc_reference
  autoware_planning_msgs
)

target_link_libraries(
  integrated_mpc_reference
  integrated_mpc_tracking
  integrated_mpc_model
)


# ============================================================
# Integrated LTV prediction model
# ============================================================

add_library(
  integrated_mpc_integrated_prediction
  SHARED
  src/integrated_prediction_builder.cpp
)

target_compile_features(
  integrated_mpc_integrated_prediction
  PUBLIC
  cxx_std_17
)

target_include_directories(
  integrated_mpc_integrated_prediction
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  integrated_mpc_integrated_prediction
  integrated_mpc_model
  integrated_mpc_reference
  integrated_mpc_prediction
  Eigen3::Eigen
)


# ============================================================
# MPC cost builder
# ============================================================

add_library(
  integrated_mpc_cost
  SHARED
  src/mpc_cost_builder.cpp
)

target_compile_features(
  integrated_mpc_cost
  PUBLIC
  cxx_std_17
)

target_include_directories(
  integrated_mpc_cost
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  integrated_mpc_cost
  autoware_planning_msgs
)

target_link_libraries(
  integrated_mpc_cost
  integrated_mpc_reference
  integrated_mpc_prediction
  Eigen3::Eigen
)


# ============================================================
# MPC constraint builder
# ============================================================

add_library(
  integrated_mpc_constraints
  SHARED
  src/mpc_constraint_builder.cpp
)

target_compile_features(
  integrated_mpc_constraints
  PUBLIC
  cxx_std_17
)

target_include_directories(
  integrated_mpc_constraints
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  integrated_mpc_constraints
  autoware_planning_msgs
)

target_link_libraries(
  integrated_mpc_constraints
  integrated_mpc_prediction
  integrated_mpc_reference
  Eigen3::Eigen
)


# ============================================================
# ROS node
# ============================================================

add_executable(
  integrated_mpc_node
  src/integrated_mpc_node.cpp
)

target_compile_features(
  integrated_mpc_node
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  integrated_mpc_node
  rclcpp
  autoware_adapi_v1_msgs
  autoware_control_msgs
  autoware_planning_msgs
  autoware_vehicle_msgs
  geometry_msgs
  nav_msgs
  autoware_osqp_interface
  autoware_internal_debug_msgs
  diagnostic_updater
)

target_link_libraries(
  integrated_mpc_node
  integrated_mpc_model
  integrated_mpc_tracking
  integrated_mpc_reference
  integrated_mpc_prediction
  integrated_mpc_integrated_prediction
  integrated_mpc_cost
  integrated_mpc_constraints
)

# ============================================================
# Model sanity test
# ============================================================

add_executable(
  model_sanity_check
  src/model_sanity_check.cpp
)

target_compile_features(
  model_sanity_check
  PUBLIC
  cxx_std_17
)

target_link_libraries(
  model_sanity_check
  integrated_mpc_model
)

# ============================================================
# Tracking state sanity test
# ============================================================

add_executable(
  tracking_state_sanity_check
  src/tracking_state_sanity_check.cpp
)

target_compile_features(
  tracking_state_sanity_check
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  tracking_state_sanity_check
  autoware_planning_msgs
  autoware_vehicle_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(
  tracking_state_sanity_check
  integrated_mpc_tracking
  integrated_mpc_model
)



# ============================================================
# Prediction reference sanity test
# ============================================================

add_executable(
  prediction_reference_sanity_check
  src/prediction_reference_sanity_check.cpp
)

target_compile_features(
  prediction_reference_sanity_check
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  prediction_reference_sanity_check
  autoware_planning_msgs
)

target_link_libraries(
  prediction_reference_sanity_check
  integrated_mpc_reference
  integrated_mpc_tracking
  integrated_mpc_model
)

# ============================================================
# Prediction matrix sanity test
# ============================================================

add_executable(
  prediction_matrix_sanity_check
  src/prediction_matrix_sanity_check.cpp
)

target_compile_features(
  prediction_matrix_sanity_check
  PUBLIC
  cxx_std_17
)

target_link_libraries(
  prediction_matrix_sanity_check
  integrated_mpc_prediction
  Eigen3::Eigen
)


# ============================================================
# MPC cost sanity test
# ============================================================

add_executable(
  mpc_cost_sanity_check
  src/mpc_cost_sanity_check.cpp
)

target_compile_features(
  mpc_cost_sanity_check
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  mpc_cost_sanity_check
  autoware_planning_msgs
)

target_link_libraries(
  mpc_cost_sanity_check
  integrated_mpc_cost
  integrated_mpc_prediction
  integrated_mpc_reference
  Eigen3::Eigen
)


# ============================================================
# MPC constraint sanity test
# ============================================================

add_executable(
  mpc_constraint_sanity_check
  src/mpc_constraint_sanity_check.cpp
)

target_compile_features(
  mpc_constraint_sanity_check
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  mpc_constraint_sanity_check
  autoware_planning_msgs
)

target_link_libraries(
  mpc_constraint_sanity_check
  integrated_mpc_constraints
  integrated_mpc_prediction
  integrated_mpc_reference
  Eigen3::Eigen
)


# ============================================================
# OSQP sanity test
# ============================================================

add_executable(
  osqp_sanity_check
  src/osqp_sanity_check.cpp
)

target_compile_features(
  osqp_sanity_check
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  osqp_sanity_check
  autoware_osqp_interface
)

target_link_libraries(
  osqp_sanity_check
  Eigen3::Eigen
)


# ============================================================
# Install libraries
# ============================================================

install(
  TARGETS
    integrated_mpc_model
    integrated_mpc_tracking
    integrated_mpc_reference
    integrated_mpc_prediction
    integrated_mpc_integrated_prediction
    integrated_mpc_cost
    integrated_mpc_constraints
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ============================================================
# Install executables
# ============================================================

install(
  TARGETS
    integrated_mpc_node
    model_sanity_check
    prediction_reference_sanity_check
    tracking_state_sanity_check
    prediction_matrix_sanity_check
    mpc_cost_sanity_check
    mpc_constraint_sanity_check
    osqp_sanity_check
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Install headers / launch / config
# ============================================================

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
