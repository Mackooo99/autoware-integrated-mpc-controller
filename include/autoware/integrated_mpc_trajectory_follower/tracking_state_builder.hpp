#pragma once

#include "autoware/integrated_mpc_trajectory_follower/integrated_kinematics_bicycle_model.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstddef>
#include <optional>

namespace autoware::integrated_mpc
{

struct TrackingState
{
  IntegratedKinematicsBicycleModel::VectorX x0{
    IntegratedKinematicsBicycleModel::VectorX::Zero()};

  std::size_t nearest_index{0};

  double reference_velocity{0.0};
  double reference_acceleration{0.0};
  double reference_curvature{0.0};
  double reference_steer{0.0};

  double current_velocity{0.0};
  double current_acceleration{0.0};
  double current_steer{0.0};

  double lateral_error{0.0};
  double heading_error{0.0};
  double velocity_error{0.0};
};

class TrackingStateBuilder
{
public:
  TrackingStateBuilder(
    double wheel_base,
    double nearest_dist_threshold,
    double nearest_yaw_threshold,
    bool use_incremental_search = false,
    std::size_t incremental_search_window = 15,
    std::size_t curvature_smoothing_stride = 1);

  std::optional<TrackingState> build(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const nav_msgs::msg::Odometry & odometry,
    const autoware_vehicle_msgs::msg::SteeringReport & steering,
    const geometry_msgs::msg::AccelWithCovarianceStamped & acceleration);

  // stride = 1 reproduces V1 exactly (uses the immediately adjacent
  // points index-1/index+1). stride > 1 widens the triangle to
  // index-stride/index+stride, analogous in purpose to stock's
  // curvature_smoothing_num_traj: less sensitive to per-point noise,
  // at the cost of some averaging over a longer arc.
  static double calculateCurvature(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    std::size_t index,
    std::size_t stride = 1);

private:
  std::optional<std::size_t> findNearestIndex(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const nav_msgs::msg::Odometry & odometry) const;

  // Searches only [previous_index - window, previous_index + window]
  // (clamped) using the same distance/yaw acceptance criteria as
  // findNearestIndex. Returns nullopt if no candidate in the window is
  // accepted, or if previous_index is out of range for `trajectory` -
  // in both cases the caller falls back to the full findNearestIndex.
  std::optional<std::size_t> findNearestIndexIncremental(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const nav_msgs::msg::Odometry & odometry,
    std::size_t previous_index) const;

  static double yawFromPose(
    const geometry_msgs::msg::Pose & pose);

  static double normalizeAngle(double angle);

  double wheel_base_;
  double nearest_dist_threshold_;
  double nearest_yaw_threshold_;

  bool use_incremental_search_;
  std::size_t incremental_search_window_;
  std::optional<std::size_t> previous_nearest_index_;

  std::size_t curvature_smoothing_stride_;
};

}  // namespace autoware::integrated_mpc
