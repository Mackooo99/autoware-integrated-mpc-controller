#include "autoware/integrated_mpc_trajectory_follower/tracking_state_builder.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace autoware::integrated_mpc
{

TrackingStateBuilder::TrackingStateBuilder(
  const double wheel_base,
  const double nearest_dist_threshold,
  const double nearest_yaw_threshold)
: wheel_base_(wheel_base),
  nearest_dist_threshold_(nearest_dist_threshold),
  nearest_yaw_threshold_(nearest_yaw_threshold)
{
  if (wheel_base_ <= 0.0) {
    throw std::invalid_argument("wheel_base must be positive");
  }

  if (nearest_dist_threshold_ <= 0.0) {
    throw std::invalid_argument(
      "nearest_dist_threshold must be positive");
  }

  if (nearest_yaw_threshold_ <= 0.0) {
    throw std::invalid_argument(
      "nearest_yaw_threshold must be positive");
  }
}

double TrackingStateBuilder::normalizeAngle(const double angle)
{
  return std::atan2(
    std::sin(angle),
    std::cos(angle));
}

double TrackingStateBuilder::yawFromPose(
  const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;

  const double sin_yaw =
    2.0 * (q.w * q.z + q.x * q.y);

  const double cos_yaw =
    1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  return std::atan2(
    sin_yaw,
    cos_yaw);
}

std::optional<std::size_t>
TrackingStateBuilder::findNearestIndex(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const nav_msgs::msg::Odometry & odometry) const
{
  if (trajectory.points.empty()) {
    return std::nullopt;
  }

  const auto & ego_pose =
    odometry.pose.pose;

  const double ego_yaw =
    yawFromPose(ego_pose);

  double best_squared_distance =
    std::numeric_limits<double>::max();

  std::optional<std::size_t> best_index;

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & ref_pose =
      trajectory.points.at(i).pose;

    const double dx =
      ego_pose.position.x -
      ref_pose.position.x;

    const double dy =
      ego_pose.position.y -
      ref_pose.position.y;

    const double squared_distance =
      dx * dx + dy * dy;

    if (
      squared_distance >
      nearest_dist_threshold_ *
      nearest_dist_threshold_)
    {
      continue;
    }

    const double ref_yaw =
      yawFromPose(ref_pose);

    const double yaw_error =
      std::abs(
        normalizeAngle(
          ego_yaw - ref_yaw));

    if (yaw_error > nearest_yaw_threshold_) {
      continue;
    }

    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      best_index = i;
    }
  }

  return best_index;
}

double TrackingStateBuilder::calculateCurvature(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::size_t index) 
{
  const auto n =
    trajectory.points.size();

  if (n < 3) {
    return 0.0;
  }

  std::size_t i0;
  std::size_t i1;
  std::size_t i2;

  if (index == 0) {
    i0 = 0;
    i1 = 1;
    i2 = 2;
  } else if (index >= n - 1) {
    i0 = n - 3;
    i1 = n - 2;
    i2 = n - 1;
  } else {
    i0 = index - 1;
    i1 = index;
    i2 = index + 1;
  }

  const auto & p0 =
    trajectory.points.at(i0).pose.position;

  const auto & p1 =
    trajectory.points.at(i1).pose.position;

  const auto & p2 =
    trajectory.points.at(i2).pose.position;

  const double v01_x = p1.x - p0.x;
  const double v01_y = p1.y - p0.y;

  const double v02_x = p2.x - p0.x;
  const double v02_y = p2.y - p0.y;

  const double cross =
    v01_x * v02_y -
    v01_y * v02_x;

  const double a =
    std::hypot(
      p1.x - p0.x,
      p1.y - p0.y);

  const double b =
    std::hypot(
      p2.x - p1.x,
      p2.y - p1.y);

  const double c =
    std::hypot(
      p2.x - p0.x,
      p2.y - p0.y);

  const double denominator =
    a * b * c;

  constexpr double epsilon = 1.0e-9;

  if (denominator < epsilon) {
    return 0.0;
  }

  /*
   * Signed curvature of the circumcircle:
   *
   * kappa = 2 * cross / (a*b*c)
   *
   * Positive -> left turn
   * Negative -> right turn
   */
  return 2.0 * cross / denominator;
}

std::optional<TrackingState>
TrackingStateBuilder::build(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const nav_msgs::msg::Odometry & odometry,
  const autoware_vehicle_msgs::msg::SteeringReport & steering,
  const geometry_msgs::msg::AccelWithCovarianceStamped & acceleration) const
{
  const auto nearest_index =
    findNearestIndex(
      trajectory,
      odometry);

  if (!nearest_index) {
    return std::nullopt;
  }

  const std::size_t index =
    nearest_index.value();

  const auto & reference_point =
    trajectory.points.at(index);

  const auto & ego_pose =
    odometry.pose.pose;

  const auto & reference_pose =
    reference_point.pose;

  const double ego_yaw =
    yawFromPose(ego_pose);

  const double reference_yaw =
    yawFromPose(reference_pose);

  const double dx =
    ego_pose.position.x -
    reference_pose.position.x;

  const double dy =
    ego_pose.position.y -
    reference_pose.position.y;

  /*
   * Lateral error expressed in the
   * reference-path coordinate frame.
   *
   * Positive = left of the reference path.
   */
  const double lateral_error =
    -std::sin(reference_yaw) * dx +
     std::cos(reference_yaw) * dy;

  const double heading_error =
    normalizeAngle(
      ego_yaw - reference_yaw);

  const double current_velocity =
    odometry.twist.twist.linear.x;

  const double reference_velocity =
    reference_point.longitudinal_velocity_mps;

  /*
   * IMPORTANT:
   *
   * Our model uses:
   *
   * e_v = v - v_ref
   */
  const double velocity_error =
    current_velocity -
    reference_velocity;

  const double current_steer =
    steering.steering_tire_angle;

  const double current_acceleration =
    acceleration.accel.accel.linear.x;

  const double reference_acceleration =
    reference_point.acceleration_mps2;

  const double reference_curvature =
    calculateCurvature(
      trajectory,
      index);

  const double reference_steer =
    std::atan(
      wheel_base_ *
      reference_curvature);

  TrackingState result;

  result.nearest_index =
    index;

  result.reference_velocity =
    reference_velocity;

  result.reference_acceleration =
    reference_acceleration;

  result.reference_curvature =
    reference_curvature;

  result.reference_steer =
    reference_steer;

  result.current_velocity =
    current_velocity;

  result.current_acceleration =
    current_acceleration;

  result.current_steer =
    current_steer;

  result.lateral_error =
    lateral_error;

  result.heading_error =
    heading_error;

  result.velocity_error =
    velocity_error;

  /*
   * x0 =
   *
   * [ e_y,
   *   e_psi,
   *   e_v,
   *   delta,
   *   a ]
   */
  result.x0 <<
    lateral_error,
    heading_error,
    velocity_error,
    current_steer,
    current_acceleration;

  return result;
}

}  // namespace autoware::integrated_mpc
