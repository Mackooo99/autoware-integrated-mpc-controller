#include "autoware/integrated_mpc_trajectory_follower/tracking_state_builder.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>

using autoware::integrated_mpc::TrackingStateBuilder;

namespace
{

geometry_msgs::msg::Quaternion quaternionFromYaw(
  const double yaw)
{
  geometry_msgs::msg::Quaternion q;

  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);

  return q;
}

bool approximatelyEqual(
  const double value,
  const double expected,
  const double tolerance = 1.0e-6)
{
  return std::abs(value - expected) <
    tolerance;
}

autoware_planning_msgs::msg::TrajectoryPoint
makeTrajectoryPoint(
  const double x,
  const double y,
  const double yaw,
  const double velocity,
  const double acceleration)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;

  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;

  point.pose.orientation =
    quaternionFromYaw(yaw);

  point.longitudinal_velocity_mps =
    static_cast<float>(velocity);

  point.acceleration_mps2 =
    static_cast<float>(acceleration);

  return point;
}

}  // namespace

int main()
{
  constexpr double wheel_base = 2.79;

  TrackingStateBuilder builder(
    wheel_base,
    5.0,   // nearest distance threshold
    1.046  // nearest yaw threshold
  );

  bool success = true;

  // ========================================================
  // TEST 1 - Straight trajectory
  // ========================================================

  autoware_planning_msgs::msg::Trajectory trajectory;

  trajectory.points.push_back(
    makeTrajectoryPoint(
      0.0, 0.0, 0.0,
      5.0, 0.2));

  trajectory.points.push_back(
    makeTrajectoryPoint(
      1.0, 0.0, 0.0,
      5.0, 0.2));

  trajectory.points.push_back(
    makeTrajectoryPoint(
      2.0, 0.0, 0.0,
      5.0, 0.2));

  nav_msgs::msg::Odometry odometry;

  /*
   * Ego vehicle:
   *
   * x = 1
   * y = +1
   *
   * Reference points in +X direction,
   * therefore ego is 1 m LEFT of path.
   */
  odometry.pose.pose.position.x = 1.0;
  odometry.pose.pose.position.y = 1.0;

  odometry.pose.pose.orientation =
    quaternionFromYaw(0.1);

  odometry.twist.twist.linear.x =
    6.0;

  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.steering_tire_angle =
    0.05F;

  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  acceleration.accel.accel.linear.x =
    0.3;

  const auto result =
    builder.build(
      trajectory,
      odometry,
      steering,
      acceleration);

  if (!result) {
    std::cerr
      << "ERROR: Could not find nearest trajectory point\n";

    return 1;
  }

  std::cout
    << std::fixed
    << std::setprecision(8);

  std::cout << "\n=== STRAIGHT TRACKING TEST ===\n";

  std::cout
    << "nearest index : "
    << result->nearest_index << "\n";

  std::cout
    << "e_y           : "
    << result->lateral_error << "\n";

  std::cout
    << "e_psi         : "
    << result->heading_error << "\n";

  std::cout
    << "e_v           : "
    << result->velocity_error << "\n";

  std::cout
    << "delta         : "
    << result->current_steer << "\n";

  std::cout
    << "a             : "
    << result->current_acceleration << "\n";

  std::cout
    << "v_ref         : "
    << result->reference_velocity << "\n";

  std::cout
    << "a_ref         : "
    << result->reference_acceleration << "\n";

  std::cout
    << "kappa_ref     : "
    << result->reference_curvature << "\n";

  std::cout
    << "\nx0:\n"
    << result->x0 << "\n";

  success &=
    result->nearest_index == 1;

  success &=
    approximatelyEqual(
      result->lateral_error,
      1.0);

  success &=
    approximatelyEqual(
      result->heading_error,
      0.1);

  success &=
    approximatelyEqual(
      result->velocity_error,
      1.0);

  success &=
    approximatelyEqual(
      result->current_steer,
      0.05,
      1.0e-5);

  success &=
    approximatelyEqual(
      result->current_acceleration,
      0.3);

  success &=
    approximatelyEqual(
      result->reference_velocity,
      5.0);

  success &=
    approximatelyEqual(
      result->reference_acceleration,
      0.2,
      1.0e-5);

  success &=
    approximatelyEqual(
      result->reference_curvature,
      0.0);

  // ========================================================
  // TEST 2 - Curvature
  // ========================================================

  autoware_planning_msgs::msg::Trajectory curved;

  constexpr double radius = 10.0;
  constexpr double theta = 0.1;

  curved.points.push_back(
    makeTrajectoryPoint(
      radius * std::cos(-theta),
      radius * std::sin(-theta),
      0.0,
      5.0,
      0.0));

  curved.points.push_back(
    makeTrajectoryPoint(
      radius,
      0.0,
      0.0,
      5.0,
      0.0));

  curved.points.push_back(
    makeTrajectoryPoint(
      radius * std::cos(theta),
      radius * std::sin(theta),
      0.0,
      5.0,
      0.0));

  const double curvature =
    builder.calculateCurvature(
      curved,
      1);

  std::cout << "\n=== CURVATURE TEST ===\n";

  std::cout
    << "calculated curvature : "
    << curvature << "\n";

  std::cout
    << "expected curvature   : "
    << 1.0 / radius << "\n";

  success &=
    approximatelyEqual(
      curvature,
      1.0 / radius,
      1.0e-6);

  std::cout
    << "\n====================================\n";

  if (success) {
    std::cout
      << "TRACKING STATE SANITY CHECK: PASS\n";

    std::cout
      << "====================================\n";

    return 0;
  }

  std::cout
    << "TRACKING STATE SANITY CHECK: FAIL\n";

  std::cout
    << "====================================\n";

  return 1;
}
