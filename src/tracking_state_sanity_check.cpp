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

  // ========================================================
  // TEST 3 - Incremental nearest-index search (Task 9)
  //
  // Verifies correctness (not just speed) of the bounded-window
  // search: normal progression, a trajectory replacement that makes
  // the previous index out of range, and a large localization jump -
  // both of the latter must fall back to the full search rather than
  // silently accept a wrong local candidate.
  // ========================================================
  {
    autoware_planning_msgs::msg::Trajectory long_straight;

    for (int i = 0; i < 40; ++i) {
      long_straight.points.push_back(
        makeTrajectoryPoint(
          static_cast<double>(i), 0.0, 0.0,
          5.0, 0.0));
    }

    autoware_vehicle_msgs::msg::SteeringReport steering3;
    steering3.steering_tire_angle = 0.0F;

    geometry_msgs::msg::AccelWithCovarianceStamped accel3;
    accel3.accel.accel.linear.x = 0.0;

    const auto makeOdomAt = [](const double x)
    {
      nav_msgs::msg::Odometry odom;
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = 0.0;
      odom.pose.pose.orientation = quaternionFromYaw(0.0);
      odom.twist.twist.linear.x = 5.0;
      return odom;
    };

    TrackingStateBuilder incremental_builder(
      wheel_base,
      3.0,    // nearest distance threshold
      1.046,  // nearest yaw threshold
      true,   // use_incremental_search
      5);     // incremental_search_window

    // 3a: normal progression
    const auto r1 =
      incremental_builder.build(
        long_straight, makeOdomAt(10.0), steering3, accel3);
    const auto r2 =
      incremental_builder.build(
        long_straight, makeOdomAt(11.0), steering3, accel3);

    std::cout << "\n=== INCREMENTAL SEARCH: NORMAL PROGRESSION ===\n";
    std::cout << "index at x=10 : " << (r1 ? r1->nearest_index : 999) << "\n";
    std::cout << "index at x=11 : " << (r2 ? r2->nearest_index : 999) << "\n";

    success &= r1.has_value() && r1->nearest_index == 10;
    success &= r2.has_value() && r2->nearest_index == 11;

    // 3b: trajectory replacement - the previous nearest index (11) is
    // out of range for this 5-point trajectory, so this must fall back
    // to the full search.
    autoware_planning_msgs::msg::Trajectory short_traj;

    for (int i = 0; i < 5; ++i) {
      short_traj.points.push_back(
        makeTrajectoryPoint(static_cast<double>(i), 0.0, 0.0, 5.0, 0.0));
    }

    const auto r3 =
      incremental_builder.build(
        short_traj, makeOdomAt(2.0), steering3, accel3);

    std::cout << "\n=== INCREMENTAL SEARCH: TRAJECTORY REPLACEMENT ===\n";
    std::cout << "index on 5-point trajectory at x=2 : "
      << (r3 ? r3->nearest_index : 999) << "\n";

    success &= r3.has_value() && r3->nearest_index == 2;

    // 3c: large localization jump on the ORIGINAL long trajectory - the
    // window around the previous index (5 +/- 5) does not reach x=35,
    // so this must fall back to the full search rather than accept
    // nothing / a wrong candidate.
    TrackingStateBuilder jump_builder(
      wheel_base, 3.0, 1.046, true, 5);

    const auto r4 =
      jump_builder.build(
        long_straight, makeOdomAt(5.0), steering3, accel3);
    const auto r5 =
      jump_builder.build(
        long_straight, makeOdomAt(35.0), steering3, accel3);

    std::cout << "\n=== INCREMENTAL SEARCH: LOCALIZATION JUMP ===\n";
    std::cout << "index at x=5  : " << (r4 ? r4->nearest_index : 999) << "\n";
    std::cout << "index at x=35 : " << (r5 ? r5->nearest_index : 999) << "\n";

    success &= r4.has_value() && r4->nearest_index == 5;
    success &= r5.has_value() && r5->nearest_index == 35;
  }

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
