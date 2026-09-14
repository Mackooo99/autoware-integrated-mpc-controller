#include "autoware/integrated_mpc_trajectory_follower/prediction_reference_builder.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>

using autoware::integrated_mpc::PredictionReferenceBuilder;

namespace
{


autoware_planning_msgs::msg::TrajectoryPoint
makePoint(
  const double x,
  const double y,
  const double velocity,
  const double acceleration)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;

  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;

  point.pose.orientation.w = 1.0;

  point.longitudinal_velocity_mps =
    static_cast<float>(velocity);

  point.acceleration_mps2 =
    static_cast<float>(acceleration);

  return point;
}



bool approximatelyEqual(
  const double value,
  const double expected,
  const double tolerance = 1.0e-6)
{
  return std::abs(
    value - expected) < tolerance;
}

}  // namespace


int main()
{
  bool success = true;

  constexpr double wheel_base = 2.79;

  // ==========================================================
  // TEST 1:
  // Constant speed: 5 m/s
  //
  // Spatial point spacing = 0.1 m
  //
  // Time spacing:
  // 0.1 / 5 = 0.02 s
  //
  // N = 5
  // minimum length = 2.5 m
  //
  // Time to cover 2.5 m:
  // 2.5 / 5 = 0.5 s
  //
  // dt = 0.5 / 5 = 0.1 s
  // ==========================================================

  autoware_planning_msgs::msg::Trajectory trajectory;

  for (std::size_t i = 0; i <= 100; ++i) {
    const double x =
      0.1 *
      static_cast<double>(i);

    const double time =
      x / 5.0;

    trajectory.points.push_back(
      makePoint(
        x,
        0.0,
        5.0,
        0.2));
  }

  PredictionReferenceBuilder builder(
    wheel_base,
    5,
    0.1,
    2.5,
    0.1);

  const auto horizon =
    builder.build(
      trajectory,
      10);

  std::cout
    << std::fixed
    << std::setprecision(6);

  std::cout
    << "\n=== CONSTANT SPEED HORIZON ===\n";

  std::cout
    << "prediction_dt = "
    << horizon.prediction_dt
    << "\n";

  std::cout
    << "size          = "
    << horizon.points.size()
    << "\n\n";

  std::cout
    << "step  index  s[m]      v_ref      a_ref      kappa\n";

  for (
    std::size_t i = 0;
    i < horizon.points.size();
    ++i)
  {
    const auto & ref =
      horizon.points.at(i);

    std::cout
      << std::setw(4) << i << "  "
      << std::setw(5) << ref.trajectory_index << "  "
      << std::setw(8) << ref.arc_length << "  "
      << std::setw(8) << ref.velocity << "  "
      << std::setw(8) << ref.acceleration << "  "
      << std::setw(8) << ref.curvature
      << "\n";
  }

  success &=
    horizon.points.size() == 5;

  success &=
    approximatelyEqual(
      horizon.prediction_dt,
      0.1);

  const std::size_t expected_indices[] =
    {15, 20, 25, 30, 35};

  for (std::size_t i = 0; i < 5; ++i) {
    success &=
      horizon.points.at(i).
        trajectory_index ==
      expected_indices[i];
  }

  success &=
    approximatelyEqual(
      horizon.total_arc_length,
      2.5);


  // ==========================================================
  // TEST 2:
  // Constant LOW speed
  //
  // v = 0.25 m/s
  //
  // Time to cover 5 m:
  // 20 seconds
  //
  // N = 50
  //
  // dt = 20 / 50 = 0.4 s
  // ==========================================================

  autoware_planning_msgs::msg::Trajectory slow_trajectory;

  for (std::size_t i = 0; i <= 100; ++i) {
    const double x =
      0.1 *
      static_cast<double>(i);

    const double time =
      x / 0.25;

    slow_trajectory.points.push_back(
      makePoint(
        x,
        0.0,
        0.25,
        0.0));
  }

  PredictionReferenceBuilder slow_builder(
    wheel_base,
    50,
    0.1,
    5.0,
    0.1);

  const auto slow_horizon =
    slow_builder.build(
      slow_trajectory,
      0);

  std::cout
    << "\n=== CONSTANT LOW SPEED HORIZON ===\n";

  std::cout
    << "prediction_dt = "
    << slow_horizon.prediction_dt
    << "\n";

  std::cout
    << "expected dt   = 0.400000\n";

  std::cout
    << "first index   = "
    << slow_horizon.points.front().
      trajectory_index
    << "\n";

  std::cout
    << "last index    = "
    << slow_horizon.points.back().
      trajectory_index
    << "\n";

  std::cout
    << "total length  = "
    << slow_horizon.total_arc_length
    << " m\n";

  success &=
    slow_horizon.points.size() == 50;

  success &=
    approximatelyEqual(
      slow_horizon.prediction_dt,
      0.4);

  success &=
    approximatelyEqual(
      slow_horizon.total_arc_length,
      5.0);


  // ==========================================================
  // TEST 3:
  // Accelerating trajectory
  //
  // v0 = 0.25 m/s
  // a  = 0.5 m/s^2
  //
  // x(t) = 0.25*t + 0.25*t^2
  //
  // x(4 s) = 5 m
  //
  // dt_length = 4 / 50 = 0.08 s
  //
  // nominal dt = 0.1 s
  //
  // therefore:
  //
  // prediction_dt = 0.1 s
  // ==========================================================

  autoware_planning_msgs::msg::Trajectory accelerating_trajectory;

  constexpr double synthetic_dt =
    0.1;

  constexpr std::size_t synthetic_points =
    101;

  for (
    std::size_t i = 0;
    i < synthetic_points;
    ++i)
  {
    const double t =
      synthetic_dt *
      static_cast<double>(i);

    const double velocity =
      0.25 +
      0.5 *
      t;

    const double x =
      0.25 *
      t +
      0.25 *
      t *
      t;

    accelerating_trajectory.points.push_back(
      makePoint(
        x,
        0.0,
        velocity,
        0.5));
  }

  PredictionReferenceBuilder accelerating_builder(
    wheel_base,
    50,
    0.1,
    5.0,
    0.1);

  const auto accelerating_horizon =
    accelerating_builder.build(
      accelerating_trajectory,
      0);

  std::cout
    << "\n=== ACCELERATING HORIZON ===\n";

  std::cout
    << "prediction_dt = "
    << accelerating_horizon.prediction_dt
    << "\n";

  std::cout
    << "expected dt   = 0.100000\n";

  std::cout
    << "first s       = "
    << accelerating_horizon.points.front().
      arc_length
    << " m\n";

  std::cout
    << "last s        = "
    << accelerating_horizon.points.back().
      arc_length
    << " m\n";

  std::cout
    << "last v_ref    = "
    << accelerating_horizon.points.back().
      velocity
    << " m/s\n";

  /*
   * This is the important regression test.
   *
   * OLD implementation would use v0=0.25 m/s and produce:
   *
   * prediction_dt = 0.4 s
   *
   * NEW implementation sees the acceleration profile and
   * produces nominal:
   *
   * prediction_dt = 0.1 s
   */
  success &=
    accelerating_horizon.points.size() ==
    50;

  success &=
    approximatelyEqual(
      accelerating_horizon.prediction_dt,
      0.1);

  /*
   * At t=5 s:
   *
   * x = 0.25*5 + 0.25*25
   *   = 7.5 m
   */
  success &=
    approximatelyEqual(
      accelerating_horizon.total_arc_length,
      7.5,
      1.0e-4);


  // ==========================================================
  // RESULT
  // ==========================================================

  std::cout
    << "\n====================================\n";

  if (success) {
    std::cout
      << "PREDICTION REFERENCE SANITY CHECK: PASS\n";

    std::cout
      << "====================================\n";

    return 0;
  }

  std::cout
    << "PREDICTION REFERENCE SANITY CHECK: FAIL\n";

  std::cout
    << "====================================\n";

  return 1;
}
