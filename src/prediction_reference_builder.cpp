#include "autoware/integrated_mpc_trajectory_follower/prediction_reference_builder.hpp"

#include "autoware/integrated_mpc_trajectory_follower/tracking_state_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoware::integrated_mpc
{

PredictionReferenceBuilder::PredictionReferenceBuilder(
  const double wheel_base,
  const std::size_t prediction_horizon,
  const double nominal_prediction_dt,
  const double min_prediction_length,
  const double minimum_prediction_velocity)
: wheel_base_(wheel_base),
  prediction_horizon_(prediction_horizon),
  nominal_prediction_dt_(nominal_prediction_dt),
  min_prediction_length_(min_prediction_length),
  minimum_prediction_velocity_(minimum_prediction_velocity)
{
  if (wheel_base_ <= 0.0) {
    throw std::invalid_argument(
      "wheel_base must be positive");
  }

  if (prediction_horizon_ == 0) {
    throw std::invalid_argument(
      "prediction_horizon must be greater than zero");
  }

  if (nominal_prediction_dt_ <= 0.0) {
    throw std::invalid_argument(
      "nominal_prediction_dt must be positive");
  }

  if (min_prediction_length_ < 0.0) {
    throw std::invalid_argument(
      "min_prediction_length must not be negative");
  }

  if (minimum_prediction_velocity_ <= 0.0) {
    throw std::invalid_argument(
      "minimum_prediction_velocity must be positive");
  }
}


// ============================================================
// Cumulative spatial arc length
// ============================================================

std::vector<double>
PredictionReferenceBuilder::calculateCumulativeArcLength(
  const autoware_planning_msgs::msg::Trajectory & trajectory) const
{
  std::vector<double> s(
    trajectory.points.size(),
    0.0);

  if (trajectory.points.empty()) {
    return s;
  }

  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & p0 =
      trajectory.points.at(i - 1).pose.position;

    const auto & p1 =
      trajectory.points.at(i).pose.position;

    const double ds =
      std::hypot(
        p1.x - p0.x,
        p1.y - p0.y);

    s.at(i) =
      s.at(i - 1) + ds;
  }

  return s;
}


// ============================================================
// ROS Duration -> seconds
// ============================================================

std::vector<double>
PredictionReferenceBuilder::calculateRelativeTime(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<double> & cumulative_arc_length,
  const std::size_t start_index) const
{
  std::vector<double> relative_time(
    trajectory.points.size(),
    0.0);

  if (
    trajectory.points.empty() ||
    start_index >= trajectory.points.size())
  {
    return relative_time;
  }

  relative_time.at(start_index) = 0.0;

  for (
    std::size_t i = start_index + 1;
    i < trajectory.points.size();
    ++i)
  {
    const double ds =
      cumulative_arc_length.at(i) -
      cumulative_arc_length.at(i - 1);

    const double v0 =
      std::abs(
        static_cast<double>(
          trajectory.points.at(i - 1).
            longitudinal_velocity_mps));

    const double v1 =
      std::abs(
        static_cast<double>(
          trajectory.points.at(i).
            longitudinal_velocity_mps));

    /*
     * Trapezoidal approximation of segment velocity.
     *
     * For a linearly changing velocity profile this gives
     * the correct segment traversal time:
     *
     *   dt = ds / ((v0 + v1)/2)
     */
    const double average_velocity =
      std::max(
        0.5 * (v0 + v1),
        minimum_prediction_velocity_);

    const double segment_dt =
      ds /
      average_velocity;

    relative_time.at(i) =
      relative_time.at(i - 1) +
      segment_dt;
  }

  return relative_time;
}


// ============================================================
// Calculate actual prediction dt
//
// We determine how much trajectory time is required to cover
// min_prediction_length.
//
// Since our prediction vector contains:
//
//   x1 ... xN
//
// and NOT x0, the complete prediction duration is:
//
//   N * dt
//
// therefore:
//
//   dt = (t_target - t_start) / N
// ============================================================


double PredictionReferenceBuilder::calculatePredictionDt(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::size_t start_index,
  const std::vector<double> & cumulative_arc_length,
  const std::vector<double> & relative_time) const
{
  if (start_index >= trajectory.points.size()) {
    throw std::out_of_range(
      "PredictionReferenceBuilder: invalid start index");
  }

  if (trajectory.points.size() < 2) {
    return nominal_prediction_dt_;
  }

  const double start_arc_length =
    cumulative_arc_length.at(start_index);

  const double desired_arc_length =
    start_arc_length +
    min_prediction_length_;

  const double target_arc_length =
    std::min(
      desired_arc_length,
      cumulative_arc_length.back());

  const auto begin =
    cumulative_arc_length.begin() +
    static_cast<std::ptrdiff_t>(
      start_index);

  const auto upper_it =
    std::lower_bound(
      begin,
      cumulative_arc_length.end(),
      target_arc_length);

  std::size_t upper_index;

  if (upper_it == cumulative_arc_length.end()) {
    upper_index =
      cumulative_arc_length.size() - 1;
  } else {
    upper_index =
      static_cast<std::size_t>(
        std::distance(
          cumulative_arc_length.begin(),
          upper_it));
  }

  if (upper_index <= start_index) {
    return nominal_prediction_dt_;
  }

  const std::size_t lower_index =
    upper_index - 1;

  const double s0 =
    cumulative_arc_length.at(lower_index);

  const double s1 =
    cumulative_arc_length.at(upper_index);

  const double t0 =
    relative_time.at(lower_index);

  const double t1 =
    relative_time.at(upper_index);

  double target_time =
    t1;

  if (s1 > s0 + 1.0e-9) {
    const double alpha =
      std::clamp(
        (target_arc_length - s0) /
        (s1 - s0),
        0.0,
        1.0);

    target_time =
      t0 +
      alpha *
      (t1 - t0);
  }

  if (
    !std::isfinite(target_time) ||
    target_time <= 1.0e-9)
  {
    return nominal_prediction_dt_;
  }

  const double dt_for_minimum_length =
    target_time /
    static_cast<double>(
      prediction_horizon_);

  return std::max(
    nominal_prediction_dt_,
    dt_for_minimum_length);
}





// ============================================================
// Time interpolation of one prediction reference point
// ============================================================

PredictionReferencePoint
PredictionReferenceBuilder::interpolateReferenceAtTime(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<double> & cumulative_arc_length,
  const std::vector<double> & relative_time,
  const std::size_t start_index,
  const double target_time,
  const double start_arc_length) const
{
  if (trajectory.points.empty()) {
    throw std::runtime_error(
      "Cannot interpolate empty trajectory");
  }



  // ----------------------------------------------------------
  // Protection for target at/before start
  // ----------------------------------------------------------

  if (target_time <= 0.0) {
    const auto & point =
      trajectory.points.at(start_index);

    const double curvature =
      TrackingStateBuilder::calculateCurvature(
        trajectory,
        start_index);

    PredictionReferencePoint result;

    result.trajectory_index =
      start_index;

    result.arc_length =
      cumulative_arc_length.at(start_index) -
      start_arc_length;

    result.velocity =
      static_cast<double>(
        point.longitudinal_velocity_mps);

    result.acceleration =
      static_cast<double>(
        point.acceleration_mps2);

    result.curvature =
      curvature;

    result.steering =
      std::atan(
        wheel_base_ *
        curvature);

    return result;
  }

  // ----------------------------------------------------------
  // Find first trajectory point whose time >= target_time
  // ----------------------------------------------------------

	const auto begin =
	  relative_time.begin() +
	  static_cast<std::ptrdiff_t>(
	    start_index);

	const auto upper_it =
	  std::lower_bound(
	    begin,
	    relative_time.end(),
	    target_time);

  // ----------------------------------------------------------
  // Prediction time exceeds trajectory duration:
  // clamp to final point
  // ----------------------------------------------------------

  if (upper_it == relative_time.end()) {
    const std::size_t last_index =
      trajectory.points.size() - 1;

    const auto & point =
      trajectory.points.back();

    const double curvature =
      TrackingStateBuilder::calculateCurvature(
        trajectory,
        last_index);

    PredictionReferencePoint result;

    result.trajectory_index =
      last_index;

    result.arc_length =
      cumulative_arc_length.back() -
      start_arc_length;

    result.velocity =
      static_cast<double>(
        point.longitudinal_velocity_mps);

    result.acceleration =
      static_cast<double>(
        point.acceleration_mps2);

    result.curvature =
      curvature;

    result.steering =
      std::atan(
        wheel_base_ *
        curvature);

    return result;
  }

const std::size_t upper_index =
  static_cast<std::size_t>(
    std::distance(
      relative_time.begin(),
      upper_it));

  if (upper_index == start_index) {
    const auto & point =
      trajectory.points.at(upper_index);

    const double curvature =
      TrackingStateBuilder::calculateCurvature(
        trajectory,
        upper_index);

    PredictionReferencePoint result;

    result.trajectory_index =
      upper_index;

    result.arc_length =
      cumulative_arc_length.at(upper_index) -
      start_arc_length;

    result.velocity =
      static_cast<double>(
        point.longitudinal_velocity_mps);

    result.acceleration =
      static_cast<double>(
        point.acceleration_mps2);

    result.curvature =
      curvature;

    result.steering =
      std::atan(
        wheel_base_ *
        curvature);

    return result;
  }

  const std::size_t lower_index =
    upper_index - 1;

  const auto & lower =
    trajectory.points.at(lower_index);

  const auto & upper =
    trajectory.points.at(upper_index);

  const double t0 =
    relative_time.at(lower_index);

  const double t1 =
    relative_time.at(upper_index);

  double alpha = 1.0;

  if (t1 > t0 + 1.0e-9) {
    alpha =
      std::clamp(
        (target_time - t0) /
        (t1 - t0),
        0.0,
        1.0);
  }

  // ----------------------------------------------------------
  // Interpolate longitudinal reference
  // ----------------------------------------------------------

  const double velocity =
    static_cast<double>(
      lower.longitudinal_velocity_mps) +
    alpha *
    (
      static_cast<double>(
        upper.longitudinal_velocity_mps) -
      static_cast<double>(
        lower.longitudinal_velocity_mps)
    );

  const double acceleration =
    static_cast<double>(
      lower.acceleration_mps2) +
    alpha *
    (
      static_cast<double>(
        upper.acceleration_mps2) -
      static_cast<double>(
        lower.acceleration_mps2)
    );

  // ----------------------------------------------------------
  // Interpolate curvature
  // ----------------------------------------------------------

  const double curvature_lower =
    TrackingStateBuilder::calculateCurvature(
      trajectory,
      lower_index);

  const double curvature_upper =
    TrackingStateBuilder::calculateCurvature(
      trajectory,
      upper_index);

  const double curvature =
    curvature_lower +
    alpha *
    (
      curvature_upper -
      curvature_lower
    );

  // ----------------------------------------------------------
  // Interpolate spatial position along trajectory
  // ----------------------------------------------------------

  const double arc_length =
    cumulative_arc_length.at(lower_index) +
    alpha *
    (
      cumulative_arc_length.at(upper_index) -
      cumulative_arc_length.at(lower_index)
    ) -
    start_arc_length;

  PredictionReferencePoint result;

  /*
   * trajectory_index is diagnostic only.
   * Use whichever source point is closer in interpolation.
   */
  result.trajectory_index =
    (alpha < 0.5) ?
    lower_index :
    upper_index;

  result.arc_length =
    arc_length;

  result.velocity =
    velocity;

  result.acceleration =
    acceleration;

  result.curvature =
    curvature;

  result.steering =
    std::atan(
      wheel_base_ *
      curvature);

  return result;
}


// ============================================================
// Build prediction reference horizon
// ============================================================


PredictionReferenceHorizon
PredictionReferenceBuilder::build(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::size_t start_index) const
{
  // ==========================================================
  // Input validation
  // ==========================================================

  if (trajectory.points.empty()) {
    throw std::runtime_error(
      "Cannot build prediction horizon from empty trajectory");
  }

  if (start_index >= trajectory.points.size()) {
    throw std::out_of_range(
      "Prediction horizon start_index is outside trajectory");
  }

  // ==========================================================
  // Spatial trajectory information
  // ==========================================================

  const auto cumulative_arc_length =
    calculateCumulativeArcLength(
      trajectory);

  // ==========================================================
  // Build internal relative time from geometry + velocity
  //
  // relative_time[start_index] = 0
  //
  // We intentionally do NOT use
  // TrajectoryPoint::time_from_start here.
  // ==========================================================

  const auto relative_time =
    calculateRelativeTime(
      trajectory,
      cumulative_arc_length,
      start_index);

  // ==========================================================
  // Prediction horizon object
  // ==========================================================

  PredictionReferenceHorizon horizon;

  horizon.points.reserve(
    prediction_horizon_);

  // ==========================================================
  // Calculate prediction dt
  // ==========================================================

  horizon.prediction_dt =
    calculatePredictionDt(
      trajectory,
      start_index,
      cumulative_arc_length,
      relative_time);

  const double start_arc_length =
    cumulative_arc_length.at(
      start_index);

  // ==========================================================
  // Build future prediction references
  //
  // x1 -> dt
  // x2 -> 2*dt
  // ...
  // xN -> N*dt
  // ==========================================================

  for (
    std::size_t i = 0;
    i < prediction_horizon_;
    ++i)
  {
    const double target_time =
      static_cast<double>(i + 1) *
      horizon.prediction_dt;

    const auto reference =
      interpolateReferenceAtTime(
        trajectory,
        cumulative_arc_length,
        relative_time,
        start_index,
        target_time,
        start_arc_length);

    horizon.points.push_back(
      reference);
  }

  // ==========================================================
  // Total spatial prediction length
  // ==========================================================

  if (!horizon.points.empty()) {
    horizon.total_arc_length =
      horizon.points.back().
        arc_length;
  }

  return horizon;
}


}  // namespace autoware::integrated_mpc
