#include "autoware/integrated_mpc_trajectory_follower/mpc_constraint_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoware::integrated_mpc
{

namespace
{

constexpr double DEG2RAD =
  0.017453292519943295769;

}  // namespace


void MpcConstraintBuilder::validateMap(
  const std::vector<double> & x_values,
  const std::vector<double> & y_values,
  const char * name)
{
  if (
    x_values.empty() ||
    y_values.empty() ||
    x_values.size() != y_values.size())
  {
    throw std::invalid_argument(
      std::string(
        "MpcConstraintBuilder: invalid ") +
      name);
  }

  for (std::size_t i = 1; i < x_values.size(); ++i) {
    if (x_values.at(i) <= x_values.at(i - 1)) {
      throw std::invalid_argument(
        std::string(
          "MpcConstraintBuilder: ") +
        name +
        " x values must be strictly increasing");
    }
  }

  for (const double value : y_values) {
    if (
      !std::isfinite(value) ||
      value <= 0.0)
    {
      throw std::invalid_argument(
        std::string(
          "MpcConstraintBuilder: invalid ") +
        name +
        " rate");
    }
  }
}


double MpcConstraintBuilder::interpolate(
  const double x,
  const std::vector<double> & x_values,
  const std::vector<double> & y_values)
{
  if (x <= x_values.front()) {
    return y_values.front();
  }

  if (x >= x_values.back()) {
    return y_values.back();
  }

  const auto upper_it =
    std::upper_bound(
      x_values.begin(),
      x_values.end(),
      x);

  const std::size_t upper_index =
    static_cast<std::size_t>(
      std::distance(
        x_values.begin(),
        upper_it));

  const std::size_t lower_index =
    upper_index - 1;

  const double x0 =
    x_values.at(lower_index);

  const double x1 =
    x_values.at(upper_index);

  const double y0 =
    y_values.at(lower_index);

  const double y1 =
    y_values.at(upper_index);

  const double alpha =
    (x - x0) /
    (x1 - x0);

  return
    y0 +
    alpha *
    (y1 - y0);
}


MpcConstraintResult MpcConstraintBuilder::build(
  const ExtendedPredictionMatrices & prediction,
  const PredictionReferenceHorizon & reference_horizon,
  const Eigen::VectorXd & initial_state,
  const double previous_steering_command,
  const double control_period,
  const MpcConstraintParameters & parameters)
{
  constexpr Eigen::Index NX = 5;
  constexpr Eigen::Index NU = 2;

  const std::size_t N =
    reference_horizon.points.size();

  if (N == 0) {
    throw std::invalid_argument(
      "MpcConstraintBuilder: empty prediction horizon");
  }

  if (initial_state.size() != NX) {
    throw std::invalid_argument(
      "MpcConstraintBuilder: initial state must have size 5");
  }

  if (reference_horizon.prediction_dt <= 0.0) {
    throw std::invalid_argument(
      "MpcConstraintBuilder: prediction dt must be positive");
  }

  if (parameters.max_steering_angle <= 0.0) {
    throw std::invalid_argument(
      "MpcConstraintBuilder: max steering angle must be positive");
  }

  if (
    parameters.min_acceleration >=
    parameters.max_acceleration)
  {
    throw std::invalid_argument(
      "MpcConstraintBuilder: invalid acceleration bounds");
  }

  if (
    parameters.min_jerk >=
    parameters.max_jerk)
  {
    throw std::invalid_argument(
      "MpcConstraintBuilder: invalid jerk bounds");
  }
  
  
  if (
	  !std::isfinite(control_period) ||
	  control_period <= 0.0)
	{
	  throw std::invalid_argument(
	    "MpcConstraintBuilder: control period must be positive");
	}

  validateMap(
    parameters.curvature_for_steer_rate,
    parameters.steer_rate_deg_by_curvature,
    "curvature steer-rate map");

  validateMap(
    parameters.velocity_for_steer_rate,
    parameters.steer_rate_deg_by_velocity,
    "velocity steer-rate map");

  const Eigen::Index state_size =
    static_cast<Eigen::Index>(N) *
    NX;

  const Eigen::Index input_size =
    static_cast<Eigen::Index>(N) *
    NU;

  if (
    prediction.Aex.rows() != state_size ||
    prediction.Aex.cols() != NX)
  {
    throw std::invalid_argument(
      "MpcConstraintBuilder: invalid Aex dimensions");
  }

  if (
    prediction.Bex.rows() != state_size ||
    prediction.Bex.cols() != input_size)
  {
    throw std::invalid_argument(
      "MpcConstraintBuilder: invalid Bex dimensions");
  }

  if (prediction.Wex.size() != state_size) {
    throw std::invalid_argument(
      "MpcConstraintBuilder: invalid Wex dimensions");
  }

  // ==========================================================
  // Constraint layout
  //
  // 0 ... N-1       : steering angle
  // N ... 2N-1      : jerk
  // 2N ... 3N-1     : acceleration state
  // 3N ... 4N-1     : steering rate
  // ==========================================================

  const Eigen::Index constraint_count =
    static_cast<Eigen::Index>(4 * N);

  MpcConstraintResult result;

  result.A =
    Eigen::MatrixXd::Zero(
      constraint_count,
      input_size);

  result.lower =
    Eigen::VectorXd::Zero(
      constraint_count);

  result.upper =
    Eigen::VectorXd::Zero(
      constraint_count);

  result.steering_rate_limit =
    Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(N));

  // ==========================================================
  // Free state response
  // ==========================================================

  const Eigen::VectorXd free_response =
    prediction.Aex *
    initial_state +
    prediction.Wex;

  const double prediction_dt =
  reference_horizon.prediction_dt;

  // ==========================================================
  // Build each constraint group
  // ==========================================================

  for (std::size_t i = 0; i < N; ++i) {
    const Eigen::Index k =
      static_cast<Eigen::Index>(i);

    const Eigen::Index steer_col =
      k * NU;

    const Eigen::Index jerk_col =
      steer_col + 1;

    // --------------------------------------------------------
    // 1. Steering command angle
    //
    // -delta_max <= delta_cmd <= delta_max
    // --------------------------------------------------------

    const Eigen::Index steering_row =
      k;

    result.A(
      steering_row,
      steer_col) =
      1.0;

    result.lower(
      steering_row) =
      -parameters.max_steering_angle;

    result.upper(
      steering_row) =
      parameters.max_steering_angle;

    // --------------------------------------------------------
    // 2. Jerk
    //
    // j_min <= j <= j_max
    // --------------------------------------------------------

    const Eigen::Index jerk_row =
      static_cast<Eigen::Index>(N) +
      k;

    result.A(
      jerk_row,
      jerk_col) =
      1.0;

    result.lower(
      jerk_row) =
      parameters.min_jerk;

    result.upper(
      jerk_row) =
      parameters.max_jerk;

    // --------------------------------------------------------
    // 3. Acceleration state
    //
    // X = Xfree + Bex U
    //
    // a_min <= a_k <= a_max
    //
    // therefore:
    //
    // a_min - a_free
    // <= B_a U <=
    // a_max - a_free
    // --------------------------------------------------------

    const Eigen::Index acceleration_state_row =
      k * NX + 4;

    const Eigen::Index acceleration_constraint_row =
      static_cast<Eigen::Index>(2 * N) +
      k;

    result.A.row(
      acceleration_constraint_row) =
      prediction.Bex.row(
        acceleration_state_row);

    const double free_acceleration =
      free_response(
        acceleration_state_row);

    result.lower(
      acceleration_constraint_row) =
      parameters.min_acceleration -
      free_acceleration;

    result.upper(
      acceleration_constraint_row) =
      parameters.max_acceleration -
      free_acceleration;

    // --------------------------------------------------------
    // 4. Steering rate
    // --------------------------------------------------------

    const auto & ref =
      reference_horizon.points.at(i);

    const double abs_curvature =
      std::abs(
        ref.curvature);

    const double abs_velocity =
      std::abs(
        ref.velocity);

    const double rate_deg_by_curvature =
      interpolate(
        abs_curvature,
        parameters.curvature_for_steer_rate,
        parameters.steer_rate_deg_by_curvature);

    const double rate_deg_by_velocity =
      interpolate(
        abs_velocity,
        parameters.velocity_for_steer_rate,
        parameters.steer_rate_deg_by_velocity);

    // Use the more restrictive of the two limits.
    const double rate_rad_per_sec =
      std::min(
        rate_deg_by_curvature,
        rate_deg_by_velocity) *
      DEG2RAD;

    result.steering_rate_limit(k) =
      rate_rad_per_sec;

    const double rate_dt =
	  (i == 0)
	  ? control_period
	  : prediction_dt;

	const double max_delta_change =
	  rate_rad_per_sec *
	  rate_dt;

    const Eigen::Index rate_row =
      static_cast<Eigen::Index>(3 * N) +
      k;

    if (i == 0) {
      // ------------------------------------------------------
      // First command:
      //
      // -d <= delta_0 - delta_prev <= d
      //
      // equivalent to:
      //
      // delta_prev - d <= delta_0 <= delta_prev + d
      // ------------------------------------------------------

      result.A(
        rate_row,
        steer_col) =
        1.0;

      result.lower(
        rate_row) =
        previous_steering_command -
        max_delta_change;

      result.upper(
        rate_row) =
        previous_steering_command +
        max_delta_change;

    } else {
      // ------------------------------------------------------
      // Subsequent commands:
      //
      // -d <= delta_k - delta_(k-1) <= d
      // ------------------------------------------------------

      const Eigen::Index previous_steer_col =
        static_cast<Eigen::Index>(i - 1) *
        NU;

      result.A(
        rate_row,
        steer_col) =
        1.0;

      result.A(
        rate_row,
        previous_steer_col) =
        -1.0;

      result.lower(
        rate_row) =
        -max_delta_change;

      result.upper(
        rate_row) =
        max_delta_change;
    }
  }

  if (
    !result.A.allFinite() ||
    !result.lower.allFinite() ||
    !result.upper.allFinite() ||
    !result.steering_rate_limit.allFinite())
  {
    throw std::runtime_error(
      "MpcConstraintBuilder: non-finite constraints");
  }

  return result;
}

}  // namespace autoware::integrated_mpc
