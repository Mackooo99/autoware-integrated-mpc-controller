#include "autoware/integrated_mpc_trajectory_follower/mpc_cost_builder.hpp"

#include <Eigen/Core>

#include <cmath>
#include <stdexcept>

namespace autoware::integrated_mpc
{

MpcCostResult MpcCostBuilder::build(
  const ExtendedPredictionMatrices & prediction,
  const PredictionReferenceHorizon & reference_horizon,
  const Eigen::VectorXd & initial_state,
  const MpcCostWeights & weights)
{
  constexpr Eigen::Index NX = 5;
  constexpr Eigen::Index NU = 2;

  // ==========================================================
  // Validation
  // ==========================================================

  const std::size_t N =
    reference_horizon.points.size();

  if (N == 0) {
    throw std::invalid_argument(
      "MpcCostBuilder: empty reference horizon");
  }

  if (initial_state.size() != NX) {
    throw std::invalid_argument(
      "MpcCostBuilder: initial state dimension must be 5");
  }

  const Eigen::Index extended_state_size =
    static_cast<Eigen::Index>(N) * NX;

  const Eigen::Index extended_input_size =
    static_cast<Eigen::Index>(N) * NU;

  if (
    prediction.Aex.rows() != extended_state_size ||
    prediction.Aex.cols() != NX)
  {
    throw std::invalid_argument(
      "MpcCostBuilder: invalid Aex dimensions");
  }

  if (
    prediction.Bex.rows() != extended_state_size ||
    prediction.Bex.cols() != extended_input_size)
  {
    throw std::invalid_argument(
      "MpcCostBuilder: invalid Bex dimensions");
  }

  if (
    prediction.Wex.size() != extended_state_size)
  {
    throw std::invalid_argument(
      "MpcCostBuilder: invalid Wex dimensions");
  }

  // ==========================================================
  // Allocate result
  // ==========================================================

  MpcCostResult result;

  result.Qex =
    Eigen::MatrixXd::Zero(
      extended_state_size,
      extended_state_size);

  result.Rex =
    Eigen::MatrixXd::Zero(
      extended_input_size,
      extended_input_size);

  result.state_reference =
    Eigen::VectorXd::Zero(
      extended_state_size);

  result.input_reference =
    Eigen::VectorXd::Zero(
      extended_input_size);

  // ==========================================================
  // Build stage cost matrices
  // ==========================================================

  for (std::size_t i = 0; i < N; ++i) {
    const auto & ref =
      reference_horizon.points.at(i);

    const double velocity =
      ref.velocity;

    const double velocity_squared =
      velocity *
      velocity;

    // --------------------------------------------------------
    // Velocity-dependent weights
    // --------------------------------------------------------

    const double q_heading =
      weights.heading_error_squared_velocity *
      velocity_squared;

    const double r_steering =
      weights.steering_input +
      weights.steering_input_squared_velocity *
      velocity_squared;

    // --------------------------------------------------------
    // State block
    //
    // x =
    // [e_y, e_psi, e_v, delta, a]
    //
    // We do NOT directly penalize delta state in V1.
    // Steering command is penalized through R.
    // --------------------------------------------------------

    Eigen::Matrix<double, NX, NX> Q =
      Eigen::Matrix<double, NX, NX>::Zero();

    Q(0, 0) =
      weights.lateral_error;

    Q(1, 1) =
      q_heading;

    Q(2, 2) =
      weights.velocity_error;

    Q(3, 3) =
      0.0;

    Q(4, 4) =
      weights.acceleration_error;

    // --------------------------------------------------------
    // Terminal cost is ADDED to the final stage cost
    // --------------------------------------------------------

    if (i == N - 1) {
      Q(0, 0) +=
        weights.terminal_lateral_error;

      Q(1, 1) +=
        weights.terminal_heading_error;

      Q(2, 2) +=
        weights.terminal_velocity_error;
    }

    const Eigen::Index state_row =
      static_cast<Eigen::Index>(i) * NX;

    result.Qex.block(
      state_row,
      state_row,
      NX,
      NX) =
      Q;

    // --------------------------------------------------------
    // State reference
    //
    // Desired:
    //
    // e_y   = 0
    // e_psi = 0
    // e_v   = 0
    // delta = delta_ref
    // a     = a_ref
    //
    // delta currently has zero state weight, but we still
    // store delta_ref for consistency/future extensions.
    // --------------------------------------------------------

    result.state_reference.segment(
      state_row,
      NX) <<
      0.0,
      0.0,
      0.0,
      ref.steering,
      ref.acceleration;

    // --------------------------------------------------------
    // Input block
    //
    // u =
    // [delta_cmd, jerk]
    // --------------------------------------------------------

    Eigen::Matrix<double, NU, NU> R =
      Eigen::Matrix<double, NU, NU>::Zero();

    R(0, 0) =
      r_steering;

    R(1, 1) =
      weights.jerk;

    const Eigen::Index input_row =
      static_cast<Eigen::Index>(i) * NU;

    result.Rex.block(
      input_row,
      input_row,
      NU,
      NU) =
      R;

    // --------------------------------------------------------
    // Input reference
    //
    // delta_cmd_ref = delta_ref
    // jerk_ref      = 0
    // --------------------------------------------------------

    result.input_reference.segment(
      input_row,
      NU) <<
      ref.steering,
      0.0;
  }

  // ==========================================================
  // Free response:
  //
  // X_free = Aex*x0 + Wex
  // ==========================================================

  result.free_response =
    prediction.Aex *
    initial_state +
    prediction.Wex;

  const Eigen::VectorXd state_offset =
    result.free_response -
    result.state_reference;

  // ==========================================================
  // QP construction
  //
  // J =
  // (B U + d)^T Q (B U + d)
  // +
  // (U - Uref)^T R (U - Uref)
  //
  // Standard form:
  //
  // J =
  // 0.5 U^T H U
  // + f^T U
  // + c
  // ==========================================================

  const Eigen::MatrixXd quadratic =
    prediction.Bex.transpose() *
    result.Qex *
    prediction.Bex +
    result.Rex;

  result.H =
    2.0 *
    quadratic;

  // Numerical symmetry protection
  result.H =
    0.5 *
    (
      result.H +
      result.H.transpose()
    );

  result.f =
    2.0 *
    (
      prediction.Bex.transpose() *
      result.Qex *
      state_offset
      -
      result.Rex *
      result.input_reference
    );

  result.constant_term =
    state_offset.dot( 
    result.Qex *
    state_offset)
    +
    result.input_reference.dot(
    result.Rex *
    result.input_reference);

  // ==========================================================
  // Numerical checks
  // ==========================================================

  if (
    !result.Qex.allFinite() ||
    !result.Rex.allFinite() ||
    !result.H.allFinite() ||
    !result.f.allFinite() ||
    !std::isfinite(result.constant_term))
  {
    throw std::runtime_error(
      "MpcCostBuilder: non-finite cost matrix");
  }

  return result;
}

}  // namespace autoware::integrated_mpc
