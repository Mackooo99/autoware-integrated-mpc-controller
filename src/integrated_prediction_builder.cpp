#include "autoware/integrated_mpc_trajectory_follower/integrated_prediction_builder.hpp"

#include "autoware/integrated_mpc_trajectory_follower/integrated_kinematics_bicycle_model.hpp"
#include "autoware/integrated_mpc_trajectory_follower/prediction_matrix_builder.hpp"

#include <Eigen/Core>

#include <stdexcept>

namespace autoware::integrated_mpc
{

IntegratedPredictionBuilder::IntegratedPredictionBuilder(
  const double wheel_base,
  const double steer_tau)
: wheel_base_(wheel_base),
  steer_tau_(steer_tau)
{
  if (wheel_base_ <= 0.0) {
    throw std::invalid_argument(
      "IntegratedPredictionBuilder: wheel_base must be positive");
  }

  if (steer_tau_ <= 0.0) {
    throw std::invalid_argument(
      "IntegratedPredictionBuilder: steer_tau must be positive");
  }
}


IntegratedPredictionResult IntegratedPredictionBuilder::build(
  const PredictionReferenceHorizon & reference_horizon,
  const Eigen::VectorXd & initial_state) const
{
  constexpr Eigen::Index NX = 5;
  constexpr Eigen::Index NU = 2;

  // ==========================================================
  // Validation
  // ==========================================================

  if (reference_horizon.points.empty()) {
    throw std::invalid_argument(
      "IntegratedPredictionBuilder: reference horizon is empty");
  }

  if (reference_horizon.prediction_dt <= 0.0) {
    throw std::invalid_argument(
      "IntegratedPredictionBuilder: prediction_dt must be positive");
  }

  if (initial_state.size() != NX) {
    throw std::invalid_argument(
      "IntegratedPredictionBuilder: initial state must have dimension 5");
  }

  const std::size_t N =
    reference_horizon.points.size();

  IntegratedPredictionResult result;

  result.A.reserve(N);
  result.B.reserve(N);
  result.W.reserve(N);

  result.reference_input =
    Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(N) * NU);

  // ==========================================================
  // Integrated vehicle model
  // ==========================================================

  IntegratedKinematicsBicycleModel model(
    wheel_base_,
    steer_tau_);

  // ==========================================================
  // Build one discrete LTV model for every horizon point
  // ==========================================================

  for (std::size_t i = 0; i < N; ++i) {
    const auto & ref =
      reference_horizon.points.at(i);

    // --------------------------------------------------------
    // Reference parameters for this prediction interval
    // --------------------------------------------------------

    model.setReference(
      ref.velocity,
      ref.curvature,
      ref.acceleration);

    // --------------------------------------------------------
    // Linearization point
    //
    // x_bar =
    // [0, 0, 0, delta_ref, a_ref]^T
    //
    // u_bar =
    // [delta_ref, 0]^T
    //
    // Jerk reference is zero in V1.
    // --------------------------------------------------------

    Eigen::Matrix<double, NX, 1> x_bar;

    x_bar <<
      0.0,              // e_y
      0.0,              // e_psi
      0.0,              // e_v
      ref.steering,     // delta
      ref.acceleration; // a

    Eigen::Matrix<double, NU, 1> u_bar;

    u_bar <<
      ref.steering,     // delta_cmd
      0.0;              // jerk

    model.setLinearizationPoint(
      x_bar,
      u_bar);

    // --------------------------------------------------------
    // Discrete model
    // --------------------------------------------------------

    Eigen::Matrix<double, NX, NX> Ad;
    Eigen::Matrix<double, NX, NU> Bd;
    Eigen::Matrix<double, NX, 1> Wd;

    model.calculateDiscreteMatrix(
      reference_horizon.prediction_dt,
      Ad,
      Bd,
      Wd);

    // --------------------------------------------------------
    // Basic numerical validation
    // --------------------------------------------------------

    if (
      !Ad.allFinite() ||
      !Bd.allFinite() ||
      !Wd.allFinite())
    {
      throw std::runtime_error(
        "IntegratedPredictionBuilder: non-finite discrete model matrix");
    }

    result.A.emplace_back(Ad);
    result.B.emplace_back(Bd);
    result.W.emplace_back(Wd);

    // --------------------------------------------------------
    // Reference input vector
    //
    // U_ref =
    // [delta_ref_0, 0,
    //  delta_ref_1, 0,
    //  ...]
    // --------------------------------------------------------

    const Eigen::Index input_index =
      static_cast<Eigen::Index>(i) * NU;

    result.reference_input(input_index) =
      ref.steering;

    result.reference_input(input_index + 1) =
      0.0;
  }

  // ==========================================================
  // Build extended prediction matrices
  // ==========================================================

  result.extended =
    PredictionMatrixBuilder::build(
      result.A,
      result.B,
      result.W);

  // ==========================================================
  // Validate extended matrices
  // ==========================================================

  if (
    !result.extended.Aex.allFinite() ||
    !result.extended.Bex.allFinite() ||
    !result.extended.Wex.allFinite())
  {
    throw std::runtime_error(
      "IntegratedPredictionBuilder: non-finite extended prediction matrix");
  }

  // ==========================================================
  // Reference-input response
  //
  // This is NOT yet the optimized MPC prediction.
  //
  // It simply answers:
  //
  // "What does our current linear model predict if we apply
  //  delta_ref and zero jerk over the entire horizon?"
  //
  // ==========================================================

  result.reference_input_response =
    result.extended.Aex *
    initial_state +
    result.extended.Bex *
    result.reference_input +
    result.extended.Wex;

  if (!result.reference_input_response.allFinite()) {
    throw std::runtime_error(
      "IntegratedPredictionBuilder: non-finite prediction response");
  }

  return result;
}

}  // namespace autoware::integrated_mpc
