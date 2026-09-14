#pragma once

#include "autoware/integrated_mpc_trajectory_follower/prediction_matrix_builder.hpp"
#include "autoware/integrated_mpc_trajectory_follower/prediction_reference_builder.hpp"

#include <Eigen/Core>

#include <vector>

namespace autoware::integrated_mpc
{

struct IntegratedPredictionResult
{
  // Local LTV models:
  // x_{k+1} = A_k x_k + B_k u_k + W_k
  std::vector<Eigen::MatrixXd> A;
  std::vector<Eigen::MatrixXd> B;
  std::vector<Eigen::VectorXd> W;

  // Extended model:
  // X = Aex*x0 + Bex*U + Wex
  ExtendedPredictionMatrices extended;

  // Nominal/reference input:
  //
  // u_ref =
  // [delta_ref, 0]^T
  //
  // second component is jerk reference = 0 for V1
  Eigen::VectorXd reference_input;

  // Response obtained by applying reference_input
  // from the current initial state.
  Eigen::VectorXd reference_input_response;
};


class IntegratedPredictionBuilder
{
public:
  IntegratedPredictionBuilder(
    double wheel_base,
    double steer_tau);

  IntegratedPredictionResult build(
    const PredictionReferenceHorizon & reference_horizon,
    const Eigen::VectorXd & initial_state) const;

private:
  double wheel_base_;
  double steer_tau_;
};

}  // namespace autoware::integrated_mpc
