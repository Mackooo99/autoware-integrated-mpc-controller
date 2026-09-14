#pragma once

#include "autoware/integrated_mpc_trajectory_follower/prediction_matrix_builder.hpp"
#include "autoware/integrated_mpc_trajectory_follower/prediction_reference_builder.hpp"

#include <Eigen/Core>

namespace autoware::integrated_mpc
{

struct MpcCostWeights
{
  // State stage weights
  double lateral_error{1.0};

  double heading_error_squared_velocity{0.3};

  double velocity_error{1.0};

  double acceleration_error{0.1};

  // Input stage weights
  double steering_input{1.0};

  double steering_input_squared_velocity{0.25};

  double jerk{0.1};

  // Terminal additions
  double terminal_lateral_error{1.0};

  double terminal_heading_error{0.1};

  double terminal_velocity_error{1.0};
};


struct MpcCostResult
{
  Eigen::MatrixXd Qex;
  Eigen::MatrixXd Rex;

  Eigen::VectorXd state_reference;
  Eigen::VectorXd input_reference;

  // Aex*x0 + Wex
  Eigen::VectorXd free_response;

  // Standard QP form:
  //
  // J(U) =
  // 0.5 * U^T H U
  // + f^T U
  // + constant_term
  Eigen::MatrixXd H;
  Eigen::VectorXd f;

  double constant_term{0.0};
};


class MpcCostBuilder
{
public:
  static MpcCostResult build(
    const ExtendedPredictionMatrices & prediction,
    const PredictionReferenceHorizon & reference_horizon,
    const Eigen::VectorXd & initial_state,
    const MpcCostWeights & weights);
};

}  // namespace autoware::integrated_mpc
