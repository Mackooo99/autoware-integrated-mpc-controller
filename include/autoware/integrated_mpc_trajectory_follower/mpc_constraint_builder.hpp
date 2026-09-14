#pragma once

#include "autoware/integrated_mpc_trajectory_follower/prediction_matrix_builder.hpp"
#include "autoware/integrated_mpc_trajectory_follower/prediction_reference_builder.hpp"

#include <Eigen/Core>

#include <vector>

namespace autoware::integrated_mpc
{

struct MpcConstraintParameters
{
  double max_steering_angle{0.7};

  double min_acceleration{-5.0};
  double max_acceleration{3.0};

  double min_jerk{-5.0};
  double max_jerk{2.0};

  std::vector<double> curvature_for_steer_rate{
    0.001, 0.002, 0.01};

  std::vector<double> steer_rate_deg_by_curvature{
    40.0, 50.0, 60.0};

  std::vector<double> velocity_for_steer_rate{
    10.0, 15.0, 20.0};

  std::vector<double> steer_rate_deg_by_velocity{
    60.0, 50.0, 40.0};
};


struct MpcConstraintResult
{
  // Standard OSQP form:
  //
  // lower <= A * U <= upper
  Eigen::MatrixXd A;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;

  // Diagnostic value for every prediction step [rad/s]
  Eigen::VectorXd steering_rate_limit;
};


class MpcConstraintBuilder
{
public:
  static MpcConstraintResult build(
	  const ExtendedPredictionMatrices & prediction,
	  const PredictionReferenceHorizon & reference_horizon,
	  const Eigen::VectorXd & initial_state,
	  double previous_steering_command,
	  double control_period,
	  const MpcConstraintParameters & parameters);

private:
  static double interpolate(
    double x,
    const std::vector<double> & x_values,
    const std::vector<double> & y_values);

  static void validateMap(
    const std::vector<double> & x_values,
    const std::vector<double> & y_values,
    const char * name);
};

}  // namespace autoware::integrated_mpc
