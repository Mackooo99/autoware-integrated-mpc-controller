#include "autoware/integrated_mpc_trajectory_follower/mpc_cost_builder.hpp"
#include "autoware/integrated_mpc_trajectory_follower/prediction_matrix_builder.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using autoware::integrated_mpc::MpcCostBuilder;
using autoware::integrated_mpc::MpcCostWeights;
using autoware::integrated_mpc::PredictionMatrixBuilder;
using autoware::integrated_mpc::PredictionReferenceHorizon;
using autoware::integrated_mpc::PredictionReferencePoint;

namespace
{

double calculateDirectCost(
  const autoware::integrated_mpc::ExtendedPredictionMatrices & prediction,
  const autoware::integrated_mpc::MpcCostResult & cost,
  const Eigen::VectorXd & x0,
  const Eigen::VectorXd & U)
{
  const Eigen::VectorXd X =
    prediction.Aex *
    x0 +
    prediction.Bex *
    U +
    prediction.Wex;

  const Eigen::VectorXd state_error =
    X -
    cost.state_reference;

  const Eigen::VectorXd input_error =
    U -
    cost.input_reference;

  return
    state_error.dot( 
    cost.Qex *
    state_error )
    +
    input_error.dot( 
    cost.Rex *
    input_error);
}


double calculateQpCost(
  const autoware::integrated_mpc::MpcCostResult & cost,
  const Eigen::VectorXd & U)
{
  return
    0.5 *
    U.dot( 
    cost.H *
    U)
    +
    cost.f.dot(U) 
    +
    cost.constant_term;
}

}  // namespace


int main()
{
  bool success = true;

  constexpr std::size_t N = 3;

  constexpr Eigen::Index NX = 5;
  constexpr Eigen::Index NU = 2;

  // ==========================================================
  // Build a small 5-state / 2-input LTV prediction model
  // ==========================================================

  std::vector<Eigen::MatrixXd> A(N);
  std::vector<Eigen::MatrixXd> B(N);
  std::vector<Eigen::VectorXd> W(N);

  for (std::size_t i = 0; i < N; ++i) {
    A.at(i) =
      Eigen::MatrixXd::Identity(
        NX,
        NX);

    B.at(i) =
      Eigen::MatrixXd::Zero(
        NX,
        NU);

    W.at(i) =
      Eigen::VectorXd::Zero(
        NX);

    // Small lateral coupling
    A.at(i)(0, 1) =
      0.2 +
      0.05 *
      static_cast<double>(i);

    // e_v <- acceleration
    A.at(i)(2, 4) =
      0.1;

    // steering command -> steering state
    B.at(i)(3, 0) =
      0.3;

    // jerk -> e_v
    B.at(i)(2, 1) =
      0.005;

    // jerk -> acceleration
    B.at(i)(4, 1) =
      0.1;
  }

  const auto prediction =
    PredictionMatrixBuilder::build(
      A,
      B,
      W);

  // ==========================================================
  // Reference horizon
  // ==========================================================

  PredictionReferenceHorizon reference;

  reference.prediction_dt =
    0.1;

  PredictionReferencePoint r0;
  r0.velocity = 2.0;
  r0.acceleration = 0.5;
  r0.curvature = 0.01;
  r0.steering = 0.02;

  PredictionReferencePoint r1;
  r1.velocity = 4.0;
  r1.acceleration = 0.3;
  r1.curvature = 0.02;
  r1.steering = 0.04;

  PredictionReferencePoint r2;
  r2.velocity = 6.0;
  r2.acceleration = 0.0;
  r2.curvature = 0.03;
  r2.steering = 0.06;

  reference.points =
    {r0, r1, r2};

  // ==========================================================
  // Initial state
  // ==========================================================

  Eigen::VectorXd x0(NX);

  x0 <<
    -0.10,  // e_y
    0.02,   // e_psi
    -1.00,  // e_v
    0.00,   // delta
    0.00;   // a

  // ==========================================================
  // Weights
  // ==========================================================

  MpcCostWeights weights;

  const auto cost =
    MpcCostBuilder::build(
      prediction,
      reference,
      x0,
      weights);

  // ==========================================================
  // Dimensions
  // ==========================================================

  std::cout
    << std::fixed
    << std::setprecision(10);

  std::cout
    << "\n=== COST MATRIX DIMENSIONS ===\n";

  std::cout
    << "Qex = "
    << cost.Qex.rows()
    << " x "
    << cost.Qex.cols()
    << "\n";

  std::cout
    << "Rex = "
    << cost.Rex.rows()
    << " x "
    << cost.Rex.cols()
    << "\n";

  std::cout
    << "H   = "
    << cost.H.rows()
    << " x "
    << cost.H.cols()
    << "\n";

  std::cout
    << "f   = "
    << cost.f.size()
    << " x 1\n";

  success &=
    cost.Qex.rows() == 15 &&
    cost.Qex.cols() == 15;

  success &=
    cost.Rex.rows() == 6 &&
    cost.Rex.cols() == 6;

  success &=
    cost.H.rows() == 6 &&
    cost.H.cols() == 6;

  success &=
    cost.f.size() == 6;

  // ==========================================================
  // Weight checks
  // ==========================================================

  std::cout
    << "\n=== WEIGHT CHECKS ===\n";

  std::cout
    << "Q heading step 0 = "
    << cost.Qex(1, 1)
    << " expected 1.2\n";

  std::cout
    << "R steering step 0 = "
    << cost.Rex(0, 0)
    << " expected 2.0\n";

  // q_heading = 0.3 * 2^2 = 1.2
  success &=
    std::abs(
      cost.Qex(1, 1) -
      1.2) <
    1.0e-10;

  // r_delta = 1 + 0.25 * 2^2 = 2
  success &=
    std::abs(
      cost.Rex(0, 0) -
      2.0) <
    1.0e-10;

  // ==========================================================
  // H symmetry
  // ==========================================================

  const double symmetry_error =
    (
      cost.H -
      cost.H.transpose()
    ).norm();

  std::cout
    << "\n=== H MATRIX CHECK ===\n";

  std::cout
    << "symmetry error = "
    << symmetry_error
    << "\n";

  success &=
    symmetry_error <
    1.0e-10;

  // ==========================================================
  // Positive definiteness
  //
  // Since R has strictly positive diagonal weights,
  // H should be positive definite in this V1 formulation.
  // ==========================================================

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>
    eigen_solver(cost.H);

  const double minimum_eigenvalue =
    eigen_solver.eigenvalues().minCoeff();

  std::cout
    << "minimum eigenvalue = "
    << minimum_eigenvalue
    << "\n";

  success &=
    eigen_solver.info() ==
    Eigen::Success;

  success &=
    minimum_eigenvalue >
    0.0;

  // ==========================================================
  // QP cost equivalence
  // ==========================================================

  Eigen::VectorXd U(6);

  U <<
    0.01,
    0.20,
    0.03,
    -0.10,
    0.05,
    0.00;

  const double direct_cost =
    calculateDirectCost(
      prediction,
      cost,
      x0,
      U);

  const double qp_cost =
    calculateQpCost(
      cost,
      U);

  const double cost_error =
    std::abs(
      direct_cost -
      qp_cost);

  std::cout
    << "\n=== COST EQUIVALENCE ===\n";

  std::cout
    << "direct MPC cost = "
    << direct_cost
    << "\n";

  std::cout
    << "QP cost         = "
    << qp_cost
    << "\n";

  std::cout
    << "error           = "
    << cost_error
    << "\n";

  success &=
    cost_error <
    1.0e-9;

  // ==========================================================
  // Second arbitrary input vector
  // ==========================================================

  Eigen::VectorXd U2(6);

  U2 <<
    -0.02,
    0.50,
    0.06,
    0.10,
    0.04,
    -0.20;

  const double direct_cost_2 =
    calculateDirectCost(
      prediction,
      cost,
      x0,
      U2);

  const double qp_cost_2 =
    calculateQpCost(
      cost,
      U2);

  const double cost_error_2 =
    std::abs(
      direct_cost_2 -
      qp_cost_2);

  std::cout
    << "\nsecond cost error = "
    << cost_error_2
    << "\n";

  success &=
    cost_error_2 <
    1.0e-9;

  // ==========================================================
  // Result
  // ==========================================================

  std::cout
    << "\n====================================\n";

  if (success) {
    std::cout
      << "MPC COST SANITY CHECK: PASS\n";

    std::cout
      << "====================================\n";

    return 0;
  }

  std::cout
    << "MPC COST SANITY CHECK: FAIL\n";

  std::cout
    << "====================================\n";

  return 1;
}
