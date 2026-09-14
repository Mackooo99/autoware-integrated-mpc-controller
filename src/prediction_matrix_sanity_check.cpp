#include "autoware/integrated_mpc_trajectory_follower/prediction_matrix_builder.hpp"

#include <Eigen/Core>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using autoware::integrated_mpc::PredictionMatrixBuilder;

namespace
{

bool matrixApproximatelyEqual(
  const Eigen::MatrixXd & a,
  const Eigen::MatrixXd & b,
  const double tolerance = 1.0e-10)
{
  if (
    a.rows() != b.rows() ||
    a.cols() != b.cols())
  {
    return false;
  }

  return (a - b).norm() < tolerance;
}

bool vectorApproximatelyEqual(
  const Eigen::VectorXd & a,
  const Eigen::VectorXd & b,
  const double tolerance = 1.0e-10)
{
  if (a.size() != b.size()) {
    return false;
  }

  return (a - b).norm() < tolerance;
}

}  // namespace


int main()
{
  bool success = true;

  constexpr std::size_t N = 3;
  constexpr Eigen::Index nx = 2;
  constexpr Eigen::Index nu = 1;

  // ==========================================================
  // Construct a small LTV system
  // ==========================================================

  std::vector<Eigen::MatrixXd> A(N);
  std::vector<Eigen::MatrixXd> B(N);
  std::vector<Eigen::VectorXd> W(N);

  A.at(0).resize(nx, nx);
  A.at(0) <<
    1.0, 0.1,
    0.0, 1.0;

  A.at(1).resize(nx, nx);
  A.at(1) <<
    1.0, 0.2,
    0.0, 0.9;

  A.at(2).resize(nx, nx);
  A.at(2) <<
    0.95, 0.15,
    0.0, 1.05;

  B.at(0).resize(nx, nu);
  B.at(0) <<
    0.0,
    0.1;

  B.at(1).resize(nx, nu);
  B.at(1) <<
    0.02,
    0.15;

  B.at(2).resize(nx, nu);
  B.at(2) <<
    0.01,
    0.12;

  W.at(0).resize(nx);
  W.at(0) <<
    0.01,
    -0.02;

  W.at(1).resize(nx);
  W.at(1) <<
    0.02,
    0.01;

  W.at(2).resize(nx);
  W.at(2) <<
    -0.01,
    0.03;

  // ==========================================================
  // Build extended matrices
  // ==========================================================

  const auto prediction =
    PredictionMatrixBuilder::build(
      A,
      B,
      W);

  std::cout
    << std::fixed
    << std::setprecision(8);

  std::cout
    << "\n=== DIMENSIONS ===\n";

  std::cout
    << "Aex = "
    << prediction.Aex.rows()
    << " x "
    << prediction.Aex.cols()
    << "\n";

  std::cout
    << "Bex = "
    << prediction.Bex.rows()
    << " x "
    << prediction.Bex.cols()
    << "\n";

  std::cout
    << "Wex = "
    << prediction.Wex.size()
    << " x 1\n";

  success &=
    prediction.Aex.rows() ==
    static_cast<Eigen::Index>(N) * nx;

  success &=
    prediction.Aex.cols() ==
    nx;

  success &=
    prediction.Bex.rows() ==
    static_cast<Eigen::Index>(N) * nx;

  success &=
    prediction.Bex.cols() ==
    static_cast<Eigen::Index>(N) * nu;

  success &=
    prediction.Wex.size() ==
    static_cast<Eigen::Index>(N) * nx;

  // ==========================================================
  // Verify first block row analytically
  // ==========================================================

  const Eigen::MatrixXd Aex0 =
    prediction.Aex.block(
      0,
      0,
      nx,
      nx);

  const Eigen::MatrixXd expected_Aex0 =
    A.at(0);

  success &=
    matrixApproximatelyEqual(
      Aex0,
      expected_Aex0);

  const Eigen::MatrixXd Bex0 =
    prediction.Bex.block(
      0,
      0,
      nx,
      static_cast<Eigen::Index>(N) * nu);

  Eigen::MatrixXd expected_Bex0 =
    Eigen::MatrixXd::Zero(
      nx,
      static_cast<Eigen::Index>(N) * nu);

  expected_Bex0.block(
    0,
    0,
    nx,
    nu) =
    B.at(0);

  success &=
    matrixApproximatelyEqual(
      Bex0,
      expected_Bex0);

  // ==========================================================
  // Verify second block row analytically
  // ==========================================================

  const Eigen::MatrixXd expected_Aex1 =
    A.at(1) *
    A.at(0);

  const Eigen::MatrixXd Aex1 =
    prediction.Aex.block(
      nx,
      0,
      nx,
      nx);

  success &=
    matrixApproximatelyEqual(
      Aex1,
      expected_Aex1);

  Eigen::MatrixXd expected_Bex1 =
    Eigen::MatrixXd::Zero(
      nx,
      static_cast<Eigen::Index>(N) * nu);

  expected_Bex1.block(
    0,
    0,
    nx,
    nu) =
    A.at(1) *
    B.at(0);

  expected_Bex1.block(
    0,
    nu,
    nx,
    nu) =
    B.at(1);

  const Eigen::MatrixXd Bex1 =
    prediction.Bex.block(
      nx,
      0,
      nx,
      static_cast<Eigen::Index>(N) * nu);

  success &=
    matrixApproximatelyEqual(
      Bex1,
      expected_Bex1);

  const Eigen::VectorXd expected_Wex1 =
    A.at(1) *
    W.at(0) +
    W.at(1);

  const Eigen::VectorXd Wex1 =
    prediction.Wex.segment(
      nx,
      nx);

  success &=
    vectorApproximatelyEqual(
      Wex1,
      expected_Wex1);

  // ==========================================================
  // Strong test:
  //
  // compare extended prediction against direct rollout
  // ==========================================================

  Eigen::VectorXd x0(nx);

  x0 <<
    1.0,
    -0.5;

  Eigen::VectorXd U(
    static_cast<Eigen::Index>(N) * nu);

  U <<
    0.2,
    -0.1,
    0.3;

  const Eigen::VectorXd X_extended =
    prediction.Aex *
    x0 +
    prediction.Bex *
    U +
    prediction.Wex;

  Eigen::VectorXd X_direct(
    static_cast<Eigen::Index>(N) * nx);

  Eigen::VectorXd x =
    x0;

  for (std::size_t k = 0; k < N; ++k) {
    const Eigen::VectorXd u =
      U.segment(
        static_cast<Eigen::Index>(k) * nu,
        nu);

    x =
      A.at(k) *
      x +
      B.at(k) *
      u +
      W.at(k);

    X_direct.segment(
      static_cast<Eigen::Index>(k) * nx,
      nx) =
      x;
  }

  const double prediction_error =
    (X_extended - X_direct).norm();

  std::cout
    << "\n=== EXTENDED PREDICTION ===\n";

  std::cout
    << "X_extended:\n"
    << X_extended
    << "\n\n";

  std::cout
    << "X_direct:\n"
    << X_direct
    << "\n\n";

  std::cout
    << "prediction error = "
    << prediction_error
    << "\n";

  success &=
    prediction_error < 1.0e-10;

  // ==========================================================
  // Result
  // ==========================================================

  std::cout
    << "\n====================================\n";

  if (success) {
    std::cout
      << "PREDICTION MATRIX SANITY CHECK: PASS\n";

    std::cout
      << "====================================\n";

    return 0;
  }

  std::cout
    << "PREDICTION MATRIX SANITY CHECK: FAIL\n";

  std::cout
    << "====================================\n";

  return 1;
}
