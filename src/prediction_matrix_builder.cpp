#include "autoware/integrated_mpc_trajectory_follower/prediction_matrix_builder.hpp"

#include <Eigen/Core>

#include <stdexcept>

namespace autoware::integrated_mpc
{

ExtendedPredictionMatrices PredictionMatrixBuilder::build(
  const std::vector<Eigen::MatrixXd> & A,
  const std::vector<Eigen::MatrixXd> & B,
  const std::vector<Eigen::VectorXd> & W)
{
  // ==========================================================
  // Basic validation
  // ==========================================================

  if (A.empty()) {
    throw std::invalid_argument(
      "PredictionMatrixBuilder: A must not be empty");
  }

  if (
    A.size() != B.size() ||
    A.size() != W.size())
  {
    throw std::invalid_argument(
      "PredictionMatrixBuilder: A, B and W must have equal horizon length");
  }

  const std::size_t N = A.size();

  const Eigen::Index nx =
    A.front().rows();

  const Eigen::Index nu =
    B.front().cols();

  if (nx <= 0 || nu <= 0) {
    throw std::invalid_argument(
      "PredictionMatrixBuilder: invalid state or input dimension");
  }

  // ==========================================================
  // Check dimensions of every LTV model
  // ==========================================================

  for (std::size_t k = 0; k < N; ++k) {
    if (
      A.at(k).rows() != nx ||
      A.at(k).cols() != nx)
    {
      throw std::invalid_argument(
        "PredictionMatrixBuilder: invalid A matrix dimension");
    }

    if (
      B.at(k).rows() != nx ||
      B.at(k).cols() != nu)
    {
      throw std::invalid_argument(
        "PredictionMatrixBuilder: invalid B matrix dimension");
    }

    if (W.at(k).size() != nx) {
      throw std::invalid_argument(
        "PredictionMatrixBuilder: invalid W vector dimension");
    }
  }

  // ==========================================================
  // Allocate extended matrices
  // ==========================================================

  ExtendedPredictionMatrices result;

  result.horizon = N;
  result.state_dimension =
    static_cast<std::size_t>(nx);

  result.input_dimension =
    static_cast<std::size_t>(nu);

  result.Aex =
    Eigen::MatrixXd::Zero(
      static_cast<Eigen::Index>(N) * nx,
      nx);

  result.Bex =
    Eigen::MatrixXd::Zero(
      static_cast<Eigen::Index>(N) * nx,
      static_cast<Eigen::Index>(N) * nu);

  result.Wex =
    Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(N) * nx);

  // ==========================================================
  // Recursive construction
  //
  // x_{k+1} =
  //   Aacc*x0 +
  //   Bacc*U +
  //   Wacc
  // ==========================================================

  Eigen::MatrixXd Aacc =
    Eigen::MatrixXd::Identity(
      nx,
      nx);

  Eigen::MatrixXd Bacc =
    Eigen::MatrixXd::Zero(
      nx,
      static_cast<Eigen::Index>(N) * nu);

  Eigen::VectorXd Wacc =
    Eigen::VectorXd::Zero(nx);

  for (std::size_t k = 0; k < N; ++k) {
    const Eigen::Index row =
      static_cast<Eigen::Index>(k) * nx;

    const Eigen::Index input_column =
      static_cast<Eigen::Index>(k) * nu;

    /*
     * Previous input effects are propagated through A_k.
     */
    Bacc =
      A.at(k) *
      Bacc;

    /*
     * Current input u_k directly enters through B_k.
     */
    Bacc.block(
      0,
      input_column,
      nx,
      nu) =
      B.at(k);

    /*
     * Affine offset propagation.
     */
    Wacc =
      A.at(k) *
      Wacc +
      W.at(k);

    /*
     * Initial-state propagation.
     */
    Aacc =
      A.at(k) *
      Aacc;

    // ========================================================
    // Store block row for x_{k+1}
    // ========================================================

    result.Aex.block(
      row,
      0,
      nx,
      nx) =
      Aacc;

    result.Bex.block(
      row,
      0,
      nx,
      static_cast<Eigen::Index>(N) * nu) =
      Bacc;

    result.Wex.segment(
      row,
      nx) =
      Wacc;
  }

  return result;
}

}  // namespace autoware::integrated_mpc
