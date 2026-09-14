#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace autoware::integrated_mpc
{

struct ExtendedPredictionMatrices
{
  Eigen::MatrixXd Aex;
  Eigen::MatrixXd Bex;
  Eigen::VectorXd Wex;

  std::size_t horizon{0};
  std::size_t state_dimension{0};
  std::size_t input_dimension{0};
};

class PredictionMatrixBuilder
{
public:
  static ExtendedPredictionMatrices build(
    const std::vector<Eigen::MatrixXd> & A,
    const std::vector<Eigen::MatrixXd> & B,
    const std::vector<Eigen::VectorXd> & W);
};

}  // namespace autoware::integrated_mpc
