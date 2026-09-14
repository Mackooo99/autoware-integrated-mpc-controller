#pragma once

#include <Eigen/Core>

namespace autoware::integrated_mpc
{

class IntegratedKinematicsBicycleModel
{
public:
  static constexpr int DIM_X = 5;
  static constexpr int DIM_U = 2;

  using MatrixA = Eigen::Matrix<double, DIM_X, DIM_X>;
  using MatrixB = Eigen::Matrix<double, DIM_X, DIM_U>;

  using VectorX = Eigen::Matrix<double, DIM_X, 1>;
  using VectorU = Eigen::Matrix<double, DIM_U, 1>;

  IntegratedKinematicsBicycleModel(
    double wheel_base,
    double steer_tau);

  void setReference(
    double velocity,
    double curvature,
    double acceleration);

  void setLinearizationPoint(
    const VectorX & x_bar,
    const VectorU & u_bar);

  /**
   * Continuous affine model:
   *
   * x_dot = Ac * x + Bc * u + cc
   */
  void calculateContinuousMatrix(
    MatrixA & Ac,
    MatrixB & Bc,
    VectorX & cc) const;

  /**
   * Discrete affine model:
   *
   * x(k+1) = Ad * x(k) + Bd * u(k) + Wd
   *
   * Exact zero-order-hold discretization of
   * the local affine model.
   */
  void calculateDiscreteMatrix(
    double dt,
    MatrixA & Ad,
    MatrixB & Bd,
    VectorX & Wd) const;

  double calculateReferenceSteer() const;

  double getReferenceVelocity() const;
  double getReferenceCurvature() const;
  double getReferenceAcceleration() const;

private:
  double wheel_base_;
  double steer_tau_;

  double ref_velocity_{0.0};
  double ref_curvature_{0.0};
  double ref_acceleration_{0.0};

  VectorX x_bar_{VectorX::Zero()};
  VectorU u_bar_{VectorU::Zero()};
};

}  // namespace autoware::integrated_mpc
