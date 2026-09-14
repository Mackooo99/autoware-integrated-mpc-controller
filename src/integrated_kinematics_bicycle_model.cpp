#include "autoware/integrated_mpc_trajectory_follower/integrated_kinematics_bicycle_model.hpp"

#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <stdexcept>

namespace autoware::integrated_mpc
{

IntegratedKinematicsBicycleModel::IntegratedKinematicsBicycleModel(
  const double wheel_base,
  const double steer_tau)
: wheel_base_(wheel_base),
  steer_tau_(steer_tau)
{
  if (wheel_base_ <= 0.0) {
    throw std::invalid_argument("wheel_base must be positive");
  }

  if (steer_tau_ <= 0.0) {
    throw std::invalid_argument("steer_tau must be positive");
  }
}

void IntegratedKinematicsBicycleModel::setReference(
  const double velocity,
  const double curvature,
  const double acceleration)
{
  ref_velocity_ = velocity;
  ref_curvature_ = curvature;
  ref_acceleration_ = acceleration;
}

void IntegratedKinematicsBicycleModel::setLinearizationPoint(
  const VectorX & x_bar,
  const VectorU & u_bar)
{
  x_bar_ = x_bar;
  u_bar_ = u_bar;
}

double IntegratedKinematicsBicycleModel::calculateReferenceSteer() const
{
  return std::atan(wheel_base_ * ref_curvature_);
}

double IntegratedKinematicsBicycleModel::getReferenceVelocity() const
{
  return ref_velocity_;
}

double IntegratedKinematicsBicycleModel::getReferenceCurvature() const
{
  return ref_curvature_;
}

double IntegratedKinematicsBicycleModel::getReferenceAcceleration() const
{
  return ref_acceleration_;
}

void IntegratedKinematicsBicycleModel::calculateContinuousMatrix(
  MatrixA & Ac,
  MatrixB & Bc,
  VectorX & cc) const
{
  /*
   * State:
   *
   * x = [ e_y,
   *       e_psi,
   *       e_v,
   *       delta,
   *       a ]^T
   *
   * Input:
   *
   * u = [ delta_cmd,
   *       jerk ]^T
   */

  const double e_y = x_bar_(0);
  const double e_psi = x_bar_(1);
  const double e_v = x_bar_(2);
  const double delta = x_bar_(3);
  const double acceleration = x_bar_(4);

  const double delta_cmd = u_bar_(0);
  const double jerk = u_bar_(1);

  /*
   * Physical vehicle velocity:
   *
   * v = v_ref + e_v
   */
  const double velocity = ref_velocity_ + e_v;

  /*
   * Frenet denominator:
   *
   * D = 1 - kappa_ref * e_y
   */
  const double D = 1.0 - ref_curvature_ * e_y;

  constexpr double min_denominator = 1.0e-6;

  if (std::abs(D) < min_denominator) {
    throw std::runtime_error(
      "Invalid Frenet state: 1 - curvature * lateral_error is too close to zero");
  }

  const double sin_epsi = std::sin(e_psi);
  const double cos_epsi = std::cos(e_psi);

  const double tan_delta = std::tan(delta);
  const double cos_delta = std::cos(delta);

  if (std::abs(cos_delta) < min_denominator) {
    throw std::runtime_error(
      "Invalid steering state: cos(delta) is too close to zero");
  }

  const double sec_delta_squared =
    1.0 / (cos_delta * cos_delta);

  /*
   * Nonlinear continuous model evaluated at
   * the linearization point.
   *
   * f(x_bar, u_bar)
   */
  VectorX f = VectorX::Zero();

  f(0) =
    velocity * sin_epsi;

  f(1) =
    velocity / wheel_base_ * tan_delta -
    ref_curvature_ * velocity * cos_epsi / D;

  f(2) =
    acceleration - ref_acceleration_;

  f(3) =
    (delta_cmd - delta) / steer_tau_;

  f(4) =
    jerk;

  /*
   * Jacobian:
   *
   * Ac = df/dx
   */
  Ac.setZero();

  // d(e_y_dot) / dx
  Ac(0, 1) =
    velocity * cos_epsi;

  Ac(0, 2) =
    sin_epsi;

  // d(e_psi_dot) / dx
  Ac(1, 0) =
    -ref_curvature_ * ref_curvature_ *
    velocity * cos_epsi /
    (D * D);

  Ac(1, 1) =
    ref_curvature_ *
    velocity * sin_epsi /
    D;

  Ac(1, 2) =
    tan_delta / wheel_base_ -
    ref_curvature_ * cos_epsi / D;

  Ac(1, 3) =
    velocity / wheel_base_ *
    sec_delta_squared;

  // d(e_v_dot) / da
  Ac(2, 4) = 1.0;

  // steering actuator
  Ac(3, 3) =
    -1.0 / steer_tau_;

  /*
   * Input Jacobian:
   *
   * Bc = df/du
   */
  Bc.setZero();

  // delta_cmd -> steering state
  Bc(3, 0) =
    1.0 / steer_tau_;

  // jerk -> acceleration state
  Bc(4, 1) =
    1.0;

  /*
   * Affine correction:
   *
   * cc =
   * f(x_bar,u_bar)
   * - Ac*x_bar
   * - Bc*u_bar
   *
   * so that the local affine model exactly
   * reproduces f at the linearization point.
   */
  cc =
    f -
    Ac * x_bar_ -
    Bc * u_bar_;
}

void IntegratedKinematicsBicycleModel::calculateDiscreteMatrix(
  const double dt,
  MatrixA & Ad,
  MatrixB & Bd,
  VectorX & Wd) const
{
  if (dt <= 0.0) {
    throw std::invalid_argument("dt must be positive");
  }

  MatrixA Ac;
  MatrixB Bc;
  VectorX cc;

  calculateContinuousMatrix(
    Ac,
    Bc,
    cc);

  /*
   * Affine continuous system:
   *
   * x_dot = Ac*x + Bc*u + cc
   *
   * Use augmented state:
   *
   * [x]
   * [u]
   * [1]
   *
   * giving:
   *
   *         [ Ac  Bc  cc ]
   * M   =   [ 0   0   0  ]
   *         [ 0   0   0  ]
   *
   * Exact ZOH:
   *
   * exp(M*dt)
   *
   * gives:
   *
   * [ Ad  Bd  Wd ]
   * [ 0   I   0  ]
   * [ 0   0   1  ]
   */

  constexpr int AUG_DIM =
    DIM_X + DIM_U + 1;

  using AugmentedMatrix =
    Eigen::Matrix<double, AUG_DIM, AUG_DIM>;

  AugmentedMatrix M =
    AugmentedMatrix::Zero();

  M.block<DIM_X, DIM_X>(0, 0) =
    Ac;

  M.block<DIM_X, DIM_U>(0, DIM_X) =
    Bc;

  M.block<DIM_X, 1>(
    0,
    DIM_X + DIM_U) =
    cc;

  const AugmentedMatrix Md =
    (M * dt).exp();

  Ad =
    Md.block<DIM_X, DIM_X>(
      0, 0);

  Bd =
    Md.block<DIM_X, DIM_U>(
      0, DIM_X);

  Wd =
    Md.block<DIM_X, 1>(
      0,
      DIM_X + DIM_U);
}

}  // namespace autoware::integrated_mpc
