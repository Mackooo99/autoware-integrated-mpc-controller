#include "autoware/integrated_mpc_trajectory_follower/integrated_kinematics_bicycle_model.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

using autoware::integrated_mpc::IntegratedKinematicsBicycleModel;

namespace
{

bool approximatelyEqual(
  const double value,
  const double expected,
  const double tolerance = 1.0e-6)
{
  return std::abs(value - expected) < tolerance;
}

}  // namespace

int main()
{
  constexpr double wheel_base = 2.79;
  constexpr double steer_tau = 0.27;
  constexpr double dt = 0.1;

  IntegratedKinematicsBicycleModel model(
    wheel_base,
    steer_tau);

  /*
   * First sanity test:
   *
   * straight road
   * v_ref = 5 m/s
   * kappa = 0
   * a_ref = 0
   */
  model.setReference(
    5.0,
    0.0,
    0.0);

  IntegratedKinematicsBicycleModel::VectorX x_bar;
  x_bar <<
    0.0,   // e_y
    0.0,   // e_psi
    0.0,   // e_v
    0.0,   // delta
    0.0;   // acceleration

  IntegratedKinematicsBicycleModel::VectorU u_bar;
  u_bar <<
    0.0,   // delta_cmd
    0.0;   // jerk

  model.setLinearizationPoint(
    x_bar,
    u_bar);

  IntegratedKinematicsBicycleModel::MatrixA Ac;
  IntegratedKinematicsBicycleModel::MatrixB Bc;
  IntegratedKinematicsBicycleModel::VectorX cc;

  model.calculateContinuousMatrix(
    Ac,
    Bc,
    cc);

  IntegratedKinematicsBicycleModel::MatrixA Ad;
  IntegratedKinematicsBicycleModel::MatrixB Bd;
  IntegratedKinematicsBicycleModel::VectorX Wd;

  model.calculateDiscreteMatrix(
    dt,
    Ad,
    Bd,
    Wd);

  std::cout << std::fixed << std::setprecision(8);

  std::cout << "\n=== Ac ===\n";
  std::cout << Ac << "\n";

  std::cout << "\n=== Bc ===\n";
  std::cout << Bc << "\n";

  std::cout << "\n=== cc ===\n";
  std::cout << cc << "\n";

  std::cout << "\n=== Ad ===\n";
  std::cout << Ad << "\n";

  std::cout << "\n=== Bd ===\n";
  std::cout << Bd << "\n";

  std::cout << "\n=== Wd ===\n";
  std::cout << Wd << "\n";

  bool success = true;

  /*
   * TEST 1
   *
   * Exact first-order steering actuator:
   *
   * delta(k+1)
   * =
   * exp(-dt/tau) delta(k)
   * +
   * (1-exp(-dt/tau)) delta_cmd(k)
   */
  const double expected_steer_state =
    std::exp(-dt / steer_tau);

  const double expected_steer_input =
    1.0 - expected_steer_state;

  std::cout << "\n=== Steering actuator check ===\n";

  std::cout
    << "Ad(3,3) = "
    << Ad(3, 3)
    << " expected "
    << expected_steer_state
    << "\n";

  std::cout
    << "Bd(3,0) = "
    << Bd(3, 0)
    << " expected "
    << expected_steer_input
    << "\n";

  success &=
    approximatelyEqual(
      Ad(3, 3),
      expected_steer_state);

  success &=
    approximatelyEqual(
      Bd(3, 0),
      expected_steer_input);

  /*
   * TEST 2
   *
   * acceleration integrator:
   *
   * a(k+1) = a(k) + dt*j(k)
   */
  std::cout << "\n=== Acceleration integrator check ===\n";

  std::cout
    << "Ad(4,4) = "
    << Ad(4, 4)
    << " expected 1.0\n";

  std::cout
    << "Bd(4,1) = "
    << Bd(4, 1)
    << " expected "
    << dt
    << "\n";

  success &=
    approximatelyEqual(
      Ad(4, 4),
      1.0);

  success &=
    approximatelyEqual(
      Bd(4, 1),
      dt);

  /*
   * TEST 3
   *
   * e_v_dot = a
   * a_dot   = j
   *
   * therefore:
   *
   * e_v(k+1)
   * =
   * e_v(k)
   * + dt*a(k)
   * + 0.5*dt^2*j(k)
   */
  const double expected_ev_acc =
    dt;

  const double expected_ev_jerk =
    0.5 * dt * dt;

  std::cout << "\n=== Velocity-error integration check ===\n";

  std::cout
    << "Ad(2,4) = "
    << Ad(2, 4)
    << " expected "
    << expected_ev_acc
    << "\n";

  std::cout
    << "Bd(2,1) = "
    << Bd(2, 1)
    << " expected "
    << expected_ev_jerk
    << "\n";

  success &=
    approximatelyEqual(
      Ad(2, 4),
      expected_ev_acc);

  success &=
    approximatelyEqual(
      Bd(2, 1),
      expected_ev_jerk);

  /*
   * TEST 4
   *
   * At straight-road ideal reference:
   * affine offset must be zero.
   */
  std::cout << "\n=== Affine offset check ===\n";

  std::cout
    << "||Wd|| = "
    << Wd.norm()
    << " expected 0\n";

  success &=
    Wd.norm() < 1.0e-9;

  std::cout << "\n====================================\n";

  if (success) {
    std::cout << "MODEL SANITY CHECK: PASS\n";
    std::cout << "====================================\n";
    return 0;
  }

  std::cout << "MODEL SANITY CHECK: FAIL\n";
  std::cout << "====================================\n";

  return 1;
}
