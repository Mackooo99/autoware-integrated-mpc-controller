#include "autoware/integrated_mpc_trajectory_follower/mpc_constraint_builder.hpp"
#include "autoware/integrated_mpc_trajectory_follower/prediction_matrix_builder.hpp"

#include <Eigen/Core>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>


using autoware::integrated_mpc::MpcConstraintBuilder;
using autoware::integrated_mpc::MpcConstraintParameters;
using autoware::integrated_mpc::PredictionMatrixBuilder;
using autoware::integrated_mpc::PredictionReferenceHorizon;
using autoware::integrated_mpc::PredictionReferencePoint;


int main()
{
  bool success = true;

  constexpr std::size_t N = 3;

  constexpr Eigen::Index NX = 5;
  constexpr Eigen::Index NU = 2;

  constexpr double DEG2RAD =
    0.017453292519943295769;

  constexpr double RAD2DEG =
    57.295779513082320876;

  constexpr double TOL =
    1.0e-10;


  // ==========================================================
  // Prediction model
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

    /*
     * Steering command influences steering state.
     */
    B.at(i)(3, 0) =
      0.3;

    /*
     * Jerk integrates into acceleration:
     *
     * a(k+1) = a(k) + 0.1 * j(k)
     */
    B.at(i)(4, 1) =
      0.1;

    /*
     * Acceleration influences velocity error.
     */
    A.at(i)(2, 4) =
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

  /*
   * Prediction sampling interval.
   *
   * IMPORTANT:
   * This is used for steering-rate constraints between
   * future MPC inputs k >= 1.
   */
  reference.prediction_dt =
    0.1;

  PredictionReferencePoint r0;

  r0.velocity =
    0.0;

  r0.curvature =
    0.0;


  PredictionReferencePoint r1;

  r1.velocity =
    5.0;

  r1.curvature =
    0.005;


  PredictionReferencePoint r2;

  r2.velocity =
    10.0;

  r2.curvature =
    0.01;


  reference.points =
    {
      r0,
      r1,
      r2
    };


  // ==========================================================
  // Initial state
  //
  // x =
  //
  // [e_y, e_psi, e_v, delta, a]^T
  // ==========================================================

  Eigen::VectorXd x0(NX);

  x0 <<
    0.0,
    0.0,
    0.0,
    0.10,
    0.50;


  /*
   * Previous steering command.
   *
   * This is the steering command from which the first
   * steering-rate constraint is constructed.
   */
  const double previous_steering_command =
    0.10;


  // ==========================================================
  // Controller period
  //
  // IMPORTANT:
  //
  // First steering-rate constraint:
  //
  //   delta_cmd(0) - delta_prev
  //
  // uses ctrl_period = 0.03 s.
  //
  // Subsequent steering-rate constraints:
  //
  //   delta_cmd(k) - delta_cmd(k-1)
  //
  // use prediction_dt = 0.1 s.
  // ==========================================================

  const double control_period =
    0.03;


  // ==========================================================
  // Constraint parameters
  // ==========================================================

  MpcConstraintParameters parameters;

  parameters.max_steering_angle =
    0.7;

  parameters.min_acceleration =
    -1.0;

  parameters.max_acceleration =
    1.0;

  parameters.min_jerk =
    -5.0;

  parameters.max_jerk =
    2.0;


  /*
   * Simple steering-rate maps chosen so the interpolated
   * values are easy to verify analytically.
   */
  parameters.curvature_for_steer_rate =
    {
      0.0,
      0.01
    };

  parameters.steer_rate_deg_by_curvature =
    {
      40.0,
      60.0
    };

  parameters.velocity_for_steer_rate =
    {
      0.0,
      10.0
    };

  parameters.steer_rate_deg_by_velocity =
    {
      60.0,
      40.0
    };


  // ==========================================================
  // Build constraints
  // ==========================================================

  const auto constraints =
    MpcConstraintBuilder::build(
      prediction,
      reference,
      x0,
      previous_steering_command,
      control_period,
      parameters);


  std::cout
    << std::fixed
    << std::setprecision(10);


  // ==========================================================
  // Dimensions
  //
  // Constraint groups:
  //
  // [0, N)       steering angle
  // [N, 2N)      jerk
  // [2N, 3N)     acceleration state
  // [3N, 4N)     steering rate
  //
  // N = 3
  //
  // therefore:
  //
  // rows = 12
  // cols = NU*N = 6
  // ==========================================================

  std::cout
    << "\n=== CONSTRAINT DIMENSIONS ===\n";

  std::cout
    << "A     = "
    << constraints.A.rows()
    << " x "
    << constraints.A.cols()
    << "\n";

  std::cout
    << "lower = "
    << constraints.lower.size()
    << " x 1\n";

  std::cout
    << "upper = "
    << constraints.upper.size()
    << " x 1\n";

  const bool dimensions_correct =
    constraints.A.rows() == 12 &&
    constraints.A.cols() == 6 &&
    constraints.lower.size() == 12 &&
    constraints.upper.size() == 12;

  std::cout
    << "dimensions correct = "
    << (
      dimensions_correct ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    dimensions_correct;


  // ==========================================================
  // Steering angle constraint
  // ==========================================================

  std::cout
    << "\n=== STEERING ANGLE CHECK ===\n";

  std::cout
    << "A(0,0) = "
    << constraints.A(0, 0)
    << "\n";

  std::cout
    << "bounds = ["
    << constraints.lower(0)
    << ", "
    << constraints.upper(0)
    << "]\n";

  const bool steering_angle_correct =
    std::abs(
      constraints.A(0, 0) -
      1.0) <
      TOL &&
    std::abs(
      constraints.lower(0) +
      0.7) <
      TOL &&
    std::abs(
      constraints.upper(0) -
      0.7) <
      TOL;

  std::cout
    << "steering angle correct = "
    << (
      steering_angle_correct ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    steering_angle_correct;


  // ==========================================================
  // Jerk constraint
  // ==========================================================

  std::cout
    << "\n=== JERK CHECK ===\n";

  std::cout
    << "A(3,1) = "
    << constraints.A(3, 1)
    << "\n";

  std::cout
    << "bounds = ["
    << constraints.lower(3)
    << ", "
    << constraints.upper(3)
    << "]\n";

  const bool jerk_correct =
    std::abs(
      constraints.A(3, 1) -
      1.0) <
      TOL &&
    std::abs(
      constraints.lower(3) +
      5.0) <
      TOL &&
    std::abs(
      constraints.upper(3) -
      2.0) <
      TOL;

  std::cout
    << "jerk constraint correct = "
    << (
      jerk_correct ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    jerk_correct;


  // ==========================================================
  // Acceleration state constraint
  //
  // Initial acceleration:
  //
  //   a0 = 0.5
  //
  // First predicted acceleration:
  //
  //   a1 = 0.5 + 0.1*j0
  //
  // Required:
  //
  //   -1 <= a1 <= 1
  //
  // therefore:
  //
  //   -1.5 <= 0.1*j0 <= 0.5
  // ==========================================================

  std::cout
    << "\n=== ACCELERATION STATE CHECK ===\n";

  std::cout
    << "A(6,1) = "
    << constraints.A(6, 1)
    << " expected 0.1\n";

  std::cout
    << "bounds = ["
    << constraints.lower(6)
    << ", "
    << constraints.upper(6)
    << "] expected [-1.5, 0.5]\n";

  const bool acceleration_correct =
    std::abs(
      constraints.A(6, 1) -
      0.1) <
      TOL &&
    std::abs(
      constraints.lower(6) +
      1.5) <
      TOL &&
    std::abs(
      constraints.upper(6) -
      0.5) <
      TOL;

  std::cout
    << "acceleration constraint correct = "
    << (
      acceleration_correct ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    acceleration_correct;


  // ==========================================================
  // Steering-rate interpolation
  //
  // step 0:
  //
  //   curvature -> 40 deg/s
  //   velocity  -> 60 deg/s
  //   min       -> 40 deg/s
  //
  // step 1:
  //
  //   curvature -> 50 deg/s
  //   velocity  -> 50 deg/s
  //   min       -> 50 deg/s
  //
  // step 2:
  //
  //   curvature -> 60 deg/s
  //   velocity  -> 40 deg/s
  //   min       -> 40 deg/s
  // ==========================================================

  std::cout
    << "\n=== STEERING RATE LIMITS ===\n";

  for (
    Eigen::Index i = 0;
    i < 3;
    ++i)
  {
    std::cout
      << "step "
      << i
      << " = "
      << constraints.
           steering_rate_limit(i) *
           RAD2DEG
      << " deg/s\n";
  }

  const bool rate_step_0_correct =
    std::abs(
      constraints.
        steering_rate_limit(0) *
        RAD2DEG -
      40.0) <
      TOL;

  const bool rate_step_1_correct =
    std::abs(
      constraints.
        steering_rate_limit(1) *
        RAD2DEG -
      50.0) <
      TOL;

  const bool rate_step_2_correct =
    std::abs(
      constraints.
        steering_rate_limit(2) *
        RAD2DEG -
      40.0) <
      TOL;

  std::cout
    << "rate step 0 correct = "
    << (
      rate_step_0_correct ?
      "true" :
      "false"
    )
    << "\n";

  std::cout
    << "rate step 1 correct = "
    << (
      rate_step_1_correct ?
      "true" :
      "false"
    )
    << "\n";

  std::cout
    << "rate step 2 correct = "
    << (
      rate_step_2_correct ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    rate_step_0_correct;

  success &=
    rate_step_1_correct;

  success &=
    rate_step_2_correct;


  // ==========================================================
  // First steering-rate bound
  //
  // IMPORTANT:
  //
  // First MPC input is applied after ctrl_period, not after
  // prediction_dt.
  //
  // previous steer:
  //
  //   delta_prev = 0.1 rad
  //
  // steering-rate limit:
  //
  //   40 deg/s
  //
  // control period:
  //
  //   0.03 s
  //
  // allowed change:
  //
  //   40 deg/s * 0.03 s
  //   = 1.2 deg
  //   = 0.020943951 rad
  //
  // therefore:
  //
  //   lower = 0.1 - 0.020943951
  //         = 0.079056049
  //
  //   upper = 0.1 + 0.020943951
  //         = 0.120943951
  // ==========================================================

  const double expected_first_change =
    40.0 *
    DEG2RAD *
    control_period;

  const double expected_first_lower =
    previous_steering_command -
    expected_first_change;

  const double expected_first_upper =
    previous_steering_command +
    expected_first_change;

  std::cout
    << "\n=== FIRST STEERING RATE BOUND ===\n";

  std::cout
    << "actual lower   = "
    << constraints.lower(9)
    << "\n";

  std::cout
    << "expected lower = "
    << expected_first_lower
    << "\n";

  std::cout
    << "actual upper   = "
    << constraints.upper(9)
    << "\n";

  std::cout
    << "expected upper = "
    << expected_first_upper
    << "\n";

  const bool first_lower_correct =
    std::abs(
      constraints.lower(9) -
      expected_first_lower) <
    TOL;

  const bool first_upper_correct =
    std::abs(
      constraints.upper(9) -
      expected_first_upper) <
    TOL;

  std::cout
    << "first lower correct = "
    << (
      first_lower_correct ?
      "true" :
      "false"
    )
    << "\n";

  std::cout
    << "first upper correct = "
    << (
      first_upper_correct ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    first_lower_correct;

  success &=
    first_upper_correct;


  // ==========================================================
  // Second steering-rate constraint
  //
  // This confirms that only the FIRST step uses ctrl_period.
  //
  // Step 1 rate:
  //
  //   50 deg/s
  //
  // prediction_dt:
  //
  //   0.1 s
  //
  // allowed difference:
  //
  //   5 deg = 0.08726646 rad
  //
  // Constraint is:
  //
  //   -limit <= delta1 - delta0 <= limit
  // ==========================================================

  const double expected_second_change =
    50.0 *
    DEG2RAD *
    reference.prediction_dt;

  std::cout
    << "\n=== SECOND STEERING RATE CHECK ===\n";

  std::cout
    << "A(10,0) = "
    << constraints.A(10, 0)
    << " expected -1\n";

  std::cout
    << "A(10,2) = "
    << constraints.A(10, 2)
    << " expected +1\n";

  std::cout
    << "bounds = ["
    << constraints.lower(10)
    << ", "
    << constraints.upper(10)
    << "]\n";

  std::cout
    << "expected = ["
    << -expected_second_change
    << ", "
    << expected_second_change
    << "]\n";

  const bool second_rate_correct =
    std::abs(
      constraints.A(10, 0) +
      1.0) <
      TOL &&
    std::abs(
      constraints.A(10, 2) -
      1.0) <
      TOL &&
    std::abs(
      constraints.lower(10) +
      expected_second_change) <
      TOL &&
    std::abs(
      constraints.upper(10) -
      expected_second_change) <
      TOL;

  std::cout
    << "second steering-rate constraint correct = "
    << (
      second_rate_correct ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    second_rate_correct;


  // ==========================================================
  // Feasible control vector
  //
  // U =
  //
  // [delta0, j0,
  //  delta1, j1,
  //  delta2, j2]
  //
  // Constant steering at 0.10 rad:
  //
  // - satisfies first rate constraint
  // - subsequent steering changes are zero
  //
  // Zero jerk:
  //
  // - acceleration remains 0.5 m/s^2
  // - inside [-1, 1]
  // ==========================================================

  Eigen::VectorXd U(6);

  U <<
    0.10, 0.0,
    0.10, 0.0,
    0.10, 0.0;

  const Eigen::VectorXd constraint_value =
    constraints.A *
    U;

  bool feasible = true;

  for (
    Eigen::Index i = 0;
    i < constraint_value.size();
    ++i)
  {
    if (
      constraint_value(i) <
        constraints.lower(i) -
        TOL ||
      constraint_value(i) >
        constraints.upper(i) +
        TOL)
    {
      feasible = false;

      std::cout
        << "Unexpected violation at row "
        << i
        << ": value="
        << constraint_value(i)
        << " lower="
        << constraints.lower(i)
        << " upper="
        << constraints.upper(i)
        << "\n";
    }
  }

  std::cout
    << "\n=== FEASIBILITY CHECK ===\n";

  std::cout
    << "candidate feasible = "
    << (
      feasible ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    feasible;


  // ==========================================================
  // Intentional absolute steering-angle violation
  //
  // delta0 = 0.8 rad
  //
  // absolute bound is:
  //
  // [-0.7, 0.7]
  // ==========================================================

  Eigen::VectorXd bad_U =
    U;

  bad_U(0) =
    0.8;

  const Eigen::VectorXd bad_value =
    constraints.A *
    bad_U;

  const bool detects_steering_violation =
    bad_value(0) >
    constraints.upper(0);

  std::cout
    << "detect steering violation = "
    << (
      detects_steering_violation ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    detects_steering_violation;


  // ==========================================================
  // Intentional FIRST steering-rate violation
  //
  // delta0 = 0.05 rad
  //
  // This is inside the absolute steering bound [-0.7, 0.7],
  // but below the first rate bound:
  //
  // delta0 >= 0.079056...
  // ==========================================================

  Eigen::VectorXd bad_rate_U =
    U;

  bad_rate_U(0) =
    0.05;

  const Eigen::VectorXd bad_rate_value =
    constraints.A *
    bad_rate_U;

  /*
   * Steering-rate constraints start at row 3*N.
   *
   * N = 3
   *
   * first steering-rate row = 9.
   */
  const bool detects_rate_violation =
    bad_rate_value(9) <
    constraints.lower(9);

  std::cout
    << "detect steering-rate violation = "
    << (
      detects_rate_violation ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    detects_rate_violation;


  // ==========================================================
  // Intentional jerk violation
  //
  // j0 = 3.0 m/s^3
  //
  // maximum allowed:
  //
  // 2.0 m/s^3
  // ==========================================================

  Eigen::VectorXd bad_jerk_U =
    U;

  bad_jerk_U(1) =
    3.0;

  const Eigen::VectorXd bad_jerk_value =
    constraints.A *
    bad_jerk_U;

  const bool detects_jerk_violation =
    bad_jerk_value(3) >
    constraints.upper(3);

  std::cout
    << "detect jerk violation = "
    << (
      detects_jerk_violation ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    detects_jerk_violation;


  // ==========================================================
  // Final result
  // ==========================================================

  std::cout
    << "\n====================================\n";

  if (success) {
    std::cout
      << "MPC CONSTRAINT SANITY CHECK: PASS\n";

    std::cout
      << "====================================\n";

    return 0;
  }

  std::cout
    << "MPC CONSTRAINT SANITY CHECK: FAIL\n";

  std::cout
    << "====================================\n";

  return 1;
}
