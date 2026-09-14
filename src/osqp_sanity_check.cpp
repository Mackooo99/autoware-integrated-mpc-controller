#include "autoware/osqp_interface/osqp_interface.hpp"

#include <Eigen/Core>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

int main()
{
  bool success = true;

  // ==========================================================
  // QP:
  //
  // minimize:
  //
  //   0.5 * x^T P x + q^T x
  //
  // subject to:
  //
  //   l <= A x <= u
  //
  // Analytical solution:
  //
  //   x* = [1, 2]
  //   J* = -5
  // ==========================================================

  Eigen::MatrixXd P =
    Eigen::MatrixXd::Zero(
      2,
      2);

  P(0, 0) = 2.0;
  P(1, 1) = 2.0;

  Eigen::VectorXd q_eigen(2);

  q_eigen <<
    -2.0,
    -4.0;

  Eigen::MatrixXd A =
    Eigen::MatrixXd::Identity(
      2,
      2);

  Eigen::VectorXd lower_eigen(2);
  Eigen::VectorXd upper_eigen(2);

  lower_eigen <<
    0.0,
    0.0;

  upper_eigen <<
    10.0,
    10.0;

  // ==========================================================
  // Autoware OSQP interface expects std::vector<double>
  // for q, lower and upper.
  // ==========================================================

  const std::vector<double> q{
    q_eigen(0),
    q_eigen(1)};

  const std::vector<double> lower{
    lower_eigen(0),
    lower_eigen(1)};

  const std::vector<double> upper{
    upper_eigen(0),
    upper_eigen(1)};

  // ==========================================================
  // Solver
  //
  // eps_abs = 1e-6
  // polish  = true
  // ==========================================================

  autoware::osqp_interface::OSQPInterface solver(
    1.0e-6,
    true);

  const auto result =
    solver.optimize(
      P,
      A,
      q,
      lower,
      upper);

  // ==========================================================
  // Solver information
  // ==========================================================

  const std::string status =
    solver.getStatusMessage();

  const auto status_value =
    solver.getStatus();

  const auto iterations =
    solver.getTakenIter();

  const double runtime =
    solver.getRunTime();

  const double objective =
    solver.getObjVal();

  const auto exit_flag =
    solver.getExitFlag();

  std::cout
    << std::fixed
    << std::setprecision(10);

  std::cout
    << "\n=== OSQP STATUS ===\n";

  std::cout
    << "status          = "
    << status
    << "\n";

  std::cout
    << "status value    = "
    << status_value
    << "\n";

  std::cout
    << "iterations      = "
    << iterations
    << "\n";

  std::cout
    << "runtime         = "
    << runtime * 1.0e3
    << " ms\n";

  std::cout
    << "objective       = "
    << objective
    << "\n";

  std::cout
    << "exit flag       = "
    << exit_flag
    << "\n";

  // ==========================================================
  // Solution validation
  // ==========================================================

  if (result.primal_solution.size() != 2) {
    std::cerr
      << "\nUnexpected primal solution size: "
      << result.primal_solution.size()
      << "\n";

    return 1;
  }

  const double x0 =
    result.primal_solution.at(0);

  const double x1 =
    result.primal_solution.at(1);

  std::cout
    << "\n=== SOLUTION ===\n";

  std::cout
    << "x[0] = "
    << x0
    << " expected 1.0\n";

  std::cout
    << "x[1] = "
    << x1
    << " expected 2.0\n";

  // ==========================================================
  // Compute objective independently
  // ==========================================================

  Eigen::VectorXd x(2);

  x <<
    x0,
    x1;

  const double direct_objective =
    0.5 *
    x.dot(
      P *
      x)
    +
    q_eigen.dot(
      x);

  std::cout
    << "\n=== OBJECTIVE CHECK ===\n";

  std::cout
    << "solver objective = "
    << objective
    << "\n";

  std::cout
    << "direct objective = "
    << direct_objective
    << "\n";

  std::cout
    << "expected         = -5.0000000000\n";

  // ==========================================================
  // Constraint validation
  // ==========================================================

  const Eigen::VectorXd Ax =
    A *
    x;

  bool constraints_satisfied = true;

  for (Eigen::Index i = 0; i < Ax.size(); ++i) {
    if (
      Ax(i) <
        lower_eigen(i) -
        1.0e-6 ||
      Ax(i) >
        upper_eigen(i) +
        1.0e-6)
    {
      constraints_satisfied = false;
    }
  }

  std::cout
    << "\n=== CONSTRAINT CHECK ===\n";

  std::cout
    << "constraints satisfied = "
    << (
      constraints_satisfied ?
      "true" :
      "false"
    )
    << "\n";

  // ==========================================================
  // Numerical checks
  // ==========================================================

  const double solution_error =
    std::sqrt(
      std::pow(
        x0 - 1.0,
        2.0)
      +
      std::pow(
        x1 - 2.0,
        2.0));

  const double objective_error =
    std::abs(
      direct_objective -
      (-5.0));

  const bool finite =
    std::isfinite(x0) &&
    std::isfinite(x1) &&
    std::isfinite(objective) &&
    std::isfinite(runtime);

  /*
   * Status text can be "solved" or, depending on tolerance,
   * "solved inaccurate". For this trivial polished problem we
   * normally expect simply "solved".
   */
  const bool solved =
    status.find("solved") !=
    std::string::npos;

  std::cout
    << "\n=== NUMERICAL CHECK ===\n";

  std::cout
    << "solution error   = "
    << solution_error
    << "\n";

  std::cout
    << "objective error  = "
    << objective_error
    << "\n";

  std::cout
    << "finite           = "
    << (
      finite ?
      "true" :
      "false"
    )
    << "\n";

  std::cout
    << "solved           = "
    << (
      solved ?
      "true" :
      "false"
    )
    << "\n";

  success &=
    finite;

  success &=
    solved;

  success &=
    constraints_satisfied;

  success &=
    solution_error <
    1.0e-5;

  success &=
    objective_error <
    1.0e-5;

  // ==========================================================
  // Result
  // ==========================================================

  std::cout
    << "\n====================================\n";

  if (success) {
    std::cout
      << "OSQP SANITY CHECK: PASS\n";

    std::cout
      << "====================================\n";

    return 0;
  }

  std::cout
    << "OSQP SANITY CHECK: FAIL\n";

  std::cout
    << "====================================\n";

  return 1;
}
