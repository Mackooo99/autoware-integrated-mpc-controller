#include "autoware/integrated_mpc_trajectory_follower/delay_compensation.hpp"

#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>

using autoware::integrated_mpc::propagateFirstOrderDelay;
using autoware::integrated_mpc::propagateIntegratorDelay;

namespace
{

bool approximatelyEqual(
  const double value,
  const double expected,
  const double tolerance = 1.0e-9)
{
  return std::abs(value - expected) < tolerance;
}

}  // namespace

int main()
{
  bool success = true;

  std::cout
    << std::fixed
    << std::setprecision(9);

  // ========================================================
  // TEST 1 - Empty history is a no-op
  //
  // No commands published yet -> the propagated state must equal the
  // measured state exactly, for both channels.
  // ========================================================

  {
    const std::deque<double> empty_history;

    const double steer_result =
      propagateFirstOrderDelay(
        0.05, empty_history, 0.27, 0.03);

    const double accel_result =
      propagateIntegratorDelay(
        0.3, empty_history, 0.03);

    std::cout << "\n=== TEST 1: empty history is a no-op ===\n";
    std::cout << "steer result : " << steer_result << "\n";
    std::cout << "accel result : " << accel_result << "\n";

    success &= approximatelyEqual(steer_result, 0.05);
    success &= approximatelyEqual(accel_result, 0.3);
  }

  // ========================================================
  // TEST 2 - Steering: constant command drives delta toward that
  // command (first-order ZOH), independent of the starting value.
  //
  // With a long enough constant-command history, delta must converge
  // arbitrarily close to the commanded value.
  // ========================================================

  {
    std::deque<double> history;

    for (int i = 0; i < 200; ++i) {
      history.push_back(0.1);
    }

    const double result =
      propagateFirstOrderDelay(
        0.0, history, 0.27, 0.03);

    std::cout << "\n=== TEST 2: steering converges to constant command ===\n";
    std::cout << "result   : " << result << "\n";
    std::cout << "expected : 0.1 (approx)\n";

    success &= approximatelyEqual(result, 0.1, 1.0e-6);
  }

  // ========================================================
  // TEST 3 - Steering: single-step exact value
  //
  // delta_1 = exp(-dt/tau)*delta_0 + (1-exp(-dt/tau))*cmd_0
  // ========================================================

  {
    constexpr double delta0 = 0.02;
    constexpr double cmd0 = 0.08;
    constexpr double tau = 0.27;
    constexpr double dt = 0.03;

    std::deque<double> history{cmd0};

    const double result =
      propagateFirstOrderDelay(
        delta0, history, tau, dt);

    const double decay = std::exp(-dt / tau);
    const double expected = decay * delta0 + (1.0 - decay) * cmd0;

    std::cout << "\n=== TEST 3: steering single-step exact value ===\n";
    std::cout << "result   : " << result << "\n";
    std::cout << "expected : " << expected << "\n";

    success &= approximatelyEqual(result, expected);
  }

  // ========================================================
  // TEST 4 - Acceleration: constant jerk over N steps is exact
  // integration: a_N = a_0 + j*N*dt
  // ========================================================

  {
    constexpr double a0 = 0.5;
    constexpr double jerk = 2.0;
    constexpr double dt = 0.03;
    constexpr int steps = 8;

    std::deque<double> history;

    for (int i = 0; i < steps; ++i) {
      history.push_back(jerk);
    }

    const double result =
      propagateIntegratorDelay(
        a0, history, dt);

    const double expected =
      a0 + jerk * static_cast<double>(steps) * dt;

    std::cout << "\n=== TEST 4: acceleration exact integration ===\n";
    std::cout << "result   : " << result << "\n";
    std::cout << "expected : " << expected << "\n";

    success &= approximatelyEqual(result, expected);
  }

  // ========================================================
  // TEST 5 - Order matters: history is oldest-first
  //
  // A history of [cmd_a, cmd_b] must NOT give the same result as
  // [cmd_b, cmd_a] whenever cmd_a != cmd_b - propagation is not
  // commutative, so this also catches an accidental reversal.
  // ========================================================

  {
    std::deque<double> forward{0.05, -0.05};
    std::deque<double> reversed{-0.05, 0.05};

    const double result_forward =
      propagateFirstOrderDelay(
        0.0, forward, 0.27, 0.03);

    const double result_reversed =
      propagateFirstOrderDelay(
        0.0, reversed, 0.27, 0.03);

    std::cout << "\n=== TEST 5: history order matters ===\n";
    std::cout << "forward  : " << result_forward << "\n";
    std::cout << "reversed : " << result_reversed << "\n";

    success &= std::abs(result_forward - result_reversed) > 1.0e-6;
  }

  std::cout
    << "\n====================================\n";

  if (success) {
    std::cout
      << "DELAY COMPENSATION SANITY CHECK: PASS\n";

    std::cout
      << "====================================\n";

    return 0;
  }

  std::cout
    << "DELAY COMPENSATION SANITY CHECK: FAIL\n";

  std::cout
    << "====================================\n";

  return 1;
}
