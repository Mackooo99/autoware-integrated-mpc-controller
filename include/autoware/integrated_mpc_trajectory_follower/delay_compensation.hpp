#pragma once

#include <cmath>
#include <deque>

namespace autoware::integrated_mpc
{

// Exact ZOH propagation of a first-order actuator state
// (dot(x) = (u - x) / tau) through a sequence of piecewise-constant
// commands u_0 .. u_{N-1} (oldest first), each held for `dt` seconds -
// i.e. the commands actually published over the last N*dt seconds. Used
// to compensate a pure actuator input delay of about N*dt seconds by
// predicting where the state will be once those already-published,
// still-in-flight commands take effect, matching stock Autoware's
// SteeringPredictor::calcSteerPrediction concept but restricted to the
// commands actually published rather than an internally dead-reckoned
// running state.
inline double propagateFirstOrderDelay(
  const double measured_state,
  const std::deque<double> & command_history,
  const double tau,
  const double dt)
{
  double state = measured_state;
  const double decay = std::exp(-dt / tau);

  for (const double command : command_history) {
    state = decay * state + (1.0 - decay) * command;
  }

  return state;
}

// Exact ZOH propagation of a pure-integrator state (dot(x) = u) through
// a sequence of piecewise-constant commands, each held for `dt` seconds.
inline double propagateIntegratorDelay(
  const double measured_state,
  const std::deque<double> & command_history,
  const double dt)
{
  double state = measured_state;

  for (const double command : command_history) {
    state += command * dt;
  }

  return state;
}

}  // namespace autoware::integrated_mpc
