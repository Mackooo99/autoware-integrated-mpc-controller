"""Tunable MPC cost-weight search space for the integrated MPC (Task 12).

Only cost weights are tuned here - never physical limits (steering/accel/
jerk bounds, steer-rate maps). CLAUDE.md is explicit that physical
constraints must not be tuned; only the V1 cost weights are dimensions
here.
"""

from collections import namedtuple
import math
import random

Dimension = namedtuple("Dimension", ["yaml_key", "low", "high", "log_scale", "default"])

# name -> Dimension. low/high are the coarse-search bounds; default is the
# current V1 value (config/integrated_mpc.param.yaml), used as the coarse
# search's implicit center of gravity is NOT assumed - the coarse phase
# samples the full [low, high] range - but IS used as one seeded coarse
# candidate so V1 itself is always in the comparison.
DIMENSIONS = {
    "q_y": Dimension("weight_lat_error", 0.1, 10.0, True, 1.0),
    "q_psi": Dimension("weight_heading_error_squared_vel", 0.03, 3.0, True, 0.3),
    "q_v": Dimension("weight_velocity_error", 0.1, 10.0, True, 1.0),
    "q_a": Dimension("weight_acceleration_error", 0.01, 1.0, True, 0.1),
    "r_delta": Dimension("weight_steering_input", 0.1, 10.0, True, 1.0),
    "r_delta_v": Dimension("weight_steering_input_squared_vel", 0.025, 2.5, True, 0.25),
    "r_j": Dimension("weight_jerk", 0.01, 1.0, True, 0.1),
    "terminal_q_y": Dimension("terminal_weight_lat_error", 0.1, 10.0, True, 1.0),
    "terminal_q_psi": Dimension("terminal_weight_heading_error", 0.01, 1.0, True, 0.1),
    "terminal_q_v": Dimension("terminal_weight_velocity_error", 0.1, 10.0, True, 1.0),
}


def default_config() -> dict:
    """The current V1 design point, expressed in dimension-name space."""
    return {name: dim.default for name, dim in DIMENSIONS.items()}


def to_yaml_overrides(config: dict) -> dict:
    """Map dimension-name space (q_y, r_delta, ...) to yaml keys."""
    return {DIMENSIONS[name].yaml_key: value for name, value in config.items()}


def _sample_one(rng: random.Random, dim: Dimension) -> float:
    if dim.log_scale:
        low, high = math.log(dim.low), math.log(dim.high)
        return math.exp(rng.uniform(low, high))
    return rng.uniform(dim.low, dim.high)


def sample_coarse(rng: random.Random) -> dict:
    """One uniform (log-scaled where configured) sample across all dims."""
    return {name: _sample_one(rng, dim) for name, dim in DIMENSIONS.items()}


def sample_local(rng: random.Random, center: dict, spread: float = 0.3) -> dict:
    """Perturb `center` for local refinement.

    In log-space for log_scale dims (multiplicative perturbation e^U(-spread,
    spread)), clipped back into [low, high]; linear perturbation otherwise.
    """
    result = {}
    for name, dim in DIMENSIONS.items():
        value = center[name]
        if dim.log_scale:
            factor = math.exp(rng.uniform(-spread, spread))
            value = value * factor
        else:
            span = dim.high - dim.low
            value = value + rng.uniform(-spread, spread) * span
        result[name] = min(max(value, dim.low), dim.high)
    return result
