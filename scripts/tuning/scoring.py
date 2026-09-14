"""Composite scoring of a completed benchmark run (Task 12/13).

Reads the same summary.json shape analyze_run.py already produces
(control_benchmark/scripts/analyze_run.py) - lower score is better.
Failed/missing runs get a large fixed penalty so they always rank last,
rather than being silently dropped or crashing the ranking.
"""

import math

# RMSE-equal-to-this-scale contributes 1.0 (pre-weight) to the score.
# Fixed engineering judgement calls (vehicle/scenario scale), not fit to
# any specific run - override at the call site if a different scale is
# wanted for a different vehicle/scenario set.
NORMALIZATION = {
    "lateral_error": 0.10,  # m
    "yaw_error": 0.05,  # rad
    "velocity_error": 0.30,  # m/s
    "jerk": 1.0,  # m/s^3 (rmse, smoothness proxy)
    "steering_rate": 0.05,  # rad/s (rmse, smoothness proxy)
}

METRIC_WEIGHTS = {
    "lateral_error": 3.0,
    "yaw_error": 1.5,
    "velocity_error": 1.0,
    "jerk": 0.5,
    "steering_rate": 0.5,
}

FAILURE_PENALTY = 1000.0
MISSING_METRIC_PENALTY = 50.0
BOUNDARY_VIOLATION_PENALTY = 1000.0

# summary["boundary"] keys (see analyze_run.py) that indicate a lane/road
# boundary was crossed when their *_min value is negative.
BOUNDARY_MIN_KEYS = (
    "left_uncrossable_boundary_distance_min",
    "right_uncrossable_boundary_distance_min",
)


def score_summary(summary: dict, status: str) -> tuple:
    """Returns (score, breakdown). status should be one of PASS/FAIL/UNKNOWN
    (run_experiment.sh's status.txt convention). Only PASS is scored
    normally; anything else is a hard failure."""
    breakdown = {}

    if status != "PASS":
        breakdown["failure_penalty"] = FAILURE_PENALTY
        breakdown["status"] = status
        breakdown["total"] = FAILURE_PENALTY
        return FAILURE_PENALTY, breakdown

    total = 0.0

    for metric, scale in NORMALIZATION.items():
        entry = summary.get(metric)
        rmse = entry.get("rmse") if isinstance(entry, dict) else None

        if rmse is None or not math.isfinite(rmse):
            breakdown[metric] = None
            breakdown[f"{metric}_rmse"] = None
            total += MISSING_METRIC_PENALTY
            continue

        weighted = (rmse / scale) * METRIC_WEIGHTS[metric]
        breakdown[metric] = weighted
        # Raw value alongside the normalized/weighted contribution above,
        # for sensitivity_analysis.py (Task 13) - a weighted score isn't
        # a physically meaningful axis to plot against a weight value.
        breakdown[f"{metric}_rmse"] = rmse
        total += weighted

    boundary = summary.get("boundary") or {}
    for key in BOUNDARY_MIN_KEYS:
        value = boundary.get(key)
        if value is not None and value < 0.0:
            breakdown[f"{key}_violation"] = BOUNDARY_VIOLATION_PENALTY
            total += BOUNDARY_VIOLATION_PENALTY

    breakdown["total"] = total
    return total, breakdown
