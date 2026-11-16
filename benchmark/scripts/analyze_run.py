#!/usr/bin/env python3
import csv
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def f(value):
    try:
        return float(value)
    except (ValueError, TypeError):
        return float("nan")


def load_csv(path):
    with open(path, newline="") as file:
        return list(csv.DictReader(file))


def arr(rows, key):
    return np.array([f(r.get(key, "nan")) for r in rows], dtype=float)


def finite(x):
    return x[np.isfinite(x)]


def stats_abs(x):
    x = finite(x)
    if len(x) == 0:
        return {"rmse": None, "mae": None, "max_abs": None, "p95_abs": None}
    ax = np.abs(x)
    return {
        "rmse": float(np.sqrt(np.mean(x ** 2))),
        "mae": float(np.mean(ax)),
        "max_abs": float(np.max(ax)),
        "p95_abs": float(np.percentile(ax, 95)),
    }


def stats_basic(x):
    x = finite(x)
    if len(x) == 0:
        return {"mean": None, "median": None, "std": None, "p95": None, "p99": None, "max": None}
    return {
        "mean": float(np.mean(x)),
        "median": float(np.median(x)),
        "std": float(np.std(x)),
        "p95": float(np.percentile(x, 95)),
        "p99": float(np.percentile(x, 99)),
        "max": float(np.max(x)),
    }


def choose_error(rows, evaluator_key, raw_key):
    evaluator = arr(rows, evaluator_key)
    if np.count_nonzero(np.isfinite(evaluator)) > max(5, len(rows) // 10):
        return evaluator, evaluator_key
    return arr(rows, raw_key), raw_key


def active_window(rows, move_threshold=0.05):
    if not rows:
        return 0, 0

    v = arr(rows, "velocity_actual")

    moving = np.where(
        np.isfinite(v) & (np.abs(v) > move_threshold)
    )[0]

    if len(moving) == 0:
        return 0, len(rows)

    start = int(moving[0])

    return start, len(rows)
    
    
    

def boundary_event_times(run_dir):
    path = run_dir / "events.csv"
    if not path.exists():
        return []

    out = []
    with open(path, newline="") as file:
        for row in csv.DictReader(file):
            if row.get("event") != "boundary_departure":
                continue

            value = (row.get("value") or "").strip()
            active = False
            try:
                active = abs(float(value)) > 1e-9
            except ValueError:
                low = value.lower()
                active = any(
                    token in low
                    for token in ["true", "error", "critical", "departure", "approaching"]
                )

            if active:
                out.append(f(row["time_s"]))

    return out


def add_event_lines(times):
    for t in times:
        plt.axvline(t, linestyle="--", alpha=0.6)


def save_plot(path, title, xlabel="Time [s]", ylabel=None):
    plt.title(title)
    plt.xlabel(xlabel)
    if ylabel:
        plt.ylabel(ylabel)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def main():
    if len(sys.argv) != 2:
        print("Usage: analyze_run.py <run_directory>")
        return 2

    run_dir = Path(sys.argv[1]).resolve()
    ts_path = run_dir / "timeseries.csv"
    if not ts_path.exists():
        raise FileNotFoundError(ts_path)

    rows_all = load_csv(ts_path)
    start, end = active_window(rows_all)
    rows = rows_all[start:end] if rows_all[start:end] else rows_all
    t = arr(rows, "time_s")

    boundary_times = boundary_event_times(run_dir)

    velocity_error, velocity_error_source = choose_error(
        rows, "velocity_error_eval", "velocity_error_raw"
    )
    lateral_error, lateral_error_source = choose_error(
        rows, "lateral_error_eval", "lateral_error_raw"
    )
    yaw_error, yaw_error_source = choose_error(
        rows, "yaw_error_eval", "yaw_error_raw"
    )

    summary = {
        "analysis_window": {
            "start_time_s": float(t[0]) if len(t) else None,
            "end_time_s": float(t[-1]) if len(t) else None,
            "duration_s": float(t[-1] - t[0]) if len(t) > 1 else 0.0,
            "samples": len(rows),
        },
        "error_sources": {
            "velocity": velocity_error_source,
            "lateral": lateral_error_source,
            "yaw": yaw_error_source,
        },
        "velocity_error": stats_abs(velocity_error),
        "lateral_error": stats_abs(lateral_error),
        "yaw_error": stats_abs(yaw_error),
        "acceleration": stats_basic(arr(rows, "acceleration_actual")),
        "jerk": stats_abs(arr(rows, "jerk_eval")),
        "lateral_acceleration_abs": stats_basic(arr(rows, "lateral_acceleration_abs")),
        "steering_rate": stats_abs(arr(rows, "steering_rate_eval")),
        "required_steer": stats_abs(arr(rows, "required_steer")),
        "reference_curvature": stats_abs(arr(rows, "ref_curvature")),
        "controller_to_actual_steering_error": stats_abs(
            arr(rows, "steering_controller") - arr(rows, "steering_actual")
        ),
        "boundary": {
            "departure_detected": len(boundary_times) > 0,
            "first_departure_time_s": boundary_times[0] if boundary_times else None,
        },
    }

    for key in [
        "left_boundary_distance",
        "right_boundary_distance",
        "left_uncrossable_boundary_distance",
        "right_uncrossable_boundary_distance",
    ]:
        x = finite(arr(rows, key))
        summary["boundary"][f"{key}_min"] = float(np.min(x)) if len(x) else None

    timings_path = run_dir / "timings.csv"
    if timings_path.exists():
        timing_rows = load_csv(timings_path)
        # lateral/longitudinal: stock's two-stage controller.
        # integrated_callback/integrated_solver: this thesis's integrated
        # MPC, which has no separate stages - do not conflate the two
        # controllers' timing semantics under the same labels.
        for controller in ["lateral", "longitudinal", "integrated_callback", "integrated_solver"]:
            values = np.array(
                [
                    f(r["processing_time_ms"])
                    for r in timing_rows
                    if r.get("controller") == controller
                ],
                dtype=float,
            )
            summary[f"{controller}_processing_time_ms"] = stats_basic(values)

    status_file = run_dir / "status.txt"
    summary["scenario_status"] = (
        status_file.read_text().strip()
        if status_file.exists()
        else "UNKNOWN"
    )

    with open(run_dir / "summary.json", "w") as file:
        json.dump(summary, file, indent=2)

    plots_dir = run_dir / "plots"
    plots_dir.mkdir(exist_ok=True)

    plt.figure(figsize=(10, 5))
    plt.plot(t, arr(rows, "ref_velocity"), label="Reference")
    plt.plot(t, arr(rows, "velocity_actual"), label="Actual")
    add_event_lines(boundary_times)
    plt.legend()
    save_plot(
        plots_dir / "01_velocity_tracking.png",
        "Velocity tracking",
        ylabel="Velocity [m/s]",
    )

    plt.figure(figsize=(10, 5))
    plt.plot(t, velocity_error)
    add_event_lines(boundary_times)
    save_plot(
        plots_dir / "02_velocity_error.png",
        "Longitudinal velocity error",
        ylabel="Error [m/s]",
    )

    plt.figure(figsize=(10, 5))
    plt.plot(t, lateral_error)
    add_event_lines(boundary_times)
    save_plot(
        plots_dir / "03_lateral_error.png",
        "Lateral tracking error",
        ylabel="Error [m]",
    )

    plt.figure(figsize=(10, 5))
    plt.plot(t, yaw_error)
    add_event_lines(boundary_times)
    save_plot(
        plots_dir / "04_yaw_error.png",
        "Yaw tracking error",
        ylabel="Error [rad]",
    )

    plt.figure(figsize=(10, 5))
    plt.plot(t, arr(rows, "required_steer"), label="Required from curvature")
    plt.plot(t, arr(rows, "steering_controller"), label="Controller")
    plt.plot(t, arr(rows, "steering_final"), label="Final gated")
    plt.plot(t, arr(rows, "steering_actual"), label="Actual")
    add_event_lines(boundary_times)
    plt.legend()
    save_plot(
        plots_dir / "05_steering.png",
        "Steering comparison",
        ylabel="Steering tire angle [rad]",
    )

    plt.figure(figsize=(10, 5))
    plt.plot(t, arr(rows, "acceleration_actual"), label="Acceleration")
    plt.plot(t, arr(rows, "jerk_eval"), label="Jerk")
    add_event_lines(boundary_times)
    plt.legend()
    save_plot(
        plots_dir / "06_acceleration_jerk.png",
        "Acceleration and jerk",
    )

    plt.figure(figsize=(7, 7))
    plt.plot(arr(rows, "ref_x"), arr(rows, "ref_y"), label="Reference")
    plt.plot(arr(rows, "x"), arr(rows, "y"), label="Actual")
    plt.axis("equal")
    plt.legend()
    save_plot(
        plots_dir / "07_xy_tracking.png",
        "XY trajectory tracking",
        xlabel="X [m]",
        ylabel="Y [m]",
    )

    plt.figure(figsize=(10, 5))
    plt.plot(t, arr(rows, "left_boundary_distance"), label="Left boundary")
    plt.plot(t, arr(rows, "right_boundary_distance"), label="Right boundary")
    plt.plot(t, arr(rows, "left_uncrossable_boundary_distance"), label="Left uncrossable")
    plt.plot(t, arr(rows, "right_uncrossable_boundary_distance"), label="Right uncrossable")
    add_event_lines(boundary_times)
    plt.legend()
    save_plot(
        plots_dir / "08_boundary_distance.png",
        "Boundary distances",
        ylabel="Distance [m]",
    )

    if timings_path.exists():
        timing_rows = load_csv(timings_path)
        plt.figure(figsize=(10, 5))
        # lateral/longitudinal: stock's two-stage controller.
        # integrated_callback/integrated_solver: this thesis's integrated
        # MPC, which has no separate stages - do not conflate the two
        # controllers' timing semantics under the same labels.
        for controller in ["lateral", "longitudinal", "integrated_callback", "integrated_solver"]:
            tt = np.array(
                [f(r["time_s"]) for r in timing_rows if r.get("controller") == controller]
            )
            yy = np.array(
                [
                    f(r["processing_time_ms"])
                    for r in timing_rows
                    if r.get("controller") == controller
                ]
            )
            if len(tt):
                plt.plot(tt, yy, label=controller.capitalize())
        add_event_lines(boundary_times)
        plt.legend()
        save_plot(
            plots_dir / "09_processing_times.png",
            "Controller processing times",
            ylabel="Time [ms]",
        )

    md = [
        "# Benchmark summary",
        "",
        f"- Status: **{summary['scenario_status']}**",
        f"- Active duration: {summary['analysis_window']['duration_s']:.3f} s",
        f"- Samples: {summary['analysis_window']['samples']}",
        f"- Boundary departure: {summary['boundary']['departure_detected']}",
        "",
        "## Main metrics",
        "",
        f"- Velocity RMSE: {summary['velocity_error']['rmse']}",
        f"- Lateral RMSE: {summary['lateral_error']['rmse']}",
        f"- Yaw RMSE: {summary['yaw_error']['rmse']}",
    ]
    (run_dir / "summary.md").write_text("\n".join(md) + "\n")

    print(f"Analysis complete: {run_dir}")
    print(f"Summary: {run_dir / 'summary.json'}")
    print(f"Plots:   {plots_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

