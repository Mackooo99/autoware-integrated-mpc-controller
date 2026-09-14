#!/usr/bin/env python3
"""Parameter sensitivity analysis for the integrated MPC (Task 13).

Operates on a real run_tuning.py manifest.json - a set of actually
evaluated (config, scenario) trials, each with its full metric breakdown
(scoring.py) and, where available, real callback/solver timing stats. It
does not fabricate data: a dimension/metric pair with no PASS trials is
skipped and reported as such, not plotted.

Produces, per cost-weight dimension (param_space.DIMENSIONS):
  - tracking error (lateral RMSE) vs weight
  - velocity error (RMSE) vs weight
  - smoothness (jerk RMSE, steering-rate RMSE) vs weight
  - controller callback time (mean) vs weight, if timing data is present
plus a raw CSV of every (config_id, scenario, weight values, metric
values) row so results remain available outside the plots.

Usage:
  python3 sensitivity_analysis.py --manifest runs/sweep_2026_09_15/manifest.json \\
      --output-dir runs/sweep_2026_09_15/sensitivity
"""

import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import param_space  # noqa: E402

METRICS = {
    "lateral_error_rmse": "Lateral tracking error RMSE [m]",
    "yaw_error_rmse": "Yaw tracking error RMSE [rad]",
    "velocity_error_rmse": "Velocity tracking error RMSE [m/s]",
    "jerk_rmse": "Jerk RMSE (smoothness) [m/s^3]",
    "steering_rate_rmse": "Steering-rate RMSE (smoothness) [rad/s]",
}


def load_pass_rows(manifest_path: Path):
    trials = json.loads(manifest_path.read_text())
    rows = []

    for t in trials:
        if t.get("status") != "PASS":
            continue

        breakdown = t.get("breakdown") or {}
        row = {"config_id": t["config_id"], "scenario": t["scenario"]}
        row.update(t["params"])

        for metric_key in METRICS:
            row[metric_key] = breakdown.get(metric_key)

        callback_stats = t.get("callback_time_stats") or {}
        row["callback_time_mean_ms"] = callback_stats.get("mean")

        rows.append(row)

    return rows


def write_raw_csv(rows, output_dir: Path):
    if not rows:
        return
    path = output_dir / "sensitivity_raw.csv"
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {path}")


def make_plots(rows, output_dir: Path):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available - skipping plots, raw CSV still written.")
        return

    for dim_name, dim in param_space.DIMENSIONS.items():
        xs_all = [r[dim_name] for r in rows if r.get(dim_name) is not None]
        if not xs_all:
            print(f"[skip] no data for dimension {dim_name}")
            continue

        for metric_key, metric_label in METRICS.items():
            xs, ys = [], []
            for r in rows:
                x, y = r.get(dim_name), r.get(metric_key)
                if x is not None and y is not None:
                    xs.append(x)
                    ys.append(y)

            if not xs:
                continue

            plt.figure(figsize=(6, 4))
            plt.scatter(xs, ys, s=14)
            if dim.log_scale:
                plt.xscale("log")
            plt.xlabel(f"{dim_name} ({dim.yaml_key})")
            plt.ylabel(metric_label)
            plt.title(f"{metric_label} vs {dim_name}")
            plt.tight_layout()
            out_path = output_dir / f"{dim_name}__{metric_key}.png"
            plt.savefig(out_path)
            plt.close()

        xs, ys = [], []
        for r in rows:
            x, y = r.get(dim_name), r.get("callback_time_mean_ms")
            if x is not None and y is not None:
                xs.append(x)
                ys.append(y)

        if xs:
            plt.figure(figsize=(6, 4))
            plt.scatter(xs, ys, s=14)
            if dim.log_scale:
                plt.xscale("log")
            plt.xlabel(f"{dim_name} ({dim.yaml_key})")
            plt.ylabel("Mean callback time [ms]")
            plt.title(f"Callback time vs {dim_name}")
            plt.tight_layout()
            plt.savefig(output_dir / f"{dim_name}__callback_time.png")
            plt.close()
        else:
            print(f"[skip] no callback-time data for dimension {dim_name} "
                  "(needs a real run, not --dry-run)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    if not args.manifest.exists():
        parser.error(f"manifest not found: {args.manifest}")

    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = load_pass_rows(args.manifest)
    print(f"{len(rows)} PASS trial rows loaded from {args.manifest}")

    if not rows:
        print("No PASS trials to analyze - nothing written.")
        return

    write_raw_csv(rows, args.output_dir)
    make_plots(rows, args.output_dir)


if __name__ == "__main__":
    main()
