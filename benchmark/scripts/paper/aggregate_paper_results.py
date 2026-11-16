#!/usr/bin/env python3

import json
from pathlib import Path

import pandas as pd

BASE = Path.home() / "control_benchmark" / "paper_dataset_20260915"
ROOT = BASE / "final_runs"
OUT_DIR = BASE / "tables"
OUT_DIR.mkdir(parents=True, exist_ok=True)

records = []

for summary_path in sorted(ROOT.glob("*/*/run_*/summary.json")):
    controller = summary_path.parts[-4]
    scenario = summary_path.parts[-3]
    run = summary_path.parts[-2]

    with open(summary_path) as f:
        s = json.load(f)

    rec = {
        "controller": controller,
        "scenario": scenario,
        "run": run,

        "velocity_rmse": s["velocity_error"]["rmse"],
        "lateral_rmse": s["lateral_error"]["rmse"],
        "yaw_rmse": s["yaw_error"]["rmse"],

        "velocity_max_abs": s["velocity_error"]["max_abs"],
        "lateral_max_abs": s["lateral_error"]["max_abs"],
        "yaw_max_abs": s["yaw_error"]["max_abs"],

        "jerk_rmse": s["jerk"]["rmse"],
        "steering_rate_rmse": s["steering_rate"]["rmse"],
        "steering_tracking_rmse":
            s["controller_to_actual_steering_error"]["rmse"],

        "lateral_accel_max":
            s["lateral_acceleration_abs"]["max"],

        "departure":
            bool(s["boundary"]["departure_detected"]),

        "status":
            s["scenario_status"],
    }

    # Stock timing
    rec["stock_lateral_time_mean_ms"] = (
        s["lateral_processing_time_ms"]["mean"]
    )
    rec["stock_lateral_time_p95_ms"] = (
        s["lateral_processing_time_ms"]["p95"]
    )
    rec["stock_longitudinal_time_mean_ms"] = (
        s["longitudinal_processing_time_ms"]["mean"]
    )
    rec["stock_longitudinal_time_p95_ms"] = (
        s["longitudinal_processing_time_ms"]["p95"]
    )

    # Integrated timing
    rec["integrated_callback_mean_ms"] = (
        s["integrated_callback_processing_time_ms"]["mean"]
    )
    rec["integrated_callback_p95_ms"] = (
        s["integrated_callback_processing_time_ms"]["p95"]
    )
    rec["integrated_solver_mean_ms"] = (
        s["integrated_solver_processing_time_ms"]["mean"]
    )
    rec["integrated_solver_p95_ms"] = (
        s["integrated_solver_processing_time_ms"]["p95"]
    )

    records.append(rec)

runs = pd.DataFrame(records)

# Čuvamo i individualne rezultate svih 18 run-ova.
runs.to_csv(OUT_DIR / "final_18_individual_runs.csv", index=False)

metrics = [
    "velocity_rmse",
    "lateral_rmse",
    "yaw_rmse",
    "jerk_rmse",
    "steering_rate_rmse",
    "steering_tracking_rmse",
    "lateral_accel_max",
    "stock_lateral_time_mean_ms",
    "stock_lateral_time_p95_ms",
    "stock_longitudinal_time_mean_ms",
    "stock_longitudinal_time_p95_ms",
    "integrated_callback_mean_ms",
    "integrated_callback_p95_ms",
    "integrated_solver_mean_ms",
    "integrated_solver_p95_ms",
]

agg = runs.groupby(["scenario", "controller"])[metrics].agg(
    ["mean", "std"]
)

# Flatten column names.
agg.columns = [
    f"{metric}_{stat}"
    for metric, stat in agg.columns
]
agg = agg.reset_index()

agg.to_csv(
    OUT_DIR / "final_mean_std_full.csv",
    index=False
)

# Glavna tabela rada
main_metrics = [
    "velocity_rmse",
    "lateral_rmse",
    "yaw_rmse",
    "jerk_rmse",
    "steering_rate_rmse",
]

main_rows = []

for (scenario, controller), g in runs.groupby(
    ["scenario", "controller"]
):
    row = {
        "scenario": scenario,
        "controller": controller,
        "n": len(g),
        "pass_count": int((g["status"] == "PASS").sum()),
        "departure_count": int(g["departure"].sum()),
    }

    for m in main_metrics:
        row[f"{m}_mean"] = g[m].mean()
        row[f"{m}_std"] = g[m].std(ddof=1)

    main_rows.append(row)

main = pd.DataFrame(main_rows).sort_values(
    ["scenario", "controller"]
)

main.to_csv(
    OUT_DIR / "main_performance_table.csv",
    index=False
)

# Percentage comparison: positive = Integrated is better
comparison_rows = []

for scenario in sorted(runs["scenario"].unique()):
    stock = runs[
        (runs["scenario"] == scenario) &
        (runs["controller"] == "baseline_stock")
    ]

    integ = runs[
        (runs["scenario"] == scenario) &
        (runs["controller"] == "final_integrated_mpc")
    ]

    row = {"scenario": scenario}

    for m in main_metrics:
        stock_mean = stock[m].mean()
        integ_mean = integ[m].mean()

        improvement = (
            (stock_mean - integ_mean) /
            stock_mean * 100.0
        )

        row[f"{m}_stock"] = stock_mean
        row[f"{m}_integrated"] = integ_mean
        row[f"{m}_improvement_percent"] = improvement

    comparison_rows.append(row)

comparison = pd.DataFrame(comparison_rows)

comparison.to_csv(
    OUT_DIR / "stock_vs_integrated_percent.csv",
    index=False
)

print("\n=== MAIN PERFORMANCE TABLE ===\n")

for _, r in main.iterrows():
    print(
        f'{r["scenario"]:>3}  '
        f'{r["controller"]:<22}  '
        f'v={r["velocity_rmse_mean"]:.5f} ± {r["velocity_rmse_std"]:.5f}  '
        f'lat={r["lateral_rmse_mean"]:.5f} ± {r["lateral_rmse_std"]:.5f}  '
        f'yaw={r["yaw_rmse_mean"]:.5f} ± {r["yaw_rmse_std"]:.5f}  '
        f'jerk={r["jerk_rmse_mean"]:.5f} ± {r["jerk_rmse_std"]:.5f}  '
        f'steer_rate={r["steering_rate_rmse_mean"]:.5f} ± '
        f'{r["steering_rate_rmse_std"]:.5f}'
    )

print("\n=== INTEGRATED IMPROVEMENT vs STOCK [%] ===")
print("(pozitivno = Integrated bolje; negativno = Stock bolje)\n")

cols = [
    "scenario",
    "velocity_rmse_improvement_percent",
    "lateral_rmse_improvement_percent",
    "yaw_rmse_improvement_percent",
    "jerk_rmse_improvement_percent",
    "steering_rate_rmse_improvement_percent",
]

print(comparison[cols].to_string(index=False))

print("\nSaved:")
for f in sorted(OUT_DIR.glob("*.csv")):
    print(f"  {f}")
