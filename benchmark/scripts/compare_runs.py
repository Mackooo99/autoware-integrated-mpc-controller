#!/usr/bin/env python3
import csv
import json
import sys
from pathlib import Path
from collections import defaultdict

import numpy as np


def nested(data, *keys):
    cur = data
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def main():
    if len(sys.argv) < 3:
        print("Usage: compare_runs.py <results_root_1> <results_root_2> [output_dir]")
        return 2

    roots = [Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()]
    output_dir = Path(sys.argv[3]).resolve() if len(sys.argv) >= 4 else Path.cwd()
    output_dir.mkdir(parents=True, exist_ok=True)

    records = []
    for root in roots:
        controller = root.name
        for summary_path in sorted(root.rglob("summary.json")):
            with open(summary_path) as f:
                s = json.load(f)

            rel = summary_path.relative_to(root)
            parts = rel.parts
            scenario = parts[0] if len(parts) >= 2 else "UNKNOWN"
            run = parts[1] if len(parts) >= 3 else summary_path.parent.name

            records.append({
                "controller": controller,
                "scenario": scenario,
                "run": run,
                "status": s.get("scenario_status", "UNKNOWN"),
                "velocity_rmse": nested(s, "velocity_error", "rmse"),
                "lateral_rmse": nested(s, "lateral_error", "rmse"),
                "yaw_rmse": nested(s, "yaw_error", "rmse"),
                "velocity_max_abs": nested(s, "velocity_error", "max_abs"),
                "lateral_max_abs": nested(s, "lateral_error", "max_abs"),
                "yaw_max_abs": nested(s, "yaw_error", "max_abs"),
                "lateral_time_mean_ms": nested(s, "lateral_processing_time_ms", "mean"),
                "lateral_time_p95_ms": nested(s, "lateral_processing_time_ms", "p95"),
                "longitudinal_time_mean_ms": nested(s, "longitudinal_processing_time_ms", "mean"),
                "longitudinal_time_p95_ms": nested(s, "longitudinal_processing_time_ms", "p95"),
                "boundary_departure": nested(s, "boundary", "departure_detected"),
            })

    if not records:
        print("No summary.json files found.")
        return 1

    fields = list(records[0].keys())
    with open(output_dir / "comparison_runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(records)

    groups = defaultdict(list)
    for r in records:
        groups[(r["controller"], r["scenario"])].append(r)

    agg_fields = [
        "controller", "scenario", "runs",
        "velocity_rmse_mean", "velocity_rmse_std",
        "lateral_rmse_mean", "lateral_rmse_std",
        "yaw_rmse_mean", "yaw_rmse_std",
        "lateral_time_p95_mean_ms",
        "longitudinal_time_p95_mean_ms",
        "pass_count", "fail_count",
    ]

    agg_rows = []
    for (controller, scenario), items in sorted(groups.items()):
        def vals(key):
            out = []
            for item in items:
                v = item.get(key)
                if isinstance(v, (int, float)):
                    out.append(float(v))
            return np.array(out, dtype=float)

        def mean_std(key):
            x = vals(key)
            if len(x) == 0:
                return None, None
            return float(np.mean(x)), float(np.std(x))

        vr_m, vr_s = mean_std("velocity_rmse")
        lr_m, lr_s = mean_std("lateral_rmse")
        yr_m, yr_s = mean_std("yaw_rmse")
        lt = vals("lateral_time_p95_ms")
        lon = vals("longitudinal_time_p95_ms")

        agg_rows.append({
            "controller": controller,
            "scenario": scenario,
            "runs": len(items),
            "velocity_rmse_mean": vr_m,
            "velocity_rmse_std": vr_s,
            "lateral_rmse_mean": lr_m,
            "lateral_rmse_std": lr_s,
            "yaw_rmse_mean": yr_m,
            "yaw_rmse_std": yr_s,
            "lateral_time_p95_mean_ms": float(np.mean(lt)) if len(lt) else None,
            "longitudinal_time_p95_mean_ms": float(np.mean(lon)) if len(lon) else None,
            "pass_count": sum(1 for i in items if i["status"] == "PASS"),
            "fail_count": sum(1 for i in items if i["status"] == "FAIL"),
        })

    with open(output_dir / "comparison_aggregate.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=agg_fields)
        w.writeheader()
        w.writerows(agg_rows)

    print(f"Wrote {output_dir / 'comparison_runs.csv'}")
    print(f"Wrote {output_dir / 'comparison_aggregate.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

