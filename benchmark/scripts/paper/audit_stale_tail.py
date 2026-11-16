#!/usr/bin/env python3

import json
from pathlib import Path

import numpy as np
import pandas as pd

BASE = Path.home() / "control_benchmark" / "paper_dataset_20260915"
ROOT = BASE / "final_runs"
OUT = BASE / "tables" / "stale_tail_audit.csv"

STALE_THRESHOLD_S = 0.5
STAMP_TOL = 1e-9

rows = []

for csv_path in sorted(ROOT.glob("*/*/run_*/timeseries.csv")):
    summary_path = csv_path.parent / "summary.json"

    controller = csv_path.parts[-4]
    scenario = csv_path.parts[-3]
    run = csv_path.parts[-2]

    df = pd.read_csv(csv_path)

    with open(summary_path) as f:
        summary = json.load(f)

    start = summary["analysis_window"]["start_time_s"]
    end = summary["analysis_window"]["end_time_s"]

    # Ispitujemo samo isti prozor koji je korišćen za summary.
    w = df[
        (df["time_s"] >= start) &
        (df["time_s"] <= end)
    ].copy()

    w = w[
        np.isfinite(w["time_s"]) &
        np.isfinite(w["odom_stamp_s"])
    ].copy()

    if len(w) < 2:
        rows.append({
            "controller": controller,
            "scenario": scenario,
            "run": run,
            "samples": len(w),
            "window_duration_s": np.nan,
            "terminal_same_stamp_rows": np.nan,
            "terminal_stale_duration_s": np.nan,
            "odom_nonmonotonic_count": np.nan,
            "flag_stale_tail": True,
        })
        continue

    t = w["time_s"].to_numpy(dtype=float)
    odom = w["odom_stamp_s"].to_numpy(dtype=float)

    # Koliko redova na kraju ima isti odom timestamp?
    final_stamp = odom[-1]
    i = len(odom) - 1

    while i > 0 and abs(odom[i - 1] - final_stamp) <= STAMP_TOL:
        i -= 1

    terminal_same_stamp_rows = len(odom) - i
    terminal_stale_duration = float(t[-1] - t[i])

    # Odom timestamp ne bi smeo da ide unazad.
    diffs = np.diff(odom)
    nonmonotonic = int(np.sum(diffs < -STAMP_TOL))

    flag = terminal_stale_duration > STALE_THRESHOLD_S

    rows.append({
        "controller": controller,
        "scenario": scenario,
        "run": run,
        "samples": len(w),
        "window_duration_s": float(t[-1] - t[0]),
        "terminal_same_stamp_rows": terminal_same_stamp_rows,
        "terminal_stale_duration_s": terminal_stale_duration,
        "odom_nonmonotonic_count": nonmonotonic,
        "flag_stale_tail": flag,
    })

result = pd.DataFrame(rows)
OUT.parent.mkdir(parents=True, exist_ok=True)
result.to_csv(OUT, index=False)

print("\n=== STALE TAIL AUDIT ===\n")
print(result.to_string(index=False))

flagged = result[result["flag_stale_tail"] == True]

print("\n============================================================")
if len(flagged) == 0:
    print("PASS: nijedan finalni run nema terminalni stale tail > "
          f"{STALE_THRESHOLD_S:.1f} s.")
else:
    print(f"WARNING: pronađeno je {len(flagged)} run-ova sa stale tail-om:")
    print(flagged[
        ["controller", "scenario", "run", "terminal_stale_duration_s"]
    ].to_string(index=False))
print("============================================================")

print(f"\nCSV saved to:\n{OUT}")
