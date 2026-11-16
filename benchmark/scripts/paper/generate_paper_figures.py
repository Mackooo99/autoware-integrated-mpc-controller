#!/usr/bin/env python3

from pathlib import Path
import json

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


BASE = Path.home() / "control_benchmark" / "paper_dataset_20260915"
FINAL = BASE / "final_runs"
ABLATION = BASE / "ablation" / "S02"
FIG_DIR = BASE / "figures"

FIG_DIR.mkdir(parents=True, exist_ok=True)

# Dimenzije pogodne za jednu kolonu rada.
FIGSIZE = (3.35, 2.35)
DPI = 300


def load_window(run_dir, columns):
    """Učitava samo analysis_window korišćen za summary.json."""
    run_dir = Path(run_dir)

    with open(run_dir / "summary.json") as f:
        summary = json.load(f)

    start = summary["analysis_window"]["start_time_s"]
    end = summary["analysis_window"]["end_time_s"]

    df = pd.read_csv(run_dir / "timeseries.csv")

    df = df[
        (df["time_s"] >= start) &
        (df["time_s"] <= end)
    ].copy()

    needed = ["time_s"] + list(columns)
    df = df[needed]

    for c in needed:
        df[c] = pd.to_numeric(df[c], errors="coerce")

    df = df.dropna(subset=needed)

    df["t_rel"] = df["time_s"] - start

    return df


def ensemble(run_dirs, column, dt=0.05):
    """Interpolira više run-ova na zajedničku vremensku osu."""
    series = []

    for run_dir in run_dirs:
        df = load_window(run_dir, [column])

        t = df["t_rel"].to_numpy(float)
        y = df[column].to_numpy(float)

        if len(t) < 2:
            continue

        series.append((t, y))

    if not series:
        raise RuntimeError(f"Nema validnih podataka za {column}")

    common_start = max(t[0] for t, _ in series)
    common_end = min(t[-1] for t, _ in series)

    grid = np.arange(
        common_start,
        common_end + 0.5 * dt,
        dt
    )

    values = []

    for t, y in series:
        values.append(np.interp(grid, t, y))

    values = np.vstack(values)

    mean = np.mean(values, axis=0)

    if len(values) > 1:
        std = np.std(values, axis=0, ddof=1)
    else:
        std = np.zeros_like(mean)

    return grid, mean, std


def ensemble_derived(run_dirs, expression, dt=0.05):
    """
    expression(df) mora vratiti numpy array iste dužine kao df.
    """
    series = []

    for run_dir in run_dirs:
        df = load_window(
            run_dir,
            ["steering_controller", "steering_actual"]
        )

        t = df["t_rel"].to_numpy(float)
        y = expression(df)

        mask = np.isfinite(t) & np.isfinite(y)
        t = t[mask]
        y = y[mask]

        if len(t) < 2:
            continue

        series.append((t, y))

    common_start = max(t[0] for t, _ in series)
    common_end = min(t[-1] for t, _ in series)

    grid = np.arange(
        common_start,
        common_end + 0.5 * dt,
        dt
    )

    values = np.vstack([
        np.interp(grid, t, y)
        for t, y in series
    ])

    mean = np.mean(values, axis=0)

    if len(values) > 1:
        std = np.std(values, axis=0, ddof=1)
    else:
        std = np.zeros_like(mean)

    return grid, mean, std


def finish_figure(fig, ax, ylabel, filename):
    ax.set_xlabel("Vreme od početka analize [s]")
    ax.set_ylabel(ylabel)

    ax.legend(
        fontsize=7,
        frameon=False
    )

    ax.tick_params(labelsize=7)

    fig.tight_layout()

    png = FIG_DIR / f"{filename}.png"
    pdf = FIG_DIR / f"{filename}.pdf"

    fig.savefig(
        png,
        dpi=DPI,
        bbox_inches="tight"
    )

    fig.savefig(
        pdf,
        bbox_inches="tight"
    )

    plt.close(fig)

    print(f"Saved: {png}")
    print(f"Saved: {pdf}")


def plot_groups(groups, column, ylabel, filename):
    fig, ax = plt.subplots(figsize=FIGSIZE)

    for label, run_dirs in groups.items():
        t, mean, std = ensemble(run_dirs, column)

        line = ax.plot(
            t,
            mean,
            linewidth=1.25,
            label=label
        )[0]

        ax.fill_between(
            t,
            mean - std,
            mean + std,
            alpha=0.15
        )

    finish_figure(
        fig,
        ax,
        ylabel,
        filename
    )


# ----------------------------------------------------------------------
# RUN GROUPS
# ----------------------------------------------------------------------

stock_s3 = [
    FINAL / "baseline_stock" / "S03" / "run_501",
    FINAL / "baseline_stock" / "S03" / "run_502",
    FINAL / "baseline_stock" / "S03" / "run_503",
]

integrated_s3 = [
    FINAL / "final_integrated_mpc" / "S03" / "run_601",
    FINAL / "final_integrated_mpc" / "S03" / "run_602",
    FINAL / "final_integrated_mpc" / "S03" / "run_603",
]

delay_off = [
    ABLATION / "run_201",
    ABLATION / "run_202",
]

delay_on = [
    ABLATION / "run_203",
    ABLATION / "run_204",
]

final_s2 = [
    FINAL / "final_integrated_mpc" / "S02" / "run_601",
    FINAL / "final_integrated_mpc" / "S02" / "run_602",
    FINAL / "final_integrated_mpc" / "S02" / "run_603",
]


# ----------------------------------------------------------------------
# FIGURE 1
# Delay ablation – lateral error
# ----------------------------------------------------------------------

plot_groups(
    {
        "Delay OFF": delay_off,
        "Delay ON": delay_on,
        "Final tuned": final_s2,
    },
    "lateral_error_eval",
    "Lateralna greška [m]",
    "01_s2_delay_ablation_lateral_error",
)


# ----------------------------------------------------------------------
# FIGURE 2
# Delay ablation – controller vs actual steering
# ----------------------------------------------------------------------

fig, ax = plt.subplots(figsize=FIGSIZE)

for label, dirs in {
    "Delay OFF": delay_off,
    "Delay ON": delay_on,
    "Final tuned": final_s2,
}.items():

    t, mean, std = ensemble_derived(
        dirs,
        lambda df:
            df["steering_controller"].to_numpy(float)
            - df["steering_actual"].to_numpy(float)
    )

    ax.plot(
        t,
        mean,
        linewidth=1.25,
        label=label
    )

    ax.fill_between(
        t,
        mean - std,
        mean + std,
        alpha=0.15
    )

finish_figure(
    fig,
    ax,
    "Greška ugla upravljanja [rad]",
    "02_s2_delay_steering_tracking_error",
)


# ----------------------------------------------------------------------
# FIGURE 3
# S3 lateral error
# ----------------------------------------------------------------------

plot_groups(
    {
        "Stock MPC + PI/PID": stock_s3,
        "Integrated MPC": integrated_s3,
    },
    "lateral_error_eval",
    "Lateralna greška [m]",
    "03_s3_lateral_error_stock_vs_integrated",
)


# ----------------------------------------------------------------------
# FIGURE 4
# S3 velocity error
# ----------------------------------------------------------------------

plot_groups(
    {
        "Stock MPC + PI/PID": stock_s3,
        "Integrated MPC": integrated_s3,
    },
    "velocity_error_eval",
    "Greška brzine [m/s]",
    "04_s3_velocity_error_stock_vs_integrated",
)


# ----------------------------------------------------------------------
# FIGURE 5
# S3 jerk
# ----------------------------------------------------------------------

plot_groups(
    {
        "Stock MPC + PI/PID": stock_s3,
        "Integrated MPC": integrated_s3,
    },
    "jerk_eval",
    "Jerk [m/s³]",
    "05_s3_jerk_stock_vs_integrated",
)


# ----------------------------------------------------------------------
# FIGURE 6
# S3 steering rate
# ----------------------------------------------------------------------

plot_groups(
    {
        "Stock MPC + PI/PID": stock_s3,
        "Integrated MPC": integrated_s3,
    },
    "steering_rate_eval",
    "Brzina promene ugla [rad/s]",
    "06_s3_steering_rate_stock_vs_integrated",
)


# ----------------------------------------------------------------------
# FIGURE 7
# Representative XY trajectory – S3
# ----------------------------------------------------------------------

stock_rep = stock_s3[1]
integrated_rep = integrated_s3[1]

stock_df = load_window(
    stock_rep,
    ["x", "y", "ref_x", "ref_y"]
)

int_df = load_window(
    integrated_rep,
    ["x", "y"]
)

# Lokalni koordinatni sistem radi čitljivosti.
x0 = stock_df["ref_x"].iloc[0]
y0 = stock_df["ref_y"].iloc[0]

fig, ax = plt.subplots(figsize=(3.35, 3.0))

ax.plot(
    stock_df["ref_x"] - x0,
    stock_df["ref_y"] - y0,
    linewidth=1.1,
    label="Referentna putanja"
)

ax.plot(
    stock_df["x"] - x0,
    stock_df["y"] - y0,
    linewidth=1.1,
    label="Stock MPC + PI/PID"
)

ax.plot(
    int_df["x"] - x0,
    int_df["y"] - y0,
    linewidth=1.1,
    label="Integrated MPC"
)

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.tick_params(labelsize=7)
ax.set_aspect("equal", adjustable="box")

ax.legend(
    fontsize=7,
    frameon=False
)

fig.tight_layout()

fig.savefig(
    FIG_DIR / "07_s3_xy_trajectory.png",
    dpi=DPI,
    bbox_inches="tight"
)

fig.savefig(
    FIG_DIR / "07_s3_xy_trajectory.pdf",
    bbox_inches="tight"
)

plt.close(fig)

print("Saved: S3 XY trajectory")


print("\n============================================================")
print("Sve figure su generisane.")
print(f"Folder: {FIG_DIR}")
print("============================================================")
