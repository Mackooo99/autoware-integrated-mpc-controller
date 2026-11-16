#!/usr/bin/env python3

from pathlib import Path
import json

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ================================================================
# PUTANJE
# ================================================================

BASE = Path.home() / "control_benchmark" / "paper_dataset_20260915"
FINAL = BASE / "final_runs"
ABLATION = BASE / "ablation" / "S02"
RESULTS = Path.home() / "control_benchmark" / "results"
FIG_DIR = BASE / "figures_final"

FIG_DIR.mkdir(parents=True, exist_ok=True)

DPI = 300
FIGSIZE = (3.5, 2.55)

# Podrazumevani Matplotlib font podržava ćirilicu, ali ga eksplicitno
# postavljamo radi konzistentnog izgleda svih figura.
plt.rcParams["font.family"] = "DejaVu Sans"


# ================================================================
# POMOĆNE FUNKCIJE
# ================================================================

def load_window(run_dir, columns):
    """
    Učitava samo vremenski prozor koji je korišćen za izračunavanje
    metrika u summary.json datoteci.
    """

    run_dir = Path(run_dir)

    with open(run_dir / "summary.json", encoding="utf-8") as f:
        summary = json.load(f)

    start = summary["analysis_window"]["start_time_s"]
    end = summary["analysis_window"]["end_time_s"]

    df = pd.read_csv(run_dir / "timeseries.csv")

    df["time_s"] = pd.to_numeric(
        df["time_s"],
        errors="coerce"
    )

    df = df[
        (df["time_s"] >= start) &
        (df["time_s"] <= end)
    ].copy()

    needed = ["time_s"] + list(columns)

    for c in needed:
        df[c] = pd.to_numeric(
            df[c],
            errors="coerce"
        )

    df = df.dropna(subset=needed)

    df["t_rel"] = df["time_s"] - start

    return df


def ensemble(run_dirs, column, dt=0.05):
    """
    Interpoliše ponovljena izvršavanja na zajedničku vremensku osu
    i vraća srednju vrednost i standardnu devijaciju.
    """

    series = []

    for run_dir in run_dirs:
        df = load_window(
            run_dir,
            [column]
        )

        t = df["t_rel"].to_numpy(float)
        y = df[column].to_numpy(float)

        if len(t) >= 2:
            series.append((t, y))

    if not series:
        raise RuntimeError(
            f"Nema validnih podataka za kolonu: {column}"
        )

    common_start = max(
        t[0]
        for t, _ in series
    )

    common_end = min(
        t[-1]
        for t, _ in series
    )

    grid = np.arange(
        common_start,
        common_end + 0.5 * dt,
        dt
    )

    vals = np.vstack([
        np.interp(
            grid,
            t,
            y
        )
        for t, y in series
    ])

    mean = np.mean(
        vals,
        axis=0
    )

    if len(vals) > 1:
        std = np.std(
            vals,
            axis=0,
            ddof=1
        )
    else:
        std = np.zeros_like(mean)

    return grid, mean, std


def prepare_axes(ax):
    """
    Standardni izgled vremenskih grafika.
    """

    ax.axhline(
        0.0,
        linewidth=0.7,
        linestyle="--",
        alpha=0.5
    )

    ax.grid(
        True,
        linewidth=0.4,
        alpha=0.25
    )

    ax.tick_params(
        labelsize=8
    )


def prepare_xy_axes(ax):
    """
    Standardni izgled XY grafika putanje.
    """

    ax.grid(
        True,
        linewidth=0.4,
        alpha=0.25
    )

    ax.tick_params(
        labelsize=8
    )

    ax.set_xlabel(
        "X [m]",
        fontsize=9
    )

    ax.set_ylabel(
        "Y [m]",
        fontsize=9
    )

    ax.set_aspect(
        "equal",
        adjustable="box"
    )


def save(fig, name):
    """
    Čuva figuru istovremeno kao PNG i PDF.
    """

    fig.tight_layout()

    fig.savefig(
        FIG_DIR / f"{name}.png",
        dpi=DPI,
        bbox_inches="tight"
    )

    fig.savefig(
        FIG_DIR / f"{name}.pdf",
        bbox_inches="tight"
    )

    plt.close(fig)


def load_xy_window(run_dir):
    df = load_window(
        run_dir,
        ["x", "y", "ref_x", "ref_y", "boundary_departure_value"]
    )

    for c in ["x", "y", "ref_x", "ref_y", "boundary_departure_value"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")

    return df


def first_boundary_departure_point(df, threshold=0.0):
    if "boundary_departure_value" not in df.columns:
        return None

    s = pd.to_numeric(df["boundary_departure_value"], errors="coerce").fillna(0.0)
    idx = s > threshold

    if idx.any():
        first_idx = df.index[idx][0]
        row = df.loc[first_idx]
        if pd.notna(row["x"]) and pd.notna(row["y"]):
            return float(row["x"]), float(row["y"])
    return None


def plot_xy_comparison(ax, ref_df, stock_df, integrated_df, title=None):
    # Referentna putanja
    ref_valid = ref_df.dropna(subset=["ref_x", "ref_y"])
    ax.plot(
        ref_valid["ref_x"],
        ref_valid["ref_y"],
        linewidth=2.0,
        linestyle="--",
        label="Referentna putanja"
    )

    # Stock
    stock_valid = stock_df.dropna(subset=["x", "y"])
    ax.plot(
        stock_valid["x"],
        stock_valid["y"],
        linewidth=1.8,
        label="Stock MPC + PI/PID"
    )

    # Integrated
    int_valid = integrated_df.dropna(subset=["x", "y"])
    ax.plot(
        int_valid["x"],
        int_valid["y"],
        linewidth=1.8,
        label="Integrisani MPC"
    )

    # Oznaka početka
    if len(ref_valid) > 0:
        ax.scatter(
            [ref_valid["ref_x"].iloc[0]],
            [ref_valid["ref_y"].iloc[0]],
            s=35,
            marker="o",
            label="Početak"
        )

    # Oznaka kraja reference
    if len(ref_valid) > 0:
        ax.scatter(
            [ref_valid["ref_x"].iloc[-1]],
            [ref_valid["ref_y"].iloc[-1]],
            s=35,
            marker="s",
            label="Kraj reference"
        )

    # Prva tačka izlaska iz granice
    stock_fail = first_boundary_departure_point(stock_valid)
    if stock_fail is not None:
        ax.scatter(
            [stock_fail[0]],
            [stock_fail[1]],
            s=45,
            marker="x",
            label="Stock: izlazak iz granice"
        )

    int_fail = first_boundary_departure_point(int_valid)
    if int_fail is not None:
        ax.scatter(
            [int_fail[0]],
            [int_fail[1]],
            s=45,
            marker="x",
            label="Integrisani: izlazak iz granice"
        )

    ax.grid(True, linewidth=0.4, alpha=0.25)
    ax.set_aspect("equal", adjustable="box")
    ax.tick_params(labelsize=8)

    ax.set_xlabel("x [m]", fontsize=9)
    ax.set_ylabel("y [m]", fontsize=9)

    if title:
        ax.set_title(title, fontsize=10)

    # Legenda izvan grafika
    ax.legend(
        fontsize=7.5,
        frameon=False,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.22),
        ncol=2
    )


def first_boundary_departure(run_dir):
    """
    Pronalazi prvu tačku u kojoj je monitor registrovao
    boundary_departure_value > 0.

    Ako takva tačka ne postoji ili signal nije dostupan,
    vraća None.
    """

    run_dir = Path(run_dir)

    with open(run_dir / "summary.json", encoding="utf-8") as f:
        summary = json.load(f)

    start = summary["analysis_window"]["start_time_s"]
    end = summary["analysis_window"]["end_time_s"]

    df = pd.read_csv(
        run_dir / "timeseries.csv"
    )

    needed = [
        "time_s",
        "x",
        "y",
        "boundary_departure_value",
    ]

    for c in needed:
        df[c] = pd.to_numeric(
            df[c],
            errors="coerce"
        )

    df = df[
        (df["time_s"] >= start) &
        (df["time_s"] <= end)
    ].copy()

    df = df.dropna(
        subset=[
            "x",
            "y",
            "boundary_departure_value",
        ]
    )

    crossed = df[
        df["boundary_departure_value"] > 0
    ]

    if crossed.empty:
        return None

    row = crossed.iloc[0]

    return float(row["x"]), float(row["y"])


def plot_boundary_marker(ax, run_dir, label):
    """
    Obeležava prvu detektovanu tačku napuštanja granice.
    """

    point = first_boundary_departure(
        run_dir
    )

    if point is None:
        return

    x, y = point

    ax.scatter(
        [x],
        [y],
        marker="x",
        s=38,
        linewidths=1.3,
        label=label,
        zorder=5
    )


# ================================================================
# GRUPE IZVRŠAVANJA
# ================================================================

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


# ================================================================
# GRANICNI SCENARIJI S4 I S5
# ================================================================

stock_s4 = (
    RESULTS /
    "baseline_stock" /
    "S04" /
    "run_01"
)

integrated_s4 = (
    RESULTS /
    "integrated_mpc_v1" /
    "S04" /
    "run_401"
)

stock_s5 = (
    RESULTS /
    "baseline_stock" /
    "S05" /
    "run_01"
)

# Sva tri integrisana S5 izvršavanja završila su kao FAIL.
# Run 502 se koristi za grafički prikaz kao reprezentativno
# ponavljanje, jer se njegova lateralna RMSE vrednost nalazi između
# vrednosti dobijenih u run_501 i run_503.
integrated_s5 = (
    RESULTS /
    "integrated_mpc_v1" /
    "S05" /
    "run_502"
)

stock_s4 = [
    Path.home() / "control_benchmark" / "results" / "baseline_stock" / "S04" / "run_01",
]

integrated_s4 = [
    Path.home() / "control_benchmark" / "results" / "integrated_mpc_v1" / "S04" / "run_401",
]

stock_s5 = [
    Path.home() / "control_benchmark" / "results" / "baseline_stock" / "S05" / "run_01",
]

integrated_s5 = [
    Path.home() / "control_benchmark" / "results" / "integrated_mpc_v1" / "S05" / "run_503",
]


# ================================================================
# FIGURA 1
# S2 – uticaj kompenzacije kašnjenja
# ================================================================

fig, ax = plt.subplots(
    figsize=FIGSIZE
)

for label, dirs in {
    "Без компензације": delay_off,
    "Са компензацијом": delay_on,
    "Коначно подешен": final_s2,
}.items():

    t, mean, std = ensemble(
        dirs,
        "lateral_error_eval"
    )

    ax.plot(
        t,
        mean,
        linewidth=1.35,
        label=label
    )

    ax.fill_between(
        t,
        mean - std,
        mean + std,
        alpha=0.13
    )

prepare_axes(ax)

# Ne prikazuje se dugačak period nakon neuspeha testa.
ax.set_xlim(
    0,
    20
)

ax.set_ylim(
    -1.6,
    2.8
)

ax.set_xlabel(
    "Време [s]",
    fontsize=9
)

ax.set_ylabel(
    "Латерална грешка [m]",
    fontsize=9
)

ax.legend(
    fontsize=7.5,
    frameon=False,
    loc="upper right"
)

save(
    fig,
    "01_s2_uticaj_kompenzacije_kasnjenja"
)


# ================================================================
# FIGURA 2
# S3 – lateralna greška
# ================================================================

fig, ax = plt.subplots(
    figsize=FIGSIZE
)

for label, dirs in {
    "Постојећи MPC + PID": stock_s3,
    "Интегрисани MPC": integrated_s3,
}.items():

    t, mean, std = ensemble(
        dirs,
        "lateral_error_eval"
    )

    ax.plot(
        t,
        mean,
        linewidth=1.3,
        label=label
    )

    ax.fill_between(
        t,
        mean - std,
        mean + std,
        alpha=0.13
    )

prepare_axes(ax)

ax.set_xlim(
    0,
    54
)

ax.set_ylim(
    -0.27,
    0.23
)

ax.set_xlabel(
    "Време [s]",
    fontsize=9
)

ax.set_ylabel(
    "Латерална грешка [m]",
    fontsize=9
)

ax.legend(
    fontsize=7.2,
    frameon=False,
    loc="lower center"
)

save(
    fig,
    "02_s3_lateralna_greska"
)


# ================================================================
# FIGURA 3
# S3 – greška brzine
# ================================================================

fig, ax = plt.subplots(
    figsize=FIGSIZE
)

for label, dirs in {
    "Постојећи MPC + PID": stock_s3,
    "Интегрисани MPC": integrated_s3,
}.items():

    t, mean, std = ensemble(
        dirs,
        "velocity_error_eval"
    )

    ax.plot(
        t,
        mean,
        linewidth=1.3,
        label=label
    )

    ax.fill_between(
        t,
        mean - std,
        mean + std,
        alpha=0.13
    )

prepare_axes(ax)

ax.set_xlim(
    0,
    54
)

ax.set_ylim(
    -0.60,
    1.30
)

ax.set_xlabel(
    "Време [s]",
    fontsize=9
)

ax.set_ylabel(
    "Грешка брзине [m/s]",
    fontsize=9
)

ax.legend(
    fontsize=7.2,
    frameon=False,
    loc="upper left"
)

save(
    fig,
    "03_s3_greska_brzine"
)


# ================================================================
# FIGURA 4
# S3 – brzina promene ubrzanja
# ================================================================

fig, ax = plt.subplots(
    figsize=FIGSIZE
)

for label, dirs in {
    "Постојећи MPC + PID": stock_s3,
    "Интегрисани MPC": integrated_s3,
}.items():

    t, mean, std = ensemble(
        dirs,
        "jerk_eval"
    )

    ax.plot(
        t,
        mean,
        linewidth=1.3,
        label=label
    )

    ax.fill_between(
        t,
        mean - std,
        mean + std,
        alpha=0.13
    )

prepare_axes(ax)

ax.set_xlim(
    0,
    54
)

ax.set_ylim(
    -2.4,
    2.5
)

ax.set_xlabel(
    "Време [s]",
    fontsize=9
)

ax.set_ylabel(
    "Брзина промене убрзања [m/s³]",
    fontsize=8.5
)

ax.legend(
    fontsize=7.2,
    frameon=False,
    loc="lower center"
)

save(
    fig,
    "04_s3_jerk"
)


# ================================================================
# FIGURA 5
# S3 – brzina promene ugla upravljanja
# ================================================================

fig, ax = plt.subplots(
    figsize=FIGSIZE
)

for label, dirs in {
    "Постојећи MPC + PID": stock_s3,
    "Интегрисани MPC": integrated_s3,
}.items():

    t, mean, std = ensemble(
        dirs,
        "steering_rate_eval"
    )

    ax.plot(
        t,
        mean,
        linewidth=1.3,
        label=label
    )

    ax.fill_between(
        t,
        mean - std,
        mean + std,
        alpha=0.13
    )

prepare_axes(ax)

ax.set_xlim(
    0,
    54
)

ax.set_ylim(
    -1.0,
    1.25
)

ax.set_xlabel(
    "Време [s]",
    fontsize=9
)

ax.set_ylabel(
    "Брзина промене угла управљања [rad/s]",
    fontsize=8.2
)

ax.legend(
    fontsize=7.2,
    frameon=False,
    loc="upper left"
)

save(
    fig,
    "05_s3_brzina_promene_upravljanja"
)



# ================================================================
# FIGURE 6
# S4 – XY putanja
# ================================================================

fig, ax = plt.subplots(figsize=(5.8, 4.6))

ref_s4 = load_xy_window(stock_s4[0])
stock_s4_df = load_xy_window(stock_s4[0])
integrated_s4_df = load_xy_window(integrated_s4[0])

plot_xy_comparison(
    ax,
    ref_s4,
    stock_s4_df,
    integrated_s4_df,
    title="S4 – Kontinuirano zakrivljena putanja"
)

save(fig, "06_s4_xy_putanja")


# ================================================================
# FIGURE 7
# S5 – XY putanja
# ================================================================

fig, ax = plt.subplots(figsize=(5.8, 4.6))

ref_s5 = load_xy_window(stock_s5[0])
stock_s5_df = load_xy_window(stock_s5[0])
integrated_s5_df = load_xy_window(integrated_s5[0])

plot_xy_comparison(
    ax,
    ref_s5,
    stock_s5_df,
    integrated_s5_df,
    title="S5 – Polukružno kretanje"
)

save(fig, "07_s5_xy_putanja")

# ================================================================
# KRAJ
# ================================================================

print()
print("============================================================")
print("Финалне фигуре су сачуване у:")
print(FIG_DIR)
print("============================================================")
