#!/usr/bin/env python3

from pathlib import Path
import json

import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# PUTANJE
# ============================================================

RESULTS = Path.home() / "control_benchmark" / "results"
OUT_DIR = Path.home() / "control_benchmark" / "paper_dataset_20260915" / "figures_final"
OUT_DIR.mkdir(parents=True, exist_ok=True)

S4_STOCK = RESULTS / "baseline_stock" / "S04" / "run_01"
S4_INTEGRATED = RESULTS / "integrated_mpc_v1" / "S04" / "run_401"

S5_STOCK = RESULTS / "baseline_stock" / "S05" / "run_01"
# Reprezentativno izvršavanje za prikaz
S5_INTEGRATED = RESULTS / "integrated_mpc_v1" / "S05" / "run_502"

# Ako želiš da promeniš:
# S5_INTEGRATED = RESULTS / "integrated_mpc_v1" / "S05" / "run_501"
# S5_INTEGRATED = RESULTS / "integrated_mpc_v1" / "S05" / "run_503"


# ============================================================
# POMOĆNE FUNKCIJE
# ============================================================

def load_run(run_dir: Path):
    """
    Učitava summary.json i timeseries.csv,
    zatim zadržava samo analysis window.
    """
    run_dir = Path(run_dir)

    with open(run_dir / "summary.json", "r", encoding="utf-8") as f:
        summary = json.load(f)

    df = pd.read_csv(run_dir / "timeseries.csv")

    # Uzimamo analysis window ako postoji
    window = summary.get("analysis_window", {})
    start = window.get("start_time_s", pd.to_numeric(df["time_s"], errors="coerce").min())
    end = window.get("end_time_s", pd.to_numeric(df["time_s"], errors="coerce").max())

    needed_cols = [
        "time_s",
        "x", "y",
        "ref_x", "ref_y",
        "boundary_departure_value",
    ]

    for col in needed_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
        else:
            df[col] = pd.NA

    df = df[(df["time_s"] >= start) & (df["time_s"] <= end)].copy()
    df["t_rel"] = df["time_s"] - start

    # Stvarna putanja
    traj = df.dropna(subset=["x", "y"]).copy()

    # Referentna putanja
    ref = df.dropna(subset=["ref_x", "ref_y"])[["ref_x", "ref_y"]].copy()
    ref = ref.drop_duplicates().reset_index(drop=True)

    return summary, df, traj, ref


def first_departure(df: pd.DataFrame):
    """
    Vraća prvi red u kome je boundary_departure_value > 0.
    Ako nema takvog reda, vraća None.
    """
    if "boundary_departure_value" not in df.columns:
        return None

    dep = df[df["boundary_departure_value"].fillna(0.0) > 0.0]
    if dep.empty:
        return None

    return dep.iloc[0]


def axis_limits(ref, traj_a, traj_b, margin_ratio=0.08):
    """
    Računa granice osa sa malom marginom.
    """
    xs = pd.concat([
        ref["ref_x"],
        traj_a["x"],
        traj_b["x"],
    ], ignore_index=True).dropna()

    ys = pd.concat([
        ref["ref_y"],
        traj_a["y"],
        traj_b["y"],
    ], ignore_index=True).dropna()

    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()

    x_range = max(x_max - x_min, 1.0)
    y_range = max(y_max - y_min, 1.0)

    x_pad = margin_ratio * x_range
    y_pad = margin_ratio * y_range

    return (
        x_min - x_pad, x_max + x_pad,
        y_min - y_pad, y_max + y_pad
    )


def annotate_departure(ax, row, text, color, dx, dy):
    """
    Crta marker za prvi izlazak iz granice i dodaje tekstualnu anotaciju.
    """
    if row is None:
        return

    x = row["x"]
    y = row["y"]

    ax.scatter(
        [x], [y],
        marker="x",
        s=180,
        linewidths=2.6,
        color=color,
        zorder=6
    )

    ax.annotate(
        text,
        xy=(x, y),
        xytext=(dx, dy),
        textcoords="offset points",
        fontsize=9,
        color="black",
        bbox=dict(
            boxstyle="round,pad=0.25",
            fc="white",
            ec=color,
            lw=0.9,
            alpha=0.95
        ),
        arrowprops=dict(
            arrowstyle="-",
            lw=1.0,
            color=color
        ),
        zorder=7
    )


def style_axes(ax):
    ax.grid(True, linewidth=0.5, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    ax.tick_params(labelsize=10)

    for spine in ax.spines.values():
        spine.set_linewidth(1.0)

    ax.set_xlabel("x [m]", fontsize=11)
    ax.set_ylabel("y [m]", fontsize=11)


def save_figure(fig, name):
    fig.savefig(OUT_DIR / f"{name}.png", dpi=300, bbox_inches="tight")
    fig.savefig(OUT_DIR / f"{name}.pdf", bbox_inches="tight")
    plt.close(fig)


def make_xy_figure(
    scenario_title: str,
    stock_dir: Path,
    integrated_dir: Path,
    out_name: str,
    stock_text_offset=(15, 12),
    integrated_text_offset=(15, -28),
):
    """
    Generiše jedan intuitivan XY grafik.
    """

    # Učitavanje
    _, df_stock, traj_stock, ref_stock = load_run(stock_dir)
    _, df_int, traj_int, ref_int = load_run(integrated_dir)

    # Referentnu putanju uzimamo iz stock podatka, ali ako je prazna, uzimamo iz integrisanog
    ref = ref_stock if not ref_stock.empty else ref_int

    # Početak i kraj reference
    ref_start = ref.iloc[0]
    ref_end = ref.iloc[-1]

    # Prvi izlazak
    dep_stock = first_departure(df_stock)
    dep_int = first_departure(df_int)

    # Granice osa
    x0, x1, y0, y1 = axis_limits(ref, traj_stock, traj_int)

    # Figura
    fig, ax = plt.subplots(figsize=(6.8, 5.4))

    # Referentna putanja
    ax.plot(
        ref["ref_x"], ref["ref_y"],
        linestyle="--",
        linewidth=2.2,
        label="Referentna putanja",
        zorder=1
    )

    # Stock putanja
    ax.plot(
        traj_stock["x"], traj_stock["y"],
        linewidth=2.4,
        label="Stock MPC + PI/PID",
        zorder=3
    )

    # Integrisana putanja
    ax.plot(
        traj_int["x"], traj_int["y"],
        linewidth=2.4,
        label="Integrisani MPC",
        zorder=4
    )

    # Početak reference
    ax.scatter(
        [ref_start["ref_x"]], [ref_start["ref_y"]],
        s=90,
        marker="o",
        label="Početak",
        zorder=5
    )

    # Kraj reference
    ax.scatter(
        [ref_end["ref_x"]], [ref_end["ref_y"]],
        s=90,
        marker="s",
        label="Kraj reference",
        zorder=5
    )

    # Anotacije za prvi izlazak
    if dep_stock is not None:
        annotate_departure(
            ax,
            dep_stock,
            f"Stock:\nprvi izlazak = {dep_stock['t_rel']:.2f} s",
            color="tab:red",
            dx=stock_text_offset[0],
            dy=stock_text_offset[1],
        )

    if dep_int is not None:
        annotate_departure(
            ax,
            dep_int,
            f"Integrisani MPC:\nprvi izlazak = {dep_int['t_rel']:.2f} s",
            color="tab:green",
            dx=integrated_text_offset[0],
            dy=integrated_text_offset[1],
        )

    # Stil
    style_axes(ax)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)

    # Naslov - manji i iznad, bez prekrivanja
    ax.set_title(scenario_title, fontsize=12, pad=10)

    # Legenda ispod figure
    ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.15),
        ncol=3,
        frameon=False,
        fontsize=9
    )

    fig.tight_layout()
    save_figure(fig, out_name)

    print(f"[OK] Sačuvano: {OUT_DIR / (out_name + '.png')}")
    print(f"[OK] Sačuvano: {OUT_DIR / (out_name + '.pdf')}")


# ============================================================
# GLAVNI DEO
# ============================================================

if __name__ == "__main__":
    make_xy_figure(
        scenario_title="S4 – Kontinuirano zakrivljena putanja",
        stock_dir=S4_STOCK,
        integrated_dir=S4_INTEGRATED,
        out_name="06_s4_prvi_izlazak_iz_granice",
        stock_text_offset=(14, 10),
        integrated_text_offset=(14, -34),
    )

    make_xy_figure(
        scenario_title="S5 – Polukružno kretanje",
        stock_dir=S5_STOCK,
        integrated_dir=S5_INTEGRATED,
        out_name="07_s5_prvi_izlazak_iz_granice",
        stock_text_offset=(14, 12),
        integrated_text_offset=(14, -32),
    )

    print()
    print("============================================================")
    print("Figure su sačuvane u:")
    print(OUT_DIR)
    print("============================================================")
