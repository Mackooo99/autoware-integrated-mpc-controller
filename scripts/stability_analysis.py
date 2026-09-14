#!/usr/bin/env python3
"""Local stability analysis tooling for the integrated MPC (Task 15).

NOT a proof of global constrained-MPC stability. This computes, at each
swept (v_ref, kappa_ref) operating point, the LOCAL closed-loop behaviour
of the UNCONSTRAINED infinite-horizon LQ feedback law implied by the V1
cost weights around the on-reference equilibrium (e_y=e_psi=e_v=0,
delta=delta_ref, a=a_ref):

  1. the continuous Jacobians Ac/Bc (CLAUDE.md #6, the same formulas
     already validated in integrated_kinematics_bicycle_model.cpp);
  2. the exact affine/ZOH discretization Ad/Bd (same construction as
     model_sanity_check.cpp - Ad(3,3)/Bd(3,0)/Ad(4,4)/Bd(4,1) are checked
     below against the tau/dt-only closed form CLAUDE.md documents,
     independent of v/kappa);
  3. the discrete-time infinite-horizon LQ gain K (scipy's DARE solver)
     for the frozen-operating-point LTI pair (Ad, Bd) against the V1 cost
     weights (config/integrated_mpc.param.yaml, via param_space.py);
  4. the closed-loop matrix A_cl = Ad - Bd @ K and its eigenvalues/
     spectral radius.

What this does NOT show:
  - constrained closed-loop stability (steering/jerk/accel/steer-rate
    bounds change the actual feedback law whenever active - this analysis
    is unconstrained);
  - global stability across the true LTV trajectory (each point here is
    its own frozen-in-time LTI approximation, not a joint/switched-system
    analysis);
  - recursive feasibility (a separate MPC-theory question, requiring a
    terminal set/cost argument this does not attempt).

Usage:
  python3 stability_analysis.py --output-dir runs/stability_2026_09_15
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import expm, solve_discrete_are

sys.path.insert(0, str(Path(__file__).resolve().parent / "tuning"))
import param_space  # noqa: E402

WHEEL_BASE = 2.79
STEER_TAU = 0.27
PREDICTION_DT = 0.1
NX = 5  # [e_y, e_psi, e_v, delta, a]
NU = 2  # [delta_cmd, jerk]


def continuous_jacobians(v_ref: float, kappa_ref: float, delta_ref: float):
    """Ac, Bc linearized at the on-reference equilibrium (e_y=e_psi=0),
    reproducing CLAUDE.md #6's Frenet model and the validated Jacobian
    entries (audit item 4). D = 1 - kappa*e_y = 1 at e_y=0."""
    Ac = np.zeros((NX, NX))
    Bc = np.zeros((NX, NU))

    Ac[0, 1] = v_ref  # d(e_y)/d(e_psi) = v*cos(e_psi), e_psi=0
    Ac[0, 2] = 0.0  # d(e_y)/d(e_v) = sin(e_psi), e_psi=0

    Ac[1, 0] = -(kappa_ref ** 2) * v_ref  # -kappa^2*v*cos(e_psi)/D^2, D=1
    Ac[1, 1] = 0.0  # kappa*v*sin(e_psi)/D, e_psi=0
    Ac[1, 2] = math.tan(delta_ref) / WHEEL_BASE - kappa_ref  # tan(delta)/L - kappa*cos/D
    Ac[1, 3] = v_ref / (WHEEL_BASE * math.cos(delta_ref) ** 2)  # v*sec^2(delta)/L

    Ac[2, 4] = 1.0
    Ac[3, 3] = -1.0 / STEER_TAU

    Bc[3, 0] = 1.0 / STEER_TAU
    Bc[4, 1] = 1.0

    return Ac, Bc


def discretize(Ac: np.ndarray, Bc: np.ndarray, dt: float):
    """Exact affine/ZOH discretization via augmented matrix exponential -
    same construction as integrated_kinematics_bicycle_model.cpp (the
    affine/constant term is unused here since we linearize exactly at the
    equilibrium, where it is zero; kept out for a pure Ad/Bd pair)."""
    n = Ac.shape[0]
    m = Bc.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = Ac
    M[:n, n:] = Bc
    expM = expm(M * dt)
    Ad = expM[:n, :n]
    Bd = expM[:n, n:]
    return Ad, Bd


def verify_decoupled_subblock(dt: float, tau: float):
    """Ad(3,3)/Bd(3,0)/Ad(4,4)/Bd(4,1) depend only on tau/dt (delta and a
    dynamics are decoupled from e_y/e_psi/e_v), so they can be checked
    against CLAUDE.md #12's documented closed form independent of any
    (v, kappa) operating point - a correctness guard for this
    reimplementation, not a re-derivation."""
    Ac, Bc = continuous_jacobians(v_ref=1.0, kappa_ref=0.0, delta_ref=0.0)
    Ad, Bd = discretize(Ac, Bc, dt)
    expected_Ad33 = math.exp(-dt / tau)
    expected_Bd30 = 1.0 - math.exp(-dt / tau)
    assert abs(Ad[3, 3] - expected_Ad33) < 1e-9, (Ad[3, 3], expected_Ad33)
    assert abs(Bd[3, 0] - expected_Bd30) < 1e-9, (Bd[3, 0], expected_Bd30)
    assert abs(Ad[4, 4] - 1.0) < 1e-12
    assert abs(Bd[4, 1] - dt) < 1e-9


def v1_cost_matrices(v_ref: float):
    """Q (5x5), R (2x2) for the frozen-v_ref LTI local analysis, using the
    current V1 default weights (param_space.py, mirrors
    config/integrated_mpc.param.yaml). Velocity-squared-scaled weights
    (heading, steering) are evaluated at this operating point's v_ref,
    matching mpc_cost_builder.cpp's convention. Steering-state weight is
    intentionally zero in V1 (CLAUDE.md #8)."""
    w = param_space.default_config()
    Q = np.diag(
        [
            w["q_y"],
            w["q_psi"] * v_ref ** 2,
            w["q_v"],
            0.0,
            w["q_a"],
        ]
    )
    R = np.diag(
        [
            w["r_delta"] + w["r_delta_v"] * v_ref ** 2,
            w["r_j"],
        ]
    )
    return Q, R


def local_closed_loop_eigenvalues(v_ref: float, kappa_ref: float):
    """Returns (eigenvalues, spectral_radius), or (None, None) if the
    frozen-operating-point pair (Ad, Bd) is not stabilizable - which is
    the CORRECT answer at v_ref=0 (and only there): with zero speed the
    e_y/e_psi rows of Ac lose all coupling to steering/velocity, so
    lateral position/heading are structurally uncontrolled (you cannot
    steer a stationary vehicle's position), not a numerical artefact."""
    delta_ref = math.atan(WHEEL_BASE * kappa_ref)
    Ac, Bc = continuous_jacobians(v_ref, kappa_ref, delta_ref)
    Ad, Bd = discretize(Ac, Bc, PREDICTION_DT)
    Q, R = v1_cost_matrices(v_ref)

    try:
        P = solve_discrete_are(Ad, Bd, Q, R)
    except np.linalg.LinAlgError:
        return None, None

    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    A_cl = Ad - Bd @ K

    eigenvalues = np.linalg.eigvals(A_cl)
    spectral_radius = float(np.max(np.abs(eigenvalues)))
    return eigenvalues, spectral_radius


def run_verification_tests():
    """Simple-operating-point checks (Task 15's "add verification tests").
    Raises AssertionError (nonzero exit) on failure rather than only
    warning, so this can gate a build the way the C++ sanity checks do."""
    # Straight road, moderate speed: the least coupled, easiest-to-reason
    # -about case - must be clearly, comfortably stable.
    _, sr_straight_moderate = local_closed_loop_eigenvalues(v_ref=5.0, kappa_ref=0.0)
    assert sr_straight_moderate < 0.99, (
        f"straight-road moderate-speed spectral radius unexpectedly high: {sr_straight_moderate}"
    )

    # Near-zero (but nonzero) speed must not crash and must resolve to a
    # finite result.
    eigenvalues_near_zero, sr_near_zero = local_closed_loop_eigenvalues(v_ref=0.5, kappa_ref=0.0)
    assert eigenvalues_near_zero is not None, "v_ref=0.5 unexpectedly not stabilizable"
    assert np.all(np.isfinite(eigenvalues_near_zero)), "v_ref=0.5 produced non-finite eigenvalues"

    # Exactly v_ref=0 IS a genuine mathematical singularity here (see
    # local_closed_loop_eigenvalues docstring), not something to paper
    # over - confirm it is detected and reported as such (None), not a
    # crash or a silently wrong number.
    eigenvalues_zero_speed, sr_zero_speed = local_closed_loop_eigenvalues(v_ref=0.0, kappa_ref=0.0)
    assert eigenvalues_zero_speed is None and sr_zero_speed is None, (
        "expected v_ref=0 to be reported as not stabilizable, got a result instead"
    )

    # A tight curve at low speed is the most demanding realistic case in
    # the sweep range - still must resolve to a finite spectral radius
    # (whether or not it is < 1 is exactly what the sweep investigates;
    # this only guards against a crash/NaN, not a specific stability
    # verdict).
    _, sr_tight_curve = local_closed_loop_eigenvalues(v_ref=1.0, kappa_ref=0.1)
    assert math.isfinite(sr_tight_curve), "tight low-speed curve produced a non-finite spectral radius"

    print("Verification tests OK: straight-road moderate-speed spectral radius "
          f"= {sr_straight_moderate:.4f} (< 0.99); near-zero-speed and tight-curve "
          "operating points resolve to finite results; exactly v_ref=0 is "
          "correctly reported as not stabilizable rather than crashing.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--velocities", type=float, nargs="+", default=[1.0, 3.0, 5.0, 8.0, 12.0, 16.0, 20.0]
    )
    parser.add_argument(
        "--curvatures",
        type=float,
        nargs="+",
        default=[-0.10, -0.05, -0.02, -0.01, 0.0, 0.01, 0.02, 0.05, 0.10],
    )
    args = parser.parse_args()

    verify_decoupled_subblock(PREDICTION_DT, STEER_TAU)
    print("Correctness guard OK: decoupled delta/a sub-block matches the "
          "tau/dt-only closed form (CLAUDE.md #12).")

    run_verification_tests()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    skipped = []

    for v_ref in args.velocities:
        for kappa_ref in args.curvatures:
            if abs(WHEEL_BASE * kappa_ref) >= 1.0:
                # atan(L*kappa) domain sanity guard; not a physically
                # meaningful steering angle beyond this anyway.
                continue

            eigenvalues, spectral_radius = local_closed_loop_eigenvalues(v_ref, kappa_ref)
            if eigenvalues is None:
                skipped.append((v_ref, kappa_ref))
                continue

            rows.append(
                {
                    "v_ref": v_ref,
                    "kappa_ref": kappa_ref,
                    "spectral_radius": spectral_radius,
                    "eigenvalues_real": [float(e.real) for e in eigenvalues],
                    "eigenvalues_imag": [float(e.imag) for e in eigenvalues],
                    "stable": spectral_radius < 1.0,
                }
            )

    csv_path = args.output_dir / "stability_sweep.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["v_ref", "kappa_ref", "spectral_radius", "stable"])
        for r in rows:
            writer.writerow([r["v_ref"], r["kappa_ref"], r["spectral_radius"], r["stable"]])
    print(f"wrote {csv_path} ({len(rows)} operating points)")

    if skipped:
        print(f"{len(skipped)} operating point(s) not stabilizable (skipped, not a crash): {skipped}")

    unstable = [r for r in rows if not r["stable"]]
    if unstable:
        print(f"WARNING: {len(unstable)} operating point(s) have spectral radius >= 1 "
              "under the UNCONSTRAINED local LQ approximation:")
        for r in unstable:
            print(f"  v_ref={r['v_ref']}, kappa_ref={r['kappa_ref']}: "
                  f"spectral_radius={r['spectral_radius']:.4f}")
    else:
        print("All swept operating points: spectral radius < 1 "
              "(locally stable under the unconstrained LQ approximation).")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        vs = sorted(set(r["v_ref"] for r in rows))
        ks = sorted(set(r["kappa_ref"] for r in rows))
        grid = np.full((len(ks), len(vs)), np.nan)
        for r in rows:
            i = ks.index(r["kappa_ref"])
            j = vs.index(r["v_ref"])
            grid[i, j] = r["spectral_radius"]

        plt.figure(figsize=(7, 5))
        im = plt.imshow(
            grid, aspect="auto", origin="lower",
            extent=[min(vs), max(vs), min(ks), max(ks)],
        )
        plt.colorbar(im, label="spectral radius")
        plt.xlabel("v_ref [m/s]")
        plt.ylabel("kappa_ref [1/m]")
        plt.title("Local closed-loop spectral radius (unconstrained LQ approximation)")
        plt.tight_layout()
        plot_path = args.output_dir / "spectral_radius_heatmap.png"
        plt.savefig(plot_path)
        plt.close()
        print(f"wrote {plot_path}")
    except ImportError:
        print("matplotlib not available - skipping heatmap, CSV still written.")


if __name__ == "__main__":
    main()
