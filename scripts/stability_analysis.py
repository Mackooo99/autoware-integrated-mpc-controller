#!/usr/bin/env python3

"""
Local stability analysis for the integrated MPC controller.

This script analyses the LOCAL stability of the frozen,
linearized, unconstrained closed-loop approximation of the
integrated MPC controller at multiple operating points
(v_ref, kappa_ref).

For every operating point it:

1. builds the continuous-time Jacobians Ac, Bc;
2. performs exact ZOH discretization to obtain Ad, Bd;
3. constructs the local Q and R matrices using the current
   integrated MPC weights;
4. solves the discrete algebraic Riccati equation (DARE);
5. computes the local LQ feedback gain K;
6. forms the closed-loop matrix

       A_cl = Ad - Bd K;

7. computes the closed-loop eigenvalues and spectral radius;
8. constructs an independent quadratic Lyapunov certificate
   by solving

       A_cl^T P_L A_cl - P_L = -I;

9. verifies that P_L is positive definite and that the
   Lyapunov equation residual is numerically small.

IMPORTANT:
This is NOT a proof of global stability of the nonlinear,
constrained MPC system.

It is a local asymptotic stability analysis of the frozen,
linearized, unconstrained closed-loop approximation.

Usage:

    python3 stability_analysis.py \
        --output-dir runs/stability_lyapunov_20260921
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import (
    expm,
    solve_discrete_are,
    solve_discrete_lyapunov,
)


# ---------------------------------------------------------------------
# Import current MPC weights
# ---------------------------------------------------------------------

sys.path.insert(
    0,
    str(Path(__file__).resolve().parent / "tuning"),
)

import param_space  # noqa: E402


# ---------------------------------------------------------------------
# Vehicle / MPC parameters
# ---------------------------------------------------------------------

WHEEL_BASE = 2.79
STEER_TAU = 0.27
PREDICTION_DT = 0.1

MAX_STEER_RAD = 0.7

NX = 5  # [e_y, e_psi, e_v, delta, a]
NU = 2  # [delta_cmd, jerk]


# Numerical tolerances used only for verification.
STABILITY_TOL = 1e-9
POSITIVE_DEFINITE_TOL = 1e-10
LYAPUNOV_RESIDUAL_TOL = 1e-8


# ---------------------------------------------------------------------
# Continuous model
# ---------------------------------------------------------------------

def continuous_jacobians(
    v_ref: float,
    kappa_ref: float,
    delta_ref: float,
):
    """
    Continuous-time Jacobians Ac and Bc evaluated at the
    on-reference equilibrium:

        e_y   = 0
        e_psi = 0
        e_v   = 0
        delta = delta_ref

    The state vector is

        x = [e_y, e_psi, e_v, delta, a]^T

    and the input vector is

        u = [delta_cmd, jerk]^T.
    """

    Ac = np.zeros((NX, NX))
    Bc = np.zeros((NX, NU))

    # e_y_dot = v sin(e_psi)
    Ac[0, 1] = v_ref
    Ac[0, 2] = 0.0

    # e_psi_dot
    # At e_y = 0 and e_psi = 0, D = 1.
    Ac[1, 0] = -(kappa_ref ** 2) * v_ref
    Ac[1, 1] = 0.0

    Ac[1, 2] = (
        math.tan(delta_ref) / WHEEL_BASE
        - kappa_ref
    )

    Ac[1, 3] = (
        v_ref
        / (
            WHEEL_BASE
            * math.cos(delta_ref) ** 2
        )
    )

    # e_v_dot = a - a_ref
    Ac[2, 4] = 1.0

    # delta_dot =
    # (delta_cmd - delta) / tau_delta
    Ac[3, 3] = -1.0 / STEER_TAU
    Bc[3, 0] = 1.0 / STEER_TAU

    # a_dot = jerk
    Bc[4, 1] = 1.0

    return Ac, Bc


# ---------------------------------------------------------------------
# Exact ZOH discretization
# ---------------------------------------------------------------------

def discretize(
    Ac: np.ndarray,
    Bc: np.ndarray,
    dt: float,
):
    """
    Exact zero-order-hold discretization using the augmented
    matrix exponential.
    """

    n = Ac.shape[0]
    m = Bc.shape[1]

    M = np.zeros((n + m, n + m))

    M[:n, :n] = Ac
    M[:n, n:] = Bc

    expM = expm(M * dt)

    Ad = expM[:n, :n]
    Bd = expM[:n, n:]

    return Ad, Bd


# ---------------------------------------------------------------------
# Independent discretization guard
# ---------------------------------------------------------------------

def verify_decoupled_subblock(
    dt: float,
    tau: float,
):
    """
    Checks the steering and acceleration sub-block against
    their exact analytical ZOH solutions.
    """

    Ac, Bc = continuous_jacobians(
        v_ref=1.0,
        kappa_ref=0.0,
        delta_ref=0.0,
    )

    Ad, Bd = discretize(
        Ac,
        Bc,
        dt,
    )

    expected_Ad33 = math.exp(-dt / tau)
    expected_Bd30 = 1.0 - math.exp(-dt / tau)

    assert abs(
        Ad[3, 3] - expected_Ad33
    ) < 1e-9

    assert abs(
        Bd[3, 0] - expected_Bd30
    ) < 1e-9

    assert abs(
        Ad[4, 4] - 1.0
    ) < 1e-12

    assert abs(
        Bd[4, 1] - dt
    ) < 1e-9


# ---------------------------------------------------------------------
# MPC/LQ cost matrices
# ---------------------------------------------------------------------

def v1_cost_matrices(
    v_ref: float,
):
    """
    Constructs local Q and R matrices using the current
    integrated MPC default weights from param_space.py.

    The velocity-dependent heading and steering weights are
    evaluated at the current operating point.
    """

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
            w["r_delta"]
            + w["r_delta_v"] * v_ref ** 2,
            w["r_j"],
        ]
    )

    return Q, R


# ---------------------------------------------------------------------
# Local stability analysis
# ---------------------------------------------------------------------

def local_stability_metrics(
    v_ref: float,
    kappa_ref: float,
):
    """
    Computes all local closed-loop stability metrics for one
    frozen operating point.

    Returns None at zero vehicle speed because the lateral
    position/orientation states cannot be controlled through
    steering when the vehicle is stationary.
    """

    if abs(v_ref) < 1e-12:
        return None

    delta_ref = math.atan(
        WHEEL_BASE * kappa_ref
    )

    # Skip operating points requiring steering beyond the
    # physical steering limit used by the controller.
    if abs(delta_ref) > MAX_STEER_RAD:
        return None

    Ac, Bc = continuous_jacobians(
        v_ref,
        kappa_ref,
        delta_ref,
    )

    Ad, Bd = discretize(
        Ac,
        Bc,
        PREDICTION_DT,
    )

    Q, R = v1_cost_matrices(v_ref)

    # --------------------------------------------------------------
    # DARE and local LQ feedback
    # --------------------------------------------------------------

    try:
        P_lqr = solve_discrete_are(
            Ad,
            Bd,
            Q,
            R,
        )
    except (np.linalg.LinAlgError, ValueError):
        return None

    K = np.linalg.solve(
        R + Bd.T @ P_lqr @ Bd,
        Bd.T @ P_lqr @ Ad,
    )

    A_cl = Ad - Bd @ K

    # --------------------------------------------------------------
    # Eigenvalue stability test
    # --------------------------------------------------------------

    eigenvalues = np.linalg.eigvals(A_cl)

    spectral_radius = float(
        np.max(
            np.abs(eigenvalues)
        )
    )

    stable_eigenvalues = (
        spectral_radius
        <
        1.0 - STABILITY_TOL
    )

    # --------------------------------------------------------------
    # DARE matrix diagnostics
    # --------------------------------------------------------------

    P_lqr_sym = 0.5 * (
        P_lqr + P_lqr.T
    )

    lambda_min_P_lqr = float(
        np.min(
            np.linalg.eigvalsh(
                P_lqr_sym
            )
        )
    )

    # --------------------------------------------------------------
    # Independent Lyapunov certificate
    #
    # We solve:
    #
    # A_cl^T P_L A_cl - P_L = -I
    #
    # Therefore:
    #
    # V(x) = x^T P_L x
    #
    # gives
    #
    # Delta V = -x^T x < 0
    #
    # for every nonzero state, provided P_L > 0.
    # --------------------------------------------------------------

    Q_lyap = np.eye(NX)

    try:
        P_lyap = solve_discrete_lyapunov(
            A_cl.T,
            Q_lyap,
        )
    except (np.linalg.LinAlgError, ValueError):
        return None

    P_lyap_sym = 0.5 * (
        P_lyap + P_lyap.T
    )

    lambda_min_P_lyap = float(
        np.min(
            np.linalg.eigvalsh(
                P_lyap_sym
            )
        )
    )

    positive_P_lyap = (
        lambda_min_P_lyap
        >
        POSITIVE_DEFINITE_TOL
    )

    # --------------------------------------------------------------
    # Verify the Lyapunov equation numerically
    # --------------------------------------------------------------

    lyapunov_residual = (
        A_cl.T
        @ P_lyap_sym
        @ A_cl
        - P_lyap_sym
        + Q_lyap
    )

    lyapunov_residual_inf = float(
        np.linalg.norm(
            lyapunov_residual,
            ord=np.inf,
        )
    )

    residual_ok = (
        lyapunov_residual_inf
        <
        LYAPUNOV_RESIDUAL_TOL
    )

    lyapunov_certified = (
        stable_eigenvalues
        and positive_P_lyap
        and residual_ok
    )

    return {
        "delta_ref": delta_ref,
        "Ac": Ac,
        "Bc": Bc,
        "Ad": Ad,
        "Bd": Bd,
        "Q": Q,
        "R": R,
        "K": K,
        "A_cl": A_cl,
        "eigenvalues": eigenvalues,
        "spectral_radius": spectral_radius,
        "lambda_min_P_lqr": lambda_min_P_lqr,
        "lambda_min_P_lyap": lambda_min_P_lyap,
        "lyapunov_residual_inf":
            lyapunov_residual_inf,
        "stable_eigenvalues":
            stable_eigenvalues,
        "positive_P_lyap":
            positive_P_lyap,
        "residual_ok":
            residual_ok,
        "lyapunov_certified":
            lyapunov_certified,
    }


# ---------------------------------------------------------------------
# Verification tests
# ---------------------------------------------------------------------

def run_verification_tests():
    """
    Basic sanity checks before the full operating-point sweep.
    """

    moderate = local_stability_metrics(
        v_ref=5.0,
        kappa_ref=0.0,
    )

    assert moderate is not None

    assert (
        moderate["spectral_radius"]
        < 0.99
    ), (
        "Moderate-speed straight-road "
        "spectral radius unexpectedly high."
    )

    assert (
        moderate["lambda_min_P_lyap"]
        > 0.0
    ), (
        "Lyapunov matrix is not "
        "positive definite."
    )

    assert (
        moderate["lyapunov_residual_inf"]
        < LYAPUNOV_RESIDUAL_TOL
    ), (
        "Discrete Lyapunov equation "
        "residual too large."
    )

    # Near-zero but non-zero speed.
    near_zero = local_stability_metrics(
        v_ref=0.5,
        kappa_ref=0.0,
    )

    assert near_zero is not None

    assert np.all(
        np.isfinite(
            near_zero["eigenvalues"]
        )
    )

    # Exactly zero speed is intentionally excluded.
    zero_speed = local_stability_metrics(
        v_ref=0.0,
        kappa_ref=0.0,
    )

    assert zero_speed is None

    # Tight curve at low speed.
    tight_curve = local_stability_metrics(
        v_ref=1.0,
        kappa_ref=0.1,
    )

    assert tight_curve is not None

    assert math.isfinite(
        tight_curve["spectral_radius"]
    )

    print()
    print("Verification tests OK")
    print("---------------------")

    print(
        "Moderate-speed straight-road point:"
    )

    print(
        f"  rho(A_cl)             = "
        f"{moderate['spectral_radius']:.8f}"
    )

    print(
        f"  lambda_min(P_L)       = "
        f"{moderate['lambda_min_P_lyap']:.8e}"
    )

    print(
        f"  Lyapunov residual inf = "
        f"{moderate['lyapunov_residual_inf']:.8e}"
    )

    print(
        f"  Lyapunov certified    = "
        f"{moderate['lyapunov_certified']}"
    )

    print()


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def main():

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=
        argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
    )

    parser.add_argument(
        "--velocities",
        type=float,
        nargs="+",
        default=[
            1.0,
            3.0,
            5.0,
            8.0,
            12.0,
            16.0,
            20.0,
        ],
    )

    parser.add_argument(
        "--curvatures",
        type=float,
        nargs="+",
        default=[
            -0.10,
            -0.05,
            -0.02,
            -0.01,
            0.0,
            0.01,
            0.02,
            0.05,
            0.10,
        ],
    )

    args = parser.parse_args()

    # --------------------------------------------------------------
    # Independent correctness guard
    # --------------------------------------------------------------

    verify_decoupled_subblock(
        PREDICTION_DT,
        STEER_TAU,
    )

    print(
        "Correctness guard OK: "
        "exact ZOH steering/acceleration "
        "sub-block verified."
    )

    # --------------------------------------------------------------
    # Basic verification tests
    # --------------------------------------------------------------

    run_verification_tests()

    # --------------------------------------------------------------
    # Output directory
    # --------------------------------------------------------------

    args.output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    rows = []
    skipped = []

    # --------------------------------------------------------------
    # Operating-point sweep
    # --------------------------------------------------------------

    for v_ref in args.velocities:

        for kappa_ref in args.curvatures:

            metrics = local_stability_metrics(
                v_ref,
                kappa_ref,
            )

            if metrics is None:

                skipped.append(
                    (
                        v_ref,
                        kappa_ref,
                    )
                )

                continue

            rows.append(
                {
                    "v_ref":
                        v_ref,

                    "kappa_ref":
                        kappa_ref,

                    "delta_ref":
                        metrics["delta_ref"],

                    "spectral_radius":
                        metrics["spectral_radius"],

                    "lambda_min_P_lqr":
                        metrics[
                            "lambda_min_P_lqr"
                        ],

                    "lambda_min_P_lyap":
                        metrics[
                            "lambda_min_P_lyap"
                        ],

                    "lyapunov_residual_inf":
                        metrics[
                            "lyapunov_residual_inf"
                        ],

                    "stable":
                        metrics[
                            "stable_eigenvalues"
                        ],

                    "positive_P_lyap":
                        metrics[
                            "positive_P_lyap"
                        ],

                    "residual_ok":
                        metrics[
                            "residual_ok"
                        ],

                    "lyapunov_certified":
                        metrics[
                            "lyapunov_certified"
                        ],

                    "eigenvalues":
                        metrics[
                            "eigenvalues"
                        ],
                }
            )

    if not rows:
        raise RuntimeError(
            "No valid operating points "
            "were analysed."
        )

    # --------------------------------------------------------------
    # Main CSV
    # --------------------------------------------------------------

    csv_path = (
        args.output_dir
        / "stability_sweep.csv"
    )

    with open(
        csv_path,
        "w",
        newline="",
    ) as f:

        writer = csv.writer(f)

        writer.writerow(
            [
                "v_ref",
                "kappa_ref",
                "delta_ref_rad",
                "spectral_radius",
                "lambda_min_P_lqr",
                "lambda_min_P_lyap",
                "lyapunov_residual_inf",
                "stable",
                "positive_P_lyap",
                "residual_ok",
                "lyapunov_certified",
            ]
        )

        for r in rows:

            writer.writerow(
                [
                    r["v_ref"],
                    r["kappa_ref"],
                    r["delta_ref"],
                    r["spectral_radius"],
                    r["lambda_min_P_lqr"],
                    r["lambda_min_P_lyap"],
                    r["lyapunov_residual_inf"],
                    r["stable"],
                    r["positive_P_lyap"],
                    r["residual_ok"],
                    r["lyapunov_certified"],
                ]
            )

    print(
        f"Wrote {csv_path} "
        f"({len(rows)} operating points)"
    )

    # --------------------------------------------------------------
    # Eigenvalue CSV
    # --------------------------------------------------------------

    eig_path = (
        args.output_dir
        / "closed_loop_eigenvalues.csv"
    )

    with open(
        eig_path,
        "w",
        newline="",
    ) as f:

        writer = csv.writer(f)

        writer.writerow(
            [
                "v_ref",
                "kappa_ref",
                "eigenvalue_index",
                "real",
                "imag",
                "magnitude",
            ]
        )

        for r in rows:

            for i, eig in enumerate(
                r["eigenvalues"]
            ):

                writer.writerow(
                    [
                        r["v_ref"],
                        r["kappa_ref"],
                        i,
                        float(eig.real),
                        float(eig.imag),
                        float(abs(eig)),
                    ]
                )

    print(
        f"Wrote {eig_path}"
    )

    # --------------------------------------------------------------
    # Global analysis
    # --------------------------------------------------------------

    unstable = [
        r
        for r in rows
        if not r["stable"]
    ]

    uncertified = [
        r
        for r in rows
        if not r[
            "lyapunov_certified"
        ]
    ]

    min_sr_row = min(
        rows,
        key=lambda r:
            r["spectral_radius"],
    )

    max_sr_row = max(
        rows,
        key=lambda r:
            r["spectral_radius"],
    )

    min_lyap_row = min(
        rows,
        key=lambda r:
            r["lambda_min_P_lyap"],
    )

    max_residual_row = max(
        rows,
        key=lambda r:
            r["lyapunov_residual_inf"],
    )

    min_sr = min_sr_row[
        "spectral_radius"
    ]

    max_sr = max_sr_row[
        "spectral_radius"
    ]

    min_lambda_P_lqr = min(
        r["lambda_min_P_lqr"]
        for r in rows
    )

    min_lambda_P_lyap = min(
        r["lambda_min_P_lyap"]
        for r in rows
    )

    max_residual = max(
        r["lyapunov_residual_inf"]
        for r in rows
    )

    # --------------------------------------------------------------
    # Console summary
    # --------------------------------------------------------------

    print()
    print("=" * 72)
    print("LOCAL STABILITY SWEEP SUMMARY")
    print("=" * 72)

    print(
        f"Analysed operating points : "
        f"{len(rows)}"
    )

    print(
        f"Skipped operating points  : "
        f"{len(skipped)}"
    )

    print()

    print(
        "Spectral radius range:"
    )

    print(
        f"  min rho(A_cl) = "
        f"{min_sr:.8f}"
    )

    print(
        f"    at v_ref="
        f"{min_sr_row['v_ref']:.3f} m/s, "
        f"kappa="
        f"{min_sr_row['kappa_ref']:.5f} 1/m"
    )

    print(
        f"  max rho(A_cl) = "
        f"{max_sr:.8f}"
    )

    print(
        f"    at v_ref="
        f"{max_sr_row['v_ref']:.3f} m/s, "
        f"kappa="
        f"{max_sr_row['kappa_ref']:.5f} 1/m"
    )

    print()

    print(
        "Positive-definiteness margins:"
    )

    print(
        f"  min lambda_min(P_LQR) = "
        f"{min_lambda_P_lqr:.8e}"
    )

    print(
        f"  min lambda_min(P_L)   = "
        f"{min_lambda_P_lyap:.8e}"
    )

    print(
        f"    worst Lyapunov point at "
        f"v_ref="
        f"{min_lyap_row['v_ref']:.3f} m/s, "
        f"kappa="
        f"{min_lyap_row['kappa_ref']:.5f} 1/m"
    )

    print()

    print(
        "Lyapunov numerical consistency:"
    )

    print(
        f"  max residual infinity norm = "
        f"{max_residual:.8e}"
    )

    print(
        f"    at v_ref="
        f"{max_residual_row['v_ref']:.3f} m/s, "
        f"kappa="
        f"{max_residual_row['kappa_ref']:.5f} 1/m"
    )

    print()

    print(
        f"Unstable points            : "
        f"{len(unstable)}"
    )

    print(
        f"Non-certified points       : "
        f"{len(uncertified)}"
    )

    print()

    if not unstable:

        print(
            "PASS: rho(A_cl) < 1 for all "
            "analysed operating points."
        )

    else:

        print(
            "WARNING: some operating points "
            "have rho(A_cl) >= 1."
        )

        for r in unstable:

            print(
                "  "
                f"v_ref={r['v_ref']}, "
                f"kappa={r['kappa_ref']}, "
                f"rho="
                f"{r['spectral_radius']:.8f}"
            )

    if not uncertified:

        print(
            "PASS: all analysed operating "
            "points satisfy the local "
            "quadratic Lyapunov certificate."
        )

    else:

        print(
            "WARNING: some operating points "
            "do not satisfy the complete "
            "Lyapunov certificate."
        )

        for r in uncertified:

            print(
                "  "
                f"v_ref={r['v_ref']}, "
                f"kappa={r['kappa_ref']}, "
                f"rho="
                f"{r['spectral_radius']:.8f}, "
                f"lambda_min(P_L)="
                f"{r['lambda_min_P_lyap']:.8e}, "
                f"residual="
                f"{r['lyapunov_residual_inf']:.8e}"
            )

    print("=" * 72)
    print()

    # --------------------------------------------------------------
    # Text summary for direct use / sharing
    # --------------------------------------------------------------

    summary_path = (
        args.output_dir
        / "stability_summary.txt"
    )

    with open(
        summary_path,
        "w",
    ) as f:

        f.write(
            "LOCAL STABILITY ANALYSIS SUMMARY\n"
        )

        f.write(
            "================================\n\n"
        )

        f.write(
            f"Analysed operating points: "
            f"{len(rows)}\n"
        )

        f.write(
            f"Skipped operating points: "
            f"{len(skipped)}\n\n"
        )

        f.write(
            f"Minimum spectral radius: "
            f"{min_sr:.10f}\n"
        )

        f.write(
            f"Maximum spectral radius: "
            f"{max_sr:.10f}\n"
        )

        f.write(
            "Worst spectral-radius point: "
            f"v_ref="
            f"{max_sr_row['v_ref']}, "
            f"kappa_ref="
            f"{max_sr_row['kappa_ref']}\n\n"
        )

        f.write(
            f"Minimum lambda_min(P_LQR): "
            f"{min_lambda_P_lqr:.10e}\n"
        )

        f.write(
            f"Minimum lambda_min(P_Lyapunov): "
            f"{min_lambda_P_lyap:.10e}\n"
        )

        f.write(
            "Worst Lyapunov-margin point: "
            f"v_ref="
            f"{min_lyap_row['v_ref']}, "
            f"kappa_ref="
            f"{min_lyap_row['kappa_ref']}\n\n"
        )

        f.write(
            "Maximum Lyapunov equation "
            "residual (inf norm): "
            f"{max_residual:.10e}\n\n"
        )

        f.write(
            f"Unstable operating points: "
            f"{len(unstable)}\n"
        )

        f.write(
            f"Non-certified operating points: "
            f"{len(uncertified)}\n\n"
        )

        f.write(
            "All spectral-radius checks PASS: "
            f"{len(unstable) == 0}\n"
        )

        f.write(
            "All Lyapunov checks PASS: "
            f"{len(uncertified) == 0}\n"
        )

    print(
        f"Wrote {summary_path}"
    )

    # --------------------------------------------------------------
    # Spectral-radius heatmap
    # --------------------------------------------------------------

    try:
        import matplotlib

        matplotlib.use("Agg")

        import matplotlib.pyplot as plt

        vs = sorted(
            set(
                r["v_ref"]
                for r in rows
            )
        )

        ks = sorted(
            set(
                r["kappa_ref"]
                for r in rows
            )
        )

        grid = np.full(
            (
                len(ks),
                len(vs),
            ),
            np.nan,
        )

        for r in rows:

            i = ks.index(
                r["kappa_ref"]
            )

            j = vs.index(
                r["v_ref"]
            )

            grid[i, j] = (
                r["spectral_radius"]
            )

        plt.figure(
            figsize=(7, 5)
        )

        im = plt.imshow(
            grid,
            aspect="auto",
            origin="lower",
            extent=[
                min(vs),
                max(vs),
                min(ks),
                max(ks),
            ],
        )

        plt.colorbar(
            im,
            label="spectral radius",
        )

        plt.xlabel(
            "v_ref [m/s]"
        )

        plt.ylabel(
            "kappa_ref [1/m]"
        )

        plt.title(
            "Local closed-loop spectral radius"
        )

        plt.tight_layout()

        plot_path = (
            args.output_dir
            / "spectral_radius_heatmap.png"
        )

        plt.savefig(
            plot_path,
            dpi=200,
        )

        plt.close()

        print(
            f"Wrote {plot_path}"
        )

    except ImportError:

        print(
            "matplotlib not available - "
            "heatmap skipped."
        )


if __name__ == "__main__":
    main()
