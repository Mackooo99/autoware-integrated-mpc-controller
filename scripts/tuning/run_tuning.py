#!/usr/bin/env python3
"""Automated MPC cost-weight tuning framework (CLAUDE.md Batch-1 Task 12).

Two-stage seeded search over the V1 cost weights (param_space.py): a coarse
log-uniform sweep, then local refinement around the best coarse candidates.
Scenarios are split into a development set (used to select the winner) and
a held-out set (only ever used to *report* how the winner generalizes,
never to select it) - so a result is not chosen because it happens to fit
the scenarios it was tuned on.

Each trial is stored under its own control_benchmark results directory
(via the existing run_experiment.sh, unmodified) and recorded in a
manifest.json in --output-dir; re-running with the same --output-dir
resumes, skipping trials already marked done/failed.

IMPORTANT: this only edits config/integrated_mpc.param.yaml for the
duration of a single trial and always restores the original content
afterwards (including on Ctrl-C / exception) - only one trial can run at
a time (sequential), and this script must not be run concurrently with
anything else that expects that file to hold the V1 defaults.

Real scenario execution currently requires the Planning/Scenario Simulator
stack, which is blocked by an unrelated rclcpp segfault (see
CLAUDE_PROGRESS.md). Until that is resolved, use --dry-run, which
exercises every part of this framework (sampling, resume, scoring,
ranking, dev/held-out split, CSV/JSON export) against a synthetic scoring
surface instead of a real simulator run - see mock_trial() below.

Use a fresh --output-dir per invocation kind: a manifest.json produced
under --dry-run holds fabricated scores and must not be resumed from for
a real sweep (or vice versa).

Usage:
  # exercise the whole pipeline without a simulator
  python3 run_tuning.py --dry-run --output-dir runs/dry_run_demo \\
      --coarse-samples 12 --refine-top-k 3 --refine-samples 4

  # real sweep, once the simulator is healthy
  python3 run_tuning.py --output-dir runs/sweep_2026_09_15 \\
      --coarse-samples 20 --refine-top-k 3 --refine-samples 6 \\
      --dev-scenarios 1 2 3 --held-out-scenarios 4 5
"""

import argparse
import csv
import json
import random
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import param_space  # noqa: E402
import scoring  # noqa: E402

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = PACKAGE_ROOT / "config" / "integrated_mpc.param.yaml"
CONTROL_BENCHMARK_ROOT = Path.home() / "control_benchmark"
RUN_EXPERIMENT_SH = CONTROL_BENCHMARK_ROOT / "scripts" / "run_experiment.sh"
RESULTS_ROOT = CONTROL_BENCHMARK_ROOT / "results"

TRIAL_TIMEOUT_S = 240


def load_yaml_text(path: Path) -> str:
    return path.read_text()


def render_overridden_yaml(base_text: str, overrides: dict) -> str:
    """Line-oriented key replacement, preserving comments/formatting/order
    for every key NOT being overridden. Avoids a yaml-library round-trip
    (which can reformat comments) for a file that is also hand-maintained.
    Every override key MUST already exist in the base file - this tunes
    existing V1 weights, it does not introduce new parameters."""
    remaining = dict(overrides)
    lines = base_text.splitlines(keepends=True)
    out = []

    for line in lines:
        stripped = line.strip()
        matched_key = None

        for key in remaining:
            if stripped.startswith(f"{key}:"):
                matched_key = key
                break

        if matched_key is None:
            out.append(line)
            continue

        indent = line[: len(line) - len(line.lstrip())]
        out.append(f"{indent}{matched_key}: {remaining.pop(matched_key)}\n")

    if remaining:
        raise KeyError(
            f"override key(s) not found in base yaml (typo, or not a "
            f"declared V1 weight?): {sorted(remaining)}"
        )

    return "".join(out)


def run_trial_real(controller_name: str, scenario_num: int, run_id: int, config: dict):
    """Writes the trial's weights into config/integrated_mpc.param.yaml,
    runs run_experiment.sh unmodified, restores the base yaml, and reads
    back the resulting summary.json/status.txt."""
    base_text = load_yaml_text(CONFIG_PATH)
    overrides = param_space.to_yaml_overrides(config)

    try:
        CONFIG_PATH.write_text(render_overridden_yaml(base_text, overrides))

        subprocess.run(
            [
                "bash",
                str(RUN_EXPERIMENT_SH),
                controller_name,
                str(scenario_num),
                str(run_id),
            ],
            timeout=TRIAL_TIMEOUT_S,
            check=False,
        )
    finally:
        CONFIG_PATH.write_text(base_text)

    run_dir = RESULTS_ROOT / controller_name / f"S{scenario_num:02d}" / f"run_{run_id:02d}"
    status_file = run_dir / "status.txt"
    summary_file = run_dir / "summary.json"

    status = status_file.read_text().strip() if status_file.exists() else "MISSING"
    summary = json.loads(summary_file.read_text()) if summary_file.exists() else {}

    return status, summary, str(run_dir)


def mock_trial(config: dict, scenario_num: int, rng: random.Random):
    """Synthetic stand-in for run_trial_real, used by --dry-run to validate
    the whole framework (sampling/resume/scoring/ranking/export) without a
    simulator. Scores a config against an arbitrary bowl-shaped surface
    centered on the V1 default, so the framework can be seen actually
    preferring configs closer to that center; occasionally injects a
    "FAIL" to exercise the failure-penalty path."""
    default = param_space.default_config()
    distance_sq = 0.0

    for name, dim in param_space.DIMENSIONS.items():
        # compare in log-space for log-scaled dims, matching how they're
        # sampled
        a = config[name]
        b = default[name]
        if dim.log_scale:
            import math

            distance_sq += (math.log(a) - math.log(b)) ** 2
        else:
            distance_sq += ((a - b) / (dim.high - dim.low)) ** 2

    noise = rng.uniform(-0.05, 0.05)
    lateral_rmse = 0.02 + 0.03 * distance_sq + noise * 0.02
    yaw_rmse = 0.01 + 0.01 * distance_sq
    velocity_rmse = 0.05 + 0.02 * distance_sq
    jerk_rmse = 0.3 + 0.1 * distance_sq
    steering_rate_rmse = 0.01 + 0.005 * distance_sq

    # deterministic-looking occasional failure far from the V1 center, to
    # exercise the failure-penalty path without making the demo flaky
    if distance_sq > 8.0 and rng.random() < 0.3:
        return "FAIL", {}

    summary = {
        "lateral_error": {"rmse": abs(lateral_rmse)},
        "yaw_error": {"rmse": abs(yaw_rmse)},
        "velocity_error": {"rmse": abs(velocity_rmse)},
        "jerk": {"rmse": abs(jerk_rmse)},
        "steering_rate": {"rmse": abs(steering_rate_rmse)},
        "boundary": {},
    }
    return "PASS", summary


class Manifest:
    def __init__(self, path: Path):
        self.path = path
        self.trials = json.loads(path.read_text()) if path.exists() else []

    def find(self, trial_id: str):
        for t in self.trials:
            if t["id"] == trial_id:
                return t
        return None

    def upsert(self, trial: dict):
        existing = self.find(trial["id"])
        if existing is not None:
            existing.update(trial)
        else:
            self.trials.append(trial)
        self.save()

    def save(self):
        self.path.write_text(json.dumps(self.trials, indent=2))


def evaluate_config(
    config: dict,
    config_id: str,
    scenarios,
    controller_prefix: str,
    manifest: Manifest,
    dry_run: bool,
    rng: random.Random,
):
    """Runs (or resumes) `config` on every scenario in `scenarios`, returns
    the mean score across them (worst-case-dominated: any FAIL/MISSING
    contributes the fixed failure penalty via scoring.score_summary, which
    is enough to make a config that fails even one scenario rank behind
    one that passes all of them)."""
    per_scenario_scores = []

    for scenario_num in scenarios:
        trial_id = f"{config_id}_S{scenario_num:02d}"
        existing = manifest.find(trial_id)

        if existing is not None and existing.get("status_stage") in ("done", "failed"):
            per_scenario_scores.append(existing["score"])
            continue

        controller_name = f"{controller_prefix}_{config_id}"

        if dry_run:
            status, summary = mock_trial(config, scenario_num, rng)
            run_dir = None
        else:
            # run_ids are just 1 here - each config gets its own
            # controller_name, so trials never collide on disk.
            status, summary, run_dir = run_trial_real(
                controller_name, scenario_num, 1, config
            )

        score, breakdown = scoring.score_summary(summary, status)
        per_scenario_scores.append(score)

        manifest.upsert(
            {
                "id": trial_id,
                "config_id": config_id,
                "scenario": scenario_num,
                "params": config,
                "status": status,
                "status_stage": "done" if status == "PASS" else "failed",
                "score": score,
                "breakdown": breakdown,
                "run_dir": run_dir,
                # For sensitivity_analysis.py's runtime-vs-configuration
                # plot (Task 13) - populated from a real run's summary.json
                # (Task 6 timing topics); absent/None under --dry-run.
                "callback_time_stats": summary.get("integrated_callback_processing_time_ms"),
                "solver_time_stats": summary.get("integrated_solver_processing_time_ms"),
            }
        )

    return sum(per_scenario_scores) / len(per_scenario_scores)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--dev-scenarios", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--held-out-scenarios", type=int, nargs="+", default=[4, 5])
    parser.add_argument("--coarse-samples", type=int, default=20)
    parser.add_argument("--refine-top-k", type=int, default=3)
    parser.add_argument("--refine-samples", type=int, default=6)
    parser.add_argument("--refine-spread", type=float, default=0.3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--controller-prefix", default="tuning")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate the framework against a synthetic scoring surface "
        "instead of the real simulator (currently blocked - see "
        "CLAUDE_PROGRESS.md). Does not touch config/integrated_mpc.param.yaml.",
    )
    args = parser.parse_args()

    if not args.dry_run and not RUN_EXPERIMENT_SH.exists():
        parser.error(f"run_experiment.sh not found at {RUN_EXPERIMENT_SH}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = Manifest(args.output_dir / "manifest.json")
    rng = random.Random(args.seed)

    # V1 default is always candidate 0, so the sweep always includes a
    # direct comparison against the current design.
    candidates = {"config_000_default": param_space.default_config()}
    for i in range(1, args.coarse_samples):
        candidates[f"config_{i:03d}_coarse"] = param_space.sample_coarse(rng)

    print(f"[coarse] evaluating {len(candidates)} candidates on dev set {args.dev_scenarios}")
    coarse_results = []
    for config_id, config in candidates.items():
        dev_score = evaluate_config(
            config, config_id, args.dev_scenarios, args.controller_prefix, manifest, args.dry_run, rng
        )
        coarse_results.append((dev_score, config_id, config))
        print(f"  {config_id}: dev_score={dev_score:.3f}")

    coarse_results.sort(key=lambda r: r[0])
    top_k = coarse_results[: args.refine_top_k]

    print(f"\n[refine] top {len(top_k)} coarse candidates:")
    for score, config_id, _ in top_k:
        print(f"  {config_id}: dev_score={score:.3f}")

    refine_results = []
    for _, center_id, center_config in top_k:
        for j in range(args.refine_samples):
            config_id = f"{center_id}_refine{j:02d}"
            config = param_space.sample_local(rng, center_config, args.refine_spread)
            dev_score = evaluate_config(
                config, config_id, args.dev_scenarios, args.controller_prefix, manifest, args.dry_run, rng
            )
            refine_results.append((dev_score, config_id, config))
            print(f"  {config_id}: dev_score={dev_score:.3f}")

    all_results = coarse_results + refine_results
    all_results.sort(key=lambda r: r[0])
    best_score, best_id, best_config = all_results[0]

    print(f"\n[best] {best_id}: dev_score={best_score:.3f}")
    print(f"  params: {json.dumps(best_config, indent=2)}")

    held_out_score = None
    if args.held_out_scenarios:
        held_out_score = evaluate_config(
            best_config,
            f"{best_id}_heldout",
            args.held_out_scenarios,
            args.controller_prefix,
            manifest,
            args.dry_run,
            rng,
        )
        print(f"[held-out] {best_id}: held_out_score={held_out_score:.3f} (scenarios {args.held_out_scenarios})")
        if held_out_score > best_score * 1.5:
            print(
                "  WARNING: held-out score is much worse than dev score - "
                "possible overfitting to the dev scenario set."
            )

    # ---- export ----
    results_csv = args.output_dir / "results.csv"
    with open(results_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["config_id", "dev_score", *param_space.DIMENSIONS.keys()])
        for score, config_id, config in all_results:
            writer.writerow([config_id, score, *(config[name] for name in param_space.DIMENSIONS)])

    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "best_config_id": best_id,
                "best_dev_score": best_score,
                "best_params": best_config,
                "held_out_score": held_out_score,
                "dev_scenarios": args.dev_scenarios,
                "held_out_scenarios": args.held_out_scenarios,
                "v1_default_dev_score": next(s for s, cid, _ in coarse_results if cid == "config_000_default"),
                "num_candidates_evaluated": len(all_results),
                "dry_run": args.dry_run,
            },
            indent=2,
        )
    )

    best_config_yaml = args.output_dir / "best_config.param.yaml"
    best_config_yaml.write_text(
        render_overridden_yaml(
            load_yaml_text(CONFIG_PATH), param_space.to_yaml_overrides(best_config)
        )
    )

    print(f"\nWrote {results_csv}, {summary_path}, {best_config_yaml}")


if __name__ == "__main__":
    main()
