# Integrated MPC — Progress

Reflects final state after commits `429d14c` through `13bd258` (Batch 1, Tasks 2–16). Task 1
(`f7fb409`, `controller_node_exe: control_state` diagnostic) precedes this range and is
summarized under "Blocker" below since it is the reason runtime validation is still pending.

Commit order (`878c47c..HEAD`):

```
f7fb409  feat: expose controller_node_exe control_state diagnostic          (Task 1)
429d14c  build: enable Release optimisation by default                      (Task 2)
fe526d7  fix: derive acceleration command from measured state               (Task 3)
3acdca8  feat: publish MPC predicted trajectory                             (Task 4)
32b8886  fix: align parameter file keys                                     (Task 5)
f4e644d  feat: publish controller and solver processing time                (Task 6)
f3e428d  perf: gate debug-only diagnostics behind debug_diagnostics         (Task 7)
1e9d429  docs: document why persistent/warm-start OSQP was not adopted      (Task 8)
898421e  perf: add optional incremental nearest-index search                (Task 9)
d3c3fa1  feat: compensate steering and acceleration actuator delay          (Task 10)
2f2366d  feat: add configurable curvature/reference smoothing               (Task 11)
4bcabae  feat: add automated tuning + sensitivity-analysis tooling          (Tasks 12/13)
c7a8d79  chore: remove accidentally committed __pycache__, add .gitignore
5309fd8  docs: derive/document lateral-accel coupling design (not built)    (Task 14)
2a62070  feat: add local stability analysis tooling                        (Task 15)
13bd258  docs: final static engineering review                             (Task 16)
```

## Blocker (still open, unrelated to this batch)

`mission_planner_container` / `motion_planning_container` (stock, unrelated to
`/control/control_container` where our node loads) segfault inside `librclcpp.so` at the same
code offset across every attempt, both before and unaffected by this batch. Autoware never
reaches `WAITING_FOR_ENGAGE`, so no full closed-loop run has been possible this session. Stock
baseline results from 2026-09-11 prove the full stack has worked on this machine before —
whatever regressed is environmental, not something Task 1–16 touched. **Do not modify stock
Autoware or rclcpp to work around this.** This is the reason every item in "Runtime validation
pending" below is pending.

---

## A. Completed and offline-validated

Everything below built cleanly (`-O3 -DNDEBUG` confirmed after a from-scratch rebuild) and passed
all 8 sanity checks (7 original + `delay_compensation_sanity_check`, added Task 10) as of
`13bd258`. Each item was additionally exercised with a concrete offline/bench check (standalone
node + `scripts/synthetic_input_publisher.py`, dedicated sanity binaries, or `--dry-run` /
synthetic-surface tooling runs) — not just compiled.

- **Task 2 — Release build default.** `CMAKE_BUILD_TYPE` defaults to `Release`. Verified
  `CXX_FLAGS = -O3 -DNDEBUG`. Isolated single-node bench: `~33.3 Hz` on `.../output/control_cmd`,
  matching `ctrl_period=0.03s` — this isolates the build-flag effect only (not a full-graph rate).

- **Task 3 — Longitudinal command-side windup fix.** Acceleration command now derived from
  measured `x0(4)` + one jerk step, not accumulated from the previous *published* command.
  Bench-verified: with measured acceleration held at 0 and jerk pinned near max, `sent accel`
  stayed flat (~+0.0414 m/s²) instead of ramping toward the limit.

- **Task 4 — Real MPC predicted trajectory.** `~/output/predicted_trajectory` now built from the
  actual `optimal_state_prediction`, only published when `solution_valid_for_control`. Bench-
  verified: straight road → 50 finite points, `y≡0`; curved road (`curvature=0.05`) → 50 finite
  points sweeping left as expected.

- **Task 5 — Parameter file alignment.** Fixed `max_steer`/`max_acc`/`min_acc` name mismatches,
  added missing-but-declared params, removed two dead (unconsumed) delay keys. Verified
  `ros2 param dump` matches the yaml exactly.

- **Task 6 — Timing/profiling infrastructure.** New `~/debug/processing_time_ms` (whole callback,
  RAII-timed, fires on every return path) and `~/debug/solver_time_ms` (OSQP-only). Per-section
  chrono checkpoints (S1–S10) logged for offline bottleneck analysis. Bench-verified: callback
  ~3.1–3.6 ms, solver ~0.66–0.73 ms in isolation. **Finding:** S5 (cost-matrix construction, real
  math) was the largest non-solver contributor — flagged for a future batch, not acted on.
  Benchmark-tooling side (`control_benchmark`, no git, changes recorded here): `logger_node.py`
  and `analyze_run.py` extended to log/aggregate the two new topics under distinct
  `integrated_callback`/`integrated_solver` labels, verified end-to-end (272 correctly-labeled
  rows produced in a bench run).

- **Task 7 — Debug-only performance gating.** New `debug_diagnostics` param (default `false`)
  gates only pure-diagnostic computation (Hessian eigendecomposition, H-Hᵀ norm, per-row
  violation string-building, objective cross-check, optimal-sequence preview, reference-
  feasibility scan) — never anything `solution_valid_for_control` or the hard clamps depend on.
  Bench-verified: S6 dropped ~0.52→~0.049 ms (~10×), whole-callback ~3.1–3.6→~2.85 ms; CLOSED LOOP
  output values unchanged, confirming safety logic untouched.

- **Task 8 — OSQP/allocation review.** Reviewed; documented why persistent/warm-start was **not**
  adopted (see "Design-only" section) rather than risking a silent wrong result.

- **Task 9 — Incremental nearest-index search.** Optional bounded-window search with guaranteed
  fallback to the full scan whenever it can't confidently reproduce it (no previous index,
  trajectory replaced, localization jump). Sanity-tested: normal progression, trajectory
  replacement, and a large localization jump all resolve to the *correct* index.

- **Task 10 — Actuator delay compensation (V2).** `delay_compensation.hpp` (`propagateFirstOrder
  Delay`, `propagateIntegratorDelay` — exact ZOH solutions, pure functions) applied to `x0(3)`/
  `x0(4)` using histories of *actually published* commands. New dedicated 5-case sanity check.
  Bench-verified: off → bit-for-bit identical to every prior baseline; on → output measurably
  differs (`sent accel` +0.041→+0.159, `raw jerk` +1.381→+1.062), confirming it only acts when
  enabled.

- **Task 11 — Configurable curvature/reference smoothing.** `calculateCurvature` generalized to a
  symmetric `index±stride` triangle; `stride=1` proven (algebraically and by test) to reproduce V1
  exactly. Threaded through `PredictionReferenceBuilder` (all 5 call sites) and the node. Sanity-
  tested: straight road (~0), constant-radius curve (magnitude `1/R`, correct sign), S-curve sign
  flip, and near-collinear/near-coincident numerical stability.

- **Tasks 12/13 — Tuning framework + sensitivity analysis.** Pure Python, never touches physical
  limits. Coarse log-uniform sweep → top-K local refinement → dev-set selection → held-out report
  (winner never *selected* using held-out scenarios). Real-mode trials write candidate weights
  into the yaml, invoke the existing unmodified `run_experiment.sh`, then always restore the
  original yaml. Validated the *entire* pipeline via `--dry-run` against a synthetic scoring
  surface: correctly selected the V1 default as best, held-out score tracked dev score (no
  overfitting warning), resume reproduced identical scores without re-sampling, and
  `sensitivity_analysis.py` produced 50 correct metric-vs-weight plots from that manifest.

- **Task 15 — Local stability analysis tooling.** Reproduces the validated continuous Jacobians,
  discretizes exactly as the C++ model does, solves the discrete Riccati equation per swept
  `(v_ref, kappa_ref)` operating point. Correctness guard against CLAUDE.md's documented
  tau/dt-only closed form: passes. Hard-assert verification tests (not just warnings): straight-
  road spectral radius `<0.99`; near-zero speed resolves finite; exactly `v_ref=0` correctly
  detected as a genuine mathematical singularity (not a crash). Default 63-point sweep: **all
  locally stable** under the unconstrained LQ approximation, spectral radius in `[0.87, 0.96]`.

- **Task 16 — Final static review.** Confirmed: no tracked stock-Autoware file changed; no
  benchmark baseline/scenario file touched; physical limits unchanged throughout every task
  (`max_steering_angle=0.7`, accel `[-5.0,3.0]`, jerk `[-5.0,2.0]`); clean from-scratch rebuild
  still produces `-O3 -DNDEBUG` and all 8 sanity checks pass; full runtime param dump matches the
  yaml exactly; `control_state` never unconditionally OK; exactly one `control_cmd` publisher.

---

## B. Runtime validation pending

None of the following can be exercised until the segfault blocker (above) clears — all require
the full Planning/Scenario Simulator stack, not just the isolated single-node bench used so far:

- **Full-graph `control_cmd` frequency** under the Release build (Tasks 2/6/7) — only measured in
  an isolated bench (~33.3 Hz); the full graph has ~150+ competing nodes and has not been
  re-measured since the old unoptimized ~6.4 Hz figure.
- **Predicted-trajectory consumer health** (Task 4) — lane departure / AEB / control_validator
  actually consuming and staying healthy on `~/output/predicted_trajectory`.
- **Delay-compensation (V2) real-plant comparison** (Task 10) — V1 vs V2 lateral RMSE / steering
  smoothness on the actual delayed simulator plant, which is what the feature is meant to improve.
- **Curvature-smoothing real-scenario comparison** (Task 11) — `kappa_ref`/`delta_ref`
  before/after on a live scenario (`verify_reference_steer.py`).
- **A real tuning sweep** (Tasks 12/13) — the whole pipeline is validated offline via `--dry-run`
  only; no real `(config, scenario)` trial has been run yet.
- **Diagnostic-graph / autonomous-mode chain end-to-end** (Task 1, prerequisite to all of the
  above) — `control_state` → `/autoware/control` → `/autoware/modes/autonomous` →
  `/system/operation_mode/availability` → gate engagement → `/control/command/control_cmd`
  forwarding the real command instead of the `(0, -1.5)` stop-hold.

---

## C. Features disabled by default (V1 preserved exactly)

| Parameter | Default | Task | Effect when enabled |
|---|---|---|---|
| `debug_diagnostics` | `false` | 7 | Re-enables expensive diagnostic-only computation (eigendecomposition, violation detail, objective cross-check, preview, reference-feasibility scan). |
| `enable_delay_compensation` | `false` | 10 | Replaces `x0(3)`/`x0(4)` with actuator-delay-compensated predictions from published-command history. |
| `use_incremental_nearest_search` | `false` | 9 | Bounded-window nearest-index search instead of the full linear scan (with guaranteed fallback). |
| `curvature_smoothing_stride` | `1` | 11 | Widens the curvature-estimation triangle beyond adjacent points. |

All four were bench/sanity-verified to leave V1 output bit-for-bit or algebraically unchanged at
their default value (see section A for each).

---

## D. Design-only (not implemented)

- **Task 14 — Shared lateral-acceleration constraint.** Linearization of
  `a_y = v²·tan(delta)/L` derived and shown to fit the existing constraint-builder pattern as a
  *hard* constraint. The task-mandated *soft* (slack) form requires widening the QP's decision
  variable from `2N=100` to `150`, reshaping `H`/`f`/`A` and re-auditing every fixed-width read of
  `optimal_input`/`optimal_state_prediction` — judged a structural QP rewrite per the task's own
  stop condition. Full derivation, exact reshaping required, and a proposed future-batch
  implementation plan are in `docs/task14_lateral_acceleration_constraint_design.md`. **No code
  changed; not enabled; no such parameter exists.**

- **Task 8 note (design decision, not a built feature):** persistent/warm-start OSQP was
  evaluated and explicitly **not** adopted — `calCSCMatrix` drops near-zero entries when
  converting to CSC, making the sparsity pattern data-dependent (changes with `v`/`kappa`), which
  would make OSQP's in-place `update_P`/`update_A` unsafe (silent wrong update risk). Documented
  at the solver construction site in `integrated_mpc_node.cpp`; the solver is still reconstructed
  fresh every cycle, unchanged from before this batch.

---

## Next action

**V1 S1 full closed-loop runtime validation.**

Once the segfault blocker clears:

```
cd ~/control_benchmark/scripts
./run_experiment.sh integrated_mpc_v1 1 1
```

using the yaml exactly as committed (`debug_diagnostics=false`, `enable_delay_compensation=false`,
`use_incremental_nearest_search=false`, `curvature_smoothing_stride=1`). Confirm, in order:

1. `control_state` reaches `OK`, `/autoware/modes/autonomous` → true, gate engages, and
   `/control/command/control_cmd` forwards the real command instead of `(0, -1.5)`.
2. The vehicle actually moves; no NaN/Inf published.
3. Steering/acceleration/jerk stay within hard limits; no command-side windup.
4. `/control/trajectory_follower/control_cmd` frequency in the full graph (resolves the Task
   2/6/7 RUNTIME VALIDATION PENDING item above).
5. Lateral/heading/velocity error behave sensibly; lane-departure/AEB/control_validator stay
   healthy with the real predicted trajectory (resolves the Task 4 item above).

Do not tune MPC weights or claim any V2 feature is better than V1 until this produces real
numbers. Only after V1 S1 is reliable, proceed to V1 S2, then the rest of the runtime test order
below.

### Full runtime test order (for reference, after V1 S1 above)

2. V1 S2 — `./run_experiment.sh integrated_mpc_v1 2 1`.
3. Delay-aware V2 S1/S2 — `enable_delay_compensation: true`, compare against V1.
4. Curvature-smoothed V2 S1/S2 — `curvature_smoothing_stride: 5` (or another value), compare.
5. S3, S4, S5 — V1 config, once S1/S2 are confirmed reliable.
6. Weight tuning — `python3 scripts/tuning/run_tuning.py --output-dir scripts/tuning/runs/<name>
   --dev-scenarios 1 2 3 --held-out-scenarios 4 5` (drop `--dry-run`).
7. Held-out validation — automatic at the end of the same `run_tuning.py` invocation; then
   `python3 scripts/tuning/sensitivity_analysis.py --manifest <run>/manifest.json --output-dir
   <run>/sensitivity`.
8. Optional lateral-acceleration-coupled MPC — not built; would need the Task 14 design doc's
   plan implemented first.
