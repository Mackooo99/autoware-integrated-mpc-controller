# Integrated MPC — Progress

## Completed

- **Task 1 — `controller_node_exe: control_state` diagnostic** (commit `f7fb409`)
  - Node renamed `integrated_mpc_node` → `controller_node_exe` (launch `name=` + `Node()` ctor).
  - `diagnostic_updater` task `control_state` added: WARN (inputs/tracking unavailable) / OK
    (`solution_valid_for_control`) / ERROR (QP rejected), independent of `publish_enabled_`.
  - Build: clean. All 7 sanity checks: PASS.
  - Standalone runtime check: `/diagnostics` reports `name: 'controller_node_exe: control_state'`
    exactly as required by `control.yaml`.

## Task 1 live-S1 validation — BLOCKED (not a Task 1 defect)

Two clean attempts to run S1 via `run_scenario.sh 1` both failed identically:

- `mission_planner_container` and `motion_planning_container` (both stock, unrelated to
  `/control/control_container` where our node loads) segfault (`exit code -11`) during startup.
- Kernel log confirms both crashes are inside `librclcpp.so` (`/opt/ros/humble`), at the same
  code offset across all 4 crashed processes over 2 runs (`...cb131` into the mapped region).
- Consequence: Autoware never leaves `PLANNING` state, `WAITING_FOR_ENGAGE` is never reached
  (scenario_test_runner times out with `AutowareError: ... time is up`), so autonomous mode,
  gate engagement and `/control/command/control_cmd` forwarding cannot be exercised end-to-end.

Partial positive evidence that Task 1 is still working correctly despite the block:

- Diagnostic leaf went from the pre-existing `STALE` to real `WARN`/`OK` transitions (confirmed
  in both the `run_scenario` log and the earlier standalone test) — the STALE root cause from
  CLAUDE.md §22 is fixed.
- The custom node itself ran, solved the QP, and reached the closed-loop publish branch with
  plausible values matching CLAUDE.md §16 (e.g. `delta_cmd≈-0.008129`, `jerk≈+2.0`).

Why this is judged pre-existing/unrelated, not a Task 1 regression:

- The crashing containers do not load our code.
- `~/control_benchmark/results/baseline_stock/S01/run_01..03/` contain successful stock runs
  from 2026-09-11, so the full stack has worked on this machine before; whatever changed is
  environmental, not something Task 1 touched.
- Not investigated further (out of scope): whether it's a resource/scheduling race made visible
  by other users' concurrent ROS 2 processes on the same default `ROS_DOMAIN_ID` (0), seen on
  this shared machine (`student1`'s `safety_monitor`/`dashboard` nodes).

**Do not modify stock Autoware or rclcpp to work around this** (CLAUDE.md §33/§34). Needs either
a clean retry (possibly at a quieter time / isolated domain), or explicit user guidance.

## Current blockers

- P0: `mission_planner_container` / `motion_planning_container` segfault in `librclcpp.so` —
  prevents any full closed-loop S1 run (stock-infrastructure issue, not in our package).

- **Task 2 — Release build default** (commit pending)
  - `CMakeLists.txt`: `CMAKE_BUILD_TYPE` defaults to `Release` when unset. Verified
    `CXX_FLAGS = -O3 -DNDEBUG` after a package-scoped clean rebuild (matches stock).
  - All 7 sanity checks: PASS (numerically unchanged despite `-DNDEBUG` disabling Eigen asserts).
  - Added `scripts/synthetic_input_publisher.py`: publishes minimal valid inputs to a standalone
    `integrated_mpc_node` so the real `onTimer` path can be exercised and measured without the
    full Planning/Scenario Simulator stack (which is currently blocked — see below).
  - Frequency measurement (`ros2 topic hz` on `.../output/control_cmd`, isolated single-node
    bench, no other Autoware graph load): **~33.3 Hz**, matching the configured `ctrl_period =
    0.03 s` almost exactly. This isolates the build-flag contribution per the audit's suggested
    measurement order; it is **not** the full-graph rate (CLAUDE.md §28 documented ~6.4 Hz under
    the old unoptimized build **in the full graph**, which has ~150+ competing nodes). Full-graph
    rate after Release remains RUNTIME VALIDATION PENDING until the segfault blocker clears.

- **Task 3 — Fix longitudinal acceleration command-side windup** (commit pending)
  - Re-checked `autoware_control_msgs/msg/Longitudinal.msg` (`acceleration` is the actuating field;
    `jerk` is informational, never consumed by `DELAY_STEER_ACC_GEARED`) and the simulator plant
    (`acc_time_delay=0.1`, `acc_time_constant=0.1`).
  - `src/integrated_mpc_node.cpp` 23.7: replaced
    `a_cmd = previous_acceleration_command_ + jerk_command*ctrl_period` (open-loop accumulator)
    with `a_cmd = x0(4) + jerk_command*ctrl_period` (measured current acceleration, receding-
    horizon consistent with the MPC's own predicted `a_1`), then hard-clamped as before.
  - `previous_acceleration_command_` retained (still used by the fallback branch and the
    first-cycle init).
  - Build: clean. All 7 sanity checks: PASS (unaffected, this code path isn't in a sanity binary).
  - Bench-verified anti-windup with `scripts/synthetic_input_publisher.py`: measured acceleration
    held at 0 while jerk stayed pinned near max, `sent accel` in the CLOSED LOOP log stayed flat
    at ~+0.0414 m/s^2 across all cycles instead of ramping toward the acceleration limit.

- **Task 4 — Publish real MPC predicted trajectory** (commit pending)
  - `src/integrated_mpc_node.cpp` 14.6: builds `~/output/predicted_trajectory` from the actual
    `optimal_state_prediction` (per-horizon-step `e_y, e_psi, e_v, delta, a`) plus each step's own
    reference point pose (`trajectory_->points[trajectory_index]`), using the same convention as
    `TrackingStateBuilder`: `x = x_ref - e_y*sin(yaw_ref)`, `y = y_ref + e_y*cos(yaw_ref)`,
    `psi = yaw_ref + e_psi`, `v = v_ref + e_v`. Only published when `solution_valid_for_control`
    (no fake/heartbeat trajectory on an invalid solve).
  - Added tiny local `yawFromQuaternion`/`yawToQuaternion` helpers (same formula as
    `TrackingStateBuilder::yawFromPose`, which is private and not reused to avoid touching a
    validated builder's interface).
  - Extended `scripts/synthetic_input_publisher.py` with a `curvature` param (straight or
    constant-radius arc) for offline geometric checks.
  - Build: clean. All 7 sanity checks: PASS.
  - Bench-verified offline: straight road -> 50 points, all finite, `y` stays exactly 0, `x`
    spans 2.0-26.0 m. Curved road (`curvature=0.05`, left turn) -> 50 points, all finite,
    `(x,y)` sweeps from (2.0, 0.02) to (19.6, 14.2), consistent with a left-turning arc.
  - Runtime consumer validation (lane departure / AEB / control_validator staying healthy on
    this topic): RUNTIME VALIDATION PENDING (needs the full graph).

- **Task 5 — Parameter/configuration cleanup** (commit pending)
  - `config/integrated_mpc.param.yaml`: renamed `max_steer`->`max_steering_angle`,
    `max_acc`->`max_acceleration`, `min_acc`->`min_acceleration` (previously silently ignored,
    node ran on hardcoded defaults that happened to match).
  - Added the previously-missing-but-declared `nearest_dist_threshold`, `nearest_yaw_threshold`,
    and all four steer-rate map parameters, explicit at their current defaults.
  - Removed `steering_input_delay` and `longitudinal_delay`: not declared/consumed anywhere yet
    (dead keys); documented in-file that they must not return until Task 10 actually declares
    and consumes them.
  - Build: clean (no code change). All 7 sanity checks: PASS.
  - Verified with `ros2 param dump /control/trajectory_follower/controller_node_exe` against a
    node launched with `--params-file` on this yaml: every key/value in the file matches the
    runtime dump exactly, and every declared node parameter is now represented in the file.

- **Task 6 — Timing/profiling infrastructure** (commit pending, controller side;
  benchmark-tooling side has no git history to commit into - see below)
  - `src/integrated_mpc_node.cpp`: `ScopedCallbackTimer` (RAII, fires on every `onTimer` return
    path) publishes whole-callback time on new `~/debug/processing_time_ms`. OSQP-only
    `total_solver_time_ms` (already computed, wall-clock around `solver.optimize()`) published on
    new `~/debug/solver_time_ms`. Both `autoware_internal_debug_msgs/Float64Stamped`, matching the
    stock topic's message type but under integrated-MPC-specific names per CLAUDE.md #34 (no
    lateral/longitudinal split to mislabel).
  - Added chrono checkpoints for S1 tracking_state, S2 reference_horizon, S3 prediction_model, S4
    constraints, S5 cost, S6 debug_validation (symmetry+eigensolver), S7 solver, S8+S9 post-solve
    debug, S10 publish; logged (not published, to avoid topic proliferation) via one throttled
    (1 Hz) `RCLCPP_INFO` block for offline bottleneck analysis (Task 7 input).
  - Build: clean. All 7 sanity checks: PASS.
  - Bench-verified: `~/debug/processing_time_ms` ~3.1-3.6 ms, `~/debug/solver_time_ms` ~0.66-0.73 ms
    (isolated single-node bench, no full-graph contention). Section log for one representative
    cycle: S1=0.005, S2=0.007, S3=0.198, S4=0.020, **S5=1.893**, S6=0.524, S7=0.807, S8+S9=0.167 ms.
  - **Finding for later:** S5 (MpcCostBuilder::build, real math, not debug-only) is the single
    largest non-solver contributor in this bench - bigger than the OSQP solve itself. This is core
    computation, not something Task 7's debug-gating touches (CLAUDE.md #7/#35: don't rewrite a
    validated builder without demonstrated evidence of a *bug*; a performance cost alone isn't
    that). Flagging as a candidate for a *future* batch, not acted on here.
  - Benchmark-tooling side (user-approved minimal extension, CLAUDE.md #33 exception granted this
    session): `~/control_benchmark/monitor_ws/src/control_benchmark_monitor/control_benchmark_monitor/logger_node.py`
    subscribes the two new topics and writes them into the existing `timings.csv` 3-column schema
    under labels `integrated_callback`/`integrated_solver` (existing `lateral`/`longitudinal` rows
    untouched). `~/control_benchmark/scripts/analyze_run.py`: both aggregation loops extended to
    include the two new labels, producing `integrated_callback_processing_time_ms` and
    `integrated_solver_processing_time_ms` in `summary.json` alongside (not replacing) the stock
    keys. Rebuilt `control_benchmark_monitor` (`--symlink-install`, develop mode, package-scoped).
    `control_benchmark` has no `.git` - these two file edits are not git-committable; recorded here
    for traceability instead. Bench-verified end-to-end (controller + synthetic publisher +
    logger_node): `timings.csv` contains 272 correctly-labeled rows each of
    `integrated_callback`/`integrated_solver` with no interference with the stock columns.
    `analyze_run.py`'s summary/plot aggregation itself was only syntax-checked
    (`python3 -m py_compile`), not run end-to-end - it needs the rest of a real scenario's
    row set (odometry, control_evaluator metrics, etc.) that this synthetic bench doesn't produce.

- **Task 7 — Performance cleanup based on measurements** (commit pending)
  - New `debug_diagnostics` param (default `false`, added to yaml). Gates only pure-diagnostic
    computation identified by Task 6's profiling, never anything `solution_valid_for_control` or
    the hard clamps depend on:
    - Hessian eigendecomposition + H-H^T norm (S6) - only fed a throttled log.
    - Per-row violation identification/string-building inside the constraint-violation scan -
      `constraint_violation_count`/`maximum_constraint_violation` (used by
      `solution_valid_for_control`) still computed unconditionally; only the "which row/step"
      detail string is gated.
    - Objective cross-check (`direct_qp_objective`/`full_mpc_cost`) - diagnostic-only.
    - Optimal-sequence preview block - diagnostic-only.
    - Reference-input feasibility scan (S9) - diagnostic-only, entire block gated.
  - Build: clean. All 7 sanity checks: PASS (unaffected - node's onTimer isn't exercised by them).
  - Bench-verified with `debug_diagnostics=false` vs the Task 6 always-on baseline: S6 dropped
    ~0.52ms -> ~0.049ms (~10x), S8+S9 ~0.17ms -> ~0.11ms; whole-callback dropped from ~3.1-3.6ms to
    ~2.85ms in the same isolated bench. Verified CLOSED LOOP output values (steer/jerk/accel)
    unchanged, confirming safety-critical logic is untouched.

- **Task 8 — OSQP/allocation performance review** (commit pending; documentation only, no
  behaviour change)
  - Local `autoware_osqp_interface::OSQPInterface` **does** structurally support a persistent
    workspace + warm start (`updateP`/`updateA`/`updateQ`/`updateBounds`/`setWarmStart`,
    `osqp_interface.hpp`).
  - **Not adopted.** `autoware::osqp_interface::calCSCMatrix` (`csc_matrix_conv.cpp:26-63`) drops
    any entry with `|value| < 1e-9` when converting a dense Eigen matrix to CSC, so the resulting
    sparsity *pattern* is data-dependent. `H`/`A` here depend on velocity and curvature (several
    Jacobian terms are exactly zero at `kappa=0` on a straight segment, nonzero on a curve), so the
    pattern is not provably invariant across operating points. OSQP's `update_P`/`update_A` only
    update values at already-existing sparsity positions - feeding them a matrix whose true
    nonzero pattern has shifted is either a silent wrong update or a hard mismatch. Not something
    to risk in a safety-critical controller; documented in-code at the solver construction site
    rather than implemented as an "unsafe wrapper" (explicit Task 8 instruction).
  - Reviewed for avoidable allocations/copies: the `q`/`lower`/`upper` `std::vector` conversions
    are single linear copies (unavoidable, OSQP's C API needs contiguous `double*`); `H`/`A` are
    passed by `const &` into `optimize()`, no extra copy. No further safe optimisation identified
    without touching QP construction itself (out of scope - "preserve QP mathematics exactly").
  - Build: clean. Sanity checks re-run (osqp, mpc_constraint, plus the other 5): all PASS.

- **Task 9 — Incremental nearest-index search** (commit pending)
  - `TrackingStateBuilder`: new optional `findNearestIndexIncremental` searches only
    `[previous_index - window, previous_index + window]` with the exact same distance/yaw
    acceptance criteria as the full scan; `build()` tries it first (when enabled and a previous
    index exists) and falls back to the existing full `findNearestIndex` whenever the incremental
    result is `nullopt` - which happens structurally on the first cycle, when the previous index is
    out of range for a new/shorter trajectory, and on a large localization jump (nothing in the
    window then satisfies the thresholds). New ctor params
    `use_incremental_search=false, incremental_search_window=15` (defaulted, so the existing 3-arg
    call sites in the node and the sanity test are unaffected); `build()` is no longer `const`
    (needs to remember the previous index) - call sites unaffected since the node holds it via
    `unique_ptr` and the sanity test's local `builder` object was never const either.
  - New node params `use_incremental_nearest_search` (default `false`, preserves V1's exhaustive
    scan) and `incremental_search_window` (default `15`), added to the yaml.
  - Extended `tracking_state_sanity_check.cpp` (append-only, existing tests untouched) with 3 new
    cases: normal progression (x=10 -> index 10, x=11 -> index 11), trajectory replacement
    (previous index 11 out of range for a new 5-point trajectory -> correctly falls back, finds
    index 2), large localization jump on the original 40-point trajectory (x=5 -> index 5, then
    x=35, window doesn't reach it -> correctly falls back, finds index 35).
  - Build: clean. All 7 sanity checks: PASS (tracking_state now covers all 3 new scenarios).

- **Task 10 — Actuator delay-compensation (V2, off by default)** (commit pending)
  - Inspected stock's approach first: `mpc_lateral_controller`'s `SteeringPredictor` and
    `pid_longitudinal_controller`'s `predictedStateAfterDelay` both predict a corrected "now" state
    by propagating through the actuator's own known dynamics using the history of commands the
    controller itself already published, rather than trusting the raw sensor value directly for a
    plant with a pure input delay the reduced control-design model does not include.
  - New header-only `delay_compensation.hpp`: `propagateFirstOrderDelay` (exact ZOH solution of
    `dot(delta)=(delta_cmd-delta)/tau` through a sequence of piecewise-constant published
    commands) and `propagateIntegratorDelay` (exact ZOH solution of `dot(a)=j`). Pure functions,
    no state, independently unit-testable.
  - `integrated_mpc_node.cpp`: new `enable_delay_compensation` param (default `false` - V1
    unchanged bit-for-bit, bench-verified below), `steering_input_delay`/`longitudinal_input_delay`
    (defaults 0.24/0.10, matching the simulator plant). When on, `x0(3)` (steering) and `x0(4)`
    (acceleration) are replaced by the propagated prediction using
    `ceil(delay/ctrl_period)`-length histories of *actually published* commands (both the MPC-
    optimal path and the fallback-hold path push into history, since both really reach the
    actuator). `e_y`/`e_psi`/`e_v` are left as measured, matching how stock scopes its own delay
    compensation to each sub-controller's own state rather than the full pose. Does not add a
    state or change the MPC model/QP.
  - New `delay_compensation_sanity_check` (5 cases: empty-history no-op, steering converges to a
    held constant command, single-step exact value against the closed-form ZOH formula, exact
    jerk integration, and an order-sensitivity check confirming history is applied oldest-first).
  - Build: clean. All 8 sanity checks (7 existing + new): PASS.
  - Bench-verified: with `enable_delay_compensation=false`, CLOSED LOOP output is bit-for-bit
    identical to every prior task's baseline (`sent accel=+0.041435`, `raw jerk=+1.381166`). With
    `enable_delay_compensation=true` (same synthetic inputs, several seconds of runtime so history
    fills), output measurably differs (`sent accel=+0.159255`, `raw jerk=+1.061697`), confirming
    the feature has real effect only when explicitly enabled.
  - Not yet done (needs the real plant, RUNTIME VALIDATION PENDING): comparing V1 vs V2 lateral
    RMSE/steering smoothness on an actual delayed simulator, which is what the delay values are
    ultimately meant to improve.

## Next task

Task 11 — curvature/reference smoothing (configurable).
