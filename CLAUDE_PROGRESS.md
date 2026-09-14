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

## Next task

Task 3 — longitudinal command semantics.
