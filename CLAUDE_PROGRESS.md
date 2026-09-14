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

## Next task

Task 6 — timing/profiling infrastructure.
