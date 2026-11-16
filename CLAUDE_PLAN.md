# Integrated MPC — Verified Audit and Implementation Plan

Phase A output per CLAUDE.md §39/§40. Investigation only; no source file was modified to produce
this document.

Every factual claim below cites a file that was opened during the audit. Claims that could **not**
be verified from local source are collected in §C.4 and are labelled `UNVERIFIED` inline. Nothing
outside those two places is speculative.

Path shorthand used throughout:

| Alias | Absolute path |
|---|---|
| `AW` | `/home/nit-user/autoware-new/Autoware/autoware` |
| `PKG` | `/home/nit-user/integrated_mpc_ws/src/autoware_integrated_mpc_trajectory_follower` |
| `BM` | `/home/nit-user/control_benchmark` |
| `WS` | `/home/nit-user/integrated_mpc_ws` |

---

## A. Architecture map

Runtime path, reconstructed from local launch files and source.

```
/planning/trajectory
        │
        ▼
custom node   name:      integrated_mpc_node
              namespace: /control/trajectory_follower
              30 ms wall timer → onTimer()
        │
        ▼
/control/trajectory_follower/control_cmd     (autoware_control_msgs/msg/Control)
        │
        ├──► autoware_shift_decider                    → /control/shift_decider/gear_cmd
        ├──► autoware_operation_mode_transition_manager (remap: trajectory_follower_control_cmd)
        ├──► topic_state_monitor_trajectory_follower_control_cmd
        └──► vehicle_cmd_gate            (remap: input/auto/control_cmd)
                │
                ▼
        /control/command/control_cmd
                │
                ▼
        autoware_simple_planning_simulator
```

Source for each edge:

- Node names, namespaces and every remap above:
  `AW/src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml`
- Custom node name/namespace/remaps: `PKG/launch/integrated_mpc.launch.xml`
- Custom node timer and publishers: `PKG/src/integrated_mpc_node.cpp:307-335`

### A.1 Stock controller identity

`AW/src/launcher/autoware_launch/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml`
loads the stock controller as a composable node into `/control/control_container`:

```xml
<composable_node pkg="autoware_trajectory_follower_node"
                 plugin="autoware::motion::control::trajectory_follower_node::Controller"
                 name="controller_node_exe" namespace="trajectory_follower">
```

It is wrapped in
`<group if="$(eval "'$(var trajectory_follower_mode)' == 'trajectory_follower_node'")">`.
`PKG/launch/integrated_planning_simulator.launch.xml` sets `trajectory_follower_mode` to
`integrated_mpc_external`, which matches neither that branch nor the `smart_mpc_trajectory_follower`
branch, so the stock controller is not loaded.

### A.2 Simulator plant model

`AW/src/launcher/autoware_launch/vehicle/sample_vehicle_launch/sample_vehicle_description/config/simulator_model.param.yaml`

```
vehicle_model_type: "DELAY_STEER_ACC_GEARED"
acc_time_delay: 0.1        acc_time_constant: 0.1
steer_time_delay: 0.24     steer_time_constant: 0.27
```

Consequence: `longitudinal.acceleration` is the actuating longitudinal signal (not `velocity`), and
the plant contains pure dead time on both channels.

---

## B. Stock vs. custom interface compatibility

| Item | Stock `controller_node_exe` | Custom `integrated_mpc_node` | Verdict |
|---|---|---|---|
| ROS node name | `controller_node_exe` | `integrated_mpc_node` | mismatch (P0) |
| Namespace | `/control/trajectory_follower` | same | match |
| Subscriptions (trajectory, odometry, steering, accel, operation_mode) | 5 | 5, identical remaps | match |
| `~/output/control_cmd` | published | published | match |
| `lateral/predicted_trajectory` | published | publisher created, **never published** | gap (P1) |
| diagnostic task `control_state` | present | **absent** | gap (P0) |
| `force_update()` per cycle | present | n/a | gap (P0) |
| `~/lateral/debug/processing_time_ms` | published | absent | gap (P1) |
| `~/longitudinal/debug/processing_time_ms` | published | absent | gap (P1) |
| `longitudinal.velocity` | `ctrl_cmd.vel` | reference velocity | equivalent |
| `longitudinal.acceleration` | `ctrl_cmd.acc` | open-loop accumulator | defect (P1) |
| `longitudinal.jerk` | never set by stock | set, `is_defined_jerk = true` | benign |
| Steering delay compensation | `input_delay: 0.24` | none | gap (P1) |
| Longitudinal delay compensation | `delay_compensation_time: 0.1` | none | gap (P1) |
| Compile flags | `-O3 -DNDEBUG` | **empty** | gap (P0) |

Sources:

- Stock node name/namespace/remaps — `control.launch.xml` (path in §A).
- Stock `control_state` diagnostic —
  `AW/src/universe/autoware_universe/control/autoware_pid_longitudinal_controller/src/pid_longitudinal_controller.cpp:1221-1241`
- Stock `force_update()` per control cycle —
  `AW/src/universe/autoware_universe/control/autoware_trajectory_follower_node/src/controller_node.cpp:274`
- Stock `setHardwareID` and second diagnostic task — same file, `:66-67`
- Stock longitudinal message construction —
  `.../autoware_pid_longitudinal_controller/src/pid_longitudinal_controller.cpp:914-921`
  (`cmd.velocity`, `cmd.acceleration` set; `cmd.jerk` never assigned)
- Message field semantics — `AW/src/core/autoware_msgs/autoware_control_msgs/msg/Longitudinal.msg`
- Stock delay parameters —
  `AW/src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml:47`
  and `.../longitudinal/pid.param.yaml:3`
- Custom node: entire file read — `PKG/src/integrated_mpc_node.cpp`.
  Node name `:37`; publishers `:307-318`; the `predicted_trajectory_pub_` handle created at `:313`
  has **no `publish()` call anywhere in the file**; there is no `diagnostic_updater` include,
  member, or call anywhere in the file.

---

## C. Root-cause tree

### C.1 Proven P0 blocker — diagnostic contract

Every link in this chain was read in local source.

1. The diagnostic graph requires a leaf diagnostic whose `DiagnosticStatus.name` is
   `controller_node_exe: control_state`.
   `AW/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/control.yaml:61-64`
   ```yaml
   - path: /autoware/control/performance_monitoring/control_state
     type: diag
     node: controller_node_exe
     name: control_state
   ```

2. The aggregator composes the expected name as `"<node>: <name>"`.
   `AW/src/universe/autoware_universe/system/autoware_diagnostic_graph_aggregator/src/common/config/loader.cpp:177-182`
   ```cpp
   const auto diag_node = yaml.required("node").text();
   const auto diag_name = yaml.required("name").text();
   const auto sep = diag_node.empty() ? "" : ": ";
   return diag_node + sep + diag_name;
   ```

3. A missing/stale leaf is promoted to ERROR.
   `.../autoware_diagnostic_graph_aggregator/src/common/logics/diag.cpp:35-39`
   ```cpp
   DiagnosticLevel DiagLogic::level() const {
     // STALE to ERROR
     return std::min(link_->level(), DiagnosticStatus::ERROR);
   }
   ```

4. `/autoware/control` is an `and` over that leaf.
   `control.yaml:1-9` lists `/autoware/control/performance_monitoring/control_state` among its links.

5. `/autoware/modes/autonomous` is an `and` that includes `/autoware/control`.
   `AW/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/autoware-main.yaml:32-41`

6. The aggregator converter turns mode availability into the availability topic.
   `.../autoware_diagnostic_graph_aggregator/src/node/converter.cpp:37,54`
   ```cpp
   create_publisher<OperationModeAvailability>("~/operation_mode/availability", rclcpp::QoS(1));
   out_.autonomous = is_available(autonomous_, out_.autonomous);
   ```

7. ADAPI rejects the transition when the mode is unavailable, and masks the reported availability.
   `AW/src/universe/autoware_universe/system/autoware_default_adapi_universe/src/operation_mode.cpp:42-50, 68-75, 149`
   ```cpp
   const auto name = "/system/operation_mode/availability";
   mode_available_[OperationModeState::Message::AUTONOMOUS] = msg->autonomous;
   if (!mode_available_[mode]) { ... "The target mode is not available. Please check the diagnostics."
   state.is_autonomous_mode_available &= mode_available_[OperationModeState::Message::AUTONOMOUS];
   ```

8. Without engagement the gate replaces the longitudinal command.
   `AW/src/universe/autoware_universe/control/autoware_vehicle_cmd_gate/src/vehicle_cmd_gate.cpp:601-603`
   ```cpp
   if (!is_engaged_) {
     filtered_control.longitudinal = createLongitudinalStopControlCmd();
   }
   ```

9. That command is exactly the observed value.
   `.../vehicle_cmd_gate.cpp:761-769`
   ```cpp
   cmd.velocity = 0.0;
   cmd.acceleration = stop_hold_acceleration_;
   ```
   `AW/src/launcher/autoware_launch/autoware_launch/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml:11`
   ```yaml
   stop_hold_acceleration: -1.5
   ```

**Conclusion.** `velocity = 0, acceleration = -1.5` is `createLongitudinalStopControlCmd()` with the
default `stop_hold_acceleration`. Only the longitudinal field is replaced, which is consistent with
the report that valid steering was still observed downstream. The custom node publishes no
diagnostic at all, so the `control_state` leaf can never be satisfied.

### C.2 Second, independent availability gate

`AW/src/universe/autoware_universe/control/autoware_operation_mode_transition_manager/src/state.cpp:215-222`

```cpp
bool AutonomousMode::isModeChangeAvailable(const InputData & input_data)
{
  if (!input_data.kinematics) return false;
  if (!input_data.trajectory) return false;
  if (!input_data.trajectory_follower_control_cmd) return false;
  if (!input_data.control_cmd) return false;
```

This requires both `/control/trajectory_follower/control_cmd` and `/control/command/control_cmd` to
have been received. It explains `is_autonomous_mode_available = false` while `publish_enabled` was
`false`. At standstill this gate then passes via `.../state.cpp:327-331`
(`allow_autonomous_in_stopped && |v| < 0.01`). The diagnostic graph in §C.1 is therefore the
remaining blocker, which matches the hypothesis in CLAUDE.md §22.

### C.3 Proven P0 — the package is compiled without optimisation

Not previously recorded in CLAUDE.md.

`WS/build/autoware_integrated_mpc_trajectory_follower/CMakeCache.txt`
```
CMAKE_BUILD_TYPE:STRING=
```

`WS/build/autoware_integrated_mpc_trajectory_follower/CMakeFiles/integrated_mpc_node.dir/flags.make`
```
CXX_FLAGS =
```

Stock Autoware, for comparison —
`AW/build/autoware_mpc_lateral_controller/CMakeCache.txt` and its `flags.make`:
```
CMAKE_BUILD_TYPE:STRING=Release
CXX_FLAGS = -O3 -DNDEBUG -fPIC -Wall -Wextra -Wpedantic -Werror -fPIC -std=c++17
```

`PKG/CMakeLists.txt` contains no `CMAKE_BUILD_TYPE`, no `add_compile_options`, and no
`CMAKE_CXX_FLAGS` (verified by grep over the whole file).

Two consequences follow directly from these facts, without further assumption:

1. The custom package runs Eigen with no optimisation and without `-DNDEBUG`, so Eigen's internal
   assertions are compiled in. The `onTimer` hot path contains, per cycle, 50 matrix exponentials
   (`PKG/src/integrated_kinematics_bicycle_model.cpp:292`, `(M * dt).exp()` from
   `unsupported/Eigen/MatrixFunctions`), construction of a 250×100 `Bex`
   (`PKG/src/prediction_matrix_builder.cpp:121-182`), and a 100×100 symmetric eigendecomposition
   (`PKG/src/integrated_mpc_node.cpp:612-629`).
2. Comparing this build's execution time against a `-O3 -DNDEBUG` stock build is not a valid
   timing comparison for the thesis (CLAUDE.md §4, §34).

The *magnitude* of the effect on the observed rate has not been measured. See §C.4.

### C.4 Unverified items — must be measured or checked before being relied upon

1. **How much of the ~6.4 Hz is attributable to the build flags.** Not measured. §C.3 establishes
   only that the flags are absent and that the hot path is Eigen-heavy. The candidate list in §E is
   ordered by inspection, not by measurement.
2. **`diagnostic_updater`'s exact name composition.** The `Updater::update()` body lives in
   `/opt/ros/humble/lib/libdiagnostic_updater.so`, not in
   `/opt/ros/humble/include/diagnostic_updater/diagnostic_updater.hpp`, so it was not read.
   Indirect evidence is strong but is not proof: `lane_departure_checker_node.cpp:152` sets
   `setHardwareID("lane_departure_checker")` while `control.yaml:57-60` expects
   `node: lane_departure_checker_node`, so the prefix must be the ROS **node name**, not the
   hardware id. Confirm at runtime with `ros2 topic echo /diagnostics --once`.
3. **Propagation of `trajectory_follower_mode`** through
   `integrated_planning_simulator.launch.xml` → `planning_simulator.launch.xml` →
   `autoware.launch.xml` → `tier4_control_component.launch.xml` → `control.launch.xml`.
   The custom launch declares the argument but does not pass it explicitly into its `<include>`.
   Prior observation (stock controller absent, `control_state` STALE) indicates it does propagate,
   but the mechanism was not verified in ROS 2 launch source. Assert with `ros2 node list`.
4. **Whether unsmoothed curvature (§D item 20) measurably degrades tracking.** Not measured.
5. **Per-section `onTimer` cost.** Not measured.

### C.5 Correction to CLAUDE.md §28

CLAUDE.md §28 links the ~6.4 Hz rate to the topic-rate diagnostics. The thresholds do not support
that link.

`AW/src/launcher/autoware_launch/autoware_launch/config/system/component_state_monitor/topics.yaml:105-118`
```yaml
node_name_suffix: trajectory_follower_control_cmd
topic: /control/trajectory_follower/control_cmd
warn_rate: 5.0
error_rate: 1.0
timeout: 1.0
```

6.4 Hz is above `warn_rate`, so `/autoware/control/topic_rate_check/trajectory_follower` is OK at
that rate. The WARN/ERROR that was observed on `topic_rate_check/control_command` refers to
`/control/command/control_cmd` (same file, `:119-130`) and is downstream of §C.1. The low rate is a
real problem for CLAUDE.md §4 and §28, but it is not a diagnostic-graph blocker.

### C.6 Secondary effects — do not chase directly

- `control_command_gate` timeout STALE: `use_control_command_gate` defaults to `false`
  (`control.launch.xml`), so that node is not launched. `control.yaml:11-15` makes
  `node_alive_monitoring/command_gate` an **`or`** of `vehicle_cmd_gate` and `control_command_gate`,
  so the missing node does not fail the subtree.
- Lane-departure diagnostic reporting OK is **not** evidence that `predicted_trajectory` is fine.
  `AW/src/universe/autoware_universe/control/autoware_lane_departure_checker/src/lane_departure_checker_node/lane_departure_checker_node.cpp:186-190`
  returns early when `predicted_trajectory_` is null, while the diagnostic callback at `:379-402`
  inspects only `output_` and therefore still reports OK. The check is inactive, not passing.
  This is a CLAUDE.md §34 concern.

---

## D. MPC mathematical audit

Classification per CLAUDE.md §40.D. All line references are in `PKG/`.

| # | Item | Status | Evidence |
|---|---|---|---|
| 1 | State `x = [e_y, e_psi, e_v, delta, a]`, input `u = [delta_cmd, j]` | VALIDATED | consistent across `src/integrated_kinematics_bicycle_model.cpp`, `src/mpc_cost_builder.cpp`, `src/mpc_constraint_builder.cpp` |
| 2 | `e_y = -sin(yaw_ref)·dx + cos(yaw_ref)·dy`, positive = left | VALIDATED | `src/tracking_state_builder.cpp:251-253`; sign-consistent with signed curvature `kappa = 2·cross/(a·b·c)` at `:164-200` |
| 3 | Continuous Frenet model `f` | VALIDATED | `src/integrated_kinematics_bicycle_model.cpp:136-150`; matches the exact Frenet kinematics in CLAUDE.md §6 |
| 4 | Jacobian `Ac = df/dx` | VALIDATED | `:157-190`. All non-zeros re-derived: `Ac(0,1)=v·cos`, `Ac(0,2)=sin`, `Ac(1,0)=-kappa²·v·cos/D²`, `Ac(1,1)=+kappa·v·sin/D`, `Ac(1,2)=tan(delta)/L - kappa·cos/D`, `Ac(1,3)=v·sec²(delta)/L`, `Ac(2,4)=1`, `Ac(3,3)=-1/tau`. `Ac(0,0)=0` is correct: `d(e_y)/dt` has no `e_y` dependence |
| 5 | Input Jacobian `Bc` | VALIDATED | `:197-205`: `Bc(3,0)=1/tau`, `Bc(4,1)=1` |
| 6 | Affine term `cc = f - Ac·x_bar - Bc·u_bar` | VALIDATED | `:218-221`; reproduces `f` exactly at the linearisation point |
| 7 | Exact affine ZOH via augmented `exp(M·dt)` | VALIDATED | `:271-305`. Augmented `M = [[Ac,Bc,cc],[0,0,0],[0,0,0]]`; top block row of `exp(M·dt)` gives `[Ad, Bd, Wd]`. Reproduces CLAUDE.md §12: `Ad(3,3) = exp(-0.1/0.27) = 0.690479` |
| 8 | Extended matrices, `X = [x_1 … x_N]` | VALIDATED | `src/prediction_matrix_builder.cpp:121-182`. `Aacc = A_k···A_0`; `Bacc = A_k·Bacc` then column block `k` set to `B_k`; `Wacc = A_k·Wacc + W_k` |
| 9 | Cost → QP conversion | VALIDATED | `src/mpc_cost_builder.cpp:266-302`. Expanding `(BU+d)ᵀQ(BU+d) + (U-Uref)ᵀR(U-Uref)` gives `H = 2(BᵀQB + R)`, `f = 2(BᵀQd − R·Uref)`, `c = dᵀQd + UrefᵀR·Uref`; all three match the code exactly |
| 10 | OSQP convention `min ½UᵀPU + qᵀU`, `P=H`, `q=f` | VALIDATED | `src/integrated_mpc_node.cpp:656-691`; independently cross-checked against `solver.getObjVal()` every cycle at `:906-952` |
| 11 | `OSQPInterface solver(1.0e-6, true)` | VALIDATED | Constructor is `(c_float eps_abs, bool polish)` — `AW/src/core/autoware_core/common/autoware_osqp_interface/src/osqp_interface.cpp:30-49`. Not a `(bool, int)` overload. Settings there: `max_iter = 4000`, `eps_rel = 1.0e-4` |
| 12 | Terminal cost added at stage `i = N-1` | VALIDATED | `src/mpc_cost_builder.cpp:150-159`. Since `X = [x_1 … x_N]`, `i = N-1` is `x_N` |
| 13 | Steering-angle and jerk box constraints | VALIDATED | `src/mpc_constraint_builder.cpp:272-309` |
| 14 | Acceleration-state constraint via `Bex` row | VALIDATED | `:325-349`: `a_min − a_free ≤ B_a·U ≤ a_max − a_free`, state row `k·NX+4` |
| 15 | First steering-rate constraint uses `ctrl_period` | VALIDATED | `:388-391` `rate_dt = (i == 0) ? control_period : prediction_dt`; `:401-426`. Matches CLAUDE.md §9/§10 |
| 16 | Future steering-rate constraints use `prediction_dt` | VALIDATED | `:427-455` |
| 17 | Solver status / size / finiteness checks | VALIDATED | `src/integrated_mpc_node.cpp:722-756`, `:1191-1199` |
| 18 | Hard clamp on the applied first input | VALIDATED | `:1896-1940`: jerk clamp, first-step steering-rate clamp, absolute steering clamp |
| 19 | `1e-3` acceptance tolerance for constraint residual | LIKELY CORRECT | `:1188`. `eps_rel = 1.0e-4` (`osqp_interface.cpp:39`) is consistent with the ~1e-4 residuals in CLAUDE.md §15; the applied input is clamped independently at `:1896-1940`, so the tolerance never licenses a physical-limit violation |
| 20 | Reference curvature from **adjacent** trajectory points | NEEDS REVIEW | `src/tracking_state_builder.cpp:135-147` uses indices `index-1, index, index+1`. Stock uses a 15-point stride: `AW/src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml:11-12` `curvature_smoothing_num_traj: 15`, `curvature_smoothing_num_ref_steer: 15`. `kappa` feeds both the Jacobian (item 4) and the feed-forward input reference `delta_ref = atan(L·kappa)` (`src/integrated_prediction_builder.cpp:169-173`). Effect on tracking is unmeasured — see §C.4 item 4 |
| 21 | No steering input delay in the model | NEEDS REVIEW | Plant `steer_time_delay: 0.24` (`simulator_model.param.yaml`); stock compensates via `input_delay: 0.24` (`mpc.param.yaml:47`). The custom model represents only `steer_tau = 0.27`. `PKG/config/integrated_mpc.param.yaml` declares `steering_input_delay: 0.24`, but that key is never declared or read in `src/integrated_mpc_node.cpp` |
| 22 | No longitudinal delay/lag in the model | NEEDS REVIEW | Plant `acc_time_delay: 0.1`, `acc_time_constant: 0.1`; stock compensates via `delay_compensation_time: 0.1` (`pid.param.yaml:3`). The custom model assumes `d(a)/dt = j`, i.e. instantaneous actuation |
| 23 | Command-side acceleration accumulator | **BUG** | `src/integrated_mpc_node.cpp:1984-1994`: `a_cmd = previous_acceleration_command_ + jerk·ctrl_period`, clamped, then stored back at `:2073-2074`. It is a pure open-loop integrator, never re-synchronised to the measured acceleration `x0(4)`. It also contradicts the MPC's own model, whose predicted `a_1` is propagated from the **measured** `a_0` |
| 24 | No lower bound on predicted velocity | NEEDS REVIEW | `src/mpc_constraint_builder.cpp:209-216` defines exactly four constraint groups; none bounds `v = v_ref + e_v` |
| 25 | Nearest-index search is a full linear scan each cycle | LIKELY CORRECT (slow) | `src/tracking_state_builder.cpp:76-115` |

**Summary.** No mathematical error was found in the state definition, continuous model, Jacobians,
affine term, ZOH discretisation, horizon indexing, extended matrices, cost construction, terminal
cost, constraint construction, QP convention, or solution extraction. The single outright defect is
item 23, which is a command-publishing defect rather than an MPC formulation defect. Items 20–22
are model-fidelity gaps affecting benchmark validity, not correctness of the implemented equations.

Per CLAUDE.md §7 and §36, nothing in this table justifies rewriting a validated builder.

---

## E. Runtime / performance audit

Configured `ctrl_period = 0.03` s (33.3 Hz); reported rate ~6.4 Hz (~156 ms/cycle); reported OSQP
time ~1.4 ms.

`getRunTime()` returns `latest_work_info_.run_time`
(`AW/src/core/autoware_core/common/autoware_osqp_interface/include/autoware/osqp_interface/osqp_interface.hpp:112`),
which for a freshly set up workspace covers setup plus solve plus polish. The ~1.4 ms figure is
therefore the whole solve, and the remaining time is outside OSQP.

Candidates, ordered by inspection only (**not measured** — §C.4 item 1). All line numbers in
`PKG/src/integrated_mpc_node.cpp` unless stated:

1. Absent optimisation flags (§C.3) — affects every item below.
2. `SelfAdjointEigenSolver` on the 100×100 Hessian, executed unconditionally every cycle at
   `:612-629`. `min_h_eigenvalue` is used only in a throttled log at `:1411`.
3. `(H − Hᵀ).norm()` on 100×100, unconditional, `:606-610`.
4. Constraint-violation scan over 200 rows plus `std::string` concatenation, `:769-899`. The loop
   and the string building sit **outside** the throttle macro, so they run every cycle.
5. Reference-input feasibility check `A · U_ref` plus a 200-row scan, `:1432-1474`, every cycle.
6. Optimal-sequence preview string building, `:979-1104`, every cycle.
7. A fresh `OSQPInterface` constructed per cycle at `:678-680` (acknowledged as provisional in the
   comment at `:671-677`), which forces dense→CSC conversion and a full KKT factorisation and makes
   warm start structurally impossible.
8. 50 matrix exponentials per cycle — `PKG/src/integrated_kinematics_bicycle_model.cpp:292`.
9. Full-trajectory scans each cycle: `PKG/src/tracking_state_builder.cpp:76-115`, and
   `PKG/src/prediction_reference_builder.cpp:67-81` and `:110-151`.

### E.1 Profiling plan (measure before changing anything)

Instrument these ranges with a section timer, aggregate mean/p95, log once per second, and remove or
permanently gate the instrumentation before the final benchmark run (CLAUDE.md §4, §28).

| Section | Range in `PKG/src/integrated_mpc_node.cpp` |
|---|---|
| S0 whole callback | all of `onTimer` (`:371-2121`) |
| S1 tracking state | `:412-430` |
| S2 reference horizon | `:437-441` |
| S3 prediction model (incl. 50 × expm) | `:449-452` |
| S4 constraints | `:459-516` |
| S5 cost | `:523-572` |
| S6 debug validation (symmetry, eigensolver) | `:579-637` |
| S7 solver construct + solve | `:656-701` |
| S8 debug violation scan / objective / preview | `:763-1104` |
| S9 debug reference feasibility | `:1432-1493` |
| S10 publish | `:1752-2118` |

Measurement order: (a) rebuild with optimisation and re-measure with **no** code change, to isolate
the build-flag contribution; (b) then gate the debug sections; (c) then consider a persistent
solver.

---

## F. Scientific-fairness review

Verified points of alignment between the two controllers — these are already fair and must not be
disturbed (`mpc.param.yaml`, `pid.param.yaml`, `PKG/config/integrated_mpc.param.yaml`,
`PKG/src/integrated_mpc_node.cpp:78-201`):

| Quantity | Stock | Custom |
|---|---|---|
| Prediction horizon | `mpc_prediction_horizon: 50` | 50 |
| Prediction step | `mpc_prediction_dt: 0.1` | 0.1 |
| Minimum prediction length | `mpc_min_prediction_length: 5.0` | 5.0 |
| Control period | `ctrl_period: 0.03` (`trajectory_follower_node.param.yaml`) | 0.03 |
| Steering time constant | `vehicle_model_steer_tau: 0.27` | 0.27 |
| Lateral-error weight | `mpc_weight_lat_error: 1.0` | 1.0 |
| Heading × v² weight | `mpc_weight_heading_error_squared_vel: 0.3` | 0.3 |
| Terminal lateral / heading | `1.0` / `0.1` | `1.0` / `0.1` |
| Steering-rate weight | `mpc_weight_steer_rate: 0.0` | absent (equivalent to 0) |
| Steer-rate limit maps | `[40,50,60]` by curvature `[0.001,0.002,0.01]`; `[60,50,40]` by velocity `[10,15,20]` | identical defaults |
| Accel / jerk limits | `max_acc 3.0`, `min_acc -5.0`, `max_jerk 2.0`, `min_jerk -5.0` | identical |

Verified asymmetries that currently **disadvantage the custom controller** and are confounds for the
stated research question (CLAUDE.md §4):

1. **Actuator delay compensation.** Stock compensates 0.24 s steering and 0.1 s longitudinal delay;
   the custom controller compensates neither (§D items 21–22). The simulator plant contains both.
2. **Build optimisation.** Stock `-O3 -DNDEBUG`, custom unoptimised (§C.3). Any timing comparison
   in this state is invalid.
3. **Curvature smoothing.** Stock smooths over a 15-point stride; the custom controller does not
   (§D item 20).

Correcting these removes confounds rather than favouring the custom controller, so it is consistent
with CLAUDE.md §34. Verified asymmetry that has **no** current effect: stock uses reduced
low-curvature weights (`mpc_low_curvature_weight_lat_error: 0.1`, `mpc.param.yaml:29-35`) which the
custom controller does not implement; this is a declarable design difference, not a confound.

**Benchmark measurement gap.** `BM/monitor_ws/src/control_benchmark_monitor/control_benchmark_monitor/logger_node.py:176-186`
subscribes to
`/control/trajectory_follower/controller_node_exe/lateral/debug/processing_time_ms` and
`.../longitudinal/debug/processing_time_ms`; `BM/scripts/analyze_run.py:195-207` aggregates them
into `lateral_processing_time_ms` and `longitudinal_processing_time_ms`. The stock baseline already
has these values — `BM/results/baseline_stock/S01/run_01/summary.json` records lateral mean
0.839 ms and longitudinal mean 0.034 ms. The custom node publishes neither topic, so the timing
half of the comparison would be empty.

---

## G. Prioritised implementation plan

Priorities per CLAUDE.md §40.E. No file listed here has been modified.

### P0-A — Compile the package optimised

- **Evidence:** §C.3.
- **File:** `PKG/CMakeLists.txt` — add, after `project(...)`:
  ```cmake
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
  endif()
  ```
- **Expected behaviour change:** `CXX_FLAGS` becomes `-O3 -DNDEBUG`, matching stock. Callback cost
  falls by an amount to be measured. Makes any timing comparison legitimate.
- **Risk:** low. `-DNDEBUG` disables Eigen assertions, so all sanity checks must be re-run to
  confirm numerical results are unchanged.
- **Validation:**
  ```bash
  rm -rf ~/integrated_mpc_ws/build ~/integrated_mpc_ws/install
  source /opt/ros/humble/setup.bash
  source ~/autoware-new/Autoware/autoware/install/setup.bash
  cd ~/integrated_mpc_ws && colcon build --symlink-install \
    --packages-select autoware_integrated_mpc_trajectory_follower --event-handlers console_direct+
  grep CXX_FLAGS build/autoware_integrated_mpc_trajectory_follower/CMakeFiles/integrated_mpc_node.dir/flags.make
  source ~/integrated_mpc_ws/install/setup.bash
  for t in model tracking_state prediction_reference prediction_matrix mpc_cost mpc_constraint osqp; do
    ros2 run autoware_integrated_mpc_trajectory_follower ${t}_sanity_check || echo "FAILED: $t"
  done
  ```

### P0-B — Satisfy the `controller_node_exe: control_state` contract

- **Evidence:** §C.1, all nine links.
- **Files:**
  - `PKG/launch/integrated_mpc.launch.xml` — `name="integrated_mpc_node"` → `name="controller_node_exe"`,
    namespace unchanged. This also aligns the benchmark timing-topic prefix in §F.
  - `PKG/src/integrated_mpc_node.cpp:37` — `Node("integrated_mpc_node")` → `Node("controller_node_exe")`;
    add a `diagnostic_updater::Updater` member; `setHardwareID(...)`;
    `add("control_state", ...)`; call `force_update()` at the end of `onTimer`, mirroring
    `AW/.../autoware_trajectory_follower_node/src/controller_node.cpp:274`.
  - `PKG/package.xml`, `PKG/CMakeLists.txt` — add the `diagnostic_updater` dependency.
- **Diagnostic semantics** (must reflect real validity — CLAUDE.md §23):
  - `OK` — all five inputs present, tracking state resolved, and `solution_valid_for_control` true
    (`PKG/src/integrated_mpc_node.cpp:1191-1199`).
  - `ERROR` — QP rejected and the fallback command is being published (`:1797-1866`); this is the
    analogue of stock `ControlState::EMERGENCY` at
    `AW/.../pid_longitudinal_controller.cpp:1233-1236`.
  - `WARN` — inputs not yet available (`:377-401`).
  - Add `stat.add<int32_t>("control_state", ...)` for parity with `.../pid_longitudinal_controller.cpp:1239`.
  - Never unconditionally `OK`.
- **Expected behaviour change:** the `control_state` leaf leaves STALE, `/autoware/control` becomes
  OK, `/autoware/modes/autonomous` becomes OK, `/system/operation_mode/availability.autonomous`
  becomes true, `change_to_autonomous` is accepted, the gate engages, and
  `/control/command/control_cmd` carries the MPC command instead of `(0, −1.5)`.
- **Risk:** low. Every topic in `PKG/launch/integrated_mpc.launch.xml` is an absolute remap, so
  renaming the node does not move any topic.
- **Validation:**
  ```bash
  ros2 topic echo /diagnostics --once            # expect name "controller_node_exe: control_state"
  ros2 topic echo /system/operation_mode/availability --once   # expect autonomous: true
  ros2 node list | grep trajectory_follower       # exactly one controller node
  ros2 topic info /control/trajectory_follower/control_cmd     # publisher count must be 1
  ros2 topic echo /control/command/control_cmd --once          # must not be (0, -1.5)
  ```
  This run also resolves §C.4 items 2 and 3.

### P1-C — Correct longitudinal command semantics

- **Evidence:** §D item 23; `PKG/src/integrated_mpc_node.cpp:1984-1994`, `:2073-2074`.
  Plant is `DELAY_STEER_ACC_GEARED`, so acceleration is the actuating signal (§A.2); stock sets
  `velocity` and `acceleration` and leaves `jerk` unset
  (`AW/.../pid_longitudinal_controller.cpp:914-921`).
- **File:** `PKG/src/integrated_mpc_node.cpp`, section 23.7.
- **Proposed fix:** derive the command from the measured state each cycle —
  `a_cmd = clamp(x0(4) + j0 · ctrl_period, min_acceleration_, max_acceleration_)`.
  This is receding-horizon consistent with the MPC's own prediction, is bounded by construction,
  and cannot wind up while actuation is blocked. Keep `velocity = reference_velocity` (equivalent to
  stock) and keep `jerk` as informational.
- **Risk:** medium — this is the longitudinal control law; it changes closed-loop behaviour.
- **Validation:** S1 closed loop; plot commanded vs. measured acceleration and confirm the command
  tracks the measurement rather than saturating at `+3.0`.

### P1-D — Publish the MPC predicted trajectory

- **Evidence:** publisher created at `PKG/src/integrated_mpc_node.cpp:313` with no `publish()` call
  in the file. Consumers wired in `control.launch.xml`:
  `autoware_lane_departure_checker` (`.../lane_departure_checker_node.cpp:186-190`),
  `autoware_control_validator`, and
  `autoware_autonomous_emergency_braking` (`.../src/node.cpp:553,584` null-guards).
- **File:** `PKG/src/integrated_mpc_node.cpp` — build the message from `optimal_state_prediction`
  (already computed at `:962-969`) together with `prediction_reference.points[k]`, reconstructing
  global poses from `(e_y, e_psi)` in the Frenet frame. Derive it from the real MPC solution; no
  placeholder (CLAUDE.md §27).
- **Expected behaviour change:** lane-departure checking and AEB path checking actually run
  (see §C.6).
- **Risk:** low functionally, but an incorrect reconstruction can make the lane-departure diagnostic
  legitimately ERROR and block autonomy. Land it after P0-B is confirmed green so the two effects
  are not confounded.
- **Validation:**
  `ros2 topic hz /control/trajectory_follower/lateral/predicted_trajectory`; overlay in RViz against
  the driven path; confirm `/autoware/control` stays OK.

### P1-E — Publish controller and solver timing

- **Evidence:** §F, benchmark measurement gap.
- **File:** `PKG/src/integrated_mpc_node.cpp` — publish `Float64Stamped`.
- **Open decision, not resolved in this audit.** The integrated MPC has no lateral/longitudinal
  split, so mapping solver time and callback time onto the two stock-named topics would mislabel
  them, which CLAUDE.md §34 forbids. The two admissible options are:
  1. Publish new `~/debug/callback_time_ms` and `~/debug/solver_time_ms` topics and extend
     `BM/monitor_ws/.../logger_node.py` and `BM/scripts/analyze_run.py`. This writes outside `PKG`
     and therefore needs explicit approval under CLAUDE.md §33.
  2. Publish the integrated total on the `lateral` topic and `0.0` on the `longitudinal` topic, then
     compare against **stock lateral + longitudinal summed**, disclosing the mapping. No benchmark
     edit; solver time still needs its own channel.
- **Validation:** run S1 and confirm `timings.csv` is non-empty and `summary.json` gains timing
  statistics.

### P1-F — Align the parameter file with the declared parameters

- **Evidence:** `PKG/config/integrated_mpc.param.yaml` sets `max_steer`, `max_acc`, `min_acc`,
  `steering_input_delay`, `longitudinal_delay`. `PKG/src/integrated_mpc_node.cpp:158-181` declares
  `max_steering_angle`, `max_acceleration`, `min_acceleration`; the two delay keys are declared
  nowhere in the file. The yaml also omits `nearest_dist_threshold`, `nearest_yaw_threshold`
  (`:63-71`) and all four steer-rate map parameters (`:183-201`).
  `max_jerk` and `min_jerk` do match (`:173-181`).
  The mismatched values coincide with the code defaults, so present behaviour is unchanged — but the
  configuration file does not in fact control the controller, which is a reproducibility hazard for
  a thesis artefact.
- **File:** `PKG/config/integrated_mpc.param.yaml`.
- **Risk:** low, but it *will* change behaviour if any value is corrected to something other than
  the current default; diff carefully.
- **Validation:** `ros2 param dump /control/trajectory_follower/controller_node_exe` and diff
  against the yaml.

### P1-G — Actuator-delay compensation

- **Evidence:** §D items 21–22; §F asymmetry 1.
- **Files:** `PKG/src/integrated_mpc_node.cpp`, `PKG/src/tracking_state_builder.cpp`.
- **Smallest correct approach:** propagate `x0` through the known input delay using the last
  `ceil(delay / ctrl_period)` published commands before forming the MPC initial state, rather than
  extending the state vector.
- **Risk:** medium — changes lateral behaviour. Land it only after a clean S1 baseline exists, so
  the before/after effect is measurable and reportable.
- **Validation:** S1 and S2; compare lateral RMSE and steering smoothness before and after.

### P1-H — Curvature smoothing

- **Evidence:** §D item 20.
- **File:** `PKG/src/tracking_state_builder.cpp:120-201` — add a configurable index stride, matching
  the stock concept of `curvature_smoothing_num_traj`.
- **Risk:** low-medium.
- **Validation:** `BM/scripts/verify_reference_steer.py`; plot `kappa_ref` and `delta_ref` before
  and after.

### P2 — After first valid results

- Gate the debug-only hot-path computations (§E items 2–6) behind a `debug_diagnostics` parameter,
  default off for benchmark runs (CLAUDE.md §4, §28).
- Persistent OSQP solver with warm start (replacing `PKG/src/integrated_mpc_node.cpp:678-680`).
- Incremental nearest-index search seeded from the previous cycle.
- Velocity lower bound in the constraint set (§D item 24).
- Replace the `/**:` wildcard in `PKG/config/integrated_mpc.param.yaml` with the explicit node name.

---

## H. Recommended commit sequence

1. `build: compile with Release optimisation by default` — `PKG/CMakeLists.txt` only.
2. `test: record sanity-check results under the optimised build` — no code change.
3. `feat: expose controller_node_exe control_state diagnostic` — node rename, `diagnostic_updater`,
   `PKG/package.xml`, `PKG/CMakeLists.txt`, `PKG/launch/integrated_mpc.launch.xml`.
4. `fix: derive acceleration command from measured state`.
5. `feat: publish MPC predicted trajectory from the optimal state prediction`.
6. `feat: publish controller and solver processing time`.
7. `fix: align parameter file keys with declared parameters`.
8. `perf: gate debug-only diagnostics behind a parameter` — only after §E.1 has produced data.
9. `feat: compensate steering and acceleration actuator delay` — separate commit, since it changes
   the control law and must be independently attributable in the thesis.

Per CLAUDE.md §42: inspect `git status` before each edit and `git diff` after each coherent fix.

---

## I. Explicitly do not change

- Anything under `AW` (CLAUDE.md §33). This audit found no reason to.
- Any diagnostic-graph definition —
  `AW/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/control.yaml`,
  `.../autoware-main.yaml` — and no lowering of `warn_rate`, `error_rate` or `timeout` in
  `.../component_state_monitor/topics.yaml`.
- `stop_hold_acceleration` or any other `vehicle_cmd_gate` parameter.
- The gate path: never publish directly to `/control/command/control_cmd` (CLAUDE.md §18).
- The first-step steering-rate constraint using `ctrl_period` while later steps use `prediction_dt`
  (`PKG/src/mpc_constraint_builder.cpp:388-391`) — validated, CLAUDE.md §9/§10.
- The `e_y` sign convention (`PKG/src/tracking_state_builder.cpp:251-253`) — validated.
- The exact affine ZOH discretisation (`PKG/src/integrated_kinematics_bicycle_model.cpp:271-305`) —
  validated; do not substitute Euler.
- The physical bounds (±0.7 rad, −5.0/+3.0 m/s², −5.0/+2.0 m/s³) — they already match stock exactly
  (§F).
- The hard clamp on the applied first input (`PKG/src/integrated_mpc_node.cpp:1896-1940`).
- The scenario files in `BM/scenarios`.
- The existing sanity checks — do not edit them to make anything pass (CLAUDE.md §34, §36).
- The recorded stock baseline in `BM/results/baseline_stock/`.

---

## J. End-to-end verification after P0-A and P0-B

```bash
source /opt/ros/humble/setup.bash
source ~/autoware-new/Autoware/autoware/install/setup.bash
cd ~/integrated_mpc_ws
colcon build --symlink-install \
  --packages-select autoware_integrated_mpc_trajectory_follower --event-handlers console_direct+
source ~/integrated_mpc_ws/install/setup.bash

# 1. all seven sanity checks pass

# 2. closed loop
cd ~/control_benchmark/scripts && ./run_scenario.sh 1

# 3. second shell
ros2 topic echo /diagnostics --once | grep -A2 control_state
ros2 topic echo /system/operation_mode/availability --once
ros2 topic info /control/trajectory_follower/control_cmd   # exactly 1 publisher
ros2 topic hz  /control/trajectory_follower/control_cmd
ros2 topic echo /control/command/control_cmd --once        # must not be (0, -1.5)
```

Pass criteria for the first closed-loop S1 (CLAUDE.md §43): the vehicle moves; no NaN or Inf is
published; steering, acceleration and jerk stay within the hard limits; the acceleration command
tracks the measured acceleration rather than saturating; callback timing is measured separately from
solver timing; and no diagnostic or safety mechanism was disabled to get there.

---

## K. Smallest first change

**P0-A** — add the Release build-type default to `PKG/CMakeLists.txt` and rebuild.

It is the smallest edit in this plan, carries no behavioural risk beyond re-running the sanity
checks, is a hard prerequisite for every timing number in the thesis (§C.3, §F), and provides the
measurement needed to resolve §C.4 item 1 before any profiling work is invested. **P0-B** follows
immediately, since it is the change that actually unblocks closed-loop operation.
