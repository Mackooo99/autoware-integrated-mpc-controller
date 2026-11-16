# Integrated Lateral-Longitudinal MPC for Autoware

## 1. Role and objective

You are working as a senior autonomous-driving / ROS 2 / control-systems software engineer.

This repository implements a custom integrated lateral-longitudinal Model Predictive Controller for Autoware Universe on ROS 2 Humble.

The immediate objective is NOT to redesign the controller from scratch.

The immediate objective is to finish the Autoware integration, validate the implementation, obtain stable closed-loop benchmark results, and identify only demonstrated technical improvements required for a master's thesis and scientific paper.

The scientific comparison is:

1. stock Autoware lateral MPC + longitudinal PID/PI;
2. this custom integrated lateral-longitudinal MPC.

The comparison must remain fair.

Do not alter the stock baseline merely to improve the custom controller's result.

---

# 2. Environment

Operating system:

Ubuntu 22.04

ROS:

ROS 2 Humble

Autoware workspace:

~/autoware-new/Autoware/autoware

Integrated MPC workspace:

~/integrated_mpc_ws

This custom package:

~/integrated_mpc_ws/src/autoware_integrated_mpc_trajectory_follower

Benchmark infrastructure:

~/control_benchmark

Scenario runner:

~/control_benchmark/scripts/run_scenario.sh

Scenario Test Runner profile:

awf/universe/20250130

Vehicle:

sample_vehicle

Sensor kit:

sample_sensor_kit

Simulator used for the scientific benchmark:

Autoware Planning Simulator + Scenario Simulator

AWSIM is not part of the immediate controller benchmark.

---

# 3. Benchmark scenarios

The existing benchmark consists of:

S1 — straight constant-speed controller benchmark

S2 — single intersection turn

S3 — alternating left-right turns

S4 — roundabout traverse / continue through the roundabout

S5 — roundabout U-turn / half-turn

Do not modify the scenarios merely to make the custom controller perform better.

---

# 4. Scientific objective

The master's thesis investigates timing and performance of Autoware vehicle control.

The custom controller is intended to integrate lateral and longitudinal control into one MPC optimization problem and compare it against Autoware's separated baseline.

Primary metrics include:

- lateral tracking RMSE;
- heading-error RMSE;
- velocity tracking RMSE;
- controller execution time;
- solver execution time;
- P95/P99 timing;
- acceleration/jerk behavior;
- steering smoothness;
- stability/constraint behavior.

Runtime measurement must distinguish:

1. OSQP solver runtime;
2. complete controller callback runtime.

Debug-only computation must not contaminate the final runtime benchmark.

---

# 5. Current controller architecture

MPC state:

x = [e_y, e_psi, e_v, delta, a]^T

where:

e_y = lateral error

e_psi = heading error

e_v = v - v_ref

delta = steering tire angle

a = longitudinal acceleration

Control input:

u = [delta_cmd, j]^T

where:

delta_cmd = steering tire command

j = longitudinal jerk

Reference:

r = [v_ref, a_ref, kappa_ref]

Reference steering:

delta_ref = atan(L * kappa_ref)

Vehicle wheelbase:

L = 2.79 m

Steering actuator time constant:

tau_delta = 0.27 s

Controller nominal period:

ctrl_period = 0.03 s

MPC prediction sampling interval:

prediction_dt = 0.1 s

Prediction horizon:

N = 50

Input ordering:

U =
[
 delta_cmd_0, jerk_0,
 delta_cmd_1, jerk_1,
 ...
 delta_cmd_49, jerk_49
]^T

---

# 6. Continuous nonlinear model

The model is based on nonlinear Frenet-frame kinematics.

Let:

v = v_ref + e_v

D = 1 - kappa * e_y

Dynamics:

dot(e_y) =
v * sin(e_psi)

dot(e_psi) =
(v / L) * tan(delta)
-
(kappa * v * cos(e_psi)) / (1 - kappa * e_y)

dot(e_v) =
a - a_ref

dot(delta) =
(delta_cmd - delta) / tau_delta

dot(a) =
j

The model is locally linearized and converted to an affine discrete model using exact affine zero-order hold.

Do not replace this with Euler discretization without a demonstrated reason.

---

# 7. Prediction form

The extended model uses:

X = Aex * x0 + Bex * U + Wex

The implementation already contains builders for:

- state construction;
- reference horizon;
- discrete vehicle model;
- extended prediction matrices;
- MPC cost;
- MPC constraints.

These pieces have already passed dedicated sanity tests.

Do not rewrite them without first proving that an implementation is incorrect.

---

# 8. MPC cost

The implemented quadratic objective has the form:

J =
(X - X_ref)^T Qex (X - X_ref)
+
(U - U_ref)^T Rex (U - U_ref)

Current V1 design includes approximately:

q_y = 1

q_psi(v) = 0.3 * v^2

q_v = 1

q_a = 0.1

r_delta(v) = 1 + 0.25 * v^2

r_j = 0.1

Terminal additions approximately include:

lateral error = 1

heading error = 0.1

velocity error = 1

The steering-state weight is intentionally zero in V1.

Do not tune these weights before closed-loop integration is proven unless a demonstrated instability requires it.

---

# 9. Constraints

Hard steering-angle limit:

-0.7 <= delta_cmd <= +0.7 rad

Longitudinal acceleration state limit:

-5.0 <= a <= +3.0 m/s^2

Jerk limit:

-5.0 <= j <= +2.0 m/s^3

Steering-rate limits are interpolated from Autoware-style curvature and velocity maps.

IMPORTANT:

The first steering-rate constraint uses the real control period:

delta_prev - rate * ctrl_period
<= delta_cmd_0 <=
delta_prev + rate * ctrl_period

where:

ctrl_period = 0.03 s

Future MPC steering-rate constraints use:

-rate * prediction_dt
<= delta_cmd_i - delta_cmd_(i-1) <=
rate * prediction_dt

where:

prediction_dt = 0.1 s

This distinction has already been explicitly tested.

Do not revert it.

---

# 10. Validated steering-rate correction

An earlier implementation incorrectly used prediction_dt = 0.1 s for the first steering-rate constraint.

That produced approximately:

delta_cmd_0 = -0.069813 rad

for a 40 deg/s limit.

The implementation was corrected to use:

40 deg/s * 0.03 s
=
1.2 deg
=
0.020944 rad

S2 then produced approximately:

delta_cmd_0 = -0.020944 rad

which is the intended result.

A dedicated constraint sanity test now validates:

- dimensions;
- steering-angle bounds;
- jerk bounds;
- acceleration-state bounds;
- steering-rate interpolation;
- first rate constraint using ctrl_period;
- subsequent rate constraints using prediction_dt;
- feasible candidates;
- intentional violations.

That test passes.

Do not undo this behavior.

---

# 11. Existing package components

The package includes approximately:

include/autoware/integrated_mpc_trajectory_follower/

- integrated_kinematics_bicycle_model.hpp
- tracking_state_builder.hpp
- prediction_reference_builder.hpp
- prediction_matrix_builder.hpp
- integrated_prediction_builder.hpp
- mpc_cost_builder.hpp
- mpc_constraint_builder.hpp

and sources including:

- integrated_mpc_node.cpp
- integrated_kinematics_bicycle_model.cpp
- model_sanity_check.cpp
- tracking_state_builder.cpp
- tracking_state_sanity_check.cpp
- prediction_reference_builder.cpp
- prediction_reference_sanity_check.cpp
- prediction_matrix_builder.cpp
- prediction_matrix_sanity_check.cpp
- integrated_prediction_builder.cpp
- mpc_cost_builder.cpp
- mpc_cost_sanity_check.cpp
- mpc_constraint_builder.cpp
- mpc_constraint_sanity_check.cpp
- osqp_sanity_check.cpp

Inspect the actual repository instead of assuming this list is perfectly current.

---

# 12. Validated model behavior

Vehicle-model sanity tests have passed.

For dt = 0.1 s and tau_delta = 0.27 s, examples include:

Ad(3,3) approximately 0.69047855

Bd(3,0) approximately 0.30952145

Ad(4,4) = 1

Bd(4,1) = 0.1

Ad(2,4) = 0.1

Bd(2,1) approximately 0.005

The prediction matrix sanity test confirms direct rollout and extended-matrix rollout match.

---

# 13. Tracking-state convention

The tracking-state builder uses:

e_y =
-sin(yaw_ref) * dx
+
cos(yaw_ref) * dy

Positive e_y means left of the reference path.

Heading error:

e_psi =
wrap(yaw - yaw_ref)

Velocity error:

e_v =
v - v_ref

This sign convention is deliberate and has been validated.

Do not change it without mathematical evidence.

---

# 14. Prediction reference

The reference-horizon builder does not trust trajectory time_from_start.

It constructs relative prediction timing using geometric distance and reference velocity.

Typical live S1 result:

prediction_dt approximately 0.1 s

N = 50

horizon length approximately 20 m

This has passed offline and live checks.

---

# 15. OSQP status

The controller uses Autoware's OSQP interface.

The real N = 50 MPC QP has solved successfully.

Typical observed solver-only runtime:

approximately 1.3–1.5 ms

Typical result:

status = solved

solution finite = true

solution size = 100

Hessian symmetric

minimum eigenvalue positive in tested cases

No relevant S1 constraint violations were observed.

In S2, small future active jerk-bound residuals around 1e-4 occurred.

These are interpreted as solver numerical tolerance effects.

Do not alter the physical bound to hide these residuals.

The actually applied first input must still be hard-clamped.

---

# 16. Open-loop S1 status

S1 open-loop prediction is considered valid.

Typical first input:

delta_cmd approximately -0.008129 rad

jerk approximately +2.0 m/s^3

Typical terminal prediction:

e_y approximately -0.00005 m

e_psi approximately -0.00005 rad

e_v approximately +0.009 m/s

No meaningful hard-constraint violation was observed.

---

# 17. Open-loop S2 status

S2 open-loop prediction is considered valid after the steering-rate correction.

Typical first inputs:

k0:
delta_cmd approximately -0.020944 rad
jerk approximately +2.0

k1:
delta_cmd approximately -0.0908 rad

etc.

The first command is now limited using ctrl_period while future differences use prediction_dt.

Do not revisit this unless new evidence shows a bug.

---

# 18. Current ROS integration architecture

The custom package replaces the stock Autoware trajectory follower controller.

The custom node publishes:

/control/trajectory_follower/control_cmd

It declares a publisher for:

/control/trajectory_follower/lateral/predicted_trajectory

The stock controller is intentionally not launched by the custom Planning Simulator wrapper.

There must be exactly one publisher of:

/control/trajectory_follower/control_cmd

The downstream Autoware vehicle command gate must remain unchanged.

Do not bypass it.

---

# 19. Current closed-loop behavior

With:

publish_enabled = false

the controller computes valid open-loop MPC solutions and does not publish commands.

With:

publish_enabled = true

the custom controller publishes:

/control/trajectory_follower/control_cmd

but the vehicle has not yet successfully entered the intended closed-loop benchmark.

Observed custom commands included valid steering and jerk output.

However downstream:

/control/command/control_cmd

was observed sending a stop-type command approximately:

velocity = 0

acceleration = -1.5

rather than forwarding the custom command.

The custom MPC must not bypass vehicle_cmd_gate to work around this.

---

# 20. Operation mode findings

Observed:

/vehicle/status/control_mode

mode = 1

but:

/api/operation_mode/state

reported:

is_autoware_control_enabled = true

is_autonomous_mode_available = false

and:

/system/operation_mode/availability

reported:

autonomous = false

Therefore the vehicle-level control mode and Autoware operation-mode availability must not be conflated.

---

# 21. Planning trajectory investigation

/planning/trajectory is healthy.

Observed:

publisher count = 1

publisher =
/planning/planning_validator

frequency approximately =
10–11 Hz

Both:

/control/trajectory_follower/integrated_mpc_node

and:

/control/autoware_operation_mode_transition_manager

were confirmed to subscribe to the same:

/planning/trajectory

Therefore there is no demonstrated planning-trajectory remapping mismatch.

Planning trajectory validation diagnostics are OK.

Do not treat /planning/trajectory as the current primary blocker.

---

# 22. Diagnostic graph investigation

The diagnostic graph provided the most important current integration finding.

Healthy items include:

- trajectory-follower topic rate eventually OK;
- planning trajectory topic OK;
- planning trajectory validation OK;
- autonomous emergency braking OK;
- lane departure checker OK;
- vehicle_cmd_gate node alive OK.

The important persistent control error is:

/autoware/control/performance_monitoring/control_state = ERROR

whose diagnostic leaf is:

controller_node_exe: control_state = STALE

The stock controller_node_exe was intentionally removed and replaced by the custom integrated MPC node.

The diagnostic graph still expects the stock-compatible control-state health diagnostic.

This is currently the strongest demonstrated integration mismatch.

---

# 23. Stock diagnostic behavior to verify

Before modifying anything, inspect the exact LOCAL Autoware source.

Previous investigation indicates that the stock longitudinal controller registers a diagnostic_updater task named:

"control_state"

and reports health based on its controller state.

Do not rely only on this statement.

Locate and read the exact local implementation.

Determine:

- which node/component creates the updater;
- exact diagnostic task name;
- exact node name expected by the diagnostic graph;
- whether the diagnostic is tied to controller_node_exe;
- which launch files/remappings provide that identity;
- what constitutes OK/WARN/ERROR/STALE;
- whether a custom drop-in controller should reproduce this interface.

Do not blindly fake an unconditional OK diagnostic.

The custom health diagnostic must represent real controller validity.

---

# 24. Likely integration strategy to evaluate

A likely solution is to make this custom controller behave as a drop-in Autoware controller interface by:

- using a ROS node name compatible with controller_node_exe;
- implementing diagnostic_updater;
- exposing a task named control_state;
- reporting OK only when required inputs are available and the current MPC solution is valid;
- reporting an appropriate non-OK state otherwise.

This is only a hypothesis until verified against the local Autoware source.

Independently confirm it before implementation.

---

# 25. Other diagnostic findings

The graph also showed at different moments:

/autoware/control/topic_rate_check/control_command
WARN or ERROR

control_command_gate timeout diagnostics
STALE

These may be secondary effects of autonomous operation not being available.

Do not weaken topic-rate thresholds or disable diagnostics yet.

Fix demonstrated root causes in dependency order and rerun the diagnostic graph after every integration change.

---

# 26. Missing stock-specific diagnostics

Because the stock controller_node_exe has been replaced, inspect whether any other stock-specific outputs besides control_state are expected elsewhere.

Do not assume control_state is the only node-name-dependent interface.

Search diagnostic graph configuration and local control launch/config files.

Produce an inventory before implementing the final compatibility layer.

---

# 27. Predicted trajectory

The custom node currently declares:

/control/trajectory_follower/lateral/predicted_trajectory

but historically it may not have actually published a meaningful MPC predicted trajectory.

Earlier logs showed consumers waiting for it.

Later diagnostic dumps showed lane-departure and AEB as OK.

Therefore do not assume predicted_trajectory is currently the root blocker.

Nevertheless, before final closed-loop benchmarking verify whether a correct predicted trajectory should be published as part of the drop-in controller contract.

If implementation is needed, derive it from the actual MPC optimal state prediction rather than publishing a fake heartbeat.

---

# 28. Current controller-frequency problem

Observed custom trajectory-follower control command frequency:

approximately 6.4 Hz

Nominal control period:

0.03 s

Expected frequency:

approximately 33.3 Hz

This discrepancy is real and must be investigated.

However it should be treated separately from the diagnostic integration blocker.

Observed OSQP runtime is only approximately 1.4 ms, so the remaining callback time is likely elsewhere.

Potential expensive operations include:

- rebuilding large matrices;
- matrix exponential operations;
- Hessian eigenvalue calculation;
- extensive matrix norms;
- solver recreation;
- excessive logging;
- debug-only diagnostics.

Do not guess.

Profile the complete onTimer path by sections.

The final benchmark must run without expensive debug-only diagnostics contaminating execution-time measurements.

---

# 29. Current longitudinal publishing concern

An early closed-loop implementation integrated command acceleration as:

a_cmd_next =
previous_acceleration_command
+
jerk * ctrl_period

While downstream actuation was blocked, this caused the internal acceleration command to continue growing even though measured acceleration stayed near zero.

This is effectively undesirable command-side windup.

Before finalizing longitudinal publishing semantics:

- inspect autoware_control_msgs Control semantics;
- inspect the local stock longitudinal-controller output behavior;
- distinguish MPC prediction_dt from actual ctrl_period;
- use measured/receding-horizon state consistently;
- avoid accumulating an unexecuted command indefinitely.

Do not treat the early acceleration publishing implementation as final.

---

# 30. Current publish safety requirements

Before a control input is applied, verify:

- solver returned success;
- solution has expected size;
- solution contains no NaN or Inf;
- cost/constraint matrices are finite;
- bounds are valid;
- constraint violation is within justified numerical tolerance;
- the actually applied first steering and jerk inputs are explicitly hard-clamped.

A small OSQP residual must not be confused with permission to violate physical limits.

---

# 31. Failure behavior

Do not publish NaN or invalid optimizer outputs.

For Planning Simulator integration, determine a safe and Autoware-compatible fallback.

Do not invent complicated production fallback behavior unless required for the benchmark.

Do not bypass command gate or safety checks.

---

# 32. Source-of-truth policy

The LOCAL Autoware checkout is authoritative for this task:

~/autoware-new/Autoware/autoware

Do not assume GitHub main or online documentation exactly matches the local checkout.

Online documentation may be used only as secondary context.

When determining an Autoware interface or behavior:

1. inspect local source;
2. inspect local launch/config;
3. inspect runtime ROS graph;
4. then draw conclusions.

Never speculate about code that has not been opened.

---

# 33. Write-scope policy

The Autoware source tree:

~/autoware-new/Autoware/autoware

is READ-ONLY reference for this project unless the user explicitly approves otherwise.

Do NOT modify stock Autoware.

The benchmark directory:

~/control_benchmark

should also not be modified during controller integration unless a benchmark bug is independently demonstrated and explicitly approved.

Primary implementation changes belong inside:

~/integrated_mpc_ws/src/autoware_integrated_mpc_trajectory_follower

including its launch/config files.

---

# 34. Scientific integrity rules

Do not:

- alter stock Autoware to favor the custom controller;
- disable safety diagnostics just to make autonomous mode available;
- weaken physical limits to make the QP easier;
- modify scenarios to make custom results look better;
- hide solver constraint residuals;
- hard-code outputs for known test cases;
- change tests merely to make an incorrect implementation pass.

If a test expectation is wrong, prove why before modifying it.

---

# 35. Avoid overengineering

This thesis is under a short deadline.

Prefer the smallest architecturally correct implementation that produces scientifically valid results.

Do not:

- perform unrelated refactors;
- redesign working builders;
- create generalized frameworks for hypothetical future requirements;
- add unnecessary abstractions;
- rewrite code for style alone.

Prioritize demonstrated blockers.

---

# 36. Existing tests

After relevant code changes, run all existing sanity checks.

At minimum inspect and run the available equivalents of:

- model sanity check;
- tracking-state sanity check;
- prediction-reference sanity check;
- prediction-matrix sanity check;
- MPC-cost sanity check;
- MPC-constraint sanity check;
- OSQP sanity check.

Do not silently modify tests.

---

# 37. Build environment

Before building:

source /opt/ros/humble/setup.bash

source ~/autoware-new/Autoware/autoware/install/setup.bash

Build:

cd ~/integrated_mpc_ws

colcon build \
  --symlink-install \
  --packages-select autoware_integrated_mpc_trajectory_follower \
  --event-handlers console_direct+

Then:

source ~/integrated_mpc_ws/install/setup.bash

---

# 38. Scenario execution

Benchmark runner:

cd ~/control_benchmark/scripts

./run_scenario.sh 1

S1 is the first closed-loop validation scenario.

Do not proceed to S2–S5 until S1 integration is stable.

---

# 39. Desired engineering workflow

Work in dependency order.

Phase A — independent audit

1. Read this CLAUDE.md fully.
2. Inspect the entire custom package.
3. Inspect relevant local Autoware source.
4. Inspect launch/config and diagnostic graph definitions.
5. Independently verify every important diagnosis stated here.
6. Identify incorrect assumptions if any.
7. Produce a root-cause tree.
8. Produce a prioritized implementation plan.

Do not edit code during Phase A.

Phase B — diagnostic compatibility

1. Fix the proven stock-controller diagnostic/interface mismatch.
2. Build.
3. Run sanity tests.
4. Run S1 with publishing disabled if appropriate.
5. Inspect diagnostics.
6. Confirm whether controller_node_exe: control_state becomes healthy.
7. Rerun operation-mode availability.

Do not weaken diagnostic graph configuration.

Phase C — control pipeline

1. Confirm exactly one publisher of trajectory_follower/control_cmd.
2. Confirm vehicle command gate receives/forwards the command under valid autonomous mode.
3. Verify output-message semantics.
4. Fix command publishing only where demonstrated necessary.
5. Validate safe fallback behavior.

Phase D — S1 closed loop

1. Run S1.
2. Verify actual vehicle motion.
3. Verify tracking errors decrease appropriately.
4. Verify no NaN/Inf.
5. Verify hard limits.
6. Verify no command-side windup.
7. Inspect operation-mode and safety diagnostics.

Phase E — timing

1. Measure whole callback.
2. Profile major sections.
3. Remove or gate debug-only expensive computations.
4. Avoid rebuilding expensive persistent objects unnecessarily where justified.
5. Retest correctness.
6. Aim to restore execution frequency close to the configured ctrl_period.

Phase F — remaining scenarios

Run:

S2
S3
S4
S5

only after S1 is reliable.

---

# 40. Required analysis output before implementation

Before changing files, produce:

## A. Architecture map

Show the runtime path:

planning trajectory
-> custom integrated MPC
-> trajectory_follower/control_cmd
-> Autoware control/safety/diagnostic nodes
-> vehicle command gate
-> Planning Simulator

Identify exact node names and topics from the local source/runtime assumptions.

## B. Root-cause tree

Separate:

- proven root causes;
- likely secondary effects;
- unresolved questions;
- non-blocking improvements.

## C. Interface compatibility table

Compare stock controller vs custom controller for:

- ROS node name;
- subscribed topics;
- published topics;
- diagnostics;
- control_state;
- predicted trajectory;
- message semantics;
- launch remaps;
- expected rates.

## D. MPC implementation review

Audit without rewriting:

- state definition/signs;
- continuous model;
- Jacobians;
- affine discretization;
- prediction indexing;
- cost construction;
- reference construction;
- terminal cost;
- acceleration constraints;
- steering-rate constraints;
- OSQP objective convention;
- solver-status checks;
- numerical tolerance handling;
- command extraction.

Mark each item:

VALIDATED
LIKELY CORRECT
NEEDS REVIEW
BUG

## E. Prioritized task list

Use priorities:

P0 = blocks closed-loop operation

P1 = required for scientifically valid benchmark

P2 = useful improvement but not required before first results

For each task give:

- evidence;
- files involved;
- proposed fix;
- risk;
- validation command.

---

# 41. Agent/subagent policy

Use stronger reasoning for:

- architecture;
- MPC mathematics;
- Autoware contract analysis;
- root-cause diagnosis;
- safety semantics;
- final review.

Cheaper subagents may be used for independent mechanical work such as:

- searching the codebase;
- locating definitions;
- inventorying topics/config files;
- summarizing logs;
- running existing tests;
- checking compiler errors;
- comparing simple interfaces.

Do not delegate safety-critical design decisions, MPC mathematics changes, or final architectural approval to a weak model without review by the lead reasoning model.

Use subagents only when tasks are genuinely independent or can run in parallel.

For simple sequential edits, work directly.

---

# 42. State tracking

Maintain a project progress file if useful, for example:

CLAUDE_PROGRESS.md

It should contain only concise engineering state:

- current proven blockers;
- completed fixes;
- tests passed;
- current runtime observations;
- next task.

Do not put large logs into it.

Git history is the primary implementation checkpoint.

Before any substantial edit:

inspect git status.

After a coherent validated fix:

inspect git diff.

Do not force-push or delete repository history.

---

# 43. Definition of done for the immediate integration

The immediate controller integration is complete when:

1. the package builds cleanly;
2. existing sanity tests pass;
3. stock Autoware source remains unmodified;
4. exactly one controller publishes trajectory_follower/control_cmd;
5. required control diagnostics are healthy;
6. autonomous operation mode becomes available when all genuine conditions are satisfied;
7. vehicle command gate forwards the custom controller command;
8. S1 runs closed-loop;
9. no NaN/Inf is published;
10. steering, acceleration and jerk remain within hard limits;
11. invalid QP results have safe behavior;
12. control-command semantics are consistent with local Autoware;
13. command-side acceleration does not wind up while downstream actuation is blocked;
14. callback timing has been measured separately from solver timing;
15. no diagnostic or safety mechanism was disabled merely to make the test pass.

---

# 44. Final rule

Investigate before modifying.

Never claim that a behavior is defined by Autoware until you have opened the relevant local source or configuration.

Prefer evidence over assumptions.

Prefer the smallest correct change over broad refactoring.

The goal is not merely to make S1 move.

The goal is to produce a technically defensible integrated MPC implementation suitable for a master's thesis and scientific comparison.
