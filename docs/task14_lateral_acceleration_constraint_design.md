# Task 14 — shared lateral-acceleration constraint: derivation and design (not implemented)

Per the task's own instruction: implement only if mathematically sound and only as a soft
(slack) constraint; if that requires a major QP architecture rewrite, stop and document the
design instead. The linearization below **is** mathematically sound and fits the existing
framework as a hard constraint would. The **soft**-constraint requirement (mandatory, to avoid
QP infeasibility) requires extending the QP's decision-variable count, which touches enough
fixed-size assumptions elsewhere in this package that it is judged a structural change, not an
incremental one. Not implemented this session; documented here instead, per that instruction.

## 1. Linearization

Bicycle-model lateral acceleration:

```
a_y = v^2 * tan(delta) / L
```

State convention (CLAUDE.md #5, validated): `v = v_ref + e_v`, `delta` is a state (not an
error). Linearizing around the per-horizon-step reference `(v_ref_k, delta_ref_k)` — the same
operating point already used everywhere else in this LTV formulation (`IntegratedKinematics
BicycleModel`'s own Jacobian linearization point) — gives:

```
a_y ≈ a_y_ref_k + c1_k * e_v_k + c2_k * (delta_k - delta_ref_k)

  a_y_ref_k = v_ref_k^2 * tan(delta_ref_k) / L
  c1_k      = 2 * v_ref_k * tan(delta_ref_k) / L        (∂a_y/∂v at the reference)
  c2_k      = v_ref_k^2 * sec^2(delta_ref_k) / L         (∂a_y/∂delta at the reference)
```

`e_v_k` and `delta_k` are both entries of the state vector `X = Aex*x0 + Bex*U + Wex`
(indices `5k+2` and `5k+3` respectively, `NX=5`), so `a_y` at step `k` is affine in `U`:

```
a_y_k(U) = c1_k * X(5k+2) + c2_k * X(5k+3) + (a_y_ref_k - c2_k * delta_ref_k)
         = [c1_k * Bex_row(5k+2) + c2_k * Bex_row(5k+3)] * U
           + [c1_k * (Aex_row(5k+2)*x0 + Wex(5k+2))
              + c2_k * (Aex_row(5k+3)*x0 + Wex(5k+3))
              + a_y_ref_k - c2_k * delta_ref_k]              (constant offset, per step)
```

This is **structurally identical** to how `MpcConstraintBuilder` already builds the
acceleration-state constraint (audit item 14: `a_min - a_free <= B_a*U <= a_max - a_free`,
`mpc_constraint_builder.cpp:325-349`) — one existing row pattern, extended to a linear
combination of two state rows instead of one. Nothing about the *hard*-constraint form requires
a QP rewrite; it is one more row block in `A`/`lower`/`upper`, built the same way the existing
four constraint groups are.

## 2. Why the *soft* form (mandatory per the task) is not an incremental change

A hard version of this constraint can make the QP infeasible whenever the existing steering/
jerk/accel/steering-rate bounds and this new bound disagree at some horizon step (e.g. a tight
turn at a speed where the reference itself already implies `|a_y| > a_y_max`) — exactly why the
task requires slack, not a hard bound. Slack means the constraint becomes:

```
lower_k - s_k  <=  a_y_k(U)  <=  upper_k + s_k,      s_k >= 0
```

with `s_k` **new decision variables**, one per horizon step (`N=50` more scalars), penalized in
the cost by `rho * sum(s_k^2)` (or `rho * sum(s_k)` for an L1 penalty). This changes the QP from
`U ∈ R^100` to `[U; s] ∈ R^150`, which means:

- `H` (currently 100x100) becomes block-diagonal 150x150: existing `H` top-left, `2*rho*I`
  (or a diagonal for the L1 case handled via bounds instead) bottom-right, zero off-diagonal.
- `f` (currently length 100) gets 50 zero entries appended (quadratic slack penalty has no
  linear cross-term with `U` in this formulation).
- `A` (currently 200x100) needs a new column block of width 50 (existing constraint rows get
  zero columns there; the new lateral-accel rows get `±1` in their own slack column) — column
  count changes from 100 to 150 for **every** existing row, not just the new ones.
- Every place in `integrated_mpc_node.cpp` that assumes `U` has exactly `2*N` entries would need
  to change from an implicit "whole vector" assumption to an explicit `U.head(2*N)`/`.segment(...)`
  split: `first_steering_command = optimal_input(0)`, `first_jerk_command = optimal_input(1)`,
  the `9C` preview loop's `input_index = 2*k`, the predicted-state reconstruction
  (`optimal_state_prediction = Aex*x0 + Bex*optimal_input + Wex` — `Bex` is `250x100`, so this
  literally requires `optimal_input` to stay length-100, i.e. `Bex * optimal_input.head(2*N)`),
  and the constraint-violation diagnostic scan's row-group indexing (`steering_angle` / `jerk` /
  `acceleration` / `steering_rate` bands assume exactly 4 groups of `N` rows each — a 5th group
  plus a slack column changes that layout too).
- `mpc_cost_builder.cpp` and `mpc_constraint_builder.cpp` are validated components
  (CLAUDE.md #7/#36: do not rewrite without demonstrated evidence of a bug); the least invasive
  path is to leave them producing the current `100`-wide `H`/`A`, and have the node
  block-diagonal-augment the result before calling OSQP — but that still means every consumer of
  `optimal_input`/`optimal_state_prediction` downstream in `onTimer` must be re-audited for the
  `100 -> 150` width change, which is a broad, cross-cutting edit, not a contained one.

That combination — QP variable-count change, cost/constraint matrix reshaping, and a
correctness-sensitive re-audit of every fixed-width assumption in the hot path that this whole
batch has been keeping deliberately small and traceable — is judged a structural rewrite, per
the task's own stop condition, not an incremental addition.

## 3. Proposed implementation for a future batch (not done here)

1. Add `MpcConstraintBuilder` output fields for the new rows' `A`/`lower`/`upper` contribution
   *unaugmented* (still `100`-wide `A` columns), built exactly as derived in §1 — this part alone
   is safe to land and test independently of the slack decision.
2. In the node, **after** calling the existing builders, construct the augmented
   `H_aug (150x150)`, `f_aug (150)`, `A_aug (rows x 150)` by block concatenation (no change to
   the builders themselves).
3. Change every fixed-`2*N`-width read of `optimal_input`/`U` to `.head(2*N)` /
   `.segment(0, 2*N)`, and read the slack block separately as `optimal_input.tail(N)` for
   diagnostics (log slack usage — the task requires this explicitly).
4. Feature flag (disabled by default), e.g. `enable_lateral_accel_coupling`, with its own
   `a_y_max` bound and slack penalty `rho` as parameters.
5. Dedicated sanity test: verify the linearization in §1 against the exact nonlinear
   `a_y = v^2*tan(delta)/L` at several `(v_ref, delta_ref)` operating points (small, moderate,
   near the physical steering limit), confirm the row/offset construction matches a
   directly-computed expectation, and confirm slack is exactly `0` whenever the hard bound alone
   would already be feasible.
6. Land the QP-widening change and the constraint-row change as **separate commits**, so a
   regression can be attributed to one or the other.

## 4. What would have to be independently re-verified

- OSQP's Hessian PSD requirement still holds for the block-diagonal-augmented `H` (it does, given
  `rho > 0`, since block-diagonal PSD ⊕ PSD = PSD — but this needs to be added to the existing
  cost sanity check, not just asserted here).
- The steering/jerk/accel/steering-rate constraint-violation diagnostic row-group indexing
  (`integrated_mpc_node.cpp`, "Identify constraint group" comment) needs a 5th named group.
- `osqp_sanity_check.cpp`/`mpc_constraint_sanity_check.cpp` currently assume the `100`/`200`-wide
  shapes documented in CLAUDE.md #12; those fixed expected sizes would need updating, which is
  exactly the kind of test-editing CLAUDE.md #34/#36 says must be justified by a proven
  formulation change, not merely code that no longer matches the old (still-correct) V1 shape.
