# Support real-world joint types + fix a latent ordering bug in the URDF parser

## Context

`src/parser.rs`'s `load_urdf` currently assumes every joint is `revolute`: it hard-requires `<axis>` and `<limit>` on every joint and never reads the `type` attribute at all. Real robot URDFs (which the user is about to load) routinely use `fixed` joints (sensor/frame mounts) and `continuous` joints (wheels), and often `prismatic` (grippers, slides) — all of which will fail to parse today. Separately, while investigating how to make command-vector indexing scale to real (possibly branching) robots, I found that `k::Chain` (the crate's own dependency, already used in `tests/fk_correctness.rs` as an independent ground truth) orders its movable joints via **DFS pre-order** (confirmed by reading `k`'s `iterator.rs`/`chain.rs`), while galaw's parser resolves joint order via **BFS**. For today's non-branching chain fixtures the two orders coincide by accident; for any branching real robot they will diverge, silently misassigning commands to the wrong joint. This plan fixes both problems together, since the joint-type work already requires touching the same code path (`cmd_idx` assignment).

User confirmed scope: add support for **revolute, continuous, prismatic, and fixed** joint types (the four that appear in effectively all real robot URDFs).

## Design

### 1. `src/types.rs` — a plain type tag + a flat, branch-free motion representation

Revolute/continuous/prismatic/fixed are all specializations of one rigid-body motion (a rotation part and a translation part where only one is ever nonzero); `Joint` stores that motion as flat vectors instead of an enum-with-data, so `compute_fk`'s hot loop never has to match on joint type per iteration:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
}

pub struct Joint {
    pub name: String,
    pub parent: String,
    pub parent_link_idx: usize,
    pub child: String,
    pub child_link_idx: usize,
    pub transform: Isometry3<f64>,
    pub joint_type: JointType,       // metadata only — compute_fk doesn't match on this
    pub rot_axis: Vector3<f64>,      // unit axis for Revolute/Continuous, else Vector3::zeros()
    pub lin_axis: Vector3<f64>,      // unit axis for Prismatic, else Vector3::zeros()
    pub limits: Option<(f64, f64)>,  // Some for revolute/prismatic, None for continuous/fixed — used only by test/bench command sampling, not by compute_fk
    pub cmd_idx: Option<usize>,
}
```

- `joint_type` is a plain tag (no associated data) kept for parser error messages and introspection; `compute_fk` never reads it. `rot_axis`/`lin_axis` are the two motion generators — exactly one is nonzero per joint (both are zero for `Fixed`), derived once during parsing from the `type` attribute + `<axis>`.
- `cmd_idx: usize` → `cmd_idx: Option<usize>` — `Fixed` joints have 0 DOF and must not consume a slot in `joint_cmds` (matches how `k::Chain::dof()`/`movable_nodes` already exclude `Fixed`).
- Add `GalawModel::num_actuated_joints(&self) -> usize` (count of `cmd_idx.is_some()`), used to size `joint_cmds` and in `compute_fk`'s length check.
- `joint_name_to_idx` (built in parser.rs) is populated via `filter_map` over joints with `cmd_idx.is_some()`, so `get_joint_idx("some_fixed_joint")` naturally returns `None` — no change needed to the accessor itself, just what gets inserted.
- Trade-off: `Joint` carries two `Vector3<f64>` (48 bytes) instead of one `Option<Unit<Vector3<f64>>>` — more memory per joint, but for robot-scale joint counts (tens, maybe low hundreds) that's a good trade for removing branch-predictor pressure from `compute_fk`'s loop.

### 2. `src/parser.rs` — split `load_urdf`, fix ordering, support all 4 types

Split the current 170-line function into:
- **`parse_link(node) -> Result<Link, _>`**
- **`parse_joint(node) -> Result<Joint, _>`** — reads the `type` attribute and, based on it, populates `rot_axis`/`lin_axis`/`limits` directly: `revolute`/`continuous` → axis goes into `rot_axis` (`lin_axis` stays zero), `limits` is `Some` for revolute, `None` for continuous (regardless of whether a `<limit>` tag with only `effort`/`velocity` is present); `prismatic` → axis goes into `lin_axis` (`rot_axis` stays zero), `limits` required; `fixed` → both axes stay `Vector3::zeros()`, `limits` is `None`, no axis/limit element read at all. Any other `type` value (`floating`, `planar`, `spherical` — the remaining URDF spec types) is a clear error naming all three as unsupported. Also makes `<origin>` default to identity and `<axis>` default to `(1,0,0)` when omitted, per URDF spec — both are commonly omitted in real files, and requiring them today would fail on well-formed real URDFs.
- **`resolve_joint_order(links, joints) -> Result<Vec<Joint>, _>`** — root-finding (logic unchanged) + build an adjacency map `HashMap<usize, Vec<usize>>` from `parent_link_idx` → joint indices **in file-declaration order** (built once, O(E); confirmed this matches how `k` itself builds its tree — it iterates `robot.joints` in file order per parent) + a **recursive DFS pre-order** walk (not the current BFS) assigning `cmd_idx = Some(counter)` to non-`Fixed` joints as they're visited, `None` to `Fixed`. Recursion depth is bounded by robot depth (never realistically more than a few dozen), so no stack-depth concern. This replaces the current O(V·E) re-filter-per-pop traversal with O(V+E) and — critically — makes galaw's joint order match `k`'s, so `joint_cmds` built by iterating `model.joints` positionally (as `main.rs`/tests/benches already do) lines up correctly for branching robots, not just chains.
- `load_urdf` becomes a short orchestrator: parse XML → collect links/joints via the two parse functions → `resolve_joint_order` → build `link_name_to_idx`/`joint_name_to_idx` → construct `GalawModel`.

**Explicitly not doing** (flagging so it's a deliberate choice, not an oversight):
- No custom error enum — repo has zero precedent (everything is ad-hoc `Box<dyn std::error::Error>`); introducing one now is scope creep beyond what was asked.
- No submodule split (`parser/xml.rs` etc.) — the file stays single, ~250-300 lines with clearly separated private functions; that's not large enough yet to warrant module boundaries.
- `<mimic>` is not read/honored. Confirmed `k` itself still gives a mimicked joint a real `Rotational`/`Prismatic` type (it still consumes a DOF slot) — so an unhandled `<mimic>` joint parses and runs fine under this design, it's just controlled independently rather than slaved to its driving joint. `<safety_controller>`/`<calibration>` are metadata-only (confirmed `k` never reads them either) — safe to silently ignore.
- No cycle detection in `resolve_joint_order`. A malformed URDF where a link is the `child` of two different joints (not a valid tree) could in principle cause unbounded recursion — this is a pre-existing gap (the old BFS had the same non-termination risk), not something introduced by this change, and out of scope for "support real, well-formed robot URDFs."

### 3. `src/kinematics.rs` — branch-free hot loop

`compute_fk`'s length check uses `self.num_actuated_joints()` instead of `self.joints.len()`. Before the loop, scatter the caller's `joint_cmds` into a dense, per-joint array sized `self.joints.len()`, defaulting slots with `cmd_idx: None` (i.e. `Fixed` joints) to `0.0` — this is the only place `cmd_idx` is consulted, and it's a one-time O(n) pass, not per-rotation-math:

```rust
let mut cmd = vec![0.0; self.joints.len()];
for (i, joint) in self.joints.iter().enumerate() {
    if let Some(idx) = joint.cmd_idx {
        cmd[i] = joint_cmds[idx];
    }
}
```

Then the per-joint loop has no match/branch at all:

```rust
for (joint, &cmd) in self.joints.iter().zip(cmd.iter()) {
    let rotation = UnitQuaternion::from_scaled_axis(joint.rot_axis * cmd);
    let translation = Translation3::from(joint.lin_axis * cmd);
    let joint_local = joint.transform * Isometry3::from_parts(translation, rotation);
    links[joint.child_link_idx] = links[joint.parent_link_idx] * joint_local;
}
```

This works for all four joint types with the same two lines of math: for `Fixed` (`rot_axis = lin_axis = 0`), `rotation` collapses to identity and `translation` to zero regardless of `cmd`'s value — verified against nalgebra 0.30.1's own doc test, `UnitQuaternion::from_scaled_axis(Vector3::zeros()) == UnitQuaternion::identity()`. For `Revolute`/`Continuous` (`lin_axis = 0`), only `rotation` contributes. For `Prismatic` (`rot_axis = 0`), only `translation` contributes.

### 4. Update callers: `src/main.rs`, `tests/fk_correctness.rs`, `benches/fk_speed.rs`

All three currently do `vec![0.0; galaw_model.joints.len()]` and/or iterate `galaw_model.joints` using `j.limit_lower..j.limit_upper` to build random commands. Change to:
- Size vectors with `galaw_model.num_actuated_joints()`.
- Filter to `j.cmd_idx.is_some()` when building per-joint random values, using `j.limits` to get a `(lower, upper)` range — when `limits` is `None` (continuous joints), fall back to a fixed test range like `-PI..PI` since there's no file-declared bound to sample.

### 5. New fixtures + tests (kept as two separate, isolated fixtures so a failure points at one concern)

- **`assets/mixed_joint_types.urdf`** — a single non-branching chain (like today's fixtures) but including one `fixed` link (e.g. a sensor mount), one `continuous` joint, and one `prismatic` joint alongside revolute joints. Isolates joint-type parsing/kinematics-math correctness.
- **`assets/branching_robot.urdf`** — a root link splitting into two child chains, plain revolute joints only. Isolates the BFS→DFS ordering fix with no confounding joint-type variables.
- Add both to the existing `fk_correctness_tests!` macro in `tests/fk_correctness.rs` (one line each) — this gets full end-to-end validation against `k::Chain` for free, for both new concerns.
- New `parser.rs` unit tests: fixed joint parses with no axis/limit; continuous joint has axis but `limits: None`; unsupported joint type (`floating`) errors clearly and names the three unsupported types; `cmd_idx` is `None` for fixed joints and contiguous-from-0 in DFS order for actuated joints; omitted `<origin>`/`<axis>` default correctly.

## Verification

- `cargo test` — the two new fixture-driven tests exercise the full pipeline against `k::Chain` as ground truth (per-link transform comparison), which is the strongest signal that both the joint-type math and the DFS ordering fix are correct.
- `cargo test --lib` (parser unit tests) for the narrower parsing-only cases.
- `cargo bench` (optional, user's call) to confirm the adjacency-map rewrite doesn't regress `fk_speed` — should improve or hold steady given O(V+E) vs O(V·E).
- Per your standing preference, I'll make the edits and explain them; you run `cargo build`/`cargo test`/`cargo bench` yourself.
