# codegen_fk — Math Optimization Plan

Optimizing the arithmetic emitted by `codegen_fk` (`src/bin/codegen_fk.rs`) for
`compute_fk` in `src/generated/*.rs`. All work happens in the generator's
code-emission logic — this is partial evaluation done once at generation
time, not a runtime library change. Every special case below must keep
today's fully-general code as a fallback, since a future URDF could break
any of the corpus-wide patterns this plan relies on.

## What's actually in the generated code today

Every joint line has the shape:

```rust
let link_X = link_Y * Isometry3::from_parts(T_fixed, R_fixed) * Isometry3::from_parts(T_joint, R_joint);
```

`T_fixed`/`R_fixed` come from the URDF `<origin>` (compile-time constants
baked in by `codegen_fk.rs:80-87`). `T_joint`/`R_joint` encode the joint's
own DOF — one of the two is always `identity()` depending on joint type
(`codegen_fk.rs:64-79`):

| joint type | `T_joint` | `R_joint` |
|---|---|---|
| fixed | identity | identity |
| revolute/continuous | identity | `from_axis_angle` |
| prismatic | `axis * cmd` | identity |

Grepping all 12 files currently in `src/generated/` (235 joint lines total):

- **110/235 (47%) are fixed joints** — the second factor is a full
  `Isometry3::from_parts(identity, identity)` multiply that does nothing.
  Wildly skewed by robot: **anymal_d is 81/96 (84%) fixed**, stretch4 is
  40/54 (74%), wuji_hand only 5/26 (19%), and every `simple_arm_*`/enlight_l
  arm is 0%.
- **125/235 (53%) have `R_fixed` exactly `Quaternion::new(1,0,0,0)`**
  (identity origin rotation).
- **61/235 (26%) have `T_fixed` exactly `(0,0,0)`**.
- **100% of rotation axes in the current corpus are exactly axis-aligned**
  (`±X`/`±Y`/`±Z`) — not one diagonal axis anywhere, even in anymal_d or the
  hand.

**Correction to the original premise**: axes are *already guaranteed
unit-length* — `codegen_fk.rs:63` uses `Unit::new_unchecked` because
`parser.rs` normalizes them before codegen ever sees them. What's *not*
guaranteed is **axis-alignment** (exactly `(0,0,±1)` etc.) — URDF permits
arbitrary diagonal axes even though nothing in this corpus uses one today.
That's the real fork every optimization below has to plan around, not
unit-vs-non-unit.

## Optimizations

**A. Strip the no-op second factor on fixed joints.**
When a joint has no `cmd_idx` at all, drop
`* Isometry3::from_parts(Translation3::identity(), UnitQuaternion::identity())`
entirely — emit `parent * Isometry3::from_parts(T_fixed, R_fixed)` (or less,
see E). Always safe, no axis assumptions. Biggest single win given 47% of
joints (up to 84% for anymal_d) are currently paying for a multiply-by-identity.

**B. Skip translation recombination for revolute/continuous joints.**
`T_joint` is *always* identity here by construction of the template — not an
assumption about the URDF, a guarantee from the codegen itself. So
`X * Isometry3::from_parts(identity, R_joint)` algebraically reduces to
`Isometry3::from_parts(X.translation, X.rotation * R_joint)`: no vector
rotate, no add. Fully general, zero caveats, applies to all 175 revolute
joints in the corpus (and any future one).

**C. Skip rotation recombination for prismatic joints**, symmetric to B:
`X * Isometry3::from_parts(T_joint, identity)` reduces to
`Isometry3::from_parts(X.translation + X.rotation * T_joint, X.rotation)`,
skipping the quaternion×quaternion multiply. Also fully general.

**D. Bypass axis-angle trig when the axis is exactly a signed basis vector.**
`from_axis_angle` computes `(cos θ/2, sin(θ/2)·ax, sin(θ/2)·ay, sin(θ/2)·az)`.
When two of `ax,ay,az` are compile-time `0.0`, those multiplies are dead —
but the Rust compiler can't fold `sin(θ)*0.0` for you without `-ffast-math`
(NaN propagation rules forbid it), so this only happens if codegen does it
explicitly. **This is the one case that genuinely needs a fallback**: detect
exact axis-alignment at generation time (compare the parsed `f64` before
formatting, not the printed string) and emit the direct 2-component
quaternion construction; otherwise fall back to today's `from_axis_angle`
call unchanged. This also lets the *following* quaternion multiply
(composing with the parent) exploit that `R_joint` has two zero components —
roughly half the multiply/add terms of a generic quaternion product.

**E. Strip identity/zero on the static origin transform.**
When `R_fixed` is exactly `Quaternion::new(1,0,0,0)` (53% of joints), the
first composition needs no quaternion multiply at all — result rotation is
just `parent.rotation`. When `T_fixed` is exactly `(0,0,0)` (26%), skip the
translate/add too. These combine with A/B: a joint that's fixed *and* has
identity `R_fixed` *and* zero `T_fixed` (e.g. `link_world → link_base_link`
in `enlight_l.rs:6`) collapses to literally copying the parent isometry —
zero arithmetic.

**F. Skipped: "nice angle" folding for `R_fixed` (90°/180° about a principal
axis).** Considered special-casing quaternions like
`(0.7071067811865476, 0.7071067811865475, 0.0, 0.0)` into sign-flip/swap
logic. Rejected — the literal dump shows many "almost 90°" values with
rounding noise from the URDF author's RPY (e.g. `0.7071068967259818` vs the
exact `0.7071067811865476`), so exact-match detection would silently miss
real cases, and epsilon-based detection risks quietly changing the emitted
transform vs. what the URDF specifies. Not worth the correctness risk for
the payoff.

**G. Build profile / inlining.**
No `[profile.release]` section exists in `Cargo.toml` today, so `lto`
defaults off and `codegen-units` defaults to 16 (opt-level 3 is already the
release default, so that part's free already). Add
`lto = true, codegen-units = 1` and mark the generated `compute_fk`
`#[inline]` so callers (e.g. an IK solver's inner loop, or the criterion
bench loop) can fuse it in. Minor, low-risk lever — nalgebra's own ops are
already `#[inline]` — the real gains are A–E, which cut actual operation
count rather than call overhead.

## Rollout order

A → B/C → E → D → G, each step re-running `scripts/codegen_all_urdfs.sh`,
`tests/fk_correctness.rs`, and `benches/fk_speed.rs` before moving on.

Worth adding one synthetic fixture URDF with a deliberately non-axis-aligned
joint axis and a non-identity/non-zero origin, so the general fallback paths
(D's else-branch, mainly) actually get exercised by the correctness test
instead of being dead code against this corpus.

## Estimated speedup

Rough flop accounting: a generic `Isometry3 * Isometry3` in nalgebra is
~28 flops for the quaternion multiply plus ~22-25 for rotating+adding the
translation ≈ 50-55 flops. Today's template does **two** of these per joint
line ≈ 100-110 flops/joint.

- **Fixed joints** (A, +E when applicable): drop from ~106 flops to ~53 (A
  alone) down to **0** (A+E, when origin is identity/zero too). Given
  anymal_d is 84% fixed joints and stretch4 74%, expect the largest wins
  there — plausibly **2-4x** on `compute_fk` for those two robots, since so
  much of the current cost is pure waste on no-op multiplies.
- **Revolute joints** (B+D+E): best case (identity `R_fixed`, axis-aligned)
  drops from ~106 flops to roughly 35-40; worst case (generic `R_fixed`,
  axis-aligned axis only) to ~65. That's roughly **1.5-2.5x** per joint for
  arm-like robots (enlight_l, `simple_arm_*`) that have zero fixed joints
  and so don't benefit from A at all.

Caveat: flop count isn't 1:1 with wall-clock time. This code is already
branch-free, allocation-free, SIMD-friendly straight-line arithmetic per
joint, and the link chain is inherently serial (each joint depends on the
previous), so latency/dependency-chain effects and the unavoidable
`sin`/`cos` calls per revolute joint will compress the realized speedup
below the raw flop-reduction ratio.

Honest range: **~2-4x for fixed-joint-heavy robots (anymal_d, stretch4),
~1.3-1.8x for revolute-heavy arms** — treat as a hypothesis to confirm, not
a number to plan around. `benches/fk_speed.rs` already benchmarks `galaw`
(dynamic) vs `galaw-generated` vs `k` side by side per robot — run it
before/after each rollout stage to get real numbers instead of trusting the
flop estimate.
