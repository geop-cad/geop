# geop-core-math

> Brief overview only — full documentation is coming later.

Scalars, interval arithmetic, linear algebra, convex hulls, the kernel's
shared error type, and geometric primitives. Every other crate in the workspace
builds its numerics on top of this one, and it has no dependency on any
other Geop crate.

## Interval arithmetic

Geop uses exclusively interval arithmetic and never plain floating-point
arithmetic. Every value is an enclosure `[lo, hi]` that is guaranteed to
contain the true result, and every operation rounds its bounds *outward*, so
the uncertainty of a computation travels along with it. It starts with simple
math:

```rust
// The famous counterexample with floats
let x = 0.1 + 0.2;
assert!(x != 0.3); // f64 silently rounds, and the answer is simply wrong

use geop_core_math::scalars::{Ring, ScalInF64, Scalar};

let a = ScalInF64::from_f64(0.1);
let b = ScalInF64::from_f64(0.2);
let c = a.add(b); // [0.3, 0.3000000000000001]: contains the true sum

assert!(c.could_be_equal(ScalInF64::from_f64(0.3)));
assert!(!c.is_sharp());
```

From there it propagates through everything built on top: NURBS
evaluation, matrix operations, geometric transformations. No result in the
kernel ever claims more precision than the computation that produced it.

### The `Scalar` trait

All numeric code is generic over the `Scalar` trait (built on the `Ring` and
`Field` traits, whose `div` returns a `GeopResult` because a divisor interval
may contain zero). Two implementations exist:

| Type          | Representation                                      |
| ------------- | --------------------------------------------------- |
| `ScalInF64`   | `[lo, hi]` as `f64`, outward-rounded by one ULP per operation |
| `ScalInFPA64` | `[lo, hi]` as `i64` fixed point at 2⁻³² resolution, with directed-rounding division and a saturating overflow sentinel |

Tests run against both through the `for_all_scalars!` macro.

### Three-valued comparisons

Two intervals cannot always be ordered, so there is no `==` or `<` on
scalars. Instead every comparison comes in a *could* and a *definitely*
form:

- `could_be_equal` / `definitely_not_equal`
- `could_be_greater` / `definitely_greater`
- `could_be_less` / `definitely_less`

The code never compares with an epsilon. Whether two values are "the same"
is decided by whether their enclosures overlap, and the width of those
enclosures is exactly the uncertainty the computation accumulated.

### Combining enclosures

- `union` — the smallest interval containing both. Use it when two
  computations enclose the *same* value and both possibilities must be
  covered; the gap between them is kept as honest uncertainty. Never average.
- `intersect` — valid when two enclosures of the same value both hold at
  once; the result is tighter than either. `interpolate` relies on this,
  evaluating `a + α(b − a)` and `(1 − α)a + αb` and keeping their
  intersection, because each form is strong where the other is weak.
- `is_subset_of` — set containment, the existence/uniqueness test a
  Krawczyk contraction needs (`K(X) ⊆ X`).
- `lower`, `upper`, `width`, `is_sharp` — sharp bounds and width of an
  enclosure.

Constants include `ZERO`, `ONE`, `PI` and `E`, plus `ENTIRE` (the interval
`(-∞, ∞)`) as a placeholder for a value that is not known yet, since it
passes every overlap check.

### Sharpening

In some algorithms the uncertainty compounds. The `.sharpen()` method
collapses an interval to a single point, and it is only legitimate where the
value is a *free choice*, meaning any point inside the interval would serve
equally well:

- a subdivision parameter, since a search may split an interval anywhere;
- the seed of each Newton iteration, since each iterate only feeds the next.

The last iteration of such a loop is the *answer*, so it must not be
sharpened. Its width states how precisely the input determines the result,
and every comparison downstream depends on that statement being honest.

## Linear algebra

- `Vector<S, N>` (with the aliases `Vector2`, `Vector3`, `Vector4`): a
  fixed-size vector of scalars with the usual operations (dot and cross
  product, norm, normalization) plus the interval-aware `could_be_equal`
  and `union`.
- `Matrix<S, R, C>`: a dense, row-major fixed-size matrix with `transpose`,
  `mul_vec` and `mul_mat`.

## Solvers

In `solvers`: `linear`, `newton`, `least_squares` and `system`. Every
generic solver of the kernel lives here, written once. What stays outside
are loops particular to one geometric problem around these (a foot point
on a surface, a marching step, a rolling ball's center) and free-choice
searches in plain numbers (a bisection for a station), whose arithmetic
construction depends on to the last bit.

- `solvers::linear`: dense linear algebra in honest enclosures, one
  implementation per algorithm — `solve_linear_system` (a small fixed-size
  system, partial pivoting), `solve_pivoted` (several right-hand sides),
  `solve_banded` (a banded system that needs no pivoting, as curve
  interpolation's), `cholesky`/`solve_cholesky`, and Gauss-Jordan
  elimination with complete pivoting over a Jacobian (`eliminate`,
  `null_space`, `rank`, `inverse`). A pivot counts as zero exactly when it
  could be. Two of them are the same algorithm with different arithmetic
  (which pivot, divide or multiply by a reciprocal); their callers build
  curves, so they are kept apart rather than merged into one that would
  round differently for one of them.
- `solvers::newton::refine`: Newton refinement of a root a subdivision
  search has isolated — sharpened iterates but the last, the result
  intersected with the box the search proved, and the box itself where
  Newton stops.

- `solvers::least_squares::minimize`: constrained Levenberg–Marquardt over any
  `Scalar` — a sum of squares minimized where other residuals vanish
  exactly, with a symmetric rank-one estimate of the constraints'
  curvature. Steps are free choices and sharpened; one is taken only where
  the sum or the violation definitely drops, so no tolerance decides when
  it has converged. A norm is taken scaled by the largest entry: squared as
  they are, residuals of `1e-6` have a norm of `[0, 4.6e-5]` in fixed point,
  which no step can definitely lower.
- `dual::Dual<S, N>`: forward-mode automatic differentiation over any
  `Scalar`, carrying the gradient with respect to the `N` variables one
  residual depends on. Built over an interval scalar, a residual's value is
  an enclosure rather than a rounded float.

## Search helpers

- `DisjointSet`: gathers the many small candidates a subdivision search
  finds near each real solution and merges every pair that
  `could_be_equal` (transitively, with `union`), so that no two entries
  describe the same solution.
- `polygon_signed_area`: the shoelace formula for 2-D polygons.

## The error type

Every fallible operation returns `GeopResult<T> = Result<T, GeopError>`.
A `GeopError` is a chain: a `Root` holding the original message and a
captured backtrace, wrapped in any number of `Context` frames that callers
add on the way up the stack.

```rust
use geop_core_math::geop_error::{GeopError, GeopResult, WithContext};

fn split_edge(edge: usize, t: f64) -> GeopResult<()> {
    let ctx = |e: GeopError| e.with_context(format!("split_edge(edge={edge}, t={t:?})"));

    evaluate(t).with_context(&ctx)?;
    Ok(())
}
# fn evaluate(_t: f64) -> GeopResult<()> { Err(GeopError::new("parameter out of range")) }
```

`with_context` accepts a plain string, a `String`, or a closure. For a
one-off message, the `with_context!` macro formats lazily, so nothing is
formatted on the success path:

```rust,ignore
validate_pcurve_start_and_end(&new_surface, &c.pcurve, &sp, &ep)
    .with_context(with_context!("reassigning coedge {member} to new face"))?;
```

This context is how the kernel is debugged: rather than adding temporary
print statements, add a context frame to the real call path and leave it
there, so the next failure of the same kind reports its own arguments.
Print numeric state with `{:?}`. On an interval scalar, `{}` shows only the
midpoint to three decimals, which hides exactly the width you are trying to
diagnose.

A frame can also carry any `DebugContext` through `with_scene`. A
`PrimitiveScene` (from `geop-ops-rasterize`'s `debug` module) attached this
way is rendered to an HTML file under `/tmp`
when the error is printed, and the chain links to it, so a failure in a
geometric search comes with a 3-D picture of what the search was looking at.

## Primitives

The `primitives` module holds `CoordinateSystem`, `TriangleFace` (a mesh
triangle with the surface normals at its corners), `Ray`, `Datum` and
`Pose`.

`Pose` is where a rigid body sits: a rotation, a unit `Quaternion`, and a
position, generic over the scalar type. A unit quaternion has no singular
configurations, and moving a point by it is a polynomial of its
components, which keeps gradients through it well behaved. A file stores
the two as they are; angles about `x`, `y` and `z` (`euler_degrees`,
`from_euler`) are only for showing it to people. What it does to geometry
(`motion`) is computed in the geometry's own scalar type. A solver steps a
pose by a twist in its own frame, through the exponential map
(`Pose::exp`), so every step is taken from where the body is and no turn
is ever far from the variables' origin.

`Datum` is reference geometry: a right-handed orthonormal
`CoordinateSystem` and the `DatumKind` saying which part of it the datum
stands for — its origin (`Point`), the line along its `w` (`Axis`), the
plane normal to it (`Plane`), or the whole frame (`Frame`, a coordinate
system). Every datum carries the whole frame whatever its kind, so anything
built on it has axes to be built along. A frame's axes and the planes
between them can each be used on their own: `Datum::component` gives the
`DatumComponent` — an axis, or the plane normal to one — as a datum of its
own.

`Ray` is a half-line from a point along a unit direction, and how near it
passes to what it might hit: a point, a segment, a triangle, a plane, the
nearest point of a line — what picking in a viewer is built on.

## Constraint systems (`solvers::system`)

One layer for every constraint system of the kernel: a sketch's numbers
(`geop-core-sketch`) and a program's state, where its parts are
(`geop_ops_constraints::solve`), are each only an adapter.

- **Blocks** of variables (`Variable`): a number, a pose. A block moves by
  steps along its own tangent space (`retract`: adding to a number; a pose
  by the exponential map of a twist in its own frame), says what keeps it
  where it was (`stay`), and — for enclosing — every value a box of steps
  reaches (`retract_over`) and the differential of that map over the box
  (`differential`): none for a number, the right Jacobian of the
  exponential map for a pose (`Pose::exp_differential`).
- **Terms** (`Equations`): constraints, each with what it asks of its
  residuals (`Bound::Zero`, or `Bound::AtMost` for a limit), and
  preferences — a drag's pulls. Evaluated at the blocks' values, a term
  gives its residuals, each a length, and its Jacobian by the steps of
  each block it reads.
- **A system** (`System`): the equations, how each block may move
  (`Mobility::Held`, held where it was as far as the constraints allow, or
  `Mobility::Follows`, moved as the others need), and the system's size,
  which a residual holds to one part in a billion of.

On it, written once:

- `groups`: blocks no term ties together are solved, enclosed and
  measured apart — a plate with hundreds of screws is hundreds of systems
  of one screw. They are worked on in lockstep (`system::lockstep`): each
  group's work asks its terms of a batch, and one `Equations::evaluate`,
  every group at its own values, answers all that wait. A program run
  again per evaluation then runs once per step of the slowest group, not
  once per step of every group; a term that cannot be made with every
  group moved is asked again of its group alone, so one group's failure
  is never another's.
- `solve`: each group that a preference pulls or whose constraints do not
  hold, by the driver (`system::driver`): the constraints held exactly,
  the preferences and staying put choosing among where they hold; where
  they cannot all hold, a second minimization takes them into the sum,
  least squares. A limit is kept by an active set: solved without the
  limits first, every one the solve ends beyond is held at its bound, and
  the group is solved again. Its `Solution` carries the enclosure of the
  groups it solved, and how many times it made the terms, every group's
  at once (`evaluations`, as the enclosure does): the work, which a test
  bounds.
- `held`: which constraints hold, and the largest residual.
- `enclose`: a rigorous box around a true solution near where a solve
  ended, group by group — or why there is none: a singular configuration
  (`EncloseError::Unproven`), or constraints that conflict, by index, with
  those they depend on (`EncloseError::Conflict`). Each block is enclosed
  in its own step coordinates around the solved point, `x = retract(x̃, δ)`
  for a box of `δ`: Newton polishes the independent rows, and Krawczyk's
  test, `K(Δ) = −Y f(x̃) + (I − Y J(Δ)) Δ ⊆ Δ` with `‖I − Y J(Δ)‖∞ < 1`,
  proves the box holds exactly one solution of them; `J(Δ)` is the terms'
  Jacobian over every value the box reaches, composed with the
  retraction's differential. Which rows are independent is decided over a
  box around the point — the point, then the solver's tolerance, doubling —
  not at the point, so a redundant constraint (a rectangle's fourth right
  angle, a mate a joint implies) is recognised. Then, over the proven box,
  the rows left out must enclose zero and add nothing to the rank: a
  constraint the others imply leaves the rank as it is wherever they all
  hold, while one left out only because the configuration is singular — a
  point on a circle and on its tangent — is independent a hair off the
  singular point, where its value is too small to tell from zero but its
  derivative is not. A row left out that does not vanish is a conflict
  only where one is proven near where the solve ended: a combination of
  the rows, which vanishes wherever they all do, definitely not zero over
  the box of the selection (or the tolerance) in every variable, the free
  ones too (`EncloseError::Conflict::within`). Interval arithmetic can
  prove a conflict, never its absence: the box encloses a true solution of
  the independent rows, the others consistent with it to every digit it
  resolves, at first order too. `enclose_rest` encloses the groups an
  enclosure does not cover — what a sketch's report needs after a solve.
- `freedom`: how many independent ways each block can still move, decided
  over the proven box (a limit leaves the motion away from it free) — at
  the point where none is proven.
- `conflicting`: a smallest set of constraints that cannot all hold
  together — named by the enclosure where it proves the conflict, and by
  leaving each constraint out in turn where nothing is proven.

`Pose::exp` sums its coefficients near zero as series with the first term
left out as their bound, so the map, and its differential, are enclosures
for a box of twists around zero too. A pose a step sharpened is a hair off
unit; `retract_over` makes it unit honestly first, since turning a point by
a quaternion off unit scales it, and a box of such poses misses every
rigid motion.
