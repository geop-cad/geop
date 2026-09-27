# geop-core-math

> Brief overview only — full documentation is coming later.

Scalars, interval arithmetic, linear algebra, convex hulls, the kernel's
shared error type, and render primitives. Every other crate in the workspace
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
  `mul_vec`, `mul_mat` and `solve_linear_system`.
- `interval_newton`: a verified Gauss-Newton-Krawczyk contraction for
  over-determined systems `F: R² → Rᴹ`, such as curve/curve intersection.
  One step either proves that a box holds exactly one root (`verified`),
  proves that it holds none (`empty`), or tightens it without deciding.

## Search helpers

- `ConvexHull<S, N>`: a point set such as a NURBS control polygon, never
  built explicitly. `could_overlap` and the containment queries run GJK
  directly on the points. Because a NURBS curve or surface lies inside the
  convex hull of its control points, this is what lets subdivision searches
  throw away regions that cannot contain a solution.
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
`PrimitiveScene` attached this way is rendered to an HTML file under `/tmp`
when the error is printed, and the chain links to it, so a failure in a
geometric search comes with a 3-D picture of what the search was looking at.

## Render primitives

The `primitives` module holds the kernel's rendering vocabulary: `Line`,
`TriangleFace`/`TriangleFace2d`, `Color10`, `CoordinateSystem`, and
`PrimitiveScene`, which collects points, lines, triangles, labels and
sampled curves and surfaces (through the `RasterizableCurve` and
`RasterizableSurface` traits) and saves them as an interactive HTML file.
`PrimitiveSceneRecorder` records a sequence of scenes, for example one per
iteration of a search.
