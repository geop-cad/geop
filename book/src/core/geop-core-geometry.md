# geop-core-geometry

> Brief overview only — full documentation is coming later.

NURBS curves and surfaces, containment queries, and curve/surface
intersection, built on the interval-bounded scalars from
[geop-core-math](../core/geop-core-math.md).

## NURBS curves and surfaces

All geometry in the kernel is NURBS, and every coordinate is an interval
scalar. A curve is `NurbCurve<S, D>`, with control points in homogeneous
coordinates:

| Alias            | `D` | Control points  | Used for                              |
| ---------------- | --- | --------------- | ------------------------------------- |
| `NurbCurve3D<S>` | 4   | `(wx, wy, wz, w)` | 3-D edge curves                     |
| `NurbCurve2D<S>` | 3   | `(wu, wv, w)`   | pcurves in a surface's `(u, v)` space |

A surface is `NurbSurface3D<S>`, a tensor-product patch with its control
points stored row-major (`control_points[i * num_v + j]`).

`try_new` checks the knot vector length and rejects any weight that is not
*definitely* positive. Positive weights keep the denominator `W(t)` away from
zero on the whole domain, which is what makes the convex hull property hold
and what lets the containment and intersection searches work with
division-free equations. (A `derivative` curve is a hodograph whose last
coordinate is `W'`, not a weight, so it is only for evaluation and must never
enter a search.)

Operations on curves and surfaces include:

- `evaluate`, `tangent`, `second_derivative`, `derivative`; for surfaces
  `derivatives`, `normal` and `curvature_radius`;
- `split` / `split_mid` / `sub_curve` and `split_u` / `split_v` /
  `sub_surface`, implemented by knot insertion shared between curves and
  surfaces (`knot_insertion`);
- `reverse` and `swap_xy` for curves, `reverse_u` for surfaces (a mirror
  that keeps the domain, so pcurves only need the same mirror applied);
- `translate`, and `sweep`, which turns a curve and an offset vector into a
  ruled surface;
- `interpolate` (global interpolation after Piegl & Tiller) and
  `interpolate_enclosing`, which widens the fit until it *encloses* the curve
  the points were sampled from rather than only passing through them;
- `NurbSurface::project` (Newton foot-point projection) and `fit_pcurve`,
  which computes the `(u, v)` trace of a 3-D curve on a surface.

`everything()` builds a placeholder curve or surface whose every coordinate
is `ENTIRE`, for geometry that is not known yet. It `could_be_equal`s any
point.

## Containment

`contains::curve::curve_could_contain` and
`contains::surface::surface_could_contain` ask "could this curve/surface pass
through this point?" and return an enclosure of every parameter at which it
could (`Some(t)` or `Some((u, v))`), or `None` if it definitely does not.

Both use per-axis **fat line clipping** rather than plain bisection. For each
Cartesian axis `k`, the zeros of the scalar spline
`g_k(t) = X_k(t) − p_k · W(t)` are exactly the parameters where that
coordinate matches. A fat line around its control polygon bounds those zeros
to an interval, and intersecting the intervals of all axes shrinks the
segment directly towards the solution. A transversal hit converges in a
handful of clips instead of one bisection per bit. Restriction cuts at the
clip's outer bounds (`lower` / `upper`) so no solution can be lost, and
bisection uses the sharpened midpoint because the split point is a free
choice. The derivations are in `curve.md` and `surface.md` next to the code.

Every search takes two budgets, `max_nodes` and `min_subdivision_size`.
Running out of `max_nodes` is always an error, never read as "not
contained": the search is incomplete, so neither answer is justified.

## Intersection

- `curve_curve_intersect(a, b, max_solutions, max_nodes, min_subdivision_size)`
  returns paired `(s, t)` parameter boxes.
- `curve_surface_intersect` returns paired `(t, (u, v))` boxes against the
  untrimmed patch.

Both build a polynomial tensor-product spline whose zeros are the
intersections. For curves `A = H_A / W_A` and `B = H_B / W_B`,

```text
g_k(s, t) = H_A,k(s) · W_B(t) − W_A(s) · H_B,k(t)
```

with coefficients `P_i,k · Q_j,w − P_i,w · Q_j,k`: no division and no
degree elevation. Its zeros are clipped in every parameter direction with
the same fat line (`clip_tensor`). Boxes that overlap are merged by `union`
into one cluster, never averaged.

The result is an `Intersections<T>`:

- `Found(v)` — the complete set of isolated crossings;
- `Coincident(v)` — the two objects overlap along a stretch. The vector
  holds the overlaps' end points, the isolated crossings elsewhere, and
  points spread evenly over the overlaps.

Coincidence is found directly, not guessed from the number of solutions
(`intersection::coincidence`). An overlap of single-piece rational geometry
can only begin or end at a boundary: an end of one curve lying on the other
object, or the other object's boundary lying on the curve. So the search
collects those candidates, sorts them along the curve, and probes the
midpoint between each consecutive pair. The crossing search then only runs
on the stretches in between, where it may assume there is no overlap.

The searches isolate solutions, they do not polish them. `refine_crossing`
(and `refine_curve_curve_crossing`) run Newton on an isolated box. They are
infallible, since a singular Jacobian, an iterate leaving the domain, or a
result disjoint from the input all just return the incoming box, so
refinement can only tighten. Callers use it where a parameter becomes a split
point, not everywhere.

The older hull-and-bisect versions (`*_bisect.rs`) are kept only as
baselines for the benchmarks in `examples/`.

## Recognizing shapes

`shape` answers *what* an entity is rather than *where*: `as_line`,
`as_arc`, `as_plane` and `axis_of_revolution` recognize a straight line,
circular arc, plane or surface of revolution in a control net. A shape is
recognized only when the net *could* be exactly that shape under the
kernel's three-valued comparisons, never because it is merely close. The
resulting `Axis`, `Circle`, `Arc` and `Plane` carry the few constructions
that are built on them: projecting onto them and intersecting them. A
reference axis along an edge and a sketch placed on a face both use this.

> Originally I was using a lot of convex hulls and gjk algorithms to detect intersections and collisions, but this approach proved to be too slow. Fat line / surface clipping methods turned out to be much more efficient for the types of geometric computations I needed.
