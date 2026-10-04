# geop-core-sketch

> Brief overview only — full documentation is coming later.

2-D constraint sketches: entities, constraints solved by the engine every
system shares ([geop-core-solve](./geop-core-solve.md)), and
profile extraction. Sketches are what get extruded or revolved into solids.

## Entities and constraints

A `Sketch` is plain `f64` design data, serializable with serde, so a part
program stores it verbatim and the web app sends it as is. Positions are
`point::P2` (`[x, y]`), with the arithmetic on them in the same module.

Points are the only entities with positions of their own. Curves refer to
points by `PointId`, so two curves sharing an endpoint share the *same*
point and stay connected by construction:

- `Line { start, end }`
- `Arc { start, end, sweep }` — counter-clockwise by `sweep` radians
- `Circle { center, radius }`
- `Spline { control_points, shape }` — a clamped, uniform B-spline of
  degree up to 3 as drawn; with a `SplineShape` (degree, knots, weights),
  any clamped NURBS — what an edge of the part projects to

A curve marked `construction` takes part in constraints but not in profiles,
e.g. a revolve axis or a symmetry line.

A point or a curve marked `fixed` is given rather than solved for: the
sketch's own origin and axes, or an edge of the part projected into its
plane. Its variables are held, never moved by a solve, and every constraint
on it is measured against it as it is; a fixed curve's points are all fixed,
and its own sweep or radius held too. Two fixed points made coincident are
not merged — neither can move — but checked: the coincidence holds or it is
reported failed.

```rust,ignore
let mut s = Sketch::new();
let p = [s.add_point(0.1, -0.1), s.add_point(2.2, 0.2),
         s.add_point(1.9, 1.3), s.add_point(-0.2, 0.8)];
let l: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
s.constrain(Constraint::Fix { point: p[0], x: 0.0, y: 0.0 });
s.constrain(Constraint::Horizontal { line: l[0] });
s.constrain(Constraint::Horizontal { line: l[2] });
s.constrain(Constraint::Vertical { line: l[1] });
s.constrain(Constraint::Vertical { line: l[3] });
s.constrain(Constraint::Length { curve: l[0], value: 2.0 });
s.constrain(Constraint::Distance { a: p[1], b: p[2], value: 1.0 });

let report = s.solve()?; // an exact 2 x 1 rectangle at the origin
assert!(report.converged && report.dof == 0);
```

The constraints are the usual CAD set:

- **Geometric:** `Coincident`, `PointOnCurve`, `Horizontal`, `Vertical`,
  `Parallel`, `Perpendicular`, `Collinear`, `Tangent`, `Equal`,
  `Concentric`, `Center` (a point is a circle's or arc's center),
  `Midpoint`, `Symmetric`.
- **Dimensional:** `Fix`, `Distance`, `DistanceX`, `DistanceY`,
  `PointLineDistance`, `Length`, `Radius`, `Diameter`, `Angle`.

## Solving

`Sketch::solve` moves every point (and every arc sweep and circle radius) so
that all constraints hold, changing the sketch as little as they allow.
`solve_with_drag` also pulls given points towards target positions, for
interactive dragging. The `SolveReport` states whether the solve converged,
the largest remaining residual, the remaining degrees of freedom, which
points and curves are still free to move, and which constraints failed.

- **Variables.** Each class of coincident points is a single `(x, y)` pair.
  `Coincident` is therefore not a residual at all; it removes two degrees of
  freedom by construction.
- **Residuals.** Every constraint contributes residuals that are zero exactly
  when it holds, all scaled to lengths so that no constraint kind dominates
  through its units. Each is a `Residual` of the sketch's variables, and
  the shared `System` of [geop-core-solve](./geop-core-solve.md) drives
  the sum of their squares to zero, pulls dragged points, reports degrees
  of freedom and encloses the exact solution.
- **Derivatives.** Residuals are written once, generic over `Scalar`, and
  evaluated with `Dual` numbers (forward-mode automatic differentiation) for
  exact gradients.

The solved positions the sketch stores are plain floats: the solver's best
approximation, what is drawn and what a program saves. The residuals are
computed on `Dual<ScalInF64>`, so the `sin`, `sqrt` and `PI` they need are
enclosed rather than rounded away.

## Enclosing the solution

What the kernel builds on is not those floats but `Sketch::enclose`: an
`Enclosure` of every point, arc sweep and circle radius that contains the
*exact* solution of the constraints. A float solution only meets
`Horizontal` or `Perpendicular` to the solver's tolerance, and built as
though exact it tilts faces by a hair where the designer meant square — a
real, tiny feature that a boolean later finds and meshes. Enclosed, the
geometry carries the uncertainty the solve really left, and the interval
tests downstream can see that two such faces may be one.

- **Free choices stay sharp.** The Jacobian at the solution says which
  variables the constraints determine and which they leave free. The free
  ones are the designer's choice, kept exactly as drawn.
- **Newton, then Krawczyk.** The determined ones are polished by a few
  Newton steps on the independent constraints, then enclosed by the
  Krawczyk test: for a box `X` around the polished point `x̃`,
  `K(X) = x̃ - Y f(x̃) + (I - Y J(X)) (X - x̃)`, evaluated in interval
  arithmetic, and `K(X) ⊆ X` proves `X` holds a solution. The box is
  widened until the test passes.
- **Honest failures.** A sketch whose constraints are not met is no
  solution, and is built as drawn. One that meets them only at a singular
  configuration cannot be proven, and fails to enclose.

`ProfileLoop::to_nurbs` builds the NURBS pieces from an `Enclosure`;
questions about the design data itself — nesting, winding, which side of an
axis — use `Enclosure::as_drawn`.

An arc is stored by its sweep rather than its curvature, because only the
sweep stays smooth through a straight arc and a half circle, and only the
sweep tells a major arc from the minor arc with the same curvature
(`geometry`).

## Profiles

`Sketch::region` turns a solved sketch into its one `Region` — a sketch is
one area to sweep, and one of several separate areas is an error — and
`Sketch::shape` into that, or, if its curves enclose nothing, the one open
chain they form. `Sketch::regions` finds every `Region`: a counter-clockwise
outer `ProfileLoop` and its clockwise holes. `ProfileLoop::to_nurbs` turns a
loop into `ProfilePiece`s, 2-D NURBS curves that extrude and revolve consume,
each recording which sketch curve it came from so that the faces built from
it can be named after it. Arcs are split into pieces of at most a quarter
turn and circles into four quarters. An open chain (a revolve profile) goes
through the same conversion.

- **Connectivity is structural.** Curves are joined only when they share a
  point, never because their ends are close. Open chains hanging off a loop
  are ignored; three or more curves meeting at a point is an error, since
  the regions they bound would be ambiguous.
- **Nesting and orientation use the real curves.** Which loop lies inside
  which, and which way a loop winds, are decided by casting a ray and
  counting crossings with the kernel's `curve_curve_intersect`, the same way
  a face's trim is tested. Polylines are only used for drawing.
