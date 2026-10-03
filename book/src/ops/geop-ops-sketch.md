# geop-ops-sketch

> Brief overview only — full documentation is coming later.

Sketches as an operation (see [geop-ops](./geop-ops.md#operations)).
`AddSketch` places a sketch, drawn with
[geop-core-sketch](../core/geop-core-sketch.md), on a planar face, a datum
plane or a plane of a coordinate system — like the part's `origin` — of the
part. The plane is resolved when the step
runs, so a sketch on a face stays where the face was, whatever later steps
do to it.

## Reference geometry

What a sketch is given rather than drawn is *reference geometry*: fixed
points and curves (see [geop-core-sketch](../core/geop-core-sketch.md#entities-and-constraints))
that the solver never moves and constraints are measured against.

- **The sketch's own origin and axes.** Every sketch has its origin as a
  point and its `x` and `y` axes as construction lines through it, so a
  point can be made coincident with the origin and a line parallel to an
  axis like with anything else drawn.
- **Projections.** A vertex, an edge or a face of the part — of a placed
  part too — projected into the plane along its normal. A projection
  refers to what it projects by name, never by position: every time the
  sketch is built it is brought up to date with the part, so a sketch
  dimensioned against a projected edge follows the edge when an earlier
  step moves it. The sketch's ids for what it projected are keyed by the
  names of the vertices and edges they come from, and stay the same.

Projecting reads what an edge *is* from its NURBS curve: a straight one is
a line, a circular one in a plane parallel to the sketch an arc or a circle
— to be constrained like one — one seen edge-on the line it collapses to
(an arc's extent found on its circle, not on its control polygon, which
reaches further), and anything else the spline it projects to exactly
(projection along the normal is affine: it moves the control points and
keeps weights and knots). An edge seen end-on is only a point.

Reference geometry is always construction geometry, drawn dotted: drawn
geometry snaps to it and is constrained against it, but it never bounds a
region itself.

## Dimensions as formulas

A dimension's value can be a formula of the part's parameters (see
[geop-ops](./geop-ops.md#parameters)) — `width / 2`, `screw.clearance` —
kept with the step and evaluated every time it is built, the sketch solved
anew when that, or what it projects, changed. Angles are in degrees.

## Editing a sketch

A new sketch has no plane and starts by picking one — a reference field like any
other, so picking another one later moves the drawing onto it — and once it
has one it is drawn on it, the viewer facing it head on. Selecting and
dragging are the editor's, as in every operation; drawing is `AddSketch`'s
own: it takes the clicks while a tool is in hand, the keys, and where its
points and curves are dragged to (see [geop-ops](./geop-ops.md#operations)).
Every tool is a button of its own, shown as an icon.

- **Drawing tools** — line, corner, center and 3-point rectangle, center
  and 3-point circle, 3-point, center-point and tangent arc, polygon, slot,
  spline, point — and fillet. Each builds its curves *and* the constraints
  that keep it what was drawn: a rectangle's sides square, a polygon's sides
  equal on a construction circle, a slot's ends tangent with their centers
  as points. The same construction previews the shape to the pointer and
  builds it on the click. Construction mode (or the curves selected) makes
  construction geometry.
- **Lines and arcs in one motion.** Drawing a chain of lines and moving the
  pointer back onto the chain's end switches the next segment to an arc
  tangent to the last one; after the arc the chain goes on with lines,
  the first of them tangent to the arc too — and horizontal or vertical as
  well if drawn so, the arc giving way to both.
- **Snapping is by constraint, never by moving geometry.** A point placed on
  an existing point — the origin among them — *is* that point; one placed
  where two lines, arcs or circles cross — the sketch's axes counted as
  infinite lines — is constrained onto both; one placed on a line's or
  arc's middle is its `Midpoint`; one placed on a curve is
  constrained onto it (`PointOnCurve`); a line drawn within 2° of horizontal
  or vertical gets that constraint. Dragging a point onto a point, a
  crossing or a middle snaps the same way, and constrains it there when let go. Where the
  pointer snaps is shown; holding shift turns snapping off.
- **Constraints, one tool each.** Coincident, horizontal, vertical,
  parallel, perpendicular, tangent, collinear, equal, concentric, midpoint,
  symmetric, fix, and the dimensions distance, horizontal and vertical
  distance, radius, diameter and angle. Each lists the kinds of operands it
  takes; with nothing selected every one can be taken up, and a selection
  greys out those it can never become. Pressed with a selection that is all
  it needs, a constraint is added at once, measured from the geometry as it
  is, so adding it moves nothing; otherwise the tool is taken up and picks
  what is clicked next until it has all it needs. A dimension is added by
  the click that says where its value goes — previewed at the pointer until
  then — and drawn there with its extension and dimension lines (an angle
  with an arc about where its lines meet). It then asks for its value in
  place; double-clicking the value asks again, and dragging it moves it out
  of the way. Where it goes is kept as an offset from what it measures, so
  it moves with the geometry. Constraints are listed, their values or
  formulas edited there too.
- **Dragging** a point, a line or a spline moves its points as far as the
  constraints let them (`Sketch::solve_with_drag`); dragging a circle sets
  its radius, an arc its sweep. Reference geometry is not dragged.

Undo and redo go edit by edit while a sketch is edited — a drag is one
edit — and back to the program's own once it is put away.

Every change is solved before it is returned, and one line of the dialog
says how that went: degrees of freedom left, or how many constraints
conflict, and how many closed regions the sketch has.
