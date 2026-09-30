# geop-ops-sketch

> Brief overview only — full documentation is coming later.

Sketches as an operation (see [geop-ops](./geop-ops.md#operations)).
`AddSketch` places a sketch, drawn with
[geop-core-sketch](../core/geop-core-sketch.md), on a planar face, a datum
plane or a plane of a coordinate system — like the part's `origin` — of the
part. The plane is resolved when the step
runs, so a sketch on a face stays where the face was, whatever later steps
do to it.

## Editing a sketch

A new sketch has no plane and starts by picking one — a reference field like any
other, so picking another one later moves the drawing onto it — and once it
has one it is drawn on it, the viewer facing it head on. Selecting and
dragging are the editor's, as in every operation; drawing is `AddSketch`'s
own: it takes the clicks while a tool is in hand, the keys, and where its
points and curves are dragged to (see [geop-ops](./geop-ops.md#operations)):

- **Tools** — line (chaining), rectangle, arc (start, end, a point it passes
  through), circle, spline, point — are state machines on the session: the
  points placed so far are the draft, previewed to the pointer.
- **Snapping is by constraint, never by moving geometry.** A point placed on
  an existing point *is* that point; one placed on a curve is constrained
  onto it (`PointOnCurve`); one placed on the sketch's origin is fixed
  there; a line drawn within 2° of horizontal or vertical gets that
  constraint. Proximity only suggests; the constraint is what the sketch
  then holds.
- **Selecting** points and curves offers the constraints that fit them, as
  actions, measured from the geometry as it is, so adding one moves nothing.
  Constraints are listed, their values edited in place (angles in degrees)
  and removed; each is marked in the sketch by a glyph that can be selected
  too. Delete removes whatever is selected.
- **Dragging** a point, a line or a spline moves its points as far as the
  constraints let them (`Sketch::solve_with_drag`); dragging a circle sets
  its radius, an arc its sweep.

Every change is solved before it is returned, and the dialog says how that
went: degrees of freedom left, or which constraints conflict.
