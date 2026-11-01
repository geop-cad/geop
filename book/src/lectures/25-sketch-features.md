# Lecture 25: Profiles, offsets and 3-D sketches

<div class="goals">

**After this lecture you can**

- explain how a solved sketch becomes regions and exact NURBS pieces, and why connectivity is structural;
- derive how an offset chain is cut, closed and tied by constraints to its source;
- say how copies and patterns in a sketch stay copies, and how a pattern is read off its constraints;
- describe what is special about text, projections and dimensions given by formulas;
- state how a 3-D sketch differs from a planar one, and what it is built for.

</div>

Two lectures made a sketch's points land where they should. But nobody sweeps a list of points. An extrusion needs an outer loop and holes, as curves. A fillet in a sketch needs an arc that stays tangent. An offset of an outline needs to keep following that outline when a dimension changes, for years. This lecture is about what is built on top of the solver, and it has one running theme: **what a sketch is, is its constraints.** Wherever it could store a derived answer, it stores the relation instead.

## From a sketch to regions

`Sketch::shape` in `core/geop-core-sketch/src/profile.rs` answers what a sketch draws to sweep: its `Regions` or, if its curves enclose nothing, the one open `Chain` they form. A `Region` is a counter-clockwise outer `ProfileLoop` and its clockwise holes. A sketch may enclose several areas apart, such as the letters of a word, and an extrusion makes each one a lump of one solid.

Finding regions is a small lesson in the kernel's style.

**Connectivity is structural.** Curves are joined only when they share a `PointId`, never because their ends are close. That removes a tolerance, and it also removes a class of silent bugs: nothing can be "almost closed". Curves marked `construction` take no part. Open chains hanging off a loop are ignored. Three or more curves meeting at a point is refused, since the regions they would bound are ambiguous.

**Nesting asks the real curves.** Does loop `$i$` lie inside loop `$j$`? `Sketch::regions` takes a point of loop `$i$` and casts a ray to count crossings with loop `$j$`, using the kernel's `curve_curve_intersect`, the same way a face's trim is tested in [Lecture 19](./19-classification.md). The box around `$j$`'s control points rules most pairs out first. One point suffices, since *loops of one profile never cross*: a point on one is either inside another or outside it. Polylines are used for drawing and for no decision.

**Winding asks the integral.** Which way does a loop wind? By its signed area, integrated from its curves by `loop_signed_area` (`core/geop-core-geometry/src/nurb_curve/area.rs`), and never from a polygon of samples. AGENTS.md's rule *Do not decide from samples* is why; [Lecture 19](./19-classification.md) has more.

`ProfileLoop::to_nurbs` then turns a loop into `ProfilePiece`s: 2-D NURBS curves that extrude and revolve consume, each recording which sketch curve it came from so that the faces built from it can be named after it. Arcs are split into pieces of at most a quarter turn and circles into four quarters, which [Lecture 8](./08-nurbs.md) explains. It takes the `Enclosure` of [Lecture 24](./24-rank-and-proof.md), so that the control points carry the widths the solve left.

## Copies that stay copies

A mirror of half an outline should stay a mirror when you edit the half. So the kernel never copies coordinates and walks away. `core/geop-core-sketch/src/copies.rs` ties every copied point to its original by a constraint:

- `Constraint::Symmetric { a, b, line }` for a mirror: the midpoint of `$a,b$` lies on the line, and `$b-a$` is square to it. Two rows.
- `Constraint::Moved { a, b, by }` for a pattern: `$b$` is `$a$` moved by the motion carrying the construction curve `by`'s start `$s$` onto its end `$e$`, that is `$q\mapsto e+R(q-s)$`, with `$R$` the turn by the arc's sweep, or the identity for a line.
- `Constraint::EqualSweep` so that an arc whose ends are images of another's ends has the same sweep, and `Constraint::Equal` for a circle's radius.

Now look at how a pattern is stored. There is no pattern record, no count field. **A pattern is its construction curve `by` and the `Moved` constraints that name it.** Each copy is the previous one moved by `by`. How many copies there are, `Sketch::pattern_count`, is read off the chain of constraints, and `Sketch::set_pattern_count` adds or removes copies at its end. The spacing is a dimension of `by`: a `Length` of the line, or an `Angle` between the radii to the arc's ends. Editing it is editing a dimension, like any other.

There is nothing to keep in sync, because there is one source of truth. The module comment says it best: patterns are structure, not records.

Where the copies go *before* the solve is a free choice, computed in plain `f64` (`core/geop-core-sketch/src/plain.rs`): a position the solver then makes exact. It is never a result the kernel reasons about. The same split holds for snapping and for the tools.

## Offsetting a chain

Offsetting an outline by a wall thickness looks like a geometric computation. In Geop it is mostly a constraint construction, with a hard geometric fallback where the curve is free-form. `Sketch::offset` in `core/geop-core-sketch/src/offset.rs` follows these rules.

**One dimension.** The offset of the chain's first curve is a `Constraint::Offset`, and carries the distance as a dimension. Every other offset curve is only `Parallel` to its line or `Concentric` with its arc. The distance carries over at each corner. So one number says how far all of it is. Change that dimension, and the whole thing moves.

Before we look at corners, see the geometry in a simple case.

<figure class="demo" data-demo="offset-polyline">
<figcaption>The same logic on a chain of straight lines, computed directly, not by constraints. Drag the chain's points and the distance. The dashed line is the offset before cutting. In the kernel the corners are true arcs tied by constraints; here they are drawn as 16 chords, so the test of what to drop allows half a percent. Try the notch: narrower than twice the distance, it is passed over.</figcaption>
</figure>

**Corners.** Take a corner of the chain, offset to its left. Let `$t_1,t_2$` be the tangents arriving and leaving.

- Where the chain runs on smoothly, a `Tangent` by constraint, the offsets meet tangentially too.
- Where it turns *away* from the offset side (`$t_1\times t_2<0$` for a left offset), a gap opens. It is closed by an arc round the corner, tangent to both offsets (`Corners::Round`), or by extending both until they meet (`Corners::Extend`). The arc's center is the corner, so its radius *is* the distance of both offsets it touches.
- Where it turns *towards* the offset side, the offsets cross: both are cut back to where they meet. A construction circle about the corner, touching both, says the distance holds there.
- Where it runs on straight, as where the halves of a mirrored outline meet on the mirror line, the offsets end closer than the solver tells points apart, and they meet square to the corner.

An open chain's offset ends square to the chain's ends.

**Crossings.** Where a piece is too short for the corners at its ends, or where offsets run into each other, as when offsetting into a notch narrower than twice the distance, the offset drawn round the chain crosses itself. It is cut where it does, and what lies nearer the chain than the distance is dropped. What is left runs on from part to part at the crossings, each part still tied to its source. The trick is what holds the distance on a part that lost its corners: a part cut off at both ends "holds it by a circle about its source's corner touching it, as large as one about the first part's". So one dimension still says how far all of it is.

Think about why that test is the right one: **a point of the raw offset that lies nearer the chain than the distance cannot be on the true offset**, because every point of the true offset is at exactly that distance from the chain. The raw offset is a superset that crosses itself; cutting at the crossings gives runs, each of which is entirely valid or entirely invalid.

**Splines.** The offset of a spline is no spline. So it is fitted: a cubic spline through points of the true offset, with as many control points as it takes to come within `FIT_TOLERANCE` of the sketch's size of it everywhere, proven (`NurbCurve2D::offset_miss`), up to `MOST_FIT_POINTS`. Each of those points is held to the spline, at its parameter, the distance away along its normal, by a `Constraint::OffsetAt`, with the distance the radius of a construction circle. So even a spline's offset stays a live constraint set. `core/geop-core-geometry/src/nurb_curve/offset.rs` holds the geometry. Note this is another fitted surface in the same spirit as [Lecture 31](./31-offsets.md): the bound is what makes it honest.

**Several chains at once.** An outline and its holes, offset together, each go into the same area: of two closed chains one inside the other, one inward and the other outward. Each is run the way that has that area on its left, so that where their offsets cross they are cut and run on from one into the other, as a chain's own do.

**Refused**, by name, as AGENTS.md asks of every operation that handles only some configurations: an arc offset past its center, a spline offset past its center of curvature, a circle's offset that another crosses, and an offset that leaves nothing.

### When a trimmed offset stalled the solver

The offset is why the sketch solver had to become more careful. Two arcs of a trimmed offset, held to one crossing by a tangency each, made the constraint rows ill-conditioned, which the normal equations then squared, so the solve stopped flat with the dimension's residual untouched. See the Householder story in [Lecture 23](./23-sketch-solver.md). Building more of a kernel with constraints stresses the solver in ways you cannot predict, which is a good reason to have one solver rather than many.

## Text

`CurveKind::Text` is the outline of a string in one embedded font, a subset of Noto Sans Regular (`core/geop-core-sketch/src/text.rs`). Text is a fine demonstration of exactness and structure:

A TrueType outline is lines and quadratic Béziers between points of whole font units, so *it is a NURBS exactly*. A run of quadratics, each starting halfway between the control points on either side of it (a TrueType "implied" point), is one quadratic B-spline on uniform knots, the off-curve points being its control points. So a run becomes one piece of the profile, which the extrusion makes one face, and not a face per Bézier.

Overlapping contours were united in preparing the font subset (with `fontTools.ttLib.removeOverlaps`), so that no two contours of a glyph cross and each contour is a loop of the profile. Only the anchor is solved for. A text takes no constraint itself, and is placed by constraining that point. Each closed contour of a glyph is a loop, its pieces numbered on from one contour to the next, so every face an extrusion makes of a letter has a name of its own.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"name_plate","example":"name_plate","yaw":0.4,"pitch":0.7}'>
<figcaption>The <code>name_plate</code> example, as the real kernel meshed it: the word in glyph outlines, each letter extruded from the plate's top face, the holes of the O and P open down to it.</figcaption>
</figure>

## What a sketch is given

Not everything in a sketch is drawn. **Reference geometry** is given: fixed points and curves that the solver never moves and constraints are measured against. Every sketch has its origin as a point and its `x` and `y` axes as construction lines. And a sketch can project a vertex, an edge, a face or another sketch's curve into its plane (`ops/geop-ops-sketch/src/references.rs`).

A projection refers to what it projects **by name, never by position**. Every time the sketch is built, each reference is brought up to date with the part (`Reference::update`), so a sketch dimensioned against a projected edge follows the edge when an earlier step moves it. This is the program model of [Lecture 2](./02-programs-of-steps.md) at work.

And here is a point of doctrine. AGENTS.md: "Where a user needs the exact line or arc back, as a sketch projecting an edge does, it is reconstructed locally where it is used (the sketch solver), not kept as a kind in the B-rep." In the B-rep an edge is a NURBS curve and nothing else. Projecting reads what an edge *is* from that curve: a straight one is a line, a circular one in a plane parallel to the sketch an arc or a circle, one seen edge-on the line it collapses to, and anything else is the spline it projects to exactly. A parallel projection is affine, so it moves the control points and keeps weights and knots ([Lecture 6](./06-bezier.md)). All of it is reference geometry: fixed, construction, and never part of a profile.

## Dimensions as formulas

A dimension's value can be a formula of the part's parameters: `width / 2`, `screw.clearance`. `AddSketchArgs::formulas` keeps them with the step, keyed by the constraint's id, and `apply_formulas` in `ops/geop-ops-sketch/src/add_sketch.rs` resolves them each time the step runs. The sketch is solved anew when a formula's value or a projection changed.

The whole sketch stays a value: serializable data, stored verbatim in the program. `Drawing` in `lib/geop-lib-stdlib/src/drawing.rs` builds on this for the standard parts, where every point has two formulas ([Lecture 23](./23-sketch-solver.md)).

## 3-D sketches

A sweep's path or a wire harness's route does not lie in a plane. `Sketch3d` in `core/geop-core-sketch/src/space/mod.rs` keeps the same principles for points in space:

- **Points carry all positions.** Curves refer to points by id. `Constraint3d::Coincident` joins separately drawn points into one, which removes variables just as in the plane.
- **What is given from outside is fixed.** A point at a vertex of the part or at a datum point is a fixed point. An edge of the part that a point is constrained onto is a `CurveKind3d::Reference`, a curve whose geometry is handed in. The solver never moves either.
- **Directions are given.** An axis or a straight edge a line is parallel to, or a tangent a spline leaves along, is a direction vector in the constraint (`Constraint3d::ParallelTo`, `Constraint3d::TangentTo`), brought up to date by the sketch's owner.

The curves are a **line** between two points, a **circular arc** from a point, through a second, to a third (it lies in their plane), a **helix** (`CurveKind3d::Helix`, the tool `h`), by its axis from `base` to `top`, its start, its turns and its hand, and where it arrives following from those (`Constraint3d::HelixEnd`, added by `Sketch3d::add_helix`, computed through `Helix3`), and a **spline** through points: the natural cubic spline, built by `NurbCurve::cubic_spline`. There is a real design decision in that last one. A spline end's tangent is *how the spline is built*, not something the solver moves points for: a spline end `TangentTo` a line leaves along the line.

The same engine does the rest. `Sketch3d::solve` and `Sketch3d::solve_with_drag` use `core/geop-core-sketch/src/space/solve.rs`, with the shared `Variables` and `System`; `Sketch3d::enclose` encloses the exact solution; `Sketch3d::chains` and `Chain3d::to_nurbs` turn the curves into NURBS in space, chain by chain, for sweeps ([Lecture 27](./27-path-sweep.md)).

**A wire is a graph.** When a 3-D sketch is built as a wire, `Sketch3d::wire_edges` and `Sketch3d::wire_edge_nurbs` (`core/geop-core-sketch/src/space/wire.rs`) cut every curve at each point it shares with another: an end of one, a point a spline is drawn through, an arc's middle point. Then the wire's edges meet only at its vertices, which is what the kernel requires of a body's edges ([Lecture 17](./17-brep.md)). `Sketch3d::check_wire` refuses curves that meet anywhere else by name, using a real `curve_curve_intersect` of every pair, "since a crossing that is no point of the sketch is one the user cannot see". The B-rep's rule, that a body's edges only touch at vertices, reaches back into sketching here.

The operation itself is `AddSketch3d` in `ops/geop-ops-sketch3d/src/add_sketch3d.rs`. It places points on the part's vertices, edges, faces and datums where the pointer hits them, or at typed coordinates.

<div class="kernel">

**In the kernel.** `core/geop-core-sketch/src/profile.rs` (regions, chains, `to_nurbs`), `core/geop-core-sketch/src/copies.rs` (`Sketch::mirror`, `Sketch::pattern`), `core/geop-core-sketch/src/offset.rs` (`Sketch::offset`, `Corners`, `Offsetted`), `core/geop-core-sketch/src/text.rs`, `core/geop-core-sketch/src/space/`. The editing side, with tools, snapping and the fillet tool, is the sketch operation, in `ops/geop-ops-sketch/src/`; snapping there is by constraint, never by moving geometry: a point placed on a curve is `PointOnCurve`.

The offset works with lines, arcs, circles and splines. The refusals are listed above. It is the sketch's largest module; do not take the demo's simple cut-and-filter on straight lines for all it does.

</div>

## Recap

<div class="recap">

- Regions come from shared point ids, nesting from real ray crossings, winding from an integral: no tolerance and no samples.
- What a sketch derives, it does not store. A copy is a constraint, a pattern is its `by` curve and a chain of `Moved`s, and an offset is one dimension and a set of `Parallel`, `Concentric` and `Tangent` constraints.
- An offset's corners close a gap by an arc about the corner, or cut back at a crossing. A self-crossing is cut, and what lies nearer the chain than the distance dropped.
- Text is exact because TrueType outlines are NURBS. A projection reconstructs the line or arc locally, referring by name.
- A 3-D sketch uses the same engine for paths and wires. A wire is a graph: curves meet only at their shared points.

</div>

## Exercises

**1.** A rectangle of 4 by 2 with counter-clockwise lines, a line first running along the `$x$` axis, is offset outwards by 1 with `Corners::Extend`. Give the offset rectangle's corners, and say which constraints tie each of its sides to its source and how many dimensions there are. What changes with `Corners::Round`?

<details class="solution"><summary>Solution</summary>

The offset's corners are `$(-1,-1),(5,-1),(5,3),(-1,3)$`, a 6 by 4 rectangle. Its first side is tied by `Offset` with the dimension; the other three are `Parallel` to their sources, with the distance carried at each corner by a construction circle about the corner, touching both. One dimension. With `Round`, each corner becomes an arc of radius 1 about the source's corner, a quarter turn, tangent to both offsets, and the sides end at the arcs: 4 sides and 4 arcs, still one dimension.

</details>

**2.** Why does `Sketch::regions` need only one probe point per loop, when "Do not decide from samples" is a rule of the kernel? What invariant makes this not a sample?

<details class="solution"><summary>Solution</summary>

Loops of one profile never cross, so for two loops a point of one is inside the other exactly when all of it is. One point is then the exact answer to a topological question, and the ray-crossing count is exact on the real curves. It would be a sample only if a loop could partly lie inside the other. (Three curves meeting at a point are refused, and loops sharing a point cannot arise otherwise.)

</details>

**3.** A pattern's construction line `by` runs from `$(0,0)$` to `$(3,1)$`. A point is at `$(1,1)$`, with 3 copies. Write the residuals for `Moved` for the first copy. Then write the residual for a circular pattern where `by` is an arc from `$(1,0)$` to `$(0,1)$` of half sweep `$\pi/4$`, and compute the image of `$(2,0)$`.

<details class="solution"><summary>Solution</summary>

Line: `$b-(e+(a-s))=b-(a+(3,1))$`, so the first copy is at `$(4,2)$`, then `$(7,3)$`, `$(10,4)$`. Arc: the motion is `$q\mapsto e+R(q-s)$`, with `$R$` the rotation by `$2\cdot\pi/4=\pi/2$`. For `$q=(2,0)$`: `$q-s=(1,0)$`, turned to `$(0,1)$`, plus `$e$` gives `$(0,2)$`, which is indeed `$(2,0)$` turned by a quarter about the origin, the arc's centre.

</details>

**4.** In the offset demo, set the distance so the notch is exactly passed over, and explain what happens as you shrink the distance below half the notch's width: which pieces of the dashed line come back?

<details class="solution"><summary>Hint</summary>

The notch is `$0.6$` wide, so half of that is `$0.3$`. Below it, the arcs round the notch's two lips no longer cross and the offset runs down its sides and along its bottom, since these runs are now at a distance `$\ge d$` from every part of the chain. Above it, the two arcs cross over the notch; the runs below that crossing are nearer the opposite wall than `$d$`, and dropped.

</details>

**5.** A TrueType quadratic run has off-curve control points `$A,B,C$`, implied on-curve points at the midpoints, and on-curve ends. What are the knots of the single B-spline, and why does the C1 joint at an implied point follow from the construction?

<details class="solution"><summary>Hint</summary>

Clamped uniform quadratic knots: each interior knot has multiplicity one, and the B-spline passes through the midpoint of two consecutive control points at the knot between them. A quadratic B-spline with simple interior knots is C1 at each one ([Lecture 7](./07-bsplines.md)); the tangent there is along `$B-A$`, the same on both sides, which is what the midpoint construction enforces on the Bézier pieces. See `outlines` in `core/geop-core-sketch/src/text.rs`.

</details>

**6.** Suppose a 3-D sketch has a spline through four points and a line that happens to pass through the spline's middle, not at any of the points. What does `Sketch3d::check_wire` do, and why not simply insert a vertex at the crossing?

<details class="solution"><summary>Hint</summary>

It refuses, by name. A vertex at an unnamed crossing is a point the user never made, cannot see or constrain, and which moves along the spline when anything changes. A coincidence of points is a statement; a numerical crossing is not. Make the crossing a sketch point the line ends at, and the spline passes through, and it is a vertex.

</details>
