# Lecture 18: Poles, seams and splicing

<div class="goals">

**After this lecture you can**

- explain what goes wrong on a surface that closes on itself or collapses to a point, and how Geop avoids or represents each case;
- describe a degenerate (`Vertex`-backed) coedge and say why a loop needs one;
- split an edge at a vertex, stating exactly which parameter it is cut at and why;
- work through all six cases of `splice_edge_into_face`, rewiring the four pointers by hand and predicting the resulting loops and faces;
- show that every splice preserves the Euler count;
- explain why "on this surface" stops being a membership test once a face has been split.

</div>

[Lecture 17](./17-brep.md) described a B-rep at rest. Now we change one. Everything a boolean does to topology comes down to two edits: *split an edge at a vertex*, and *splice an edge into a face*. Before that, we have to deal with the two places where a surface's coordinates stop being a faithful picture of the surface.

## The trouble with coordinates

A pcurve lives in the `(u, v)` plane. Face containment ([Lecture 19](./19-classification.md)) counts crossings of a ray with the pcurves in that plane, and so relies on a fact that sounds too obvious to state: **a face's loops are closed polygons in `(u, v)`**. Two kinds of surface break the picture.

**A closed surface has a seam.** Take a whole cylinder as one patch, `$u\in[0,1)$` round the axis. The lines `$u=0$` and `$u=1$` are one line in space. A loop around the wall is closed in space and is no closed curve in `(u, v)`. It runs from `$(0,v)$` to `$(1,v)$`. The classical remedy is a *seam edge*, one edge that a single face uses twice, once on each side of the parameter domain. Then the outer loop goes along the bottom, up the seam, back along the top, and down the seam. It works, and it infects everything: an edge whose two coedges are on one face, a point on the seam with two `(u, v)`, a surface evaluated past its domain's end by anyone who walks across. And a NURBS patch cannot close on itself smoothly and stay a single simple tensor-product patch unless it is periodic.

**A pole collapses a row.** Take a sphere with `$v$` running pole to pole. The whole row `$v=0$`, every `$u$`, is one point in space. Here the surface is perfectly smooth and only the coordinates are singular. As AGENTS.md puts it, *the surface is fine, the coordinates are not*.

### Geop's way with seams: there are none

Every surface in Geop is a plain patch over a rectangle, and nothing closes on itself. A full turn of a revolve is cut into four spans, each a face of its own, so a cylinder is four quarters; `lib/geop-lib-sweep/src/revolve.rs` does this, and the wall of the drilled box in Lecture 17 was four faces for this reason. A "seam" in Geop is an ordinary edge between two ordinary faces.

STEP files, from systems which do use seams, have periodic faces with one edge used twice. The importer takes them out, cutting such a face along meridians into sectors, each on a patch of its own (`ops/geop-ops-step/src/import/body.rs`; see [Lecture 35](./35-step-and-drawings.md)). So the rest of the kernel does not need a seam path at all. It is the same bargain as *Everything is a NURBS*: one more piece of geometry, and one whole class of branch gone.

<div class="kernel">

**In the kernel.** You will see the word *seam* in the code in two loose senses. In `core/geop-core-topology/src/model/iterate.rs` it refers to a loop running along one edge twice, which the topology allows (as a spur does). In `lib/geop-lib-booleans/src/remesh/remesh_edges_x_faces.rs`, `seam_plane` means an edge between two coplanar patches, and it decides whether such an edge carries anything a third face needs. Neither is a periodic surface.

</div>

### A pole needs a coedge with no edge

A pole cannot be cut away: a cone has an apex and a sphere has two poles, whichever way you cut it. So look at one octant of Geop's sphere (`lib/geop-lib-sweep/src/shapes/sphere.rs`). It is a face over the unit square whose row at the pole has collapsed. Three of its sides are real arcs. The fourth is a single point in space.

Now consider the loop. In space it goes round three arcs: down a meridian, along the equator, back up. It is closed. But in `(u, v)`, one meridian ends at `$(0,1)$`, say, and the other starts at `$(1,1)$`. *Both corners are the pole.* If the loop jumped from one to the other, a ray in `(u, v)` could pass through the gap uncounted, and the parity of the crossings would be wrong for points near it. It is exactly the kind of bug no single-coedge check can see, and `core/geop-core-topology/src/validation/pcurve_loop_continuity.rs` exists to catch it.

So the loop needs a piece of boundary that runs along the row in `(u, v)` while staying at one point in space. That is `CoedgeGeometry::Vertex(id)`: "a degenerate loop segment that sits at a single, already-existing vertex the whole way". It has a pcurve, along the row. It has no edge, and so no opposite: there is nothing to share between two faces. It adds no vertex and no edge, so it leaves the Euler count alone. In a `BodySpec`, it is `CoedgeOn::Vertex`.

Picture the octant's outer loop, with its pole at `$v=1$`:

```text
(0,1) ◀── Vertex coedge along the row ── (1,1)     all of this row is the pole
  │                                        ▲
  ▼  meridian 1                  meridian 2│
(0,0) ───────── equator arc ─────────▶ (1,0)
```

Four coedges, closed in `(u, v)`; three edges, closed in space.

### One vertex at many `(u, v)`

A pole then behaves unlike any other vertex: a loop passes it at several places in parameter space. Which corner an arriving edge belongs to is told by *its* `(u, v)`, not by the direction it leaves the vertex in. This caused a real bug: pinning an arriving edge to the first coedge that happened to end at the vertex bends the edge to a corner at the row's end, wherever it actually arrives.

The fix (AGENTS.md, *A pole is one vertex at many `(u, v)`*) is in `splice_edge_into_face`. It first fits the new edge's pcurve leaving its end at a pole unpinned. Then `split_pole_row` looks at where the pcurve ended. If that is strictly inside the row's coedge, it cuts the row in two there, at a sharp `$u$` (where to cut is a free choice), and the new edge is spliced between the two halves. So the row gets a corner exactly where the edge arrives.

Degenerate parametrizations have other consequences, for projection and for Newton, which we saw in [Lecture 14](./14-newton.md): a seed on a pole never leaves it unless you step along a meridian. Together these give the general lesson. Whenever an algorithm reads a coordinate near a pole, ask whether what it wants is a fact about the coordinates or about the surface.

## Edit one: splitting an edge at a vertex

Suppose an edge passes through a point where a vertex is wanted. `Model::split_edge_at_vertex` (`core/geop-core-topology/src/edit/split_edge_at_vertex.rs`) makes two edges joined at that vertex. The existing edge keeps the first piece, and a new edge holds the second, "in the same direction". Every coedge along it is split to match, and the two pieces are linked into the loop in place of the one.

It sounds simple. The code's comments say why it is not. Three parameters are involved: `$t$` on the 3-D curve, and one on each pcurve.

**Where to cut is a free choice, and a good one is a sharp one.** The search that found the crossing returned an enclosure of the parameter. The cut goes at the sharp middle of its Newton refinement, `NurbCurve::split_parameter_at_point` (`core/geop-core-geometry/src/nurb_curve/refine.rs`), which then checks that the curve reaches the vertex at that parameter. AGENTS.md's section *Width along a split edge* gives the numbers behind this. Cutting instead at the whole refined enclosure carried the parameter's width through Boehm's formula (`$\alpha=(t-a)/(b-a)$`) into the pieces' control points and knots. The next cut's parameter, found on the wider piece, came out wider. The width grew threefold per cut: a quarter arc cut forty times went from `$3\times10^{-15}$` to `$0.08$`. Cut at a sharp point, each cut adds one subdivision's rounding.

**Validate the value you are about to use.** The check is done at the sharpened parameter, not the wide one. A wide box genuinely contains the vertex, but "is the vertex somewhere along this stretch?" and "does cutting exactly here give that vertex?" are different questions.

**Cut the pcurves at that same point.** The point where the pieces will meet, `$C(t)$`, is taken first. Each coedge's face then locates that point on its surface (`NurbSurface::locate`), not the vertex: "Cut each at a point of the vertex of its own and the pieces of the edge and of its pcurves end apart, by as much as the vertex is wide". In the case that exposed this, `1.4e-13` apart, where a cubic crossed a plane at its inflection.

**Refine the target, then the parameter.** The `(u, v)` has to be pinned down first. A pcurve parameter refined against a target only as sharp as the containment search's tolerance gives a wide residual, a wide step, and a result no narrower than it started, silently. That is why it is `surface.locate` here: located, not merely isolated.

Everything is checked before anything is changed, so a pcurve that cannot be split does not leave a half-updated model.

Do not reconcile a mismatch by moving geometry. AGENTS.md records pinning the split curve's endpoint onto the vertex: it looked like it worked for the 3-D curve, and applied to the pcurve it pinned a control point to a box so wide the pcurve lost its tangent, taking a sweep of test scenes from 0/175 failing to 12/175. Fix the parameter, not the geometry.

## Edit two: splicing an edge into a face

`Model::splice_edge_into_face` (`core/geop-core-topology/src/edit/splice_edge_into_face.rs`) takes an edge that **already exists**, usually because two faces are meant to share it, and makes it part of one face's boundary. It adds two new coedges on that face, `fwd` and `rev`, one each way along the edge. (So each face gets a pair. A finished manifold edge has one coedge on each of two faces; what a splice imprints *into* a face is a cut with a side for each piece of it. When the face is split, the two end up on two different faces. In a boolean, faces on the other side of the cut are dropped.)

What happens depends only on which of the edge's end vertices are already on the face's boundary and, if both are, on which boundaries.

| The edge's ends on the face | Result |
|---|---|
| neither | a new ring of two coedges inside the face: a **hole** |
| exactly one | a **spur**: out along the edge and back, inserted into that vertex's loop |
| both, on the same hole | a **new face** from the ring of positive area; the other ring stays a hole |
| both, on two different holes | the holes **merge** into one |
| both, on a hole and the outer loop | the hole is **absorbed** into the outer loop |
| both, on the outer loop | the face is **split in two** |

The last four all begin with the same rewiring. Let `a` be a coedge arriving at the edge's start, `b` a coedge arriving at its end. Then four assignments:

```rust,ignore
let a_next = a.next;
let b_next = b.next;
link(a, fwd);      // arrive at the start, run the edge forward
link(fwd, b_next); // leave the end along what used to follow b
link(b, rev);      // arrive at the end, run the edge back
link(rev, a_next); // leave the start along what used to follow a
```

These are the actual lines, abbreviated. Think through what they do to a ring. If `a` and `b` are on one ring `$R$`, write `$R=a\,X\,b\,Y$`, with `$X$` the coedges from `a_next` to `b`, and `$Y$` those from `b_next` round to `a`. After the rewiring, `$a\to\text{fwd}\to Y\to a$` is one ring, and `$X\to b\to\text{rev}\to X$` is another. **One ring became two.** If `a` and `b` are on two different rings, the same four assignments give one ring: a walk goes `a, fwd`, all round the second ring, `b, rev`, then round the first back to `a`. **Two rings became one.** All of the case analysis follows from this simple fact: the same four pointer moves either divide a ring or join two.

Try every case. In the demo, click two vertices of the boundary, or click in the middle of the face for a free point (it snaps to a half-unit grid). It runs the same case analysis and the same rewiring on a small real half-edge structure, and reports what it found. The total area of the faces is shown too. It never changes: a splice rewires boundaries, and moves no material.

<figure class="demo" data-demo="splice-cases">
<figcaption>Click two points. A straight edge is spliced between them. Try, each from the start: a vertex of one hole to a vertex of the other (merge); the corner (0,0) to the hole's corner (1,1) (absorb), and then (1,3) to the corner (0,4), both now on the outer loop (split); two free points (hole); one vertex and one free point (spur). To get the same-hole case, merge the two holes at (2,1) and (4,1), then connect (2,3) and (4,3). Each arrow is one coedge, on the left of it the material of its face. The kernel's edges are curves with pcurves; these are straight, and a crossing is refused rather than split first.</figcaption>
</figure>

### The six cases

**Neither end on the face.** The new edge floats in the interior of the face. `fwd` and `rev` link to each other to make a ring of two coedges, and that ring is pushed onto `holes`. It encloses no area. It happens when an edge lying in the face is imprinted before anything else has reached it: the next edge imprinted beside it attaches to this ring, as we will see in the next lectures.

**One end on the face.** `splice_spur` inserts `fwd` and `rev` after the arriving coedge `a`: out and straight back. The ring gets longer by two and its area does not change. `face.holes` and the set of faces are unaffected. The code insists that the free end "has to lie strictly inside" the face, with a test by `face_contains`, and returns an error rather than a loop that runs out of the face and back. That check exists because of the test scene `narrow_groove`, whose failure surfaced far downstream in classification.

**Both on two different holes.** Rings join. Set the boundary at hole `$i$` to the new ring and remove hole `$j$`.

**A hole and the outer loop.** Rings join. The hole "stops being a separate boundary and its coedges become part of the one ring that now bounds the face". The face has a slit from its outer boundary to what was the hole. It is a valid face, with a loop that runs along that slit twice.

**Both on the outer loop.** One ring becomes two, both bounding material. This is the case the boolean is built on. `split_face` keeps one ring in the existing face and makes a new face from the other, on the same surface, in the same shell (`new_face_from_ring`). Then each of the old face's holes must go with whichever piece it now lies in: take a point on the hole, and ask `face_contains`. "Inside" moves it to the new face, anything else stays. A hole with no usable point stays put "rather than being dropped: leaving it on the wrong face is a recoverable error, losing it silently fills in a hole that should exist".

Before doing that, it computes the signed area of both rings. "A half of zero area means the new edge runs along the stretch of boundary between its own endpoints." It returns the error `DEGENERATE_SPLIT` rather than building a sliver, because a zero-area face is structurally perfect and every other check accepts it. (We see the bug that led here in [Lecture 21](./21-remesh.md).)

**Both on one hole.** One ring becomes two, but which is which? Let the hole be the region in the middle, and let the edge run from one point of its boundary through material to another. Together with one arc of the hole, the edge encloses a patch of material, now cut off. Together with the other arc, it goes around everything: the hole, plus that patch. The first ring bounds material, and is a new face. The second still has material outside it, and stays a hole. To tell them apart: "both rings pass through the same vertices and contain the same edge, so no purely topological test can tell them apart". Only the winding can: the ring of positive signed area bounds material. If the area cannot be decided, the code reports it, rather than guessing.

(AGENTS.md's short summary says two points of one hole "divide it into two holes", and says only the outer split creates a face; and the doc table above `splice_edge_into_face` still says "divided into two holes". The code is what is described here, and its own comment agrees: dividing a hole is a case that creates a face.)

### Which corner?

A vertex can appear more than once on a boundary. The root of a spur is passed twice, once on each side of the spur; so is a point where a face touches itself. Then there are several arriving coedges, each with its own corner, and the new edge belongs in the one it leaves into. `choose_corner` decides it from the rule that the material fills "the angle swept counter-clockwise from the way the boundary leaves to the way it arrived from". The test scene `three_inscribed_bores_remesh_imprints_every_arc` shows the failure. An arc spliced as a spur from a point of a bore's rim, and the rim arc spliced at that same point a moment later, landed the spur in the wrong piece of the wall. Since the corner choice can move the pin, the new pcurve is then refitted.

### Pins

The new pcurve has to start exactly where the arriving coedge ends in `(u, v)`, for the loop to stay closed there. `pin` takes the arriving end, **intersected** with the vertex's projection onto the surface. Using the end alone let `interpolate_enclosing`'s width spread around the loop until a split came out degenerate. Both values enclose the one `(u, v)` the vertex is at, so their intersection is honest and tighter. AGENTS.md uses this as its example of *combine two enclosures of the same value*: union when you must cover both possibilities, intersect when both constraints must hold, average never ([Lecture 5](./05-honest-numerics.md)).

## Splitting a face invalidates "on this surface"

There is a subtle consequence of the face-split case. Both halves share one surface, so every edge already imprinted into the original is still "on the surface" of the half it does not belong to. Before faces had proper topology, an imprinting loop asked `curve_surface_intersect` whether an edge was on a face's surface, found the edge on the other half too, imprinted it there, split that half, and so on, forever.

Faces with real outer loops and holes can answer the right question: is this edge inside *this face's trim*? That is `face_contains`, in `find_coincident_pair` of `lib/geop-lib-booleans/src/remesh/remesh_edges_x_faces.rs`. AGENTS.md's *Proximity is not membership* generalizes this: being near, or on an untrimmed surface, is not belonging to the face. The surface of a face runs on past its trim.

<div class="kernel">

**In the kernel.** Other edits, all in `core/geop-core-topology/src/edit/`: `merge_vertex.rs` and `merge_edge.rs` merge duplicates (a merged vertex's point is the `union` of both). `reverse_face.rs` we met in the last lecture. `assemble_solid.rs` makes the result of a boolean from the faces to keep, deleting what is no longer reachable. `transform_body.rs` moves a body.

`core/geop-core-topology/src/merge.rs` goes the other way, undoing what splices did: `BodySpec::merged` joins faces lying on one surface into one and drops vertices between two edges of the same two faces. A union of two blocks flush with each other would otherwise leave a seam edge that is no edge of the shape.

</div>

<div class="recap">

## Recap

- Geop has no periodic faces. A closed surface is cut into patches; imported seams are cut away. A seam in Geop is an ordinary edge.
- A pole is a row of parameter space at one point of space. A `Vertex` coedge runs along the row, so that the loop closes in `(u, v)`.
- Splitting an edge cuts the curve at a sharp parameter it has checked, and cuts each pcurve at the point that gave; width does not accumulate.
- Splicing an edge makes two coedges and four pointer moves: they divide one ring in two or join two rings in one. Everything else is deciding what those rings mean, from `BoundaryIndex` and, for one hole, from the signed area.
- Faces share surfaces after a split, so "on the surface" no longer means "belongs to the face".

</div>

## Exercises

**1.** Show that every case of the splice table preserves `$V-E+F-L$`.

<details class="solution"><summary>Solution</summary>

Let the edge be counted in `$E$`, and its vertices in `$V$` if new.

- Neither end on the face: `$V{+}2$`, `$E{+}1$`, `$L{+}1$`. Change: `$2-1-1=0$`.
- One end: `$V{+}1$`, `$E{+}1$`. Change: 0.
- Two holes merge: `$E{+}1$`, `$L{-}1$`. Change: `$-1+1=0$`.
- Hole absorbed: `$E{+}1$`, `$L{-}1$`. Change: 0.
- Outer split: `$E{+}1$`, `$F{+}1$`. Change: 0.
- One hole: `$E{+}1$`, `$F{+}1$`, and `$L$` the same. Change: 0.

A split of an edge at a new vertex: `$V{+}1$`, `$E{+}1$`. Change: 0. (This counts the edge as new to the body it is spliced into. In the middle of a boolean an edge is spliced into faces of both bodies, and the formula does not apply to that unfinished state.)

</details>

**2.** In the demo, merge the two holes by connecting vertices `(2,1)` and `(4,1)`. Then connect `(2,3)` and `(4,3)`. What case is the second, what are the rings, and what is the area of the new face?

<details class="solution"><summary>Solution</summary>

Both ends are on the one merged hole, so it is the same-hole case. The rings are the rectangle `$(2,1),(4,1),(4,3),(2,3)$`, of area 4, which runs counter-clockwise and becomes a new face; and the outline of everything else, hole 0 and the rectangle and hole 1, of area 8, clockwise, which stays a hole. The old face's material is now `$24-8=16$`, and with the new face's 4 the total is still 20. The test `one_hole_makes_a_new_face` in `book/scripts/demo_tests/splice-cases.test.mjs` checks the same case on a hole with a bite taken out of it.

</details>

**3.** Write out, in terms of `$R=a\,X\,b\,Y$`, what happens when `a` and `b` are the same coedge: an edge from a vertex back to itself.

<details class="solution"><summary>Hint</summary>

Take `$X$` empty and treat `b_next` and `a_next` carefully: with `a` = `b`, the four assignments overwrite each other. The kernel's cases do not arise this way, because each end is chosen separately by `choose_corner` and an edge with `start_vertex == end_vertex` is a closed loop in the pcurve. Work out the result on paper and compare it with what you would want: a closed curve from a vertex on the loop back to itself, which should cut off one region.

</details>

**4.** `new_face_from_ring` decides which of the old face's holes move to the new face by one point of each hole. Why is a single point enough?

<details class="solution"><summary>Solution</summary>

A hole is one connected curve that does not cross the new edge or either ring, since the remesh already put a vertex at every crossing (`check_edges_disjoint` would reject otherwise). So the hole lies wholly in one of the two pieces, and any point on it decides which. It is the same argument as for `holes_inside_outer`, Lecture 20. The argument requires a point away from the shared boundary: in the same-hole case, a point on the hole being divided is on the new ring. `face_contains` then answers `OnCoedge`, not `Inside`, and the hole stays.

</details>

**5.** Why does `split_pole_row` take a sharp `$u$` at which to cut the row, when the end of the pcurve is an enclosure? Is this the "free choice" of AGENTS.md's rule on sharpening?

<details class="solution"><summary>Solution</summary>

Along the row, every `$u$` is one point in space, so every value in the enclosure ends the edge at the pole. Which `$u$` the row is cut at changes nothing in 3-D. It is a free choice, and sharp is the good one, since the row's two pieces and the new pcurve then meet at one number. The new pcurve is then refitted, pinned to that cut. Compare a point on an ordinary edge, where a parameter's enclosure maps to a stretch of space: a sharp value is then still a free choice only because the cut is re-validated against the vertex (`split_parameter_at_point`).

</details>
