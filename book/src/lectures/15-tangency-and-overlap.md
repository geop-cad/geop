# Lecture 15: Tangency and overlap

<div class="goals">

**After this lecture you can**

- compute how a crossing's enclosure widens as a crossing approaches tangency, and why the answer at a tangency is the square root of the data's precision;
- say what each tool of the last three lectures does at a tangency: search, clip, Newton;
- explain why counting solutions cannot detect an overlap, and derive the direct test the kernel uses;
- state the equation behind a tangent branch leaving an edge;
- recognise a configuration your own code created, and avoid it: a tool's curve lying in a face, a cut on an existing corner, an approximate tangency.

</div>

The last three lectures assumed that two curves cross at a good angle. Real CAD models are full of the opposite. A fillet is tangent to the faces it blends, by definition. A hole drilled to the same radius as a boss has coincident walls. Two parts are mated with faces flush against each other. So a kernel's hardest cases are not rare accidents. They are what users do on purpose.

This lecture is about the geometry of those cases, and what an honest kernel can and cannot say about them.

## A crossing is an equation, and its condition number is the angle

Consider the simplest possible model. A curve `$y=t^2-c$` crosses the line `$y=0$`, and we know its height only to within `$\pm\varepsilon$`: that is, a coordinate of the curve is an interval of half-width `$\varepsilon$`. Where can the crossing be?

`$$t^2-c\in[-\varepsilon,\varepsilon]\iff t^2\in[c-\varepsilon,c+\varepsilon].$$`

**Far from tangency**, when `$c\gg\varepsilon$`, there are two crossings. Each lies in an interval around `$\pm\sqrt c$` of width

`$$\sqrt{c+\varepsilon}-\sqrt{c-\varepsilon}\approx\frac{\varepsilon}{\sqrt c}.$$`

The crossing angle has slope `$2\sqrt c$`, so this is the data's whole uncertainty `$2\varepsilon$` divided by the slope. A shallower crossing is less well determined, in proportion.

**At tangency**, with `$c=0$`, there is one contact and its enclosure is

`$$t\in[-\sqrt\varepsilon,\sqrt\varepsilon],\qquad\text{width }2\sqrt\varepsilon.$$`

A coordinate known to `$10^{-12}$` pins the contact point to `$2\times10^{-6}$`. In double precision with `$\varepsilon\approx10^{-16}$`, half of the digits are gone before any algorithm starts. No method can recover them, because the data does not contain them. Every parameter in that range is equally a true contact point of some curve consistent with the data.

In between, when `$c\approx\varepsilon$`, there is a question that has no answer. With `$c=10^{-13}$` and `$\varepsilon=10^{-12}$`: are there two crossings, or one, or none? All three are consistent with the data. An honest search returns one interval, wide, and it may hold anything.

<figure class="demo" data-demo="tangency-width">
<figcaption>Move c through zero, and vary the uncertainty ε. Above: the parabola, the band of its uncertainty, and in red the enclosures of its crossings with the line. Below: the widest enclosure against ε on log axes, between the reference lines ε and 2√ε; on the transversal side it follows the slope of ε, and at c = 0 the slope of √ε. The enclosure is computed in interval style by a search on Bernstein ranges, not from the formula; the readout also shows how many iterations Newton needs.</figcaption>
</figure>

Here are numbers from that search (checked against the exact set `$t^2\in[c-\varepsilon,c+\varepsilon]$` in `book/scripts/demo_tests/tangency-width.test.mjs`).

| `$c$` | `$\varepsilon=10^{-4}$` | `$\varepsilon=10^{-8}$` | `$\varepsilon=10^{-12}$` |
|---|---|---|---|
| 0.25 | two, `$2.0\times10^{-4}$` | two, `$2.0\times10^{-8}$` | two, `$2.0\times10^{-12}$` |
| 0.01 | two, `$1.0\times10^{-3}$` | two, `$1.0\times10^{-7}$` | two, `$1.0\times10^{-11}$` |
| 0.0001 | one, `$2.8\times10^{-2}$` | two, `$1.0\times10^{-6}$` | two, `$1.0\times10^{-10}$` |
| 0 | one, `$2.0\times10^{-2}$` | one, `$2.0\times10^{-4}$` | one, `$2.0\times10^{-6}$` |

Read each row. Each one is `$\varepsilon/\sqrt c$` until `$\varepsilon$` reaches `$c$`, where it merges into one interval of width about `$2\sqrt{c+\varepsilon}$`. That is the whole theory. What about geometry in more dimensions? A surface bending away from another along a curve is the same picture in a cross-section. Near any tangent contact, the two sides separate quadratically, and so the same square root appears.

## What each tool does at a tangency

**The search** ([Lecture 12](./12-subdivide-to-isolate.md)). Leaves spread across a stretch of about `$\sqrt h$`, so there are `$h^{-1/2}$` of them, all touching. They merge into one cluster, and the cluster's width is `$\sqrt h$`, not `$h$`. See the tangent scene of Lecture 12's demo. This is the same square root again, this time from the search's own resolution rather than from the data.

**The clip** ([Lecture 13](./13-fat-lines.md)). A clip divides by the slope of the distance function, and at a tangency that slope is about the size of the piece, not a constant. So the zone it keeps is `$O(h^2)/O(h)=O(h)$`, a constant fraction of the piece. Clipping stops being quadratic, it fails the 20% rule, and the search goes back to halving. In `core/geop-core-geometry/src/intersection/curve_curve.rs`, a comment on the converged branch of `curve_curve_crossings` describes the outcome:

```rust,ignore
// The pieces' whole domains, not the tighter clip: pieces the
// search could not separate — a tangency converges on a chain
// of adjacent ones — must merge into one unresolved cluster,
// and adjacent domains share an endpoint where clips need not touch.
solutions.insert((a.domain_as_scalar(), b.domain_as_scalar()));
```

The boxes are merged by `union` (a `DisjointSet`), never averaged. The answer is a wide box and the statement "somewhere in here".

**Newton** ([Lecture 14](./14-newton.md)). For the double root of `$t^2$`, a step is `$t\mapsto t-t^2/2t=t/2$`. The error halves per step, down from squaring: in the demo's readout, Newton from `$t=0.7$` to a step of `$10^{-14}$` takes 6 iterations for `$c=0.25$`, 11 for `$c=10^{-4}$`, and 46 at `$c=0$`. With intervals it fares worse: the Jacobian's enclosure soon contains zero, the linear solve fails, and `newton::refine` returns the incoming box. This is correct. Refinement promised to tighten or leave alone, and here it leaves alone.

So at a tangency the tools all agree. They report a wide, honest box. What is then to be done with that box is up to the caller, and every caller has to be written with it in mind. Which is why a kernel should try to not make tangent contact where it can choose ("A tilt", below), and try to make the topology answer where it cannot.

## Overlap: why counting does not work

A tangency is a crossing that has gone soft. An overlap is a different thing: two curves running along the same arc, or a curve lying in a surface, so that the solution set is not points at all.

Lecture 12 showed that an overlap leaves `$O(1/h)$` leaves. It is tempting to detect it that way: "if the search returns more than `max_solutions` answers, the curves must coincide". That was indeed the code's approach once, and it is wrong in both directions, which AGENTS.md lists under "Proximity is not membership": the same shape of bug appears in `find_coincident_pair` reading a solution count as coincidence.

- A transversal crossing at a very fine handoff, among curves with width, also leaves many leaves. Lecture 12's signal depends on stopping early.
- A short overlap, or a coarse handoff, leaves few leaves. So a partial overlap can be missed.
- Many isolated crossings, say a wavy curve along a line, look like an overlap by count, and are not.

And a search that hits its budget hasn't answered at all.

### The direct test

There is a much better way, based on a piece of algebra. A rational curve is an analytic function of its parameter. If two rational curve pieces coincide on any interval, they lie on the same algebraic curve throughout. So a stretch of overlap between two single-piece curves cannot just stop in the interior of both. It can only begin or end **where one of the curves itself ends**.

This means there are finitely many candidate places, and you know where they are:

- an end of curve `$A$` that lies on curve `$B$`;
- an end of curve `$B$` that lies on curve `$A$`.

These are point-containment questions, which we solved in Lecture 12. Sort the candidates along `$A$`. Between two neighbouring candidates, `$A$` either lies on `$B$` throughout, or nowhere in the interior. So *one probe at the midpoint decides it*.

That is `find_overlaps` in `core/geop-core-geometry/src/intersection/coincidence.rs`, and `curve_curve_overlaps` in `curve_curve.rs` is the candidate list. (The module states this reasoning for single-piece geometry, and the candidates are the ends of the curves as given. A curve of several spans is one analytic function only span by span.)

Let us do an example. Let `$A$` be the straight curve from `$(0,0)$` to `$(1,1)$` with `$s\in[0,1]$`, and `$B$` from `$(0.5,0.5)$` to `$(2,2)$`.

1. Candidates. `$A(0)$` is not on `$B$`. `$A(1)=(1,1)$` is on `$B$`. `$B$`'s start `$(0.5,0.5)$` is on `$A$` at `$s=0.5$`. `$B$`'s end `$(2,2)$` is not on `$A$`. So the candidates are `$s=0.5$` and `$s=1$`.
2. Probe `$s=0.75$`: the point `$(0.75,0.75)$` lies on `$B$`. So `$[0.5,1]$` is an overlap.
3. The crossing search then runs on the rest of `$A$`, here `$[0,0.5]$`, where it is entitled to assume no overlap.

For a curve against a surface, `curve_surface_overlaps` adds the surface's boundary. A stretch of a curve lying in a patch can end at the curve's own ends, or where it crosses one of the patch's four boundary curves, and those are found by the curve-curve search again.

The result is the variant of `Intersections` we saw: `Found` (no overlap, isolated crossings only), or `Coincident`, which lists the overlaps' ends, then crossings on the rest, and then points spread evenly over the overlaps.

The module's documentation is honest about the limit: "The midpoint probe is the one place this is a test rather than a proof: a curve touching the other object exactly at two candidates *and* the midpoint between them, while leaving it in between, is read as an overlap." Note how little that leaves. It is one specific algebraic coincidence, stated in the code, not a tolerance.

### Overlap needs the same direction

The test of the midpoint, as stated, asks whether a point lies on the other curve. Enclosures can answer yes to that for two curves that plainly cross. A helix met a cylinder's rim at about three degrees, in the example `square_thread`, and validation called the two edges coincident along an arc. Each carried the width its construction left it, each end was found on the other curve at a parameter of its own, and those two ends lay no further apart than a shared vertex's width. Probed between them, on so short a stretch, one curve "lies on" the other at any angle, as far as enclosures tell.

The geometry has more to say: two curves lying on each other run the same way. So `curve_curve_overlaps` also asks that the tangents could be parallel there, `same_way`, every 2-by-2 minor of the two tangents' components having to enclose zero. A caller who gave a `reach` above zero is exempt, since a curve within `reach` of another counts as one with it whichever way it runs. The test `edges_meeting_at_a_shallow_angle_do_not_overlap` in `curve_curve.rs` states what must hold, with two straight edges `1e-10` wide meeting at three degrees, and two lying on each other for comparison. `curve_surface_overlaps` has no such test.

Note the shape of this fix. It adds no tolerance: it checks another condition an overlap has to meet, in the same three-valued arithmetic.

### Asking the right question about membership

After a boolean starts to split faces, one more trap appears. Splitting a face leaves both halves on one surface, so "this edge lies on this face's surface" no longer means it belongs to this face. An edge already imprinted into one half still lies on the surface of the half it does not belong to, and an earlier `find_coincident_pair` re-imprinted it, splitting again, forever. The fix is to ask the question about the *face's trim*, whether the edge lies inside it (`face_contains`, [Lecture 19](./19-classification.md)), and to first rule out the exact topological case, `edge_is_boundary_of_face`. AGENTS.md tells this story in "Make the invariant a type, not a convention".

This is the general form of the rule we keep meeting: *proximity is not membership*. When you select something because it is near or on, ask what property you are about to assume it has, and test that.

## Tangent branches

The subtlest configuration in this lecture arises when a revolve and an extrude are built from one sketch. The revolved surface at its seam and the extruded wall both contain the profile and the direction perpendicular to the sketch. So they touch tangentially all along the shared profile edge. Their intersection contains that edge, and it may also contain other curves that leave the edge transversally. Where such a branch leaves, an intersection curve ends, and so a vertex is needed. But the edge lies on the other surface throughout, so no piercing search can single out that point.

`core/geop-core-geometry` does not solve this; `lib/geop-lib-booleans/src/remesh/remesh_tangent_branches.rs` does, and its module doc has the derivation. Let `$s$` run along the edge `$E$` and `$h$` across it, in the common tangent plane. One surface stands off the other by a height `$f(s,h)$`. Tangency along `$E$` means that `$f$` and `$\partial f/\partial h$` both vanish at `$h=0$`. So `$f$` has a double factor:

`$$f(s,h)=h^2\,g(s,h).$$`

The surfaces meet where `$f=0$`, that is on `$\{h=0\}\cup\{g=0\}$`. The first set is `$E$` itself. The second is the other branches, and they meet `$E$` where

`$$g(s,0)=\tfrac12\frac{\partial^2f}{\partial h^2}(s,0)=0.$$`

The second derivative of the height difference across `$E$` is the difference of the two surfaces' normal curvatures across `$E$`. Along `$E$`'s own direction the normal curvatures agree, as both surfaces contain `$E$`, and they share `$E$`'s normal. So the points are those where the two surfaces' *mean curvatures* agree.

A geometric puzzle has turned into a scalar function along an edge, and a search finds its sign changes (`sign_changes` in `core/geop-core-math/src/solvers/bisect.rs`): subdivide along the edge to isolate them, then bisect. This is a fine example of what to do with a degeneracy. Do not widen a tolerance until the search stops failing; find the *exact equation that holds on the degenerate set*, factor out what you already know, and search the remainder.

A related search is in `core/geop-core-geometry/src/intersection/surface_surface.rs`. A boolean finds the curves in which two faces meet by tracing from where edges pierce faces ([Lecture 16](./16-tracing-and-pcurves.md)), and a closed loop lying inside both faces is pierced by no edge. So `turning_points` looks for points where the curve turns back along a chosen direction `$d$`, solving `$S_a=S_b$` together with `$(n_a\times n_b)\cdot d=0$`. Where the normals could be parallel, that equation is vacuous, so those pairs are set aside as `tangent`, and `refine_node` tells apart isolated nodes (where branches cross) from contacts along a curve. Its doc is exact about what remains: "a loop shrinking into a tangency is the one case left to the caller to notice, as a face the boolean cannot classify".

## Do not make the degenerate case yourself

So far, tangency arrived from outside. Often we cause it ourselves. AGENTS.md has a whole sequence of stories on this, and they generalize to one question: *did something our code chose put the entities there?*

### Name the entities before you name the cause

A revolve cut "up to next" failed with a vertex too wide to validate, and was explained to the user as a near-tangency. The user looked at the scene: there was none. Naming the vertex showed that it lay where the drilled hole's rim crossed the revolve's *end face*, and the end face was there because the operation's own fallback had stopped the tool flat at the first point of contact, an existing corner. The bug was a logic error in our own code, not numerics.

"Near-tangent crossing" sounds plausible and cannot be checked. Before you state a cause, resolve the failing vertex or edge to its entities by name (`names_mentioned`, used by the regression tests' `assert_builds_valid`), and test the explanation against them. [Lecture 42](./42-case-studies.md) has more of this.

### Never place a cut on a corner that already exists

The same story generalizes. When an operation chooses where a new face goes, landing it exactly on an existing vertex or edge asks the boolean to cross an edge at its own end point. That configuration is degenerate, and no enclosure stays tight through it. So choose positions the geometry actually defines. For up to next, that means going up to the next face from where the tool enters the target (or all the way round, or through all), rather than stopping flat at a sampled contact point.

### A tool's own curves must not lie in the solid's faces

A blend tool meets the solid where the boolean finds it. Any curve of the tool lying *in* a face of the solid, crossing that face's boundary, asks the boolean to resolve an overlap rather than a crossing, and it exhausts its search. AGENTS.md lists four instances, all from `geop-ops-fillet`. Here is the most instructive one. A straight chamfer built as a tool once used its chord's ends on the faces: exact lines lying in them. Extending the chord past the faces, as the swept chamfer always did, made it cross them instead. In other words, the tool was made to *cross* rather than *touch*, at the cost of nothing, because the extra part of the tool is outside the solid and is dropped.

That is a general trick. A boolean is robust on transversal crossings and fragile on contacts, and a tool is yours to shape. Where a patch ends is a free choice. And the last of the four instances adds a warning about how to make it: reaching a third of a patch's size past a section only moved its side onto a stepped block's chamfer contact, because "a fraction of one design dimension lands on another". The cap now reaches past the extent of every edge of the model (`FlatPatch::past`), where nothing can lie along it.

### An approximate tangency dips through the face

Here is the case with the most interesting cure. A surface only approximately tangent to a face, say a rolling-ball blend skinned through exact stations, is tangent at each and interpolated in between. It then crosses the face as often as not between the stations, by a hair. A boolean then has to cut the face along each such crossing: near-duplicate vertices next to the contact, "nothing to split" errors. As AGENTS.md puts it: tangency to within the approximation's error is no tangency to the boolean.

The cure, in `ops/geop-ops-fillet/src/rolling.rs`, is to leave each face at a millionth of a radian towards the ball (`TILT` there, equal to `DEVIATION`, `$10^{-6}$`), and to check between stations that it really leaves on that side. A tangency is thereby turned into a very shallow but definite crossing, along the contact curve alone. The boolean can do that, as Lecture 13's clip has a slope to divide by, if a small one.

And an exact tangency needs no tilt, and must not get one: AGENTS.md records that, tilted, a revolved handle's rim showed two silhouettes per face where the crease ran, and a draft no longer saw the fillet as tangent. Where the untilted skin meets the ball between stations to rounding (`span_is_exact`), it is kept untilted. [Lecture 29](./29-fillets.md) develops all this.

<div class="kernel">

**In the kernel.**

| What | Where |
|---|---|
| Overlaps from boundary candidates and midpoint probes | `find_overlaps`, `assemble`, `Overlap`, `Hit` in `core/geop-core-geometry/src/intersection/coincidence.rs` |
| The candidates for curves and for a curve and a surface | `curve_curve_overlaps` in `intersection/curve_curve.rs`, `curve_surface_overlaps` in `intersection/curve_surface.rs` |
| Unresolved clusters | `curve_curve_crossings`, with `DisjointSet` from `core/geop-core-math/src/disjoint_set/mod.rs` |
| Tangent branches leaving an edge | `remesh_tangent_branches` in `lib/geop-lib-booleans/src/remesh/remesh_tangent_branches.rs` |
| Closed loops, and nodes among tangent pairs | `turning_points`, `refine_node` in `core/geop-core-geometry/src/intersection/surface_surface.rs` |
| Coincident pairs while imprinting | `find_coincident_pair`, `edge_is_boundary_of_face` in `lib/geop-lib-booleans/src/remesh/remesh_edges_x_faces.rs` |

`curve_curve_overlaps_and_crossings` takes a `reach`, so that a caller working to a resolution of its own can count two curves that close as one: a drawing, to what a sheet shows ([Lecture 35](./35-step-and-drawings.md)). Without it, a long stretch of almost coincident curves would have to be covered with pairs of pieces no larger than `min_subdivision_size`, which no node budget does. Note the difference: `reach` is stated by the caller, as a fact about its purpose, and is not a tolerance hidden in the kernel.

</div>

## Recap

<div class="recap">

- A crossing's enclosure has width `$\varepsilon/\sqrt c$`, and at tangency `$2\sqrt\varepsilon$`: half the digits are gone, in the data. Near tangency the count of crossings has no definite answer.
- At a tangency the search leaves a wide cluster, clipping stalls, and Newton halves or gives up. All report an honest wide box.
- Counting solutions does not detect overlap. An overlap can begin and end only at a curve's or surface's boundary, so test the midpoints between those candidates.
- A degeneracy has an equation: a tangent branch leaves an edge where the mean curvatures agree.
- Ask first whether your own choice put the entities in a degenerate place. Make tools cross, not touch; and tilt what is only approximately tangent.

</div>

## Exercises

**1.** Derive the width `$\varepsilon/\sqrt c$` and the merging point. For `$\varepsilon=10^{-12}$`, at what `$c$` do the two enclosures of the demo merge? Check with the demo.

<details class="solution"><summary>Solution</summary>

`$\sqrt{c+\varepsilon}-\sqrt{c-\varepsilon}=\dfrac{2\varepsilon}{\sqrt{c+\varepsilon}+\sqrt{c-\varepsilon}}\approx\dfrac{\varepsilon}{\sqrt c}$`. They merge once `$c-\varepsilon\le0$`, that is at `$c=\varepsilon=10^{-12}$`. There the single enclosure is `$[-\sqrt{2\varepsilon},\sqrt{2\varepsilon}]$`, width `$2\sqrt2\times10^{-6}$`. The demo's slider in `$c$` is too coarse for this. Set `$\varepsilon$` to `$10^{-3}$` and `$c=0.001$` instead, where the same statement gives a merge, and a width of `$2\sqrt{2\times10^{-3}}\approx0.089$`. The third row of the table shows the same thing at `$10^{-4}$`.

</details>

**2.** The surfaces `$z=\tfrac12\kappa_1x^2$` and `$z=\tfrac12\kappa_2x^2$` touch along the `$y$` axis. Given data uncertainty `$\varepsilon$` in `$z$`, how far from the axis could the two be said to meet? What does a containment search on their pieces see, and what would make `refine_crossing` on a curve in one of them give up?

<details class="solution"><summary>Solution</summary>

`$\lvert\tfrac12(\kappa_1-\kappa_2)x^2\rvert\le2\varepsilon$`, so `$\lvert x\rvert\le2\sqrt{\varepsilon/\lvert\kappa_1-\kappa_2\rvert}$`. A strip of that half-width around the axis is the honest answer. A search sees boxes overlapping all along that strip, and without a topological fact has to treat the whole axis as candidates. A curve lying in one surface and crossing the axis has a tangent within the other's tangent plane at the axis, since the tangent planes agree there, so `$\det J=C'\cdot(S_u\times S_v)$` vanishes, and refinement leaves its box alone.

</details>

**3.** Show by example that the midpoint probe can be fooled: construct two curves that coincide at the two candidates and the midpoint, but are not the same between. Why is this not a practical problem for two lines? For a line and a conic?

<details class="solution"><summary>Solution</summary>

A line `$y=0$` on `$[0,1]$` and the cubic `$y=x(x-\tfrac12)(x-1)$`, sharing ends and the midpoint, is such a pair. Two lines that share two points are one line: no problem. A line meets a conic in at most two points, so three common points imply the conic contains the line and is degenerate. In general, by Bézout, two curves of degrees `$p$` and `$q$` with more than `$pq$` common points coincide. With degree 3 and 1 the bound is 3 points, which is exactly why the example needed a cubic.

</details>

**4.** Go through `$f(s,h)=h^2g(s,h)$` for two cylinders of radii `$a$` and `$b$`, axes parallel to `$y$`, touching along a line, one with radius changing linearly along `$s=y$`, so that `$b=b_0+\gamma y$`, with `$b_0=a-\gamma y_0$`. Where does a transversal branch leave the common line, approximately? (You can ignore that the second is then only a cone.)

<details class="solution"><summary>Solution</summary>

Cross-sections near the contact: heights `$h^2/2a$` and `$h^2/2b$`, so `$f\approx\tfrac12h^2\bigl(1/a-1/b(y)\bigr)$` and `$g(y,0)=\tfrac12\bigl(1/a-1/b(y)\bigr)$`. It vanishes at `$b(y)=a$`, that is `$y=y_0$`. There the two normal curvatures across the line agree, and a branch can leave at that point.

</details>

**5.** A colleague fixes a "nothing to split" error by moving a fillet's boundary 1e-9 off the face. Explain why that differs from a tilt, and give a configuration where it fails.

<details class="solution"><summary>Hint</summary>

A parallel offset by a constant leaves the surfaces at a gap of `$10^{-9}$` throughout or crossing wherever the approximation's error exceeds it: the crossing's angle is left to the noise. It also leaves the fillet not touching the face at all, or touching it in many places. A tilt has a definite direction at the contact, its size is checked against the approximation between stations (or proven, `strays`), and the crossing is along the contact curve alone. Where the approximation error exceeds `$10^{-9}$`, the offset still dips through the face.

</details>

**6.** Look at the tangent scene in Lecture 12's demo and the demo above. Both have square roots. What is the same and what differs?

<details class="solution"><summary>Hint</summary>

Both come from a separation that is quadratic near contact. In this lecture the resolution is the data's width `$\varepsilon$`, and no algorithm can do better. In Lecture 12 it is the search's handoff `$h$`, a tunable: a finer handoff narrows the cluster, down to the point where the piece size reaches the data's width.

</details>
