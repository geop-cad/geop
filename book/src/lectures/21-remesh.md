# Lecture 21: The remesh pipeline

<div class="goals">

**After this lecture you can**

- state what a remesh guarantees, and why a boolean becomes a table lookup once that holds;
- describe each phase of `remesh`, in order, and which earlier phases each relies on;
- explain why the phases run in increasing dimension;
- follow a real example entity by entity: a blind hole imprinted in a box, and a crossing bore traced through a shaft;
- explain tracing's start points, how a trace ends, how a second trace of the same curve is recognized, and what happens to closed curves no edge reaches;
- say what grows with the number of cuts and how the search avoids rescanning.

</div>

We now have every ingredient: exact curves and surfaces (Part II), intersection with proof (Part III), and a B-rep with editing operations (Lectures 17–20). A boolean operation is the kernel's central trial, because it needs all of it. The kernel's design splits it into two parts, with a sharp line between them: a **remesh** does all the hard geometry and leaves a very specific guarantee, and then the **boolean** proper is almost nothing. This lecture is the first part; [Lecture 22](./22-boolean.md) is the second.

## The contract

Take two bodies `$a$` and `$b$`. `remesh` in `lib/geop-lib-booleans/src/remesh/mod.rs` edits both so that "every coincidence between them (vertex/vertex, vertex/edge, edge/edge, edge/face) is resolved into shared topology, so that afterwards the two bodies meet only along entities they genuinely share".

That has a powerful consequence. Before a remesh, a face of `$a$` may pass in and out of `$b$`. After it, the face has been split along every intersection curve, so each piece is *wholly* inside `$b$`, *wholly* outside, or *wholly* on its boundary. No face straddles. From a boolean's point of view, classifying one point of a piece classifies all of it. That is the whole idea: reduce an infinite question to a finite one by cutting the input where the answer changes.

Remesh takes `impl Into<Body>`, so either body may be a sheet, which is what lets a split cut a solid with a sheet (`lib/geop-lib-booleans/src/split.rs`). It asks nothing of a body but its faces, edges and vertices. The same machinery also serves trim-up-to-next (`trim.rs`) and imprint (`imprint.rs`).

## Increasing dimension

The phases run in strict order: vertices, then edges against vertices, then edges against edges, then edges against faces. Why?

Each phase can then assume that everything of lower dimension has settled, which rules out cases. Take the simplest example, an edge passing exactly through a vertex of the other solid. If the vertices have not been merged yet, and two vertices of `$a$` and `$b$` sit together with a third edge passing through them, then "split this edge at a vertex" has two candidates. With vertices merged first, there is just one vertex there. The doc comment states the consequence: `remesh_vertices_x_edges` "treats a point-coincidence with a *different* vertex id as a bug, since `remesh_vertices` should already have merged them".

The principle is general: order the work so that, at each stage, a whole class of degenerate configurations has already been reduced to a simpler one. As a bonus, each phase's search only needs to consider its own kind of coincidence, and an unresolved one from an earlier phase is a bug and not a case.

Before going on, see the pipeline one dimension down. In the plane, two polygons have only vertices and edges, and the top of the pipeline has nothing to do. But the shape is the same.

<figure class="demo" data-demo="boolean-2d" data-args='{"mode":"remesh"}'>
<figcaption>Step through it. Drag vertices (they snap to a grid, so they can land exactly on each other and on edges): put a vertex of one polygon exactly on a vertex of the other to see a merge (green), on an edge for a split (purple); crossings are red. After the last step, every piece is on one side of the other polygon: try to make a piece that straddles, and you cannot. In the plane, an edge plays the part of a face, there is no tracing, and exact grid coordinates stand in for the kernel's interval comparisons.</figcaption>
</figure>

## Phase by phase

### 1. Vertices (`remesh_vertices`)

For each vertex of `$a$`, for each vertex of `$b$`: if their points `could_be_equal`, merge them (`lib/geop-lib-booleans/src/remesh/remesh_vertices.rs`, `merge_vertex` in `core/geop-core-topology/src/edit/merge_vertex.rs`). The survivor's point is the `union` of both: two enclosures of one point, combined honestly (AGENTS.md, *Combine two enclosures*). This phase creates nothing and so needs no names.

Note the search: it does not restart from scratch after each merge. `settled` remembers the vertices of `$a$` found to coincide with none. That stays true while merging goes on, since a merge deletes a vertex of `$b$` and grows only the vertex of `$a$` it merged into.

### Tangent branches: one exception (`remesh_tangent_branches`)

Before the next phase comes one question out of order. Suppose an edge `$E$` of one solid lies on a face `$F$` of the other, while a face `$G$` it bounds touches `$F$` *tangentially* all along it: a revolve and an extrude of one sketch share a profile like this. Then `$G\cap F$` contains `$E$`, and may contain other branches leaving `$E$` transversally. Where such a branch meets `$E$`, an intersection curve ends, so it needs a vertex. Nothing else puts one there: `$E$` lies on `$F$` throughout, so no piercing search can pick that point out.

The module comment derives it. With `$s$` along `$E$` and `$h$` across it in the common tangent plane, `$F$` stands off `$G$` by `$f(s,h)=h^2g(s,h)$`. Tangency along `$E$` is what makes both `$f$` and `$\partial f/\partial h$` vanish at `$h=0$`. So `$G\cap F=\{h=0\}\cup\{g=0\}$`, and the other branches meet `$E$` where `$g(s,0)$`, half the difference of the two surfaces' normal curvatures across `$E$`, is zero. Those are the points where the two surfaces' mean curvatures agree. So the phase finds zeros of a scalar function along the edge: a subdivision to isolate, then bisection on sign changes.

### 2. Vertices against edges (`remesh_vertices_x_edges`)

For each edge of one body, for each vertex of the other: is the vertex on the curve (`curve_could_contain`)? If it is, and is not already an end of the edge, split the edge there (`split_edge_at_vertex`, [Lecture 18](./18-poles-seams-splicing.md)). It is called twice, with the bodies swapped. "Vertex positions never change (splitting only refines topology), so running direction A then direction B can't reopen the other."

### 3. Edges against edges (`remesh_edges_x_edges`)

For each edge of `$a$` and each edge of `$b$`:

- If they already have the same two end vertices, they might be duplicates. Coincidence is detected first by *vertex identity*, as "cheap, exact, and (post vertex/edge-vertex remesh) authoritative". Duplicates are merged (`merge_edge`).
- Otherwise run `curve_curve_intersect` ([Lectures 12](./12-subdivide-to-isolate.md) and [13](./13-fat-lines.md)), and split each edge at each crossing. A vertex is created there, or reused if one is already at that point, so that three edges meeting at one point share the one the first pair created (`find_vertex_at_point`). That search is limited to the two bodies' own vertices: "a vertex of another body, or of a wire, that happens to lie there is near, not theirs."

Here `RemeshParams::curve_curve_min_subdivision_size` is used, deliberately looser than for single-shape containment, for the reason we saw in `ValidationParameters`: a pairwise search needs it. It is only the handoff to Newton (`refine_curve_curve_crossing`), not an accuracy target ([Lecture 14](./14-newton.md)).

### 4. Edges against faces (`remesh_edges_x_faces`)

All of the hard work is here, in six sub-steps, in the order of `remesh_edges_x_faces` in `lib/geop-lib-booleans/src/remesh/remesh_edges_x_faces.rs`.

**(a) Split at piercings: `split_piercing_crossings`.** For each edge of one body against each face of the other, in both directions, run to a fixed point. `curve_surface_intersect` finds where the edge crosses the face's surface, and the edge is split there at a new vertex, or at one already at that point. Three things are not a piercing. A coincidence belongs to the next sub-step. A crossing at the edge's own end is a shared corner. And a crossing where the curve runs along the surface's tangent plane is a touch: `find_piercing_crossing` asks that over the box the search proved, not at Newton's answer, because at a tangential contact Newton "stops beside the contact". (Two spheres touching at a point once had each meridian through the contact "pierce" the other sphere `1e-8` from it.) A curve tangent at an inflection really does cross, and `odd_crossing` keeps that case, by finding a bracket on whose ends the curve lies definitely on different sides.

Note that this search is against the *untrimmed* surface: `find_piercing_crossing` has no trim test. The edge may gain a vertex where it crosses the surface past the face's trim. That costs a vertex and changes no answer, since whether a curve lies in the face is asked at the tracing step (`face_contains`, in `trace_one_side`).

**(b) Imprint edges lying in faces: `imprint_coincident_pairs`.** An edge can lie entirely within a face of the other body: the search returns `Intersections::Coincident` ([Lecture 15](./15-tangency-and-overlap.md)). Then `splice_edge_into_face` makes it part of that face's boundary. This is where AGENTS.md's infinite loop lived: after a split, both halves share a surface, so each imprinted edge is on the surface of the half it does not belong to. So `find_coincident_pair` asks about the trim (`face_contains`): `Inside` only, judged on all of the crossings the search returned, not on one midpoint, since "an edge can run along this face's boundary for part of its length and cut through the interior elsewhere".

One subtle case: `seam_plane` and `faces_are_coplanar` deal with an edge between two coplanar patches, a seam internal to one plane, which carries nothing for a face flush with it but everything for a face that crosses that plane.

**(c) Find start points: `find_tracing_start_points`.** Now, with splits and imprints settled, scan for every vertex of one body lying on a face of the other on which it has no boundary. This is a fresh scan of the settled state, "not something threaded through step 1's own mutations: a start point recorded mid-loop could be silently invalidated by a *later* split further along the very same edge."

**(d) Nodes.** Where two branches cross with the faces tangent, inside both faces, no corrector can march through (the Jacobian is singular there). `insert_face_points` with `FacePoint::Node` puts a vertex at each such node, found by a subdivision search over each pair of surfaces (`core/geop-core-geometry/src/intersection/surface_surface.rs`), so that traces arriving there have somewhere to stop.

**(e) Trace.** From each start point, `trace_from_start_point` tries each pair of faces at the vertex, one from each body, and `trace_one_side` marches the intersection curve of those two faces ([Lecture 16](./16-tracing-and-pcurves.md)) until it reaches a vertex. The curve is then fitted as a cubic through the march (`fit_branch`) and becomes a new edge. Its name records the faces and the vertices at its ends. It is spliced into **both** faces.

**(f) Closed loops no edge leads to.** Two surfaces may meet in a closed curve that no edge pierces, like a sphere poking through the middle of a plate's top face. No start point leads to it. So a vertex is put on each of their turning points along a direction, by `insert_face_points` with `FacePoint::Turning`, and the loops are traced between them as above.

Having described the sub-steps, here is the logic that ties them together, as a pair of invariants:

- **One scan, one pass.** Tracing creates no new start points: "a traced curve runs between two vertices that are already here, and splitting a face along it moves the trim boundaries around without moving any surface, so nothing that was not already a crossing becomes one." An older version re-scanned until no more work was found, and only found more because tracing had skipped curves it should have drawn. Keeping the pass single "is what keeps the omission visible instead of papered over". AGENTS.md's maxim: fix the underlying logic, do not iterate until the symptom stops.
- **A curve either fully duplicates an edge or does not touch it.** Since a vertex has been put at every coincidence before tracing starts, no curve can *partially* overlap an edge. That is what enables the exact test of the next section. It has one more condition: a trace must stop at every vertex on its curve. `trace_one_side` once chose its first direction by a trial step, and the full step landed past a near vertex and was accepted, and so spliced a curve that partially duplicated an edge. "A step must never decide a direction past a vertex known to lie on the curve."

### How a trace ends

A traced curve ends at a vertex, chosen by `candidate_within`. That is the most-quoted bug in this part. It once picked "the nearest vertex within one marching stride". But a traced curve lies on both of the surfaces, so its end must too. Now the candidate must satisfy `surface_could_contain` on both surfaces, and also lie on the stretch the stride runs along: "one definitely behind `point` is not it", as on the scene `narrow_groove`, where a trace found a vertex behind its start.

Every vertex of the two bodies is a candidate, not only the start points. A curve can equally end at a corner where the two solids' edges were merged, which "never becomes a piercing point".

### Every curve is traced twice

An intersection curve has two ends, and both are piercing points. So both are start points, and each curve would be traced from each end. A second trace would splice a duplicate edge between the same two vertices. That splits nothing: it runs along the boundary the first splice just created, carving a zero-area sliver.

One tempting fix is to sample the finished curve and require some point of it to be strictly inside the face. AGENTS.md records that it was tried, worked, and was wrong: "a sampling heuristic standing in for a question with an exact answer", costing a `face_contains` per sample, and papering over the duplicate rather than recognising it. The right question is *is this curve already represented here?*, and the vertices nearly answer it: look for an existing edge that joins the same two vertices.

Make sure you see why that can be enough. It relies on the second invariant above, which relies on the phases run before it. Without them, a curve might overlap an edge between other vertices, and this test would find nothing.

The code at the end of `trace_one_side` adds one more condition. Two different branches can share both ends, an arc and its complement. So it also requires the existing edge to contain the middle point of the march, by `curve_could_contain`: "both ends and an interior point together pin the branch down". That is asked after marching, since only the finished curve can answer it, which costs a march in the duplicate case. A duplicate found this way is used instead of a new edge. Being one point on an already identified curve, not a sample deciding membership, it is a different thing from the rejected test.

## Worked example 1: a blind hole, imprinted

The example `box_with_drill_hole` extrudes a 2 by 2 square by 1, then cuts it with a tool: a circle of radius 0.4 on the top face, extruded down by 0.5. So the tool is a cylinder from `$z=1$` to `$z=0.5$` with its start cap in the plane of the box's top.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"box_with_drill_hole","example":"box_with_drill_hole"}'>
<figcaption>A 2 x 2 x 1 box with a blind hole of radius 0.4. This mesh, and the names below, are the real kernel's.</figcaption>
</figure>

Walk through what happens. The names are from running the example (`geop` 0.12.0, `list_elements`).

1. **Vertices.** None of the 8 box corners coincides with any of the 8 points of the tool (`$z=1$` and `$z=0.5$`, at `$(1\pm0.4,1)$` and `$(1,1\pm0.4)$`). Nothing to merge.
2. **Vertices against edges.** None lies on an edge of the other.
3. **Edges against edges.** The rim arcs and the box's top edges do not meet.
4. **Edges against faces.**
   - *Piercing:* do any cross transversally? The tool's vertical seam lines start in the plane of the box's top, at their end points, already vertices, and go down into the box: an endpoint touch, no interior piercing.
   - *Imprint:* the tool's four rim arcs, `extrude(hole,hole_sketch,c1,start)` and its three companions, lie *in* the box's top face `extrude(box,end)`. `curve_surface_intersect` says `Coincident`. `face_contains` says their points are inside the trim. So `imprint_coincident_edge` splices each into the top. The cases of [Lecture 18](./18-poles-seams-splicing.md) now happen one after another. An arc with no end on the face yet makes a ring of its own. An arc with one end on a ring is a spur on it. An arc joining two rings merges them. And the arc that closes the circle has both ends on one hole: the circle's inside, a patch of material, becomes a new face.
   - *Trace:* the rim vertices lie on the top's surface and so may be listed as start points. But where the walls meet the top is along the rim arcs, already edges. A trace finds its duplicate, and no new edge results.

After the remesh the box's top is in two pieces: a square with a circular hole, and a disc. No curve was traced here at all. This is the common situation with features started on a face, and it is the reason why imprinting exists as a phase of its own.

After [Lecture 22](./22-boolean.md)'s classification, 11 faces remain. The box's 6, with the top as the ring. The tool's 4 walls and its floor `extrude(hole,end)`, reversed. The disc and the tool's start cap are dropped. That makes 16 vertices, 24 edges and 11 faces. Volume `$4-\pi(0.4)^2(0.5)\approx3.7487$`; area `$16+2\pi(0.4)(0.5)\approx17.2566$`, as the `inspect` tool reports (`17.256637`).

## Worked example 2: a crossing bore, traced

The example `cross_drilled_shaft` revolves a stepped section (radius 1 up to `$z=1$`, radius 0.6 up to `$z=3$`), then subtracts a bore of radius 0.25 at `$z=2.2$`, running along `$y$` through both sides. Here no edge of either lies in a face of the other, and the walls cross in curves.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"cross_drilled_shaft","example":"cross_drilled_shaft"}'>
<figcaption>A stepped shaft with a bore straight through it.</figcaption>
</figure>

The bore's wall is four quarter faces, with four lateral seam lines parallel to `$y$`: at `$x=\pm0.25,z=2.2$`, and at `$x=0,z=2.2\pm0.25$`. The shaft's upper wall is also four quarters, whose seams are the vertical lines at `$(0,\pm0.6)$` and `$(\pm0.6,0)$`. Now the phases:

- **Edges against edges.** The bore's seams at `$x=0$` run through `$(0,\pm0.6,z)$`, and so cross the shaft's seams there: at `$(0,0.6,2.45)$`, `$(0,0.6,1.95)$`, `$(0,-0.6,2.45)$`, `$(0,-0.6,1.95)$`. These four vertices are made here. The kernel names one `combine(bore,extrude(bore,bore_sketch,c1@3),revolve(shaft,section,c9,a0),0,1)`: the first of one crossings of those two edges. The name pattern is `$N(E_1,E_2,i,n)$`.
- **Piercing.** The other two seams, at `$x=\pm0.25$`, pierce the shaft's wall faces, at `$y=\pm\sqrt{0.6^2-0.25^2}=\pm\sqrt{0.2975}\approx\pm0.545$`. This makes four vertices, such as `combine(bore,extrude(bore,bore_sketch,c1@0),revolve(shaft,section,c9,q3),0,1)` at `$(0.25,0.545,2.2)$`. The pattern is `$N(E,F,i,n)$`.
- **Tracing.** The eight new vertices are the start points. From each, the march follows where a bore quarter meets a shaft quarter, reaching the next vertex around the rim: 8 arcs, four around each end of the bore. Each edge is named after its two faces and two end vertices: `$N(F_1,F_2,P,Q)$`.

The total: the shaft's 18 vertices plus 8 makes 26. The result has 50 edges and 24 faces. `$26-50+24=0$`: one shell, genus 1, and no hole loops.

Check the first number by the geometry: the rim's height at a bore seam at `$x=\pm0.25$` is 2.2, and at a seam at `$x=0$` is `$2.2\pm0.25$`, as the vertices `$(0,0.6,2.45)$` report. The cross-check agrees with the kernel's output.

Notice what kind of work was involved: this model has a line crossing a line, a line piercing a surface and a curve of two cylinders, in all, and no special case. No pair of surface kinds is special-cased: a plane crossing a cylinder and two cylinders crossing go through the one path. See AGENTS.md's *Everything is a NURBS curve or surface*.

## What grows with every cut

A spur gear is a disc with a gap cut per tooth, twenty to a hundred booleans on one body. AGENTS.md's *Many booleans on one body* lists what was harmless once and grew. Two of these are here.

**Rescans.** The piercing search started over from every edge-by-face pair after each split, which made a boolean of many crossings quadratic in them. The fix is an invariant: "a piece of a curve crosses nothing the whole did not." So once a pair is found clean, it stays clean, and `settled` remembers it. Similarly, imprinting (`find_coincident_pair`) and the vertex phase. Note how each of these "settled" sets has a *proof* in its doc comment for why it stays correct. It is a cache whose invariant is stated.

**Width along a split edge.** The threefold growth per cut from [Lecture 18](./18-poles-seams-splicing.md). "A disc's rim cut by thirty-two slots ended `4e-10` below its cap, and gears stopped at 80 teeth, their rims drawn in twelve arcs to spread the cuts." With cuts at sharp parameters, the rim is a circle again.

The remaining two (names, traced fits) are in the next lecture and [Lecture 16](./16-tracing-and-pcurves.md).

<div class="kernel">

**In the kernel.** `RemeshParams` bundles every tunable: `max_edge_intersections`, `max_nodes`, `min_subdivision_size`, `curve_curve_min_subdivision_size`, `max_trace_steps`, `max_rewrites`. "Every field only affects *how hard a search tries*, never what a converged answer means." A stage that runs out of `max_rewrites` is an error, never taken as settled (`geop_core_math::solvers::repeat`).

`imprint` in `lib/geop-lib-booleans/src/imprint.rs` is the same remesh with no faces classified or dropped: it divides each body's faces where the other's boundary crosses them. `imprint_on_face` divides one face only, by remeshing against a copy of it. `section.rs` is a read-only variant: the curves where two bodies cross, as a wire.

The scenes in `lib/geop-lib-booleans/src/scenes.rs` are a grid of offsets of two identical solids and a hand-picked set; `remesh/render_test.rs` renders each stage. `lib/geop-lib-booleans/src/regression_tests.rs` holds rebuilt reproducers.

The phases of this pipeline depend on the intersection searches of `core/geop-core-geometry/src/intersection/`. Those are being changed in parallel with this book, so read their current doc comments for the details of how a crossing is found.

</div>

<div class="recap">

## Recap

- After a remesh, no face straddles the other body: every coincidence has become shared topology.
- Phases run in increasing dimension, each assuming the last has settled, so what an earlier phase should have handled is a bug and not a case.
- Vertices are merged, edges split at vertices and at crossings, then edges split at piercings and imprinted into faces they lie in; a fresh scan gives start points; curves are traced from them, and closed loops from their turning points.
- A trace ends only at a vertex lying on both surfaces. A second trace of one curve is recognized topologically, using the invariant that no curve partially overlaps an edge.
- Every "settled" cache has a proof that it stays true, and rewrites are bounded by a budget that errs.

</div>

## Exercises

**1.** Why is `remesh_vertices_x_edges` called in both directions, while `remesh_edges_x_edges` is called once?

<details class="solution"><summary>Solution</summary>

The vertex-on-edge test takes a body's edges and the other body's vertices, so one direction covers only `$a$`'s edges against `$b$`'s vertices. The edge-edge test takes every `$a$` edge and every `$b$` edge in one nested loop, splitting both sides, so one call covers the symmetric pairs. The first phase's second direction cannot reopen the first, as positions do not move.

</details>

**2.** In the drilled box, what would go wrong if the hole's tool started slightly above the top face (at `$z=1.1$`) rather than in it? Trace the phases.

<details class="solution"><summary>Solution</summary>

No rim arc lies in the top now, and no imprint happens. Instead, the tool's seam lines, passing through `$z=1$`, pierce the top: four vertices, by `split_piercing_crossings`. From each, tracing follows the top's plane against a wall quarter to the next vertex, four arcs, fitted cubics (rational arcs may be used where they fit; see `MAX_WIDTH`'s comment on the "boolean's exact arc"). The result is the same shape with different entities: the rim traced, not imprinted, and the top cut by traced edges. The tool's cap, above the box, is classified outside and dropped.

</details>

**3.** In the second example, what if the bore were rotated slightly about `$z$`, so that no seam of the bore meets a seam of the shaft? How many start points, what names, and how would `$V-E+F$` change?

<details class="solution"><summary>Solution</summary>

Then all eight new vertices are piercings, named `$N(E,F,i,n)$`, with seams of the bore piercing the shaft's wall in pairs, four seams times two ends. But the shaft's own seams at `$(0,\pm0.6)$` now pass through the bore's wall, where the vertical line pierces it: two vertices per shaft seam that the bore reaches, so more vertices. The count `$V-E+F-L=0$` still holds, since the genus has not changed, whatever the split into pieces. So a rotation changes the numbers and not their alternating sum; `$L$` may become nonzero, if a shaft quarter now has a whole bite inside it with its rim not crossing any seam. That depends on the angle. Try it in the web editor and read `list_elements`.

</details>

**4.** Explain in your own words why a second trace can be recognized by its end vertices and one interior point, and construct a configuration in which that would be wrong if the invariant failed.

<details class="solution"><summary>Solution</summary>

If a curve and an edge overlapped partially, the curve might join two vertices *different* from the edge's ends, with no edge joining those two, while still running along it: the topological test finds nothing, and the sliver appears. Conversely, two genuinely different curves can join the same two vertices (an arc and its complement), which is why names get a trailing `,k` and why the test also asks about one interior point. With every coincidence given a vertex, partial overlap cannot occur.

</details>

**5.** Trace why `settled` in `find_piercing_crossing` stays correct across a split. What would break if the split made a new edge's id unrelated to any settled pair? What if the operation moved a vertex?

<details class="solution"><summary>Solution</summary>

A pair `(edge, face)` is settled after finding no piercing. A split keeps the edge's id for the first piece; since a piece of a curve crosses nothing the whole did not, that pair stays clean. The new piece's id has no settled entry and is asked afresh, which costs a little and can never be wrong. Face splits keep one piece's id; a piece's points are a subset of the whole. So every step shrinks point sets, which is why the proof works. If an operation moved geometry (a vertex's position, or a widening curve), a settled pair could become wrong. Remesh never moves anything: "Vertex positions never change." `merge_vertex` unites enclosures, but that happens in an earlier phase, and the vertex phase's `settled` set has its own argument.

</details>
