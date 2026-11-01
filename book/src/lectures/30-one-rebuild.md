# Lecture 30: One rebuild: shell, draft, moving faces

<div class="goals">

**After this lecture you can**

- recognize that draft, shell, face move, offset, replace, delete-and-heal and knit are one problem;
- state the rebuild's algorithm: room, vertices, edges, pcurves, and what it checks at each stage;
- move a vertex to where its surfaces meet, and explain the condition groups a smooth surface in pieces must form;
- say when a rebuild must refuse, and why refusal is by name;
- explain why carrying a patch on must leave it as it was, with the kernel's numbers;
- say why a shell at an inward corner is answered by a boolean.

</div>

So far, operations have made new solids from profiles or tools. Now consider operations that take an existing solid and change it without changing what touches what. Tilt a side of a box by 3° so it comes out of a mould. Push a face out by 5 mm. Hollow a part into walls. In each case some faces get new surfaces, and everything else must follow: the vertices have to move to where the surfaces now meet, and the edges have to run along their intersections.

A kernel's traditional structure is an operation per feature, each with its own treatment of each surface pair. Geop has one function, and it is worth understanding why it can be one.

## One problem, several operations

Look at what these share. Each changes "the surface a face lies on", and each keeps (or, when deleting, first edits) the topology.

| Operation | New surfaces | Code |
|---|---|---|
| Draft | a plane tilted about its neutral line | `draft` in `ops/geop-ops-plastic/src/draft.rs` |
| Move face | the surface translated | `MoveFace` in `ops/geop-ops-edit/src/direct.rs` |
| Offset face (push-pull) | the surface's offset | `OffsetFace` |
| Replace face | another face's surface | `ReplaceFace` |
| Delete face, healed | none: neighbours grow over the gap | `delete_faces`, `Heal::Extend`, `DeleteFace` |
| Shell | every kept face offset inward | `shell` in `lib/geop-lib-shell/src/shell.rs` |
| Knit across a gap | vertices merged, then surfaces' meetings re-found | `Knit` in `ops/geop-ops-surface/src/knit.rs` |

All call into `rebuilt` in `lib/geop-lib-shell/src/rebuild.rs`. Its signature tells the whole story:

```rust,ignore
pub fn rebuilt(
    spec: &BodySpec,      // the body, with the new surfaces on the faces marked
    names: &BodyNames,    // what each entity is called, for errors
    replaced: &[bool],    // which faces lie on new surfaces
    reseat: &[bool],      // which vertices to move even if no replaced face reaches them
) -> GeopResult<BodySpec>
```

A `BodySpec` goes in and a `BodySpec` comes out. There is no branch on surface kinds. Lecture 1's rule that everything is a NURBS is what makes it possible: a shell used to have a lines-and-arcs-only offset, and when it went, so did the refusal of every traced edge a boolean or fillet left behind. AGENTS.md's summary of this is *One rebuild, and what a face move asks of it*.

And why not a boolean? Could a draft cut the solid with a wedge? The doc comment in `draft.rs` gives the reason: it "would have to put the wedge's tilted face exactly through the face's edge on the neutral plane — a cut along an edge that already exists, the one configuration a boolean cannot stay tight through — and a wedge beside an inward corner would cut into the neighbour." Rebuilding the geometry avoids both, and keeps every name.

## A picture in the plane

Here is the algorithm in a world simple enough to see, a polygon whose sides lie on lines.

<figure class="demo" data-demo="face-move">
<figcaption>A 2-D analogy, and only that: faces become sides, surfaces become lines, and the rebuild's vertices are intersections of two lines. Choose a side and move or tilt its line (orange). Every corner goes where its two lines now meet, and each side is checked to run the way it ran. Drag "distance" past where a neighbouring side collapses: the rebuild refuses, naming the side, in the wording of the kernel's refusal. "Inner copy of all" is what a shell asks of a solid.</figcaption>
</figure>

Play until you have seen these:

- moving a side changes only its two corners;
- at some distance, a neighbouring side shrinks to a point, and beyond that distance the neighbour would run backwards. That is **refused**;
- with "inner copy of all" and the *chamfered* shape, the chamfer collapses before anything else does;
- tilting a side until it is parallel to a neighbour breaks a corner: those two lines no longer meet.

In 3-D, there are more than two surfaces at a vertex, edges are curves, and nothing is straight. But the four stages below are this same picture.

## Stage 1: room

A face's boundary may move past where its patch ends. A plane that was cut off by the box around a profile, or a cylinder's quarter, needs to exist beyond its trim, or its neighbour's new edge has no surface to run on. So every surface touched gets **room**: `room` in `rebuild.rs` carries it on past its sides. How much is a free choice, for "the patch's own size is room enough unless a face moves as far as it is wide". A straight direction is carried on from the rows it has, halved as often as needed. A curved direction goes by as much as the end span is long, because carried further, "a short end span of a curved patch extrapolates far, its control points running off by the cube of how much further". And not past a pole, which would end up inside the patch, where it has no normal.

### Carrying a patch on must leave it as it was

Here is a measurement-driven piece of design, from AGENTS.md. `NurbCurve::extended` used to replace the end piece by the piece over the longer span, recomputing all its control points by extrapolation. A sphere's octant carried on then evaluated its *old* region four orders of magnitude wider, and a shell's sphere vertex went from `1e-15` to `7e-10` wide.

The first reaction might be to give curved surfaces less room. The fix was instead to keep the curve as it was, control point for control point, and join the continuation on as a piece of its own at a knot of full multiplicity (`core/geop-core-geometry/src/nurb_curve/extend.rs`). Room is then free: where the face stays, nothing changed. The join is a knot of full multiplicity, so by its knots the curve is only `$C^0$` there, but geometrically the continuation is the same polynomial, carried on.

The general lesson: any operation that only needs *more* of a curve or surface should not touch the part it already had.

<div class="kernel">

**In the kernel.** `rebuilt`'s doc comment records a subtlety about the order. New surfaces "made from the old ones given room first": "a fitted offset given room after, its end span of many carried on, is extrapolated and widened, and an exact one given room before and after is widened twice".

</div>

## Stage 2: vertices

A vertex moves to the point nearest where it was that lies on the smooth surfaces around it. That is `meet` from Lecture 29, with no plane and with offset conditions of zero or the shell's thickness:

`$$n_k\cdot(X-\mathrm{foot}_k(X))=d_k\quad\text{for each smooth surface }k.$$`

Each iteration computes the residual of every condition, and takes the *shortest* step that corrects them all to first order (`shortest_solution`). A step's rows are the normals at the foot points.

**A worked example.** A box `$[0,2]\times[0,1]\times[0,1]$`, whose top is moved to `$z=1.5$`. The vertex `$(0,0,1)$` sits on planes `$x=0$`, `$y=0$`, `$z=1.5$`, with residuals `$0,0,-0.5$` measured along outward normals. The linear system is square and nonsingular: the step is `$(0,0,+0.5)$`, and one iteration lands on `$(0,0,1.5)$` exactly. With fewer than three conditions the system is underdetermined, and the shortest step keeps it as near as possible: "the point slides along where they meet, by no more than it has to". The rebuild falls back on that only for a vertex on a sheet's free border. A vertex at the end of a seam between two pieces of one smooth surface stays on the seam: it goes where the seam's curve, carried on, crosses the other surface (`on_seam`).

### One smooth surface, one condition

Here is a bug worth understanding, for the fix is general. Two quarters of one sphere meet at its pole. They are one smooth surface, hence one condition. The shell compared normals at the foot points, and those differ a little because each patch clamps its foot to its own domain. So the quarters counted as two nearly equal conditions, and the system went singular. The fix is `tangent_groups` in `core/geop-core-geometry/src/meet.rs`: decide once, *at the vertex*, which pieces form a smooth surface: those whose normals there could be parallel. Each group is one `Offset` with several `pieces`, and its foot point is the nearest over the union of the pieces. "Decided once, at the point, not at the foot points a search moves through."

### Check the answer against every condition

AGENTS.md again: "An iteration that stops is not an iteration that converged." `meet` returned a hemisphere's pole at `$(0,0.9,0.1)$`: on the offset plane but off the offset sphere. The failure surfaced two steps later, as "not on the circle". There were two silent causes, a dropped condition and a projection that never moved.

The second cause is itself a story. `NurbSurface::project` skipped any Newton step whose Jacobian could be singular. From a seed on a pole, every step is singular, so it returned the pole for every target. In truth only the collapsed parameter stops moving the point. The other one still moves it, so step along that one, down a meridian. Which meridian is a free choice, since at the pole the collapsed parameter says nothing: take the one that heads most towards the target and leads into the domain. That is `off_pole` in `core/geop-core-geometry/src/nurb_surface/project.rs`.

Checking the returned point against every surface it was asked to lie on turned that bug into an error at the source, naming the surface missed, with the numbers. It is now in `meet`, for good: "a condition lost on the way — a foot point clamped to the edge of its patch, a projection that never moved — is an error here, naming the condition, not a wrong point found two steps later". It costs one projection per condition.

## Stage 3: edges

An edge between two smooth surfaces is **traced where they meet between its moved vertices, following the edge it was**. For a sequence of points along the old edge, each new point is found as the meeting of the two surfaces and a third condition: the plane square to the old edge there. The old point shifted with its vertices seeds it. Note the kinship to `meet` and to Lecture 16: the old curve is a good predictor, and following it keeps the trace on the branch the old edge was on.

The result is a cubic through those points, widened to enclose the curve between them (`interpolate_enclosing`, Lecture 16). The legs start at `FIRST_LEGS` (8) and are doubled, up to `MOST_LEGS` (256), while the fit's enclosure is wider than `TRACE_DRIFT` (`1e-5`). This is "how fast the fit converges, not what it means". The sample count follows the width it produces.

Two cases are simpler:

- A straight edge that stays on both surfaces is **kept straight**. Keeping it straight is cheaper and more precise.
- An edge within one smooth surface, a seam, is its old curve carried on or cut back to its moved ends.

But "straight stays straight" is subtler in `$(u,v)$`. A pcurve may be kept straight only along an iso-line or on a plane. A sphere's meridian cut by a plane has an inner copy that is a small circle. `iso_line` reads a straight pcurve whose ends share one parameter value as a row of the surface, exactly; any other straight pcurve "is only as good as the surface's parametrization is affine". We return to this in [Lecture 31](./31-offsets.md).

## Stage 4: pcurves

Every coedge that changed is fitted anew on its face's surface, between `$(u,v)$` worked out *once per junction*, so that the coedges meeting there agree on it exactly. Finally, every curve built is checked to lie on the surfaces of its faces (`surface_missed`).

## When to refuse

An edge that turns around is the 3-D form of what you did in the demo: a face moved so far that a neighbour vanishes, or two of them now cross. `turns_around` tests it along the traced points: each step between consecutive points must definitely run the way the old edge's tangent runs. If not, the refusal names the entity:

> moved this far, it turns around between ... and ...: a face it bounds would vanish, or two would cross

(and the rebuild's context supplies the edge's name). The same check covers planes and fitted blends alike, and it is a topological question, "does this edge still run the way it did?", answered by a direction.

Why is refusal so central? Because without it, what comes out is an invalid solid that nobody notices until a downstream boolean fails. AGENTS.md's *Refuse what you do not support: early, by name, and say why* tells how the shell failed "in five different ways" before `inward_corner` recognized the case up front, naming the faces and the edge. A refusal names the entities and the condition, and says what is supported, "so the user can change the model and the next session can extend the code".

## Shell: the rebuild on every face

A shell hollows a solid into walls. As the module doc of `shell.rs` says, it is "built directly, not cut out by a boolean": the walls' inner side is the solid's own boundary moved inward, face for face, so it has exactly the solid's topology. Every face kept is offset inward (`NurbSurface3D::offset`, [Lecture 31](./31-offsets.md)), and every face taken away stays where it is, because its inner copy is what the opening leaves of it. Then `rebuilt` does the rest. Without an opening, the inner side is a void of its own, a second shell. With one, what is left of an open face `$F$` is `$F$` without its inner copy `$F'$`: a rim, the walls' cut edge.

There are further cases, each named in that module's doc. A face whose offset collapses, say a blend narrower than the walls are thick, has no inner copy, and the solid is **contracted** before being rebuilt (`Contracted`, `collapse_edges`): a collapsed blend's edges along the faces it rounds share one copy, a sharp inner corner. A chamfer narrower than the walls has an offset, but the offsets of the faces it cuts meet before it, so its edges across it turn around. Such an edge vanishes, and a face left with two edges vanishes too. The "chamfered" polygon in the demo shows this shape of failure, where the demo refuses and the kernel handles the case.

Thicken (`thicken`) is the same on a sheet: its faces are the outer side, their inner copies the inner one, and along every free edge a wall joins the edge to its inner copy.

### Where the topology changes, reuse the boolean

A face taken away at an inward corner, such as a pocket's floor, a step, or the face a boss stands on, is a different thing. Its inner copy reaches past the face: the wall kept there is thickened away from it, and stands on its surface beyond its edge. What is left on that surface is the face less its copy (the rim), and the copy less the face (the underside of the wall, facing the other way): crossings, edges split across faces, two loop sets.

That was designed as new machinery, and never built. Then the realization: the solid less its inner side, closed into a solid, *is* that thing. The boolean already finds it, coplanar faces and all. So `shell` closes the inner side into a solid and calls `boolean` with `BooleanOp::Difference` (`subtracted`). AGENTS.md's question to keep in mind: "Before writing a construction that changes topology, ask whether an operation the kernel already has computes it."

That is why the rebuild itself says, in its doc, "The topology never changes here."

## Delete and heal

`delete_faces` with `Heal::Extend` removes faces and grows the neighbours over the gap. It has two cases, decided by topology:

- **A hole closed.** Faces that fill holes of their neighbours (a boss, a hole, a pocket) are bounded by whole hole loops. Remove them and the holes, and the neighbours are otherwise unchanged. Nothing moves.
- **Faces grown over.** Faces between others (a chamfer, a fillet, a cut-off corner) leave a gap bounded by sides on neighbours' loops. Each three neighbours meet at a corner, each two sharing two corners along a new edge, and that amounts to a triangulation of the gap's polygon of sides. A gap of three sides leaves a corner, four an edge across, and more a tree of edges.

Now, which triangulation? This is decided "not by kind but by trying every way, from the first the rebuild accepts: the others fail where they would make faces meet that never do, as a box's two ends, or edges that turn around". The rebuild's refusal is thus doing double duty as the *selection* criterion. A trustworthy refusal is what makes that possible.

## Tested by sweeping

A hand-written test proved little here. `ops/geop-ops-shell/src/stress_tests.rs` shells a sweep of bodies: `block`, `block_drilled`, `block_stepped`, `cylinder_with_boss`, `sphere_with_side_cut_off`, `pipe_tee`, and so on, closed, and open at each face in turn. AGENTS.md records that the first run of those stress tests "found five shell bugs that no hand-written test had reached", and draws the conclusion that a sweep must tell a refusal apart from a failure, and require that whatever is built is valid.

One more story from *Test what the user does*: the shell's faces could never be picked in the editor. `EntityRef::lies_in` fell back to `self == scope` for every case it did not list, and a face never equals its solid. The shell tests set the face names directly. So what was truly untested was the road a user travels.

To try a real rebuild, open [`rounded_box` in the editor](https://app.geop-cad.dev/?example=rounded_box), add a face move or a shell as a new step, and drag it until it refuses.

<div class="recap">

## Recap

- Draft, face move, offset, replace, delete-and-heal, shell and knit all change surfaces and keep topology: one function, `rebuilt`.
- Stages: room around every surface touched; vertices by `meet`, one condition per smooth surface; edges traced following the old edge; pcurves fitted from one `$(u,v)$` per junction.
- Carrying a surface on must leave its old part bit for bit: join a piece on.
- A vertex's answer is checked against every condition before it is returned.
- An edge that no longer runs the way it did is refused by name. A trustworthy refusal also selects among healing options.
- Where topology changes, ask whether a boolean already computes it: at an inward corner, it does.

</div>

## Exercises

**1.** A box `$[0,4]\times[0,3]\times[0,2]$`. The face `$x=4$` moves out by `$1$`. Which vertices move, where to, and which edges change length?

<details class="solution"><summary>Solution</summary>

The four vertices with `$x=4$` move to `$x=5$`. The four edges of that face keep their lengths, and the four edges running to it along `$x$` go from length 4 to 5. Nothing else moves. All through `meet`, one step per vertex.

</details>

**2.** Tilt a side of a width-4 rectangle `$[0,4]\times[0,2]$` by angle `$\theta$` about the midpoint of the side `$x=4$`, in the plane. For what `$\theta$` does the rebuild refuse? Check with the demo.

<details class="solution"><summary>Solution</summary>

The moved line passes through `$(4,1)$`. It meets `$y=0$` at `$x=4+\tan\theta$` and `$y=2$` at `$x=4-\tan\theta$` (for a suitable sign of `$\theta$`). The top or bottom side keeps its direction while `$4-\tan\theta>0$`, so until `$\tan\theta=4$`: `$\theta\approx76°$`. The refusal then comes from the top side turning around. At `$\theta=90°$` the line is parallel to the top and bottom and "no longer meets" them. The demo's tilt slider stops at 60°, so try the shape `chamfered` instead: tilting its side `e2` by about `-21°` makes the short side `e1` turn around.

</details>

**3.** Why would "compare normals at foot points" be wrong as a smoothness test, though "compare normals at the vertex's own `$(u,v)$`" is right? Think of a sphere's pole, then a cylinder's seam.

<details class="solution"><summary>Hint</summary>

Foot points move while searching. They clamp at patch boundaries, and each patch does that on its own, so the normals you compare are not at one point. At a vertex, each surface has one known `$(u,v)$`, so one question, answered once: are these tangent here? See `tangent_groups`. For a pole, the normal exists (the `pole_normal` used in `project.rs`) even though `$S_u\times S_v$` vanishes.

</details>

**4.** A curve `$C$` over `$[0,1]$` is carried on to `$[0,1.25]$` by the old `extended` (replace the piece) and the new one (join a piece). Give two reasons why the old one widens the part in `$[0,1]$`, using Lecture 9.

<details class="solution"><summary>Solution</summary>

Extrapolating de Casteljau with `$s>1$` multiplies by factors greater than one and differences, so errors grow (the coefficients are no longer convex); and the new span's control points now carry that width. Evaluating over `$[0,1]$` with them reads every one of them, so the enclosure can no longer be better than they are. Joined as a separate piece, evaluation in `$[0,1]$` reads only the old control points.

</details>

**5.** Why does healing "try every triangulation" rather than deciding by the normals of the faces? Give an example where a plausible triangulation fails and how the failure appears.

<details class="solution"><summary>Hint</summary>

A box with a chamfered edge has a gap of four sides when the chamfer is deleted: the two faces it cut, and the two end faces at its ends. An edge across between the two end caps would make "a box's two ends" meet, which never do: `meet` finds them parallel, or an edge turns around. The other triangulation is accepted. A rule based on normals would need to anticipate the outcome of the rebuild, for any NURBS. Letting the rebuild decide means no such rule exists.

</details>

**6.** *(Reading.)* The module doc of `shell.rs` ends with a list "What it does not do (yet)". Name two items, and say what the kernel does in each case.

<details class="solution"><summary>Solution</summary>

"take away a face tangent to faces kept along some of its edges and not others" and "open the wall round a void" (refused: "face ... bounds a void of the solid: opening the wall round a void is not supported"). Also: "two faces crossing along a curve no edge reaches are not noticed", a known gap in the check rather than a refusal.

</details>
