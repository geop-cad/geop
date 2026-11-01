# Lecture 26: Sweeps: extrude and revolve

<div class="goals">

**After this lecture you can**

- describe every extrusion and revolution as one object, a *grid* of a profile carried through stations along spans;
- count that grid's vertices, edges and faces in closed form and check them against the Euler–Poincaré formula;
- write down the control net of a wall for a straight span and for a circular span, and say why each is exact;
- explain what an axis does to the grid: poles, degenerate coedges, curves that sweep nothing, voids;
- describe a body whole as a `BodySpec`, and say why a sweep is built that way rather than one entity at a time;
- explain why multiplying by an exact 1 can break a boolean six operations later.

</div>

Parts I to VI gave us the raw material: curves and surfaces with honest enclosures, a B-rep to hold them, booleans to combine them, and sketches to draw profiles. This part is about the operations a designer actually asks for. We begin with the two oldest ones. An *extrude* pushes a flat profile along a straight line, and a *revolve* turns one about an axis. Nearly every mechanical part begins with one of these.

The usual way of teaching them is as two operations with two sets of formulas. Geop has one. They are the same algorithm with a different path, and so are a screw thread and a sweep along a curve. Seeing why that works is the main thing to take from this lecture, and the rest of Part VII reuses it.

## A sweep is a grid

Let a profile be a closed loop of `$n$` curves `$c_0,\dots,c_{n-1}$`, drawn in a plane, joined at `$n$` *joints*. Carry it through a *path*: a list of *stations*, where the profile is placed, and between consecutive stations a *span*, which says how it gets from one to the next. Now list what that creates:

| What | Is | How many, open path |
|---|---|---|
| joint at a station | a **vertex** | `$n\cdot s$` |
| curve at a station | an **edge** | `$n\cdot s$` |
| joint along a span | a **lateral edge** | `$n\cdot m$` |
| curve along a span | a **wall**, a face | `$n\cdot m$` |
| the two ends of an open path | two flat **caps** | 2 |

where `$s$` is the number of stations and `$m$` the number of spans. On an open path, `$s=m+1$`; on a closed path, a full turn, `$s=m$`, and there are no caps.

That is the whole topology, for any path whatever. The geometry can differ from station to station but the *shape of the description* never does. Check the count on a box, with a square profile extruded through one span: `$n=4,\ m=1,\ s=2$` gives `$V=8$`, `$E=8+4=12$`, `$F=4+2=6$`. A closed path gives a torus-like solid: `$n=4,\ m=s=4$` gives `$V=16,\ E=32,\ F=16$`.

### Why the counts must come out

A closed surface without holes in its faces satisfies Euler's formula `$V-E+F=2-2g$`, where `$g$` is the genus, the number of handles. In general, with `$R$` extra loops ("rings") in faces, `$S$` shells and `$G$` total genus, the **Euler–Poincaré formula** says

`$$V-E+F-R=2(S-G).$$`

For the open grid, `$V-E+F = ns-(ns+nm)+(nm+2)=2$`. For the closed one, `$V-E+F=ns-(ns+nm)+nm=0$`. The first is a sphere's characteristic and the second a torus's. The formula is a useful habit. When you build topology by rule, count it first, and let the count catch errors in the rule. Lecture 20 does the same with validation, on the built model rather than the plan.

<figure class="demo" data-demo="sweep-grid">
<figcaption>Drag to turn the view. The demo builds the grid by the rules of <code>lib/geop-lib-sweep/src/skin.rs</code>, counts what it built, and prints the Euler–Poincaré check and the closed forms below. Try a full turn with a hole, then a full turn with a side on the axis, whose poles are drawn in red. The profile's curves here are straight lines and the view's arcs are polylines: only the counting is what the kernel does.</figcaption>
</figure>

<div class="kernel">

**In the kernel.** The grid's vocabulary is exactly the table's. `Path` in `lib/geop-lib-sweep/src/sweep.rs` holds `stations`, `spans` and `closed`, plus a name for every station and span; `Path::check` refuses a path whose counts do not fit. The grid itself is built in `skin` in `lib/geop-lib-sweep/src/skin.rs`, whose `Strip` is one section loop carried through the stations. That module is shared with lofts and with a blend's tool, which we meet in Lectures 28 and 29. What differs between them is only where the geometry comes from, through the trait `StripGeometry`, with its four questions `joint`, `curve`, `lateral` and `wall`.

</div>

## Placing the profile: frames

A station is a `Frame`: an origin and two axes. A profile point `$(x,y)$` lies at

`$$p = o + x\,e_1 + y\,e_2.$$`

Placing a profile curve at a station is then an affine map of its control points, so by the affine invariance of Lecture 6 it is *exact*, whatever the curve is. `Frame::curve` does just that, through `NurbCurve::embed`. Note that this works in homogeneous coordinates: a control point `$(wx,wy,w)$` maps to `$(w o + wx\,e_1 + wy\,e_2,\ w)$`, weights untouched, which is why a rational profile stays exact.

Extruding along `$w$` through heights `$h_0<h_1<\dots$` means stations with the same axes and origins `$o+h_jw$`, as `extrusion` in `lib/geop-lib-sweep/src/extrude.rs` builds. The profile's plane need not be square to the path in general. Revolving about an axis means stations whose origin stays put and whose `$e_1$` turns: `revolution` in `lib/geop-lib-sweep/src/revolve.rs` uses

`$$e_1(a)=\cos a\,u+\sin a\,v,\qquad e_2=w,$$`

so the profile is read in `$(r,z)$`: `$r$` away from the axis, `$z$` along it.

## Walls are tensor products

The wall a curve sweeps through a span is a tensor-product surface, `$u$` along the span and `$v$` along the curve. In the code, `Path::wall` lays one row of control points for every control point of the span, each row being the curve's control points placed by that row's frame. Consider what that gives in the two basic cases.

**A straight span.** Degree 1 along `$u$`, knots `$[0,0,1,1]$`, and two rows: the curve at the first station and the curve at the last. Between them lies a ruled surface, exactly the translated curve at each `$u$`.

**A circular span.** Every point `$p$` of the profile moves along a circle about the axis. Recall from Lecture 8 that an arc of angle `$\theta$` is a rational quadratic: end points, a middle control point where the end tangents meet, and middle weight `$\cos(\theta/2)$`. For a profile point at radius `$r$`, the middle control point is at radius `$r/\cos(\theta/2)$`, at the mid angle. That is linear in `$r$`, so *one frame serves the whole row*: a middle frame whose `$e_1$` is the sum of the two end `$e_1$`s scaled to length `$1/\cos(\theta/2)$`. Its `$z$` is unscaled, because the height stays put.

Let us check that with a quarter turn. Then `$\theta=90°$`, `$w=\cos45°=\tfrac{\sqrt2}{2}$`, and `$e_1^{(0)}=u,\ e_1^{(1)}=v$`. The sum `$u+v$` has length `$\sqrt2=1/\cos45°$`, so the sum is already the right vector. In general, `$e_1^{(0)}+e_1^{(1)}$` has length `$2\cos(\theta/2)$`, so the factor needed is `$1/(2\cos^2(\theta/2))=1/(1+\cos\theta)$`. That is `arc_span`'s second result, called `outwards` in `revolution`, computed from the step angle. For the profile point `$(r,z)=(2,1)$`, the wall's control points for the quarter turn are:

| row | point | weight |
|---|---|---|
| start | `$(2,0,1)$` | 1 |
| middle | `$(2,2,1)$` | `$\tfrac{\sqrt2}{2}$` |
| end | `$(0,2,1)$` | 1 |

At the middle parameter, the rational quadratic gives `$\dfrac{\tfrac14(2,0,1)+\tfrac12\cdot\tfrac{\sqrt2}{2}(2,2,1)+\tfrac14(0,2,1)}{\tfrac14+\tfrac12\cdot\tfrac{\sqrt2}{2}+\tfrac14}=(\sqrt2,\sqrt2,1)$`, at radius exactly 2. The cylinder, cone, sphere and torus are therefore no special case. They are what such walls come to for a line or arc as the profile curve, and no code knows their names.

The span's *rows* are always the first section placed by each row's frame, and `Path::rows` makes that a single loop. Because a row is `Row::First(frame, weight)` with `None` for "no weight given", the station rows are not multiplied by anything, for a reason we shall come to.

Why at most a quarter turn per span? The formula `$\cos(\theta/2)$` is positive up to a half turn, and vanishes there, so a rational quadratic cannot reach `$180°$` with a positive weight, and the weights get small before that. `revolution` takes `$\lceil|\text{sweep}|/90°\rceil$` equal spans for a partial turn, and always four for a full one.

## Which way do the walls point

A solid's faces must point out of the material. For a wall, the normal is `$\partial_u S\times\partial_v S$`. Take the square from the box example, wound counter-clockwise in `$(x,y)$`, on a path along `$-z$`. Along its bottom side the curve runs `$+x$`, with material on the `$+y$` side. Then `$\partial_u\times\partial_v=(-z)\times x=-y$`, which points out of the material. So:

> The walls point out when the profile runs counter-clockwise in `$(e_1,e_2)$` and the path runs along `$-(e_1\times e_2)$`, or clockwise and along `$e_1\times e_2$`.

`Path::along_normal` records which of the two a path does, and `sweep_drawn` reverses the loops when it has to, so callers draw profiles with the ordinary convention: outer loops counter-clockwise and holes clockwise. The kernel has no "flipped" flag on a face. As we saw in Lecture 17, orientation lives in the parametrization.

## Caps

An open path needs two caps, flat faces trimmed by the profile. A cap needs a surface, which for us is a flat bilinear patch, and the patch has to lie around the profile. `FlatPatch::around` in `core/geop-core-geometry/src/nurb_surface/flat.rs` makes a box around the curves, in the plane's coordinates, and gives each curve its pcurve on it. The trim loops are the profile's curves again, so the wall's edge and the cap's edge are the very same edge. `areas` in `sweep.rs` decides which loops share a cap: going through the loops in order, one winding like the first starts a new area, and the following ones winding the other way are its holes. It reads winding by `loop_signed_area`, by integration, not sampling (Lecture 19). So a text's several letters give several caps, and hence several lumps.

## An axis degenerates the grid

Revolve a profile that touches its axis and two entries of the table collapse.

**A joint on the axis is a pole.** It does not move, so it is *one* vertex at all stations and has no lateral edges. The wall next to it has a whole side collapsed to a point. To keep the wall's loop closing in `$(u,v)$`, a coedge runs along that side, attached to no edge: `CoedgeOn::Vertex`. Lecture 18 discusses what these are for.

**A curve on the axis sweeps nothing.** It has no wall. With caps it is the one edge both caps share, and on a full turn it does not exist at all.

The counts become, with `$p=2$` poles and one curve on the axis:

`$$V=(n-2)s+2,\qquad E=(n-1)s+\epsilon+(n-2)m,\qquad F=(n-1)m+c,$$`

where `$\epsilon$` is 1 with caps and 0 without, and `$c$` the number of caps. For a full turn, `$s=m$`, `$\epsilon=c=0$`, and `$V-E+F=(n-2)m+2-(n-1)m-(n-2)m+(n-1)m=2$`. It is a sphere topologically, as it should be: the profile closes against the axis, and a solid cylinder is such a body. For a half-disc of two quarter arcs and a side on the axis, `$n=3,\ m=4$` gives `$F=8$`, the sphere's eight octants. `sphere_solid` in `lib/geop-lib-sweep/src/shapes/sphere.rs` writes that same body out directly: "2 rows (north pole to equator, and equator to south pole) x 4 quadrants".

Which flags apply is the caller's business: `SweepLoop` carries `poles` and `on_axis`, one flag per joint and per curve. The grid obeys them. The revolve operation (`ops/geop-ops-extrude-revolve/src/revolve.rs`) measures them from the solved geometry too: a point is on the axis if the constraints say so, or if the solved sketch cannot tell it from the axis, since a corner there is a pole however it got there.

### Holes, voids and shells

A hole in the profile is a second loop, swept the same way. On an extrude its wall is the inside of a tube, and the caps get a ring each, which is the `$R$` in Euler–Poincaré. Now revolve the same thing a full turn: there are no caps to connect the hole's walls to the outer walls. The hole's walls form a closed set of faces of their own, a *void*. Nothing was programmed for this. `connected_faces` groups the faces that share an edge into `spec.shells`, and two groups come out. The count agrees: `$V-E+F-R=0$`, two shells, and each of them a torus gives `$2(S-G)=2(2-2)=0$`.

Try "full turn" with "with a hole" in the demo.

## Naming

Later steps refer to what a sweep built, so every entity gets a name, derived from the names of the things that made it. With `N` the step's namer, `X` a curve of the profile, `P` a joint, `q` a span and `s` a station:

| Entity | Name |
|---|---|
| wall `X` sweeps through `q` | `N(X,q)`, or `N(X)` where one span needs no telling apart |
| curve `X` at `s` | `N(X,s)`; `N(X)` on the axis |
| lateral edge of `P` through `q` | `N(P,q)`, or `N(P)` |
| vertex of `P` at `s` | `N(P,s)`; `N(P)` for a pole |
| caps | `N(start)` and `N(end)` |

An extrude's stations are `start`, `l1`, `l2`, ..., `end`. A partial turn's are `a0`, `a1`, ... and its spans `q0`, `q1`, .... A full turn has a curiosity, which is worth knowing because you will see it in names: it always runs *backwards*, `q3` from `a0` to `a3`, then `q2`, `q1`, `q0`, so that `a1` lies a quarter turn on from `a0` as the angle increases. `revolution` documents the order, and `Path::along_normal` follows from it.

Why does a chapter on geometry dwell on names? Because a parametric program's whole promise (Lecture 2) is that `fillet(extrude(box,c3,start))` still means the same edge after an earlier dimension changed. That is only possible if the sweep names things by *where they came from*, never by index or position.

## Described whole: `BodySpec`

You could imagine building a sweep incrementally, adding a vertex here, an edge there, a face when its loop is complete. The kernel does not. A sweep comes in many shapes (one span or several, closed or not, with poles, with holes, with caps) and each is much simpler to describe at once. `BodySpec` in `core/geop-core-topology/src/build.rs` is a plain list of vertices, edges, faces and shells, referring to each other by *index*. `BodySpec::check` then says whether the description is a consistent body before anything is inserted:

- every index names something;
- every loop closes up, each coedge ending where the next starts;
- every face is in exactly one shell;
- every edge and vertex is used;
- an edge is used twice, in opposite senses, or, where two lumps of a solid touch along it, four times or more, as often one way as the other; only an edge of a sheet may be used once.

Only then does `Model::build_body` insert it. `NamedModel::build_body` adds the names. So an error in a sweep's topology is an error at the source that mentions a loop and a coedge, not a validation failure two operations later.

"Used twice" is the test in the demo too: `book/scripts/demo_tests/sweep-grid.test.mjs` checks it over hundreds of configurations of the same rules.

<div class="kernel">

**In the kernel.** The whole path of an extrude, top to bottom, is short:

- `ops/geop-ops-extrude-revolve/src/extrude.rs`, the operation: resolves the sketch to profiles and the plane, and combines the tool with the target solid;
- `extrude` in `lib/geop-lib-sweep/src/extrude.rs`: `extrusion` makes the path, and `sweep_drawn` orients the loops;
- `sweep` in `lib/geop-lib-sweep/src/sweep.rs`: `SweepLoop::at_corners` cuts curves at internal creases, and `skin` copies the section to every station;
- `skinned`: makes strips and caps, and `skin::skin` writes the `BodySpec`;
- `NamedModel::build_body`: checks and inserts.

Note the division of labour from AGENTS.md's *Where things go*: the algorithm takes `Profile`s, planes and heights, and it is the operation that resolves references. That is why the fillet can use the same machinery with no dependency on the extrude operation.

`SweepLoop::at_corners` deserves a word. A polyline given as one degree-1 curve with internal knots, or a curve from a STEP file with a kink, would sweep into one face creased across its interior. Every question about a face's normal assumes it is smooth, so such a wall misleads an offset, a blend and a silhouette alike. So a curve is cut wherever a knot's incoming tangent is definitely not along the outgoing one, and the crease becomes an edge.

</div>

## Construction code is sensitive at the last bit

Here is a story from AGENTS.md worth knowing by heart. Extrude and revolve used to be separate builders. They were rewritten as the one sweep builder of this lecture, with identical geometry, and six "inscribed cylinder" boolean tests broke (for instance `cube_minus_inscribed_cylinder` in `lib/geop-lib-booleans/src/boolean.rs`).

Nothing about the shape had changed, and in floating-point terms the numbers were equal to within rounding. What had changed was the following. The new builder multiplied every station's row by that row's weight. At a station that weight is 1, and an exact 1. In `f64`, `$x\cdot1=x$` and all is well. But `Scalar` is an interval, and its multiplication rounds outward. So `$x\cdot1$` comes out an ulp wider than `$x$`, on every coordinate of every control point. An inscribed cylinder touches the cube's face tangentially, and whether a search can settle that configuration depends on interval widths down to the last ulp. The width was the only thing that mattered, and it was the thing that had changed.

The fix is the shape of `Path::rows`: a station row is `Row::First(a, None)`, "no weight", and `weighted` does nothing for `None`. Its doc comment states the reason: "A station's row has no weight: multiplying by one would only widen every enclosure by rounding."

You can see the same care throughout construction code. `helix_rows`, for instance, takes `$\text{rise}(0)=0$` and `$\text{rise}(\text{spans})=\text{total}$` as they are, "rather than multiplied by 0 or 1". The rules from AGENTS.md:

- Never multiply or add by exact constants needlessly in construction code.
- When you change how a shape is built, diff old and new models entity by entity (build both from a `git worktree` of HEAD outside the repo), comparing interval bounds, not only midpoints.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"box_with_drill_hole","example":"box_with_drill_hole"}'>
<figcaption>An extrusion with an extruded hole cut from it, as the real kernel meshed it. The program is two sketches and two extrudes: open it in the editor and look at the names of the walls.</figcaption>
</figure>

<div class="recap">

## Recap

- Extrude, revolve, screw and path sweep are one grid: joints × stations are vertices, curves × stations and joints × spans are edges, curves × spans are walls.
- Count the grid in closed form, and check with Euler–Poincaré: a good habit for all topology built by rule.
- Placing a profile by a frame is an affine map of control points, hence exact. A wall is a tensor product of a span's rows and a curve's control points; straight spans are degree 1 and circular ones rational quadratics.
- An axis collapses part of the grid: a pole is one vertex with degenerate coedges, and a curve on the axis has no wall.
- A hole on a full turn becomes a void by itself, because shells are computed from connectivity.
- A `BodySpec` describes the body whole and is checked before it is built.
- An exact 1 still rounds. Shapes feed booleans, and booleans at tangency read the last bit.

</div>

## Exercises

**1.** Count `$V,E,F$` of a hexagonal prism extruded through three heights. Check with Euler's formula. What is `$R$` if the hexagon has a circular hole made of four arcs?

<details class="solution"><summary>Solution</summary>

`$n=6,\ m=3,\ s=4$`: `$V=24,\ E=24+18=42,\ F=18+2=20$`, and `$24-42+20=2$`. With the hole: `$n=6+4=10$`, so `$V=40,\ E=70,\ F=30+2=32$`, and `$R=2$`, one ring in each cap. Then `$40-70+32-2=0=2(1-1)$`, one shell of genus 1.

</details>

**2.** Prove the `$n,m,s$` count for a partial revolve with one side on the axis: show `$V-E+F=2$`.

<details class="solution"><summary>Solution</summary>

`$s=m+1$`, `$\epsilon=1$`, `$c=2$`. `$V=(n-2)(m+1)+2$`, `$E=(n-1)(m+1)+1+(n-2)m$`, `$F=(n-1)m+2$`. Then `$V-E+F=(n-2)(m+1)+2-(n-1)(m+1)-1-(n-2)m+(n-1)m+2$`. Collect: `$(n-2)(m+1)-(n-1)(m+1)=-(m+1)$` and `$-(n-2)m+(n-1)m=m$`. The sum is `$-m-1+m+2-1+2=2$`.

</details>

**3.** A revolved span of 60°. Give its middle weight and the length of its middle frame's `$e_1$`. Verify that the arc's midpoint lies at radius `$r$` for the profile point at radius `$r$`.

<details class="solution"><summary>Solution</summary>

`$w=\cos30°=\tfrac{\sqrt3}{2}$`. The middle `$e_1$` has length `$1/\cos30°=\tfrac{2}{\sqrt3}$`. Take end points `$A,B$` at radius `$r$`, and the middle point `$M$` at radius `$\tfrac{2r}{\sqrt3}$` on the bisector. Along the bisector, `$A$` and `$B$` have component `$r\cos30°=\tfrac{\sqrt3}{2}r$`. So the midpoint's component is `$\dfrac{\tfrac14\cdot\tfrac{\sqrt3}{2}r\cdot 2+\tfrac12\cdot\tfrac{\sqrt3}{2}\cdot\tfrac{2r}{\sqrt3}}{\tfrac12+\tfrac12\cdot\tfrac{\sqrt3}{2}}=\dfrac{\tfrac{\sqrt3}{4}r+\tfrac12r}{\tfrac12+\tfrac{\sqrt3}{4}}=r$`, and across the bisector the two end components cancel.

</details>

**4.** Why does `Frame::curve` not need to sample a rational curve to place it, and why would a projective map (a perspective view) be different?

<details class="solution"><summary>Hint</summary>

Affine maps act on `$(wx,wy,w)$` by a linear map that leaves `$w$` alone, so `$\sum B_iw_iP_i/\sum B_iw_i$` maps term by term. A projective map changes the weights too. It can still be applied to a NURBS exactly, by a linear map on homogeneous coordinates, but the weights change and might not stay positive, so the convex hull property could be lost.

</details>

**5.** A designer draws a square profile on `$z=0$`, wound counter-clockwise when seen from `$+z$`, and extrudes along `$+z$`. Use the wall rule to decide whether `sweep` alone or `sweep_drawn` gives outward walls, and what the latter does.

<details class="solution"><summary>Solution</summary>

With `$e_1=x,\ e_2=y$`, `$e_1\times e_2=+z$`. The path runs along `$e_1\times e_2$`, so `along_normal` is true, and the rule wants a clockwise loop there. `sweep` with the counter-clockwise loop as drawn would give inward walls. `sweep_drawn` reverses the loops (`SweepLoop::reversed`) first.

</details>

**6.** *(Reading.)* Find in `lib/geop-lib-sweep/src/skin.rs` what an *apex* is, and how it differs from a pole. What operation uses it?

<details class="solution"><summary>Hint</summary>

A pole is one joint that stays put through all stations. An apex is a *station* where the whole loop collapses to one point, as a loft to a point ends: one vertex for every joint, no edges there, and each wall closes over it with a degenerate coedge. See `SweepLoop::apex` and `Ends` in `lib/geop-lib-sweep/src/loft.rs`.

</details>
