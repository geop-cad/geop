# Lecture 17: The boundary representation

<div class="goals">

**After this lecture you can**

- explain why a CAD kernel represents a solid by its boundary, and what that buys over a mesh, a voxel grid or a tree of primitives;
- name every entity of a B-rep (vertex, edge, coedge, loop, face, shell, solid, sheet, wire), say what each owns and what it merely refers to;
- walk a loop with `next`, `prev` and the other coedge of an edge, by hand and in code;
- use the Euler–Poincaré formula to check a body's counts, and work out a real body's counts from what the kernel built;
- say where orientation lives in Geop, and why a face has an `outer` field and a `holes` field rather than a list.

</div>

Part III gave us curves and surfaces we can intersect with proof. But a curve is not a part. A part has an inside, and it has faces you can pick, edges you can round and a volume you can weigh. Something has to say how pieces of surface fit together into a closed skin, and that something is the subject of Part IV. It is much less mathematical than what came before and every bit as easy to get wrong.

## What should a solid be, in a computer?

A solid is a set of points in space. A computer cannot store a set of infinitely many points, so it stores a description of it. There are four serious candidates.

**Voxels** divide space into small cubes and mark each as in or out. Booleans are trivial, one logical operation per voxel, and that is the end of the good news. A face is never flat, a dimension is never exact, and memory grows with the cube of the resolution.

**Triangle meshes** store the skin as flat triangles. Every graphics system uses them, and Geop's rasterizer (`core/geop-core-rasterize`) produces them for drawing. But a mesh of a cylinder is not a cylinder. It has no radius you can read and no tangent you can trust, and a fillet between two faces needs to know what those faces are.

**Constructive solid geometry (CSG)** stores a tree: this box, minus that cylinder, union this sphere. The description is exact and compact. But the faces and edges the result has are nowhere in it, so you cannot pick "that edge" and round it without first computing what the tree means.

**Boundary representation (B-rep)** stores the skin itself: exact pieces of surface, each trimmed by exact curves, and a record of which piece meets which. A point is inside if it is enclosed by the skin. This is what every mechanical CAD system of consequence uses, because it answers the questions a designer asks: there is an entity for the top face and an entity for the edge around the hole, each with exact geometry.

The price is that a B-rep has two kinds of content that must agree. *Geometry* says where things are: surfaces, curves, points. *Topology* says what is connected to what: this edge bounds these two faces, this vertex ends these three edges. Most of what makes a boolean hard is keeping the two in agreement while both are being changed. This lecture introduces the data structure. [Lecture 18](./18-poles-seams-splicing.md) changes it. [Lecture 20](./20-validation.md) checks the agreement.

Geop is also a little CSG-like at a higher level: a part is a program of steps ([Lecture 2](./02-programs-of-steps.md)), and each step's output is a B-rep, not a tree.

## The entities, from the bottom up

Take a cube. It has 8 corners, 12 straight sides and 6 square faces. The B-rep has entities for those, and needs a few more besides.

**A vertex** is a point in space. In Geop, `Vertex` (`core/geop-core-topology/src/vertex.rs`) holds exactly one field, `point: Vector3`, and the three coordinates are interval `Scalar`s, as everywhere. So a vertex is an enclosure of a point (see [Lecture 4](./04-intervals.md)), and two vertices are the same point only if they are the same entity.

**An edge** is an arc of a curve between two vertices. `Edge` (`core/geop-core-topology/src/edge.rs`) holds `curve: Curve3`, `start_vertex` and `end_vertex`. Note a simplification that many kernels do not make: there is no window `[t0, t1]` into a longer curve. The curve is trimmed so that its own domain is exactly the edge, `start_vertex` at the start of the domain and `end_vertex` at the end. Splitting an edge splits its curve.

**A face** is a piece of a surface. `Face` (`core/geop-core-topology/src/face.rs`) holds a `surface: NurbSurface3D`, one `outer` boundary, a list of `holes`, and the `shell` it belongs to. It means: everything of the surface inside the outer boundary and outside every hole.

How is a boundary given? You might think: by a list of edges. That is where the naive design fails, and understanding why is the key to the whole structure.

### Why an edge is not enough: the coedge

Consider the edge between the top and front faces of the cube. It belongs to two faces. Walking the boundary of the top face counter-clockwise (seen from outside), you pass along it in one direction. Walking the boundary of the front face, also counter-clockwise, you pass along it in the *other* direction. Moreover, each of the two faces has its own coordinates: the same 3-D curve appears as one curve in the `(u, v)` plane of the top face and as a different curve in the `(u, v)` plane of the front face.

So there are facts that belong neither to the edge, of which there is one, nor to the face, which has many sides. They belong to *one side of the edge, on one face*. This is what a **coedge** is. (Other texts call it a half-edge, or a *fin*, or a *use* of an edge.) `Coedge` (`core/geop-core-topology/src/coedge.rs`) has:

| Field | Meaning |
|---|---|
| `geometry` | `CoedgeGeometry::Edge(id)`, the edge it runs along (or `CoedgeGeometry::Vertex(id)`, see [Lecture 18](./18-poles-seams-splicing.md)) |
| `sense` | `Sense::Forward` if it runs the edge from start to end, `Sense::Reversed` otherwise |
| `pcurve` | a `Curve2`: where the edge runs in the `(u, v)` plane of this face |
| `next`, `prev` | the coedges before and after it around its loop |
| `face` | the face it belongs to |

The `pcurve` deserves a moment. We met pcurves in [Lecture 16](./16-tracing-and-pcurves.md). A curve in parameter space is the only way to say exactly which part of a surface a face is. Its existence means every edge of a solid is represented three times: once in 3-D, and once in each of its two faces' parameter planes. They must agree, and they must agree *to within their widths*. That is the most-checked fact in the kernel.

### Loops

A face's boundary is one or more closed cycles of coedges. Such a cycle is a **loop**. In Geop a loop is not an entity at all. A face's boundary is a `BoundaryType` (`core/geop-core-topology/src/boundary.rs`), and `BoundaryType::Loop(id)` names one coedge of the cycle, the *anchor*. Following `next` from the anchor walks the loop until it returns. `Model::iterate_loop_coedges` does this. There is nothing else to keep consistent.

(`BoundaryType` has a second variant, `Vertex(id)`, for a boundary that is a bare point, a face with no edges yet or another body touching a face at a point. Then there is no pcurve, only the vertex. A face always has an outer boundary, so "no boundary" is not a state the type can even express. We return to this.)

Now use the demo. It is a real half-edge structure, built from a list of faces the way a `BodySpec` describes a body. Each arrow is a coedge, drawn shifted to its left, into the material of the face it bounds.

<figure class="demo" data-demo="brep-walk">
<figcaption>Walk with <em>next</em> and <em>prev</em>, cross to the neighbouring face with <em>other side</em>, and drag to turn the body. Check for yourself: every <em>next</em> ends where it starts; <em>other side</em> twice comes back; and it always reverses the direction. Then choose the block with a through hole, and find the loop that is a hole. In the kernel, faces are curved and each coedge has a pcurve; here, faces are flat and only the connectivity is shown.</figcaption>
</figure>

Notice what you never saw: a pointer from a coedge to its opposite. Geop does not store one. The opposite coedge is found through the edge, by `Model::coedges_of_edge`, because a stored pointer would be one more thing that could disagree with the rest.

Why a doubly linked list rather than an array in each face? Because the operations of the next lecture are all of the form "cut this ring here and join it to that ring there". On linked lists these are four pointer assignments. On arrays, they are reorderings and index fix-ups everywhere.

### Shell, solid, sheet, wire, body

**A shell** is a connected set of faces. `Shell` (`core/geop-core-topology/src/shell.rs`) holds `faces` and `solid: Option<SolidId>`.

**A solid** is a region of space bounded by closed shells. `Solid` (`core/geop-core-topology/src/solid.rs`) holds `shells`. It usually has one. It has more if it consists of separate lumps, or has a void inside. What is interesting is how lumps and voids are told apart: they are not. The doc comment says it directly: "a point is inside the solid when it is inside an odd number of them, so a shell is a lump of its own or a void in one by where it lies, not by its place in `shells`". A void's faces point into it, out of the material, and the volume it encloses comes out negative.

**A sheet** is a shell that belongs to no solid: `solid` is `None`. Its faces stand on their own, bounding nothing, and its border edges are used by one face only. A sheet is what a surface operation builds, and what a split cuts a solid with.

**A wire** (`core/geop-core-topology/src/wire.rs`) holds edges and vertices that bound no face: curves in space. A sweep's path can be one.

`Body` (`core/geop-core-topology/src/body.rs`) is a plain enum of `Solid(SolidId)` and `Sheet(ShellId)`: what operations act on. Remesh and imprint ([Lecture 21](./21-remesh.md)) take bodies because all they ask of one is its faces, edges and vertices.

The model is the arena that holds it all:

```text
Solid ──▶ Shell(s) ──▶ Face(s) [+Surface] ──▶ Coedge(s) [+Curve2] ──▶ Edge [+Curve3] ──▶ Vertex
Wire ──▶ Edge(s) [+Curve3], Vertex(es)
```

<div class="kernel">

**In the kernel.** `Model` (`core/geop-core-topology/src/model/mod.rs`) has one map per kind of entity, from a typed id to the entity: `vertices`, `edges`, `coedges`, `faces`, `shells`, `solids`, `wires`. The ids (`VertexId`, `EdgeId`, `CoedgeId`, `FaceId`, `ShellId`, `SolidId`, `WireId`, from `core/geop-core-topology/src/ids.rs`) are distinct types wrapping a `u64`, so that passing a face where an edge is wanted does not compile. They come from one counter, `next_id`, so an id's order is the order of creation.

The maps are `imbl::OrdMap`s, persistent ordered maps. Two properties matter. A copy of a model shares its structure with the original, so keeping the model after every step costs little. And iteration goes in id order, so that every walk over a model sees the entities in the same order on every run. They were `HashMap`s once; a validation failure that depended on which pair of faces was checked first appeared and disappeared between runs. The comment on `Model` records that, and AGENTS.md's *A hash map's order is no order* gives the general rule: wherever an order reaches the result, sort by the ids.

Geometry is owned by the entity it belongs to, with no arena of its own: nothing else ever refers to a curve or a surface. That is why two faces cut from one surface each hold their own copy. Since `Curve3`, `Curve2` and `NurbSurface3D` are not shared, there is no aliasing to reason about.

</div>

## Orientation has no flag

Which side of a face is outside? In many kernels, a face carries a boolean, "the normal is reversed". Geop has no such flag. The material side follows from the surface alone: the normal is `$S_u\times S_v$`, and the convention is that it points *out of* the solid. A face that has to be turned around has its surface turned around.

The conventions, all of which are checked (`core/geop-core-topology/src/validation/face_orientation.rs`):

1. The normal `$S_u\times S_v$` points out of the material.
2. In the `(u, v)` plane, an outer loop runs counter-clockwise and a hole runs clockwise. Equivalently, **the material lies to the left of every coedge**, for outer loops and holes alike.
3. Two coedges of an edge have opposite senses.

Are these independent? Check with the top of a cube. If `$(u, v)$` are `$(x, y)$`, the normal is `$+z$`, up and out. An outer loop counter-clockwise in `$(u, v)$` is counter-clockwise seen from above, with the top of the square to the left. Fine. Now the neighbouring face must traverse the shared edge the other way, for it too to have its material on its left when seen from its own outside: fold the two faces flat along the edge to see it. So (3) follows from (1) and (2) on a closed orientable surface.

`Model::reverse_face` (`core/geop-core-topology/src/edit/reverse_face.rs`) shows what turning a face around amounts to. It mirrors the surface in `u` (`NurbSurface::reverse_u`), applies the same mirror to every pcurve, and reverses each loop: swap `next` and `prev`, run each pcurve the other way, flip each `sense`. The edges, the vertices and the coedges on the other side of the edges are not touched, which is what lets a boolean flip one operand's faces while they stay glued to the other's.

## Make the invariant a type

`Face` once had a flat `Vec` of boundaries with "index 0 is the outer loop" as an unwritten rule. It now has `outer: BoundaryType` and `holes: Vec<BoundaryType>`, and lookups return a `BoundaryIndex`, either `Outer` or `Hole(i)`. AGENTS.md calls this "not cosmetic", and this is why.

An edge joining two points of a face's boundary means different things according to which boundaries the points lie on. Two different holes merge. A hole bridged to the outer loop stops being a boundary. Two points of the outer loop cut the face in two. With a flat list, these looked like "same index or different index": two cases, handled as two cases, which silently produced faces carrying several disjoint patches. As a type, `match (loop_a, loop_b)` over `BoundaryIndex` is four cases, and the compiler insists on all of them. We work through them in the next lecture.

The general lesson goes beyond this one type. Any time a comment says "by convention, the first element is special" or "this is only set if", ask whether the type could make the wrong state unrepresentable. Each such state a type rules out is a state nothing else ever has to handle.

## Counting: Euler and Poincaré

There is a global check on all these counts. For a polyhedron with `$V$` vertices, `$E$` edges and `$F$` faces, topologically a sphere,

`$$V-E+F=2.$$`

For a cube: `$8-12+6=2$`. The reason is worth having in your fingers. Remove one face and flatten the rest onto the plane. Now repeatedly remove things: an edge between two faces (removes one edge and one face, leaves `$V-E+F$` unchanged), or a dangling edge with its end vertex (removes one vertex and one edge, unchanged). Eventually one vertex remains, for which `$V-E+F=1$`. Add back the face we removed first, for 2.

Two generalizations cover everything a B-rep can be. A *handle*, like the hole through a ring, lowers the count by 2 per handle. A face with a *hole loop* is not simply connected, so the argument above needs one extra edge to bridge the hole to the rest of the boundary, which adds one to `$E$` without adding a face. So for `$S$` shells, total genus `$G$`, and `$L$` hole loops over all faces,

`$$V-E+F-L=2(S-G).$$`

Choose the block with a through hole in the demo. It reads `$16-24+10-2=0$`, so one shell of genus 1, as a square ring has.

<div class="kernel">

**In the kernel.** The formula is stated in the comment on `Model`, and it is what `Coedge`'s documentation means when it says a `Vertex`-backed coedge leaves "the Euler–Poincaré count as it is". But no validation in `core/geop-core-topology/src/validation/` computes it. The closure of a shell is checked directly, by counting how each edge is used (`validation::manifold_check`, [Lecture 20](./20-validation.md)). So use the formula as you would use a checksum, in your own head, when looking at a body.

</div>

### A worked example from the real kernel

The example `box_with_drill_hole` is a 2 by 2 by 1 box with a blind hole of radius 0.4 and depth 0.5 in its top. Running it (`geop` 0.12.0, `list_elements` over `geop mcp`, capped) gives 16 vertices, 24 edges and 11 faces:

| Entity | Count | Which |
|---|---|---|
| vertices | 16 | the box's 8; 4 on the hole's rim at `$z=1$`, 4 on its floor at `$z=0.5$` |
| edges | 24 | the box's 12; 4 rim arcs; 4 floor arcs; 4 straight seam lines on the wall |
| faces | 11 | the box's 6; the wall in 4 quarters; the floor |

The circle is four quarter arcs because a sketched circle is four rational arcs, and a sweep turns each into a wall of its own (see [Lecture 8](./08-nurbs.md) on why a quarter arc is a good piece, and [Lecture 26](./26-sweeps.md) on the sweep). Now `$V-E+F=16-24+11=3$`. The top face of the box has the rim as a hole loop, so `$L=1$` and `$V-E+F-L=2$`: one shell, genus 0. A blind hole is no handle. Correct.

Compare `cross_drilled_shaft`, a stepped shaft with a hole straight through it from side to side (the same run): 26 vertices, 50 edges, 24 faces, so `$V-E+F=0$`. A through hole makes genus 1, and `$2(1-1)=0$`, so `$L=0$`: no face there has a hole loop. That is true because the hole's rim runs across the shaft's quarter seams, so each bite is part of the outer loop of the quarter it is in. The formula told you something about how the kernel cut the faces. We return to this example in [Lecture 21](./21-remesh.md).

<figure class="demo" data-demo="model-viewer" data-args='{"model":"box_with_drill_hole","example":"box_with_drill_hole"}'>
<figcaption>The body the table above describes, as meshed by the kernel. Turn it and count the faces. Opening it in the editor shows every face, edge and vertex by name.</figcaption>
</figure>

## Saying a body whole

How do you make a B-rep? You could insert entities one at a time, keeping a partly built body valid after each step. The operations of the next lecture do that, because they are editing what exists. But a sweep comes in many shapes (one span or several, closed around an axis or not, with poles, holes or caps), and each would need its own sequence of steps. It is far simpler said whole: these vertices, these edges between them, these faces, each bounded by these coedges.

That is `BodySpec` (`core/geop-core-topology/src/build.rs`). It is plain data, entities referring to each other by index in lists: `vertices`, `edges` (`EdgeSpec`: a curve, a `start`, an `end`), `faces` (`FaceSpec`: a surface, an `outer` loop, `holes`), `shells`, and a flag `solid`. A loop is a list of `CoedgeSpec`, each saying what it runs along (`CoedgeOn::Edge(index, sense)` or `CoedgeOn::Vertex(index)`) and giving its pcurve.

`Model::build_body` takes a spec, runs `BodySpec::check`, and only then inserts anything. The check is entirely combinatorial. Every edge runs between vertices that exist. Every coedge ends at the vertex where the next one starts, so that every loop closes. Every face is in exactly one shell. Every vertex is used. And the key test: an edge is used an even number of times, as often one way as the other, with exactly one use allowed only for a sheet; used twice in the same sense, or by one face in a solid, it is an error that names the edge.

This is a worthwhile design to imitate. A construction builds a description, the description is checked before the model is touched, and a failed check leaves nothing half done. `Model::body_spec` goes the other way, describing faces of a model as a copy, and says where each entity came from (`SpecSources`). This pair is what `geop_lib_sweep`'s builders and the operations that copy or rebuild faces are written with. A description with no faces is a wire.

<div class="kernel">

**In the kernel.** To see the structure in use, start at `core/geop-core-topology/src/test_fixtures.rs` (`test_cube_solid`, the smallest whole solid) and `lib/geop-lib-sweep/src/shapes/sphere.rs` (`FaceSpec::on_unit_square` with `CoedgeOn::Vertex`, a sphere as eight faces). For reading: `Model::iterate_loop_coedges`, `Model::iterate_face_coedges`, `Model::coedges_of_edge`, `Model::face_corners`, `Model::body_faces` (`core/geop-core-topology/src/model/iterate.rs`). `face_corners` is what error messages print: each loop, as the vertices it runs through.

</div>

<div class="recap">

## Recap

- A B-rep stores the skin of a solid: exact geometry (surfaces, curves, points) plus topology (what bounds what). Most of its difficulty is keeping the two in agreement.
- A coedge is one side of an edge on one face. It holds a sense, a pcurve and its neighbours in a loop. A loop is no entity, only a ring of coedges with an anchor.
- Geop stores entities in ordered arenas by typed id, geometry owned by the entity. The opposite coedge is found through the edge.
- Orientation lives in the surface's parametrization: the normal points out, and the material lies to the left of every coedge.
- `$V-E+F-L=2(S-G)$` is a checksum on counts, for your head; the kernel checks closure directly, edge by edge.
- `BodySpec` says a body whole, and is checked before it is inserted.

</div>

## Exercises

**1.** A triangular prism has how many vertices, edges, faces and coedges? Verify `$V-E+F=2$`, then choose the prism in the demo to check, and walk one of its triangular loops.

<details class="solution"><summary>Solution</summary>

`$V=6$`, `$E=9$`, `$F=5$`, and each edge has two coedges, so 18 coedges: three on each triangle and four on each of the three rectangles, `$2\cdot3+3\cdot4=18$`. `$6-9+5=2$`.

</details>

**2.** Geop's sphere is eight faces meeting at two poles (`lib/geop-lib-sweep/src/shapes/sphere.rs`). Count `$V$`, `$E$`, `$F$`, the real coedges and the degenerate ones. Check the Euler formula. Why does a degenerate coedge leave the count alone?

<details class="solution"><summary>Solution</summary>

Vertices: two poles and four on the equator, `$V=6$`. Edges: four equator arcs and eight meridian arcs, four in each hemisphere, `$E=12$`. Faces: `$F=8$`. `$6-12+8=2$`. Real coedges: `$2E=24$`, which is three on each of the eight faces. Each face also has one `CoedgeGeometry::Vertex` coedge along its pole's collapsed row, 8 in all. It adds neither a vertex nor an edge, and the formula counts neither loops' lengths nor coedges, so nothing changes.

</details>

**3.** A disc with two round holes is extruded. Each circle is a single closed edge from a vertex to itself, ignoring how Geop would cut it into quarters. Count `$V,E,F,L$` and find `$G$`.

<details class="solution"><summary>Solution</summary>

Three circles at each of two heights, each with one vertex: `$V=6$`. Edges: 6 circles and 3 straight seams, `$E=9$`. Faces: top, bottom and 3 walls, `$F=5$`. Hole loops: two on the top and two on the bottom, `$L=4$`. `$6-9+5-4=-2=2(1-G)$`, so `$G=2$`: a two-holed block is a double torus. Each wall is here one periodic surface with a seam, used twice by one face. Geop avoids this kind of face, as Lecture 18 explains.

</details>

**4.** Show that a coedge's pcurve, its edge's curve and the face's surface being all exact does not by itself prevent `$S(p(t))\ne C(t)$`. What would have to hold, and what is the kernel's stance on "hold" given that everything is an enclosure?

<details class="solution"><summary>Hint</summary>

`$p$` and `$C$` are independently stored data, and their parametrizations may differ, so the identity one can ask for is about the point sets. The kernel requires that at matching fractional positions the two enclosures `could_be_equal`, in `validation::curve_and_surface_sampling_check`; and at the ends that they land on the vertices, in `validation::check_curves_and_surfaces_vertices`. Interval overlap at samples is necessary, not sufficient. The discussion continues in Lecture 20.

</details>

**5.** Rules (1) and (2) of *Orientation has no flag* imply (3). State this as a precise claim about two faces sharing a straight edge in the plane `$z=0$`, one to each side, with both surfaces parametrized so that the normal is `$+z$`.

<details class="solution"><summary>Solution</summary>

Let the edge lie along the `$x$` axis, the first face in `$y>0$` and the second in `$y<0$`, each surface with `$(u, v)=(x, y)$`. By (2), the first face's boundary has the face on its left. For a coedge along the `$x$` axis with `$y>0$` on its left, the direction is `$+x$`. For the second, with `$y<0$` on its left, it is `$-x$`. So the two coedges run in opposite directions: (3).

</details>

**6.** Why does `Model` not store an `opposite` pointer on `Coedge`, and what is the cost of that choice? How would you decide whether to add one?

<details class="solution"><summary>Hint</summary>

The benefit is one thing fewer to keep in agreement. A cost is that `coedges_of_edge` has to find the coedges rather than read them; read `core/geop-core-topology/src/model/iterate.rs` for how. AGENTS.md's *Count the work, not the seconds* gives the method for deciding: count how often the walk actually costs something in a real operation, before optimizing it.

</details>
