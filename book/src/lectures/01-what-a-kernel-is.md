# Lecture 1: What a CAD kernel is

<div class="goals">

**After this lecture you can**

- say what a geometric modelling kernel does, and what it leaves to the application around it;
- compare four ways of representing a solid (mesh, voxels or distance fields, CSG, boundary representation) and say why mechanical CAD uses the last;
- name the three problems that make a kernel hard, and explain why each one is a problem of *decisions* and not of accuracy;
- find your way around the Geop repository: which tier a piece of code belongs in and why;
- state Geop's design position, including what it deliberately refuses to do, and what it does not do yet.

</div>

This book teaches you how to build a CAD kernel. You will need linear algebra, calculus, and enough programming to read Rust. You do not need to know anything about CAD or computational geometry; we build all of that up.

We teach through one real kernel, Geop, whose code is in the same repository as this book. Whenever we say "the kernel does X", there is a file and a function you can read, and it is named in the text. The kernel changes, so if the text and the code disagree, believe the code.

## A solid in a computer

Imagine a steel block with a hole drilled through it and every edge rounded. You want a computer to hold this block in such a way that it can answer questions about it:

- What is its volume, and where is its centre of mass?
- What is the distance between the hole's axis and the block's side, exactly?
- Is the point `$(3, 4, 5)$` inside the steel?
- What does the block become if I also cut a slot through it?
- What curve should a milling machine follow along that rounded edge?
- If I make the block 2 mm longer, what happens to everything built on it?

A **geometric modelling kernel** is the library that holds such a description and answers such questions. It has no buttons and no window. It takes in operations ("extrude this outline", "subtract that solid", "round this edge by 3") and gives out models, measurements, meshes for display and files for other programs.

Commercial CAD systems are built over a small number of such kernels. Parasolid sits under Siemens NX and SolidWorks; ACIS is licensed by Spatial; Open CASCADE is open source and sits under FreeCAD. SMLib is another, and `docs/smlib-comparison/README.md` compares Geop against it feature by feature. What separates these kernels from each other is rarely the mathematics of a fillet. It is how often the fillet works.

The kernel is distinct from the rest of a CAD program in a useful way:

| Belongs to the kernel | Belongs around it |
|---|---|
| curves and surfaces, intersecting them | drawing them on the screen |
| which faces meet along which edges | which face the user is hovering over |
| what a fillet of radius 3 *is* | the dialog that asks for the 3 |
| whether a solid is valid | undo, files, the ribbon of buttons |

Geop's repository contains both, since a kernel is hard to test or to use with nothing around it. But the separation is strict, and we will see below that a script checks it.

## Four ways to represent a solid

Let us compare the serious candidates by asking each the same questions.

### Triangle meshes

A mesh is a list of triangles. Games, film and 3-D printers run on them. Meshes are simple, uniform, and very fast to draw, and the algorithms are short.

But a mesh only approximates a curved surface. A circle of radius `$r$` replaced by a regular `$n$`-gon misses by the *sagitta* `$r(1-\cos(\pi/n))\approx r\pi^2/(2n^2)$`. A hole of radius 5 drawn with 16 segments is `$0.096$` off, which in a machine shop is an enormous error. Getting to `$10^{-6}$` on a radius of 50 takes about 15 700 segments per circle (exercise 2), and a surface needs the square of that. There is no finite mesh of a cylinder from which the radius can be read back exactly, nor one that is tangent to anything exactly. And where two facetted surfaces meet, the intersection is a jagged polyline with thousands of tiny segments, whose next boolean will be worse.

### Voxels and distance fields

A voxel grid stores inside or outside per cube of space. A *signed distance field* stores the distance to the surface at each point, or a formula for it. Booleans become trivial: the union of two solids given by distance functions `$d_1,d_2$` is `$\min(d_1,d_2)$`. Fillets can even be had by smoothing that minimum. The trouble is that the field's sharp edges are lost at the resolution of the grid, nothing about the result is exact, and a question like "what is the axis of this hole?" has no answer in the representation, since a hole is not a thing in it. These representations are excellent for some purposes and they are growing, but mechanical parts are defined by exact dimensions and exact relationships.

### Constructive solid geometry

CSG stores the solid as a tree: `(box - cylinder) ∪ other_box`. It is compact and exact, and point classification is easy: evaluate the tree at the point. But a tree does not know its own surface. To draw it, measure it, or pick "this edge" in it you must *evaluate* the tree, which means computing where those solids' surfaces meet. That is exactly the hard part, deferred. There is no such thing as "round this edge" in a tree, because a tree has no edges.

### Boundary representation

A **boundary representation (B-rep)** describes a solid by its skin. It has two layers:

- **Geometry**: exact mathematical surfaces and curves. "This face lies on the cylinder of radius 5 about this axis." "This edge lies on this circle."
- **Topology**: how the pieces connect. "These twelve edges bound these six faces. This edge is shared by exactly those two faces. These faces enclose a solid."

A cube is 8 vertices, 12 edges and 6 faces. A face's surface extends without limit in principle; the *loop* of edges around the face trims it to the part that is actually the solid's skin. So the top of our block with a drilled hole is one plane, an outer loop of four edges, and an inner loop of one or more edges around the hole.

Topology is what makes questions about the part *discrete*. You can point to a face, name it, ask for its neighbours. "All edges of this face" is an exact finite list, not something that has to be recovered from a cloud of triangles. Since the geometry is exact, measurements and tangency are exact too. B-rep is the representation of essentially all mechanical CAD. Its one defect is that its operations are hard, and this book is about that.

Look at a real one. This is a box with every edge rounded, as Geop builds it. The picture is a mesh, because a screen draws triangles, but the mesh is only a *rendering*. The model behind it has faces lying on planes, cylinders and spheres.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"rounded_box","example":"rounded_box"}'>
<figcaption>A 2 x 1.5 x 1 box rounded by 0.2, built by the real kernel (the example <code>rounded_box</code>) and meshed coarsely so that your browser can draw it. Drag to turn it. Show the triangles to see how the mesh is laid per face. The link opens the same program in the editor, where the kernel runs as WebAssembly in your browser.</figcaption>
</figure>

The mesh you see has 1328 triangles (`-O quality=8`). With the kernel's default quality it has 17 328 (we ran both, see [Lecture 2](./02-programs-of-steps.md)), from the same model. That default mesh's volume, computed by summing tetrahedra, is `$2.85566$`. The exact volume of this shape is `$2.85620$`, which you can compute in closed form (exercise 3). Our mesh's flat facets lie inside the curved surface, and that is the difference. A B-rep's volume is an integral over exact surfaces, not a sum over an approximation: `Model::mass_properties` in `core/geop-core-topology/src/mass.rs` turns it into line integrals around the faces' trim curves by Green's theorem.

## What makes it hard

All the algorithms we will meet are, at their mathematical core, not very complicated. A boolean difference can be explained on a blackboard in ten minutes: find where the boundaries of `$A$` and `$B$` cross, cut them there, keep the pieces of `$\partial A$` outside `$B$` and the pieces of `$\partial B$` inside `$A$`. So why are industrial kernels the work of decades? There are three hard problems, and they feed each other.

### Problem 1: decisions from inexact numbers

The computer holds coordinates in finite precision ([Lecture 3](./03-floating-point.md)). But the output of a kernel is mostly not a number. It is a *decision*: this curve crosses this face, this point is on this boundary, these two vertices are the same. Decisions are discrete, and they have to be consistent with one another.

Suppose three operations each ask whether point `$P$` lies on plane `$E$`. If the answer comes from `$|\text{distance}|<10^{-9}$`, then a slight change in how `$P$` was computed changes the answer. A boolean that believes in one place that an edge crosses a face, and in another place that its end point lies on the face, has built something that isn't a solid. The result of inconsistency is rarely a slightly inaccurate model. It is a crash, a hole in the shell, an infinite loop, or a model that looks fine and fails three operations later. Accuracy can degrade gracefully; topology cannot.

### Problem 2: intersection

The central geometric question is "where do these two things meet?". For two lines or two planes, a formula answers it. For two free-form surfaces, the answer is a space curve with no closed form, of degree up to hundreds if you write it algebraically. It may have several disconnected branches, closed loops that touch no boundary, cusps, or isolated tangent points, and the two surfaces may coincide along a stretch. And whatever algorithm you pick has to find **all** of it, because a missed branch means a wrong solid, and do so with the inexact numbers of problem 1. Part III of this book is about this.

### Problem 3: keeping topology and geometry consistent

An operation changes a B-rep through many small steps: split this edge, imprint this curve in that face, drop these faces. After each one the model has to remain a meaningful thing. Every edge lies on both its faces' surfaces. Every loop closes. Each vertex is on all its edges' ends. Yet the *geometric* facts in that list are never exactly true in finite precision: an edge computed as the intersection of two surfaces does not lie exactly on either. So what does "lies on" mean? Traditional kernels answer with a *tolerance* stored per entity. As we will see in [Lecture 5](./05-honest-numerics.md), Geop answers differently. Parts IV and V are about this.

There is also a fourth problem, which belongs to parametric CAD and not to kernels as such: when the block gets longer and the model is rebuilt, "this edge" in a later operation must still mean the same edge. This goes by the name of the *topological naming problem*, and [Lecture 2](./02-programs-of-steps.md) shows how Geop handles it.

### An example of how it goes wrong

Take a box and cut from it a slightly rotated box such that one of the tool's edges passes very nearly through an edge of the first. The two edges cross at a shallow angle. Computing the crossing's position is ill-conditioned, so the answer is uncertain along the edges by far more than across them. Suppose the piercing search reports two crossings where the truth has one, separated by `$10^{-13}$`. Now an edge of length `$10^{-13}$` exists. Nothing can be located along it. A face bounded by it may have no interior point at all. The boolean's classifier asks for a point inside each face, finds none, and returns an error about a face far from any near-coincidence. The failure report names the distant symptom rather than the cause.

Nearly all the rules this book teaches are about never getting into such a state, or at least about finding out at the source if we do.

## Geop's position

Geop takes distinct positions on each hard problem. They are its reason for existing, and each gets a part of the book.

**Every number is an interval.** A coordinate is not a value `$x$` but an enclosure `$[x_{lo},x_{hi}]$` that provably contains the true value, and a comparison answers "definitely", "definitely not" or "could be". No epsilon ever decides anything. This is [Lecture 4](./04-intervals.md) and the rules of [Lecture 5](./05-honest-numerics.md), and it is the answer to problem 1. It is what a tolerance would be if it were computed rather than chosen.

**Every curve and surface is a NURBS.** No line class, plane class, cylinder class or torus class. A plane is a degree-1 NURBS surface. A cylinder is a surface that is a rational quadratic one way and linear the other. No algorithm branches on what kind of surface it was given. From `AGENTS.md`, the repository's rulebook:

> Every curve and surface stays a NURBS, and no algorithm branches on what kind of surface it was given: no plane × cylinder intersection formula, no offset of a revolution through its profile, no torus fillet because the faces happen to be coaxial.

This is an unusual choice. A traditional kernel has one analytic surface type per common shape, with intersection formulas for many pairs, and falls back on a general NURBS code path for the rest. Those formulas are fast and exact, and every one of them is a separate code path with its own corner cases: the plane × cylinder formula has separate branches for a plane parallel to the axis, tangent, perpendicular and oblique. The mature kernels are, as `AGENTS.md` puts it, full of such branches, and Geop means to "port the *configurations* they handle (tangency, vanishing faces, seams), never the type dispatch". When the general path is wrong or too wide for a case, it is the general path that gets fixed. Since intersection and tracing are fast for linear cases anyway, a plane special case buys little. The kernel is meant to stay small: "a compressed B-rep kernel, simple rather than special-cased".

Where something does need to know a shape, as a user's sketch does when it wants to project a circular edge as a circle rather than a spline, the shape is *recognized* from the control points, locally, at the place it is used (`as_line`, `as_arc`, `as_plane` in `core/geop-core-geometry/src/shape.rs`), and never stored as a kind in the B-rep.

**Subdivide to isolate, Newton to refine.** Intersection is not a matter of solving formulas, but of a global search that uses the NURBS control points' hull to discard regions with proof, and a local polish with Newton's method. Parts II and III.

**A part is a program.** The stored object is not a model but a list of steps that build it, each step with arguments that refer to what earlier steps built by stable names. [Lecture 2](./02-programs-of-steps.md).

**Refuse by name.** When an operation can't handle a configuration it says which entities and which condition, instead of returning a subtly invalid solid. For example, the shell, asked to do what it does not support, says that "face … is taken away where it meets face … at an inward corner, along edge …, and the walls grow outward: not supported" (`lib/geop-lib-shell/src/shell.rs`).

<div class="kernel">

**In the kernel.** The rulebook is `AGENTS.md`. Despite its name, written for the coding agents working on the repository, it is the best single document about how this kernel thinks. Its sections are hard-won lessons, each from a real bug, and we teach many of them. `DEVELOPERS.md` describes the crate layout, the continuous integration and the scripts.

</div>

## How we got here

`DEVELOPERS.md` opens with a candid list of what did not work. It is worth reading because each attempt rejected something you may be tempted by.

1. **Geop 1** used lines, arcs, spheres, tori and spirals as primitives, with explicit intersection formulas. It "didn't scale as equations became too complex". Interval arithmetic was already there, "and 2d booleans worked reliably".
2. **Geop 2** went into algebra: symbolic math, ideals, Gröbner bases. It was "too slow and too much overhead", and the decision was made to use only numerical methods from then on.
3. **Geop 3** focused on numerical methods, with first proofs of concept for intersecting NURBS and solving nonlinear equations.
4. **Geop 4** "got stuck in numerical solvers and topology. Booleans looked promising but tracing was too difficult, too many edge cases."
5. **Geop 5** went back to subdivision only, with fat lines and fat surfaces in place of convex hulls, intersection curves never traced explicitly, and no topology, only a bag of bounded patches.

The kernel you are reading about differs in turn from Geop 5: it keeps the fat-line clipping ([Lecture 13](./13-fat-lines.md)), but it does have full topology and it does trace intersection curves ([Lecture 16](./16-tracing-and-pcurves.md)). The lesson is not that any of these ideas was foolish, but rather how long it takes to learn which complexity is essential.

## A tour of the repository

Geop is a Rust workspace in four tiers, with a web front end on top. The rule of the layout is that **a crate's place follows from what it knows**.

| Tier | What it knows | Examples |
|---|---|---|
| `core/` | geometry with no notion of an operation; what a program and an editor are | `core/geop-core-math`, `core/geop-core-geometry`, `core/geop-core-topology`, `core/geop-core-sketch`, `core/geop-core-rasterize`, `core/geop-core-program`, `core/geop-core-editor` |
| `lib/` | algorithms two operations share, taking geometry and returning geometry | `lib/geop-lib-booleans`, `lib/geop-lib-sweep`, `lib/geop-lib-shell`, `lib/geop-lib-constraints` |
| `ops/` | one crate per family of operations, with its forms, tests and examples | `ops/geop-ops-fillet`, `ops/geop-ops-extrude-revolve`, `ops/geop-ops-assembly`, `ops/geop-ops-step`, and the shared vocabulary `ops/geop-ops-model` |
| `cad/` | composition: everything together, for a browser and a command line | `cad/geop-cad-base`, `cad/geop-cad-web`, `cad/geop-cad-cli` |
| `web/` | a React and three.js front end, which loads the kernel as WebAssembly and knows operations by key alone | `web/src/App.tsx`, `web/src/SceneViewer.tsx` |

As of this writing, that is 7 core crates, 7 lib crates, 21 op crates and 3 cad crates, and about 250 000 lines of Rust, tests included (a line count of `core`, `lib`, `ops` and `cad`).

Dependencies point down only. An operation is a *leaf*: it may use `core/`, `lib/` and `ops/geop-ops-model`, never another op crate. Thus a change to the fillet cannot break the shell. `scripts/check-layering.sh` checks these rules in continuous integration, and its allow-list is empty. Composition happens in one place, `cad/geop-cad-base/src/registry.rs`, which calls one `register` per op crate, in one line each. [Lecture 40](./40-writing-an-operation.md) shows what such a crate looks like.

A map of every crate is in [Appendix C](../appendix/crates.md). For the bottom of the stack, here is how the layers of knowledge stack up, with the part of this book that teaches each.

```text
a part as a program           Part IX     core/geop-core-program, core/geop-core-editor
operations                    Part VII    ops/*, lib/geop-lib-sweep, lib/geop-lib-shell
booleans                      Part V      lib/geop-lib-booleans
sketches                      Part VI     core/geop-core-sketch
B-rep topology                Part IV     core/geop-core-topology
intersection                  Part III    core/geop-core-geometry/src/intersection
NURBS curves and surfaces     Part II     core/geop-core-geometry
numbers                       Part I      core/geop-core-math
```

## What Geop does not do

Honest accounts include what is missing. `docs/cad-gap.md` compares Geop with Fusion 360, Creo, Onshape, NX and SolidWorks, and marks each capability done, partial or missing. `ROADMAP.md` states its yardstick: the features an engineer reaches for when designing a real machine such as a robot, leaving out "CAM, CAE, rendering, PDM". Individual operations refuse configurations they do not support, and we will mention those as we come to them. Geop is also younger than the kernels it is compared with: `docs/smlib-comparison/README.md` measures itself against about 870 000 lines of mature C++ and lists where it is behind as well as ahead.

## The plan of this book

- **Part I, numbers you can trust** (Lectures 3 to 5): floating point, intervals, and the rules that keep interval answers honest.
- **Part II, curves and surfaces** (6 to 11): Bézier curves, B-splines, NURBS and exact circles, subdivision, surfaces, and evaluating them accurately far from the origin.
- **Part III, intersection** (12 to 16): the global search, fat-line clipping, Newton, tangency and overlap, tracing curves.
- **Part IV, topology** (17 to 20): the B-rep's data structure, poles and seams, point classification and validation.
- **Part V, booleans** (21 and 22): how all of the above turns into the operation everyone has heard of.
- **Part VI, sketches** (23 to 25): solving constraints, and proving the solution.
- **Part VII, operations** (26 to 35): sweep, loft, fillet, shell and the rest.
- **Part VIII, assemblies** (36 and 37): mates and the solver behind them.
- **Part IX, the program model** (38 to 40): runs, incremental builds, the editor and how to write an operation.
- **Part X, the craft** (41 and 42): performance and debugging, from real case studies.

Read it in order. The lectures have a goal box at the start, and a recap and exercises at the end; do the exercises, since some of them prove things the text then uses. The demos are real computations: drag things, and see whether what you see is what the text said.

<div class="recap">

## Recap

- A kernel holds an exact description of a solid and answers questions and performs operations on it; everything about windows and buttons lies outside.
- A boundary representation describes a solid by exact geometry plus discrete topology. Unlike meshes, fields and CSG trees, it has both exact measurements and addressable faces and edges.
- The three hard problems are consistent decisions from inexact numbers, finding all of an intersection, and keeping topology consistent with geometry that never quite fits.
- Geop's answers: every number an interval, every curve and surface a NURBS, subdivision to isolate and Newton to refine, a part as a program, refusal by name.
- Crates are layered by what they know, the layering is machine-checked, and operations depend on no other operation.

</div>

## Exercises

**1.** Count the vertices, edges and faces of a box with a square hole through it, top to bottom. Check the Euler–Poincaré formula `$V-E+F-(L-F)=2(S-G)$`, where `$L$` is the number of loops, `$S$` the number of shells and `$G$` the genus (the number of through holes). Why does the formula need `$L$`, when `$V-E+F=2$` works for a plain box?

<details class="solution"><summary>Solution</summary>

`$V=16$`, `$E=24$`. Faces: four outer sides, four walls of the hole, top and bottom, so `$F=10$`. Top and bottom have two loops each, the other eight have one, so `$L=12$`. Then `$16-24+10-(12-10)=0$`, and `$2(S-G)=2(1-1)=0$`. For a plain box `$L=F$`, so that term vanishes. It is needed because a face with a hole is not a disc: the classic Euler formula counts discs. This is a useful consistency check of a B-rep, but only a necessary condition.

</details>

**2.** A circle of radius `$r$` is replaced by the inscribed regular `$n$`-gon. Show that the maximum distance between them is `$r(1-\cos(\pi/n))$`, and how many segments it takes to stay within `$10^{-6}$` for `$r=50$`. How many triangles does that imply for a sphere of that radius, roughly?

<details class="solution"><summary>Solution</summary>

The largest gap is at the middle of a chord, whose half-angle is `$\pi/n$`, at distance `$r\cos(\pi/n)$` from the centre. With `$1-\cos\theta\approx\theta^2/2$`, we need `$50\pi^2/(2n^2)\le10^{-6}$`, or `$n\ge\pi\sqrt{50/(2\cdot10^{-6})}\approx15\,708$`. A sphere meshed with about `$n$` segments around and `$n/2$` latitude rows has about `$n^2$` triangles, some `$2.5\times10^8$`. The error falls only as `$1/n^2$` and the storage grows as `$n^2$`: every extra digit of accuracy costs ten times the triangles.

</details>

**3.** The rounded box above is the set of points within distance `$r=0.2$` of a box of sides `$x=1.6,y=1.1,z=0.6$`. Derive its volume by adding the pieces: the inner box, six slabs, twelve quarter cylinders, eight sphere octants. Check that you get `$2.8562$`. Why is the mesh's `$2.85566$` *less* than this, always?

<details class="solution"><summary>Solution</summary>

`$V=xyz+2r(xy+yz+zx)+\pi r^2(x+y+z)+\tfrac43\pi r^3$`. The terms are `$1.056+1.352+0.41469+0.03351=2.8562$`. The vertices of the mesh are on the solid's surface, and each facet is a chord of a convex surface, so each lies inside the solid. The mesh is thus a polyhedron inscribed in a convex solid, of smaller volume. (For a non-convex solid, the sign of the error depends on where you look.)

</details>

**4.** In a CSG system, how would you decide whether a point lies inside `$(A\cup B)\setminus C$`, given a point-inside test for each primitive? Now list what you would need to know to *draw the edges* of the same solid. Which of those can the tree give you without intersection computations?

<details class="solution"><summary>Hint</summary>

Membership is the boolean expression `$(a\lor b)\land\lnot c$` evaluated on three point tests. Drawing edges takes knowing where the primitives' surfaces meet, and then which parts of those curves lie on the final boundary (classification). The tree alone provides neither, so "evaluating the boundary" of a CSG tree is the same problem as a B-rep boolean.

</details>

**5.** Predict from the tier rule which of these a crate in `ops/` may depend on: `core/geop-core-topology`, `lib/geop-lib-booleans`, `ops/geop-ops-sketch`, `cad/geop-cad-base`. Then read `scripts/check-layering.sh` and check. Where does a function go that both the fillet and the shell need?

<details class="solution"><summary>Solution</summary>

An op crate may use `core/` and `lib/` crates and `geop-ops-model`, so the first two are allowed. Another op like `geop-ops-sketch` is not, except as an optional dependency of the `examples` or `testing` feature, which the script reports as a warning. Nothing but `cad/` may depend on `cad/`. A function both operations need goes in a `geop-lib-*` crate if it is an algorithm, or in `geop-ops-model` if it is shared vocabulary such as a target and its references.

</details>

**6.** A traditional kernel stores a tolerance on each edge: "this edge lies within `$10^{-5}$` of its faces". Think of one way this could be wrong to compare with an interval enclosure. (Think about who chooses the number, and what happens when edges with different tolerances meet at a vertex.)

<details class="solution"><summary>Hint</summary>

A chosen number need not be related to what the computation established: too small, and the claim is false; too big, and two distinct features merge. At a vertex, a kernel must inflate the vertex's tolerance to cover its edges, which then affects everything else touching that vertex. An enclosure is computed by arithmetic which proves it. Still, it does not make the problem disappear, as an enclosure can grow too wide to be useful, which [Lecture 5](./05-honest-numerics.md) treats as a bug to find rather than a number to raise.

</details>
