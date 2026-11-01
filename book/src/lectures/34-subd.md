# Lecture 34: Subdivision surfaces

<div class="goals">

**After this lecture you can**

- apply one step of Catmull–Clark subdivision by hand, including at an extraordinary vertex and along a crease;
- explain why on a regular grid it reproduces the uniform bicubic B-spline, and what breaks at an extraordinary vertex;
- state exactly how Geop turns a cage into a B-rep of B-spline faces, and which part of it is an approximation;
- derive the shapes of the kernel's patch masks from the regular case;
- describe the box modelling tools, why names of cage elements must survive edits, and what the mug's handle taught the classifier.

</div>

Sweeps, lofts and blends all start from curves. But an organic form, a mouse's hump or a mug's handle, is awkward to build from sections: you think of it as a lump you push around. A **subdivision surface** works that way. You edit a coarse mesh, the **control cage**, and the smooth surface is defined by a rule for repeatedly refining it. Topology is free, so a handle is just two faces extruded and bridged.

There is a tension for a kernel with our rules. A subdivision surface's limit is defined by a *process*, and near some vertices its limit is not a polynomial patch at all. Everything in this kernel is a NURBS ([Lecture 1](./01-what-a-kernel-is.md)). How that is reconciled, and at what cost, is the heart of this lecture.

## Why bicubic B-splines subdivide

Start in one dimension, with what you already know. A uniform cubic B-spline with control points `$\dots,P_{i-1},P_i,P_{i+1},\dots$` can have a knot inserted midway in every span ([Lecture 9](./09-subdivision.md)). The new control points are given by two masks:

`$$E_i=\tfrac12(P_i+P_{i+1})\ \ \text{(edge points)},\qquad V_i=\tfrac18(P_{i-1}+6P_i+P_{i+1})\ \ \text{(vertex points)}.$$`

These are local averaging rules, and repeating them converges to the spline itself. Every point goes to its limit position: the spline at knot `$i$` is `$\tfrac16(P_{i-1}+4P_i+P_{i+1})$`.

For a tensor-product bicubic the masks are products. On a regular grid, where each vertex has four faces around it:

| | mask | |
|---|---|---|
| face point | `$\tfrac12\otimes\tfrac12$` | the average of a quad's 4 corners |
| edge point | `$\tfrac12\otimes\tfrac18(1\ 6\ 1)$` | |
| vertex point | `$\tfrac18(1\ 6\ 1)\otimes\tfrac18(1\ 6\ 1)$` | |

Write the edge mask out: `$\tfrac1{16}$` of `$(1,6,1)$` along one side and along the other. Its value equals the average of the edge's two ends and the two adjacent faces' *face points*, as you can verify on paper. Write the vertex mask with `$F$` the four face points, `$E$` the far ends of the four edges: it equals `$\tfrac1{16}(\sum F+\sum E+8V)$`.

The insight of Catmull and Clark (1978) is that these expressions *do not mention the grid*. They mention faces, edges, and how many faces meet at a vertex. So they make sense for a mesh of any topology.

## The Catmull–Clark rules

Geop's rules are in `ops/geop-ops-subd/src/subdivide.rs`. One step, for a mesh of faces of any number of sides:

**Face point.** The average of the face's corners.

**Edge point.** The average of the edge's two ends and its two faces' face points.

**Vertex point.** For a vertex `$V$` with `$n$` faces around it, `$F_j$` their face points and `$E_j$` the far ends of its edges:

`$$V'=\frac{\sum_jF_j+\sum_jE_j+n(n-2)\,V}{n^2}.$$`

(Textbooks write this `$(\bar F+2\bar R+(n-3)V)/n$` with `$\bar R$` the mean of the edges' midpoints. It is the same thing: `$2\bar R=V+\bar E$`.)

**Connect.** A face of `$k$` sides becomes `$k$` quads around its face point, each running from one of its corners through the edge points on the two sides there. So after one step, every face is a quad, whatever the cage was made of, and the only irregularity left is at vertices with `$n\ne4$`: **extraordinary vertices**. Their number never changes after the first step: new vertices all have `$n=4$`.

### Worked example

Take the cube with corners `$(\pm1,\pm1,\pm1)$`, at `$V=(1,1,1)$`, with `$n=3$`.

The face points of the three faces at `$V$` are `$(1,0,0),(0,1,0),(0,0,1)$`, so `$\sum F=(1,1,1)$`. The far ends are `$(-1,1,1),(1,-1,1),(1,1,-1)$`, so `$\sum E=(1,1,1)$`. Then

`$$V'=\frac{(1,1,1)+(1,1,1)+3\cdot1\cdot(1,1,1)}{9}=\frac59(1,1,1).$$`

The edge from `$(1,1,1)$` to `$(1,1,-1)$` has face points `$(1,0,0)$` and `$(0,1,0)$`, so its edge point is `$\tfrac14\bigl((2,2,0)+(1,1,0)\bigr)=(\tfrac34,\tfrac34,0)$`. A cube shrinks towards a sphere-like shape. It is not a sphere: try the demo.

<figure class="demo" data-demo="catmull-clark">
<figcaption>Drag to turn it. Dashed orange: the cage. Move <em>level</em> to run the rules above, or show "one step, then patches": what the kernel actually builds, each bicubic patch sampled here for drawing. Crease the top rim (red) and it becomes a sharp edge: the surface above stays smooth. The L block has two pentagons, so its first step leaves more extraordinary vertices. Simplified from the kernel: closed cages only, sharp creases only, no soft ones.</figcaption>
</figure>

Its math is checked in `book/scripts/demo_tests/catmull-clark.test.mjs`: on a torus of random points (every vertex regular), one step gives exactly the tensor masks above, the limit points are `$\tfrac16(1\ 4\ 1)$` each way, and the patches below are exactly the bicubic B-spline.

### The limit surface

Three facts about the limit, stated without proof (see the references in [Appendix A](../appendix/references.md)).

Away from extraordinary vertices, after one or two steps the mesh is regular, so the limit there is precisely bicubic B-spline: `$C^2$`, polynomial. At an extraordinary vertex with `$n\ne4$`, the limit is `$C^1$` but in general only that, and in the vicinity it is made of infinitely many polynomial rings shrinking towards the vertex, so it is **not** a finite set of polynomial patches. Stam showed in 1998 how to evaluate it exactly from an eigendecomposition of the subdivision matrix, but what it evaluates is not a NURBS.

So a kernel in which every surface is a NURBS cannot hold the exact limit surface near such a vertex. Something must give.

## Creases

Real parts have sharp edges. Mark some edges of the cage as creased, and change the rules so a creased edge is its own cubic B-spline curve:

- **Edge point on a crease:** the midpoint.
- **Vertex point**, by how many sharp edges meet at it (`VertexKind`):
  - none or one (a *dart*): the smooth rule;
  - two: `$\tfrac18(A+6V+B)$`, with `$A,B$` the far ends along the crease;
  - three or more: a *corner*, which stays where it is.

Look at the crease rule: it is the one-dimensional cubic mask. The crease curve thus subdivides as a B-spline by itself, independently of the surface on either side. The limit has a sharp edge along it. A mesh's boundary counts as creased.

A **soft** crease has sharpness `$s<1$` (`Crease`). Its edge point blends `$(1-s)$` of the smooth rule with `$s$` of the sharp one, and the vertex is moved correspondingly by the mean sharpness of its creases; after that one step, its halves are smooth. The result "rounds off tighter there, without a corner". This is simpler than the semi-sharp creases of some other systems, in which sharpness decays over several steps. Here it acts on the first step only, because there *is* only one step, as we now see.

## From a cage to a B-rep

You might expect a kernel to subdivide several times and fit something. It does not. The pipeline (`ops/geop-ops-subd/src/lib.rs`) is:

1. **One** Catmull–Clark step (`subdivide`), after which every face is a quad.
2. For each of those quads, **one bicubic Bézier patch** (`control_net` in `ops/geop-ops-subd/src/limit.rs`).
3. Those patches as a closed solid or an open sheet (`ops/geop-ops-subd/src/brep.rs`).

Step 2 is Loop and Schaefer's approximation of Catmull–Clark surfaces by bicubic patches (2008, "ACC"). Let us derive its masks from what they must do on a regular grid.

A bicubic B-spline span, written in Bézier form, has 16 control points. Three kinds:

**Corner: a limit position.** In the regular case, `$\tfrac1{36}(1\ 4\ 1)\otimes(1\ 4\ 1)$`, which is `$\tfrac{1}{36}(16V+4\sum E+\sum D)$`, with `$D$` the diagonal corners of the four faces. ACC generalizes `$16$` to `$n^2$` and `$36$` to `$n(n+5)$`:

`$$\text{corner}=\frac{n^2V+4\sum_jE_j+\sum_jD_j}{n(n+5)}.$$`

This is the true limit position, for any `$n$`: exact even at an extraordinary vertex. That formula is the one in `control_net`.

**Interior point near a corner.** The regular case is `$\tfrac19(4,2,2,1)$` over the four corners of the quad, with 4 at the near corner. In ACC the near corner's weight becomes `$n$`:

`$$\text{interior}=\frac{n\,P_0+2P_1+2P_3+P_2}{n+5}.$$`

For `$n=4$` it is exactly the bicubic's. For `$n\ne4$` it is the approximation.

**Edge point near a vertex.** The average of the two interior points beside it, across the edge. So the two patches sharing an edge use the same control points along it: **they share their boundary curve exactly**, and a B-rep can be built with a single shared edge, closed with no gaps. This matters more to us than anything else in the construction. Interval-valid topology requires that an edge lie on both its faces.

Then one correction. Near a smooth extraordinary vertex, that edge point is **projected into the limit tangent plane**, whose normal `tangent_normal` computes from the eigenvector masks of Catmull–Clark subdivision. With that, the normal is continuous at every vertex.

The resulting surface, per the comment in `limit.rs`:

> where all four corners of a quad and its neighbours are regular — smooth, four faces each — it gives exactly the uniform bicubic B-spline that is the limit surface there. Around an extraordinary vertex it is an approximation: the patches meet along shared curves, with a continuous normal at every vertex [...] but across the edges out of an extraordinary vertex the normal turns by a few degrees.

So **be exact about what is claimed**. The kernel's surface is:

- exactly the Catmull–Clark limit on regular regions;
- correct at extraordinary vertices' positions (corner points are true limit positions);
- not the limit near an extraordinary vertex, by some small amount, and only `$G^0$` across the edges leaving it, with the normal turning a few degrees.

There is **no** fill, tilt, or N-sided patch construction of the kind in [Lecture 28](./28-lofts-and-fills.md) and [Lecture 29](./29-fillets.md) involved. A subd body in Geop is simply this set of bicubic patches. Whether a designer sees those few degrees depends on the view. A renderer would hide it with Loop and Schaefer's separate normal patches; the kernel, which has to be honest about geometry, just has the surface it has.

What is gained is that everything downstream works. A subd's body is an ordinary B-rep of NURBS. A fillet, a boolean, a shell or a draft takes it without a line of special code. No branch anywhere says "if subd". That is why "Everything is a NURBS" pushes this design.

### Creases in the patches

For creased edges the same scheme applies in sectors (the `sector_sizes` function): the interior point's weight `$n$` is "how many faces if this sector were mirrored into a regular vertex": `$n$` at a smooth vertex, twice the sector's size along a crease, four times at a corner. And the crease's own Bézier edge points are `$\tfrac13(2V+W)$`, the cubic B-spline in Bézier form, so a crease curve is a true piecewise cubic.

A crease's limit corner is `$\tfrac16(A+4V+B)$`, the 1-D limit mask, and a corner stays at `$V$`.

### One face per cage quad

The B-rep should have the cage's own topology: that is what a user picks and names. Each cage quad became four patches, so `brep.rs` joins them: one bicubic B-spline of `$7\times7$` control points over `$[0,2]^2$`, its inner knots triple (`cubic_knots`). Triple inner knots are what lets each of the four patches keep its own control points in one spline. Every cage edge is one edge, a cubic of two spans. Every cage vertex is one vertex at its limit position.

A face with `$k\ne4$` sides cannot be one tensor-product surface, so it stays as the patches it became, one face per corner, joined at its face point. Its sides are split at their middles.

Names follow the cage: the face of cage face `f7` is `subd(S,f7)`, and for an n-gon, `subd(S,f7,v3)` at its corner `v3`. And cage elements have ids that are kept through every edit (`CageVertex`, `CageFace`, `next_id`), so a later step that referred to `subd(S,f7)` still finds it after a loop cut elsewhere. An edit "keeps the id of every element it does not take away, and gives what it adds ids no element has had" (`ops/geop-ops-subd/src/edit.rs`).

<figure class="demo" data-demo="model-viewer" data-args='{"model":"subd_mouse","example":"subd_mouse"}'>
<figcaption>A mouse from half a box cage, mirrored in x = 0, its hump raised and the rim of its sole creased. The sole is one of the examples of a surface whose coordinates fail where the geometry is fine: see below.</figcaption>
</figure>

## Box modelling

`ops/geop-ops-subd/src/edit.rs` has what you would expect from a box modeller, each a pure function of the cage and its arguments: `Cage::cuboid`, `Cage::cylinder`, `Cage::sphere` and `Cage::plane` to start; `Cage::extrude`, `Cage::inset`, `Cage::bevel`, `Cage::insert_loop` for a loop cut, `Cage::edge_slide`, `Cage::merge_vertices`, `Cage::dissolve_edges`, `Cage::dissolve_vertices`, `Cage::bridge`, `Cage::fill_hole`, `Cage::set_sharpness`, `Cage::delete_faces`, `Cage::halve` at a mirror plane and `Cage::unmirrored`. A `Mirror` lets the cage hold one half, the other being a mirror image joined along vertices on the plane: `v3` and its image `v3m`.

The operation `Subd` is a single step of a program holding the whole `Cage` as its argument. So in the program model the whole of a mug is a single step; edits change the step's argument and rebuild.

Note what is not stored: the sequence of edits. A step's arguments are the result. Each edit is deterministic, its order fixed by the ids ("A hash map's order is no order", in AGENTS.md).

## Coordinates fail where geometry does not

Bicubic patches derived mechanically can have bad coordinates at their corners, and two examples from this crate are instructive.

**The mouse's sole.** The mouse's sole is flat, its rim creased. At the corners of the sole, where the crease runs on smoothly through a vertex with two sharp edges, the patch's two sides leave the corner along one line: `$S_u$` and `$S_v$` are collinear and opposite. The regression test `sole_of_the_mouse_has_a_normal_at_its_corners` (`ops/geop-ops-subd/src/regression_tests.rs`) records the numbers: `$|S_u\times S_v|$` is `1.5e-14` where `$|S_u|$` and `$|S_v|$` are both `0.52`. The surface is flat there, a plane; the *coordinates* are singular. The fix is not in this crate at all but in `NurbSurface::corner_normal` (`core/geop-core-geometry/src/nurb_surface/normal.rs`), general surface code. With `$S_v=\mu S_u$` at the corner, next to it `$S_u\times S_v$` is to first order `$du\,S_u\times(S_{uv}-\mu S_{uu})+dv\,S_u\times(S_{vv}-\mu S_{uv})$`. The corner has a normal exactly when those two terms agree in direction, and it is theirs; where they do not, that is an error, not an arbitrary pick. This is "Degenerate parametrizations: the surface is fine, the coordinates are not", from [Lecture 18](./18-poles-seams-splicing.md).

**The mug's handle.** AGENTS.md tells this under "A probe is a sample too". Which side of a face the solid lies on was once decided by classifying a point a small distance off the face. Now it is decided by the parity of a ray's crossings past the face (`solid_contains_beside`, [Lecture 19](./19-classification.md)), and each ray's crossings are kept with the verdict (`Beside`) and named in the error. "That is what showed the subd mug's handle patch to be a ribbon folded onto itself, a ray leaving it crossing its other half `6e-7` on, which no probe had seen."

Consider what that means. A cage edit can produce a cage whose patch *folds over itself* so tightly that its two sheets are `6e-7` apart. Bicubic patches of a handle, with their control net bent sharply enough, do this. A probe a distance `$\delta$` off the surface with `$\delta>6\times10^{-7}$` reaches right past the second sheet and reports nonsense silently. The exact ray reports the real geometry: this surface is folded.

The cage edits have their own open defect, kept as a reproducer as AGENTS.md requires. `loop_cut_across_the_mug_handle_builds_a_valid_body` is `#[ignore]`d: a loop cut across the mug's handle builds an invalid body. Its comment records what is known: the ring around the handle visits an edge twice, so the cut is laid into it twice over; and what is ruled out: the crease, and the cut's position (0.3 of the way is still invalid). This is what a known limit looks like when written down honestly: not fixed, not forgotten, and the next session starts from evidence.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"subd_mug","example":"subd_mug"}'>
<figcaption>The mug, half an eight-sided cylinder mirrored in y = 0, its handle two side faces extruded and bridged, its inside the top inset and extruded down, the bottom's rim creased sharp and the lip creased soft.</figcaption>
</figure>

<div class="recap">

## Recap

- Catmull–Clark generalizes the bicubic B-spline's refinement masks to any mesh; on regular regions the limit *is* bicubic.
- After one step all faces are quads, and only `$n\ne4$` vertices are irregular. The limit near them is `$C^1$` but not a finite set of polynomial patches.
- Geop takes one step, then one bicubic patch per quad, Loop and Schaefer's ACC. It is exact on regular regions and at limit positions, an approximation near extraordinary vertices, and its normal turns a few degrees across edges leaving them.
- Patches share boundary curves exactly, so it is an ordinary closed B-rep of NURBS, on which every other operation works.
- Creases are 1-D masks on the crease's own curve. Cage ids survive edits so names stay put.
- A surface can fold in on itself at `6e-7`: only an exact ray notices.

</div>

## Exercises

**1.** At the cube's corner `$V=(1,1,1)$`, compute the limit position by the ACC corner formula, and compare with `$\tfrac59(1,1,1)$` from the first step. Then apply the rule to the *first step's* mesh, where the vertex is still `$n=3$`, and describe how one would compute a limit position of the original cage.

<details class="solution"><summary>Hint</summary>

The formula takes the once-subdivided mesh: `$V=\tfrac59(1,1,1)$`, its edge neighbours the edge points such as `$(\tfrac34,\tfrac34,0)$`, its diagonals the face points such as `$(0,0,1)$`. Work it out with `$n=3$`, dividing by `$n(n+5)=24$`: you should get `$\tfrac12(1,1,1)$`. The demo's "one step, then patches" draws the patches from exactly that corner.

</details>

**2.** Show that the edge rule is the tensor mask on a regular grid. Name the six points involved and write each one's weight both ways.

<details class="solution"><summary>Solution</summary>

An edge from `$A$` to `$B$` with neighbouring rows `$C,D$` beside `$A,B$` on one side and `$E,F$` on the other. Face points: `$\tfrac14(A+B+C+D)$` and `$\tfrac14(A+B+E+F)$`. So `$\tfrac14(A+B+\text{faces})=\tfrac14A+\tfrac14B+\tfrac1{16}(2A+2B+C+D+E+F)=\tfrac38(A+B)+\tfrac1{16}(C+D+E+F)$`. The tensor `$\tfrac12(1,1)\otimes\tfrac18(1,6,1)$`, gives weight `$\tfrac{6}{16}=\tfrac38$` on `$A,B$`, `$\tfrac1{16}$` on `$C,D,E,F$`. The same.

</details>

**3.** How many faces, edges and vertices does a cube have after `$k$` steps? Verify Euler's formula `$V-E+F=2$` for `$k=2$`. How many extraordinary vertices are there at each level? Check with the demo's readout.

<details class="solution"><summary>Solution</summary>

`$F_k=6\cdot4^k$`, `$E_k=2F_k$` since all quads with each edge shared, `$V_k=2+F_k$` from Euler. For `$k=2$`: `$F=96,E=192,V=98$`, `$98-192+96=2$`. There are 8 extraordinary vertices (`$n=3$`) at every level.

</details>

**4.** A cage quad has `$n=5$` at one corner. Using the ACC formula, what is the weight on that corner in the interior point, and what would it be in the regular case? In which direction does this move the interior point, and how does this affect the surface's shape near the vertex?

<details class="solution"><summary>Hint</summary>

`$\tfrac5{10}$` vs. `$\tfrac49$`. A greater weight pulls the interior control point closer to the vertex. Compare with the 3-valent case: `$\tfrac38$`.

</details>

**5.** Explain, with reference to [Lecture 17](./17-brep.md)'s invariant, why it matters more to this kernel that patches share boundaries exactly than that the normal be continuous across them. What would validation report if adjacent patches used different edge control points?

<details class="solution"><summary>Solution</summary>

An edge is one curve lying on both its faces. If the two boundaries differed, the single edge could enclose only one of them, or be widened until it enclosed both, and its pcurves would no longer trace it: `curve_and_surface_sampling_check` and the gap checks in validation would report it. A slight crease in normals, on the other hand, is a true geometric property the rest of the kernel handles like any other edge between faces at an angle.

</details>

**6.** On the mouse's sole, the corner has `$S_u$` and `$S_v$` collinear. Write down a small bicubic patch with this property that is nevertheless a flat, valid region, and calculate its normal at that corner by the first-order terms. Why is the corner normal a *surface* question and not one for the subd crate?

<details class="solution"><summary>Hint</summary>

Take a flat patch with control points `$b_{ij}$` in the plane `$z=0$`, with `$b_{10}-b_{00}$` parallel to `$b_{01}-b_{00}$`. `$S_u\times S_v$` vanishes at the corner, and the first-order terms given in the lecture have a direction. In a flat patch both terms are normal to the plane, so they agree, and the normal is `$(0,0,\pm1)$`. Any NURBS can have such a corner, from any operation; hence `NurbSurface::corner_normal` belongs to the surface code.

</details>
