# Lecture 32: Patterns, deformations, holes and threads

<div class="goals">

**After this lecture you can**

- say why moving a body by a rigid motion or any affine map costs nothing, and what a mirror must do besides;
- explain why a pattern of a feature copies the feature's tool instead of running its step again, and what that gives up;
- compose a Bézier piece with a taper, twist or bend on its control points, predict the degree of the result, and bound its error;
- describe how a hole and a modelled thread are built, and when a thread is only recorded;
- name the four quantities that grow when one body takes a hundred booleans, using the spur gear as the example.

</div>

The operations of the last few lectures made shape: a sweep, a blend, a moved face. This one is about operations that *reuse* shape. A bolt circle is one hole done eight times; a gear is one gap done a hundred times; a twisted blade is a straight blade with every point moved by a formula. Two questions run through all of them. How much of the exact representation survives the map? And what grows when the same thing is done many times?

## Maps of space

Let `$M$` be a map of space. Applying it to a body means applying it to every vertex, every edge's curve and every face's surface. For a NURBS curve `$C(t)=\sum_i R_i(t)P_i$` with `$R_i$` the rational basis of [Lecture 8](./08-nurbs.md), the question is when `$M(C(t))$` is again a NURBS, and how big.

**Affine maps** are the free case. If `$M(x)=Ax+b$`, then because the `$R_i$` sum to one,

`$$M\bigl(C(t)\bigr)=A\sum_i R_iP_i+b\sum_i R_i=\sum_i R_i\,M(P_i).$$`

Same knots, same weights, same degree, moved control points. A rotation, a translation, a uniform or non-uniform scaling, a shear and a mirror are all of this kind. Note the parameters do not change: `$M(C(t))$` is the *same* `$t$`, so a pcurve that traced an edge on a surface before the move still traces it after. Nothing in `$(u,v)$` space has to be touched.

In the kernel this is `Model::transform_body` (`core/geop-core-topology/src/edit/transform_body.rs`). It takes a `Deformation`, a trait with `point`, `curve` and `surface` methods (`core/geop-core-geometry/src/deform.rs`), and `Motion` from `core/geop-core-math/src/primitives/pose.rs` implements that trait by mapping control points. One function moves a body, however it is being moved.

**Orientation** is the only catch. Geop stores no flag saying which side of a face is outside: the outward normal is `$S_u\times S_v$`, a fact about the parametrization ([Lecture 17](./17-brep.md)). A mirror reverses the handedness of space, so `$S_u\times S_v$` of the image points *into* the image solid. So `transform_body` asks the map `Deformation::mirrors`, and if it does, it also calls `Model::reverse_face` on every face, which reverses the surface in `$u$` and mirrors its pcurves the same way. This is why the trait has that method at all.

### Repeating bodies

`ops/geop-ops-pattern` offers `LinearPattern`, `CircularPattern`, `Pattern`, `Mirror`, `MoveBody`, `ScaleBody` and `DeformBody`. A pattern is a list of frames plus a seed. The list is the type `Frames` (`ops/geop-ops-model/src/spacing/frames.rs`): `Frames::Linear`, `Frames::Circular`, `Frames::Curve`, `Frames::Points`, and the others its doc comment lists, offsets from a table, a filled face, the holes features drilled. Where each copy goes is decided in one place, and what to do at each frame in another.

Copies are kept as new bodies or combined with a solid (`Combine`), as an extrude's tool is. Names follow the copy: `pattern(P,i,X)` for entity `X` in the copy labelled `i`, `i.j` in a grid. A name is a statement a later step can rely on, so it is built from labels of *where* the copy went, not from the order in which something happened to find it.

## A pattern of a feature

A hole is not a body, it is the absence of one. To repeat a hole, what do we repeat?

The obvious answer is to run the hole's step again with moved arguments. Geop deliberately does not, and the reason is worth following. A step's arguments belong to the program, and a step sees only the part the steps before it built ([Lecture 2](./02-programs-of-steps.md)). Running a step again would mean moving a sketch, a picked face and a set of hole centres, each operation in its own way, and the name of a picked face means nothing at the place where the copy is.

So a step that combines tools with a solid *records what it combined*. The record is `Feature` (`ops/geop-ops-model/src/feature.rs`), a list of `FeatureTool`, each holding the tool as a `BodySpec`, the entity names, the `BooleanOp` and the tool's reach. A pattern of the feature then copies those tools, moves them with `transform_body`, and combines each by the one boolean every combining step already uses (`combine_tools` in `lib/geop-lib-booleans/src/combine.rs`). Holes, extrudes, revolves, sweeps, lofts, ribs and patterns themselves can then all be patterned, with no code of their own for it. See `ops/geop-ops-pattern/src/features.rs`, whose module comment gives this reasoning.

The price is an end condition chosen at the copy's place. A tool that goes "up to next" is recorded with its start and end faces named, and its copy is cut back to the next face where it is. But a tool that went "through all" reaches only as far past the solid as it did where it was built, and a blind one goes as deep. If the solid is thicker at the copy, the copy does not go further. An extrude's fallbacks are not tried again; a copy nothing stops all round is refused.

This is a typical kernel trade. A rule that is simple and always the same costs a little expressiveness at the corners, and the refusal tells you where.

## Deformations are not affine

Now take maps that are not affine. `DeformBody` tapers, pinches, shears, twists or bends a body about an axis, over a zone `$[t_0,t_1]$` along it. In a frame whose third axis is the deformation's axis, the kernel's `AxialMap` writes all of them in one form:

`$$(x,y,t)\ \longmapsto\ x\,a(t)+y\,b(t)+c(t),$$`

affine in `$x,y$`, with coefficient vectors that depend on `$t$` alone. For `$s=t-t_0$`, zone length `$L=t_1-t_0$`:

| map | across the axis | formula |
|---|---|---|
| taper to factor `$f$` | scale by `$1+\frac{f-1}{L}s$` | `$x\to x(1+\tfrac{f-1}{L}s),\ y\to y(1+\tfrac{f-1}{L}s),\ t\to t$` |
| twist by `$\Theta$`, `$\omega=\Theta/L$` | turn by `$\omega s$` | `$x\to x\cos\omega s-y\sin\omega s,\ y\to x\sin\omega s+y\cos\omega s,\ t\to t$` |
| bend by `$\Theta$`, `$R=L/\Theta$` | carry the plane round a circle | `$x\to x\cos\omega s+R(1-\cos\omega s),\ y\to y,\ t\to t_0+(R-x)\sin\omega s$` |

Check the bend. On the axis, `$x=0$`, the point goes to `$(R(1-\cos\omega s),\,t_0+R\sin\omega s)$`: a circle of radius `$R$` about `$(R,t_0)$`, and the arc length from the start is `$R\omega s=s$`, so the axis is wound onto the arc with its length kept. A point at `$x$` towards the centre stays at distance `$R-x$` from it. A plane across the axis stays a plane, turned with it.

Past the zone, the map continues as the affine map it is tangent to at the zone's end (`Motion::affine`): carried on rigidly for a twist or bend, scaled for a taper. The operation first divides the body where it crosses the zone's ends, so each piece lies in one regime, and refuses a zone end through a corner: that would ask the imprint to cut through a vertex, the configuration of "Never place a cut on a corner that already exists" in AGENTS.md.

### Composition by products

On a Bézier piece, `$x(u),y(u),t(u)$` are polynomials in `$u$` (take all weights one for now). If `$a(t)$` is a polynomial of degree `$d$`, then `$a(t(u))$` has degree `$dn$`, where `$n$` is the piece's degree, and the product `$x(u)\,a(t(u))$` has degree `$(d+1)n$`.

And that product needs no sampling at all, since Bernstein polynomials multiply in closed form ([Lecture 6](./06-bezier.md), exercise 4):

`$$B_i^mB_k^n=\frac{\binom{m}{i}\binom{n}{k}}{\binom{m+n}{i+k}}\,B_{i+k}^{m+n}.$$`

This is `core/geop-core-geometry/src/bernstein.rs`: a `Bernstein` holds a polynomial's coefficients over a rectangle, and `Bernstein::product`, `Bernstein::power` and `Bernstein::combination` do arithmetic on them. No evaluation, no fit, and each operation widens by one rounding per coefficient.

With weights the same thing works in the homogeneous form. Write the piece as `$X(u)/W(u)$`, and a coefficient as `$\sum_k f_k(t-t_c)^k$` about the piece's middle `$t_c$`. With `$U=T-t_cW$`, the term `$(t-t_c)^k$` becomes `$U^k/W^k$`, and multiplying by `$W^d/W^d$` gives the composition

`$$a(t)=\frac{\sum_k f_k\,U^k\,W^{d-k}}{W^d},$$`

so the image is a rational piece of numerator degree `$(d+1)n$` and weight `$W^{d+1}$`. The parametrization is the same `$u$`, so pcurves on the deformed faces need no change at all, as with an affine map. That is the design's main gain: the operation never fits a pcurve.

**The degree multiplies.** Directly from the formula:

| map | `$d$` | a line (`$n=1$`) | a quarter arc (`$n=2$`) | a cubic (`$n=3$`) |
|---|---|---|---|---|
| taper, pinch | 1 | 2 | 4 | 6 |
| twist, bend | 3 | 4 | 8 | 12 |

(A shear is affine everywhere; `AxialMap::as_motion` recognizes that and applies it as a `Motion`, so its degree does not change at all.) `AxialMap::interpolant_degree` returns 1 or 3. On a surface the same holds along each direction, as the products are over a rectangle.

There is one more case the code handles. A face across the axis, whose whole piece spans almost nothing along `$t$`, has coefficients that are constant to within what the target can see, and is composed with `$d=0$`: the map is affine on it, and the plane stays a plane of the same degree.

### Worked example: a taper of a straight edge

Take `$L=2$`, `$f=\tfrac12$`, so `$a(t)=1-t/4$`. Take an edge in the plane `$y=0$` from `$(x,t)=(1,0)$` to `$(2,2)$`, with `$x(u)$` and `$t(u)$` linear. Then `$a(t(u))=1-u/2$`, which has Bernstein coefficients `$[1,\tfrac12]$`, and `$x(u)$` has `$[1,2]$`. By the product formula with `$m=n=1$`:

`$$c_0=1\cdot 1=1,\qquad c_1=\tfrac12\bigl(1\cdot\tfrac12+2\cdot 1\bigr)=\tfrac54,\qquad c_2=2\cdot\tfrac12=1.$$`

So `$X(u)=B_0^2+\tfrac54B_1^2+B_2^2$`. At `$u=\tfrac12$` this gives `$\tfrac14+\tfrac58+\tfrac14=\tfrac98$`, and directly `$x\,a=\tfrac32\cdot\tfrac34=\tfrac98$`. A straight edge has become an exact parabola, of degree 2 as the table says. A taper is exact: no approximation at all, only the outward rounding of the arithmetic.

### When the map is not polynomial

A twist's coefficient is `$\cos\omega s$`, which is no polynomial of any degree. So the kernel replaces it, over the piece's reach along the axis, by the cubic that interpolates it at Chebyshev points, and it *proves* how far off that cubic can be. `AxialFunction::interpolant` writes `$\sigma=(t-t_c)/h$` with `$h$` the half reach, and uses the classical remainder for polynomial interpolation at nodes `$\sigma_j$`:

`$$\bigl|f(\sigma)-p(\sigma)\bigr|\ \le\ \frac{\max\lvert f^{(4)}\rvert}{4!}\ \max_{\sigma\in[-1,1]}\Bigl\lvert\prod_{j=0}^3(\sigma-\sigma_j)\Bigr\rvert.$$`

For `$f=c\cos(\omega h\sigma)+s\sin(\omega h\sigma)$` the fourth derivative is at most `$(|c|+|s|)(\omega h)^4$`. For Chebyshev nodes the product's maximum is exactly `$2^{-3}=\tfrac18$`; the kernel, which proves everything it uses, bounds it by the largest Bernstein coefficient of the node polynomial over sixteen pieces of `$[-1,1]$`, which comes out slightly above that. Both are valid; only one is computed in intervals.

That error is on the coefficient, so multiplied by what `$x$` and `$y$` reach over the piece it bounds how far the composed point can be from the true image. **Every control point is then widened by it.** The result is a proper enclosure of the true image, not an approximation that "looks right": the rule that a value is honest, from [Lecture 5](./05-honest-numerics.md).

The bound falls as `$h^4$`, so a piece that is too coarse is halved, along the direction it runs most along the axis, until the bound falls below a target (`RELATIVE_TARGET`, `1e-9` of the geometry's size, a target for how finely to cut, not a tolerance any answer relies on). The pieces are joined back at knots of full multiplicity, the shared control points being the union of the two enclosures. A curve or surface that would need more than `MAX_PIECES` (4096) pieces is refused by name.

Numbers for a sense of scale: a twist of `$90°$` over a piece as long as the zone has `$\omega h=\pi/4$`, so the coefficient's bound is `$(\pi/4)^4/4!/8\approx 2.0\times10^{-3}$`. To get below `$10^{-9}$` that must fall by `$2\times10^6$`, a factor of 16 per halving: six halvings, so about 64 pieces along the axis. The demo shows this behaviour directly.

<figure class="demo" data-demo="deform-poly">
<figcaption>Drag the cubic's control points (grey, up the axis). Blue and green: the image composed on the control points, one colour per piece; its control polygon is much longer than the source's since the degree multiplies. Orange circles: the true map applied to samples, one by one, which the composition must pass through. Change <em>pieces</em> and watch the bound drop by 16 per halving. Simplified from the kernel: a polynomial curve in one plane, the whole curve inside the zone, equal pieces rather than halving where the bound is worst, and floats rather than intervals.</figcaption>
</figure>

<div class="kernel">

**In the kernel.** The whole mechanism is `core/geop-core-geometry/src/deform.rs`: `AxialMap::taper`, `AxialMap::shear`, `AxialMap::twist`, `AxialMap::bend`, with `AxialMap` implementing `Deformation`, so the operation reaches it through `transform_body`, exactly as a rigid move does. `ops/geop-ops-pattern/src/deform_body.rs` holds the operation: it imprints the planes at the zone's ends (`AxialMap::plane_across`) and calls the map. Note `Deformation::affine`: a map that is not affine first cuts a face's surface down to what the face uses, because its cost grows with the size of what it maps. A taper's factor must be definitely positive, and a bend of a whole turn or more is refused.

</div>

## Holes

A hole wizard is a table lookup and a revolve. `ops/geop-ops-hole/src/iso.rs` holds the coarse metric series of ISO 261 from M1.6 to M24, with pitch, tap drill (ISO 2306), three clearance fits (ISO 273), counterbore and countersink. For example, the row for M6 reads: pitch 1, tap drill 5, clearance 6.4, 6.6 or 7, counterbore 11 (and head depth `$k=D=6$`), countersink 12.6. `ops/geop-ops-hole/src/pipe.rs` does the same for NPT from 1/16 to 2 inch, whose walls taper 1 in 16 on the diameter.

The shape is a half section in `$(r,z)$`, revolved a full turn into a solid tool (`hole_tool` in `ops/geop-ops-hole/src/hole.rs`), then cut away by `combine_tools`. The section's curves are named `top`, `counterbore`, `shoulder`, `countersink`, `wall`, `bottom` or `point`, and each names the faces it leaves. A drill's point is a cone of `DRILL_POINT_ANGLE`, `118°`.

Nothing here is special. The top of the tool is a disc that the cut takes away, *past* the face being drilled, so the tool's face does not lie on the part's. It is the principle of [Lecture 29](./29-fillets.md): a tool's own surfaces must cross the solid's faces, not lie on them.

## Threads: recorded or modelled

A modelled M6 thread has ten turns per centimetre of bolt, and its four-sided section sweeps four faces per quarter turn, so sixteen faces per turn, each of them cut into the part by a boolean's intersection curves. A part with fifty bolts would spend all its time on helices that nobody needs to see. So the common case is not to model a thread at all. A tapped hole or a `Thread` step by default records a `CosmeticThread` (`ops/geop-ops-model/src/threads.rs`): its designation, the axis, radius, taper and length, kept as geometry, so that it says the same whatever later steps do to the face. Drawings and the viewer read it (`draw_threads`). The solid is left alone.

Asked to model it (`ThreadArgs::modelled`), `thread_tool` in `ops/geop-ops-hole/src/thread.rs` sweeps a section along a helix and cuts it away. The section is a four-sided polygon in `$(r,z)$`. Both thread families are based on a 60° triangle of height

`$$H=\frac{\sqrt3}{2}P,$$`

(`TRIANGLE_HEIGHT`), cut flat at crests and roots. The basic minor diameter of an ISO metric thread is `$D-\tfrac54H$` (`MetricSize::minor_diameter`, after ISO 724). For M6, with `$P=1$`, that is `$6-1.25\cdot0.866=4.917$`, against a tap drill of 5.

Three details of the construction are each a lesson from an earlier lecture.

**The tool runs past the wall.** A bolt's thread is cut by the groove between its teeth, and that groove's outer end lies a sixteenth of `$H$` beyond the shaft's surface, so it *cuts through* the wall rather than along it. For a pipe thread, whose truncation is smaller, half of that truncation is used instead, because at the sharp V's base a tooth is a whole pitch wide and consecutive turns would touch.

**The tool stays narrower than a pitch.** Otherwise turns touch, and a body touching itself is not a solid. `screw` in `lib/geop-lib-sweep/src/revolve.rs` states this condition in its doc.

**Where it starts is a free choice, so choose well.** `clear_start` turns the helix's start away from the corners of the wall's faces, for the reason you know: a cut that lands on an existing corner is a degenerate configuration. On a tapered wall the section's corners are adjusted by the taper times their height, since a corner half a pitch up lies that much nearer the wall; without this a pipe thread's corner came within `2e-3` of the wall (a comment in `thread_tool`).

The path is `Helix` (`core/geop-core-geometry/src/nurb_curve/helix.rs`), built in spans of at most a quarter turn of rational quadratics: seen down the axis it is exactly the kernel's circle, and only the height is approximate. Its module comment gives the error: at most 0.53% of the pitch for quarter-turn spans, 5 µm on an M6. So a modelled thread is not exactly a helicoid, but it is closer than the thread's own tolerance; the figure is measured by a test in that file. `Helix::tapered` moves the profile outward as it rises, for NPT. The sweep itself is the subject of [Lecture 27](./27-path-sweep.md).

A thread can also be drawn rather than taken from a table: a section on the wall, a helix in a 3-D sketch, and a sweep along it, which the kernel carries by the same screw motion, a quarter turn at a time (`cut_helices`). The example `square_thread` is that: a rectangle standing on a revolved cylinder, swept five turns and joined to it, the wall it stands on lying in the cylinder's. It also found a bug. Where the helix met the cylinder's rim, the two edges run about three degrees apart, and validation called them coincident; [Lecture 15](./15-tangency-and-overlap.md) tells that story.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"square_thread","example":"square_thread"}'>
<figcaption>The example <code>square_thread</code>, as the kernel meshed it. Drag to turn it.</figcaption>
</figure>

<figure class="demo" data-demo="model-viewer" data-args='{"model":"cross_drilled_shaft","example":"cross_drilled_shaft"}'>
<figcaption>A stepped shaft revolved about its axis, with a circle extruded through both sides and cut away, as the kernel meshes it. The tool crosses the shaft's wall in a curve neither surface knows about: a traced intersection, from Lecture 16.</figcaption>
</figure>

## Many booleans on one body

The standard library's `spur_gear` (`lib/geop-lib-stdlib/src/drive.rs`) makes a gear in a way that sounds naive: extrude a disc, sketch one gap, and subtract a `CircularPattern` of it. "A boolean per tooth, the price of a tooth count that is a parameter like any other", as its comment says. It supports up to `MOST_TEETH` = 120. Getting there took finding four things, each harmless once, that grow with each cut. AGENTS.md's section *Many booleans on one body* records them, and you should know them, because they recur whenever one body takes many operations.

**1. Names.** A piece of a split edge is named after the edge and the vertex it starts at; that vertex is named after the edge again. So the name doubles per cut, and twenty cuts make names of tens of megabytes. Geop now spells a name's argument longer than `LONGEST_ARGUMENT` (160 bytes, in `ops/geop-ops-model/src/names.rs`) by its digest. Nobody thinks of a name as a resource until it is one.

**2. Rescans.** The piercing search started over from every edge × face pair after each split. But a pair once found clean stays clean: a piece of a curve crosses nothing the whole did not. So it is not asked again.

**3. Width along a split edge.** This is the deep one. An edge was cut at the *whole* refined enclosure of where it reaches the vertex. That parameter's width went through Boehm's `$\alpha=(t-a)/(b-a)$` ([Lecture 9](./09-subdivision.md)) into the pieces' control points and end knot; the next cut's parameter, found on the wider piece, came out wider, and the width grew threefold per cut. A quarter arc cut forty times went from `3e-15` to `0.08` in AGENTS.md's measurement. A disc's rim cut by thirty-two slots ended `4e-10` below its cap, and gears stopped at 80 teeth, with rims drawn in twelve arcs to spread the cuts.

The fix follows the rule that sharpening is allowed exactly where the value is a free choice. Where in the enclosure to cut *is* a free choice: every parameter in it ends a piece on the curve and within the vertex. So `split_parameter_at_point` (`core/geop-core-geometry/src/nurb_curve/refine.rs`) cuts at its sharp middle and checks the curve reaches the vertex there. Each cut then adds one subdivision's rounding, linear growth rather than exponential, and the rim can be a plain circle again. The current gear's rim is one circle in four quarters, ending on multiples of 90°, placed (per the comment in `spur_gear`) at tooth centres or at least 0.015 of a module from where any gap of up to 120 teeth crosses it.

**4. Traced fits.** An involute's curvature changes tenfold along the flank, so the cubic through the march's points drifted a hundred times the usual. The legs are now split finer while the drift is what makes the curve wide (see [Lecture 16](./16-tracing-and-pcurves.md)).

There is a fifth trap, and it is not in the boolean at all. A sketch whose arcs' sweeps must be solved for can be sent off to infinity by the solver, if a parameter puts its points far from where they were drawn ([Lecture 23](./23-sketch-solver.md)). So the gear's gap is drawn in lines and Bézier flanks only, every point placed by a formula: "a sketch the solver places in one step for any size". The root is therefore a chord of the root circle, at most `$r(1-\cos(\pi/2z))$` below it, 0.04 of a module for 12 teeth, and the code says so. An honest small deviation, chosen for robustness and documented, against a fragile exact arc.

The flanks themselves (`lib/geop-lib-stdlib/src/involute.rs`) are worth a look. The involute of a circle is no polynomial, so a flank is its Taylor polynomial of degree `DEGREE` = 9 about where it starts, written as a Bézier curve, its two ends put back onto the involute exactly. `Gap::deviation` is a bound from the series' tail, not a sample, and a test confirms the flank follows the involute within it. A gear has `$z\ge$` `FEWEST_TEETH` = 12, since with fewer the flanks undercut. The gap also reaches past the disc on every side it leaves by, so that no gap's face lies on the disc's.

<div class="recap">

## Recap

- Affine maps act on control points alone: same knots, weights, degree and pcurves. A mirror must also turn every face around.
- A pattern of a feature copies the tool the feature's step recorded and combines it again. It cannot choose a new end condition at the copy.
- An axial deformation is `$x\,a(t)+y\,b(t)+c(t)$`. Composition by Bernstein products multiplies degree by `$d+1$`, keeps the parametrization, and is exact for a taper; twist and bend interpolate at Chebyshev points with a proven bound that widens the result.
- Holes are tables and a revolved tool. A thread is by default a record; modelled, it is a section swept along a helix that crosses the wall.
- One body taking many booleans exposes what grows per cut: names, rescans, width and traced drift.

</div>

## Exercises

**1.** A rational quarter arc in a plane through the axis is twisted. What are its numerator degree and its weight's degree? What about after tapering it?

<details class="solution"><summary>Solution</summary>

The arc has `$n=2$`. Twist: `$d=3$`, so numerator degree `$(d+1)n=8$` and weight `$W^{4}$`, also of degree 8. Taper: `$d=1$`, numerator degree 4, weight `$W^2$` of degree 4. The weights remain positive, since `$W$` was.

</details>

**2.** Verify the table's bend formula: show that the Jacobian of the bend at `$x=0$`, restricted to the axis direction, has length 1, and at distance `$x$` towards the centre has length `$1-x/R$`. What does this say about a point at `$x>R$`, and what does the operation's doc say about it?

<details class="solution"><summary>Solution</summary>

Differentiating `$(x\cos\omega s+R(1-\cos\omega s),\ t_0+(R-x)\sin\omega s)$` in `$s$` with `$\omega=1/R$`: `$((R-x)\omega\sin\omega s,\ (R-x)\omega\cos\omega s)$`, of length `$(R-x)/R=1-x/R$`. At `$x=R$` the map folds everything to the centre; past it, the material would be wound past its centre and inside out. `DeformBody`'s doc refuses "a bend ... that would wind material past its centre"; `AxialMap` checks it in `check_axis` on the range `$x$` reaches over the piece.

</details>

**3.** Using the deform-poly demo, set a twist of 170° and read the bound for 1, 2, 4 and 8 pieces. Do the ratios agree with `$16$`? Does the measured deviation fall at the same rate? Why might the measured deviation be far below the bound?

<details class="solution"><summary>Hint</summary>

For a curve whose reach along the axis halves with each cut, the bound falls by 16. With equal pieces in `$u$` and a non-uniform `$t(u)$` the worst piece decides, so the ratio is only near 16. The measured value is below the bound because the bound takes `$|x|$` at its worst over the piece and the node polynomial at its worst, and these do not need to happen together.

</details>

**4.** `RELATIVE_TARGET` is `1e-9`. Is that an epsilon in the sense AGENTS.md forbids? State what would change in the *meaning* of the result if the constant were `1e-6`.

<details class="solution"><summary>Solution</summary>

No. It decides how finely to cut, not what any comparison answers. With `1e-6` there would be fewer pieces, and each control point would be widened by a larger bound. The result would still be a valid enclosure of the true image, only wider; later searches would see that width. It is a tunable of "how hard to try", as `max_nodes` and `min_subdivision_size` are.

</details>

**5.** Why does `Frames` live in `ops/geop-ops-model` and not in `ops/geop-ops-pattern`? What other crate would otherwise have to depend on the pattern crate? (Look at who uses `geop_ops_model::spacing`.)

<details class="solution"><summary>Hint</summary>

Operations may not depend on one another (`scripts/check-layering.sh`). Holes, and any operation reading points or spacings, need the same vocabulary. See the section "Where things go" in AGENTS.md and [Lecture 40](./40-writing-an-operation.md).

</details>

**6.** A designer wants a thread on a cylinder of diameter 6.05 tagged M6. Read `ThreadPlacement::check_fits` and decide whether it is accepted. What does this tell you about where the kernel draws the line between a design decision and an error?

<details class="solution"><summary>Hint</summary>

Read the function. It compares the wall with the standard's diameters and taper, and refusals name the face and the condition: see "Refuse what you do not support" in AGENTS.md.

</details>

**7.** In the gear's case, width grew threefold per cut before the fix. Suppose the first cut has parameter width `$w_0$` and each cut multiplies it by `$g$`. How many cuts until the width reaches a target `$w_{\max}$`? With `$g=3$`, `$w_0=3\times10^{-15}$`, `$w_{\max}=10^{-4}$`, what do you get? What does additive growth of one rounding per cut (`$\approx 10^{-16}$`) give?

<details class="solution"><summary>Solution</summary>

`$k=\log(w_{\max}/w_0)/\log g=\log(3.3\times10^{10})/\log3\approx22$` cuts for geometric growth. With additive growth, `$(10^{-4}-3\times10^{-15})/10^{-16}\approx10^{12}$` cuts: never. This is the difference between a feature that works up to some count and a feature that scales.

</details>
