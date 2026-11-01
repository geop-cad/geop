# Lecture 29: Fillets and chamfers

<div class="goals">

**After this lecture you can**

- define a fillet as what a rolling ball sweeps, and find the ball's centre and contacts without ever building an offset surface;
- write the blend's section as an exact rational arc, and the tool a boolean cuts with;
- explain why an interpolated blend must not be tangent, and how a millionth of a radian fixes that;
- explain why the same code path still gives exact cylinders and tori where they exist;
- choose how a blend ends at a corner, one vertex at a time;
- name four ways a tool's own curves can lie in the solid's faces, and why each breaks the boolean.

</div>

Sharp edges concentrate stress and are unpleasant to touch, so most real parts have their edges rounded or bevelled. These operations look small and are among the hardest a kernel has.

The reason is that a fillet is defined by a *condition*, tangency to two surfaces at once along a moving contact, and no formula satisfies it for general surfaces. In the kernel it runs to thousands of lines, `ops/geop-ops-fillet/src/rolling.rs` being the largest file. This lecture explains the ideas, with the kernel's own numbers.

## The rolling ball

Take an edge between two faces. Imagine a ball of radius `$r$` that rolls along it, touching both faces all the time, on the material's side of them for a convex edge (it lies inside the solid), and on the air's side for a concave one. Then:

- the ball's centre moves along a curve called the **spine**, where the two faces' offsets by `$r$` meet;
- where it touches each face it traces a **contact curve**, the spine projected back onto the face;
- the surface swept between the contact curves is the **fillet**.

Before looking at surfaces, look at a section. In the plane square to the edge the problem is two dimensional.

<figure class="demo" data-demo="rolling-ball" data-args='{"mode":"section"}'>
<figcaption>A ball between two curved "faces" meeting at the corner, in section. Drag the control points and the radius. The centre is where the two offset curves (dashed) cross, but the demo never builds them to find it: it runs Gauss-Newton on "r from each curve, along the normal at the foot point", as the kernel's <code>meet</code> does. Past the curves' ends (dotted), a contact means the ball is too big for its face.</figcaption>
</figure>

### Finding the ball without an offset surface

The obvious algorithm is to offset both faces and intersect them, but an offset of a free-form NURBS is no NURBS ([Lecture 31](./31-offsets.md)). There is a neater way. Write the condition on the centre `$C$` directly, using the fact that the gradient of the distance to a surface is the surface's normal at the foot point:

`$$n_a\cdot(C-\mathrm{foot}_a(C))=r,\qquad n_b\cdot(C-\mathrm{foot}_b(C))=r,$$`

plus a third condition that places the ball along the edge:

`$$t\cdot(C-E)=0,$$`

`$E$` being the station's point on the edge, and `$t$` the edge's tangent there. Three equations in three unknowns, whose Jacobian rows are `$n_a,n_b,t$` (for a foot point moving with `$C$`, the distance's gradient is exactly the normal, so the moving foot adds nothing at first order). So this is a Gauss-Newton iteration requiring nothing but foot-point projection, which we already have. It is exact in the sense that matters: each condition is checked at the end.

<div class="kernel">

**In the kernel.** This is `meet` in `core/geop-core-geometry/src/meet.rs`, with `Offset` conditions for the faces and one `planes` entry for the edge's square plane. It takes "the shortest step that corrects all of them at once, to first order" through `shortest_solution`, and "between planes that is exact in one step". As the doc comment says, it checks the answer against every condition before returning (AGENTS.md's *Check an iterative answer against every condition it was asked to meet*, and [Lecture 30](./30-one-rebuild.md)), and keeps two copies, per Lecture 5: sharpened seeds for iterating, the unsharpened sum of steps as the answer.

An instructive failure lives behind it. AGENTS.md's *Newton needs the second derivatives where it is far from flat*: `NurbSurface::project` used to step by Gauss-Newton on the foot point, the first fundamental form alone. Over a curved surface, that overshoots by the target's distance over the radius of curvature, so a target twice the radius off a cylinder swings from side to side of its foot for ever. A pocket's rim rounded with the radius its corner's fill was rounded with put the ball's centre exactly there. The step is now Newton's on the squared distance wherever its Hessian is definitely positive definite.

</div>

### The section is an exact arc

At a station we have `$C$` and contacts `$T_a,T_b$`. Between them, the ball's section is an arc of its great circle in the plane of `$C,T_a,T_b$`. Lecture 8 gives the arc in closed form: a rational quadratic from `$T_a$` to `$T_b$`, with middle control point `$E$` where the two tangents meet, and middle weight the cosine of half the angle it turns.

Let's work through the simplest corner. In section, a convex 90° edge at the origin, faces running along `$-x$` and `$-y$`, material in the quadrant `$x<0,y<0$`. Then

`$$C=(-r,-r),\quad T_a=(-r,0),\quad T_b=(0,-r),\quad E=(0,0),\quad w=\frac{r}{\lvert E-C\rvert}=\frac{\sqrt2}{2}.$$`

At the arc's middle, `$\dfrac{\tfrac14T_a+\tfrac{w}{2}E+\tfrac14T_b}{\tfrac12+\tfrac w2}=-\dfrac{r/4}{0.8536}(1,1)=-0.2929r\,(1,1)$`, and `$C+\tfrac{r}{\sqrt2}(1,1)$` is the same point.

For two faces at interior angle `$\varphi$`, `$\lvert E-C\rvert=r/\sin(\varphi/2)$`, so `$w=\sin(\varphi/2)$`. This agrees with the cosine of half the arc's turn, which is `$\pi-\varphi$`.

### The tool

Geop does not edit the faces to make a fillet. It builds a solid **tool** covering just the material to be removed (or, on a concave edge, added), and runs a boolean. That reuses everything we built in Part V: the two faces are trimmed along the contact curves, by the boolean, and nothing else needs to know how.

A fillet's tool, in section, is the arc plus two lines from its ends to the apex `$Q=2E-C$`, on the far side of the corner. So at the worked corner above, `$Q=(r,r)$`. A chamfer's is the triangle on the chord, with the chord extended past both ends. The blend is then one face of the tool, and the rest of the tool lies in air. Stations of the section are carried along the edge by the grid of Lecture 26, here through `lib/geop-lib-sweep/src/skin.rs` as `StripGeometry`.

<figure class="demo" data-demo="rolling-ball" data-args='{"mode":"rim"}'>
<figcaption>A ball rolled round the rim of a disc: top plane <code>z = 0</code>, wall a cylinder of radius 2. The spine is the dashed circle, the contact curves blue and orange, and the red arc is the current section. Here, as in the kernel, the blend is exactly a piece of a torus, because the section is carried round the axis unchanged.</figcaption>
</figure>

## From stations to a surface

The kernel places stations along the edge and skins through them. Every control point of the section, multiplied by a weight we shall come to, is interpolated along the edge by one cubic, at the same parameters (`NurbCurve::interpolate_homogeneous`). The result is rational quadratic across and cubic along, passing through every station's arc exactly.

Between stations it only approximates the rolling ball, and the question is by how much. So the kernel rolls the ball to points between the stations, and measures. The stations are doubled from `FIRST_STATIONS` (8 per edge) until the skin follows the ball to within `DEVIATION` of the radius, which is `$10^{-6}$`, or up to `MOST_STATIONS` (256), and failing that the edge is refused, saying how far it got.

Contact curves are the one place where an approximation "must not lie". They bound the faces the boolean trims, so they are widened to enclose the true contact curves, by `NurbCurve::enclosing_pad`, and the blend's boundary rows go with them.

## Why an approximate tangency must not be tangent

Now comes the most important idea of the lecture. Consider the skin's direction into the fillet at the contact curve. At a station it lies exactly in the face's tangent plane. Between stations, it is an interpolant of those directions, and the face's normal turns in its own way. Let `$\mathrm{into}(s)$` be the skin's direction across the contact, `$n(s)$` the face's normal, and

`$$f(s)=\mathrm{into}(s)\cdot n(s).$$`

At every station `$f=0$`. Between stations, `$f$` is the interpolation error projected on the normal, and an interpolation error changes sign. So **the blend crosses the face as often as not, between stations, by a hair**. Tangency to within the approximation's error is no tangency to the boolean. It will try to cut along every such crossing, and what comes out are near-duplicate vertices next to the contact and "nothing to split" errors. That is what AGENTS.md calls *An approximate tangency dips through the face*.

More stations do not fix it. They shrink the dips, but they do not remove them. The fix is to stop aiming for tangency. At each station, turn the arc by a tiny angle towards the ball (`TILT` is `$10^{-6}$` radians), so that `$f=+\tau$`, and `$f(s)=\tau+\text{error}(s)$` stays positive once the error is smaller than `$\tau$`. Then the blend leaves the face in its contact curve alone. The boolean meets a clean transversal crossing at a tiny angle: "a crease of a millionth of a radian nobody sees, where exact tangency was never to be had".

<figure class="demo" data-demo="rolling-ball" data-args='{"mode":"tilt"}'>
<figcaption>The dip, computed. A contact line whose face normal turns as <code>φ(s) = 0.5 sin 5s</code>, and a C² cubic spline through the section's exact direction at the stations (dots). Plotted is <code>f(s) = into · n</code>, autoscaled: the coloured half is "through the face". Raise the spans: the dips shrink (fourth order) but never go away. Turn the tilt on, and it clears them once the tilt exceeds the error. A model of one component of the skin, not the kernel's skin; the mechanism is the same.</figcaption>
</figure>

### Exact where it can be

A tilt of `$10^{-6}$` must not be applied where it is not needed. An exact tangency needs no tilt and must not get one. A cylinder round a straight edge between planes is exactly tangent, and if it got a tilt, then a revolved handle's rim would show two silhouettes per face where the crease ran, and a draft would no longer see the fillet as tangent (both from AGENTS.md).

The kernel used to handle that by separate paths: extruded along a straight edge between planes, revolved round a circle, and rolled everywhere else. AGENTS.md's rule says one rolled path must be as good. The trick that makes it so is `along_weight`:

**The sections are interpolated multiplied by the edge's weight at their parameter.** Take an edge that is a rational quadratic, say an arc, with weight function `$W(t)$`. A section carried unchanged round it has homogeneous points `$W(t)\,R(t)p$`, where `$R(t)$` is the turn at `$t$`. A rational parametrization of a turn is itself a ratio by that `$W$`, so `$W(t)R(t)p$` is a *polynomial* in `$t$`, of the edge's degree. A cubic through enough stations reproduces a polynomial of degree at most 3 exactly. Hence, without any branch on surface kinds, the same interpolation yields exactly the swept section.

So `span_is_exact` then asks a purely numerical question: between stations, does the untilted skin agree with the ball to rounding (`EXACT` is `DEVIATION` squared, so `$10^{-12}$`, relative to the radius or the distance from the origin)? If so it stays untilted, and "as plain as the edge, one Bézier piece of the lowest degree that is still the ball". A cubic through ten stations of a straight blend would be exact too, but AGENTS.md notes that its offset in a shell came out `1.2e-4` wide. The tilted and untilted tools are built with the same apex, "the untilted arc's either way", so that where they cross they meet in their apex edges rather than a tilt apart.

### Station breaks where the data is only C1

Stations are placed "half a step off" the vertices of a chain. Where a face's curvature jumps across an edge, or a radius changes its rate at a vertex, the data is only `$C^1$`, and no number of stations will make a `$C^2$` cubic follow it to `$10^{-6}$`. Don't try. Put a station exactly there and break the spline. The ball's contact on the edge between the two faces is found on that edge, not near it (`crossing`), and "the tool's spline breaks there". Use the structure you know before you sample blindly.

## Chains

An edge rarely stands alone. A circle is four quarter arcs, a slot's rim is lines and arcs. Blending one quarter would leave a mess at its ends, so `tangent_chain` follows every edge it runs on into tangentially, between faces running on tangentially, and the ball rolls along all of it. A closed chain has no ends. A crease inside a chain is refused: "a ball rolling over it would touch the crease's edge itself".

## Ends: decided at each vertex, on its own

An open chain ends at a corner, and how it should end depends on what's there. AGENTS.md's *Refuse what you do not support* notes that each end is decided on its own, since one rule for the whole edge was wrong for one of the cases in each combination. `tool_end` in `ops/geop-ops-fillet/src/blend.rs` returns an `End`:

- **`End::RunOut`**: the edge leaves the solid through the third face, and the tool is a cut. Run out past the face, "any face, the tool running on into the air", by `run_out_length`.
- **`End::Flush` and `End::Wall`**: material added must not stick out past the face, and a cut must not cut what lies behind a wall it runs into. The tool's end must lie in the face. That needs the face to be a plane square to the edge. If not, the blend runs on past the face and is cut off by the face's surface (`RunOn`).
- **`End::Corner`**: every edge at the corner is filleted with one radius. The fillets end at the ball touching all three faces, and the corner is rounded by the ball's piece, a spherical triangle between three great arcs (`ops/geop-ops-fillet/src/corner.rs`), and the three tools are joined into one solid there. The ball's centre is `meet` with three offset conditions.
- **`End::Vanish`**: the two faces meet tangentially at the vertex, such as two equal pipes joined where they touch. The blend narrows to nothing, "every section's apex is drawn in as the section narrows", and the tool ends in that point.

At an **inward corner**, a pocket's rim, two convex edges run into each other's walls. Each tool ends flush with the other's wall, and that leaves the corner between the walls uncut. The cut there is found with no mitre surface at all: each tool is run on past the corner, and the corner is cut by the *intersection* of the two. "Behind both walls, what both would cut" (`mitre`).

<figure class="demo" data-demo="model-viewer" data-args='{"model":"rounded_box","example":"rounded_box"}'>
<figcaption>The kernel's <code>rounded_box</code>: a 2 x 1.5 x 1 box with every edge rounded by 0.2, in one step. Each corner is a piece of the ball touching its three faces. Open it in the editor and inspect the faces of the step named <code>round</code>.</figcaption>
</figure>

## A tool's own curves must not lie in the solid's faces

A boolean can cross a curve through a surface. What it cannot settle within budget is a curve that lies *in* the surface while crossing that face's boundary: an overlap instead of a crossing, and the search exhausts itself. That general point came up four times in the fillet (AGENTS.md), and each is a clear lesson.

1. **A rolled chamfer's run-out** ended its last span with a station at the chain's end vertex, its section in the third face's plane. The chord crossed that face's boundary. The run-out is now part of the last span (a repeated knot), so the face is crossed by the span's interior.
2. **A straight chamfer built as a whole tool** used the chord's ends on the faces, exact lines lying in them. Extending the chord past the faces made it cross them instead. (A fillet's contact curves do lie on faces, but tangentially, and padded to enclose the true contact.)
3. **Mitring a rolled blend** at an inward corner was tried, running it on straight from its station at the vertex. Where the walls stand square to the shared face, that station's section lies in the *other* wall's plane, its curves on that wall's face. It failed in the boolean. The intersection of two tools above needs no mitre at all.
4. **A cap in a wall's plane**. Ending flush puts the tool's cap in the wall's plane, and the cap's surface was the box around its section, `FlatPatch::around`, with sides through the contacts. When a pocket's upright edge is rounded with the floor edge it stands on by one radius, the upright fillet's contact on the wall runs exactly along the floor tool's cap side, `1.6e-12` off it, "which no search tells from an overlap". Where a patch ends is a free choice, yet reaching a third of its size past the section only moved the side onto a stepped block's chamfer contact: a fraction of one design dimension lands on another. The cap now reaches past the extent of every edge of the model (`FlatPatch::past`), "where nothing can lie along it".

It helps to think about all four as relatives of AGENTS.md's *Never place a cut on a corner that already exists*. Choosing a position which a design dimension could coincide with is choosing a degeneracy.

## Two more ways of reading a whole from a point

From AGENTS.md's *Past a face is not off its surface*. A blend run on past the third face it leaves through was allowed to follow the ball loosely there, since "what it does in the air decides nothing". But a contact past the face can still lie on the face's *surface*, which runs on past its trim. Round a hole breaking out through a block's bottom, the wall's quarter goes on below the block to the circle's lowest point. Unenclosed, the contact ran a hair off that surface, neither on it nor clear of it, and the boolean's search for crossings never ended. So a curve a boolean will meet with a surface must enclose the truth wherever it could touch that surface, not only where the face is.

The same part showed two more:

- A patch's side was judged to be within reach by the speed at the vertex, and a quarter arc, faster in its middle, was not carried on past its end. Take the least the surface runs on the way.
- A traced edge carried on past its end drifts off its faces (`5e-4` a fifth of the way round a rim), so the edge carried on is computed where the faces carried on meet.

From *Faces a union leaves in two are one wall*: a plate standing flush with the side of what it stands on is two faces in one plane, so a corner on that side has four faces, not three, and what the fillet needs of the third face, a plane, square to the edge, is true of both. `tool_end` counts faces of one plane as one wall, and a refusal says which faces meet where (`corners_of`).

## Chamfers, and how it is tested

A chamfer of anything but a straight edge between planes or a whole circle is rolled in the same way (`plan_chamfer`), its sections chords rather than arcs, and the only difference is how they are placed: `Placing`. Where a chord's point on a side moves from face to face, the chord whose point lies on the edge between them is a station.

`ops/geop-ops-fillet/src/stress_tests.rs` blends every edge, and every corner's edges together, of boxes, cylinders and spheres, whole, drilled, pocketed, stepped, cut off and with a boss, from `geop_lib_fixtures::bodies`. The rule is in its header: "An operation may refuse what it does not support — saying so — but what it builds has to be a valid solid, and it must not fail otherwise." The fast bodies run on every `cargo test`, and the rest are `#[ignore]`d, run with `cargo test -- --ignored`, and AGENTS.md says to run them before changing blends. A story from *Test what the user does*: "Fillet tool ends were tested per edge, and broke only when two blended edges met at a corner."

<div class="recap">

## Recap

- A fillet is what a rolling ball sweeps. Find the ball with `meet`: "r from each face" and "in the plane square to the edge". No offset surface is needed.
- Each section is an exact rational arc. The blend is built as a tool for a boolean.
- A skin through tangent stations crosses the face between them. Tilt it by `$10^{-6}$` radians, and check between stations.
- Where the section is carried unchanged, `along_weight` makes it a polynomial that the cubic reproduces exactly. `span_is_exact` then keeps it untilted, in one path with no kinds.
- Where the data is `$C^1$`, put a station there and break.
- Each end of a chain is decided on its own. Corners are balls, inward corners are intersections of tools.
- A tool's curves must cross faces, never lie in them.

</div>

## Exercises

**1.** A fillet of radius `$r=2$` on a convex edge between planes at interior angle 120°. Compute `$\lvert E-C\rvert$`, the tangent length `$\lvert E-T\rvert$`, the weight `$w$`, and the apex distance `$\lvert Q-E\rvert$`.

<details class="solution"><summary>Solution</summary>

`$\lvert E-C\rvert=r/\sin60°=2.309$`. `$\lvert E-T\rvert=r/\tan60°=1.155$`. `$w=\sin60°=0.866$`. `$Q=2E-C$`, so `$\lvert Q-E\rvert=\lvert E-C\rvert=2.309$`.

</details>

**2.** Prove that `$w=\cos(\text{half the arc's turn})$` equals `$r/\lvert E-C\rvert$`.

<details class="solution"><summary>Solution</summary>

`$C,T,E$` is a right triangle with the right angle at `$T$` and hypotenuse `$\lvert E-C\rvert$`. The angle at `$C$` between `$CT_a$` and `$CE$` is half the arc's turn. So its cosine is `$\lvert CT\rvert/\lvert CE\rvert=r/\lvert E-C\rvert$`.

</details>

**3.** The interpolation error in `$f$` falls as the fourth power of the station spacing. A blend with 16 stations has `$\max\lvert f\rvert=4\times10^{-6}$`. How many stations before the tilt of `$10^{-6}$` clears it? Why does the kernel not simply continue to reach a far smaller error and set a smaller tilt?

<details class="solution"><summary>Solution</summary>

Halving gives `$2.5\times10^{-7}$` at 32 stations, already below the tilt. As for a smaller tilt: a smaller tilt needs more stations, so more cost, and the boolean's enclosures must still be able to tell a crossing at that angle. In the code the two are tied together: `const TILT: f64 = DEVIATION`.

</details>

**4.** Explain in your own words why multiplying by `$W(t)$` makes "a section round a circle" polynomial. Take a quarter arc, and a point at radius `$\rho$`.

<details class="solution"><summary>Solution</summary>

The edge is `$A(t)/W(t)$` with `$A$` and `$W$` quadratics. The point on the swept section at parameter `$t$` is at radius `$\rho$`, on the same rational parametrization of the circle, so it is `$\rho\,A(t)/(RW(t))$` relative to the centre (for an edge of radius `$R$`), plus a fixed height. Multiplied by `$W(t)$`: `$\rho A(t)/R+hW(t)$`, a quadratic. A cubic through at least four stations reproduces it exactly. Without that factor, the interpolated quantity would be a rational function, and a cubic could only approximate it.

</details>

**5.** A designer fillets an upright edge of a pocket and the floor edge it stands on, both with radius 3, and the pocket's depth happens to be 30. Name two choices of a cap's size that could land on another dimension in such a part, and say why `FlatPatch::past` avoids this.

<details class="solution"><summary>Hint</summary>

A side "through the contacts" lands on the other tool's contact by symmetry of the radii. "A third of the patch's size" lands wherever another dimension is a third of it. Any fraction of one design dimension can equal another. `FlatPatch::past` is given the extent of every edge of the model, so no edge can lie along its sides at all.

</details>

**6.** In the "tilt" demo, read the largest lean with no tilt at 6, 12, 24 and 48 spans. How well does it follow the fourth power of the spacing? What does the count of stretches through the face do meanwhile, and what does that say about refining as a cure?

<details class="solution"><summary>Solution</summary>

The demo gives about `$1.1\times10^{-2}$`, `$8.2\times10^{-5}$`, `$1.5\times10^{-5}$` and `$6.7\times10^{-7}$`. The last ratio, 22, is near the fourth-order 16; the first halving is much bigger, since six spans barely resolve the wave in the data. The number of stretches through the face does not fall: 4, 12, 12, 15. The dips get shallower, not fewer. So refining alone never leaves the face alone; only a tilt larger than the error does.

</details>
