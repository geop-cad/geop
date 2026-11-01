# Lecture 16: Tracing and pcurves

<div class="goals">

**After this lecture you can**

- set up the system of equations that a point of a surface-surface intersection satisfies, and find its tangent;
- write a predictor-corrector march, and say why the corrector solves one square system rather than projecting on each surface in turn;
- say how far to step, when to stop, and why neither "near a vertex" nor "the corrector converged" is enough on its own;
- fit a curve to the march's points that encloses the true curve, and a pcurve of it on each surface;
- explain each decision through the bug it fixed.

</div>

An edge crosses a face at a point. A face crosses a face along a *curve*. And we have no formula for that curve: two bicubic patches meet in a curve of very high algebraic degree, and it has no NURBS form of any low degree. Yet the boolean's result needs an edge there, with a NURBS curve for it, and a `$(u,v)$` curve on each face.

So the curve has to be *traced*: followed numerically, step by step, from one end, then fitted. Tracing is where Lectures 12 to 15 come together in the kernel's most delicate loop. As the developer notes in `DEVELOPERS.md` record, an earlier generation of the kernel got stuck here: "Booleans looked promising but tracing was too difficult, too many edge cases". This lecture goes through those cases one at a time, because each has a reason.

## The equations

Let `$S_a(u_a,v_a)$` and `$S_b(u_b,v_b)$` be two surfaces. A point on both satisfies

`$$S_a(u_a,v_a)-S_b(u_b,v_b)=0,$$`

three equations in four unknowns. Where the two surfaces are not tangent, the solution set is locally a curve (the implicit function theorem: the `$3\times4$` Jacobian has full rank when the two tangent planes differ).

In which direction does the curve go? It lies in both tangent planes, so its tangent is square to both normals:

`$$T=\frac{n_a\times n_b}{\lVert n_a\times n_b\rVert},\qquad n_a=\partial_uS_a\times\partial_vS_a,\ \ n_b=\partial_uS_b\times\partial_vS_b.$$`

Where the normals are parallel, `$T$` does not exist. This is the tangent contact of [Lecture 15](./15-tangency-and-overlap.md): either the surfaces touch there, or two branches of the curve cross. A trace cannot go through such a point on its own account.

The curve's ends are the first thing to know. Every intersection curve inside two trimmed faces ends where it leaves one of them. It leaves a face through that face's boundary, so at a point where an *edge* of one solid pierces a *face* of the other. The boolean's earlier stages have already found those piercing points and put a vertex at each ([Lecture 21](./21-remesh.md)). So every open curve is known at both ends before tracing begins: `find_tracing_start_points` lists them. Closed loops have no ends; they are found by the different search `turning_points` of the last lecture.

## Predictor and corrector

From a point `$p_k$` on the curve, with tangent `$T_k$` and a step length `$\ell$`, the **predictor** is the obvious guess:

`$$\hat p=p_k+\ell\,T_k.$$`

That point lies on neither surface, by about `$\tfrac12\kappa\ell^2$` for a curve of curvature `$\kappa$`. The **corrector** brings it back. There are two ways to do so.

**Alternating projection.** Project `$\hat p$` onto `$S_a$`, and onto `$S_b$`. The trouble, as the doc of `predictor_corrector_step` in `core/geop-core-geometry/src/intersection/branch.rs` explains: separate projections each enforce their own surface while ignoring the other, so they "land a full prediction-error apart and stay there no matter how good the predictor is". Alternating between them converges slowly (linearly, at a rate set by the angle between the surfaces).

**One square system.** Solve for all four parameters at once. The three equations above are short of one, and the missing one is what pins the free direction. Add a fourth: the point must lie on the plane through the prediction, normal to the direction of march.

`$$F(u_a,v_a,u_b,v_b)=\begin{bmatrix}S_a(u_a,v_a)-S_b(u_b,v_b)\\T_k\cdot\bigl(S_a(u_a,v_a)-\hat p\bigr)\end{bmatrix}=0.$$`

The Jacobian is square:

`$$J=\begin{bmatrix}\partial_uS_a&\partial_vS_a&-\partial_uS_b&-\partial_vS_b\\T_k\cdot\partial_uS_a&T_k\cdot\partial_vS_a&0&0\end{bmatrix},$$`

where each `$\partial S$` in the top row is a 3-vector, giving three rows. This is an ordinary Newton iteration on a square system ([Lecture 14](./14-newton.md)), and it lands on the curve at the plane the predictor stepped to. The corrector "may move freely onto the curve but not drift along it", as the code says.

Note the role of the fourth row in the stopping test. The corrector is handed `$(u,v)$` that already lie on both surfaces. If only the first three residuals were checked, it would report success without moving, "leaving the march pinned at its first step forever". The fourth residual says that the point has actually advanced.

<figure class="demo" data-demo="tracing">
<figcaption>A sphere (grey) and a cylinder (orange) meet in a closed curve, traced by exactly the corrector above. Orange: the prediction. Red: where the corrector took it. Play it, then widen the stride until the march fails. Slide the cylinder's offset towards 0.5, where the two loops of the curve almost touch, and try stride 0.2 at offset 0.495. Below: the pcurve on the sphere, the march's (u, v). This loop goes round the sphere's axis, so its pcurve leaves the (u, v) rectangle on one side and comes back on the other: a seam, the subject of <a href="./18-poles-seams-splicing.md">Lecture 18</a>. The kernel's stride adapts (see below), its comparisons are intervals where this demo tests the residual against a number, and its surfaces are NURBS where these are formulas; drag to orbit.</figcaption>
</figure>

On this scene with cylinder offset `$a=0.35$`, radius `$0.5$`, and a sphere of radius 1, the demo's algorithm gives (checked in `book/scripts/demo_tests/tracing.test.mjs`):

| stride | steps | outcome | most corrector iterations in a step |
|---|---|---|---|
| 0.02 | 166 | closes | 4 |
| 0.1 | 33 | closes | 5 |
| 0.2 | 17 | closes | 6 |
| 0.4 | 8 | closes | 9 |
| 0.5 | 3 | corrector does not converge in 20 iterations | 20 |

So longer strides cost corrector iterations and then fail outright. The curve's length here is about 3.3, and it bends with a radius of the order of the cylinder's, 0.5: a stride of that size leaves the region where Newton converges.

Worse than failing is succeeding at the wrong place. With offset `$a=0.495$` and stride 0.2 the demo's march does not fail. Both surfaces are smooth and the corrector converges at every step. But the loops pass within 0.2 of each other near `$(1,0,0)$`, since `$a+r=0.995$` leaves them a gap of `$2\sqrt{1-0.995^2}\approx0.2$`. The march hops from the upper loop to the lower one, and then goes round that loop until it runs out of its 400 steps, never coming back to its start. A corrector that converges has only found *a* point of the intersection. It says nothing about whether it is on the same branch.

### Sharp and honest, in the same loop

The corrector is the place where the sharpening rules of [Lecture 5](./05-honest-numerics.md) had to be worked out, and it contains three instances.

**The prediction is sharp, once.** A prediction is a seed and a bias, free to be any point along the step, and it fixes the plane the corrector solves against. Likewise the plane's normal: any plane crossing the curve pins the free direction equally well. Left wide, the normal is the one wide row of an otherwise sharp Jacobian. Where the two surfaces meet at a shallow angle, the normalized cross product of nearly parallel normals is wide by percents, enough to put zero inside a pivot of a nearly singular system. So both are sharpened, right at the top of the function, and "Nothing downstream of this line sharpens".

**Two copies of the iterate.** `iterate` keeps a sharp seed for the next step, and the same value with its width as the answer. The sharp seed is necessary: left wide, 20 Newton steps compound width until `evaluate` fails outright, measured at 36 of 175 scenes. The honest answer is equally necessary, and here is the case that taught it.

`predictor_corrector_step` once returned sharp `$(u,v)$`. A traced curve that ran exactly along a face's trim boundary therefore came back with a sharp `$(u,v)$` sitting `$6\times10^{-17}$` off that boundary, and `face_contains` classified it `Inside`, since `could_be_equal` had no width to work with. The direction check that exists to reject such a curve waved it through. The curve was spliced in as a duplicate of the boundary, carving a zero-area sliver, which surfaced much later as a face a boolean could not classify, since it had no interior point. As AGENTS.md notes, there was no geometric error in this at all. What sharpening destroyed was the *statement of uncertainty* every three-valued comparison depends on.

**The start of each step, too.** The `$(u,v)$` handed in from the previous step are that step's honest answer, and if used as the seed unsharpened, they compound through the march. The test `cube_minus_box_with_wide_corners`, on a tool whose corners are only known to `$10^{-12}$`, reached a whole patch's width within a dozen steps that way.

**Union at the end.** When the corrector is done, it has two enclosures of one point, from the two surfaces. They are combined by `union`, the rule of [Lecture 5](./05-honest-numerics.md): an average would claim more precision than either had, and hide how far apart they were.

### The solve

The linear system is solved by damped normal equations:

`$$(J^\top J+\lambda I)\,\delta=-J^\top F,\qquad\lambda=10^{-12}\ (\texttt{CORRECTOR\_DAMPING}).$$`

The damping only keeps the step finite where `$J$` loses rank (nearly parallel surfaces, a pole), at the cost of a shorter step that the next iteration continues. It cannot change what a converged answer means, since what decides convergence is the interval test on the residuals. And `$J^\top J+\lambda I$` is sharpened before it is solved. That is legitimate, by the logic of Krawczyk's preconditioner `$Y$` from [Lecture 14](./14-newton.md): the matrix only steers the step, and the residual `$F$` alone decides where the fixed point is. The step's honest width is the residual's, carried through a sharp matrix. Left as intervals, the elimination would amplify the surfaces' width by the system's condition, which `$J^\top J$` squares.

A corrector that fails to bring the two surfaces onto a common point within `CORRECTOR_ITERATIONS` (20) reports an error, never a best guess.

## Which way, how far

### The first step

From a start vertex, a curve leaves along `$+T$` or `$-T$`. Exactly one of those (for a typical piercing point) runs into the shared interior of the two trimmed faces. To decide, `trace_one_side` tries a step each way, and asks `face_contains` on each face whether the result is `Inside`. Neither direction being viable is a legitimate outcome: the start points are found against untrimmed surfaces, so two cubes meeting at a corner produce a start point from which no curve really leaves.

Returning quietly carries a risk, which the source code names. A trace that should have produced a curve and didn't looks identical to this. So it is guarded from outside: `box_cylinder_blind_hole_remesh_succeeds` asserts that tracing produced edges shared between both solids' faces, which a dead trace cannot fake.

The first step has a trap of its own, which AGENTS.md calls "the invariant has one more condition": a trace must stop at every vertex on its curve. If a known vertex lies nearer than the trial step, the full step lands past it. Where the half step toward it was rejected, because the curve runs along a boundary up to that vertex, the full step was accepted, and the trace jumped the vertex and spliced a curve partially duplicating an existing edge. So the trial now goes half way to any near vertex first, and the full step is not allowed to decide a direction past a vertex known to lie on the curve.

### The stride

`adaptive_step_size` in `lib/geop-lib-booleans/src/remesh/remesh_edges_x_faces.rs` takes the smaller of two lengths.

- A sixteenth of the smaller surface (`STEPS_ACROSS_PATCH`). The curve lies on both surfaces, so that is the scale of features it can pass, and the stride grows and shrinks with the part. A fixed 0.1 took 400 steps along a 40 mm cut, and was also more than a cut 0.05 wide.
- The arc that takes a sixty-fourth of a full turn (`STEPS_PER_REVOLUTION`) around the tighter surface. A step has to be short compared to how fast the curve turns, or the straight-line predictor leaves the surface and the corrector drags it somewhere else entirely.

A surface that runs straight the way the curve does, `curvature_radius_along` returning `None`, imposes no limit. There is a good story behind this exception.

Sized only by the tightest bend, the trace misbehaves at the apex of a cone. There the tightest bend is the circle round the axis, which shrinks with the distance to the apex. A curve running along a generator into it, a straight line, took a step a fraction of its distance to the apex each time and never got there: 359 steps, spaced geometrically, for a straight line. The interpolation through those points then widened exponentially along the chain, to 111 in the control points, and the splice failed on a "corner" it could not decide.

Going further, sizing the step by the bend along the curve for every surface, was tried and is wrong: where the surfaces bend a curve a little here it may bend sharply a stride on (the blades of `turbine_blade`, whose pattern's trace left its patch). So the rule is narrow: no limit from a surface that does not bend the curve at all, the tightest bend from any that does. AGENTS.md's lesson: look at the number of steps a trace took. A line that took hundreds went wrong before it was fitted.

### When to stop

A trace ends when it reaches a known vertex. This takes more care than it sounds.

**Proximity is not membership.** `candidate_within` chose the nearest vertex within one stride. But a traced curve lies on both of the surfaces it traces between, so its terminating vertex must lie on both too, and an oversized or overhanging solid puts plenty of unrelated vertices within a stride of the curve's end. Adopting one of those ended the edge at a point on neither face. `fit_pcurve` then had nothing to project onto and silently returned the nearest foot point, about `$8\times10^{-3}$` away, which finally surfaced as 32 `validate_fast` errors about pcurve endpoints not matching their 3-D points. The fix asks for what the geometry asserts: `surface_could_contain` on both surfaces.

AGENTS.md adds a diagnostic that paid for itself. Reporting whether the point lies on the surface *at all* separates "the pcurve names the wrong `$(u,v)$`" from "the edge was spliced onto a face it does not belong to". The error text is identical, the fixes are opposite. Measuring it: 32 failures, every one with exactly one endpoint on-surface and one off.

**Ahead of us, on this stride.** A candidate must also lie between the current point and the plane the corrector lands on. One definitely behind is not it: on `narrow_groove`, a trace leaving the cut's inner corner found, one stride later, the vertex 0.05 *behind* its start, nearer than anything ahead, and spliced a spur out of the wall to it. One definitely past the plane is the next stride's to reach.

**Before and after each stride.** A vertex is looked for within the stride ball first. It matters where the curve ends at the edge of a patch the stride would have to leave. Then, once more within the chord actually taken: where the curve bends away from `$T$`, it meets the plane farther than `$\ell$` away, and a vertex can lie ahead of the plane but outside the ball. Checking only the ball stepped over such a vertex on `chained_differences_block_with_two_slots_and_a_sphere`.

**Never at a guess.** "An intersection curve of two faces has to end at a vertex." If a trace exhausts `max_trace_steps`, the result is an error naming the last point and how many steps were taken. If a stride fails on the way, the error's context says which vertex on both surfaces lies nearest within four strides, so the next failure reports itself: AGENTS.md's rule of adding context to the real path instead of debugging it from outside.

### The same curve, twice

Every curve has two ends, both start points, so it would be traced twice. The second trace would re-derive the curve and splice a duplicate edge between the same two vertices, carving a zero-area sliver along the first.

The tempting fix samples the finished curve, and requires some sample strictly inside the face. AGENTS.md's "Ask a topological question topologically" says this works and is wrong: "a sampling heuristic standing in for a question with an exact answer". The right question is *is this curve already represented here?*, answered from the topology: an edge joining the same two vertices. In the current code (`trace_one_side`), the check first requires the same two end vertices, then asks one more thing, whether that edge's curve could contain the trace's middle point (`curve_could_contain`), which separates an arc from its complement sharing both ends. Ties go to the lowest id, not to whatever a hash map yields first.

What makes such a cheap test sufficient is a pipeline invariant: the edge × edge stage has already put a vertex at every coincidence, so no curve can *partially* overlap an edge. A curve either fully duplicates one or does not touch it. Where an invariant like that holds, reach for the exact test it enables.

## Fitting the curve

The march leaves a list of points with `$(u,v)$` on both surfaces. An edge needs a curve. Interpolating a cubic through the points (`NurbCurve::interpolate`, Piegl and Tiller's method) passes through each exactly, and drifts from the true curve between them. For the interval kernel this drift is no detail: "otherwise a point on the true curve tests as *not* on its interpolant, and every containment or intersection question asked of it is answered for the wrong curve".

So the kernel measures it, and encloses it. `NurbCurve::interpolate_enclosing` takes, besides the points, for each interval some true points of the curve between them, located by the same corrector (`true_point_fractions` says at which fractions, because a single midpoint under-measures a hump whose peak can lie elsewhere). The control points are then *widened* until the curve holds these. The result has the honest width of the fit, and its error is a property of the data structure, not a guess about it.

`fit_branch` in `core/geop-core-geometry/src/intersection/branch.rs` wraps this with `NurbCurve::fit_enclosing`: every fit has at least `MIN_FITTED_LEGS` legs, and while the width is more than `FIT_DRIFT` (`$10^{-5}$`) and halving the legs still halves the width, the legs are split finer. Once halving no longer halves it, the width is the points' own, and the coarser fit is kept. A gear's involute flank, whose curvature radius grows tenfold from root to tip, left a cubic drifting a hundred times the usual, as the code comment records; this is what handles it.

Since cubic interpolation is smooth to second order, its drift shrinks as `$h^4$` on smooth data. A good test of a trace is thus to read the width: `trace_one_side` records its widest marched point and the widest true point between them, which tells where a wide curve got wide.

## Pcurves

An edge of a B-rep has one 3-D curve, and, for each face it bounds, a curve in that face's `$(u,v)$` domain: its **pcurve** ([Lecture 17](./17-brep.md)). The face's trim is defined by those. Answering "is this `$(u,v)$` inside the face?" takes a 2-D question about pcurves, not a 3-D one about edges.

A new traced edge has both faces' `$(u,v)$` at each marched point, and a 3-D curve. Those marched values are not what the pcurve is made from. `trace_one_side` splices the edge into each face with `splice_edge_into_face`, and that calls `NurbSurface::fit_pcurve` (`core/geop-core-geometry/src/nurb_surface/fit_pcurve.rs`), which builds a pcurve from the edge's curve alone, on any surface: sample the curve, project each sample onto the surface with Newton, seeding each projection from the previous one's result so the walk stays continuous, and interpolate. Enough detail matters here to go through the pieces.

**The first seed is found globally.** Newton from anywhere settles on whichever local foot point is nearest. So the start is either `pin_start`, the face's authoritative `$(u,v)$`, or `locate`. A curve that does not start on the surface is an error: there is no trace of it to fit.

**The ends are pinned.** Where a curve's end is at a place the face already has a coedge for, that coedge's own pcurve end is what the new one has to match, exactly: a boundary loop requires `could_be_equal` continuity from one pcurve's end to the next one's start. An independently projected `$(u,v)$` lands a hair away. And the clamped interpolant passes exactly through its end data, so the ends are free to pin. (`splice_edge_into_face` goes further, pinning each end of a new loop to the arriving end's `$(u,v)$` *intersected* with the vertex's projection: both enclose the same point. Using the end alone let width spread around the loop until a face split came out degenerate.)

**Use the structure you know.** The kernel's arcs and helices are rational quadratics, only `$C^1$` where their spans meet. `fit_pcurve` once fitted one `$C^2$` cubic through 48 samples of the whole curve. Across those joints, its drift shrank only as `$h^2$`, and a helix of pitch 1 came within 15% of the `$10^{-4}$` an entity may carry (`MAX_WIDTH`, [Lecture 20](./20-validation.md)). The joints are known, they are the knots of the source curve. So it now fits each smooth piece on its own and joins them.

**Then adapt.** A piece is sampled more densely while its pcurve is wider than a target, between `MIN_PIECE_SAMPLES` and `MAX_PIECE_SAMPLES`. A pcurve still drifting at the maximum has a trace that is not smooth where the knots say nothing, say at a surface's seam: "it is returned as it is, wide and honest". As AGENTS.md concludes: use the structure you know before you sample blindly, and let the sample count follow the width it produces.

**The answer keeps its width.** The seeds of `project` are sharp, the projected `$(u,v)$` is not, because these values enter the fit and are later compared against the edge's 3-D points. Narrowing them would claim a precision the projection did not have.

<div class="kernel">

**In the kernel.**

| What | Where |
|---|---|
| One predictor-corrector step | `predictor_corrector_step` in `core/geop-core-geometry/src/intersection/branch.rs` |
| The march, its direction, its ends | `trace_one_side`, `trace_from_start_point`, `adaptive_step_size`, `candidate_within` in `lib/geop-lib-booleans/src/remesh/remesh_edges_x_faces.rs` |
| Where to start | `find_tracing_start_points`, same file |
| Budgets | `max_trace_steps` (1000) in `RemeshParams`, `lib/geop-lib-booleans/src/remesh/mod.rs` |
| The enclosing fit | `fit_branch`, `NurbCurve::fit_enclosing` (`core/geop-core-geometry/src/nurb_curve/fit.rs`), `NurbCurve::interpolate_enclosing` (`nurb_curve/interpolate.rs`) |
| Pcurves | `NurbSurface::fit_pcurve` in `core/geop-core-geometry/src/nurb_surface/fit_pcurve.rs` |
| Closed loops | `turning_points` in `core/geop-core-geometry/src/intersection/surface_surface.rs` |

`branch.rs` deliberately knows only how to follow a branch from point to point. "Where the branch runs and where it ends is the caller's to decide": the boolean marches from a piercing point to a vertex, and a shell knows both ends, the moved vertices, and the edge whose inner copy the branch is ([Lecture 30](./30-one-rebuild.md)).

</div>

## Recap

<div class="recap">

- A surface-surface intersection is three equations in four unknowns, a curve with tangent `$n_a\times n_b$`. The ends of an open curve are piercing points, already vertices.
- The corrector solves one square system: `$S_a=S_b$` plus the plane through the prediction. The prediction and the plane are sharp, the answer is not.
- A converged corrector only found *a* point of the intersection. A step must be short against the curve's turn, and yet must arrive; so the stride depends on the surfaces' size and bend, except a bend the curve does not follow.
- A trace ends at a vertex on both surfaces, on the stretch of the stride, and never by being near.
- The fit encloses the true curve, measured at true points. A pcurve is fitted per smooth piece, its ends pinned to what the face already says, its density following its width.

</div>

## Exercises

**1.** In the demo, find (roughly) the longest stride that closes at offset `$a=0.35$`, and at `$a=0.45$`. Relate them to the curve's tightest bend. Then read off the kernel's stride for a sphere of radius 1: what would `STEPS_PER_REVOLUTION` give?

<details class="solution"><summary>Solution</summary>

At `$a=0.35$` stride 0.4 closes and 0.5 fails. At `$a=0.45$` stride 0.2 fails after 25 steps, with the march near the top of the loop, and 0.05 closes. The curve is tighter there. The kernel's limit from the surface: the sphere's curvature radius is 1, so `$\ell\le2\pi/64\approx0.098$`; the cylinder's radius 0.5 gives `$\approx0.049$` unless the curve runs along it. So the kernel would use about 0.05 here: safely inside what the demo tolerates.

</details>

**2.** Compute the prediction's distance from the sphere for stride `$\ell$` on a great circle of a unit sphere, starting on it. How does the corrector's first residual scale with `$\ell$`? Why does this suggest the corrector iteration counts in the table?

<details class="solution"><summary>Solution</summary>

`$\lVert p+\ell T\rVert=\sqrt{1+\ell^2}$`, so the distance is `$\sqrt{1+\ell^2}-1\approx\ell^2/2$`. For `$\ell=0.1$` that is `$5\times10^{-3}$`, for `$\ell=0.4$` it is `$0.077$`. With Newton squaring the residual, from `$5\times10^{-3}$` to `$10^{-12}$` takes about 3 steps, and from `$0.08$` a step or two more, and the constant is larger for a big step. Which matches the trend from 5 to 9 iterations.

</details>

**3.** Why does `predictor_corrector_step` plant its plane at `$\hat p$`, and not at `$p_k$`? Why is a plane better than a sphere of radius `$\ell$` around `$p_k$`?

<details class="solution"><summary>Hint</summary>

A plane at `$p_k$` is solved already: no advance. A sphere has two solutions along the curve, one behind, and a Jacobian row that is `$2(S-p_k)$`, which is nearly singular when small, and the system is nonlinear in one more way. The plane's row is constant. For `candidate_within`, the plane also gives a clean statement of how far a stride reaches.

</details>

**4.** A trace ran from vertex `$v$` to a vertex `$w$`, and a second trace from `$w$` found a curve to `$v$`. Under what circumstance are they two different branches, and how does `trace_one_side` tell?

<details class="solution"><summary>Solution</summary>

A cylinder's rim crossing a plane twice, where two arcs of a circle join the same two vertices, is a clear example: an arc and its complement. Same ends. So the code tests if the existing edge's curve could contain the new trace's middle point: only if it can is this a duplicate. With an arc and its complement the mid-point of one is not on the other.

</details>

**5.** The kernel's pcurves are fitted to the edge's 3-D curve, even though the march has `$(u,v)$` for both surfaces at each point. Argue for this choice. (Hint: what is compared with what, later?)

<details class="solution"><summary>Hint</summary>

Validation and later searches compare a pcurve, through its surface, with the *edge's curve*. So what must agree is `$S(\text{pcurve}(t))$` with `$C(t)$`, at the same parameter `$t$` of the fitted curve, not with the march's points. The marched `$(u,v)$` have a different parametrization than the fit. Also, the same function is reused for any curve and any surface, including edges that were never traced, which is what a small kernel wants.

</details>

**6.** Let a cubic interpolate samples of a curve at spacing `$h$`. Its drift on a smooth stretch is `$O(h^4)$`, and across a jump in curvature `$O(h^2)$`. Estimate how many times fewer samples are needed if you know where the jump is, for a target drift `$10^{-5}$` and a curve of unit size.

<details class="solution"><summary>Solution</summary>

Take constants of order one. Smooth: `$h^4=10^{-5}$` gives `$h\approx0.056$`, about 18 samples per unit length. Across a jump: `$h^2=10^{-5}$` gives `$h\approx0.0032$`, about 316 samples per unit length if all samples are spaced to that resolution. Knowing the joint and breaking the fit there saves a factor of about 18, and that factor grows as the target tightens. This is the reason `fit_pcurve` breaks at knots, and the same reason fillets put a station at the kink (Lecture 29).

</details>
