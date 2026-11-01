# Lecture 14: Newton to refine

<div class="goals">

**After this lecture you can**

- derive Newton's method in one variable and for systems, and prove its quadratic convergence at a regular root;
- project a point onto a curve or surface with Gauss-Newton and with Newton, and say exactly when each diverges;
- write a refinement that is honest in the interval sense: sharpen every iterate but the last, keep two copies, intersect with the box the search proved;
- say why a refinement must be infallible, why its target must be refined first, and why a caller should refine only where a parameter is used;
- state interval Newton and Krawczyk's test, and the difference between a refined parameter and a proven one.

</div>

[Lecture 12](./12-subdivide-to-isolate.md) and [Lecture 13](./13-fat-lines.md) leave us with a small box known to hold every solution near it. For many purposes that box is too wide: a split parameter that wide makes a fat vertex and a fat sub-curve, and the width then grows (we saw why in [Lecture 9](./09-subdivision.md)). More subdivision would narrow the box by a bit per level. There is a better tool for the last stretch.

## Newton in one variable

Near a point `$x_k$`, a smooth function looks like its tangent:

`$$f(x)\approx f(x_k)+f'(x_k)(x-x_k).$$`

Solve the right-hand side for zero, and call that the next guess:

`$$x_{k+1}=x_k-\frac{f(x_k)}{f'(x_k)}.$$`

For `$f(x)=x^2-2$`, starting from `$x_0=1.5$`:

| `$k$` | `$x_k$` | `$\lvert x_k-\sqrt2\rvert$` |
|---|---|---|
| 0 | 1.5 | `$8.6\times10^{-2}$` |
| 1 | 1.41666666666667 | `$2.5\times10^{-3}$` |
| 2 | 1.41421568627451 | `$2.1\times10^{-6}$` |
| 3 | 1.41421356237469 | `$1.6\times10^{-12}$` |
| 4 | 1.41421356237310 | 0 (to double precision) |

The number of correct digits doubles every step. Here is why. Let `$x^*$` be a root with `$f'(x^*)\ne0$`, and let `$e_k=x_k-x^*$`. Taylor's theorem around `$x_k$` gives

`$$0=f(x^*)=f(x_k)-f'(x_k)e_k+\tfrac12f''(\zeta)e_k^2.$$`

Divide by `$f'(x_k)$`, and rearrange using the definition of `$x_{k+1}$`:

`$$e_{k+1}=\frac{f''(\zeta)}{2f'(x_k)}\,e_k^2.$$`

So the error is squared, up to the constant `$\lvert f''/2f'\rvert$`, as long as `$f'$` stays away from zero. Three caveats hide in this one line, and each of them has its own section below or in [Lecture 15](./15-tangency-and-overlap.md).

1. It is a statement about a neighbourhood of a root. Newton *finds* nothing: from far away it may go anywhere.
2. The constant blows up where `$f'(x^*)$` is small. At a double root the method degrades to halving the error per step.
3. In a kernel the data has width, and so the answer does. The iteration must not pretend otherwise.

This is the division of labour from AGENTS.md's "Subdivide to isolate, then Newton to refine", which the kernel follows everywhere. Subdivision reaches machine accuracy in about 50 levels, of which about 7 isolate the solution. Newton then polishes it in a dozen iterations, and no more accuracy is asked of the subdivision than separating the solutions.

## Newton on a system

For `$F:\mathbb R^n\to\mathbb R^n$` the tangent is the Jacobian `$J=\partial F/\partial x$`, and each step solves a linear system:

`$$J(x_k)\,\delta=-F(x_k),\qquad x_{k+1}=x_k+\delta.$$`

The convergence proof is the same with norms. For a curve crossing a surface, `$F(t,u,v)=C(t)-S(u,v)$` has three equations in three unknowns, with

`$$J=\begin{bmatrix}C'(t)&-S_u(u,v)&-S_v(u,v)\end{bmatrix},\qquad \det J=C'\cdot(S_u\times S_v).$$`

The determinant says the geometry out loud. It vanishes when the curve's tangent lies in the surface's tangent plane (a tangency), or where `$S_u\times S_v=0$` (a pole). That is `refine_crossing` in `core/geop-core-geometry/src/intersection/curve_surface.rs`, which builds exactly this 3×3 system.

Two curves in 3-D give more equations than unknowns: three equations in `$(s,t)$`. The system is overdetermined, so `refine_crossing` in `intersection/curve_curve.rs` steps by least squares, solving the normal equations `$(J^\top J)\delta=-J^\top F$`. That is **Gauss-Newton**: it uses `$J$` only.

## Projection: the problem that teaches the most

Finding the point on a curve or surface nearest a given target is a kernel's most common iterative computation. It is `NurbSurface::project` in `core/geop-core-geometry/src/nurb_surface/project.rs`, called on every sample of every `fit_pcurve`. So let us look carefully.

Take a curve `$C(t)$` and a target `$T$`. The nearest point is a critical point of

`$$\phi(t)=\tfrac12\lVert r(t)\rVert^2,\qquad r(t)=T-C(t),$$`

so it satisfies `$\phi'(t)=-r\cdot C'=0$`: the residual is square to the curve. Now there are two reasonable steps.

**Gauss-Newton** linearizes the *curve*: `$C(t+\delta)\approx C+C'\delta$`, and minimizes `$\lVert r-C'\delta\rVert^2$`:

`$$\delta_{\text{GN}}=\frac{r\cdot C'}{C'\cdot C'}.$$`

This is the projection of the residual on the tangent. No second derivative is needed, which is its attraction.

**Newton** linearizes the *condition* `$\phi'=0$`: `$\phi''=C'\cdot C'-r\cdot C''$`, so

`$$\delta_{\text{N}}=\frac{r\cdot C'}{C'\cdot C'-r\cdot C''}.$$`

The difference is the term `$r\cdot C''$`. It is the residual's length times the curvature, so it is small when the target is near the curve, and not small otherwise.

### The circle, in closed form

Let `$C(t)=R(\cos t,\sin t)$`, and put the target at distance `$\rho$` from the centre, at angle `$\theta$`. Let `$e=\theta-t$` be the angle still to go (the foot point is `$t=\theta$`). Then `$r\cdot C'=R\rho\sin e$`, `$C'\cdot C'=R^2$`, and `$r\cdot C''=-R\rho\cos e+R^2$`. So

`$$\delta_{\text{GN}}=\frac{\rho}{R}\sin e,\qquad \delta_{\text{N}}=\tan e.$$`

The errors evolve as

`$$e_{\text{GN}}'=e-\frac\rho R\sin e\approx\Bigl(1-\frac\rho R\Bigr)e,\qquad e_{\text{N}}'=e-\tan e\approx-\frac{e^3}{3}.$$`

Write `$\rho=R+d$`, with `$d$` the target's distance outside the circle. Then Gauss-Newton multiplies the error by `$-d/R$` per step.

**That is the whole story of Gauss-Newton on a curved surface.** Its convergence is linear, at the rate *distance over radius of curvature*. A target on the surface converges at once. A target half the radius away, with `$-\tfrac12$` per step. A target exactly one radius off has a multiplier of `$-1$`: to first order it stays at its distance, swinging from side to side, and only the next term of the sine wears it down, too slowly for any iteration budget (from `$e_0=0.5$` the error is still `$0.24$` after 20 steps). Beyond that the foot point is repelling, and the iterates swing out until they settle in a cycle.

Newton, on the circle, is cubic: the second derivative here is so well behaved that even the second-order term cancels. On a general curve it is quadratic.

Let us check with numbers, with `$R=1$`, a target at `$\rho=3$`, so twice the radius off, and seed `$e_0=0.2$`. (The demo's `algo`, and these claims, are tested in `book/scripts/demo_tests/newton-foot.test.mjs`.)

| `$k$` | Gauss-Newton `$t_k$` | Newton `$t_k$` |
|---|---|---|
| 0 | 0.2000 | `$2.0\times10^{-1}$` |
| 1 | −0.3960 | `$-2.7\times10^{-3}$` |
| 2 | 0.7612 | `$6.6\times10^{-9}$` |
| 3 | −1.3082 | 0 |
| 4 | 1.5890 | 0 |
| ... | settles at `$\pm1.4958$` | |

Gauss-Newton ends in a 2-cycle: `$e\mapsto-e$` with `$2e=3\sin e$`, which gives `$e\approx1.4958$`. It will swing between the two sides for ever.

<figure class="demo" data-demo="newton-foot">
<figcaption>Drag the target. Blue is Gauss-Newton, orange Newton, each iterate's foot point joined to the target by a thin line; the readout tabulates their errors. Inside the circle, near its centre, and more than one radius outside, compare the two. The ellipse shows Newton's error squared per step rather than cubed. Where the second derivative makes Newton's denominator non-positive, this demo, like <code>NurbSurface::project</code>, takes Gauss-Newton's step.</figcaption>
</figure>

### A bug this explains

AGENTS.md's section "Newton needs the second derivatives where it is far from flat" has this from real life. `NurbSurface::project` used to step by Gauss-Newton, the first fundamental form alone. A pocket's rim rounded with the radius its corner's fill was rounded with put the ball's centre at just such a distance, and `meet`, the iteration that moves a vertex onto offset surfaces, ran off along the fill's end. As the code's comment puts it: a target twice the radius off a cylinder "swung from side to side of its foot point for ever".

The current `project` takes Newton's step wherever its Hessian is definitely positive definite, and Gauss-Newton's where not:

`$$H=\begin{bmatrix}S_u\cdot S_u-r\cdot S_{uu}&S_u\cdot S_v-r\cdot S_{uv}\\S_u\cdot S_v-r\cdot S_{uv}&S_v\cdot S_v-r\cdot S_{vv}\end{bmatrix}.$$`

Why fall back at all? Newton's step targets any critical point. If `$H$` is not positive definite, the iterate is not near a minimum of the distance, and Newton could step to a saddle or a maximum. Gauss-Newton's matrix (the first fundamental form) is always positive semi-definite, so its step is a descent direction. In interval arithmetic "definitely positive definite" is a real test: `h11.definitely_greater(Scalar::ZERO)` and the determinant likewise.

### Clamping, and poles

Two practical matters.

**The domain.** A surface's domain is a rectangle. An iterate can step outside on its way in, and evaluating a B-spline outside its knots is not defined, so `project` clamps into `domain_u` and `domain_v` before each step. A foot point clamped at the edge means the nearest point is on the boundary (the true minimum may be beyond, on the surface carried on).

**The pole.** At a pole a row of control points collapses to a point, so `$S_v=0$` there, and the first fundamental form is singular (see [Lecture 10](./10-surfaces.md)). A seed on the pole would then never move, whatever the target. As AGENTS.md's "Degenerate parametrizations" notes: only the collapsed parameter stops moving the point. The other still moves it, so step along that one, down a meridian. Which meridian is a free choice, since at the pole the collapsed parameter says nothing, and a free choice should be a good one: of a few candidates, `off_pole` takes the one along which the step goes furthest towards the target.

### The first seed has to come from a search

Newton converges to whichever foot point it is near. On a curved patch that may be a local minimum, or one clamped against a domain edge where the residual is merely square to the boundary. So `NurbSurface::locate` first proves a box with `surface_could_contain`, and only then refines with `project`. That is subdivide-then-Newton in one function. Similarly `fit_pcurve` seeds its first projection globally, unless its caller already knows the `(u, v)`, and every later one from its predecessor.

And the same reasoning makes `NurbSurface::closest_point` call `least_distance` first, then refine each surviving patch ([Lecture 12](./12-subdivide-to-isolate.md)). The search proves no basin was missed. Newton's job is the digits.

## Honest refinement

So far this was classical numerics. Now the interval rules of [Lecture 5](./05-honest-numerics.md) meet it, and all of this section is hard-won.

Suppose every number is an interval. Run `$x_{k+1}=x_k-f(x_k)/f'(x_k)$` on intervals, and the iterates *widen*: `$x_k$` appears twice, the two appearances are uncorrelated, and the width feeds forward. Left alone it compounds over a few iterations until `evaluate` fails: AGENTS.md measured this at 36 of 175 test scenes for the tracing corrector.

The way out is to notice that **an iterate is a seed, and a seed is a free choice**. Any point of the iterate serves equally well to start the next step from. So sharpen it: replace the interval with its midpoint, and take the next step from that one point. That removes the compounding at no cost.

But the *last* iterate is not a seed. It is the answer. Its width says how well the data pins down the root: the residual `$F(x_k)$` inherits the data's width, the step inherits the residual's, and that is the honest statement of how precisely an uncertain target determines a foot point. Sharpen the last one too, and we return one sharp `$(u,v)$` for a target that only ever determined a *range* of them. Then every later comparison against anything else derived from that uncertain point fails.

This is no theoretical nicety. `NurbSurface::project` used to sharpen every iteration including the final one. Fixing just this took `validate_fast` failures across the scene sweep from 21/175 to 1/175, as AGENTS.md records.

So keep **two copies**, one as the seed and the other as the answer. Here is the idea in full, from `core/geop-core-math/src/solvers/iterate.rs`:

```rust,ignore
pub fn iterate<X: Iterate, E>(
    seed: X,
    answer: X,
    steps: usize,
    repeat: Repeat,
    settle: impl Fn(&X) -> (X, X),
    step: impl FnMut(&X) -> Result<Step<X>, E>,
) -> Result<Iterated<X>, E>
```

Every Newton-type loop in the kernel runs through this: foot points, a ball rolled between faces, a vertex moved onto offset surfaces, a traced curve's corrector. `step` takes a seed to the next iterate. `settle` turns that iterate into two values, the sharp seed of the next step and the honest answer it gave. The returned width is that of a *single* step from a sharp seed: bounded, not compounded, and not discarded.

### A repeating seed is done

Newton with sharp seeds rarely hits an exact fixed point in floating point. It usually enters a cycle in the last bit, and it would run out its iterations going round. But the map from a sharp seed to the next is deterministic, so once a seed repeats exactly, all the rest is periodic and the end result known. `iterate` with `Repeat::Cycle` compares bit for bit (`Iterate::same`), and stops there, with the identical result, in less than half the iterations (AGENTS.md, "Count the work"). This is no convergence tolerance: the seed is bit for bit one seen before.

### The refinement a search hands to

The generic version sits in `core/geop-core-math/src/solvers/newton.rs`:

```rust,ignore
pub fn refine<const N: usize>(
    start: [Scalar; N],
    domain: [(Scalar, Scalar); N],
    step: impl Fn([Scalar; N]) -> Option<[Scalar; N]>,
) -> [Scalar; N]
```

Read the design choices, each of which comes from a rule.

- **It returns a value, not a `Result`.** Refinement is infallible: a singular Jacobian at a tangential crossing or a pole, a step that comes back non-finite (over a singular Jacobian a step can be everything, and its middle is no number), or a refined box disjoint from the start, returns the incoming box unchanged, and an iterate leaving the domain is clamped back into it, since a foot point can step outside on its way in. "Refinement may only tighten, never fail, or callers start needing fallbacks of their own" (AGENTS.md).
- **The result is `start.intersect(x)`.** The search proved the root lies in `start`, and Newton computed another enclosure of the same root. Both hold, so their intersection is valid and tighter than either: exactly the case [Lecture 5](./05-honest-numerics.md) says intersection is for.
- **Disjoint means reject.** If Newton ended somewhere that does not overlap the proved box, it wandered off, and `start` is the answer.
- **Clamp after sharpening.** Clamping an interval that straddles a domain bound leaves it straddling, because its three-valued comparisons have no answer. Clamping a sharp value is exact.

`ITERATIONS` is 12. Quadratic convergence makes that plenty, and it cannot change which solutions are found.

The doc comment adds a warning: this is "plain, unverified Newton: it does not itself certify existence or uniqueness, so callers check that the refined box can still hold what they are looking for". And they do: `refine_crossing` evaluates both shapes over the refined box and returns the incoming one unless they could coincide. AGENTS.md has a term for this habit: *check an iterative answer against every condition it was asked to meet*.

### Refine the target first

A trap is worth describing separately. A refinement is only as good as its target. Refining a pcurve parameter against a `$(u,v)$` that came straight from a search, itself `min_subdivision_size` wide, gives a wide residual, a wide step, and a result no narrower than it started. Silently, because the path that fails looks identical to success. The target has to be refined first (`NurbSurface::project` on the vertex point). The same applies to a vertex built by evaluating a wide crossing parameter: fix the parameter at the source, or everything downstream inherits its width.

### Refine where the parameter is used

`curve_surface_intersect` deliberately does not refine everything it returns. Most callers only need to know where and how many, and refining changes results they already agree with. As AGENTS.md notes, doing it unconditionally broke `offset_sphere_is_valid`. So refining is done when the parameter becomes a split point, where width genuinely matters.

That leads to the kernel's rule for cutting a curve at a vertex, which illustrates everything above in one function: `split_parameter_at_point` in `core/geop-core-geometry/src/nurb_curve/refine.rs`.

1. Refine the parameter with Newton on `$C(t)=$` the vertex.
2. Refuse if it could be an end of the domain: the vertex is then an end of the curve, and there is nothing to cut.
3. Choose where to cut, a sharp middle of the refined enclosure, or a knot if one lies in it. That is legitimate: every parameter of that enclosure ends a piece on the curve and within the vertex, so it is a free choice.
4. **Check the value you are about to use.** Evaluate at the sharp parameter, and refuse unless the curve reaches the vertex there.

Step 4 comes from a bug that AGENTS.md describes under "Validate the value you are about to use, not a wider one". `Model::split_edge_at_vertex` checked `curve.evaluate(edge_t).could_be_equal(&vertex_point)` on the *wide* `edge_t`, a fat box that genuinely contains the vertex, and then split at the *sharpened* `edge_t`. The check answered "is the vertex somewhere along this stretch?" and the split asked "cut exactly here". A different question.

## Interval Newton and Krawczyk: refining with a proof

All the above gives a narrow box, and a check that it could hold a root. It does not prove that a root exists. Sometimes we want a proof, and interval analysis has a neat way of getting one.

Take the mean value theorem. For a root `$x^*$` in an interval `$X$` and a point `$m\in X$`,

`$$0=f(x^*)=f(m)+f'(\zeta)(x^*-m)\quad\text{for some }\zeta\in X.$$`

Solving for the root, with `$f'(\zeta)$` enclosed by `$F'(X)$` over the interval:

`$$x^*\in N(X)=m-\frac{f(m)}{F'(X)}.$$`

This is the **interval Newton operator**, and it gives three outcomes.

- If `$N(X)\cap X$` is empty, then `$X$` holds no root.
- Otherwise every root in `$X$` lies in `$N(X)\cap X$`: an honest contraction.
- If `$N(X)\subset X$`, then `$X$` holds exactly one root: one exists by Brouwer's fixed point theorem applied to the operator, and there is no second since `$F'(X)$` does not contain 0.

Example: `$f(x)=x^2-2$` on `$X=[1.4,1.5]$`. Take `$m=1.45$`, `$f(m)=0.1025$`, `$F'(X)=[2.8,3]$`. Then `$f(m)/F'(X)=[0.03417,0.03661]$`, and `$N(X)=[1.41339,1.41583]\subset X$`. We have just *proved* that `$\sqrt2$` is in that narrow interval, using finitely many interval operations.

For systems, dividing by an interval matrix is awkward. **Krawczyk's** form avoids it, with a fixed preconditioner `$Y\approx J(m)^{-1}$`:

`$$K(X)=m-YF(m)+\bigl(I-YJ(X)\bigr)(X-m).$$`

Again every root in `$X$` lies in `$K(X)$`, and if `$K(X)\subset X$` there is exactly one. `$Y$` is a free choice: the proof is valid for any invertible `$Y$`, and only its tightness depends on how close it is to the true inverse. So it can be sharp. In the example, `$Y=1/2.9$`, `$I-YJ(X)=[-0.0345,0.0345]$`, and `$K(X)=[1.41293,1.41638]\subset X$`.

<div class="kernel">

**In the kernel.** The refinements of Part III are plain Newton with checks. Geop uses Krawczyk's test where a proof is the point:

- `NurbSurface::enclose_foot` in `core/geop-core-geometry/src/nurb_surface/project.rs` proves that the foot point of a target (or a box of targets) lies in a box, by the test above on the distance's derivative, with `$Y$` the inverse Hessian at the middle. A patch's tilt off a curved face is proven over that box (`NurbSurface3D::strays`; see [Lecture 28](./28-lofts-and-fills.md)).
- The sketch and assembly solvers prove their configurations in `core/geop-core-math/src/solvers/system/enclose.rs`: see [Lecture 24](./24-rank-and-proof.md).

Elsewhere:

| What | Where |
|---|---|
| The two-copy loop | `iterate`, `Repeat`, `Iterate::same` in `core/geop-core-math/src/solvers/iterate.rs` |
| Refine an isolated root | `newton::refine` in `core/geop-core-math/src/solvers/newton.rs` |
| Refine a curve-surface or curve-curve crossing | `refine_crossing`, re-exported from `core/geop-core-geometry/src/intersection/mod.rs`, with `refine_curve_curve_crossing` |
| Foot point | `NurbSurface::project`, `locate`, `off_pole` in `core/geop-core-geometry/src/nurb_surface/project.rs` |
| Where to cut a curve at a vertex | `NurbCurve::refine_parameter_at_point`, `split_parameter_at_point` in `core/geop-core-geometry/src/nurb_curve/refine.rs` |

The note `core/geop-core-geometry/src/intersection/curve_surface.md`, section 4, discusses a verified Krawczyk refinement of crossings; `refine_crossing` as implemented is the plain form with a containment check, not that verified form.

</div>

## Recap

<div class="recap">

- Newton squares the error near a regular root, and finds nothing on its own. It starts where a search has left off.
- Gauss-Newton's error is multiplied by `$-d/R$` per step on a curve of radius `$R$`, for a target `$d$` outside it. From `$d=R$` on it does not converge in any useful sense. Newton's second-derivative term fixes that.
- An iterate is a seed, a free choice: sharpen it. The last one is the answer: keep its width. Keep two copies.
- A refinement returns the intersection with the proved box, and never fails. It is only as good as its target, and it is only done where the parameter is used.
- Check the value you are about to use, not a wider one.
- Interval Newton and Krawczyk turn the same formula into a proof of existence and uniqueness.

</div>

## Exercises

**1.** Derive `$\delta_{\text{N}}=\tan e$` for the circle, and show that `$e-\tan e=-\tfrac{e^3}{3}+O(e^5)$`. Then run the demo's ellipse and show from its readout that the error there is squared rather than cubed.

<details class="solution"><summary>Solution</summary>

With `$T$` at angle `$\theta$`: `$r=T-C$`, `$r\cdot C'=T\cdot C'$`, since `$C\cdot C'=0$`, and that is `$R\rho\sin(\theta-t)=R\rho\sin e$`. And `$r\cdot C''=T\cdot C''-C\cdot C''=-R\rho\cos e+R^2$`, so `$C'\cdot C'-r\cdot C''=R\rho\cos e$`. The ratio is `$\tan e$`. Using `$\tan e=e+e^3/3+\dots$` gives the expansion. On the ellipse with target `$(2.5,1.2)$` the demo's algorithm gives errors `$3.0\times10^{-1}$`, `$3.7\times10^{-2}$`, `$7.6\times10^{-4}$`, `$3.3\times10^{-7}$`, `$6.1\times10^{-14}$`. The ratios `$e_{k+1}/e_k^2$` are about `$0.42$`, `$0.55$`, `$0.56$`, `$0.56$`: a constant, as for quadratic convergence.

</details>

**2.** What does Gauss-Newton do on a concave side, that is for a target inside the circle at `$\rho<R$`? For which `$\rho$` is it fastest? What about `$\rho=0$`?

<details class="solution"><summary>Solution</summary>

The multiplier `$1-\rho/R$` lies in `$(0,1)$`: convergence without oscillation, and slower the closer to the centre. It is exact in one step at `$\rho=R$`. At `$\rho=0$`, `$T$` is the centre, the residual is everywhere square to the curve, `$\delta=0$`, and every point of the circle is a foot point. The problem has no unique answer, and no method can fix that.

</details>

**3.** Why would a "convergence tolerance" be wrong in `project`? Suppose we stop once `$\lVert\delta\rVert<10^{-12}$`. Give one way this goes wrong for a small model and one for a large model.

<details class="solution"><summary>Hint</summary>

The parameter's scale is not the geometry's scale. On a large-domain surface, a step of `$10^{-12}$` in parameter can mean a large distance; on a surface with a tiny parameter domain, the stop triggers immediately. And interval scalars cannot judge "close enough" anyway: the right test is exact repetition of a seed, or a fixed budget, with the width left honest.

</details>

**4.** Consider `$f(x)=x^2$` with the double root at 0. Starting from `$x_0=1$`, what does Newton do? What does this say about `refine_crossing` at a tangency? Compare with the iteration counts in the readout of the [tangency demo](./15-tangency-and-overlap.md).

<details class="solution"><summary>Solution</summary>

`$x_{k+1}=x_k-x_k^2/2x_k=x_k/2$`. Each step halves the error, as bisection would, and that takes 50 steps for double precision. In the interval setting, the Jacobian's enclosure soon contains zero, the linear solve fails, and `newton::refine` returns the incoming box. So at a tangency refinement silently does not narrow. That is correct, not a failure: see the next lecture.

</details>

**5.** Do one Krawczyk step by hand for the system `$x^2+y^2=1$`, `$x=y$`, on `$X=[0.6,0.8]^2$`, with `$m=(0.7,0.7)$`, and `$Y$` the inverse of `$J(m)$`. Does `$K(X)\subset X$` prove a unique root?

<details class="solution"><summary>Solution</summary>

`$F(m)=(0.98-1,\ 0)=(-0.02,\,0)$`, `$J(x,y)=\begin{bmatrix}2x&2y\\1&-1\end{bmatrix}$`, so `$J(m)=\begin{bmatrix}1.4&1.4\\1&-1\end{bmatrix}$`, whose inverse is `$Y=\begin{bmatrix}1/2.8&1/2\\1/2.8&-1/2\end{bmatrix}$`. `$-YF(m)=(0.02/2.8,\ 0.02/2.8)=(0.00714,\ 0.00714)$`. Over `$X$`, `$J(X)=\begin{bmatrix}[1.2,1.6]&[1.2,1.6]\\1&-1\end{bmatrix}$`, so `$J(X)-J(m)$` has `$[-0.2,0.2]$` in the first row only. `$I-YJ(X)=-Y(J(X)-J(m))$` then has every entry in `$\tfrac{1}{2.8}[-0.2,0.2]=[-0.0714,0.0714]$`. Times `$X-m=[-0.1,0.1]^2$`, each row sums to `$[-0.0143,0.0143]$`. So `$K(X)=0.70714\pm0.0143$` per coordinate, which is `$[0.6929,0.7214]\subset[0.6,0.8]$`. Yes: there is exactly one root in `$X$`, and it lies there. It is `$\tfrac{\sqrt2}{2}=0.70711$`.

</details>

**6.** `newton::refine` sharpens *before* clamping. Show why the other order can produce an iterate outside the domain.

<details class="solution"><summary>Hint</summary>

Let the iterate be an interval straddling `hi`. `clamped` compares three-valued, and it cannot pull back an interval that merely straddles the bound. Sharpening it afterwards picks a midpoint that can lie past `hi`, and evaluating a B-spline there is not allowed. Sharp first makes the comparison exact, so the seed is in the domain.

</details>
