# Lecture 8: NURBS and exact conics

<div class="goals">

**After this lecture you can**

- explain a rational curve as a polynomial curve in one more dimension, seen from the origin;
- state the rational basis, and say what a weight does to the curve, and what it does not do;
- derive the middle weight `$\cos(\theta/2)$` that makes a quadratic an exact arc, and check it numerically;
- classify a rational quadratic as ellipse, parabola or hyperbola from its weights;
- differentiate a rational curve;
- explain why every weight in this kernel is required to be definitely positive, and why its circles are only `$C^1$` at the joints.

</div>

[Lecture 7](./07-bsplines.md) gave us piecewise polynomials, and left one gap: a polynomial curve cannot be a circle (exercise 5 of [Lecture 6](./06-bezier.md)). This is no academic detail. A hole, a shaft, a fillet round a straight edge, a thread: machined parts are made largely of cylinders and cones, and their cross sections are circles. Approximating a circle by a cubic to `$10^{-5}$` is fine for a picture, and worthless for a kernel, where two cylinders that are in truth tangent must be recognised as tangent to within a few ulps.

Older kernels answer this by storing a circle as a separate kind of thing, with its own intersection formulas against every other kind. The history in `DEVELOPERS.md` ("Geop 1") records that this was tried and "didn't scale as equations became too complex". The other answer is to widen the representation slightly, until the circle is merely a particular set of control points. That is NURBS: *non-uniform rational B-splines*. "Non-uniform" we have already met, the knots may be anywhere. The new word is "rational".

## A curve seen from the origin

Take a polynomial curve in `$\mathbb{R}^3$`, and write its coordinates `$(X(t),Y(t),W(t))$`. Now look at it from the origin. Each point on it lies on a ray through the origin, and the ray crosses the plane `$W=1$` in one point:

`$$C(t)=\left(\frac{X(t)}{W(t)},\ \frac{Y(t)}{W(t)}\right).$$`

So the *shadow* of a polynomial curve, projected through the origin onto a plane, is a curve whose coordinates are ratios of polynomials. That is what a rational curve is: **a polynomial curve in a space of one more dimension, projected**. The three-dimensional space is called **homogeneous space**.

Everything we know about polynomial curves applies upstairs, and only at the end do we divide. The kernel takes this literally: control points are stored upstairs. `NurbCurve<4>` has `$(wx,wy,wz,w)$`, `NurbCurve<3>` has `$(wu,wv,w)$` (see the doc comment in `core/geop-core-geometry/src/nurb_curve/mod.rs`), and `de_boor` runs on them as a plain polynomial computation "before any division by the weight", as the header of `core/geop-core-geometry/src/spline.rs` says.

## From control points to weights

A control point upstairs is a point `$(X_i,Y_i,W_i)$`. Dividing by the last coordinate gives its shadow `$P_i=(X_i/W_i,Y_i/W_i)$`, and the value `$w_i=W_i$` is its **weight**. So `$(X_i,Y_i,W_i)=(w_ix_i,w_iy_i,w_i)$`. Summing upstairs and dividing:

`$$C(t)=\frac{\sum_i N_{i,p}(t)\,w_i\,P_i}{\sum_i N_{i,p}(t)\,w_i}.$$`

Grouping the terms gives a **rational basis**:

`$$C(t)=\sum_i R_i(t)\,P_i,\qquad R_i(t)=\frac{N_{i,p}(t)\,w_i}{\sum_j N_{j,p}(t)\,w_j}.$$`

If all weights are equal, they cancel, and we have the polynomial B-spline back. If all weights are positive, the `$R_i$` are non-negative and sum to one. So *with positive weights all our properties survive*: end-point interpolation for clamped knots, local support, and the strong convex hull: a span of the curve lies in the hull of the shadows of its active points.

Weights act like a pull. Raising `$w_i$` draws the curve towards `$P_i$` without changing `$P_i$`.

One property is even better than before. A polynomial curve is preserved by an affine map, but a rational one is preserved by any **projective** map: apply a matrix upstairs, and the curve follows. So a perspective view of a NURBS curve is again a NURBS curve, with no approximation.

<figure class="demo" data-demo="nurbs-weights">
<figcaption>Upper plot: a rational quadratic in the plane. Drag the control points, change the weights, and try the presets. Lower plot: the same curve upstairs. Blue is the polynomial curve in homogeneous space, with its control points at <code>(w x, w y, w)</code>; the other curve is its shadow in the plane w = 1, on the rays from the origin. Drag this plot to turn it. The readout gives the conic type, and for the circle presets the largest error in the radius over 2000 samples per span.</figcaption>
</figure>

Things to try: set all three weights to 2 and see nothing move in the upper plot, while upstairs the curve is scaled along its rays. Then try the preset "same curve" with weights `$(2,\sqrt2,1)$`. The curve in the plane is the same set of points as with weights `$(1,1,1)$`, but the red point has moved along it: the *parametrization* changed. Only one combination of the weights matters for the shape. We come to it now.

## Rational quadratics are conics

Consider a single quadratic span with weights `$w_0,w_1,w_2$`:

`$$C(t)=\frac{(1-t)^2w_0P_0+2t(1-t)w_1P_1+t^2w_2P_2}{(1-t)^2w_0+2t(1-t)w_1+t^2w_2}.$$`

Assume `$P_0,P_1,P_2$` are not collinear, and write any point `$Q$` of the plane in barycentric coordinates relative to them: `$Q=aP_0+bP_1+cP_2$`, `$a+b+c=1$`. Reading the coefficients off the formula, a point on the curve has `$a:b:c=(1-t)^2w_0:2t(1-t)w_1:t^2w_2$`, so

`$$b^2\,w_0w_2=4\,a\,c\,w_1^2.$$`

This is a homogeneous quadratic equation in `$(a,b,c)$`, and those are affine functions of position. So every rational quadratic lies on a **conic section**, with `$P_0P_1$` and `$P_2P_1$` as tangents at its ends. Conversely, any conic arc whose end tangents meet in a finite point is one such span.

The equation has one parameter. Dividing through, only the ratio

`$$k=\frac{w_1^2}{w_0w_2}$$`

appears, so `$k$` determines the shape alone. Changing the three weights while keeping `$k$` is the reparametrization you tried in the demo, `$t\mapsto\frac{\rho t}{1-t+\rho t}$`. We can therefore normalize to `$w_0=w_2=1$`, leaving `$w=w_1=\sqrt{k}$`:

| `$w$` | conic |
|---|---|
| `$0<w<1$` | ellipse |
| `$w=1$` | parabola |
| `$w>1$` | hyperbola |

The parabola case is what you should expect: equal weights give a polynomial quadratic, and a polynomial quadratic is a parabola. As for the rest, look at the middle of the span. By the formula `$C(\tfrac12)$` is on the segment from the chord's midpoint `$M=\tfrac12(P_0+P_2)$` to `$P_1$`, at the fraction `$\frac{w}{1+w}$` of the way. So `$w$` runs from the chord (`$w=0$`) towards `$P_1$` (`$w\to\infty$`) with the parabola at the half.

## The exact circle

Now the construction every circle in this kernel is built from. We want an arc of a circle of radius `$r$` centred at the origin, turning through an angle `$\theta<\pi$`, symmetric about the `$x$`-axis. Take

`$$P_0=r(\cos\tfrac\theta2,-\sin\tfrac\theta2),\qquad P_2=r(\cos\tfrac\theta2,\sin\tfrac\theta2).$$`

A conic arc's end tangents meet at `$P_1$`, so `$P_1$` has to be where the circle's tangents at `$P_0$` and `$P_2$` meet:

`$$P_1=\left(\frac{r}{\cos\frac\theta2},\ 0\right).$$`

By symmetry, the curve's midpoint is on the `$x$`-axis, and it must be at `$(r,0)$`. The midpoint is `$M+\frac{w}{1+w}(P_1-M)$`, with `$M=(r\cos\frac\theta2,0)$`. Write `$c=\cos\frac\theta2$`:

`$$r=rc+\frac{w}{1+w}\left(\frac rc-rc\right)\ \Longrightarrow\ 1-c=\frac{w}{1+w}\cdot\frac{1-c^2}{c}\ \Longrightarrow\ \frac{w}{1+w}=\frac{c}{1+c}\ \Longrightarrow\ w=c.$$`

So the middle weight is

`$$w=\cos\frac\theta2.$$`

For a quarter turn, `$\theta=90^\circ$`, this is `$\cos45^\circ=\frac{\sqrt2}{2}$`. Since `$w<1$` the conic is an ellipse, and we still have to check that this ellipse is a circle. It is: a conic is fixed by five conditions, we imposed two end points, two tangents, and one more point; the circle meets them all. For a direct check, take the quarter arc `$P_0=(1,0)$`, `$P_1=(1,1)$`, `$P_2=(0,1)$`, `$w=\frac{\sqrt2}{2}$`:

`$$x=\frac{(1-t)^2+\sqrt2\,t(1-t)}{D},\quad y=\frac{\sqrt2\,t(1-t)+t^2}{D},\quad D=(1-t)^2+\sqrt2\,t(1-t)+t^2,$$`

and a short expansion (exercise 2) gives `$x^2+y^2=1$` for all `$t$`.

Try the circle presets in the demo: the largest deviation from the radius, over 2000 samples of every span, is of the order of `$10^{-15}$`. That is rounding in the double-precision sampling of `$w$` and the arithmetic. It is not an approximation error, and doesn't shrink or grow as you add spans.

### A whole circle

A full circle needs more than one span, as `$\theta<\pi$`. It is usually four quarters. Each span is its own Bézier piece, so each interior knot has multiplicity two, and the curve passes through a control point at every joint:

`$$U=(0,0,0,\ \tfrac14,\tfrac14,\ \tfrac12,\tfrac12,\ \tfrac34,\tfrac34,\ 1,1,1),$$`

with nine control points (the first and last coincide) and weights `$1,\frac{\sqrt2}2,1,\frac{\sqrt2}2,\dots$`. This is what `NurbSurface::revolve` writes in `$u$`: `$2n+1$` rows, knots at multiplicity two, in `core/geop-core-geometry/src/nurb_surface/revolve.rs`.

### The price of the circle

Two things are lost.

**The parameter does not run at constant speed.** No rational parametrization of a circle has `$\lvert C'\rvert$` constant. Over a quarter arc with `$w=\frac{\sqrt2}2$` the speed in the middle is about 1.17 times the speed at the ends (exercise 3). So the parameter is not the angle, and finding "the point at 30 degrees" takes a little computation. The kernel's helices run into this directly: the header of `core/geop-core-geometry/src/nurb_curve/helix.rs` explains that a helix cannot be a rational curve, because its height grows linearly with the angle, and "no rational parametrization of a circle turns at a constant rate". So it keeps the circle exact, and approximates the height: quarter-turn spans stay within 0.53% of the pitch, "5.3e-3 times the pitch, measured" in the comment. The curve lies exactly on its cylinder; only its rise is approximate.

**The joints are only `$C^1$`, even where the shape is perfect.** Across a knot of multiplicity 2 in a quadratic, a rational curve is only guaranteed `$C^0$`, by the table in Lecture 7. With equal spans of a circle, the two speeds at a joint agree and so does the direction, so the first derivative is continuous. The second is not: the speed is rising on one side of the joint and falling on the other. The circle has constant curvature, so the *shape* is as smooth as can be; only its parametrization has kinks in its acceleration.

That detail has practical consequences. AGENTS.md's section on hash maps ends with a passage on exactly this: the kernel's arcs and helices are rational quadratics, "only C1 where their spans meet". `fit_pcurve` used to fit one `$C^2$` cubic through 48 samples of the whole curve, and across those joints the drift shrank only as `$h^2$`. AGENTS.md says a helix of pitch 1 came within 15% of the `1e-4` an entity may carry. The joints are known: they are the knots of the source curve. So it now fits each smooth piece separately and joins them. The lesson: **use the structure you know before you sample blindly**. We return to this in [Lecture 16](./16-tracing-and-pcurves.md).

<div class="kernel">

**In the kernel.** The constructors are short and worth reading in `core/geop-core-geometry/src/nurb_curve/construct.rs`. `NurbCurve3D::arc` takes the three points and the weight, with its doc comment stating the construction above: "With `mid` where the tangents of a circle at `p0` and `p2` meet and `w` the cosine of half the angle between them, it is that circle's arc exactly".

`sqrt2_over_2` and `arc_span` are in `core/geop-core-geometry/src/nurb_curve/helix.rs`. `arc_span(step)` returns the weight `$\cos(\text{step}/2)$`, and the factor `$1/(1+\cos\text{step})$` that puts the middle point where the tangents meet, along the sum of the unit directions to the ends. Check it against our formula: `$\lvert\hat p_0+\hat p_2\rvert=2\cos\frac\theta2$`, so the distance out is `$\frac{2c}{1+\cos\theta}=\frac{2c}{2c^2}=\frac1c$`, as derived. Revolves and helices both use it, "so a helix's quarter lies on the very circle a revolved quarter does."

`cos_sin` returns exact values at every quarter turn. This is not vanity: at those angles a revolve lands exactly on the plane it should, and the construction code is sensitive at the last bit ([Lecture 26](./26-sweeps.md)).

A caution about the word "exact": `sqrt2_over_2` is a sharp `Scalar` holding the double nearest to `$\frac{\sqrt2}{2}$` that the division gives, not an enclosure of the irrational number. So the construction is exact in form, and in numbers it is exact to the rounding of that weight. Where code has to recognise the shape back from a net, as a sketch projecting an edge does, `as_arc` in `core/geop-core-geometry/src/shape.rs` asks whether the net *could* be exactly an arc under the three-valued comparisons.

</div>

## Positive weights, always

Negative or zero weights are legal in some systems. Geop refuses them. Look at the check in `NurbCurve::try_new`, in `core/geop-core-geometry/src/nurb_curve/mod.rs`:

```rust
if !p[D - 1].definitely_greater(Scalar::ZERO) { /* error */ }
```

The reason is everything built on the denominator. If all weights are positive, then `$W(t)=\sum_iN_i(t)w_i$` is positive on the whole domain, because it is a convex combination of positive numbers. Then:

- The convex hull property holds, and every search's first test is valid.
- `$C(t)$` never divides by zero, and in fact the division can be avoided entirely. The containment search asks where `$X_k(t)-p_kW(t)=0$` (`contains::curve`), a polynomial equation with the same zeros and no quotient. We will use this in [Lecture 12](./12-subdivide-to-isolate.md).
- Splitting forms convex combinations of existing homogeneous points, so it preserves positivity.

Note the three-valued test: a weight must be *definitely* positive, not merely `$>0$` at its midpoint. An interval weight of `$[-10^{-20},1]$` is rejected.

What about a semicircle in one span? `$\theta=\pi$` gives `$w=0$`, with `$P_1$` at infinity. A homogeneous control point `$(X,Y,0)$` is well defined upstairs; downstairs it is a direction. This is the classical trick, and it is refused here for the reasons above. Revolves and helices are therefore cut into spans of at most a quarter turn (`NurbSurface::revolve`, `helix_rows`).

One more thing worth noting: extrapolating a rational curve is no convex combination, so it can produce a weight that is not positive. `NurbCurve::extended`'s comment says it fails "where the continued weights stop being positive", and since it ends in `try_new`, that is where such a failure would surface.

## Derivatives of a rational curve

The quotient rule applies, but it pays to organize it upstairs, where everything is polynomial. Write `$A(t)=W(t)\,C(t)$`, with `$A$` the numerator. Then by Leibniz

`$$A^{(k)}=\sum_{i=0}^{k}\binom{k}{i}W^{(i)}C^{(k-i)},$$`

and solving for the last term:

`$$C^{(k)}=\frac{1}{W}\left(A^{(k)}-\sum_{i=1}^{k}\binom{k}{i}W^{(i)}C^{(k-i)}\right).$$`

For `$k=1$`: `$C'=(A'-W'C)/W$`. This is algorithm A4.2 of Piegl and Tiller, and it is `rational_derivatives` in `core/geop-core-geometry/src/spline.rs`, line for line. `$A^{(k)}$` and `$W^{(k)}$` are derivatives of polynomials, computed by Lecture 7's hodograph.

As an example, take the quarter arc at `$t=0$`. `$A'(0)=2(w_1P_1-w_0P_0)$` with `$w_0=1$`, `$w_1=\frac{\sqrt2}2$`, `$P_0=(1,0)$`, `$P_1=(1,1)$`, so `$A'(0)=2(\frac{\sqrt2}{2}-1,\frac{\sqrt2}{2})$`, and `$W'(0)=2(\frac{\sqrt2}2-1)$`. Then `$C'(0)=A'(0)-W'(0)P_0=(0,\sqrt2)$`. So `$\lvert C'(0)\rvert=\sqrt2$` for a unit circle. In general `$C'(0)=2\frac{w_1}{w_0}(P_1-P_0)$`, which is a useful fact: the end tangent points along the first leg, as in the polynomial case, scaled by the ratio of the weights.

The middle of the arc goes faster: by symmetry `$W'(\tfrac12)=0$`, so `$\lvert C'(\tfrac12)\rvert=\lvert A'(\tfrac12)\rvert/W(\tfrac12)$`, which exercise 3 works out to about 1.657, against 1.414 at the ends.

Look again at the formula for `$C'$`, and think about what happens for an *interval* `$t$`. `$A'$`, `$W'$`, `$C$` and `$W$` are all enclosures, each computed separately, and interval arithmetic cannot know that they are correlated. Far from the origin, `$\lvert A\rvert$` is large, and this formula is in trouble. That is the subject of [Lecture 11](./11-evaluate-locally.md).

## Recap

<div class="recap">

- A rational curve is a polynomial curve one dimension up, projected through the origin. The kernel stores the upstairs points, and works there.
- With positive weights, the rational basis is non-negative and sums to one, so every property of B-splines survives. Geop requires every weight to be definitely positive.
- A rational quadratic is a conic, of shape set by `$w_1^2/(w_0w_2)$`: ellipse below one, parabola at one, hyperbola above.
- An arc through `$\theta$` has its middle control point where the end tangents meet, and middle weight `$\cos(\theta/2)$`.
- The price is a non-uniform parameter, and joints no smoother than `$C^1$`.
- Derivatives come from the Leibniz rule upstairs, `$C^{(k)}=(A^{(k)}-\sum\binom kiW^{(i)}C^{(k-i)})/W$`.

</div>

## Exercises

**1.** Show that multiplying every weight of a NURBS curve by the same constant leaves the curve unchanged, parametrization included. What does this correspond to upstairs?

<details class="solution"><summary>Solution</summary>

Numerator and denominator of `$R_i$` both scale by the constant. Upstairs the whole curve is scaled by a factor about the origin, which keeps every point on its ray.

</details>

**2.** Check `$x^2+y^2=1$` for the quarter arc given in the lecture.

<details class="solution"><summary>Solution</summary>

Put `$a=(1-t)^2$`, `$b=\sqrt2t(1-t)$`, `$e=t^2$`. Then `$x=(a+b)/D$`, `$y=(b+e)/D$`, `$D=a+b+e$`. We need `$(a+b)^2+(b+e)^2=(a+b+e)^2$`, i.e. `$b^2=2ae$` after expanding. Indeed `$b^2=2t^2(1-t)^2=2ae$`. The weight `$\frac{\sqrt2}2$` is what makes this hold.

</details>

**3.** Compute `$\lvert C'(t)\rvert$` at `$t=0$` and `$t=\tfrac12$` for the unit quarter arc. What is their ratio? Do the same for a span of `$120^\circ$`.

<details class="solution"><summary>Solution</summary>

At `$t=0$`: `$2w=\sqrt2\approx1.414$`. For `$t=\tfrac12$`: from the formula `$C'=(A'-W'C)/W$`, `$W'(\tfrac12)=0$`, `$W(\tfrac12)=\tfrac14+\tfrac12w+\tfrac14=\tfrac12(1+w)$`, and `$A'(\tfrac12)=(P_2-P_0)$`, of length `$\sqrt2$`. So `$\lvert C'(\tfrac12)\rvert=\frac{2\sqrt2}{1+w}\approx1.657$`. So it is faster in the middle, by the ratio `$2/(1+w)\approx1.17$`. (AGENTS.md's "Past a face is not off its surface" has a bug that came from this: a patch's side judged by the speed at its vertex was not carried far enough, because a quarter arc is faster in its middle.) For a `$120^\circ$` span, `$w=\tfrac12$`, the ends' speed is `$2w\lvert P_1-P_0\rvert=\tan60^\circ\approx1.732$`, the middle's `$\lvert P_2-P_0\rvert/W(\tfrac12)=\sqrt3/\tfrac34\approx2.309$`: ratio `$\tfrac43$`. The ratio grows with the angle.

</details>

**4.** Derive the formula `$C(\tfrac12)=M+\frac{w}{1+w}(P_1-M)$` for a rational quadratic with `$w_0=w_2=1$`. What does the formula say about `$w\to0$` and `$w\to\infty$`?

<details class="solution"><summary>Solution</summary>

At `$t=\tfrac12$` the numerator is `$\tfrac14(P_0+P_2)+\tfrac12wP_1$`, the denominator `$\tfrac12(1+w)$`. So `$C=\frac{\frac12(P_0+P_2)+wP_1}{1+w}=\frac{M+wP_1}{1+w}=M+\frac{w}{1+w}(P_1-M)$`. As `$w\to0$` it goes to the chord's midpoint and the curve degenerates to the segment `$P_0P_2$`. As `$w\to\infty$` it approaches `$P_1$` and the curve to the two legs.

</details>

**5.** Write the nine homogeneous control points of a unit circle in the plane as `$(wx,wy,w)$`. Then verify by evaluating with the formula at `$t=\tfrac18$` in the knot vector above.

<details class="solution"><summary>Solution</summary>

With `$h=\frac{\sqrt2}2$`: `$(1,0,1),(h,h,h),(0,1,1),(-h,h,h),(-1,0,1),(-h,-h,h),(0,-1,1),(h,-h,h),(1,0,1)$`. On the first span, local parameter `$t'=\tfrac12$`, so `$X=\tfrac14+\tfrac12h=\tfrac14+\tfrac{\sqrt2}{4}$`, `$Y=\tfrac12h+\tfrac14$` (equal), `$W=\tfrac14+\tfrac12h+\tfrac14$`. Then `$x=y=\frac{1+\sqrt2}{2+\sqrt2}=\frac{\sqrt2}{2}$` and the point is at `$45^\circ$`.

</details>

**6.** *Reading the code.* `NurbSurface::revolve` writes a control row with `homogeneous(middle, w.mul(middle_weight))`. Why does a profile control point's own weight `w` multiply the arc's weight? What surface would you get by revolving the unit quarter arc, itself rational, about an axis through its centre?

<details class="solution"><summary>Solution</summary>

A tensor-product weight is the product of the two directions' weights. Upstairs the surface is the product construction: the row in `$u$` of a profile point at weight `$w$` is that point, scaled by the arc's homogeneous factors. Revolving a quarter arc about an axis through its centre gives a patch of a sphere. Its weights are products: `$1,\frac{\sqrt2}2,\frac12$`. This patch has a pole if an end of the arc lies on the axis.

</details>
