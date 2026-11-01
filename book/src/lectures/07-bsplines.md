# Lecture 7: B-splines

<div class="goals">

**After this lecture you can**

- define a B-spline curve by its degree, control points and knot vector, and compute its basis functions by the Cox-de Boor recursion;
- explain local support, the strong convex hull property, and how knot multiplicity sets continuity;
- evaluate a curve by de Boor's algorithm, and compute its derivative as another B-spline;
- say why a clamped knot vector makes a curve's ends meaningful;
- fit a spline through points, and explain why the choice of parameters is not innocent;
- read `core/geop-core-geometry/src/spline.rs` and say what an interval parameter does to each function in it.

</div>

[Lecture 6](./06-bezier.md) ended with a defect. A Bézier curve is one polynomial, so a change to any control point changes all of it, and a long intricate curve needs a high degree. Neither is acceptable in design. An engineer who adjusts the bulge near one end of a wing section does not want the other end to move. And a degree-40 polynomial is a numerically fragile object in any basis.

The fix is obvious in outline: use many low-degree polynomial pieces, joined with an agreed smoothness at their joints. What is not obvious is that this can be done with a single formula, keeping every good property of the Bernstein form, in such a way that the joint constraints hold *automatically* for any choice of control points. That formula is the B-spline.

## Why not simply chain Bézier curves?

You could. Two cubics `$P_0,\dots,P_3$` and `$Q_0,\dots,Q_3$` meet with a common tangent if `$Q_0=P_3$` and `$Q_1-Q_0=P_3-P_2$`. Asking for a continuous second derivative as well adds a further relation among `$P_1,P_2,P_3,Q_0,Q_1,Q_2$`. So out of the eight control points of two pieces, only a few are actually free. If a user drags one of them, you must move others to preserve the joints, and the correction propagates along the chain.

What we would like is to work with *only the free quantities*: a set of numbers such that any values give a curve with the desired smoothness. For a chain of `$s$` cubic pieces joined `$C^2$`, a count gives `$4s$` coefficients per coordinate, minus `$3(s-1)$` constraints, which is `$s+3$` free numbers. So we seek `$s+3$` basis functions, each piecewise cubic and `$C^2$`, such that any weighted sum has those joints. That is exactly what B-spline basis functions are.

## The knot vector

Fix a degree `$p$` and a non-decreasing sequence of numbers called the **knot vector**,

`$$U = (u_0, u_1, \dots, u_{m}),\qquad u_i\le u_{i+1}.$$`

The intervals `$[u_k,u_{k+1})$` are the **knot spans**. They are where the curve is a single polynomial. A value may repeat; the number of times it does is its **multiplicity**. Repeating a knot will turn out to be how we ask for less smoothness at that joint.

## The Cox-de Boor recursion

The basis functions are defined from the knots alone, by recursion on the degree:

`$$N_{i,0}(t) = \begin{cases}1 & u_i\le t<u_{i+1}\\ 0 & \text{otherwise}\end{cases}$$`

`$$N_{i,p}(t) = \frac{t-u_i}{u_{i+p}-u_i}\,N_{i,p-1}(t) + \frac{u_{i+p+1}-t}{u_{i+p+1}-u_{i+1}}\,N_{i+1,p-1}(t),$$`

with the convention that a fraction whose denominator is zero counts as zero (`$0/0=0$`): its partner `$N$` is zero too, on an empty span.

Each level is a blend of two neighbours of the level below, with a weight that runs linearly from 0 to 1 across the support. Compare it with `$P_i^k=(1-t)P_i^{k-1}+tP_{i+1}^{k-1}$`: this is de Casteljau's idea, with `$t$` replaced by a local fraction depending on the knots.

Given `$n+1$` control points `$P_0,\dots,P_n$`, the **B-spline curve** is

`$$C(t) = \sum_{i=0}^{n} N_{i,p}(t)\,P_i.$$`

The counting that makes this consistent: `$n+1$` basis functions of degree `$p$` need `$n+p+2$` knots, `$u_0,\dots,u_{n+p+1}$`. The curve is defined on the **domain** `$[u_p,u_{n+1}]$`. Outside it fewer than `$p+1$` basis functions are in play and they no longer sum to one.

<figure class="demo" data-demo="bspline-basis">
<figcaption>Upper plot: drag the control points. The curve changes colour at each knot, where one polynomial ends and the next begins. Control points circled in red are the only ones active at the red point. Lower plot: the basis functions, with one handle per interior knot; drag a knot onto its neighbour to raise the multiplicity. The readout says the continuity at each joint, and gives the value both by de Boor and by the sum of basis functions.</figcaption>
</figure>

Try these while reading on. Move a control point far from the red marker and notice which colours of the curve stay put. Drag two knots together in degree 3 and watch the curve acquire a visible crease in curvature. Press "Triple knot": the curve now passes through a control point with a corner. Set the degree to 6: with seven control points, this is one span, and the basis functions are the Bernstein polynomials.

### A small worked example

Take `$p=2$` and knots `$U=(0,0,0,1,2,2,2)$`, so `$n+1=4$` control points and domain `$[0,2]$`. On the first span `$[0,1)$`, at `$t=\tfrac12$`, work upward. Only `$N_{2,0}(t)=1$`. At degree 1:

`$$N_{1,1}=\frac{u_3-t}{u_3-u_2}N_{2,0}=\tfrac12,\qquad N_{2,1}=\frac{t-u_2}{u_3-u_2}N_{2,0}=\tfrac12.$$`

At degree 2:

`$$N_{0,2}=\frac{u_3-t}{u_3-u_1}N_{1,1}=\tfrac12\cdot\tfrac12=\tfrac14,$$`
`$$N_{1,2}=\frac{t-u_1}{u_3-u_1}N_{1,1}+\frac{u_4-t}{u_4-u_2}N_{2,1}=\tfrac12\cdot\tfrac12+\tfrac34\cdot\tfrac12=\tfrac58,$$`
`$$N_{2,2}=\frac{t-u_2}{u_4-u_2}N_{2,1}=\tfrac14\cdot\tfrac12=\tfrac18.$$`

They sum to `$\tfrac14+\tfrac58+\tfrac18=1$`, and `$N_{3,2}(\tfrac12)=0$`. With `$P=(0,0),(1,2),(3,2),(4,0)$` we get `$C(\tfrac12)=\tfrac58(1,2)+\tfrac18(3,2)=(1,\tfrac32)$`.

## Properties

**Local support.** By induction, `$N_{i,p}$` is zero outside `$[u_i,u_{i+p+1})$`. So on a span `$[u_k,u_{k+1})$`, only the `$p+1$` functions `$N_{k-p,p},\dots,N_{k,p}$` can be nonzero. Moving `$P_i$` affects at most `$p+1$` spans of the curve. This is the local control we were after, and it is why evaluation costs `$O(p^2)$` however many control points there are.

**Non-negative.** On its support every factor in the recursion is between 0 and 1, so `$N_{i,p}\ge0$`.

**Partition of unity.** On the domain, `$\sum_i N_{i,p}(t)=1$`. By induction on `$p$`: expanding the recursion and collecting the two terms in which `$N_{i,p-1}$` appears gives the coefficient `$\frac{t-u_i}{u_{i+p}-u_i}+\frac{u_{i+p}-t}{u_{i+p}-u_{i}}=1$`, so `$\sum_i N_{i,p}=\sum_i N_{i,p-1}$`, and at degree zero exactly one function is 1.

**The strong convex hull.** Together the last three give something stronger than Bézier's: on span `$k$`, the curve lies in the convex hull of *just its `$p+1$` active control points*, `$P_{k-p},\dots,P_k$`. A long B-spline is enclosed in a chain of small hulls rather than one big one.

**Affine invariance.** As before: partition of unity means that transforming the control points transforms the curve.

**It contains Bézier curves.** Take `$U=(\underbrace{0,\dots,0}_{p+1},\underbrace{1,\dots,1}_{p+1})$`. The recursion then reproduces the Bernstein polynomials (exercise 2). The kernel uses precisely this: `bezier_knots` in `core/geop-core-geometry/src/nurb_curve/construct.rs` is that vector, and a straight line is `NurbCurve::line` over `bezier_knots(1)`.

## Multiplicity and continuity

Each `$N_{i,p}$` is a polynomial of degree `$p$` on each span. Across a knot of multiplicity `$m$`, it is `$C^{p-m}$`. Therefore so is the curve:

| multiplicity at a joint | continuity of a cubic (`$p=3$`) | what you see |
|---|---|---|
| 1 | `$C^2$` | smooth, including curvature |
| 2 | `$C^1$` | tangent continuous, curvature may jump |
| 3 | `$C^0$` | a corner is possible; the curve passes through a control point |
| 4 | none | the curve may jump; it is really two curves |

This table is a statement about the parameter, not the shape: a `$C^1$` joint can still look perfectly smooth, and a curve can even be `$C^2$` and have a cusp if its derivative vanishes there. Geometric continuity (`$G^1$`: the unit tangent is continuous; `$G^2$`: the curvature is too) is the shape-level notion, and we meet it in [Lecture 29](./29-fillets.md).

Multiplicity gives us the vocabulary to reconcile Bézier and B-splines. A knot of multiplicity `$p$` at a joint makes the curve pass through a control point there, with each piece on its own. A knot of multiplicity `$p+1$` cuts the control points into two independent groups. Raising every interior knot to full multiplicity turns any B-spline into separate Bézier curves, which is how `NurbCurve::bezier_pieces` (`core/geop-core-geometry/src/nurb_curve/split.rs`) gets them, splitting at each knot. Knot insertion is the subject of [Lecture 9](./09-subdivision.md).

## Clamped knot vectors

If the first `$p+1$` knots are equal, and likewise the last `$p+1$`, the knot vector is **clamped**. Then, exactly as for Bézier curves, `$C(u_p)=P_0$` and `$C(u_{n+1})=P_n$`, and the end tangents lie along the first and last legs of the polygon. An unclamped curve starts somewhere inside the hull of its first few control points, a place you cannot read off from any single number.

For a kernel this matters a lot. An edge's curve has to start exactly at a vertex. If the first control point *is* the start of the curve, then the vertex is a stored number and no evaluation (and no rounding) is needed. The kernel relies on this in several places: `NurbCurve3D::start` and `NurbCurve3D::end` simply divide the first and last control points by their weights, for "the clamped knot vectors every curve here has", and `evaluate_into` (`core/geop-core-geometry/src/nurb_curve/evaluate.rs`) returns the end control point when a parameter pins a clamped end sharply. `is_clamped_at` in `core/geop-core-geometry/src/knot_insertion.rs` is the test, and `NurbCurve::extended` refuses to carry on a curve past an end that is not clamped.

## De Boor's algorithm

The sum of basis functions is the definition, but not how to compute. Just as de Casteljau beats the Bernstein sum, **de Boor's algorithm** evaluates by repeated linear interpolation of control points directly.

Let `$t$` lie in span `$k$`, `$u_k\le t<u_{k+1}$`. Start with the active points, `$d_j^0=P_j$` for `$j=k-p,\dots,k$`, and for `$r=1,\dots,p$`:

`$$d_j^{r} = (1-\alpha_j^{r})\,d_{j-1}^{r-1}+\alpha_j^{r}\,d_j^{r-1},\qquad \alpha_j^{r}=\frac{t-u_j}{u_{j+p-r+1}-u_j},\quad j=k-p+r,\dots,k.$$`

The answer is `$C(t)=d_k^{p}$`. Note that the denominators stretch over `$p-r+1$` spans around `$t$`, and for `$t$` in span `$k$` every `$\alpha_j^r$` lies in `$[0,1]$`, so each step is a convex combination. On one Bézier span, `$\alpha_j^r$` reduces to `$(t-a)/(b-a)$` for every `$j,r$`, and this is de Casteljau's algorithm.

Run the worked example: `$k=2$`, `$t=\tfrac12$`. Level one: `$d_1^1=\tfrac12P_0+\tfrac12P_1=(\tfrac12,1)$` (the denominator `$u_3-u_1=1$`), and `$d_2^1=\tfrac34P_1+\tfrac14P_2=(\tfrac32,2)$` (the denominator `$u_4-u_2=2$`). Level two: `$\alpha_2^2=(t-u_2)/(u_3-u_2)=\tfrac12$`, so `$d_2^2=\tfrac12(\tfrac12,1)+\tfrac12(\tfrac32,2)=(1,\tfrac32)$`, as before. The demo's grey polygons are the levels of this triangle.

<div class="kernel">

**In the kernel.** `de_boor` in `core/geop-core-geometry/src/spline.rs` is this triangle over homogeneous points (the next lecture says what those are), and it is the one place a curve or surface is evaluated. Two details are worth reading closely. A zero denominator is detected by `denom.could_be_equal(Scalar::ZERO)`, three-valued, not by comparing to an epsilon. And the convex combination goes through `Vector::interpolate`, which goes through `Scalar::interpolate`, whose choice of formula is the subject of a story in [Lecture 9](./09-subdivision.md).

`find_span` is the span search. Its comparisons are the interval ones of [Lecture 4](./04-intervals.md): the span is the first `$k$` for which `$t$` is not definitely below `$u_k$` and definitely below `$u_{k+1}$`, and a `$t$` that could be the domain's end takes the last non-empty span. A `$t$` definitely outside the domain is an error rather than a guess.

</div>

## Derivatives

Differentiating the recursion and simplifying gives a formula as clean as the Bézier one (Piegl and Tiller derive it in full; see [Appendix A](../appendix/references.md)):

`$$C'(t)=\sum_{i=0}^{n-1} N_{i+1,p-1}(t)\,Q_i,\qquad Q_i=\frac{p}{u_{i+p+1}-u_{i+1}}\,(P_{i+1}-P_i).$$`

The derivative is a B-spline of degree `$p-1$`, on the same knots with the first and last dropped, whose control points are scaled differences. It has the same properties, so the hull of the `$Q_i$` bounds every tangent on a span: a finite proof about all the tangents of a curve. For repeated knots the denominator vanishes and the term is dropped.

Why does a knot of multiplicity `$p$` allow a corner? The derivative spline has degree `$p-1$` and carries the same knot, now of multiplicity equal to its degree plus one: that is the row of the table where a spline may jump, here the tangent.

<div class="kernel">

**In the kernel.** `homogeneous_derivatives` forms only the differences acting on the span (`hodograph`) and evaluates each by `de_boor`. The doc comment says why it does not take the textbook route of summing basis function derivatives: there `$t$` occurs in every term, and for an interval `$t$` each occurrence is treated as independent, the dependency problem of [Lecture 4](./04-intervals.md). Convex combinations with differences taken first keep the enclosures tight. `NurbSurface::partials` follows the same order for the same reason, and its comment records what the other order cost: third differences of independently evaluated rows, divided by the span's width cubed, "came out a hundred times wider than the partial itself."

</div>

## A parameter that is an interval

So far `$t$` has been a number. In this kernel it is often an interval, the output of a search over a box. What does it mean to evaluate a spline over an interval that reaches across several knots?

The natural mistake is the one the kernel used to make. `find_span` picks one span. Evaluating that span's polynomial over a `$t$` that runs past the span's end is not the spline any more, it is the *polynomial continued*, and a polynomial piece of a fitted curve continued across other spans goes wild. It is no enclosure of the spline there. It holds the right value only by luck.

AGENTS.md records the consequence in *Count the work, not the seconds*: a spline evaluated over an interval reaching past one knot span used that span's polynomial for all of it, and came out 25 times too wide over a fitted curve, which a search then subdivided away at great cost. The comment on `find_spans` gives the same figure for a cubic fitted through 48 points and evaluated over its whole domain. And a test (`a_wide_parameter_encloses_the_curve_tightly` in `core/geop-core-geometry/src/nurb_curve/evaluate.rs`) records the extreme: a quarter circle of radius 2.25, interpolated through 49 points, came out 57 wide over its whole domain.

The fix is `find_spans`: when `$t$` is an interval reaching past one span, it returns every span it reaches, with the piece of `$t$` in that span; the curve's evaluation is the union of the pieces. Union, not intersection, since these are different points of the curve and the answer must cover them all.

Notably this is not yet complete, and the source says so. Curves unite their spans (`evaluate_into`, `cartesian_derivatives`), and so do a surface's `partials`, but `NurbSurface::evaluate` still extrapolates from `find_span`'s one span. The comment on `find_spans` records that uniting the pieces there left a boolean in `chamfer_cylinder_flat_side` with a face straddling a cylinder's cap: something downstream leans on the extrapolated width, and that has not been tracked down. AGENTS.md adds that the tangent-branch search on blends pays for this. This is honest engineering: a known limit, written next to the code that has it.

## Fitting a spline through points

A kernel makes splines two ways. A designer gives control points, or an algorithm gives **points on the curve**, typically the samples of a traced intersection ([Lecture 16](./16-tracing-and-pcurves.md)). Then we need the control points that pass through them.

Given data points `$Q_0,\dots,Q_m$`, the method (Piegl and Tiller, chapter 9) is:

1. **Choose parameters** `$t_k$`, one for each data point.
2. **Choose knots** so the system is well posed. The averaging technique sets `$u_{j+p}=\frac1p\sum_{i=j}^{j+p-1}t_i$`, which guarantees each span contains a parameter value; by the Schoenberg-Whitney condition, the matrix is then nonsingular.
3. **Solve** `$\sum_i N_{i,p}(t_k)P_i=Q_k$` for the control points. Because of local support, this system is banded.

Step 1 looks like a free choice, and in the mathematical sense it is: any increasing sequence gives a curve through the points. But AGENTS.md asks that a free choice also be a good one, and here is a measured example. A marching trace takes even strides, then one short final leg onto the vertex where it terminates. With uniform parameters the spline must cover the long legs and the short one in equal parameter time, and overshoots. According to the comment on `chord_length_params` in `core/geop-core-geometry/src/nurb_curve/interpolate.rs`, the resulting edge left the true intersection by `7.1e-4`, and the same arc traced from its two ends produced two curves `1.3e-4` apart, which no containment search can recognise as one curve. Chord-length parameters, each point getting the fraction of the polyline's length before it, fixed this:

`$$t_0=0,\qquad t_k=t_{k-1}+\frac{\lvert Q_k-Q_{k-1}\rvert}{\sum_j\lvert Q_j-Q_{j-1}\rvert}.$$`

### An interpolant lies between its points

A curve through samples of a true curve is not that curve. Between samples it drifts, and in a kernel where everything is an enclosure, that drift must be stated, or the kernel will conclude that a point on the true curve is not on its own interpolant.

So there are two functions. `NurbCurve::interpolate` passes exactly through its points. `NurbCurve::interpolate_enclosing` is also given true points between the samples; it fits through the samples' sharp centres (a free choice, and solving through wide samples would amplify their width through the elimination), then measures how far each true point sticks out of the curve's enclosure, and widens every control point by the largest such amount in each coordinate. Since the basis is non-negative and sums to one, this widens the curve by exactly that amount everywhere (`widen_to_enclose`, with the measured pad from `enclosing_pad`).

Note the honest caveat in that comment: the enclosure is exact at the checked points and "as good as that measurement in between". This is a place the kernel measures rather than proves. Where the true points are placed is a matter of care: `true_point_fractions` uses one midpoint for an interior interval, where a cubic interpolant's drift peaks, and seven for the two end intervals, where a clamped end makes the drift lopsided and several times larger.

## Where does this leave us?

A B-spline does everything we asked of a piecewise curve, but it still cannot draw a circle: its pieces are polynomial, and exercise 5 of Lecture 6 still holds. And its derivative test says something about the parameter, not the shape. Lecture 8 fixes the first by adding one more coordinate.

## Recap

<div class="recap">

- A B-spline is a degree, a knot vector and control points. The Cox-de Boor recursion gives its basis functions; there are `$n+p+2$` knots for `$n+1$` points, and the domain is `$[u_p,u_{n+1}]$`.
- Local support and partition of unity give local control and the strong convex hull: a span lies in the hull of its `$p+1$` active points.
- A knot of multiplicity `$m$` leaves `$C^{p-m}$` at that joint. Clamped ends make the end points stored values, which the kernel relies on.
- De Boor's algorithm evaluates by convex combinations. Derivatives are B-splines of scaled differences.
- Over an interval reaching across knots, a curve is the union of its spans' pieces; one span's polynomial continued encloses nothing.
- Fitting needs parameters, knots and a banded solve; chord length is the good free choice; and a fit must widen to enclose what it was fitted to.

</div>

## Exercises

**1.** Show that a B-spline with `$n+1$` control points and degree `$p$` has `$n-p+1$` knot spans in its domain (counting empty ones). How many free numbers per coordinate does a chain of `$s$` cubic pieces joined `$C^2$` have? Check that a clamped cubic B-spline with `$s$` non-empty spans has that many control points.

<details class="solution"><summary>Solution</summary>

The domain runs from `$u_p$` to `$u_{n+1}$`, which is `$n+1-p$` spans. For `$s$` cubic pieces, `$4s$` coefficients less `$3(s-1)$` joint conditions gives `$s+3$`. With `$s=n-3+1$`, the control point count `$n+1=s+3$`. They agree.

</details>

**2.** Show that with `$U=(0,\dots,0,1,\dots,1)$`, `$p+1$` of each, `$N_{i,p}=B_i^p$`. Start with `$p=1$` and induct.

<details class="solution"><summary>Hint</summary>

On `$[0,1)$`, the only non-empty span is `$[u_p,u_{p+1})$`, so `$N_{p,0}=1$`. In the recursion, every nonzero denominator equals 1, and every numerator is `$t$` or `$1-t$`. So the recursion becomes `$N_{i,p}=tN_{i,p-1}+(1-t)N_{i+1,p-1}$`, the Pascal identity of Bernstein polynomials with the index shift you should work out.

</details>

**3.** For the quadratic knots `$U=(0,0,0,1,2,3,3,3)$` compute by hand all `$N_{i,2}(1.5)$` and check that they sum to one. Which control points are active at `$t=1.5$`? Then use the demo with degree 2 to see the same active-point pattern.

<details class="solution"><summary>Solution</summary>

`$t=1.5$` lies in span `$k=3$`, `$[u_3,u_4)=[1,2)$`, so `$P_1,P_2,P_3$` are active. At degree one: `$N_{2,1}=(u_4-t)/(u_4-u_3)=\tfrac12$` and `$N_{3,1}=\tfrac12$`. At degree two: `$N_{1,2}=\frac{u_4-t}{u_4-u_2}N_{2,1}=\frac{0.5}{2}\cdot\tfrac12=\tfrac18$`; `$N_{2,2}=\frac{t-u_2}{u_4-u_2}N_{2,1}+\frac{u_5-t}{u_5-u_3}N_{3,1}=\tfrac34\cdot\tfrac12+\tfrac34\cdot\tfrac12=\tfrac34$`; `$N_{3,2}=\frac{t-u_3}{u_5-u_3}N_{3,1}=\tfrac14\cdot\tfrac12=\tfrac18$`. The sum is 1.

</details>

**4.** Using the derivative formula, show that a clamped curve leaves `$P_0$` with `$C'(u_p)=\frac{p}{u_{p+1}-u_1}(P_1-P_0)$`. Then explain in one sentence why this denominator is a single span's width for a clamped curve, whatever the rest of the knots are.

<details class="solution"><summary>Solution</summary>

At `$t=u_p$` only `$N_{1,p-1}$` is nonzero, with value 1, so `$C'=Q_0$`, with denominator `$u_{p+1}-u_1$`. Clamped means `$u_1=\dots=u_p$`, so the denominator is `$u_{p+1}-u_p$`, the first span's width.

</details>

**5.** Show by example that uniform parameters can overshoot. Take four collinear points at `$x=0,1,2,2.01$` on the `$x$`-axis and fit a single cubic with `$t_k=k/3$`. How far past `$x=2.01$` does the curve go? Then repeat with chord length. Why is this the same defect as the traced curve's last leg?

<details class="solution"><summary>Hint</summary>

For a single cubic, `$x(t)$` is the cubic polynomial through the four data. With uniform `$t_k$`, it is the Lagrange polynomial through `$(0,0),(\tfrac13,1),(\tfrac23,2),(1,2.01)$`, whose maximum on `$[0,1]$` is about 2.19, near `$t=0.845$`. With chord length, `$t_k=x_k/2.01$`, so `$x(t)=2.01t$`, exactly linear. A short last leg in equal parameter time asks the curve to slow down sharply just at the end, and a low-degree polynomial cannot, so it overshoots first.

</details>

**6.** *Reading the code.* Open `find_spans` in `core/geop-core-geometry/src/spline.rs`. Under what two conditions does it return a single span with `$t$` unchanged? Why is the first condition, `t.is_sharp()`, safe even if the sharp `$t$` lies exactly on a knot?

<details class="solution"><summary>Solution</summary>

It returns one span if `$t$` is sharp, or if `$t$` lies wholly within the span found, its ends included. A sharp `$t$` on a knot is a single number, and a B-spline is continuous there unless the multiplicity is full, so both spans' polynomials give the same value: the one span is correct. (At a knot of full multiplicity the curve may have two values, and `find_span`'s rule says which: that of the span starting there.)

</details>
