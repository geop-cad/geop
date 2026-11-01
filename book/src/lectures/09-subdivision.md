# Lecture 9: Knot insertion and subdivision

<div class="goals">

**After this lecture you can**

- derive Boehm's knot insertion formula, and apply it by hand;
- use knot insertion to split a curve, and see de Casteljau as its special case;
- explain why subdivided control polygons converge to the curve, and how fast;
- trace what the width of a split parameter does to the pieces, through `$\alpha=(t-a)/(b-a)$`, and why cutting at a sharp parameter stops it compounding;
- choose between `$a+\alpha(b-a)$` and `$(1-\alpha)a+\alpha b$`, and say why the kernel uses both;
- explain why carrying a curve on past its end must not touch the part it already has.

</div>

Every algorithm in [Part III](./12-subdivide-to-isolate.md) comes down to the same question: can we zoom in on a piece of a curve? We can evaluate a NURBS at a point, but that says nothing about what lies between points. What the searches need is another *representation of the same curve, valid over a piece of it*, whose control points give fresh, tighter hulls. That operation is knot insertion, and this lecture is about what it does when the numbers are enclosures. It is the place in this book where algebra and rounding interact most instructively.

## The problem

Given a B-spline of degree `$p$` with knots `$U$` and control points `$P_i$`, we want to add a knot `$\bar t$` to `$U$` and find new control points `$Q_i$` giving **exactly the same curve**. This must be possible: adding a knot only enlarges the spline space, since it allows the curve to have less smoothness at `$\bar t$` but does not require it.

Since the curve must not change, we need

`$$\sum_i N_{i,p}(t)P_i=\sum_i\bar N_{i,p}(t)Q_i,$$`

where `$\bar N$` are the basis functions on the new knots. There is one control point more than before. By local support, nothing far from `$\bar t$` can change. The answer turns out to be pleasingly simple.

## Boehm's formula

Let `$\bar t\in[u_k,u_{k+1})$`. Then

`$$Q_i=\begin{cases}P_i & i\le k-p,\\[2pt] \alpha_iP_i+(1-\alpha_i)P_{i-1} & k-p<i\le k,\\[2pt] P_{i-1} & i>k,\end{cases}\qquad \alpha_i=\frac{\bar t-u_i}{u_{i+p}-u_i}.$$`

The new knot vector is `$U$` with `$\bar t$` inserted after `$u_k$`. So exactly `$p$` old points are replaced, by points on the legs of the old polygon, and the rest are kept. No system of equations is solved, and because each `$\alpha_i$` is in `$[0,1]$`, every new point is a convex combination of old ones. So **the new polygon is the old polygon with its corners cut**.

The derivation is the Cox-de Boor recursion run the other way: one shows that each old basis function is a combination of two new ones,

`$$N_{i,p}=\frac{\bar t-\bar u_i}{\bar u_{i+p+1}-\bar u_i}\bar N_{i,p}+\frac{\bar u_{i+p+2}-\bar t}{\bar u_{i+p+2}-\bar u_{i+1}}\bar N_{i+1,p},$$`

with the convention that a term whose denominator vanishes is dropped, and this by induction on `$p$`. Substituting into the left-hand sum and collecting the coefficient of each `$\bar N_{i,p}$` gives the formula above (Piegl and Tiller, section 5.2, have it in full; Boehm's paper of 1980 is the original). The **Oslo algorithm** generalizes this to several knots at once; the kernel does not use it, and inserts knots one at a time.

### A worked example

Take the quadratic from [Lecture 7](./07-bsplines.md), `$U=(0,0,0,1,2,2,2)$`, `$P=(0,0),(1,2),(3,2),(4,0)$`, and insert `$\bar t=\tfrac12$`. Here `$k=2$` and `$p=2$`, so `$i=1,2$` change:

`$$\alpha_1=\frac{\frac12-u_1}{u_3-u_1}=\frac12,\qquad \alpha_2=\frac{\frac12-u_2}{u_4-u_2}=\frac14.$$`

`$$Q_1=\tfrac12P_1+\tfrac12P_0=(\tfrac12,1),\quad Q_2=\tfrac14P_2+\tfrac34P_1=(\tfrac32,2).$$`

So `$Q=(0,0),(\tfrac12,1),(\tfrac32,2),(3,2),(4,0)$`, with knots `$(0,0,0,\tfrac12,1,2,2,2)$`. Those two new points are `$d_1^1$` and `$d_2^1$` of the de Boor triangle we computed for the same `$t$`. That is no accident: each knot insertion is one level of de Boor, and if we insert `$\bar t$` `$p$` times in all, we run through the whole triangle, and one control point is `$C(\bar t)$` itself.

<figure class="demo" data-demo="knot-insertion">
<figcaption>Move the slider and press "Insert at t": the orange polygon gets one point more, the dotted one is the polygon before, and the black curve does not move. "Halve every span" inserts a knot in the middle of each span. The shaded boxes hold, for each span, the points active on it: by the strong convex hull property the curve lies in their union. The total area and the distance of the farthest control point from the curve are in the readout (the distance is sampled, for this display only). Drag the control points before refining.</figcaption>
</figure>

## Splitting is inserting to full multiplicity

Insert `$\bar t$` until its multiplicity reaches `$p+1$`. Then, by Lecture 7's table, the curve is allowed to jump at `$\bar t$`, so the control points separate into two groups with nothing connecting them: **the curve has been split into two curves**. Each piece has a clamped knot vector.

For one Bézier span on `$[a,b]$` all those `$\alpha$` have the same value: every level of the triangle uses `$\alpha=(\bar t-a)/(b-a)$`. It is de Casteljau, and the two polygons along the sides of the triangle in [Lecture 6](./06-bezier.md) are the pieces.

<div class="kernel">

**In the kernel.** All of this is in `core/geop-core-geometry/src/knot_insertion.rs`. The module works on *rows*: a curve is one row; a surface split in `$u$` is its `num_v` columns, which share one knot vector, so the alphas are computed once and applied to every row.

- `insert_once` is Boehm's formula in place. Its comment notes that overwriting downward reads every `$P_{i-1}$` and `$P_i$` before replacing them, and the alphas use the knots *before* insertion.
- `split` calls `insert_to_full_multiplicity`, then cuts. `NurbCurve::split` in `core/geop-core-geometry/src/nurb_curve/split.rs` wraps it.
- `split_bezier_span` is the fast path for a single Bézier span: "the same geometry with one division instead of Boehm's ~p(p+1)/2 — and without Boehm's later-level alphas like `(t - t) / (b - t)`, which interval arithmetic can only widen." In exact arithmetic that alpha is 0. In interval arithmetic `$t-t$` for a wide `$t$` is an interval around zero. It is the dependency problem of [Lecture 4](./04-intervals.md) again, so the kernel takes care to *never form the expression*.
- `restrict` cuts both ends in one pass, for `sub_curve`.

In `insert_once`, look at how the extreme alphas are handled: a knot that is `$\bar t$` itself gives an alpha of exactly 0 or 1, and then "the point is one of the two, exactly: interpolating would round it outwards". Every operation avoided here is an ulp of width saved, and [Lecture 26](./26-sweeps.md) will show why the last ulp matters.

</div>

## Convergence

Why does subdividing help? Two facts.

First, with each insertion the polygon's corners are cut, so the polygon moves toward the curve. Second, the rate is quadratic. For a curve with parameter span `$h$`, the distance between the curve and the chord of a piece is bounded by

`$$\max_{t}\lvert C(t)-L(t)\rvert\le\frac18h^2\max\lvert C''\rvert,$$`

the classical bound for linear interpolation. Applied to a piece's control polygon, it shows the distance shrinking by about a factor of four when `$h$` halves. Press "Halve every span" a few times and read the figures in the demo. With the starting polygon, we measured the farthest control point at `$1.10$`, `$0.36$`, `$0.11$`, `$0.028$`, `$0.007$` over the rounds. The summed area of the boxes falls more slowly, `$28.6$`, `$22.7$`, `$15.9$`, `$10.0$`, `$5.7$`, tending to a halving per round: there are twice as many boxes each round, each a quarter of the area.

A hull of `$p+1$` points around a nearly straight piece is therefore a thin sliver. An axis-aligned box of those points is not thin if the curve runs diagonally: it is `$h\times h$`. That is why a box test alone makes pairs of pieces multiply near a crossing, and why the kernel measures hulls across a direction when it can. See [Lecture 13](./13-fat-lines.md) and the branch and bound in AGENTS.md's "Do not decide from samples": "Measured across the direction between two pieces, the hull bound is off by the square of the pieces' size... A box bound is off by the size itself, and with it the survivors multiply."

## Now the numbers are intervals

So far all this is classical. Here is the part specific to a kernel of enclosures. Let `$t$` be an interval (because some search produced it), and let the knots `$a,b$` be intervals (because they came from earlier cuts). Then

`$$\alpha=\frac{t-a}{b-a}$$`

is an interval, and its width has two sources:

`$$\text{width}(\alpha)\approx\frac{\text{width}(t)+\text{width}(a)}{\lvert b-a\rvert}+\lvert\alpha\rvert\,\frac{\text{width}(a)+\text{width}(b)}{\lvert b-a\rvert}.$$`

Every input width is **divided by the span's size**. Then in a control point `$(1-\alpha)P+\alpha P'$`, it comes out multiplied by `$\lvert P'-P\rvert$`, which is the speed of the curve times the span's size. Those cancel, and the control point widens by roughly the width of `$t$` times the speed of the curve: a *width in position*, which is at least what you asked for by giving a wide `$t$`.

So one split at a wide `$t$` widens control points by `$\lvert C'\rvert\,\text{width}(t)$`. No surprise. The trouble comes when the width returns. The next parameter is found *on the widened piece*, so its enclosure is at least the piece's width divided by the speed, and then the new knot `$a=t$` is wide as well and enters `$\alpha$` twice. Each cut multiplies the width by a constant bigger than one, giving exponential growth in the number of cuts.

This is not a thought experiment. AGENTS.md records it under "Many booleans on one body", in the section *Width along a split edge*:

> An edge was cut at the whole refined enclosure of where it reaches the vertex. That kept every piece it could be cut into there in one: the parameter's width went through Boehm's `α = (t - a) / (b - a)` into the pieces' control points and end knot, the next cut's parameter, found on the wider piece, came out wider, and the width grew threefold per cut. A quarter arc cut forty times went from `3e-15` to `0.08`; a disc's rim cut by thirty-two slots ended `4e-10` below its cap, and gears stopped at 80 teeth, their rims drawn in twelve arcs to spread the cuts.

Before the fix, then, a width of `0.08` on an arc that started at `3e-15`, with nothing wrong geometrically. Run it yourself:

<figure class="demo" data-demo="knot-insertion" data-args='{"mode":"width"}'>
<figcaption>A real computation in outward-rounded interval arithmetic, in your browser: a quarter arc of radius 4 is cut at evenly spaced points, keeping the far piece each time, as a rim is cut by slots. Each parameter comes from Newton on the piece, with the last step unsharpened. In red the piece is cut at that whole enclosure; in blue at its sharp middle. The plot is the widest coordinate of any control point. Simplified from the kernel: one Bézier span, its own Newton, so its numbers are of the same kind but not the kernel's. When the width grows so far that the denominator could be zero, the red line stops: the computation has broken down.</figcaption>
</figure>

With the default settings you can read the growth off the plot. In this simplified setting we measured a factor between 3 and 4 per cut, reaching `0.05` by the 25th cut, and two cuts later the division fails. The blue line stays below `1e-13` all the way to forty cuts. So the demo shows the same phenomenon as the AGENTS.md record, and it reproduces the *rate*, though not the same numbers. A real test in the kernel, `a_rim_cut_many_times_stays_as_narrow_as_cut_once` in `core/geop-core-geometry/src/nurb_curve/split.rs`, cuts an arc forty times and asserts that the widest control point stays under `1e-13`.

## Where to cut is a free choice

The fix is the rule from [Lecture 5](./05-honest-numerics.md), and this is its canonical example. The vertex is an enclosure, and every parameter of the refined enclosure of where the curve reaches it ends a piece on the curve and within the vertex. Where to cut within it is therefore a **free choice**: any parameter there would serve. So choose one, sharply, and check that it does the job. The vertex keeps its width. The pieces carry only the rounding of the cut.

`NurbCurve::split_parameter_at_point` in `core/geop-core-geometry/src/nurb_curve/refine.rs` is that function: it cuts at the sharp middle of the refined enclosure and checks the curve reaches the point there. Compare what `NurbCurve::split` says about itself:

> `t` is used exactly as given — it is **not** sharpened here. Whether `t`'s width may be discarded belongs to whoever produced it, since only they know what it means, so this function neither assumes nor imposes an answer.

The abstraction is careful. `split` cannot know. A subdivision search may split at a sharp midpoint. Code cutting an edge at a vertex goes through `split_parameter_at_point`. A caller that wanted the width to stay could give a wide `$t$`.

And the rule's limit: AGENTS.md's section "Validate the value you are about to use, not a wider one" tells the story of checking `curve.evaluate(edge_t).could_be_equal(&vertex_point)` on the *wide* parameter, then splitting at the sharpened one. The check answers "is the vertex somewhere along this stretch?", while the split asks "cut exactly here". Those are different questions. So when you sharpen, re-validate the sharp value: `split_parameter_at_point` does.

### Pinning does not help

An idea you will surely have: after splitting, set the piece's end control point to the vertex exactly. AGENTS.md records this as tried: "Pinning the split curve's endpoint onto the vertex was tried and looked like it worked for the 3-D curve; applied to the pcurve — whose `(u, v)` came from a `min_subdivision_size`-bounded containment search — it pinned a control point to a box so wide the pcurve had no usable tangent left ("Cannot normalize zero"), and took the sweep from 0/175 failing scenes to 12/175." Fix the parameter, not the geometry.

## Which formula for a lerp?

Look closer at the heart of it all: a point between two enclosures. In exact arithmetic these are the same:

`$$a+\alpha(b-a)\qquad\text{and}\qquad(1-\alpha)a+\alpha b.$$`

In interval arithmetic they are not, and neither is always better. Let us compute with `$a=b=[1,1]$`, as with a weight that should stay 1 through many subdivisions, and `$\alpha=[0.3,0.4]$`:

- `$a+\alpha(b-a)$`: `$b-a=[0,0]$` (up to outward rounding), and so the result is `$[1,1]$` (up to the same). Tight.
- `$(1-\alpha)a+\alpha b$`: `$[0.6,0.7]+[0.3,0.4]=[0.9,1.1]$`, of width `$0.2$`. The two alphas are treated as independent.

So for equal values the first wins clearly. Now take wide `$a$` and `$b$`: `$a=[0,1]$`, `$b=[0.1,1.1]$`, with a sharp `$\alpha=\tfrac12$`:

- `$a+\alpha(b-a)$`: `$[0,1]+\tfrac12[-0.9,1.1]=[-0.45,1.55]$`, width 2. Since `$a$` is mentioned twice, its width counts twice.
- `$(1-\alpha)a+\alpha b$`: `$\tfrac12[0,1]+\tfrac12[0.1,1.1]=[0.05,1.05]$`, width 1. Tight.

The first case is what a repeatedly split curve's weights need; the second is what a patch with truly wide control points needs. Since both are valid enclosures of the same number, **intersect them**. This is `Scalar::interpolate` in `core/geop-core-math/src/scalars/mod.rs`:

```rust
if a.is_exactly(b) && alpha.lo >= 0.0 && alpha.hi <= 1.0 {
    return a;
}
let via_delta = a.add(alpha.mul(b.sub(a)));
let via_weights = Self::ONE.sub(alpha).mul(a).add(alpha.mul(b));
via_delta.intersect(via_weights)
```

The opening line is an extra observation: every point between two equal enclosures lies in that enclosure, whatever the two values stand for. Without it, `$b-a$` rounds outward even where it is exactly zero, and `$a$` plus that is `$a$` widened by an ulp. The comment notes the benefit: "A polynomial patch cut in pieces keeps its weights exactly one, and stays a polynomial." That has a practical consequence: `core/geop-core-topology/src/mass.rs` can then integrate with exact rules ([Lecture 41](./41-performance.md)).

The story of how the kernel got here is in AGENTS.md's "Trace numerical bugs to their root cause": a blowup in `curve_curve_intersect` was first "fixed" by trimming curves near a shared vertex, which only moved away from the region where the bug lived. Tracing down one level at a time found the real chain. Repeated `NurbCurve::split` calls compounded width in a control point's weight (root cause one, fixed by the tighter formula above), and in knot values divided by an ever-shrinking span (root cause two, fixed by sharpening a free-choice value right after the operation that introduced the rounding noise). Use the second control of the demo to compare: with only one of the formulas, how do the two curves change?

## Carrying a curve on

There is one more operation in this family, and it has a lesson. Booleans and shells often want a curve or surface *longer* than it is: a face that moves needs room on its neighbours' surfaces. Mathematically the polynomial just continues. De Casteljau at `$s>1$` extrapolates as exactly as it interpolates, in that the left polygon of a "split" at `$s$` is the control polygon over `$[0,s]$`.

Note that this isn't a convex combination any more. `$\alpha>1$` amplifies, and a rational curve's weights can fall.

The first implementation replaced the end piece by the piece over the longer span, recomputing all its control points by extrapolation. AGENTS.md's "Carrying a patch on must leave it as it was" gives the result: "a sphere's octant carried on evaluated its old region four orders of magnitude wider, and a shell's sphere vertex went from `1e-15` to `7e-10` wide."

The fix "was not to give curved surfaces less room but to keep the curve as it was, control point for control point, and join the continuation on as a piece of its own at a knot of full multiplicity. Room is then free: where the face stays, nothing changed." `NurbCurve::extended` in `core/geop-core-geometry/src/nurb_curve/extend.rs` does that: it isolates the end piece by knot insertion, finds the continuation by `continued` and `beyond`, then appends the continuation's points to the *untouched* original ones, with the shared end point the `intersect` of its two enclosures. It fails on a knot vector not clamped at that end, and where the continued weights stop being positive.

The general rule: **any operation that only needs more of a curve should not touch the part it already had.**

## Recap

<div class="recap">

- Boehm's formula adds a knot and changes `$p$` control points, by convex combinations, with no change to the curve. `$p+1$` copies of a knot split the curve; on one Bézier span, this is de Casteljau.
- Control polygons converge to the curve quadratically in the span width, which is what makes hull-based searches work.
- A wide split parameter widens the pieces through `$\alpha=(t-a)/(b-a)$`. When the next parameter is found on the wider piece, width compounds exponentially: in the kernel it was a factor of three per cut.
- Where to cut within a refined enclosure is a free choice: cut at a sharp parameter, and re-check it.
- `$a+\alpha(b-a)$` and `$(1-\alpha)a+\alpha b$` are both enclosures with opposite strengths; `Scalar::interpolate` intersects them.
- Carry a curve on by adding a piece, never by recomputing the piece it has.

</div>

## Exercises

**1.** Insert `$\bar t=1$` into the worked example's knots `$U=(0,0,0,1,2,2,2)$`, and write the new control points. What is the multiplicity of the knot now, and what does the curve do at `$t=1$` relative to the new control polygon?

<details class="solution"><summary>Solution</summary>

`$\bar t=1$` falls in `$[u_3,u_4)$`, `$k=3$`. Then `$i=2,3$` change: `$\alpha_2=\frac{1-u_2}{u_4-u_2}=\tfrac12$` and `$\alpha_3=\frac{1-u_3}{u_5-u_3}=0$`. So `$Q_2=\tfrac12P_2+\tfrac12P_1=(2,2)$`, and `$Q_3=P_2=(3,2)$`. With `$Q_0=P_0$`, `$Q_1=P_1$` unchanged, and `$Q_4=P_3$`. The knot `$1$` has multiplicity 2, equal to the degree, so the curve passes through a control point there: `$C(1)=Q_2=(2,2)$`.

</details>

**2.** Prove that the number of changed control points in one insertion is `$p$`, and that the `$\alpha_i$` lie in `$[0,1]$`. What happens to the formula when `$\bar t$` is already a knot of multiplicity `$s$`? How many points really change?

<details class="solution"><summary>Hint</summary>

The range `$k-p<i\le k$` has `$p$` values. For them `$u_i\le u_k\le\bar t$` and `$\bar t<u_{k+1}\le u_{i+p}$`, so the fraction is in `$[0,1)$`. If `$\bar t=u_k=\dots=u_{k-s+1}$`, then `$\alpha_i=0$` for `$i$` from `$k-s+1$` to `$k$`: those `$Q_i$` equal `$P_{i-1}$`, so only `$p-s$` points are new.

</details>

**3.** In the width demo, take `$\alpha$` with the two single formulas, and observe the two lines. Which formula is worse for the wide cuts, and which for the sharp ones? Give a reason using the two examples in the lecture.

<details class="solution"><summary>Hint</summary>

We measured for a quarter arc, forty cuts: with sharp cuts, `a + α(b−a)` alone climbs to about `2e-11`, while the other stays near `4e-14`; with wide cuts, `(1−α)a + αb` alone breaks down earliest, at the 17th cut. Sharp `$\alpha$` with wide-ish control points favours the second formula, as `$a$` is not counted twice. Wide `$\alpha$` favours the first, since `$b-a$` is small and it does not split `$\alpha$` into two independent factors. Intersected, you get the best of both.

</details>

**4.** Suppose a search returns `$t$` of width `$10^{-10}$` on a curve with `$\lvert C'\rvert\approx1$`. The width of the resulting split point in space is what? Compute `$\text{width}(\alpha)$` for spans of `$10^{-2}$` and of `$10^{-8}$`. Why, at a span of `$10^{-8}$`, does it hardly matter any more what the control points were?

<details class="solution"><summary>Solution</summary>

The width in space is about `$10^{-10}$`. `$\text{width}(\alpha)\approx10^{-10}/h$`: `$10^{-8}$` for `$h=10^{-2}$`, and `$10^{-2}$` for `$h=10^{-8}$`. In the second case one-hundredth of the distance between adjacent control points is uncertain, but those are `$\approx10^{-8}$` apart, so the result is the same `$10^{-10}$` in space. The division by a small span and the multiplication by a small leg cancel. The real risk comes when the next span's own ends are that wide: the denominator `$b-a$` could then contain zero, and the division fails.

</details>

**5.** Why does `split_bezier_span` make all levels use one `$\alpha$`, while Boehm's insertion would compute `$(t-t)/(b-t)$` at later levels? Compute `$[t-\varepsilon,t+\varepsilon]-[t-\varepsilon,t+\varepsilon]$`.

<details class="solution"><summary>Solution</summary>

After the first insertion, `$t$` is a knot, and the second insertion computes `$\alpha_i$` with `$u_i=t$` for some `$i$`. In exact arithmetic that alpha is 0. In interval arithmetic the numerator is `$[-2\varepsilon,2\varepsilon]$`, a nonzero width. So `$\alpha$` is wide around 0 where it should be exactly 0. (Note `insert_once`'s test `knots[i] == t_bar` catches the identical enclosure in Boehm's path too.) One `$\alpha$` for all levels forms none of these.

</details>

**6.** Show that the left polygon of de Casteljau at `$s>1$` is the control polygon of the same polynomial over `$[0,s]$`. Compute the weights of that polygon for the unit quarter arc at `$s=2$`. For which `$s$` does its middle weight reach zero? Then compute the weights of the polygon over `$[1,s]$` only, which `beyond` in `core/geop-core-geometry/src/nurb_curve/extend.rs` keeps. What does that tell you about adding a piece rather than recomputing?

<details class="solution"><summary>Solution</summary>

The proof is the same as for `$s<1$`: it is polynomial algebra. For the arc, the weights are `$w=(1,h,1)$` with `$h=\frac{\sqrt2}2$`. Level one at `$s=2$`: `$1+2(h-1)=2h-1\approx0.414$`, and `$h+2(1-h)=2-h\approx1.293$`. Level two: `$0.414+2(1.293-0.414)\approx2.172$`. So over `$[0,2]$` the weights are `$1,\ 0.414,\ 2.172$`. The middle one, `$1+s(h-1)$`, is zero at `$s=1/(1-h)\approx3.41$`, and negative past that, although `$W(s)=1-2(1-h)s+2(1-h)s^2$` has no real root.

Over `$[1,s]$` the weights are the blossom values `$W(1,1)=1$`, `$W(1,s)=h+s(1-h)$` and `$W(s,s)=W(s)$`, all positive for every `$s>1$`. So for this arc the piece added stays valid where a recomputed whole polygon would not. (`extended`'s doc comment says it fails where continued weights stop being positive. That can happen for other weights: with `$w_1>w_2$`, `$W(1,s)=w_1(1-s)+w_2s$` reaches zero at a finite `$s$`.)

</details>
