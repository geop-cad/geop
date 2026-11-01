# Lecture 6: Bézier curves

<div class="goals">

**After this lecture you can**

- define a Bézier curve by de Casteljau's construction and by Bernstein polynomials, and prove that they agree;
- state and use the properties a kernel relies on: end-point interpolation, the convex hull, affine invariance, variation diminishing;
- compute derivatives from control-point differences, and split a curve in two;
- say which of these properties Geop's searches actually depend on, and why it does not store "Bézier curves" as a kind of its own.

</div>

Everything so far was about numbers. From here on we are concerned with shape. A CAD kernel has to represent a free-form curve exactly, change it cheaply, and above all *answer questions about it with proof*: does this curve cross that one? Where? Could it reach this point? The whole of the next ten lectures hangs off one remarkable representation that makes those questions tractable, and it starts with nothing more than repeated linear interpolation.

## From a line to a curve

Take two points `$P_0, P_1$` in the plane. The line segment between them is the set of all

`$$(1-t)\,P_0 + t\,P_1,\qquad t\in[0,1].$$`

We call this *linear interpolation*, and the only thing it asks of a space is that you can add vectors and scale them. Now take three points. Interpolating on each leg of the polygon gives two points moving along the two legs, and interpolating between *those* with the same `$t$` gives a third:

`$$\begin{aligned}
P_0^1(t) &= (1-t)P_0 + tP_1,\\
P_1^1(t) &= (1-t)P_1 + tP_2,\\
P_0^2(t) &= (1-t)P_0^1(t) + tP_1^1(t).
\end{aligned}$$`

As `$t$` runs from 0 to 1, `$P_0^2(t)$` traces a parabola. Nothing in the construction is special to three points. Start from `$n+1$` *control points* and repeat, each level one point shorter than the one before:

`$$P_i^0 = P_i,\qquad P_i^{k}(t) = (1-t)\,P_i^{k-1}(t) + t\,P_{i+1}^{k-1}(t),\quad k=1,\dots,n,\ i=0,\dots,n-k.$$`

The last point `$C(t)=P_0^n(t)$` is the *Bézier curve* of *degree* `$n$`, and this procedure is **de Casteljau's algorithm**. Play with it before reading on.

<figure class="demo" data-demo="de-casteljau">
<figcaption>Drag the control points, or the slider for <em>t</em>. Each coloured polygon is one level of de Casteljau's construction; the red point is on the curve. "Add point" raises the degree without changing the curve (see the exercises). "Split at t" draws the two pieces' control polygons.</figcaption>
</figure>

Things to notice while you drag:

1. The curve starts at `$P_0$` and ends at `$P_n$`, but does not in general pass through the others.
2. The curve never leaves the shaded hull of the control points.
3. At `$t=0$` the curve leaves along the first leg `$P_0P_1$`, and it arrives along the last leg.
4. Moving one control point moves the whole curve, though least where `$t$` is far from `$i/n$`.

Each of these can be proved, and each of them will be used.

## Bernstein polynomials

Unroll the recursion by hand for `$n=2$`:

`$$C(t) = (1-t)^2 P_0 + 2t(1-t)P_1 + t^2 P_2.$$`

The coefficients are the terms of `$\bigl((1-t)+t\bigr)^2$`. This is general: a path through the triangle of the construction from `$P_i$` to the top takes `$n$` steps, `$i$` of them to the right (each with a factor `$t$`) and `$n-i$` to the left (each with a factor `$1-t$`), and there are `$\binom{n}{i}$` such paths. So

`$$C(t) = \sum_{i=0}^{n} B_i^n(t)\,P_i,\qquad B_i^n(t)=\binom{n}{i}t^i(1-t)^{n-i}.$$`

The `$B_i^n$` are the **Bernstein polynomials** of degree `$n$`. The demo prints both evaluations under the plot: the two numbers agree, as `book/scripts/demo_tests/de-casteljau.test.mjs` also checks. Three facts about them carry the rest of this lecture.

**They are a partition of unity.** By the binomial theorem,
`$$\sum_i B_i^n(t) = \bigl((1-t)+t\bigr)^n = 1.$$`

**They are non-negative on the interval.** For `$t\in[0,1]$`, every factor of `$B_i^n(t)$` is at least zero.

**They are a basis.** The `$n+1$` polynomials `$B_0^n,\dots,B_n^n$` span the polynomials of degree at most `$n$`, so every polynomial curve of degree `$n$` has exactly one set of Bézier control points. A Bézier curve is therefore not a new kind of curve, only a *polynomial written in a clever basis*.

Why is that basis clever? Because its coefficients are points that have geometric meaning. In the monomial basis `$a_0+a_1t+a_2t^2+\dots$` the coefficient `$a_3$` is not a location anywhere, and a small change in it can throw the curve far away. Worse, the monomial form is badly conditioned for evaluation and root finding on `$[0,1]$`; Farouki and Goodman proved that the Bernstein basis is the best conditioned among polynomial bases that are non-negative on the interval. A kernel living on rounding-error margins cares.

## What the representation gives you

**Convex hull.** Since the Bernstein values are non-negative and sum to one, `$C(t)$` is a convex combination of the control points, for every `$t\in[0,1]$`. So the curve lies in the convex hull of its control points.

This single property is the foundation of every intersection algorithm in this book. If the hulls of two curves are disjoint, then the curves do not meet, and that is a *proof*, not an estimate: it holds for all infinitely many points of the curve, using finitely many numbers. Compare it with sampling, which proves nothing about what lies between two samples (we return to that in [Lecture 12](./12-subdivide-to-isolate.md)).

**Affine invariance.** The weights sum to one, so `$A\bigl(\sum B_iP_i\bigr)=\sum B_iA(P_i)$` for an affine map `$A(x)=Mx+b$`. Translating, rotating or scaling a curve means transforming its control points, not sampling and refitting. Note the weights must sum to one for the translation part to survive: this will be what the "homogeneous coordinates" of [Lecture 8](./08-nurbs.md) are about.

**End points and end tangents.** `$B_0^n(0)=1$` and all others vanish, so `$C(0)=P_0$`; likewise `$C(1)=P_n$`.

**Derivative.** Differentiating the Bernstein form and collecting terms gives
`$$C'(t) = n\sum_{i=0}^{n-1} B_i^{n-1}(t)\,(P_{i+1}-P_i).$$`
The derivative of a Bézier curve is a Bézier curve of one degree less, the *hodograph*, whose control points are the differences of the original ones. At `$t=0$` it is `$n(P_1-P_0)$`, which is observation 3 above. The convex hull applies to the hodograph too: if all differences `$P_{i+1}-P_i$` lie in some open half plane, then no tangent of the curve can vanish, or turn back through that direction. In one line of arithmetic we have bounded every tangent of the curve.

**Variation diminishing.** A straight line crosses the curve at most as many times as it crosses the control polygon. We state this without proof (see Farin in [Appendix A](../appendix/references.md)). It means that a curve whose control polygon has no wiggle cannot have a wiggle either.

## Splitting

Press *split at t* in the demo. The two polygons drawn along the outside of de Casteljau's triangle are themselves control polygons, and of the two pieces of the curve exactly:

`$$\text{left: } P_0^0, P_0^1, \dots, P_0^n,\qquad \text{right: } P_0^n, P_1^{n-1}, \dots, P_n^0.$$`

The right-hand piece, reparametrized to `$[0,1]$`, is the same point set as the original curve restricted to `$[c,1]$`. Why? Because a polynomial is determined by its values on any interval, and one can check by induction that the left polygon's Bézier curve evaluates to `$C(ct)$`.

Splitting is what makes subdivision work. It costs `$O(n^2)$` additions, it is numerically benign (every step is a convex combination, so errors are not amplified), and each piece's control polygon is closer to the curve than the original's. Indeed, for pieces of parameter width `$h$`, the distance between a piece and its control polygon's chord shrinks as `$h^2$`. So repeated splitting converges quadratically to line segments, each boxed in by its hull. Everything in Part III is built from this.

### A worked example

Let `$P_0=(0,0)$`, `$P_1=(1,2)$`, `$P_2=(3,2)$`, `$P_3=(4,0)$` and `$t=\tfrac12$`.

| level | points |
|---|---|
| 0 | `$(0,0),\ (1,2),\ (3,2),\ (4,0)$` |
| 1 | `$(\tfrac12,1),\ (2,2),\ (\tfrac72,1)$` |
| 2 | `$(\tfrac54,\tfrac32),\ (\tfrac{11}{4},\tfrac32)$` |
| 3 | `$(2,\tfrac32)$` |

So `$C(\tfrac12)=(2,\tfrac32)$`, and by the Bernstein sum `$\tfrac18(0,0)+\tfrac38(1,2)+\tfrac38(3,2)+\tfrac18(4,0)=(2,\tfrac32)$`. The left piece has control points `$(0,0),(\tfrac12,1),(\tfrac54,\tfrac32),(2,\tfrac32)$`, the right piece `$(2,\tfrac32),(\tfrac{11}{4},\tfrac32),(\tfrac72,1),(4,0)$`. The tangent at `$t=\tfrac12$` is `$3\bigl((\tfrac{11}{4},\tfrac32)-(\tfrac54,\tfrac32)\bigr)=(\tfrac92,0)$`, horizontal, as the symmetry demands.

## Where one polynomial is not enough

Bézier curves have two limitations that drive the next two lectures. First, moving any control point moves the *whole* curve: there is no local control. A long curve needs a high degree, and at high degree a control point's influence becomes weak and global. The fix is to chain many low-degree pieces with an agreed continuity at the joints, which leads to B-splines ([Lecture 7](./07-bsplines.md)). Second, a polynomial cannot draw a circle: `$x(t)^2+y(t)^2=1$` has no non-constant polynomial solution (exercise 5). The fix is a rational weight, which leads to NURBS ([Lecture 8](./08-nurbs.md)).

<div class="kernel">

**In the kernel.** There is no `BezierCurve` type in Geop. A curve is a `NurbCurve` (`core/geop-core-geometry/src/nurb_curve/mod.rs`), and a Bézier curve is just one whose knot vector is that of a single span, made by `bezier_knots` (`core/geop-core-geometry/src/nurb_curve/construct.rs`). This is the rule of [Everything is a NURBS](../lectures/01-what-a-kernel-is.md): no algorithm branches on what kind of curve it was given.

The Bernstein form does appear, by name, where polynomial arithmetic is needed with no evaluation and no sampling: `core/geop-core-geometry/src/bernstein.rs` holds polynomials over a rectangle by their Bernstein coefficients, multiplied by the identity `$B_i^mB_k^n = \binom{m}{i}\binom{n}{k}\big/\binom{m+n}{i+k}\,B_{i+k}^{m+n}$` (exercise 4). `core/geop-core-geometry/src/deform.rs` uses this to compose a patch with a polynomial map of space.

The convex hull appears in its cheapest form as an axis-aligned box of the control points, which every curve and surface carries: `compute_aabb` in `core/geop-core-geometry/src/aabb.rs`, read through `NurbCurve::aabb`. Note the rule from AGENTS.md's *A cache derived from a value is derived one way*: this box is always computed by the one function, never patched along by a shortcut.

</div>

## Recap

<div class="recap">

- A Bézier curve is repeated linear interpolation, or equivalently a polynomial written in the Bernstein basis.
- Non-negativity and partition of unity give the convex hull property: a finite proof about infinitely many points.
- Derivatives are curves of differences of control points; splitting is reading the sides of the de Casteljau triangle.
- A single polynomial lacks local control and cannot draw a circle, which is why the kernel stores NURBS.

</div>

## Exercises

**1.** Compute `$C(\tfrac14)$` for the worked example by de Casteljau, and verify with the Bernstein sum.

<details class="solution"><summary>Solution</summary>

Level 1: `$(\tfrac14,\tfrac12),(\tfrac32,2),(\tfrac{13}{4},\tfrac32)$`. Level 2: `$(\tfrac{9}{16},\tfrac78),(\tfrac{31}{16},\tfrac{15}{8})$`. Level 3: `$(\tfrac{29}{32}, \tfrac98)$`, the first coordinate from `$\tfrac34\cdot\tfrac{9}{16}+\tfrac14\cdot\tfrac{31}{16}=\tfrac{58}{64}$`. The Bernstein weights at `$\tfrac14$` are `$\tfrac{27}{64},\tfrac{27}{64},\tfrac{9}{64},\tfrac{1}{64}$`, so `$x=\tfrac{27}{64}\cdot 1+\tfrac{9}{64}\cdot 3+\tfrac{1}{64}\cdot 4=\tfrac{58}{64}$` and `$y=\tfrac{27}{64}\cdot2+\tfrac{9}{64}\cdot2=\tfrac{72}{64}=\tfrac98$`.

</details>

**2.** Prove the derivative formula from the Bernstein form. Start from `$\frac{d}{dt}B_i^n=n(B_{i-1}^{n-1}-B_i^{n-1})$`.

<details class="solution"><summary>Hint</summary>

Differentiate `$\binom{n}{i}t^i(1-t)^{n-i}$` by the product rule to get the identity, then sum over `$i$` and shift the index in one of the sums.

</details>

**3.** *Degree elevation.* Show that the same curve has degree `$n+1$` control points `$Q_0=P_0$`, `$Q_{n+1}=P_n$`, and `$Q_i=\frac{i}{n+1}P_{i-1}+\bigl(1-\frac{i}{n+1}\bigr)P_i$`. Check it with the demo's "Add point" button, which uses this formula: the curve does not move.

<details class="solution"><summary>Hint</summary>

Multiply `$C(t)$` by `$1=(1-t)+t$`, and rewrite `$(1-t)B_i^n$` and `$tB_i^n$` as multiples of `$B_i^{n+1}$` and `$B_{i+1}^{n+1}$`. Collect the coefficient of each `$B_i^{n+1}$`.

</details>

**4.** Prove `$B_i^mB_k^n=\binom{m}{i}\binom{n}{k}\big/\binom{m+n}{i+k}\,B_{i+k}^{m+n}$`. Use it to write down the control points of the product of two scalar Bézier polynomials. This is what `Bernstein` in `core/geop-core-geometry/src/bernstein.rs` implements. Why can it do this with no evaluation and no sampling?

<details class="solution"><summary>Solution</summary>

Multiply: `$\binom{m}{i}\binom{n}{k}t^{i+k}(1-t)^{m+n-i-k}$`, which is `$\binom{m}{i}\binom{n}{k}$` times `$B_{i+k}^{m+n}/\binom{m+n}{i+k}$`. For polynomials `$f=\sum a_iB_i^m$` and `$g=\sum b_kB_k^n$`, the coefficient of `$B_j^{m+n}$` in `$fg$` is `$\sum_{i+k=j}a_ib_k\binom{m}{i}\binom{n}{k}/\binom{m+n}{j}$`. It is an identity between the coefficients, so an exact, finite computation answers it, with interval rounding on each operation only.

</details>

**5.** Show that no polynomial curve traces a piece of the unit circle: there are no non-constant polynomials `$x,y$` with `$x^2+y^2=1$`.

<details class="solution"><summary>Solution</summary>

Let `$d$` be the larger degree of `$x,y$`, with `$d\ge1$`. Then the coefficient of `$t^{2d}$` in `$x^2+y^2$` is `$a^2+b^2$` with `$a,b$` the leading coefficients (one possibly zero, not both). This is positive, so `$x^2+y^2$` has degree `$2d>0$` and cannot be the constant 1. (Over the complex numbers this argument fails, which is the seed of the rational parametrization in Lecture 8.)

</details>

**6.** Suppose a cubic's control points are `$(0,0),(1,1),(2,-1),(3,0)$`. Without computing the curve, bound its `$y$` and its slope `$dy/dx$` for all `$t$`. How wide are those bounds, and how would you tighten them?

<details class="solution"><summary>Solution</summary>

By the hull, `$-1\le y\le1$`. The hodograph's control points, up to the factor 3, are `$(1,1),(1,-2),(1,1)$`: `$x'$` is the constant 3, and `$y'/3$` lies between `$-2$` and `$1$`, so the slope lies in `$[-2,1]$`. Splitting the curve at `$t=\tfrac12$` and redoing this on each half tightens both, quadratically in the piece's width. That is precisely the argument the intersection searches run on.

</details>
