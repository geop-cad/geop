# Lecture 13: Fat-line clipping

<div class="goals">

**After this lecture you can**

- derive Sederberg and Nishita's Bézier clipping: the fat line, the distance function, its control polygon and the clip interval;
- explain why clipping converges quadratically at a transversal crossing while bisection gains one bit per level;
- say when clipping stalls, and what the search does then;
- derive the kernel's form of the same idea: the division-free spline `$g_k(t)$`, its Greville abscissae, the chord band of `fat_line_zeros`, and the tensor collapse of `clip_tensor`;
- explain why the kernel unions over one index and intersects over another, and why swapping the two would be wrong.

</div>

The search of [Lecture 12](./12-subdivide-to-isolate.md) has one weakness: it asks yes-or-no questions. A piece either survives or it does not, and a piece that survives gets cut exactly at its middle, wherever the solution is. Each cut gains one bit, no matter how much the control points know about where the solution lies.

They know a lot. Think of a curve crossing a nearly straight line. The control points say almost exactly where along the curve the crossing is. Throwing that away and halving is a waste. In 1990, Sederberg and Nishita showed how to read it off: the *Bézier clipping* method ([Appendix A](../appendix/references.md)). This lecture derives it, and then the form Geop uses, which is the same idea in clothes better suited to intervals.

## The fat line

Let `$Q(s)$` be a Bézier curve with control points `$Q_0,\dots,Q_m$`. If the curve were straight, we would know its line. In general it is not, but the line `$L$` through its end points `$Q_0$` and `$Q_m$` is a fair stand-in. Write `$L$` in normal form, with unit normal `$n$`:

`$$\mathrm{dist}(x) = n\cdot x + c,$$`

the signed distance of a point `$x$` from `$L$`. The control points are at distances `$\mathrm{dist}(Q_j)$`, and `$\mathrm{dist}$` is affine, so the curve's distances are a convex combination of the control points' distances. Therefore the whole curve lies in the band

`$$d_{\min}\le \mathrm{dist}(Q(s))\le d_{\max},\qquad d_{\min}=\min_j \mathrm{dist}(Q_j),\ d_{\max}=\max_j \mathrm{dist}(Q_j).$$`

This band, a line with a thickness, is the **fat line** of `$Q$`. It is a better bound than a box. A box of an oblique curve is thick in both directions, whereas a fat line is aligned with the curve, and its thickness is of the order of the curve's sag `$\tfrac18 \kappa h^2$`. It is second order in the piece's length `$h$`, where a box's thickness is first order. All of this lecture follows from that.

## The distance function is a Bézier function

Now let `$P(t)$` be the other curve, with control points `$P_0,\dots,P_n$`. Measure its distance from `$L$`:

`$$d(t) = n\cdot P(t) + c = \sum_{i=0}^n B_i^n(t)\,d_i,\qquad d_i = n\cdot P_i + c.$$`

We only used that the Bernstein polynomials sum to one. The distance along `$P$` is a scalar polynomial in Bernstein form, whose coefficients are the control points' distances. That is, computing it costs one dot product per control point.

We want the `$t$` where `$P(t)$` can be in the fat line of `$Q$`, that is where `$d(t)\in[d_{\min},d_{\max}]$`. Now comes the step that gives the method its power. Since `$\sum_i \tfrac in B_i^n(t)=t$` (Bernstein polynomials have *linear precision*), the *graph* of `$d$` is itself a Bézier curve in the `$(t,d)$` plane:

`$$\bigl(t,\ d(t)\bigr)=\sum_{i=0}^n B_i^n(t)\,\Bigl(\tfrac in,\ d_i\Bigr).$$`

Its control points are `$(\tfrac in, d_i)$`. The convex hull property now applies to the graph. So: **the graph lies in the convex hull of the points `$(\tfrac in,d_i)$`, and any `$t$` where the graph is inside the band lies where the hull is inside the band.** That is the clip: intersect the hull with the horizontal strip `$d_{\min}\le d\le d_{\max}$`, and read off the range of `$t$`. All `$t$` outside that range are proved to hold no crossing.

### A worked example

Suppose a cubic's distances from a fat line are `$d=(-1,\,-0.4,\,0.5,\,1)$`, and the band is `$[-0.1,0.1]$`. The graph's control points are `$(0,-1)$`, `$(\tfrac13,-0.4)$`, `$(\tfrac23,0.5)$`, `$(1,1)$`.

The upper hull runs from `$(0,-1)$` to `$(\tfrac23,0.5)$` with slope `$2.25$`, then on to `$(1,1)$`; the point `$(\tfrac13,-0.4)$` lies below that first edge, whose height at `$\tfrac13$` is `$-0.25$`. The lower hull runs from `$(0,-1)$` to `$(\tfrac13,-0.4)$`, then from there to `$(1,1)$` with slope `$2.1$`.

The leftmost point of the hull inside the band is where the upper hull reaches `$d=-0.1$`: `$-1+2.25\,t=-0.1$`, or `$t=0.4$`. The rightmost is where the lower hull reaches `$d=0.1$`: `$-0.4+2.1(t-\tfrac13)=0.1$`, or `$t=\tfrac13+\tfrac{0.5}{2.1}\approx0.5714$`. So the clip is

`$$[0.4,\ 0.5714],$$`

and 83% of the curve is proved not to meet the other one, in a single step. Bisection would have thrown away at most half, and that only if the other half's box happened to be clear.

## The iteration

One clip narrows `$P$`. Now restrict `$P$` to `$[t_0,t_1]$` by de Casteljau (`$O(n^2)$`, and exact as far as the arithmetic is), and swap roles. The restricted `$P$` is shorter, so its fat line is much thinner, and `$Q$` is clipped against that. Then `$P$` again. Three things can happen to a pair.

- **The intersection with the band is empty.** The pair holds no crossing: rejected.
- **The clip removes a good part of the piece.** Restrict and go on. The usual rule, which is the kernel's too, is that a clip is *progress* if it removes at least 20%.
- **The clip barely removes anything.** The piece holds several crossings, or a tangency, or the arithmetic has reached its limit. Halve the longer piece and go on with both halves. Clipping cannot separate two crossings on one piece, since the hull covers both of them, but bisection can.

So the search stays the branch and bound of Lecture 12. Clipping replaces only the "cut at the middle" with "cut where the solutions are", wherever that is possible.

<figure class="demo" data-demo="fat-line-clip">
<figcaption>Step through the clips. Above, the two curves, the current pieces in bold, and the green fat line of one of them. Below, that piece's distance function: its Bézier graph, its control polygon's hull in blue, the green band, and in orange the clip interval. Switch between the exact hull and the kernel's coarser chord band. These curves are polynomial and the arithmetic plain floating point with a small margin in place of outward rounding; the kernel's work is rational and in intervals.</figcaption>
</figure>

The readout under the demo compares the search with plain bisection on the same scene. For its default curves, with two crossings and a width of `$10^{-9}$` in both parameters, the exact hull took 21 clips and one halving, the chord band 40 clips and 5 halvings. Plain bisection with boxes looked at 501 pairs of pieces.

### Why quadratic

Follow a single transversal crossing. The curves in `book/scripts/demo_tests/fat-line-clip.test.mjs` cross once at a good angle, and here are the widths of the parameter ranges as the demo's search narrows them, to a target width of `$10^{-12}$`:

| clip of | width of `$s$` range | width of `$t$` range |
|---|---|---|
| start | `$1$` | `$1$` |
| A | `$8.8\times10^{-2}$` | `$1$` |
| B | `$8.8\times10^{-2}$` | `$2.5\times10^{-2}$` |
| A | `$4.4\times10^{-5}$` | `$2.5\times10^{-2}$` |
| B | `$4.4\times10^{-5}$` | `$9.9\times10^{-6}$` |
| A | `$6.6\times10^{-12}$` | `$9.9\times10^{-6}$` |
| B | `$6.6\times10^{-12}$` | `$1.2\times10^{-12}$` |

Eight clips in all, against 479 pairs of boxes for plain bisection on the same two curves to the same width. The exponent roughly doubles per round trip: `$10^{-2}$`, `$10^{-5}$`, `$10^{-12}$`.

The argument is short. Let `$Q$` have parameter width `$h_Q$`, so its fat line is `$O(h_Q^2)$` thick. Let `$P$` be nearly straight, of width `$h_P$`, so its distance function is nearly linear in `$t$`, with a slope `$\sigma$` determined by the crossing angle, and departs from linear by `$O(h_P^2)$`. Then the zone where `$d(t)$` can lie in the band has width

`$$\frac{O(h_Q^2)+O(h_P^2)}{\sigma},$$`

which is second order in the sizes. So one clip squares the widths, up to the constant, and it does so as long as `$\sigma$` is not small: as long as the crossing is transversal. When `$\sigma$` is small, the denominator gives that away. We meet this at tangencies in [Lecture 15](./15-tangency-and-overlap.md). At a tangency, clipping stalls and the search falls back to bisection, with all the `$h^{-1/2}$` costs of Lecture 12.

That gives a useful comparison. Clipping is a solver with the convergence of Newton and the safety of subdivision: it cannot lose a root, and it does not need a start value. Its cost is rebuilding and restricting pieces, which is why the kernel still hands over to Newton at the end ([Lecture 14](./14-newton.md)).

## From the published method to the kernel's

Geop implements this differently in four ways. Each serves one of its rules.

### 1. No division: the rational case as a polynomial

A NURBS curve is a quotient, `$C(t)=H(t)/W(t)$` in homogeneous form. Clipping is about polynomials. It would seem you need to divide, and dividing intervals is what makes them wide ([Lecture 11](./11-evaluate-locally.md)). But weights are definitely positive (`NurbCurve::try_new` rejects anything else). So for a target coordinate `$p_k$`,

`$$C_k(t)=p_k \iff g_k(t) := H_k(t)-p_k\,W(t)=0.$$`

This is a polynomial spline again, with coefficients computed straight from the stored homogeneous coordinates:

`$$d_{k,i}=P_{i,k}-p_k\,P_{i,w},$$`

one multiplication and one subtraction per coefficient. That is the containment search in `core/geop-core-geometry/src/contains/curve.rs`, and the opening lines of its `clip` are nothing but this formula:

```rust,ignore
d.extend(
    seg.control_points
        .iter()
        .map(|q| q[k].sub(point[k].mul(q[D - 1]))),
);
let zeros = fat_line_zeros(xi, &d)?;
```

For two curves `$A=H_A/W_A$` and `$B=H_B/W_B$` the same trick, with cross multiplication, gives a spline in two independent parameters:

`$$g_k(s,t)=H_{A,k}(s)\,W_B(t)-W_A(s)\,H_{B,k}(t),\qquad d_{k,ij}=P_{i,k}\,Q_{j,w}-P_{i,w}\,Q_{j,k}.$$`

The derivation is in `core/geop-core-geometry/src/intersection/curve_curve.md`. Equating the two curves at one shared parameter would be a mistake here, since a crossing's two parameters generally differ.

### 2. Greville abscissae: B-splines rather than Béziers

A segment in the kernel need not be a single Bézier span, and the abscissae `$\tfrac in$` have a B-spline generalization. The **Greville abscissae** of degree `$p$` on a knot vector `$u$` are

`$$\xi_i=\frac{u_{i+1}+\dots+u_{i+p}}{p}.$$`

B-splines have linear precision too, `$\sum_i\xi_iN_{i,p}(t)=t$`, so the graph `$(t,g(t))$` is a B-spline curve with control points `$(\xi_i,d_i)$`. On a Bézier segment over `$[a,b]$` this reduces to `$a+\tfrac in(b-a)$`. See `greville_abscissae` in `core/geop-core-geometry/src/fat_line.rs`.

### 3. A chord band in place of the hull

Computing an exact hull with intervals is treacherous. As `core/geop-core-geometry/src/contains/curve.md` points out, a monotone-chain hull's orientation tests are three-valued under intervals, and both keeping and dropping a doubtful vertex move the chain to the wrong place, which is unsound. An all-pairs form has no such decision, but costs `$O(n^2)$`. The kernel uses the cheaper one, called *Clip B* in that note, with all the control points' uncertainty in one number.

Take the chord of the graph, from its first control point to its last, with slope `$s$`:

`$$s=\frac{d_n-d_0}{\xi_n-\xi_0},\qquad \ell(t)=d_0+s\,(t-\xi_0).$$`

Take the union of every control point's signed vertical offset from it:

`$$\Delta=\bigcup_i\bigl(d_i-\ell(\xi_i)\bigr).$$`

All graph control points lie in the band `$\ell(t)+\Delta$`, so a zero of `$g$` needs `$\ell(t)\in-\Delta$`, that is

`$$\hat Z=\xi_0-\frac{d_0+\Delta}{s}.$$`

This single interval expression is the clip. Evaluated in interval arithmetic it encloses every real choice of the inputs; losing the correlation between `$d_0$`, `$s$` and `$\Delta$` can only widen it. It is `fat_line_zeros`, and there is little more to it than this formula, with two safeguards. First, a test independent of the slope: on the domain, `$\ell$` stays between `$d_0$` and `$d_n$`, so if `$(d_0\cup d_n)+\Delta$` misses zero there is no root. Second, if `$s$` could be zero, the division fails and the axis gives no information, which is said honestly by returning the whole line (`ENTIRE`).

Note that `$\Delta$` is the *signed* union, generally not symmetric, and contains zero. "Half a tolerance on each side" would be either too loose or, on the far side, unsound.

Let us apply this to the worked example. With the band included, the coefficients are the intervals `$D_i=d_i-[-0.1,0.1]$`:

`$$D=\bigl([-1.1,-0.9],\ [-0.5,-0.3],\ [0.4,0.6],\ [0.9,1.1]\bigr).$$`

The slope is `$D_3-D_0=[1.8,2.2]$`. The offsets `$D_i-D_0-\xi_i\,s$` are `$[-0.2,0.2]$`, `$[-\tfrac13,0.2]$`, `$[-\tfrac16,0.5]$`, `$[-0.4,0.4]$`, so `$\Delta=[-0.4,0.5]$`. Then `$D_0+\Delta=[-1.5,-0.4]$` and

`$$\hat Z=-\frac{[-1.5,-0.4]}{[1.8,2.2]}=\Bigl[\frac{0.4}{2.2},\ \frac{1.5}{1.8}\Bigr]=[0.1818,\ 0.8333].$$`

Compare the exact hull's `$[0.4,0.5714]$`. The chord band is coarser, as it must be, since the band contains the hull. It is much cheaper, however, and it is still second order, so that with the same search only a few extra clips are needed: 8 either way on the one-crossing curves above, where the `$s$` widths ran `$1$`, `$0.23$`, `$9.9\times10^{-4}$`, `$2.6\times10^{-9}$`.

### 4. The second parameter: union along one index, intersection across equations

The published method alternates: clip `$P$` against `$Q$`'s fat line, then the other way. The kernel has one tensor of coefficients `$d_{ij}$` in two parameters, and needs to clip each one. How?

Fix `$t$`. Then `$g(s,t)=\sum_i\bigl(\sum_jd_{ij}M_j(t)\bigr)N_i(s)$`, and the inner sum is a convex combination of the row `$d_{i0},d_{i1},\dots$`. So it lies in the row's union

`$$D^s_i=\bigcup_jd_{ij}.$$`

For every `$t$`, the spline in `$s$` has coefficients in `$D^s_i$`. A clip of the envelope `$(\xi_i,D^s_i)$` therefore keeps every zero for every `$t$`. That is what `clip_tensor` does, in every direction.

Now look at what this is for two polynomial curves, measured in one direction `$n$`. The coefficient is `$d_{ij}=n\cdot P_i-n\cdot Q_j$`, so that

`$$D^s_i=n\cdot P_i-\bigcup_j n\cdot Q_j.$$`

That is the distance of `$P_i$` minus a band: **it is the fat line of `$Q$` measured along `$n$`**. If `$n$` is the normal of `$Q$`'s chord, the band is exactly Sederberg and Nishita's fat line. So the tensor formulation is the same method, and fat lines come out of it on their own. Which `$n$`? Any `$n$` is valid, since `$g_n=\sum_kn_kg_k$` vanishes wherever all `$g_k$` do: a free choice, and a good one is across the other curve. `clip` in `core/geop-core-geometry/src/intersection/curve_curve.rs` takes the directions square to `$B$`'s chord, nearly independent of `$t$` and so pinning `$s$`, then those square to `$A$`'s (`chord`, `spanning` in `fat_line.rs`). If the directions fail to span space, say for a closed segment without a chord or for two collinear curves, the axes are added.

Why not simply clip the coordinate axes? Because projections lose coupling. Two curves crossing on a diagonal, seen in the `$x$` and `$y$` axes alone, have boxes overlapping wherever their ranges do. A combination across the chord is nearly independent of the other parameter, so its envelope is thin, which is the whole point. `curve_curve.md` works through this example.

Two more rules deserve to be memorized, since they are correct for a reason that is not obvious.

**Union over eliminated indices, intersection over equations.** Each row of a tensor is an alternative possibility, so the rows are combined by `union`. Each equation is a constraint that must hold, simultaneously, so the clips are combined by `intersect`. Intersecting per-row clips would be wrong: as the doc of `clip_tensor` says, "a zero of the tensor need not be a zero of any row". This is the contrast from [Lecture 5](./05-honest-numerics.md): union when you must cover both possibilities, intersect when both constraints must hold at once.

**`intersect` of disjoint enclosures is not empty.** It returns an input. So disjointness is checked first, every time, with `could_be_equal`. Look at `clip_tensor` or `clip` for this pattern; it would be a nasty bug to forget it.

## Restriction and the shared schedule

Curve containment, surface containment and the two intersection searches share one schedule for what to do with a clipped box, in `fat_line.rs`.

`restriction` decides whether to restrict. A direction is cut when its clip removed at least 20% of its domain (`min_progress` is `$\tfrac45$`) and one of the clip's sharp outer bounds lies strictly inside the domain. Note what is and is not sharpened here, by the rules of [Lecture 5](./05-honest-numerics.md). The restriction cuts at the clip's *outer* bounds, `Scalar::lower` and `Scalar::upper`: any cut outside the enclosure loses nothing, and a cut through it would. A halving, by contrast, uses a sharpened midpoint, since splitting may happen anywhere.

`stalled` decides, for a box clipping could not shrink, whether it has converged or which direction to bisect, largest spatial extent first (parameter widths of differently scaled domains are not comparable). A direction with nothing left to give is skipped. Those skips have a history worth repeating. At a pole, where the solutions are a whole row and clipping closes in on it by a constant factor, it did so for thousands of steps, down to boxes of `$10^{-280}$`, which is why `restriction` stops once the spatial extents are within the width the data carries (`carried_width`).

And note how `min_subdivision_size` enters: it only bounds how far *bisection* goes. Clipping is never cut short by it, so a point merely within `min_subdivision_size` of a curve is still rejected if clipping can reject it. That makes a search's answer more precise than its handoff, without any extra thought.

<div class="kernel">

**In the kernel.** `core/geop-core-geometry/src/fat_line.rs` is under 400 lines: `greville_abscissae`, `fat_line_zeros`, `clip_tensor`, `restriction`, `stalled`, `carried_width`, `extent`, `spanning`, `chord`. Its callers:

- `curve_could_contain` (`contains/curve.rs`), with the derivation in `contains/curve.md`;
- `surface_could_contain` (`contains/surface.rs`), derivation in `contains/surface.md`;
- `curve_curve_crossings` (`intersection/curve_curve.rs`), derivation in `intersection/curve_curve.md`;
- `curve_surface_crossings` (`intersection/curve_surface.rs`), derivation in `intersection/curve_surface.md`.

For a surface, `$g_k$` has the coefficients `$d_{k,ij}=P_{ij,k}-p_kP_{ij,w}$` and two parameters; for a curve and a surface, it has three. The same collapse handles each, with the same `clip_tensor`. An exact hull (*Clip A*) is derived in `contains/curve.md`, but not implemented: its note says it could be added as a second stage where Clip B neither rejects nor contracts.

A personal note from the existing crate page: the kernel first used convex hulls and GJK, and fat line and fat surface clipping turned out far cheaper for what it needed (see [geop-core-geometry](../core/geop-core-geometry.md)).

</div>

## Recap

<div class="recap">

- A fat line is second order in a piece's length, where a box is first order. That is why clipping converges quadratically.
- The distance from a line along a Bézier curve is a Bézier function; its graph's hull meets the band in the clip interval.
- A clip is a solver with no start value that cannot lose a root. It stalls on several roots or a tangency, and there the search halves.
- The kernel's form is division-free (`$g_k=H_k-p_kW$`), on B-splines (Greville abscissae), with a chord band in place of the hull (`fat_line_zeros`), and a tensor collapsed by union per index (`clip_tensor`).
- Union over alternatives, intersect over simultaneous constraints, and test for disjointness first.

</div>

## Exercises

**1.** Compute the clip for `$d=(-2,\,-1,\,1,\,3)$` against the band `$[-0.5,0.5]$`, both by the exact hull and by the chord band. Check your answer in a calculator or by reasoning on the picture.

<details class="solution"><summary>Solution</summary>

Graph points: `$(0,-2),(\tfrac13,-1),(\tfrac23,1),(1,3)$`. Upper hull: `$(0,-2)$` to `$(\tfrac23,1)$` has slope `$4.5$`; is `$(\tfrac13,-1)$` below it? at `$\tfrac13$` that edge is at `$-0.5$`, so yes. Then `$(\tfrac23,1)$` to `$(1,3)$`, slope 6, which is steeper, so `$(\tfrac23,1)$` is *not* on the upper hull: the upper hull is the chord `$(0,-2)$` to `$(1,3)$`, slope 5. Lower hull: `$(0,-2),(\tfrac13,-1)$` slope 3, `$(\tfrac13,-1),(\tfrac23,1)$` slope 6, `$(\tfrac23,1),(1,3)$` slope 6. So left bound: `$-2+5t=-0.5$`, `$t=0.3$`. Right bound: `$-1+6(t-\tfrac13)=0.5$`, `$t=\tfrac13+\tfrac14=0.5833$`. Hull clip: `$[0.3,0.5833]$`.

Chord band: `$D_0=[-2.5,-1.5]$`, `$D_1=[-1.5,-0.5]$`, `$D_2=[0.5,1.5]$`, `$D_3=[2.5,3.5]$`, so the slope is `$[4,6]$`. The offsets `$D_i-D_0-\xi_is$` are `$[-1,1]$`, `$[-2,\tfrac23]$`, `$[-2,\tfrac43]$` and `$[-2,2]$`. So `$\Delta=[-2,2]$`, `$D_0+\Delta=[-4.5,0.5]$`, and `$-[-4.5,0.5]/[4,6]=[-0.125,1.125]$`, which clamped to the domain is `$[0,1]$`: no information at all. The band here is wide compared to the curve's rise. It thins quadratically as the pieces shrink, and the interval slope sharpens with it, so a few levels on the same formula is sharp.

</details>

**2.** In the demo, try to make clipping stall. What kind of curves make it need many halvings? Make a pair that has two crossings close together. What do the first clips do?

<details class="solution"><summary>Hint</summary>

A clip's hull covers both crossings, so the first clips keep a range containing both, and remove little. Once the range is short enough that the fat line separates the two crossings, or after a halving between them, each half converges fast. Crossings closer together need more halvings first; this is the isolation that subdivision is for.

</details>

**3.** Show that a Bernstein polynomial `$\sum_i\tfrac inB_i^n(t)$` equals `$t$`, using the derivative formula of [Lecture 6](./06-bezier.md). Conclude the graph claim.

<details class="solution"><summary>Solution</summary>

`$f(t)=\sum_i\tfrac inB_i^n(t)$` is a Bézier function whose first differences of coefficients are all `$\tfrac1n$`, so `$f'(t)=n\sum_iB_i^{n-1}(t)\tfrac1n=1$`. Also `$f(0)=0$`, since the coefficient at `$i=0$` is 0. Hence `$f(t)=t$`. Therefore `$(t,d(t))=\sum_iB_i^n(t)(\tfrac in,d_i)$`.

</details>

**4.** Why does `fat_line_zeros` test `$(d_0\cup d_n)+\Delta$` before dividing, and why would a test of `$d_0+\Delta$` alone be unsound? Construct an example.

<details class="solution"><summary>Solution</summary>

Use exact numbers first. A straight spline with `$d_0=1$`, `$d_n=-3$` has `$\Delta=\{0\}$`, so `$d_0+\Delta=\{1\}$` excludes zero, and yet it has a root in between: testing `$d_0+\Delta$` alone would reject a true solution. It would seem harmless, since here the slope is plainly not zero, and the division would answer correctly. But this test is the one used when the division cannot be done, because the slope *could* be zero; and a slope that could be zero may still not be. So take `$d_n=[-3,3]$`, with true value `$-3$`. The chord `$\ell$` of the true spline moves across the whole hull of `$d_0$` and `$d_n$`, and that hull, plus `$\Delta$`, is what has to be tested. `contains/curve.md`, section 4, states this.

</details>

**5.** Consider a pair of curves in 3-D that do not cross. Could the tensor clip fail to reject them, however far subdivided, if it only used the three coordinate axes? What about in-plane directions?

<details class="solution"><summary>Hint</summary>

A sound clip must not reject a solution, but it may keep non-solutions while the pieces are big. As the pieces shrink, boxes separate, since the curves are a positive distance apart, and each axis's coefficients become sign-definite in one direction at least. So it will reject in the end. What the choice of directions buys is how soon. In 3-D, `curve_curve::clip` uses the `$C-1=2$` directions square to the other chord, and those pin the parameter far earlier than axes do.

</details>

**6.** A colleague suggests a speedup: instead of the union over `$j$` in `clip_tensor`, use the row at the middle of the other parameter, since "it is the most representative". Show with a small tensor that this can reject a true solution.

<details class="solution"><summary>Solution</summary>

Take `$g(s,t)=(s-\tfrac12)+(t-\tfrac12)\cdot 4$`, bilinear, on `$[0,1]^2$`. Its coefficients are `$d_{00}=-2.5$`, `$d_{01}=1.5$`, `$d_{10}=-1.5$`, `$d_{11}=2.5$`. At `$t=\tfrac12$`, the row in `$s$` is `$-0.5,0.5$`, whose clip gives `$s=\tfrac12$` exactly. But `$(s,t)=(0,\tfrac58)$` is a zero, with `$s=0$`. A clip built from that row alone would drop it. The union, `$D^s_0=[-2.5,1.5]$`, `$D^s_1=[-1.5,2.5]$`, keeps every `$s$`, and rightly so: this direction has no information at this size.

</details>
