# Lecture 11: Evaluating relative to the patch

<div class="goals">

**After this lecture you can**

- explain why `$A/W$` over an interval parameter has a width that grows with the distance from the origin, and compute that width;
- evaluate a rational spline relative to a sharp control point, and say what that costs;
- explain why the kernel intersects the absolute and relative enclosures of a curve point;
- form the mean value form, and say when it beats direct evaluation;
- name the rule behind all this: move correlated quantities to a local origin;
- write a test that asserts the width of an enclosure, not just that it encloses.

</div>

This lecture is about one line of arithmetic, and it is a model of how numerical debugging goes in this kernel. It starts with a real failure, descends to a specific division, and ends with a rule. We will compute it live.

## A cylinder that overlapped itself

AGENTS.md, in "Evaluate relative to the patch", records a failure in fixed-point arithmetic. A cylinder's cap lying 50 along the axis came out, when evaluated over a parameter box, spanning `$[38,94]$` in height. All the points of the cap are at height 50. Validation found the cylinder's two caps overlapping. A quarter arc at `$(1000,-500)$` gave a tangent 30 wide.

Today's `Scalar` is one concrete `f64` interval (`core/geop-core-math/src/scalars/mod.rs`), and the `f64` format hides the *rounding* part of this. It does not hide the rest, as we are about to see: what went wrong is the formula, and it can be reproduced in doubles.

## The formula

Recall from [Lecture 8](./08-nurbs.md) that a point of a NURBS curve is `$C(t)=A(t)/W(t)$`, with `$A$` and `$W$` polynomial. Take the height `$z$` of a curve lying in the plane `$z=z_0$`. Upstairs `$A_z(t)=z_0W(t)$`, so `$C_z(t)=z_0$`, exactly.

Now let `$t$` be an interval, as it often is. Interval de Casteljau gives an enclosure of `$A_z$` and of `$W$`, computed separately. Let `$W(t)$` range over `$[W_{\min},W_{\max}]$` on this box. Then `$A_z$` encloses `$[z_0W_{\min},z_0W_{\max}]$`, and interval division gives

`$$\frac{[z_0W_{\min},z_0W_{\max}]}{[W_{\min},W_{\max}]}=z_0\left[\frac{W_{\min}}{W_{\max}},\frac{W_{\max}}{W_{\min}}\right].$$`

The numerator and denominator move together, exactly in step, but interval arithmetic has no way to know that. So the quotient has width about

`$$\text{width}\approx 2\,\lvert z_0\rvert\,\frac{W_{\max}-W_{\min}}{W}.$$`

It is **proportional to the distance from the origin**, and to how much the weight varies over the box (in practice more than the true `$W_{\max}-W_{\min}$`, since interval de Casteljau overestimates that range too). It has nothing to do with the geometry: a plane at height `$z_0$` has no extent in `$z$`. Move the part away from the origin and the enclosure grows, though the part has not changed.

The same formula shows why this kernel's arcs are the usual victims. Polynomial curves have `$W=1$`, and this effect is absent. On a quarter arc, `$W$` runs from 1 at the span's ends to about 0.85 in its middle.

Let's put numbers to it. We computed an arc of radius 3 at `$(1000,-500)$` at height 50, the configuration of the kernel's test `far_arc` in `core/geop-core-geometry/src/nurb_curve/evaluate.rs`, with `$t\in[0.33,0.34]$`, by the demo below. The height's enclosure comes out `$[49.66,50.34]$`, width `$0.67$`. A wide `$t\in[0.1,0.9]$` gives `$[22.2,112.4]$`. In doubles! So that fixed-point number `$[38,94]$` was not primarily about fixed point. The test `a_cap_far_from_the_origin_stays_flat` in `nurb_surface/evaluate.rs` records the original case.

## The cure: a local origin

Why should the distance to the origin matter at all? There is nothing special about the origin. The patch has its own natural place: **any control point of it**.

So translate the patch to bring one of its control points to the origin, evaluate, and translate back. Pick `$O=P_k$`, as a sharp point. Upstairs, translation by `$-O$` keeps the weights and changes the numerators: `$A_i\mapsto A_i-w_iO$`. Then

`$$C(t)=O+\frac{\tilde A(t)}{W(t)},\qquad\tilde A=\sum_iN_i(t)\,w_i\,(P_i-O).$$`

Now `$\lvert\tilde A\rvert$` is at most about the patch's size, however far the patch is from the origin. The same `$A/W$` width formula now gives

`$$\text{width}\approx 2\,\lvert\text{extent}\rvert\,\frac{W_{\max}-W_{\min}}{W},$$`

proportional to the **patch's size**, and zero along any direction the patch is flat in. Adding `$O$` back costs a single rounding.

Which point is a free choice. And in the spirit of "a free choice still has to be a good one", the kernel takes a control point at the nearer end of the span: `nearer_end` in `core/geop-core-geometry/src/spline.rs`. The doc comment on `centered` adds why that end is the best: at a pole, the homogeneous rows differ by their weights times the pole's distance from the origin, and so do `$A_u$` and `$W_uS$`, whose difference is the derivative along the row. "Relative to the pole, both are of the order of that distance, and so is their difference."

One more point of detail, typical of this kernel. The origin is sharpened: `at.sharpen()`, since which origin to use is a free choice and a sharp one costs nothing. It is one division, "not a reciprocal and a product: a quotient the format holds, `2 / 1`, is then the middle of its enclosure, and the point sits exactly on the origin, with nothing of its own left over for the derivatives to see."

Now see it for yourself.

<figure class="demo" data-demo="centered-eval">
<figcaption>Real outward-rounded interval arithmetic, in your browser: a quarter arc of radius 3 is placed at a distance (the horizontal axis), and evaluated over a parameter box of the chosen width. Red: A / W. Blue: relative to a control point, as <code>spline::centered</code> does. Green: the mean value form. Dashed: the true extent of the piece of arc the box covers. The height (the arc is flat in z) is in the readout. The computation mirrors <code>evaluate_into</code> for one Bézier span; the kernel does not use the mean value form on curves, only on surfaces (see below). Try a sharp t: then only rounding is left, which does grow with distance on every line, since it is a relative thing.</figcaption>
</figure>

Things to notice:

1. With a box of width `$0.01$`, red rises in proportion to the distance; blue stays flat at about `$0.065$`, for distances up to `$10^6$`, while the true stretch is about `$0.043$`.
2. In the readout, the height of the relative form is narrower than `$10^{-10}$` at any distance on the slider, where the direct form's reaches `$6.7\times10^2$` at `$10^6$`.
3. Change the box to "sharp": red and blue come within a small factor of each other. Both are rounding only.

That last point is exactly the AGENTS.md remark: "in `f64` the effect hides behind the format's relative precision until the width is read". A test that only checks that the enclosure contains the right value keeps passing, because a wide enclosure contains everything. That is why **a test of geometry far from the origin must assert the width of what it gets**:

```rust
assert!(p[2].width().to_f64() < 1e-6, "{p:?}");
```

is from `an_arc_far_from_the_origin_is_as_wide_as_its_stretch` in `nurb_curve/evaluate.rs`. And `vertex.point[k].width().to_f64() < 1e-9` is in `ops/geop-ops-surface/src/knit/tests.rs`, among others.

## That one rounding is not free

Should we then always evaluate relative to the patch? AGENTS.md's next paragraph says no, with a beautifully tiny example:

> A STEP edge lay `2e-31` above its plane face. Evaluated relative to a point `1.8e-15` up, its enclosure took in the plane, the importer measured no gap and did not widen the edge, and validation's clipping, which reads the control points, found the face's points off it.

Follow what happens. A number near `$10^{-31}$` is held with a relative precision of `$10^{-16}$`, so to about `$10^{-47}$`. It is added to an origin `$O=1.8\times10^{-15}$`, whose spacing between doubles is about `$4\times10^{-31}$`. That single rounding is bigger than the value. The direct form never adds that origin back, so it has no such rounding.

So neither form dominates. Direct evaluation is tighter where the numbers are small to begin with; relative evaluation is tighter where the patch is far from the origin. Both are valid enclosures of one value. By the rule of [Lecture 5](./05-honest-numerics.md): **intersect** them. In `evaluate_into`:

```rust
let absolute = de_boor(p, knots, local, piece, p);
let (moved, origin) = centered(local, near);
let relative = de_boor(p, knots, &moved, piece, p);
// The weights are not moved: one `W` for both.
let inv_w = inverse(absolute[D - 1], span)?;
for (c, o) in out.iter_mut().enumerate() {
    let near = origin[c].add(relative[c].mul(inv_w));
    let value = absolute[c].mul(inv_w).intersect(near);
    *o = if reached { o.union(value) } else { value };
}
```

Read that code as a summary of three lectures. The `intersect` combines two enclosures of the same point. The `union` covers the spans a wide parameter reaches, which are different points ([Lecture 7](./07-bsplines.md)). And the division by `$W$` has one numerator for each form, with the same `$W$`, as translating leaves it unchanged.

`NurbCurve::cartesian_derivatives` in `nurb_curve/tangent.rs` does the same for derivatives: `at(local)`, and `at(&centered(local, near).0)`, intersected. Derivatives do not depend on where the curve lies, but their computation does.

Surfaces follow the pattern differently. `NurbSurface::evaluate_at` evaluates only relative to the corner nearest the point (`corner_nearest`), and does not form the absolute value at all.

## Beyond the local origin: the mean value form

Moving the origin takes away the part of the width caused by distance. There is a second effect it does not fix. Look at the formula again: even relative to a control point, the width is `$2\lvert\text{extent}\rvert\Delta W/W$`, so the quotient widens with how far the point lies from the span's control point times how much the weight changes. On a span of a rational surface, with a foot-point box of `$(u,v)$`, that can still be much too wide.

AGENTS.md's "A bound is as local as what it bounds" gives the case. A shell's copy of a traced edge, fitted through foot points evaluated over the boxes they project to, came out eight times as wide as the edge, and two offsets in a row went past the `1e-4` an entity may carry. The doc comment on `NurbSurface::evaluate` adds: "a cylinder evaluated at a foot point a micron wide came out ten microns wide along its axis".

Enter a classic tool of interval analysis. By the mean value theorem, for `$t$` in a box with middle `$m$`,

`$$S(t)\in S(m)+S'(\text{box})\,(\text{box}-m).$$`

This is the **mean value form**. Its width is `$\lvert S'(\text{box})\rvert$` times the box's width: the actual stretch of geometry, as long as the derivative's enclosure is decent. It adds no factor of `$\Delta W/W$`. The price is computing the derivative over the box. A classical result of interval analysis is that the mean value form's excess width shrinks as the *square* of the box's width, whereas the direct evaluation's shrinks only with its width. So for narrow boxes it wins, and for wide ones it can be worse, in a given coordinate: when `$S'(\text{box})$` is wide, the product is large. In the demo, at a box of `$0.0001$`, green lies on the dashed line, at `$0.01$` just above it, and at `$0.1$` further off.

So `NurbSurface::evaluate` again intersects:

```rust
let direct = self.evaluate_at(u, v)?;
if u.is_sharp() && v.is_sharp() {
    return Ok(direct);
}
// Only ever tighter: where the partials over the box cannot be had,
// as at a pole or past the domain, the direct enclosure stands.
let (um, vm) = (u.midpoint(), v.midpoint());
let (Ok(middle), Ok((su, sv))) = (self.evaluate_at(um, vm), self.derivatives(u, v)) else {
    return Ok(direct);
};
let (du, dv) = (u.sub(um), v.sub(vm));
Ok(Vector3::from_array(std::array::from_fn(|c| {
    let mean = middle[c].add(su[c].mul(du)).add(sv[c].mul(dv));
    direct[c].intersect(mean)
})))
```

Note the structure: the improved form may only tighten, and any failure leaves the direct enclosure standing. That matches the refinement rule of [Lecture 14](./14-newton.md): make it infallible, or callers start needing fallbacks of their own. Curves do not currently do this: `evaluate_into` has no mean value step.

### Ratios of Bernstein coefficients

The same section of AGENTS.md has one more related trick, worth noting here. A ratio of two polynomials bounded by `$\max(\text{numerator})/\min(\text{denominator})$` over a box suffers the same `$\Delta W$` factor. For a polynomial `$\sum B_ia_i$` over a positive polynomial `$\sum B_id_i$`, there is a sharper bound:

`$$\frac{\sum_iB_ia_i}{\sum_iB_id_i}\le\max_i\frac{a_i}{d_i},$$`

as the left side is a convex combination of the right-side ratios. This is free, and it took a vertex's width from `6e-11` to `1.4e-12`, according to AGENTS.md (`ratio_bound`, in the fitted offset). It is the same phenomenon: when correlated quantities are divided, take them together, not one at a time.

## The general rule

We can now state it, in AGENTS.md's words: "Wherever an interval formula subtracts or divides two large correlated quantities, move it to a local origin first."

You met the phenomenon as `$x-x$` in [Lecture 4](./04-intervals.md). Here it appears in its mature form. Ask of any interval formula: which quantities in it are in truth related, by how much, and is there a form of the computation where the relation is exploited?

- `$A/W$` with `$A\propto W$`: subtract the local origin first, so that `$A$` is small.
- `$(1-\alpha)a+\alpha b$` with `$a\approx b$`: take `$b-a$` first ([Lecture 9](./09-subdivision.md)).
- A sum of basis-function derivatives with `$t$` in every term: difference the control points first ([Lecture 7](./07-bsplines.md)).
- A rational surface over a box: take its value at the middle, and add the derivative over the box times the half width (the mean value form).

None of those are approximations, and none use a tolerance. Each is a different *exact* expression for the same number, chosen so that outward rounding overestimates it less.

## Why this matters so much

Every search in the following lectures is an interval computation over a box. If the enclosure of a surface over a box is `$1000$` times wider than the piece of surface, then "could these overlap?" is true far too often, the search subdivides for nothing, and at the bottom it reports solutions where none are. A part's distance from the world origin is not supposed to change anything, and now it doesn't.

An exercise in humility, also from AGENTS.md: a plate placed at `$(10000,-5000,2000)$` once took its extent as the greatest coordinate less the least over *all three axes*: 15080, for a plate 80 long. It is the same mistake in a different place: a question about the patch answered in terms of the world. [Lecture 19](./19-classification.md) tells the rest.

## Recap

<div class="recap">

- `$A/W$` over a parameter box divides two enclosures that move together: the width is `$2\lvert A/W\rvert\,\Delta W/W$`, proportional to the distance from the origin.
- Evaluate relative to a sharp control point near the parameter: the width is then proportional to the patch's size, and zero where it is flat.
- That costs one rounding; near the origin that can be more than the absolute form's. So curves evaluate both ways and intersect.
- The mean value form, `$S(m)+S'(\text{box})(\text{box}-m)$`, removes the remaining `$\Delta W$` factor where partials can be had. Surfaces intersect it with the direct form.
- A test of geometry far from the origin must read the enclosure's width: in `f64`, a wide enclosure can otherwise hide behind the right value.

</div>

## Exercises

**1.** Take `$P_0=(100,0)$`, `$P_1=(100,1)$`, `$P_2=(99,1)$` with weights `$1,\frac{\sqrt2}2,1$`, a unit quarter arc around `$(99,0)$`. For `$t\in[0.4,0.6]$`, compute by hand the enclosure of `$W$`, and the width of `$x$` as `$A_x/W$`, ignoring rounding. Now do it relative to `$P_0$`. Compare with the actual `$x$`-extent of the arc over that box, using the formula `$x-99=(a+b)/(a+b+e)$`.

<details class="solution"><summary>Solution</summary>

With `$h=\frac{\sqrt2}2$`, `$W(t)=1-2(1-h)t(1-t)$`, which has its minimum at `$t=\frac12$`: the true range over the box is `$[0.8536,0.8594]$`. Direct: `$x$` is near `$99.7$`, so the width formula with this range gives about `$2\cdot99.7\cdot\frac{0.0058}{0.856}\approx1.35$`. That is a lower limit: interval de Casteljau overestimates `$W$`'s range, and the demo's `direct` gives a width of `$21.8$`, the interval `$[89.1,110.9]$`. Relative to `$P_0$`: `relative` gives a width of `$0.256$`. The true extent is `$x(0.4)-x(0.6)=0.8138-0.5811=0.233$`. So the relative form is within ten percent of the truth, and the direct form is a hundred times off. The mean value form gives `$0.304$` in `$x$` over this rather wide box, and `$0.290$` in `$y$` against the relative form's `$0.450$`: which is the tighter depends on the coordinate.

</details>

**2.** Write `$C(t)=O+\tilde A/W$` for the point `$O$` a sharp point near the patch but *not* a control point. Is the formula still valid? Which properties of the choice did the kernel's `nearer_end` optimize, and what is lost with another `$O$`?

<details class="solution"><summary>Hint</summary>

The formula is an identity for any sharp `$O$`. Choosing a control point means that `$\lvert\tilde A\rvert$` is bounded by the patch size, and that at a clamped end, the value is exactly zero, plus the point. The nearer end makes the moved points near the parameter small, which matters for the pole case in the doc comment on `centered`.

</details>

**3.** The demo at sharp `$t$` shows both forms growing with distance. Explain that growth: where does it come from, and why is it unavoidable? At what distance is the absolute precision of an `f64` coordinate `$10^{-9}$`?

<details class="solution"><summary>Solution</summary>

A double near `$d$` has spacing `$d\cdot2^{-52}$`: this is the format's relative precision. The result has to be stored as a number near `$d$`, so no formula can do better than that. Spacing `$10^{-9}$` occurs at `$d\approx4.5\times10^{6}$`. The goal of `centered` was to remove the excess beyond that limit: the part due to the correlation, not the format.

</details>

**4.** Prove the mean value form: for `$f$` continuously differentiable and `$t$` in a box `$B$` with middle `$m$`, `$f(t)\in f(m)+f'(B)(B-m)$`. What does the form give in the degenerate case `$f(t)=z_0$` (constant height), and how does this compare with `$A/W$`?

<details class="solution"><summary>Solution</summary>

By the mean value theorem `$f(t)=f(m)+f'(\xi)(t-m)$` with `$\xi\in B$`, so `$f'(\xi)\in f'(B)$` and `$t-m\in B-m$`. The enclosure follows by inclusion monotonicity. For `$f=z_0$`: `$f'$` is about zero (rounding only), so the width is that of `$f(m)$` plus rounding. Compare: `$A/W$` has the width `$2z_0\Delta W/W$`. So the mean value form sees the flatness, as long as `$f'(B)$` is computed so as to see it. This is why the kernel differences first when computing partials.

</details>

**5.** Given the Bernstein coefficients of numerator `$a=(2,1,3)$` and denominator `$d=(1,2,1)$`, bound `$a(t)/d(t)$` on `$[0,1]$` by (a) `$\max a/\min d$`, (b) `$\max a_i/d_i$`, and (c) the true maximum. Which rule does `ratio_bound` use, and why is it free?

<details class="solution"><summary>Solution</summary>

(a) `$3/1=3$`. (b) `$\max(2,\tfrac12,3)=3$`. Equal here; take `$d=(2,2,1)$`, `$a=(4,1,2)$`, and (a) gives `$4$`, while (b) gives `$2$`. For (c) with the first pair: the ratio at `$t=1$` is 3, so the bound (b) is attained. The kernel uses (b): it reads only the coefficients at hand, no search and no evaluation, and the bound holds since the ratio is a convex combination of the `$a_i/d_i$` with weights `$B_id_i/\sum_jB_jd_j$`.

</details>

**6.** *Test design.* Write the assertions a test should make for a cylinder of radius 2 placed with its axis through `$(5000,-2000,800)$`, after one boolean. Name at least three: what would you measure, and what do you compare it to?

<details class="solution"><summary>Hint</summary>

Compare the same operation at the origin. Each vertex's width (`width().to_f64()`) should not grow by more than the format's relative precision implies: bounded by a small multiple of `$d\cdot2^{-52}$`, plus what the part at the origin has. The same goes for the width of each edge's control points. Then check the result's validity, and that a cap's height enclosure stays narrow when evaluated over a *wide* box. A test that only asks whether the right value is contained is the failure this lecture warns about.

</details>
