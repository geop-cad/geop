# Lecture 4: Interval arithmetic and enclosures

<div class="goals">

**After this lecture you can**

- define an enclosure, and compute with intervals by hand: sum, difference, product, quotient;
- state and prove the inclusion property, the theorem everything else rests on;
- explain outward rounding and how Geop's `Scalar` does it;
- recognize the dependency problem, and reduce it by rewriting the formula, by the mean value form, and by subdividing;
- use three-valued comparisons, and say what each of `could_be_equal`, `definitely_greater`, `is_exactly` and friends means;
- explain the wrapping effect, and why construction code is sensitive at the last bit.

</div>

The previous lecture ended with a problem: a computed float is wrong by an amount it does not tell you, and comparisons based on it are guesses. The remedy is older than CAD. Moore's *Interval Analysis* dates from 1966. Rather than one number, carry two: a lower and an upper bound on the true value. The bounds are themselves floats, and all arithmetic is arranged so that the truth always stays between them.

## Enclosures

Let `$x^*$` be a real number we would like to know: the height of some surface at some parameter. An **enclosure** of `$x^*$` is an interval `$[x_{lo},x_{hi}]$` with

`$$x_{lo}\le x^*\le x_{hi}.$$`

A narrow enclosure is more useful than a wide one, but a wide enclosure is still *true*. This reverses the usual relationship between numerics and correctness. Ordinarily an answer of `$3.14159$` is more or less accurate, and you don't know how much. An enclosure is `$[3.14,3.15]$`, and it is either valid or a bug. All the imprecision is out in the open, and it is measured, as its **width** `$x_{hi}-x_{lo}$`.

## The arithmetic

To add two enclosures: if `$x\in[a,b]$` and `$y\in[c,d]$` then `$x+y\in[a+c,b+d]$`. Each operation asks for the least and greatest of `$x\circ y$` over the box.

`$$\begin{aligned}
[a,b]+[c,d]&=[a+c,\;b+d]\\
[a,b]-[c,d]&=[a-d,\;b-c]\\
[a,b]\cdot[c,d]&=[\min(ac,ad,bc,bd),\;\max(ac,ad,bc,bd)]\\
[a,b]/[c,d]&=[a,b]\cdot[1/d,\;1/c]\qquad\text{if }0\notin[c,d]
\end{aligned}$$`

Subtraction crosses over: the lowest difference pairs the lowest `$x$` with the highest `$y$`. Division by an interval containing zero is undefined, since the quotient could be anything; we return an error rather than something that merely looks like an answer. Monotone functions like `$\sqrt{\cdot}$` take their end points to the result's end points. Non-monotone ones need more care, `$\sin$` for instance has to include 1 if the interval passes through `$\pi/2$`.

A few by hand:

| Expression | Result |
|---|---|
| `$[1,2]+[3,4]$` | `$[4,6]$` |
| `$[1,2]-[3,4]$` | `$[-3,-1]$` |
| `$[-1,2]\cdot[3,4]$` | `$[-4,8]$` |
| `$[1,2]/[4,5]$` | `$[0.2,0.5]$` |
| `$[1,2]/[-1,1]$` | error |

### The inclusion property

Why trust this? Because of one simple theorem, which is why interval analysis works at all.

**Theorem (inclusion).** Let `$f$` be a function written as an expression in `$+,-,\cdot,/$` and elementary functions, each variable replaced by an interval. Let `$F(X_1,\dots,X_n)$` be the result of evaluating that expression with the interval operations above. Then for all `$x_i\in X_i$`,
`$$f(x_1,\dots,x_n)\in F(X_1,\dots,X_n).$$`

*Proof.* By induction over the expression tree. It holds for a variable. For a node `$A\circ B$`, the induction hypothesis says that the true values of `$A$` and `$B$` lie in the computed intervals, and by construction of `$\circ$`, every `$a\circ b$` for `$a,b$` in those intervals lies in the output. `$\square$`

The theorem's quiet power: it holds for **every point in the box at once**. Evaluate `$f$` on `$[0,1]$` and you have a claim about uncountably many inputs, using a finite number of operations. If the result excludes zero, then it is *proved* that `$f$` has no root in `$[0,1]$`. No sample can do that. This is what makes subdivision searches in Part III possible: they evaluate over a whole region, and may discard the whole region with certainty.

## Outward rounding

In a computer, `$a+c$` must itself be rounded to a float. If it rounds downward for the upper bound, then the truth might fall outside. So lower bounds must round down and upper bounds up: **outward rounding**. Inclusion then still holds, whatever the rounding errors were.

IEEE 754 hardware offers rounding modes for this, but switching modes is awkward and slow, and WebAssembly, where Geop also runs, only has round-to-nearest. Geop does something simpler. Since round-to-nearest is within half an ulp of the exact result, stepping the result by one whole float outward is guaranteed to enclose it:

```rust,ignore
pub fn add(self, other: Self) -> Self {
    Scalar::new(next_down(self.lo + other.lo), next_up(self.hi + other.hi))
}
```

This is `Scalar::add`, from `core/geop-core-math/src/scalars/mod.rs`; `Scalar` is simply a struct of two public fields `lo` and `hi`, both `f64`. The other operations are the same with the formulas above. So:

| Rust | `lo` | `hi` |
|---|---|---|
| `Scalar::from_f64(0.1).add(Scalar::from_f64(0.2))` | `0.3` | `0.3000000000000001` |
| `Scalar::from_f64(0.7).mul(Scalar::ONE)` | `0.6999999999999998` | `0.7000000000000001` |
| `Scalar::from_f64(0.7).sub(Scalar::from_f64(0.7))` | `-5e-324` | `5e-324` |

Each row deserves a remark.

*Row one.* The exact sum lies at `$0.3000000000000000166$`, between the two bounds, so the enclosure is true. It isn't the tightest interval possible, since rounding by a full ulp each side covers more than needed, but it's cheap and it is what the kernel uses. And it does tell you what `0.1 + 0.2` did not: that the result is not to be compared with equality.

Note that `from_f64(0.1)` means *exactly the double nearest 0.1*, a sharp interval of width zero: the kernel takes that number at its word as the value meant. If you mean the true rational `$1/10$`, `Scalar::from_ratio(1, 10)` encloses it: one ulp to each side of the rounded quotient.

*Row two.* Multiplying by exactly 1 widened the value by an ulp each side. Rounding outward cannot tell that the operation was exact. This seems harmless, but as `AGENTS.md` records, rewriting extrude and revolve as one sweep builder multiplied every station row by a weight of exactly 1, and that alone broke six "inscribed cylinder" boolean tests, whose geometry was identical. The rule from there: **never multiply or add by exact constants needlessly in construction code**. The kernel has `Scalar::is_exactly` partly for this: "A weight that is exactly one is skipped rather than multiplied by, since the product would only be widened by rounding."

*Row three.* `$0.7-0.7$` is exactly 0, yet the result has width `$10^{-323}$`. In `next_up`, the step from zero goes to the smallest subnormal. So that result is not `is_zero()` (which needs both bounds to be 0 exactly), but it `could_be_equal` zero.

Functions from the math library get wider treatment, because they are not correctly rounded. `Scalar::sin` and `Scalar::cos` evaluate at both ends, step outward by an ulp, and recognize the extrema inside the interval structurally rather than numerically, through `contains_periodic`, then they clamp to `$[-1,1]$`. `Scalar::atan2` is more interesting: it takes the bounds in plain `f64` at the corners of the box, and *proves* them afterwards in outward-rounded arithmetic, moving a bound outward if the proof fails. A good pattern to remember: a float is a free choice of where to put the bound, as long as you prove it holds.

`Scalar::div` fails with "divisor interval contains zero" instead of returning an unbounded interval. A computation that loses its denominator in rounding errors therefore comes back as an error, with context, at the place it happened.

## Three-valued comparison

Given two enclosures, what can we say about the true values? These are the kernel's comparison operators, all one line each:

| Method | True when |
|---|---|
| `could_be_equal` | the intervals overlap, so the values might be equal: `$a_{lo}\le b_{hi}$` and `$b_{lo}\le a_{hi}$` |
| `definitely_not_equal` | they are disjoint, proven different: `$a_{hi}<b_{lo}$` or `$a_{lo}>b_{hi}$` |
| `definitely_greater` | every value of `$a$` exceeds every value of `$b$`: `$a_{lo}>b_{hi}$` |
| `could_be_greater` | some value of `$a$` exceeds some value of `$b$`: `$a_{hi}>b_{lo}$` |
| `definitely_less` | `$a_{hi}<b_{lo}$` |
| `could_be_less` | `$a_{lo}<b_{hi}$` |
| `is_subset_of` | as sets, what a Krawczyk proof asks ([Lecture 24](./24-rank-and-proof.md)): `$b_{lo}\le a_{lo}$` and `$a_{hi}\le b_{hi}$` |
| `is_exactly` | subset both ways: the same value or the same range, bit for bit |

`definite_sign` sums it up for one number: `Some(true)`, `Some(false)`, or `None` if it could be zero.

Try them on some cases:

| `$a$` | `$b$` | `could_be_equal` | `definitely_less` | `could_be_less` | `definitely_greater` |
|---|---|---|---|---|---|
| `$[1,2]$` | `$[3,4]$` | no | yes | yes | no |
| `$[1,3]$` | `$[2,4]$` | yes | no | yes | no |
| `$[1,2]$` | `$[2,3]$` | yes | no | yes | no |
| `$[2,2]$` | `$[2,2]$` | yes | no | no | no |

The operators give a logic with three outcomes in effect: yes, no, *undecided*. Two different kinds of question get asked of it, and the difference is crucial:

- "Could this curve cross this face?" If the answer is "could be", the search must keep looking: discarding a region without proof could lose a real intersection.
- "Is this definitely inside?" If the answer isn't "definitely", you may not build on it, and need to subdivide, try another point, or stop and report.

Undecided is a real answer that must be handled. There is no `if a < b { } else { }`: you always have to think about the third case. This is more work to write, and exactly what makes the code correct in the bad cases. In the orientation demo of [Lecture 3](./03-floating-point.md), the intervals give no wrong sign at any of the 65 536 positions of the largest grid. Where f64 gave a confident wrong answer, they say "cannot tell".

You will also notice operations which are not comparisons:

- `Scalar::width` returns `$hi-lo$` as a **sharp** value, so that a comparison of widths is always decided;
- `Scalar::lower` and `Scalar::upper` return the bounds as sharp values, for cutting a domain at the outer bound of an enclosure, which loses nothing;
- `Scalar::sharpen` collapses to the midpoint. It is dangerous, and has the next lecture to itself;
- `Scalar::ENTIRE`, `$(-\infty,\infty)$`, is the interval that could be anything. It stands in for a value not yet known;
- `Scalar::max`, `Scalar::min` and `Scalar::clamped` are honest versions of their namesakes: `clamped` moves a value to a bound only where it is *definitely* beyond it.

## The dependency problem

Interval arithmetic has one great weakness, and much of the craft is about managing it. What is `$x-x$` for `$x\in[1,2]$`? It is plainly 0. But the formula gives

`$$[1,2]-[1,2]=[1-2,\;2-1]=[-1,1].$$`

Interval subtraction treats the two operands as *independent*. The formula's two appearances of `$x$` might as well be different numbers, each of which may independently be anywhere in the interval. All correlation between them is forgotten. The result is true but needlessly wide. This is the **dependency problem**.

A second example: let `$f(x)=x(1-x)$`, for `$x\in[0,1]$`. The true range is `$[0,\tfrac14]$`. Interval evaluation gives `$[0,1]\cdot[0,1]=[0,1]$`, four times too wide. Now try it yourself.

<figure class="demo" data-demo="interval-lab">
<figcaption>The bars under the plot, on one scale, are the true range of <em>f</em> over the shaded input (computed from the ends and the zeros of the derivative, not from samples), then three enclosures: naive evaluation (red), the mean value form (blue), and naive evaluation over pieces (green), whose boxes are drawn on the curve. The arithmetic is that of the kernel's <code>Scalar</code>, one ulp outward per operation. Shrink the half width and read the widths below the plot: naive excess falls in proportion, mean value excess with the square.</figcaption>
</figure>

There are three standard ways out, and the kernel uses all of them.

**Rewrite the formula so each variable occurs once.** If each variable appears only once in an expression, then interval evaluation gives the *exact* range (rounding aside). For `$x(1-x)=\tfrac14-(x-\tfrac12)^2$`, with a proper square that knows it's non-negative, you get `$[0,\tfrac14]$` on `$[0,1]$`. When you can, this is the cheapest fix. A related lesson is in `Scalar::interpolate`, with the two forms of linear interpolation:

`$$a+\alpha(b-a)\qquad\text{and}\qquad(1-\alpha)a+\alpha b.$$`

Mathematically the same; numerically opposites. The first mentions `$a$` twice and counts its width twice, but `$b-a$` is computed first. So if `$a$` and `$b$` are the same value, as with a weight of 1 that has to stay 1 through many subdivisions, the difference vanishes, whatever the width of `$\alpha$`. The second mentions each of `$a,b$` once, so it doesn't double-count wide control points, but splits `$\alpha$` into two uncorrelated factors, so it can't see the cancellation. Neither dominates. So the kernel computes both, and returns their intersection (see below). `AGENTS.md` recounts this as the fix to the first root cause of a real `curve_curve_intersect` blowup.

**Mean value form.** By the mean value theorem, for any `$x,c\in X$`, there is a `$\xi$` between them with `$f(x)=f(c)+f'(\xi)(x-c)$`. Thus
`$$f(X)\subseteq F_{mv}(X)=f(c)+F'(X)\,(X-c).$$`
Here the variable occurs in only two places, and `$X-c$` is small and centred about zero. Worked example, for `$f=x(1-x)$`, `$X=[0.4,0.6]$`, `$c=0.5$`:

- `$f(0.5)=0.25$`
- `$F'(X)=1-2X=[-0.2,0.2]$`
- `$X-c=[-0.1,0.1]$`
- So `$F_{mv}=0.25+[-0.02,0.02]=[0.23,0.27]$`, width `$0.04$`

The true range is `$[0.24,0.25]$`, width `$0.01$`, and naive evaluation gives `$[0.4,0.6]\cdot[0.4,0.6]=[0.16,0.36]$`, width `$0.2$`.

How do they converge? Halve `$X$` to `$[0.45,0.55]$`. Mean value gives `$0.25\pm0.005$`: width `$0.01$`, true width `$0.0025$`, excess `$0.0075$`, a quarter of what it was (`$0.03$`). Naive gives `$[0.2025,0.3025]$`: width `$0.1$`, excess `$0.0975$`, about half of what it was (`$0.19$`). In general:

- naive interval evaluation has an excess width **linear** in the width of the input;
- the mean value form has an excess **quadratic** in the width of the input.

This is why `NurbSurface3D::evaluate` in `core/geop-core-geometry/src/nurb_surface/evaluate.rs` intersects its direct enclosure with the mean value form ("the middle plus the partials over the box times the half widths"), as told in the section "A bound is as local as what it bounds" of `AGENTS.md`.

**Subdivide.** Split `$X$` into pieces, evaluate on each, and take the union. For `$X=[0.4,0.6]$` in two pieces: `$[0.4,0.5]\cdot[0.5,0.6]=[0.2,0.3]$` and `$[0.5,0.6]\cdot[0.4,0.5]=[0.2,0.3]$`, so the union is `$[0.2,0.3]$`, half as wide as naive. Doubling the pieces halves the excess. It is expensive when used alone, but subdivision has another virtue: where the pieces are far from a solution, they can be thrown away entirely. That is Part III.

Rationals show the dependency problem in an especially costly form. Take `$x/(1+x)$` on `$[1000,1001]$` (the demo has this function, "x / (1 + x)", over smaller values). Numerator and denominator move together, but the naive formula pairs the smallest numerator with the largest denominator: `$[1000/1002,\;1001/1001]=[0.99800,1.0]$`, when the true range is `$[0.999001,0.999002]$`. A NURBS curve evaluates to exactly such a ratio, `$A/W$`, and its width then scales with the distance from the origin. [Lecture 11](./11-evaluate-locally.md) is entirely about this one effect.

### Union and intersection

An interval is a set, so it has lattice operations. `Scalar::union` is the smallest interval containing both. `Scalar::intersect` is their overlap. Be aware of what the kernel's `intersect` does when they do not overlap: it returns the narrower input rather than an inverted interval. Its documentation says that callers must only intersect two enclosures of the same exact value. They then *must* overlap unless something has gone wrong.

Think about when each is right:

- Two enclosures of the **same value**, coming from different formulas: the truth is in both. So it is in the intersection, which is tighter than either. That is what `Scalar::interpolate` does with its two forms.
- A quantity that **may be either of two things**, or one value with two estimates that must both be covered: the union.
- Averaging: never. [Lecture 5](./05-honest-numerics.md) explains.

### The wrapping effect

The dependency problem has a geometric sibling. A point in space is a vector of three intervals, that is a **box** aligned with the axes. Rotate that box by 45 degrees. The true set is the same box, turned. But the only thing a vector of intervals can represent is an axis-aligned box, and the smallest one around a turned unit square is `$\sqrt2$` times as wide. Repeat this, and the width multiplies by `$\sqrt2$` per step, even though the true set never grows.

This effect is why interval methods work best with computations that aren't long chains of transformations, and why construction code in a kernel thinks carefully about which frame to compute in. When growth of a width looks mysterious, check for wrapping.

<div class="kernel">

**In the kernel.** `core/geop-core-math/src/scalars/mod.rs` is short and worth reading in full. A few things to take from it:

- `Scalar` is the single, concrete number type. The commit "One scalar: the interval type is concrete, the trait is gone" removed an earlier abstraction over number types. Some stories in `AGENTS.md` mention runs "in fixed point"; today's kernel computes in intervals of `f64` alone.
- The trait `Number` in `core/geop-core-math/src/scalars/number.rs` is what a residual of the constraint solvers is written in: either a `Scalar`, or a `Dual` (`core/geop-core-math/src/dual.rs`) carrying derivatives. Thus one formula gives both the value and its Jacobian, in interval arithmetic ([Lecture 23](./23-sketch-solver.md)).
- `Display` for a `Scalar` prints the midpoint to three decimals. This is for humans, and it hides exactly what you need when debugging. `Debug` prints both bounds. When an error's cause needs the numbers, use `{:?}`.
- A kernel-wide bound on how much uncertainty an entity may carry exists: `MAX_WIDTH`, `$10^{-4}$`, in `core/geop-core-topology/src/validation/numerical_accuracy.rs`. Its comment says "Not a comparison tolerance — nothing is ever tested for equality against it." It is a budget, and exceeding it means a bug upstream.

</div>

<div class="recap">

## Recap

- An enclosure is a pair of floats proven to bracket the true value. Width measures what is not known; a wide enclosure is still true.
- By the inclusion theorem, an interval evaluation holds for every point in the input box at once, so a single evaluation can prove a whole region empty.
- Outward rounding keeps that true in floats. `Scalar` steps each result one float outward, which also widens exact operations, so construction code avoids needless ones.
- Comparisons are three-valued; undecided is an answer you must handle.
- The dependency problem makes expressions with repeated variables wider than need be. Rewrite them, use the mean value form (quadratic rather than linear excess), or subdivide.
- Intersect two enclosures of the same value; union to cover alternatives; never average.

</div>

## Exercises

**1.** Evaluate by hand: `$[-2,3]\cdot[-1,4]$`, `$[1,2]/[2,4]$`, `$[1,2]^2$` as a product, and `$[-1,2]^2$` as a product. What is the true range of the last, and how would you write a proper "square" operation?

<details class="solution"><summary>Solution</summary>

`$[-8,12]$`; `$[0.25,1]$`; `$[1,4]$`; `$[-2,4]$`. The true range of `$x^2$` over `$[-1,2]$` is `$[0,4]$`. The product form takes `$-1\cdot2$` as if the two factors were independent. A square knows they aren't: `$[0,\max(a^2,b^2)]$` if `$a\le0\le b$`, otherwise the squares of the ends, ordered.

</details>

**2.** Prove inclusion for `$f(x,y)=(x+y)x$`, by following the theorem's induction for `$X=[0,1]$`, `$Y=[1,2]$`. Compare with the true range. Then give an expression for the same function with less overestimation on that box.

<details class="solution"><summary>Solution</summary>

`$X+Y=[1,3]$`, times `$X=[0,1]$` gives `$[0,3]$`. The true range: `$x^2+xy$` increases in both on this box, so `$[0,3]$`. No overestimation here at all, even though `$x$` occurs twice, since the expression is monotone in each occurrence. A repeated variable only hurts when its occurrences pull in opposite directions, as they do in `$x(1-x)$`.

</details>

**3.** Use the mean value form to enclose `$f(x)=x^3-2x$` on `$[0.9,1.1]$`, with `$c=1$`, and compare with naive evaluation and the true range. Use the demo to check.

<details class="solution"><summary>Solution</summary>

`$f(1)=-1$`. `$F'(X)=3X^2-2=[0.43,1.63]$`. `$X-c=[-0.1,0.1]$`. So `$F_{mv}=-1+[-0.163,0.163]=[-1.163,-0.837]$`. Naive: `$[0.729,1.331]-[1.8,2.2]=[-1.471,-0.469]$`. True: `$f$` has a minimum at `$\sqrt{2/3}\approx0.816$`, outside the interval, so it is monotone there: `$[f(0.9),f(1.1)]=[-1.071,-0.869]$`. In the demo, pick "x^3 - 2x", centre 1, half width 0.1.

</details>

**4.** Why does `Scalar::interpolate` return `a` directly if `a.is_exactly(b)` and `alpha` lies in `$[0,1]$`? Why isn't computing `a + alpha*(b - a)` enough? (Read the comment in the code.) What's the risk if this were not done across forty repeated subdivisions?

<details class="solution"><summary>Solution</summary>

Because `b - a` is rounded outward even where it is exactly zero, so it's `$[-5\times10^{-324},5\times10^{-324}]$` for a sharp value, or twice the width for a wide one, then `a` plus that widens `a`. Yet every point between two enclosures of equal range lies in that range, so `a` itself is an enclosure, with no widening. Without it, a weight that is exactly 1 gets a little wider with each subdivision: after enough cuts the patch is no longer recognized as polynomial, and the width compounds through every later split.

</details>

**5.** Enclose `$g(x)=(x+1)/(x+2)$` on `$[100,101]$` naively. Then rewrite it so that `$x$` occurs once and compare. What general trick for ratios is this?

<details class="solution"><summary>Solution</summary>

Naive: `$[101,102]/[102,103]=[0.98058,1.0]$`. Rewritten, `$g(x)=1-\frac{1}{x+2}$`, which gives `$1-[1/103,1/102]=[0.990196,0.990291]$`, the exact range. The trick is to pull out a nearby constant and keep the remainder small, which is what evaluating relative to a local point does ([Lecture 11](./11-evaluate-locally.md)).

</details>

**6.** A box `$[-1,1]^2$` is rotated by `$\theta=10°$` in an interval computation, 36 times, by the formula `$x'=x\cos\theta-y\sin\theta$`, `$y'=x\sin\theta+y\cos\theta$`, for a full turn. How wide is the result, ignoring rounding? How could you rotate 36 times by `$10°$` *without* that?

<details class="solution"><summary>Solution</summary>

Per step the half-width is multiplied by `$\lvert\cos\theta\rvert+\lvert\sin\theta\rvert\approx1.1585$`. After 36 steps that's `$1.1585^{36}\approx 200$` times the original half-width, for a box that truly came back to where it started. To avoid it, compose the rotation matrix first, on sharp values, and apply one matrix once, to the original box. Or apply the transformation to the box's centre and handle the width separately. Rule: compute long chains on values with small or no width, and apply uncertainty once at the end.

</details>
