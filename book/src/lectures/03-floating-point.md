# Lecture 3: Floating point

<div class="goals">

**After this lecture you can**

- describe how IEEE 754 doubles represent numbers, and compute the spacing (the *ulp*) of the numbers near any value;
- use the standard model of rounding to bound the error of a computation, and recognize when it fails badly (cancellation);
- explain why addition of floats is not associative, and why that order of operations changes answers;
- show by example why `(a - b).abs() < epsilon` has no correct value of epsilon;
- explain what goes wrong when a geometric predicate is computed in floating point, with the orientation test as evidence.

</div>

Geop stores coordinates in `f64`, IEEE 754 double precision. Before we can build the rest of Part I on top of it, we need to know exactly what those numbers are and what they do. Most programmers carry around a vague picture ("floats are slightly inaccurate"). That picture leads directly to the wrong fix, namely comparing with an epsilon. The right picture is much more precise, and much more useful.

## What a double is

A `f64` is 64 bits: one sign bit `$s$`, eleven exponent bits, fifty-two fraction bits. For all but the extremes, the bits stand for the number

`$$x = (-1)^s\left(1+\frac{f}{2^{52}}\right)2^{e},\qquad 0\le f<2^{52},\quad -1022\le e\le 1023.$$`

So a double is a 53-bit binary integer scaled by a power of two. It is *floating* in that where the binary point sits depends on the exponent. And it is **exact**: every bit pattern (besides NaN and infinities) denotes one precise rational number. There is nothing fuzzy about the number 1.9999999999999998 that you met in Lecture 2. It is exactly `$2-2^{-52}$`.

All numbers with one exponent `$e$` are equally spaced, `$2^{e-52}$` apart. This spacing is the **unit in the last place**, or **ulp**. When the exponent goes up by one, the spacing doubles:

- near 1, numbers are `$2^{-52}\approx2.2\times10^{-16}$` apart;
- near 1000, they are `$1.1\times10^{-13}$` apart;
- near `$10^5$`, `$1.5\times10^{-11}$` apart;
- near `$10^{16}$`, 2 apart. The number `$10^{16}+1$` does not exist.

So floats are a *ruler whose ticks spread out as you go away from zero*. Relative to the number's size the spacing is nearly constant: `$\mathrm{ulp}(x)/\lvert x\rvert$` is between `$2^{-53}$` and `$2^{-52}$`. Absolute spacing is not constant at all, and this will decide our question about epsilons. Below is the ruler itself, computed from the bits of the numbers in your browser.

<figure class="demo" data-demo="float-ruler">
<figcaption>The ticks are the actual floating point numbers next to <em>x</em>, placed exactly. Nothing is representable between them. Below, ulp(<em>x</em>) for values from <em>x</em>/16 to 16<em>x</em>: a staircase, doubling at each power of two. Jump to a power of two and look at the ticks to the left of <em>x</em>: they are half as far apart as the ones to the right. Compare the formats: the spacing in f32 is 2<sup>29</sup> times that of f64.</figcaption>
</figure>

At the two ends: the largest exponent gives about `$1.8\times10^{308}$`, beyond which an operation yields infinity. At the small end, *subnormal* numbers fill in the gap between `$2^{-1022}\approx2.2\times10^{-308}$` and zero, with fixed spacing `$2^{-1074}\approx5\times10^{-324}$`. Zero has a sign, and `-0.0 == 0.0`.

### Most decimals are not representable

Binary fractions terminate only for denominators that are powers of two. `$\tfrac1{10}$` in binary is `$0.000110011001100\ldots_2$`, repeating, so the source text `0.1` becomes the nearest double. To all its digits, these are the actual values:

```text
0.1 is   0.1000000000000000055511151231257827021181583404541015625
0.2 is   0.200000000000000011102230246251565404236316680908203125
0.3 is   0.299999999999999988897769753748434595763683319091796875
```

Now add the first two. The exact sum of those two doubles lies between two representable numbers, and rounds to the nearest one:

```text
0.1 + 0.2 is   0.3000000000000000444089209850062616169452667236328125
```

That is not the double called `0.3`, which is one ulp lower. This is why `0.1 + 0.2 == 0.3` is false, in every language. Nothing was wrong with the addition; it was as accurate as it can be. The inaccuracies were *both* there already: `0.1`, `0.2` and `0.3` are all slightly off, in different directions.

## The standard model of rounding

IEEE 754 demands that `+`, `-`, `*`, `/` and `sqrt` be **correctly rounded**: the result is what you would get by computing the exact real result, and then rounding it to the nearest float. This makes float arithmetic amenable to analysis. If `$\circ$` is one of those operations and `$\hat\circ$` its float version, then (barring overflow and underflow)

`$$x\mathbin{\hat\circ}y = (x\circ y)(1+\delta),\qquad \lvert\delta\rvert\le u=2^{-53}\approx1.1\times10^{-16}.$$`

The number `$u$` is called the **unit roundoff**. Each operation, taken alone, is nearly perfect, with a relative error of at most `$u$`. So where do bad answers come from? From what errors do when they meet later operations. Functions such as `sin` and `cos`, by contrast, come from the math library and are not guaranteed correctly rounded at all. This matters for later: the kernel's interval `sin` has to allow for it.

### Accumulation and order

Consider `$s_n=s_{n-1}+x_n$`. Each addition contributes a relative error up to `$u$` *of the running sum*. Summing `$n$` positive numbers therefore has an error bound growing like `$nu$` times the sum. The bound suggests the order matters: put the largest numbers last, so that small additions do not suffer rounding against a large running total. In the extreme, adding 1 to `$10^{16}$` changes nothing at all, because the result rounds back:

`$$(10^{16}+1)-10^{16}=0,\qquad(10^{16}-10^{16})+1=1.$$`

So **float addition is not associative**. Even `0.1 + 0.2 + 0.3` depends on the order: we computed `(0.1+0.2)+0.3 = 0.6000000000000001` and `0.1+(0.2+0.3) = 0.6`.

<figure class="demo" data-demo="float-lab" data-args='{"lab":"sum"}'>
<figcaption>The harmonic sum 1 + 1/2 + ... + 1/N, added largest term first (red) and smallest first (green), both measured against a compensated sum in f64 (Neumaier's). In f32, the difference is dramatic, and you can read the numbers below the plot. In f64 the effect has the same shape, but some nine orders of magnitude lower, close to where the reference itself stops being reliable.</figcaption>
</figure>

At `$N=10^6$`, the true value is `$14.39272672286\ldots$`. In f32, largest first gives `$14.3574$`, wrong in the fourth digit. Smallest first gives `$14.39265$`, right to five digits. The two runs have the same numbers and the same operations, and differ only in order. (This demo is a calculation you could repeat in any language; `book/scripts/demo_tests/float-lab.test.mjs` compares it with an independent one.)

For a kernel, the lesson goes beyond summing. **Two ways of computing the same value give different floats.** So an evaluation by one route is not `==` to an evaluation by another, even if there has been no mistake at all, and even if the formula says they are equal.

### Cancellation

The worst thing that happens to an honest computation is that it subtracts two nearly equal numbers. Each number has an error of perhaps one ulp. Subtraction itself is *exact* in this situation (a theorem by Sterbenz), but the leading digits vanish and what remains is mostly those errors, now promoted to the leading position. The relative error is no longer `$u$`, but `$u$` times how much the magnitude has shrunk.

As an example, take

`$$f(x)=\frac{1-\cos x}{x^2}.$$`

Its limit as `$x\to0$` is `$\tfrac12$`. Let us try it: at `$x=10^{-8}$`, `$\cos x$` is `$1-5\times10^{-17}$`, closer to 1 than half an ulp, so it rounds to 1. The numerator is zero: `$f(10^{-8})=0$`. At `$x=2\times10^{-8}$`, the result is `$0.5551$`. No digit is right.

By the half-angle identity, `$1-\cos x=2\sin^2(x/2)$`, so the same function is `$2\sin^2(x/2)/x^2$`. That formula has no subtraction, and it gives `$0.5$`, accurate to the last bit, at those same arguments.

<figure class="demo" data-demo="float-lab" data-args='{"lab":"cancel"}'>
<figcaption>The relative difference of the naive formula against the half-angle formula (red), as <em>x</em> shrinks, and of the half-angle formula against its Taylor series (green), which shows that it is the accurate one. Around <em>x</em> = 1e-4 half the digits are gone, by <em>x</em> = 1e-8 all of them. Note how noisy the red curve is: which digits survive depends on how the bits happen to fall.</figcaption>
</figure>

Rewriting a formula to avoid subtracting nearby numbers is a major technique. We will see it again in [Lecture 11](./11-evaluate-locally.md): a rational patch evaluated in world coordinates loses accuracy with its distance from the origin, and the fix is to subtract a nearby point *first*, where it can be done exactly, and add it back after.

## Why no epsilon is correct

Now to the central claim. Since computed values are inexact, a natural thought is: to test whether `$a$` equals `$b$`, test

```rust,ignore
(a - b).abs() < EPSILON
```

for some small constant. This looks like a reasonable engineering solution, and it is the source of much of the misery in geometric software. Let us see why, in three steps.

**There is no right size.** An epsilon has units, but the rounding error is relative to what was being computed and how. Take `EPSILON = 1e-9`.

- At coordinates near `$10^8$`, the ulp is `$1.5\times10^{-8}$`, greater than epsilon. The test is the same as `a == b`, and any rounding error at all makes it fail.
- A value that accumulated a thousand operations at coordinates near `$10^5$`, where the ulp is `$1.5\times10^{-11}$` and errors can reach `$10^{-8}$`, fails the test though the true values are equal.
- For a part measured in nanometres, `$10^{-9}$` could be most of a feature.

Use a relative epsilon, `$\lvert a-b\rvert<\varepsilon\max(\lvert a\rvert,\lvert b\rvert)$`? It is then useless near zero, where `$10^{-20}$` and `$-10^{-20}$` are "different" for any `$\varepsilon<2$`, though both are obviously rounding noise. In the file of Lecture 2, a point's `x` was stored as `$9.857\times10^{-19}$`, standing for 0. And the magnitude of `$a$` tells nothing about the size of the error anyway: `$a$` could be the small difference of two huge numbers.

The kernel's own experience makes the point. As `AGENTS.md` records, a STEP edge lay `$2\times10^{-31}$` above its plane face, and that mattered. At the other end, a quarter arc cut forty times once came out `$0.08$` wide. No single constant is both smaller than the first and larger than the second.

**It is not transitive.** Equality has a property you rely upon without saying so: `$a=b$` and `$b=c$` imply `$a=c$`. With an epsilon they do not. With `EPSILON = 1e-9`, take `$a=0$`, `$b=0.9\times10^{-9}$`, `$c=1.8\times10^{-9}$`. Then `a` "equals" `b`, `b` "equals" `c`, and `a` does not "equal" `c`.

So suppose three vertices with those coordinates lie along an edge. A boolean merges coincident vertices. If it merges `a` with `b`, then `b`'s neighbour `c` is no longer within epsilon of the merged vertex, or is within epsilon, depending on which one is kept. Thus the topology depends on the order of the merges, and the order came out of a hash map. This failure cannot be fixed with a better number.

**It conflates two different questions.** "Are these the same?" has three honest answers: definitely yes (exact), definitely no, and *I cannot tell at the accuracy I have*. An epsilon test collapses the third answer into one of the first two, without telling you. Any later code therefore acts on a belief it has no grounds for. That loss is the real harm, and [Lecture 4](./04-intervals.md) is about not losing it.

## Geometry on floats: the orientation test

Here is the classic demonstration that this is not just about equality. The most basic predicate of computational geometry asks on which side of the line through `$q$` and `$r$` the point `$p$` lies. The answer is the sign of a determinant:

`$$\operatorname{orient}(p,q,r)=\operatorname{sign}\bigl((q_x-p_x)(r_y-p_y)-(q_y-p_y)(r_x-p_x)\bigr).$$`

It takes two multiplications and five subtractions. If `$p$`, `$q$` and `$r$` are nearly collinear, the two products are nearly equal, and cancellation strikes. Kettner, Mehlhorn, Pion, Schirra and Yap collected examples of what follows in *Classroom examples of robustness problems in geometric computations*, from which the configuration below is taken.

Fix `$q=(12,12)$` and `$r=(24,24)$`, and let `$p$` take each of the `$256\times256$` values `$(0.5+i\,\mathrm{ulp},\,0.5+j\,\mathrm{ulp})$`, with `$\mathrm{ulp}=2^{-53}$`. The true line is `$y=x$`. Thus the true sign is the sign of `$j-i$`: the diagonal is collinear, and each side has its own sign. To be careful, we compute the exact sign too, in rational arithmetic, using `BigInt`.

<figure class="demo" data-demo="orient-grid">
<figcaption>Each square is one point <em>p</em>, one ulp from its neighbours. Choose "sign in f64" and see the computed sign (blue positive, orange negative, grey zero), then "exact sign" for the truth, then "where f64 is wrong". The last option, "interval verdict", is the promise of the next lecture: outward-rounded intervals never give a wrong sign. So close to the line, they say "cannot tell" almost everywhere (grey): 64 016 of the 65 536 points of the largest grid. That is the honest answer at this precision.</figcaption>
</figure>

What you are looking at is the geometry of a line, as seen by floating point: a jagged mosaic, with blocks of wrong signs far from the true diagonal. We counted the failures: on the `$16\times16$` grid, 114 of 256 points get the wrong sign, or a false "collinear". On the `$256\times256$` grid it is 11 972 of 65 536. Note also that it can't be fixed by a threshold on the determinant. Any threshold just turns some of the wrong signs into "zero", and it also turns right signs into "zero".

Why does this break algorithms and not merely give small errors? Because a geometric algorithm builds on such predicates as axioms. A convex hull algorithm relies on the fact that if `$p$` is left of `$qr$` and `$r$` is left of `$pq$`, certain other things follow. When the predicates are inconsistent, the algorithm's invariants fail. The cited paper shows incremental convex hull algorithms that then lose points or produce a polygon that isn't convex. The output isn't a slightly wrong hull; it is not a hull.

## What can be done about it

There are three broad strategies, and each CAD kernel is a compromise among them.

1. **Exact arithmetic.** Rational numbers or algebraic numbers keep each comparison exact. Shewchuk's adaptive predicates make this efficient for determinants of points. But coordinates of exact intersection points of NURBS are algebraic numbers of very high degree. `DEVELOPERS.md` records that Geop 2 tried algebra, "ideals, Groebner bases", and found it "too slow and too much overhead".

2. **Tolerances.** Decide that things within `$\varepsilon$` are the same. A kernel stores a tolerance with each entity and tries to keep the decisions consistent. Most classical kernels do so, and years of engineering have made them work remarkably often, at the cost of the problems shown above. `docs/smlib-comparison/README.md` describes SMLib as "a mature NURBS B-rep kernel built on tolerances".

3. **Enclosures.** Keep the inexact numbers, but compute along with every value a *rigorous bound* on how inexact it may be, and let comparisons say when they can't tell. This is Geop's approach, and the next lecture.

<div class="kernel">

**In the kernel.** The bit-level "next number" operations are `next_up` and `next_down` in `core/geop-core-math/src/scalars/mod.rs`, which add or subtract one in the bit pattern, as `float-ruler` above does. A comment there is a little story of its own about subnormals: zero once stepped to `MIN_POSITIVE`, about `$2.2\times10^{-308}$`, so every value computed near zero carried that much width, while values a subnormal apart were sharp, and "a point between them could not be told on or off the side". It now steps to the smallest subnormal, `$5\times10^{-324}$`.

Plain `f64` still occurs in the kernel, "for what is only ever a free choice or a picture", "never for a result the kernel reasons about": see the module comment of `core/geop-core-math/src/plain.rs`.

</div>

<div class="recap">

## Recap

- A double is an exact rational: a 53-bit integer times a power of two. The ticks of this ruler spread out with distance from zero: the ulp is relative to the number's size.
- Each basic operation is correctly rounded, with relative error at most `$u=2^{-53}$`. Trouble comes from what happens next: accumulation, order dependence, and above all cancellation.
- Two routes to one value give different floats, so `==` is meaningless on computed values.
- `(a - b).abs() < EPSILON` has no right size, is not transitive, and silently replaces "cannot tell" by a guess.
- A predicate evaluated in floats can give wrong signs in a mosaic pattern, which breaks an algorithm's logic rather than merely blurring its output.

</div>

## Exercises

**1.** Show that `0.1` in binary is `$0.0\overline{0011}_2$`. What is `$\mathrm{ulp}(0.1)$`? Check the actual double's error from the text's expansion against half of it.

<details class="solution"><summary>Solution</summary>

`$\tfrac1{10}=\tfrac{3}{32}\cdot\tfrac{16}{15}=\tfrac{3}{32}(1+\tfrac1{16}+\tfrac1{256}+\dots)$`, and `$3/32=0.00011_2$` then repeats every four digits. Since `$2^{-4}\le0.1<2^{-3}$`, the exponent is `$-4$` and the ulp is `$2^{-56}\approx1.39\times10^{-17}$`. The double's error is `$5.55\times10^{-18}$`, which is less than half the ulp, `$6.94\times10^{-18}$`, as nearest rounding requires.

</details>

**2.** What are the following in `f64`, and why? (a) `1e16 + 1.0 - 1e16` (b) `1e16 - 1e16 + 1.0` (c) `1.0 + 1e-16 == 1.0` (d) `1.0 - 1e-16 == 1.0`.

<details class="solution"><summary>Solution</summary>

(a) 0: the ulp at `$10^{16}$` is 2 and the tie rounds to the even mantissa, here `$10^{16}$` itself. (b) 1. (c) true: `$10^{-16}$` is less than half of `$\mathrm{ulp}(1)=2.2\times10^{-16}$`. (d) false: *below* 1, the ulp is `$1.1\times10^{-16}$`, half of what it is above. So `$1-10^{-16}$` rounds to `$1-2^{-53}$`. Move the slider to a power of two in the ruler demo and you see this asymmetry.

</details>

**3.** The smaller root of `$x^2-10^8x+1=0$` is about `$10^{-8}$`. Compute it by the textbook formula `$\frac{b-\sqrt{b^2-4c}}{2}$` with `$b=10^8$`, `$c=1$`. Then derive a formula with no cancellation. How many digits does each give?

<details class="solution"><summary>Solution</summary>

`$b^2-4$` rounds to about `$10^{16}$`, and its square root is `$99999999.99999999$`, one ulp (`$1.49\times10^{-8}$`) below `$b$`. Half their difference is `$7.45\times10^{-9}$`, which has no correct digits. Multiplying numerator and denominator by the conjugate gives `$\frac{2c}{b+\sqrt{b^2-4c}}$`, which has no subtraction of close numbers, and gives `$10^{-8}$` to full accuracy.

</details>

**4.** A program merges vertices closer than `$10^{-6}$`, and processes vertices in the order given. Give four vertices on a line such that the result depends on the order in which they are processed. How would you make it independent of order? What would your fix do on a long chain of vertices each `$0.9\times10^{-6}$` from the next?

<details class="solution"><summary>Solution</summary>

Take `$0,\;0.9,\;1.8,\;2.7$`, in units of `$10^{-6}$`. Merging in pairs from the left gives two clusters, `$\{0,0.9\}$` and `$\{1.8,2.7\}$`; if the second and third get merged first, at their midpoint `$1.35$`, then the first and last are too far from it and stand alone, and you get three. An order-independent rule is the transitive closure: all connected vertices merge. But then a long chain collapses into one vertex, though its ends are a long way apart. Each repair exchanges one wrong outcome for another, since the underlying relation is not an equivalence.

</details>

**5.** Change the grid in the orientation demo to `q=(3,3), r=(100,100)` and to `p=(0.1,0.1), q=(1.7,1.7), r=(2.9,2.9)`. Is the mosaic still there? Is the true line still the diagonal, in the second case? (Careful: the double called `1.7` is not exactly 1.7.)

<details class="solution"><summary>Hint</summary>

Look at "exact sign". In the third configuration the three points are not exactly collinear, even though the source text suggests so, because the decimals were rounded. So the boundary of the true sign does not run exactly along the diagonal of the grid. This is itself a warning about tests written in decimal.

</details>

**6.** A surface is evaluated at a parameter, in two ways, giving `$z_1$` and `$z_2$`, both within a few ulps of the same value. Suppose your code selects the larger for one purpose and the smaller for another. Could the topology you build depend on the platform's math library? On the compiler's optimization level, on hardware with fused multiply-add? What does that imply for a test suite comparing files between machines?

<details class="solution"><summary>Hint</summary>

Yes to the first. Rust does not contract `a*b+c` into a fused multiply-add on its own, so basic arithmetic gives the same bits, but `sin`, `cos` and others come from libm and may differ by platform. If a decision hangs on those last bits, then the topology can differ between a native build and WebAssembly. Enclosures that are wide enough to cover those differences make decisions independent of them, as long as the decision is "definitely"; "could be" cases have to be handled by logic that does not depend on the exact bits.

</details>
