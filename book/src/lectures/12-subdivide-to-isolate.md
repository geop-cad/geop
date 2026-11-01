# Lecture 12: Subdivide to isolate

<div class="goals">

**After this lecture you can**

- state the two questions every geometric search in a kernel reduces to, *could this piece hold a solution?* and *is this piece small enough to stop?*;
- write a branch-and-bound search over pieces of a curve or a pair of curves, and argue that it can never lose a solution;
- explain why samples prove nothing, and what the control points prove instead;
- predict how many leaves a search leaves at a crossing, at a tangency and along an overlap, and say why those numbers are what they are;
- say what `max_nodes` and `min_subdivision_size` mean in the kernel, and why running out of budget has to be an error rather than an answer.

</div>

Part II gave us a representation of shape: a curve is a list of control points, a surface a grid of them. In Part III we ask that representation questions. Does this edge cross that face? Where? Does this curve pass through this point? Almost everything a boolean or a fillet does comes down to these.

They are hard because a curve has infinitely many points. To say "these two curves do not meet" we have to make a statement about all of them, and a program that runs for a finite time has to make it from finitely many numbers. This lecture shows how, using nothing beyond the convex hull property of [Lecture 6](./06-bezier.md) and the splitting of [Lecture 9](./09-subdivision.md).

## Why not just sample?

The obvious algorithm takes 100 points along each curve and looks for a pair that is close. It has two faults, and each one is fatal.

**It cannot prove absence.** Suppose the closest pair of samples is 0.01 apart. That says nothing about what lies between the samples. Two curves can cross and leave again between two neighbouring samples, and no number of samples rules that out.

**It cannot say what "close" means.** To decide that a pair is a crossing, it needs a threshold. Too small a threshold misses crossings that the sample spacing skipped. Too large a threshold reports curves that pass near each other. The threshold is an epsilon by another name, and [Lecture 5](./05-honest-numerics.md) explained why those are not allowed.

The kernel's rule is blunt: *a fixed number of samples along a curve or over a surface proves nothing about what lies between them* (AGENTS.md, "Do not decide from samples"). The rule is there because the code broke it. As AGENTS.md records, which way a loop runs was once decided from a polygon of eight points per pcurve, in five places with five sample counts, and a nearest point was found as the best of a few samples per span refined by Newton, so that an interference check could miss two faces touching. Both now come from the representation itself: the first from an integral (`nurb_curve::loop_signed_area`), the second from the search we study at the end of this lecture.

What we need is a test that talks about a whole piece at once.

## The test: a box around a piece

Recall that a Bézier curve lies in the convex hull of its control points, and that the hull of a *piece* of the curve is found by splitting (the same holds for a NURBS curve with positive weights, [Lecture 8](./08-nurbs.md)). A convex hull is awkward to compute with, so take the coarser and cheaper thing, the axis-aligned box of the control points. Then:

> If the boxes of two pieces are disjoint, the two pieces do not meet.

Read how much this says. It holds for all the infinitely many points of both pieces. It uses `$O(n)$` numbers. And with interval coordinates it remains a proof: two boxes are "definitely disjoint" only if the comparison says so, and rounding is already enclosed.

The converse is false. Overlapping boxes do not mean the pieces meet, only that we have not yet ruled it out. So the test has three possible readings: *no*, or *maybe*. There is no *yes*, which is why the kernel's containment functions are named `curve_could_contain` and `surface_could_contain`. The word "could" is part of the contract.

This is what splitting is for. A "maybe" is resolved by cutting the piece in two and asking each half again. The hull of a piece hugs the piece more closely the shorter it gets: for a piece of parameter width `$h$`, the distance from the curve to the chord between its ends is `$O(h^2)$`. So every "maybe" that is really a "no" turns into a "no" after a finite number of cuts, provided the two curves are a positive distance apart there.

## Branch and bound

Put together, this is one of the oldest ideas in numerical search.

```text
pending = [ the whole problem ]
while pending is not empty:
    take a piece from pending
    if the piece provably holds no solution:   drop it
    elif the piece is small enough to stop:    report it
    else:                                      cut it, add both halves to pending
```

The search's central property is easy to prove: **no solution is ever lost**. A solution lies in the root piece. If it lies in a piece, that piece is not dropped (since dropping requires a proof that no solution lies there), and on cutting, the solution lies in one of the halves. So at the end every solution lies in a reported piece.

Note what is *not* promised. A reported piece need not hold a solution. It is a *candidate*. The kernel says so in as many words: in `curve_curve_crossings` a pair that converges is "a *candidate*, not an existence proof".

For a pair of curves the "piece" is a pair of pieces, and cutting means halving one of the two. Which one? A free choice, and as AGENTS.md warns, a free choice still has to be a good one: cut the one whose box is larger, since parameter widths from differently scaled curves are not comparable.

<figure class="demo" data-demo="subdivide-isolate">
<figcaption>Step through the levels of the search. Boxes shown are the pieces still alive, blue for one curve and orange for the other; a red stretch is a cluster of leaves. Try the scenes, drag the control points, and vary the handoff. This demo tests boxes only and halves the larger piece; the kernel also clips each pair (<a href="./13-fat-lines.md">Lecture 13</a>), which changes the counts but not what they mean.</figcaption>
</figure>

Run the "three crossings" scene level by level. For the first few levels, nothing is rejected, because the boxes are large and overlap anyway. Then the pairs near no crossing start to fall away, and the alive set collapses onto three spots.

### Counting the work

How many nodes does this cost? Three cases are worth doing carefully, since they explain much of what follows in Part III. All numbers in this section come from the demo's algorithm, run at decreasing handoff sizes `$h$` (the test `book/scripts/demo_tests/subdivide-isolate.test.mjs` checks the trends).

**A transversal crossing.** Near a crossing at a good angle, two pieces of size `$h$` have overlapping boxes only if they lie within about `$h$` of the crossing. So at every level a bounded number of pairs survive, regardless of how deep we are. Cost: a constant number of nodes per level, or `$O(\log(1/h))$` overall.

| handoff `$h$` | nodes | leaves | clusters | width of the widest cluster in `$s$` |
|---|---|---|---|---|
| 0.08 | 143 | 7 | 3 | `$1.6\times10^{-2}$` |
| 0.02 | 199 | 7 | 3 | `$3.9\times10^{-3}$` |
| 0.005 | 253 | 7 | 3 | `$9.8\times10^{-4}$` |
| 0.00125 | 303 | 6 | 3 | `$2.4\times10^{-4}$` |

Going 64 times finer took two times the nodes, and the leaves stayed constant. The width of what is reported fell in proportion to `$h$`.

**A tangency.** Now let the curves touch. Near the contact they separate as `$\tfrac12\kappa x^2$`, where `$x$` is the distance along the common tangent and `$\kappa$` the difference of the two curvatures. A box of a piece of size `$h$` running obliquely across the axes has a thickness of about `$h$`, so two such boxes overlap until `$\kappa x^2\approx h$`, which is at `$x\approx\sqrt{h/\kappa}$`. Surviving leaves therefore cover a stretch of length `$\sqrt{h}$`, using pieces of length `$h$`, so there are `$\sqrt{h}/h=h^{-1/2}$` of them.

| handoff `$h$` | nodes | leaves | clusters | width of the cluster in `$s$` |
|---|---|---|---|---|
| 0.08 | 109 | 8 | 1 | `$1.1\times10^{-1}$` |
| 0.02 | 217 | 16 | 1 | `$5.9\times10^{-2}$` |
| 0.005 | 421 | 30 | 1 | `$2.8\times10^{-2}$` |
| 0.00125 | 811 | 60 | 1 | `$1.4\times10^{-2}$` |

Each quadrupling of the resolution doubles the leaves and only halves the width: the signature of `$\sqrt{h}$`. A tangency is hard not because anything breaks, but because the data only determines the contact to the square root of its precision. [Lecture 15](./15-tangency-and-overlap.md) takes this up. (The preset is tilted on purpose. If the common tangent runs along a coordinate axis, a box's thickness across that axis is small near the contact, and leaves stay bounded. A search's cost depends on how the problem sits in its coordinates. The kernel's clip, too, measures across the other curve's chord rather than along the axes, as [Lecture 13](./13-fat-lines.md) explains.)

**An overlap.** If the two curves share an arc, *every* pair of pieces along the arc that are near each other survives, at all depths. Then `$O(1/h)$` leaves remain:

| handoff `$h$` | nodes | leaves | clusters |
|---|---|---|---|
| 0.08 | 629 | 130 | 1 |
| 0.02 | 2529 | 513 | 1 |
| 0.005 | 10079 | 2053 | 1 |
| 0.00125 | 40277 | 8187 | 1 |

Leaves double per halving. Follow this to a kernel-sized handoff of `$10^{-7}$` and the budget is gone. An overlap cannot be solved by search alone, and it cannot be recognised by counting either: read on.

## Stopping: the handoff

When should a piece count as "small enough"? The naive answer is "when it is as narrow as I want the answer to be". Subdivision gains one bit per cut, so a parameter accurate to machine precision would take about fifty levels. AGENTS.md puts the comparison plainly: about 50 levels against the roughly 7 that isolate a solution.

The better answer is that subdivision and accuracy are separate concerns.

- **Subdivision is the global method.** It finds every solution, separates them, and its proof of absence is the thing nothing else provides. Its convergence is linear and slow.
- **Newton is the local method.** It cannot find anything, but given an isolated solution it doubles the number of correct digits at every step ([Lecture 14](./14-newton.md)).

So `min_subdivision_size` is a *handoff threshold*, not an accuracy target: subdivide until the solutions are separated, then refine each one. In AGENTS.md's words, the tolerance then stops influencing the final accuracy at all, which is a stronger form of the epsilon rule than making the search itself tolerance-free.

There is a second, subtler reason not to run subdivision to the bottom. At a very fine handoff, a transversal crossing surrounded by numerical width also leaves many leaves, so any signal one might read from the number of leaves disappears.

### Width: where a search must stop anyway

A coordinate of the kernel is an interval. Its width is a statement about how well that coordinate is known. Cutting a piece halves its *extent*, the part of its box that comes from the curve actually going somewhere, but it does nothing about the *width* the control points carry. Once a piece's extent is below its own width, further cuts multiply survivors without telling us anything new.

The kernel's searches take this into account. In `core/geop-core-geometry/src/fat_line.rs`, `stalled` declares a box converged once every spatial `extent` is no longer `definitely_greater` than the larger of `min_subdivision_size` and `carried_width`, the width the data carries. Its documentation says why, and it is the right way to think about it: "resolving one object finer than another's width answers nothing new — every piece within that width stays a candidate — and only multiplies them".

## Budgets, and why running out is an error

Every search has a second tunable: `max_nodes`, the number of pieces it may look at. What should a search return when it runs out?

The tempting answers are all wrong. "No solution" is wrong, because a solution may be in the pieces not yet visited. "These solutions" is wrong for the same reason: there may be more. An incomplete search has established neither, and a downstream decision built on either would be wrong in a way nothing can detect.

So the kernel gives the search a way to say so. `core/geop-core-math/src/solvers/subdivide.rs` is the whole of the driver, under a hundred lines:

```rust,ignore
pub enum Split<T> {
    Done,           // nothing more to search in it
    Restrict(T),    // the piece narrowed to where its solutions can be
    Halves(T, T),   // the piece cut in two, both to be searched
}

#[must_use]
pub enum Searched {
    Complete,
    Exhausted { pending: usize },
}

pub fn subdivide<T, E>(
    roots: impl IntoIterator<Item = T>,
    order: Order,
    max_nodes: usize,
    visit: impl FnMut(T, usize) -> Result<Split<T>, E>,
) -> Result<Searched, E>
```

This is branch and bound stripped to its skeleton. Which pieces hold nothing, when one is small enough and how to cut it are the caller's `visit`; the driver promises only completeness, or says otherwise. `Searched` is `#[must_use]`, and every caller turns `Exhausted` into an error. In `curve_curve_crossings`, for example:

```text
curve_curve_crossings: exhausted max_nodes=... with ... pairs pending; the result would be incomplete
```

This is the rule from AGENTS.md's "Never use epsilons": `max_nodes` and `min_subdivision_size` "are the only tunables allowed to affect *how hard the search tries*". Change either, and the running time changes, and how wide an answer is; what a converged answer *means* stays the same. Their defaults live in `core/geop-core-geometry/src/search.rs`: `MAX_NODES` is 20 000, `MIN_SUBDIVISION_SIZE` is `$10^{-7}$`. The boolean's `RemeshParams` (`lib/geop-lib-booleans/src/remesh/mod.rs`) gives its pairwise searches a looser `curve_curve_min_subdivision_size` of `$10^{-4}$`, as it documents, because a pairwise search empirically needs that to converge within its budget at all.

The default order is breadth first (`Order::BreadthFirst`), level by level. That spends the budget evenly over the region, so one difficult corner does not eat the budget before the rest has been looked at.

## The question of containment

The simplest search in the kernel has one curve and a point: *could this curve pass through this point, and at which parameters?* It is `curve_could_contain` in `core/geop-core-geometry/src/contains/curve.rs`, and `surface_could_contain` in `contains/surface.rs` is the same for a surface. Its signature is the whole contract:

```rust,ignore
pub fn curve_could_contain<const D: usize, const C: usize>(
    curve: &NurbCurve<D>,
    point: &Vector<C>,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Option<Scalar>>
```

- `Ok(None)`: a proof that the curve misses the point.
- `Ok(Some(t))`: an enclosure of every parameter at which it could pass through. If the curve runs through the point twice, `$t$` is the `union` of both places and is wide. It is a true statement, and the caller who needs them separately asks differently.
- `Err(..)`: the search ran out. No statement about the point.

Its derivation is written up next to the code, in `core/geop-core-geometry/src/contains/curve.md` and `contains/surface.md`, and it is worth reading after the next lecture. The visit function does what the skeleton above says, with one addition: a piece is not only rejected or cut, it is also *clipped*, narrowed directly to where its solutions can lie (`Split::Restrict`). This is the subject of [Lecture 13](./13-fat-lines.md).

Here is the first test `curve_could_contain` applies to each piece: "Cheap prefilter against the cached bounding box."

```rust,ignore
if !aabb_could_contain(&seg.aabb, point) {
    return Ok(Split::Done);
}
```

`seg.aabb` is the box of the control points the curve already carries (`compute_aabb`, in `core/geop-core-geometry/src/aabb.rs`). This is the box test of this lecture, in its plainest form, with a `could` in its name.

### A worked example

Take `$P_0=(0,0)$`, `$P_1=(1,2)$`, `$P_2=(3,2)$`, `$P_3=(4,0)$`, the cubic of Lecture 6, and ask whether it could pass through `$q=(2,1.7)$`.

The whole curve's box is `$[0,4]\times[0,2]$`, which contains `$q$`: maybe. Split at `$\tfrac12$`. The pieces, as computed in Lecture 6, have control points `$(0,0),(\tfrac12,1),(\tfrac54,\tfrac32),(2,\tfrac32)$` and `$(2,\tfrac32),(\tfrac{11}{4},\tfrac32),(\tfrac72,1),(4,0)$`. Their boxes are `$[0,2]\times[0,\tfrac32]$` and `$[2,4]\times[0,\tfrac32]$`. Since `$1.7>1.5$`, neither contains `$q$`. We have proved, with two splits' worth of arithmetic, that the curve misses `$q$`. That is a statement about all the curve's points. Notice that the original control polygon went up to `$y=2$`, so that one level of subdivision was what brought the bound below `$q$`.

## The nearest point: branch and bound with a bound on the answer

The same skeleton answers a different question, and seeing it in that form is instructive. *What is the least distance between two sets?* It is what `closest_point` and the interference check need, and it was the second of the two AGENTS.md stories above. The answer now comes from `least_distance` in `core/geop-core-geometry/src/nearest.rs`.

Branch and bound over pairs of pieces (a span of a curve, a patch of a surface, a point) keeps two bounds.

- A **lower bound** for each pair, from the hulls: the two pieces are at least this far apart.
- An **upper bound** on the answer overall: a point of each piece that is actually attained gives a real distance, and the best so far is what the answer is at most.

A pair whose lower bound is definitely above the best attained distance holds nothing nearer, and it is dropped; any other pair is cut, its larger piece in two. Pairs are taken least lower bound first, from a priority queue, which is why this search does not use `subdivide`: the order is part of the method, and when the pair at the head of the queue is beyond the best distance, so is everything waiting. The result is itself an enclosure of the distance: from the least lower bound still standing to the attained distance.

Two details are worth keeping.

**Which lower bound matters.** Axis-aligned boxes give a lower bound that is off by the size of the piece, since an oblique piece has a fat box. Measured instead *along the direction between the two pieces*, the hulls give a bound that is off by the square of the piece's size. "Near a minimum that keeps a handful of pairs alive at each level, rather than a number growing with every level", as the module doc says. This is the same contrast we just saw in the tangent preset: what a search survives depends on how tightly its bound sits around the geometry.

**The handoff again.** The distance is flat at its least, so the search pins the point attaining it only to the square root of the distance's precision. So here too subdivision isolates and Newton refines: `NurbSurface::closest_point` calls `least_distance` with `NEAREST_HANDOFF` (`$10^{-3}$` of the size of what it searches), then Newton from each surviving pair. The search's job is to prove that no basin was missed.

<div class="kernel">

**In the kernel.**

| What | Where |
|---|---|
| The driver | `subdivide`, `Split`, `Searched`, `Order` in `core/geop-core-math/src/solvers/subdivide.rs` |
| Default effort | `MAX_NODES`, `MIN_SUBDIVISION_SIZE`, `NEAREST_HANDOFF`, `PROJECT_ITERATIONS` in `core/geop-core-geometry/src/search.rs` |
| Could a curve or surface hold this point? | `curve_could_contain` in `core/geop-core-geometry/src/contains/curve.rs`, `surface_could_contain` in `core/geop-core-geometry/src/contains/surface.rs` |
| When a stalled box is done | `stalled`, `carried_width`, `extent` in `core/geop-core-geometry/src/fat_line.rs` |
| What an intersection returns | `Intersections` in `core/geop-core-geometry/src/intersection/mod.rs` |
| Least distance | `least_distance` in `core/geop-core-geometry/src/nearest.rs` |

`Intersections<T>` has two variants, `Found` and `Coincident`. `Found` holds the distinct isolated solutions. `Coincident` says the two overlap along a shared region, and carries the overlaps' ends, the isolated crossings, and points spread over the overlaps. Overlaps are not inferred from reaching a maximum number of solutions: see [Lecture 15](./15-tangency-and-overlap.md).

</div>

## Recap

<div class="recap">

- A search's job is to prove absence where it can and to report the rest as candidates. Samples can do neither.
- The box of a piece's control points is a finite proof about infinitely many points, and splitting tightens it.
- Branch and bound never loses a solution. It reports candidates, not existence.
- Leaves at a handoff `$h$`: bounded at a transversal crossing, `$h^{-1/2}$` at a tangency, `$h^{-1}$` along an overlap.
- `min_subdivision_size` is where subdivision hands over to Newton, and `max_nodes` is how hard to try. A search that runs out returns an error, never a partial answer.

</div>

## Exercises

**1.** In the demo, choose "three crossings", set the handoff to 0.04, and step to the last level. Then drag one control point until two crossings merge and vanish. How do the leaves and clusters behave just before the crossings merge?

<details class="solution"><summary>Hint</summary>

Just before they merge, the two crossings are close to a tangency, so the clusters grow wide and merge into one candidate well before the crossings have actually gone. That cluster may hold two crossings, or none. The search cannot say which, and it is honest about it: a candidate is not a proof.

</details>

**2.** Prove termination. Assume two polynomial Bézier curves are at a distance `$\delta>0$` from each other, and that the search halves the larger piece. Show that the search stops, and give an order of magnitude for its depth in terms of `$\delta$` and the curves' sizes.

<details class="solution"><summary>Solution</summary>

A piece of parameter width `$w$` has a box of size at most `$Mw$`, where `$M$` bounds the speed of the curve over its hodograph's hull. Once both pieces are smaller than `$\delta/\sqrt2$` in each direction, their boxes can overlap only if some of their points are closer than `$\delta$`, which they are not. So every pair is rejected once `$w<\delta/(\sqrt2 M)$`, that is after about `$\log_2(\sqrt2M/\delta)$` halvings of each curve, or twice that many levels.

</details>

**3.** A search reports a leaf whenever both pieces are below `$h$`, but it could equally test whether their *boxes* overlap by a little. Suppose a colleague proposes to drop a pair when the boxes overlap by less than `$10^{-9}$`, "since that can only be rounding". Give a concrete way in which this loses a real crossing.

<details class="solution"><summary>Solution</summary>

Two curves whose ends just touch: one ends exactly at a point on the other, with the boxes meeting in a thin strip because that curve only reaches the other at its end point. Their overlap can be arbitrarily thin at every level, while the crossing is real. Dropping that pair loses a vertex of the model, which surfaces far later as an open solid. A real doubt about rounding is already expressed as width in the interval coordinates; adding a second allowance is an epsilon.

</details>

**4.** Write out the search for `$p(t)=t^3-t+\tfrac14$` on `$[0,1]$` by hand, using the Bernstein coefficients as the range bound, cutting at the midpoint, down to width `$\tfrac18$`. How many roots does it find? Which intervals survive? (On `$[0,1]$` the coefficients are `$\tfrac14,-\tfrac1{12},-\tfrac5{12},\tfrac14$`.)

<details class="solution"><summary>Solution</summary>

The coefficients change sign, so `$[0,1]$` survives. Splitting it gives `$\tfrac14,\tfrac1{12},-\tfrac1{12},-\tfrac18$` on `$[0,\tfrac12]$` and `$-\tfrac18,-\tfrac16,-\tfrac1{12},\tfrac14$` on `$[\tfrac12,1]$`. Both survive. At the next level, `$[0,\tfrac14]$` has `$\tfrac14,\tfrac16,\tfrac1{12},\tfrac1{64}$`, all positive, and `$[\tfrac12,\tfrac34]$` has `$-\tfrac18,-\tfrac7{48},-\tfrac{13}{96},-\tfrac5{64}$`, all negative: both dropped. `$[\tfrac14,\tfrac12]$` has `$\tfrac1{64},-\tfrac5{96},-\tfrac5{48},-\tfrac18$` and `$[\tfrac34,1]$` has `$-\tfrac5{64},-\tfrac1{48},\tfrac1{12},\tfrac14$`: both survive. At width `$\tfrac18$`, `$[\tfrac38,\tfrac12]$` is all negative and `$[\tfrac78,1]$` all positive, so they are dropped, and `$[\tfrac14,\tfrac38]$` and `$[\tfrac34,\tfrac78]$` remain. Two candidates, and the roots are indeed in them, at `$0.26959\ldots$` and `$0.83757\ldots$`.

</details>

**5.** Why does `least_distance` not call `subdivide`? What would go wrong if its queue were first in, first out?

<details class="solution"><summary>Hint</summary>

Taken in order of lower bound, the search can stop as soon as the head of the queue is beyond the best attained distance, since all others are too. Breadth first has no such stopping rule: it has to look at every level's pairs near every local minimum, including ones that an attained distance elsewhere will later eliminate. The number of nodes is much higher, and the budget is spent on the wrong basins.

</details>

**6.** The "overlapping" preset is two pieces of one curve. Why can't a search in principle distinguish it from two curves that are `$10^{-30}$` apart? What can a kernel do instead?

<details class="solution"><summary>Hint</summary>

To a given precision of the data, both have the same control points. No search can reject a pair of pieces whose separation is below the width they carry. So an overlap is not something to read off the search's output. It is a claim to test directly, at the places where an overlap can begin and end: exactly what [Lecture 15](./15-tangency-and-overlap.md) does.

</details>
