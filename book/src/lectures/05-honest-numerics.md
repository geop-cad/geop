# Lecture 5: The rules of honest numerics

<div class="goals">

**After this lecture you can**

- say what may be tuned in a search and what may not, and why that excludes epsilons;
- choose correctly between `union` and `intersect`, and say why an average is never right;
- decide whether a given use of `sharpen` is legitimate, and write the "two copies" loop;
- recognize the traps: validating a wider value than you use, asking a box a question only a point can answer, taking what is near for what belongs;
- trace a numerical failure to a specific line of arithmetic, and leave the diagnostic in the code.

</div>

Intervals give us a number that does not lie. But a program can still make it lie: narrow an enclosure past what was established, compare it with a fudge factor, or ask it a question whose answer doesn't apply to it. This lecture is a list of rules, each with the bug that taught it. They come from the section headings of `AGENTS.md`, and the stories, and numbers, are from there. You'll meet most of the code involved again later, in its proper place. What matters now is the reasoning.

The rules boil down to one sentence:

> We are always seeking the mathematical truth: bounds should be as tight as they can legitimately be made, but a bound must never be narrowed past what the computation actually established.

Keep that in mind. Dropping uncertainty does not make a result more accurate. It makes it *wrong in a way nothing downstream can detect*, and in a geometry kernel the concrete consequence is a missed intersection.

## Rule 1: never use an epsilon

No `(a - b).abs() < EPSILON`, and no new constant to paper over a comparison. [Lecture 3](./03-floating-point.md) showed why it cannot be right, and [Lecture 4](./04-intervals.md) what to write instead: `could_be_equal`, `definitely_greater`, and so on.

There are tunable numbers in the kernel, though. A search has a node budget (`max_nodes`) and a size below which it stops subdividing (`min_subdivision_size`). The distinction is stated like this: these numbers are allowed to affect *how hard the search tries*. More budget or a tighter tolerance changes the running time, and how quickly a search converges. It does not change what a converged answer means. A search that hasn't converged returns an error, `None` or a wide interval, never a value that is merely "close enough".

`RemeshParams` in `lib/geop-lib-booleans/src/remesh/mod.rs` collects all the numbers of a boolean. Read its comments: "Node budget for every subdivision search", "An effort budget only: a trace that reaches its vertex does so in the same steps whatever the budget."

So how do you tell a budget from an epsilon? Ask: *if I changed this number, could a correct result become incorrect?* If yes, it's an epsilon in disguise. And running out of budget has to be an error. [The geometry crate's page](../core/geop-core-geometry.md) says it plainly: "Running out of `max_nodes` is always an error, never read as 'not contained': the search is incomplete, so neither answer is justified."

There is one number that looks like a tolerance and isn't: `MAX_WIDTH` in `core/geop-core-topology/src/validation/numerical_accuracy.rs`, `$10^{-4}$`, the widest a coordinate of any stored entity may be. No comparison is made against it to decide geometry. It is a bound on how much uncertainty an entity may *carry*, because beyond it the searches can't locate anything. Past that, validation reports a bug, typically "a missing refinement".

If a result comes out wider than expected, the fix is to understand why, not to shrink a number until the symptom disappears. That is Rule 8.

## Rule 2: union to cover, intersect to constrain, never average

Suppose a point on an intersection curve of two surfaces is computed twice, once by projecting onto each surface. You get two enclosures of the same point:

`$$A=[1.000,1.002],\qquad B=[1.001,1.005].$$`

What should you do with them? There are three candidates.

- **Average** the midpoints: `$1.002$`. This is a sharp value that claims more precision than either input had, and discards the most useful signal in the pair: *how far apart the two answers were*. That gap is the honest measure of how much the computations disagree, and later convergence tests need to see it.
- **Union**: `$[1.000,1.005]$`. Every value either computation allows. It keeps the gap as width.
- **Intersection**: `$[1.001,1.002]$`. Every value both computations allow.

Which of the last two is correct depends on what you know:

**Union when you must cover both possibilities.** A point on a face × face curve, projected once onto each surface, is combined this way: the gap between the two projections is what later "have I arrived?" tests need to see. `merge_vertex` (`core/geop-core-topology/src/edit/merge_vertex.rs`) combines vertex points this way too.

**Intersect when both constraints must hold at once.** If each is a valid enclosure of the very same number, then the truth lies in both, so in the intersection, and we rightly keep less width. Example: `splice_edge_into_face` pins each end of a new loop to the arriving end's `$(u,v)$` intersected with the vertex's projection onto the surface. Using the end alone let the width of `interpolate_enclosing` spread around the loop until a face split came out degenerate.

**Average never.** It is not an enclosure of anything.

## Rule 3: sharpen only where the value is a free choice

`Scalar::sharpen` collapses an interval to its midpoint. Its documentation warns that it is for "when you are free to pick *any* value within `self`". It is the sharpest tool for violating honesty, and has a rule of its own. It is legitimate **only where any value inside the interval would serve equally well**, so that collapsing to one loses no mathematical accuracy. There are three such places:

1. **Subdivision parameters.** A search that splits an interval may split it anywhere; the split point is the algorithm's choice, not a claim about geometry. Sharpening here is necessary, too: repeatedly re-deriving a split point from a wide domain compounds width through knot insertion, with `$\alpha=(t-a)/(b-a)$`, a span divided by a shrinking span, until the search is useless.
2. **The predictor, before the corrector.** A predicted point is a seed: the corrector converges to the same fixed point from any seed nearby. See the comment "Sharpened exactly once, here" in `predictor_corrector_step` (`core/geop-core-geometry/src/intersection/branch.rs`).
3. **Where to cut an edge at a vertex, once refined.** Every parameter of the refined enclosure ends a piece on the curve and within the vertex. Cut at a sharp one and check it. Cutting at all of them, by contrast, makes each piece wider than its parent. `AGENTS.md` measured a quarter arc cut forty times going from `$3\times10^{-15}$` wide to `$0.08$`, where it now adds only one subdivision's rounding per cut (`NurbCurve::split_parameter_at_point`).

Everywhere else, sharpening a value that is an *answer* is a bug. The first big example: `NurbSurface::project`, which finds the point of a surface nearest a target, sharpened *every* Newton iterate, including the last. For all but the last that is correct, since they are only seeds. But the final step's `du`, `dv` inherit the target's width through the residual, and that width honestly states how precisely an uncertain target pins down a foot point. Sharpening returned one sharp `$(u,v)$` for a target that only determined a *range*, and every later comparison against anything else derived from the same uncertain point failed. Fixing that one thing took `validate_fast` failures across the scene sweep from 21 of 175 to 1 of 175.

### A sharpened answer lies to every interval test downstream

The worst of it is that the damage shows up three modules away. `predictor_corrector_step` sharpened its iterate at every Newton step, including the one it returned. A traced intersection curve running exactly along a face's trim boundary therefore came back with a *sharp* `$(u,v)$` sitting `$6\times10^{-17}$` off the boundary. So `face_contains` classified it `Inside`, since `could_be_equal` had no width to work with. The direction check whose job is to reject exactly such a curve waved it through. The curve was spliced in as a duplicate of the boundary, carving a zero-area sliver. Much later, this surfaced as a face that a boolean could not classify, because it had no interior point.

Notice what the sharpening did *not* do: it caused no geometric error. The point was where it should be, to within `$6\times10^{-17}$`. What it destroyed was the **statement of uncertainty**. A value that is honestly `$[0,1.2\times10^{-16}]$` answers "could this be on the boundary?" correctly. The same value sharpened to `$6\times10^{-17}$` answers it wrongly, with total confidence.

### Keep two copies

The fix generalizes. Whenever a loop needs sharp seeds but produces an answer, keep two copies:

- **the seed**, sharpened, which starts the next step. Sharpening here is a free choice, and without it, 20 Newton steps compound width until `evaluate` fails outright (measured at 36 of 175 scenes);
- **the answer**, the same update with its width as computed, which is what you return.

The returned width is that of a *single* step from a sharp seed, so it is bounded per step rather than compounded or thrown away. Here's the shape of it, from the loop in `NurbSurface::project` (`core/geop-core-geometry/src/nurb_surface/project.rs`):

```rust,ignore
|&(u, v)| (clamp((u.sharpen(), v.sharpen())), clamp((u, v))),
```

The pair is (seed, answer). `core/geop-core-math/src/solvers/iterate.rs` holds the loop that all of the kernel's Newton-type methods share, and its module comment summarizes the idea: "Which of the two a caller returns says whether its value is a free choice … or an answer that later interval tests compare against."

Now see it for yourself, in a real computation, with the kernel's own rounding. We solve `$(x+s)^3=c$` by Newton's method, in interval arithmetic, where `$c$` is computed as `$(b+s)^3$` for a given `$b$`. So the *true* solution is exactly `$b$`. Then we ask the three-valued question of `face_contains`: is the answer inside, outside, or on the boundary `$b$`?

<figure class="demo" data-demo="sharpen-trap">
<figcaption>Top: the width of each iterate. "Seed as it is" (orange) doubles each step until a divisor could be zero and the method fails. "Two copies" (green) stays at the width of one step. Bottom: the final answers near <em>b</em>, in a window of ±16 ulps. The red tick, "sharpen everything", has no width, and when it falls beside the boundary, the classification comes out with confidence and is wrong. The loop is a one-dimensional picture of what <code>NurbSurface::project</code> and <code>predictor_corrector_step</code> do, not the kernel's code.</figcaption>
</figure>

We ran this for `$b=0.7$`, `$s=0.3$`. The answer sharpened is `0.7000000000000001`, one ulp above `$b$`, and says "outside": wrong. Two copies give an interval `$1.3\times10^{-15}$` wide around it, which says "on boundary": right. Across a thousand values of `$b$` in `$[0.5,2)$`, sharpening everything gives a wrong classification for 87 of them, and two copies for none, in every one of which the interval contains `$b$`. If instead the iterate is used as its own seed, width doubles at every step, and at step 50, for `$b=0.7$`, the divisor could be zero and the iteration fails with the iterate about `$10^4$` wide.

Note what the "two copies" answer is, and is not. A single Newton step from a sharp seed encloses the Newton image of that seed, which differs from the root by second order. Here the rounding width covers that difference once the seed has converged, as the survey's "contains" count confirms. But it's not a *proof* that the root lies inside. For a proof you need a test like Krawczyk's ([Lecture 24](./24-rank-and-proof.md)). The kernel takes the same stance: the width states how well the data pins the answer down, and the point where existence must be established uses a proof.

## Rule 4: validate the value you are about to use, not a wider one

`Model::split_edge_at_vertex` once checked `curve.evaluate(edge_t).could_be_equal(&vertex_point)` on the wide `edge_t`, a fat box that genuinely contains the vertex, and then split at the *sharpened* `edge_t`. The check answers "is the vertex somewhere along this stretch of curve?", while the split asks "cut exactly here". Those are different questions, and the gap between them is real geometry: the true crossing `$t^*$` lies somewhere in `edge_t`, the midpoint generally isn't `$t^*$`, and `$C(t_{sharp})$` is a different point from `$C(t^*)=$` vertex, off by `$\lvert t_{sharp}-t^*\rvert\,\lvert C'(t)\rvert$`.

Note what is not wrong there: the two halves are exact sub-arcs. What's wrong is asserting a topological identity ("this split point is that vertex") that the numbers never established. So when you sharpen a parameter, re-validate the sharpened one, or better, refine it until the identity is true, by Newton on `$C(t)=$` vertex. That is `NurbCurve::split_parameter_at_point` today.

Do not patch a mismatch by moving geometry to match the assertion. Pinning a split curve's end point onto the vertex was tried, and looked as if it worked for the 3-D curve. Applied to the pcurve, whose `$(u,v)$` came from a search `min_subdivision_size` wide, it pinned a control point to a box so wide that the pcurve had no usable tangent ("Cannot normalize zero"), and took the sweep from 0 failing scenes of 175 to 12. Fix the parameter, not the geometry.

## Rule 5: ask a question about a box only where its answer holds for a box

The converse trap. `face_interior_point_where` (`core/geop-core-topology/src/contains/face.rs`) tests a box around a candidate, so that the accepted point keeps clear of the boundary. Vertex and pcurve checks answer that honestly for a box. But the ray casting that came after built its ray from the box, which turned the ray into a *strip*. A strip merely grazing a pcurve overlaps it just as a crossing does, so that gives one hit, and the parity flips. A point `$0.44$` from a hole's axis, with the rim at `$0.33$`, was found "inside" the disc cut from a plate's top. The boolean kept that disc, and the result was left open.

Parity counting only means something for a ray from one point. Once the box is known to touch no boundary, all of it lies on one side, and one sharp point of it decides which. So before you run an interval computation on a wide input, check that its answer still means something *for every point in the input*.

The inclusion theorem of the previous lecture guarantees an enclosure of `$f$` over the box. It doesn't guarantee that what you did with that enclosure, here counting, makes sense over a box.

## Rule 6: proximity is not membership

Something is selected because it is *near*, and then used as if it *belonged*. A traced intersection curve ends when it reaches a known vertex, and `candidate_within` chose the nearest vertex within one marching stride. But a traced curve lies on both surfaces it is traced between, so its end vertex must lie on both too, and an oversized solid puts many unrelated vertices within a stride of the curve's end. Adopting one of those ended the edge at a point on neither face. `fit_pcurve` then had nothing to project onto, and silently returned the nearest foot point, about `$8\times10^{-3}$` away, which finally surfaced as 32 `validate_fast` errors about pcurve end points.

The fix requires what the geometry actually asserts: `surface_could_contain` on both surfaces. When you select by proximity, ask what property the code is about to *assume*, and test that directly.

A diagnostic that pays for itself: when such a check fails, report whether the point lies on the surface **at all**. "The pcurve names the wrong `$(u,v)$` for a point that is on the surface" and "the edge was spliced onto a face it does not belong to" produce identical error text, but need opposite fixes. One extra `surface_could_contain` in the error path separated them: of those 32 failures, every one had exactly one end point on the surface and one off.

## Rule 7: check an iterative answer against everything it was asked

An iteration that stops is not an iteration that converged. The shell moves a vertex onto the offsets of its faces, by Gauss-Newton in `meet` (`core/geop-core-geometry/src/meet.rs`). It returned a hemisphere's pole at `$(0,0.9,0.1)$`, on the offset plane but off the offset sphere, and the failure surfaced two steps later as "not on the circle". There were two silent causes, a dropped condition and a projection that never moved. Checking the returned point against every surface it was asked to lie on turned this into an error at the source, naming the surface it missed, with the numbers. It costs one projection per condition. It is what makes the next bug of the kind take minutes rather than an afternoon.

## Rule 8: trace numerical bugs to their root cause

When a search misbehaves, there's a strong pull towards a local workaround: trim the input away from the trouble spot, pad a result "to be safe", seed the search with a known-good answer, add a special case for the one failing pair. These can make the symptom vanish while leaving the defect in place. Some (padding, seeding) smuggle an epsilon back in by a side door.

Instead, keep descending until the answer is a specific line of arithmetic, not a vague "it's imprecise near shared vertices". From `AGENTS.md`, a real example. A `curve_curve_intersect` blowup was first "fixed" by trimming curves near a shared vertex. That worked, but only by avoiding the region where the bug lived. The chain as eventually found, one level at a time:

1. Repeated `NurbCurve::split` calls compounded interval width in a control point's weight. Fixed by a tighter `interpolate` formula.
2. The same with knot values divided by an ever-shrinking span. Fixed by sharpening a free-choice value immediately after the operation that introduced the rounding noise, not by padding the result.
3. Only with both fixed did it become clear that a third, independent thing was needed: a domain-width convergence signal. The first two had buried it under noise.

### Name the entities before you name the cause

Explanations such as "near-tangent crossing" and "imprecise near a shared vertex" sound plausible, cannot be checked and are often wrong. Before you state why something failed, identify the exact entities at the failing vertex or edge by name, and check the explanation against them.

A revolve cut "up to next" failed with a vertex too wide to validate, and it was explained to the user as a near-tangency. The user looked at the scene: there was none. Naming the vertex showed it was where the drilled hole's rim crossed the revolve's *end face*, and that face was there because the operation's own fallback had stopped the tool flat at the first point of contact, an existing corner. The bug was a logic error, not numerics.

So first ask whether something *our code chose* (a stop position, a cap, a split point) put the entities there. Blame numerics only after that is ruled out. [Lecture 42](./42-case-studies.md) has more cases of this.

## Rule 9: debug with context in the real call stack

When you need to know why something fails, do not write `eprintln!` or a scratch harness. Add context to the real path:

```rust,ignore
let ctx = |e: GeopError| {
    e.with_context(format!(
        "Model::split_edge_at_vertex(edge_id={edge_id}, edge_t={edge_t}, vertex_id={vertex_id}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
    ))
};

let edge = self.get_edge(edge_id).with_context(&ctx)?.clone();
```

That is real code from `core/geop-core-topology/src/edit/split_edge_at_vertex.rs`. The context is permanent: it stays and helps the next failure of the kind report itself. You saw what it gives in [Lecture 2](./02-programs-of-steps.md): the root cause, then a chain of what was being done, with stable names and numbers.

When a cause depends on actual numbers, print them with `{:?}`. `Display` on a `Scalar` truncates to three decimals of the midpoint, silently hiding exactly the width you're trying to diagnose. `GeopError` in `core/geop-core-math/src/geop_error.rs` is the error type.

The sanctioned alternative is a regression test. Every bug fix begins by pinning the failure into its own standalone `#[test]`, red for exactly that reason. Then fix, then require both the reproducer and the broader suite to be green. A scene that fails as one line of a 175-scene log tells you almost nothing, while the same scene as its own test can be run in a second. Fixes that green the reproducer but regress the sweep are not fixes, and it has happened here. Over time, the suite gets one test per real defect.

## Rule 10: refuse what you do not support, early and by name

An operation handling only some configurations must recognize the others before it starts building. Otherwise the unsupported case appears deep in a boolean, as a degenerate splice, or worse, as an invalid model that nobody notices. The shell assumed a removed face's inner copy lies inside the face, which is false where that face meets a kept face at an inward corner. It failed in five different ways before `inward_corner` (`lib/geop-lib-shell/src/shell.rs`) recognized that up front:

```text
face … is taken away where it meets face … at an inward corner, along edge …, and the walls grow outward: not supported
```

A refusal names the entities and the condition, so the user can change the model and the next session can extend the code. This costs a little code. It saves a lot of despair.

<div class="recap">

## Recap: a checklist

Before you commit numerical code, ask:

- Am I comparing with a constant? Then what does `could_be_equal` or `definitely_*` say instead, and what do I do when it can't tell?
- If I change this tunable, can a correct answer become wrong? Then it's an epsilon.
- Do I combine two enclosures? Same value: intersect. To cover both: union. Average: never.
- Am I sharpening? Is that value a free choice? If an answer comes out of it, keep two copies.
- Do I check one value and use another? Re-validate what I use.
- Do I ask a box a question that only makes sense for a point?
- Did I select this by closeness? What property am I about to assume? Test that.
- Did the iteration converge, or merely stop? Check every condition.
- Is my fix a trim, a pad or a seed? Keep descending to the line of arithmetic.
- Have I named the entities? Did our own code put them there?
- Is it red in its own test? Does the failure explain itself?
- Should this be a refusal, by name, before anything is built?

</div>

## Exercises

**1.** Classify each as a legitimate or illegitimate use of `sharpen`, and say why: (a) choosing the parameter at which to bisect a domain in a containment search; (b) storing the vertex where a curve pierces a face; (c) choosing the point at which to evaluate a surface's normal in order to fix a plane for a corrector step; (d) returning the arc length of a curve; (e) evaluating a sample point to describe a problem in an error message.

<details class="solution"><summary>Solution</summary>

(a) Legitimate: a free choice, and necessary. (b) Illegitimate: a vertex is an answer, and later comparisons need its width. (c) Legitimate: `predictor_corrector_step` sharpens the plane's point and normal, because any plane crossing the curve serves. (d) Illegitimate: a length is an answer. (e) Harmless: it decides nothing, it's only a description. Though prefer `{:?}` of the unsharpened one, since the width may be the clue.

</details>

**2.** Two computations of a parameter give `$[0.30,0.34]$` and `$[0.33,0.40]$`. Give what you'd return, if (a) both are rigorous enclosures of one number; (b) the same rigorous enclosures, but of two different branches of a curve, and you want a box around both; (c) rigorous enclosures of one number, and the second is really `$[0.35,0.40]$`. What does (c) tell you, and what does `Scalar::intersect` do with it?

<details class="solution"><summary>Solution</summary>

(a) `$[0.33,0.34]$`. (b) `$[0.30,0.40]$`. (c) They are disjoint, which proves that at least one isn't an enclosure of that number: there is a bug, or the two aren't about the same number. `Scalar::intersect` returns the narrower input rather than an inverted interval; its documentation restricts it to "two enclosures of the same underlying exact value". A careful caller that can meet this should check `could_be_equal` first and report it.

</details>

**3.** Write the two-copies loop as pseudocode for a one-dimensional Newton iteration with an interval residual `f` and derivative `df`. Which of `seed` and `answer` does each of these get: the evaluation of `f`? The value returned? The check of whether the seed has repeated? The intersection with the box that subdivision proved?

<details class="solution"><summary>Solution</summary>

```text
seed = start.sharpen()
answer = start
for k in 0..N:
    next = seed - f(seed) / df(seed)
    answer = next
    seed = next.sharpen()
    if seed was seen before: break
return answer.intersect(box)
```

`f` and `df` are evaluated at `seed`. `answer` is returned. A repeat of `seed` ends the iteration, since a deterministic iteration from a repeated seed is periodic. (That's `Repeat` in `core/geop-core-math/src/solvers/iterate.rs`, and no tolerance is involved: the seed is equal bit for bit.) The intersection is taken with the answer, as both enclose the same value.

</details>

**4.** In the sharpen demo, drag `$b$` until "sharpen everything" says something other than "on boundary", then look for a `$b$` where it is right. The readout also counts over 1000 boundaries. Why can the answer be off by one ulp, though the problem is well-conditioned?

<details class="solution"><summary>Hint</summary>

The conditioning is fine: the error of the true Newton fixed point in floats is a small multiple of `$u$`. But "correct to within an ulp or two" is not "equal", and classification against an exact boundary tests equality. That's the point: the fault isn't with Newton's accuracy but with presenting that value as exact.

</details>

**5.** A search returns `None` when it exhausts `max_nodes`, which the caller reads as "no intersection". Give a scenario for a wrong solid, and fix the interface. Then find what the kernel actually does: read the doc comment of `ValidationParameters::max_nodes` in `core/geop-core-topology/src/validation/parameters.rs`.

<details class="solution"><summary>Solution</summary>

Two surfaces meet in a small region; the search spends its budget in a large, nearly coincident region elsewhere and gives up. "No intersection" means that no cut is made, and the boolean's classification then keeps or drops a whole face wrongly. The fix: an exhausted budget is an error, distinct from "searched and found nothing". The comment says the pairwise searches "now error, rather than silently returning a truncated result, if they exhaust it".

</details>

**6.** Which rule does each of these break? (a) "Project the vertex onto the surface, and if the foot point is within `1e-6`, it is on the surface." (b) "This vertex is the closest one, so it must be the end of the edge." (c) "We ran Newton for 20 iterations, so `(u, v)` is the solution." (d) "The ray from this box crossed the loop three times, so the box is inside." (e) "The boolean fails near the corner, so we trim the tool `1e-5` away from it."

<details class="solution"><summary>Solution</summary>

(a) Rule 1. Use `surface_could_contain`, which is a three-valued question. (b) Rule 6. (c) Rule 7. (d) Rule 5. (e) Rule 8, and see "Never place a cut on a corner that already exists" in `AGENTS.md`: choose positions the geometry defines instead.

</details>
