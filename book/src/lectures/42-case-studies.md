# Lecture 42: Case studies in debugging

<div class="goals">

**After this lecture you can**

- apply one debugging method to a numerical geometry failure: isolate it, cap it, name the entities, descend to a line of arithmetic, fix the cause, keep the reproducer;
- tell a symptom from a cause, and recognise the common false explanations;
- recognise a dozen recurring shapes of bug, each from a real case in this kernel;
- predict, given a new symptom, what you would check next.

</div>

This is the last lecture, and it is made of real bugs. Each comes from `AGENTS.md`, the rulebook of this repository, where it was written down by whoever chased it so that nobody would have to chase it twice. Numbers in this lecture are the numbers recorded there. Where I retell the temptations involved, I say what was actually tried if `AGENTS.md` records it, and otherwise I mark it as the reasoning a reader might fall into.

Geometry bugs have an awful property: the failure usually appears far from the fault. A wrong bit in a corrector shows up three modules later as a face a boolean cannot classify. So the method matters more than in most software, and we state it first.

## The method

**1. Isolate first.** Pull the failure out of whatever sweep surfaced it into a single standalone `#[test]` that fails for exactly that reason. A scene that fails as one line of a 175-scene log tells you almost nothing; the same scene as its own test can be run in a second, read in full, and reasoned about. With the reproducer red, you have an unambiguous signal for whether a change addresses the cause or shifts the symptom elsewhere in the sweep, which is easy to mistake for progress when the only metric is an aggregate count. A user's `.geop` file is rebuilt in Rust in the `regression_tests.rs` of the crate whose operation failed, never committed. Confirm that the Rust version still fails without the fix.

**2. Cap the run.** Every program runs with a memory cap and a timeout:

```sh
timeout 600 prlimit --as=16000000000 cargo test -p geop-ops-fillet some_test
```

A search that runs away grows until the machine dies. A capped run fails fast, and that is a finding: interrupt it under `gdb` to see where.

**3. Add context to the real call stack.** Never `eprintln!`, never a scratch harness. Wrap fallible calls with a small closure, as `Model::split_edge_at_vertex` does:

```rust
let ctx = |e: GeopError| {
    e.with_context(format!(
        "Model::split_edge_at_vertex(edge_id={edge_id}, edge_t={edge_t}, vertex_id={vertex_id}, ...)"
    ))
};
let edge = self.get_edge(edge_id).with_context(&ctx)?.clone();
```

(`core/geop-core-topology/src/edit/split_edge_at_vertex.rs`, and the `with_context!` macro in `core/geop-core-math/src/geop_error.rs` builds the same without a cost on the success path.) This context is permanent. It stays in the codebase and makes the next failure of this kind self-report. When numbers matter, print them with `{:?}`, not `{}`: the `Display` of an interval scalar truncates to 3 decimals of the *midpoint*, which silently hides exactly the width you are trying to diagnose.

**4. Name the entities before you name the cause.** "Near-tangent crossing" and "imprecise near a shared vertex" sound plausible, cannot be checked, and are often wrong. Identify the exact entities at the failing vertex or edge by name. `assert_valid` in `ops/geop-ops-model/src/testing.rs` appends `names_mentioned` to its report, which resolves the ids in a validation error to names like `extrude(box,end)`.

**5. Ask whether something our code chose put the entities there.** Blame numerics only after that is ruled out.

**6. Descend until the answer is a specific line of arithmetic.** Not "it's imprecise", but which formula, which factor.

**7. Then verify both.** The reproducer must go green, and the full suite must stay green. Keep the reproducer, named after the scenario, not the bug. If it must be committed while still failing, mark it `#[ignore]` with what is known and what has been ruled out.

**8. When you change how a shape is built, diff old and new models entity by entity.** Build both from a `git worktree` of HEAD outside the repo, comparing interval bounds, not just midpoints. Construction code is sensitive at the last bit ([Lecture 26](./26-sweeps.md)).

Now the cases. The first one is the one that gave rule 4 its name.

---

## Case 1: The near-tangency that was not there

**Symptom.** A revolve cut "up to next" failed with a vertex too wide to validate.

**The explanation given.** It was explained to the user as a near-tangency.

**What the user did.** Looked at the scene. There was none.

**Named entities.** Naming the vertex showed it was where *the drilled hole's rim* crossed *the revolve's end face*. Why was there an end face at all? Because the operation's own fallback had stopped the tool flat at the first point of contact, which was an existing corner.

**Root cause.** A logic error in our own code, not numerics. An operation chose a position that landed exactly on an existing vertex. That asks the boolean to cross an edge at its own end point, a degenerate configuration no enclosure stays tight through.

**Fix.** Choose positions the geometry actually defines: go up to the next face from where the tool enters the target, or all the way round or through all if part of it never meets the target again, rather than stopping flat at a sampled contact point.

**Lessons.** *Do not present an explanation as fact until it is traced to specific entities.* And: **never place a cut on a corner that already exists.**

---

## Case 2: The blowup in curve-curve intersection

**Symptom.** A `curve_curve_intersect` blowup.

**What looked like a fix.** Trimming the curves near a shared vertex. It worked, but only because it avoided the region where the bug lived.

**The chain.** Traced one level at a time:

1. Repeated `NurbCurve::split` calls compounded interval width in a control point's *weight*. Fixed by a tighter `interpolate` formula.
2. Width compounded in *knot values* divided by an ever-shrinking span (the `$\alpha=(t-a)/(b-a)$` of Boehm's insertion, [Lecture 9](./09-subdivision.md)). Fixed by sharpening a free-choice value right after the operation that introduced rounding noise, not by padding the result afterwards.
3. Only after both were fixed did it become clear that a third, independent thing was needed: an additional domain-width convergence signal. The first two fixes revealed it cleanly instead of burying it under noise.

**Lesson.** A workaround that makes a symptom go away can leave the defect in place, ready to resurface in a slightly different shape. Worse, padding and seeding smuggle a magic tolerance back in through a side door, against "never use epsilons". Keep descending.

---

## Case 3: The sharp answer that lied three modules away

**Symptom.** A face a boolean could not classify, because it had no interior point.

**Named entities and chain, from the far end back.** The face was a zero-area sliver. The sliver was carved by a curve spliced in as a duplicate of a trim boundary. The direction check that exists to reject such a curve had waved it through, because `face_contains` classified its `(u, v)` as `Inside`. That `(u, v)` was *sharp*, sitting `6e-17` off the boundary, so `could_be_equal` had no width to work with. And it was sharp because `predictor_corrector_step` sharpened its iterate every Newton step, including the one it returned.

**What it was not.** Geometric error: the point was where it should be to within `6e-17`. What was destroyed was the statement of uncertainty. A value honestly `[0, 1.2e-16]` answers "could this be on the boundary?" correctly. The same value sharpened to `6e-17` answers it wrongly, with total confidence.

**Fix.** Keep two copies. Sharpen the one that seeds the next iteration, because that is a free choice, and without it 20 Newton steps compound width until `evaluate` fails outright (measured: 36/175 scenes). Carry the unsharpened update alongside as what gets returned. This is now `core/geop-core-math/src/solvers/iterate.rs`.

**Lesson.** Sharpen only where the value is a free choice, never where it is an answer ([Lecture 5](./05-honest-numerics.md)).

---

## Case 4: Every curve traced twice

**Symptom.** The same one: zero-area slivers. A second, independent cause.

**Chain.** Every intersection curve has two ends, both piercing points, so both land in `find_tracing_start_points`' output, and every curve is traced *twice*, once from each end. The second trace splices a duplicate edge between the same two vertices, which runs along the boundary the first splice just created.

**What was tried.** Sampling the finished curve, and requiring some point of it to be strictly inside the face. *It works, and it is wrong:* a sampling heuristic standing in for a question with an exact answer, at a cost of a `face_contains` per sample, papering over the duplicate.

**Fix.** The test `edge_is_boundary_of_face` embodies: is this curve already represented here? Look for an edge that already joins the same two vertices on both faces. What makes that sufficient is a pipeline invariant: `remesh_edges_x_edges` runs first and has put a vertex at every coincidence, so no curve can *partially* overlap an edge.

**The invariant then failed once.** `trace_one_side` chose its first direction by a trial step, and when the half step towards a near vertex was rejected, the full step landed past that vertex and was accepted. So the trace jumped the vertex and spliced a partial duplicate. A step must never decide a direction past a vertex known to lie on the curve.

**Lesson.** Ask a topological question topologically. Where an invariant holds, use the exact test it enables.

---

## Case 5: The crash in the boolean that was in the rasterizer

**Symptom.** A revolve "crashed the boolean": it ran out of memory.

**Named location.** Only interrupting the capped run showed where it was: not in the boolean at all, but in the rasterizer. Clipping a concave hole of many corners split it into exponentially many pieces.

**Line of arithmetic.** `subtract_convex` (`core/geop-core-rasterize/src/clip.rs`) kept degenerate leftovers. A concave hole is taken out as many triangles, one after another, and cutting every piece along every triangle's sides, whether it overlaps or not, multiplies the pieces. Now, if the leftover of a subtraction is empty, the peeled pieces are only the subject cut up for nothing, and the subject is returned whole.

**Lesson.** The runaway is often far from where the failure was reported. Do not raise the cap; find where it ran away.

---

## Case 6: The hemisphere's pole, off the sphere

**Symptom.** A shell's "not on the circle" error, two steps after the real failure.

**Named entities.** The shell moves a vertex onto the offsets of its faces by Gauss-Newton (`meet`, `core/geop-core-geometry/src/meet.rs`). For a hemisphere's pole it returned `(0, 0.9, 0.1)`: on the offset plane, off the offset sphere.

**Chain.** Two silent causes.

- A dropped condition, which `AGENTS.md` names without more detail.
- A projection that never moved. `NurbSurface::project` skipped any Newton step whose Jacobian could be singular. From a seed on the pole every step is singular, so it returned the pole for every target. At a pole only the collapsed parameter stops moving the point; the other one still moves it. So it steps along that one, down the meridian that heads most towards the target and leads into the domain (`off_pole`).

**A neighbouring bug at the same pole.** Two quarters of one sphere meet there, one smooth surface, so one condition. The code compared normals at foot points each patch clamps to its own domain; those differed a little, the quarters counted as two nearly equal conditions, and the system went singular. Now surfaces are grouped by tangency once, at the vertex (`tangent_groups`).

**Fix that made it findable.** Check the returned point against every surface it was asked to lie on. That turned the bug into an error at the source, naming the surface it missed, with the numbers.

**Lesson.** An iteration that stops is not an iteration that converged. And the surface is fine at a pole: only the coordinates are singular ([Lecture 18](./18-poles-seams-splicing.md)).

---

## Case 7: Inside a hole, by parity

**Symptom.** A boolean kept a disc cut from a plate's top, and left the result open.

**Named entities.** A point 0.44 from a hole's axis, where the rim is at 0.33, was found "inside" the disc.

**Chain.** `face_interior_point_where` asks `face_contains` about a box of `$\pm\epsilon$` around a candidate, so the accepted point keeps clear of the boundary. The vertex and pcurve checks answer that honestly for a box. But the ray casting after them built its ray from the box, which turns the ray into a strip. A strip that merely grazes a pcurve overlaps it the way a crossing does, so it gives one hit, and the parity flips.

**Fix.** Once the box is known to touch no boundary, all of it lies on one side, and one sharp point of it decides which side.

**Lesson.** Before you run an interval computation on a wide input, check that its answer still means something for every point in that input. Parity counting does not: counting crossings only makes sense for a ray from a single point ([Lecture 19](./19-classification.md)).

---

## Case 8: 32 errors about pcurve endpoints

**Symptom.** 32 `validate_fast` errors, all about pcurve endpoints not matching their 3-D points.

**Diagnostic that decided it.** Report whether the point lies on the surface **at all**. "The pcurve names the wrong `(u, v)` for a point that is on the surface" and "the edge was spliced onto a face it does not belong to" give identical error text and need opposite fixes. One extra `surface_could_contain` in the error path turned the guess into a measurement: 32 failures, every one with exactly one endpoint on-surface and one off.

**Chain.** A traced intersection curve terminates when it reaches a known vertex, and `candidate_within` chose the nearest vertex within one marching stride. An oversized or overhanging solid puts plenty of unrelated vertices within a stride. Adopting one ended the edge at a point on neither face. `fit_pcurve` then had nothing to project onto and silently returned the nearest foot point, about `8e-3` away.

**Fix.** Require what the geometry actually asserts: a traced curve lies on both surfaces, so its terminating vertex must too: `surface_could_contain` on both.

**Lesson.** Proximity is not membership. When you select by nearness, ask what property the code is about to *assume*, and test that property directly.

---

## Case 9: The cut that widened the curve threefold

**Symptom.** A disc's rim cut by thirty-two slots ended `4e-10` below its cap, and gears stopped at 80 teeth, with their rims drawn in twelve arcs to spread the cuts. A quarter arc cut forty times went from `3e-15` to `0.08`.

**Chain.** An edge was cut at the whole refined enclosure of where it reaches the vertex. That kept every piece it could be cut into there in one. The parameter's width went through Boehm's `$\alpha=(t-a)/(b-a)$` into the pieces' control points and end knot. The next cut's parameter, found on the wider piece, came out wider. `AGENTS.md` records that the width grew threefold per cut; the quarter arc above grew by a factor of about `$2.7\times10^{13}$` in forty cuts, on average over twofold per cut.

**Fix.** Where in that enclosure to cut is a free choice: every parameter in it ends a piece on the curve and within the vertex. `split_parameter_at_point` cuts at its sharp middle and checks the curve reaches the vertex there. Each cut then adds one subdivision's rounding, and the rim is a circle again. The pcurves are cut at that same point, not each at a point of the vertex of its own: where a cubic crosses a plane at its inflection the vertex is wide, and two choices in it ended the edge and its pcurve `1.4e-13` apart.

**Companions from the same gear.** Three other things grew with every cut (*Many booleans on one body*, in `AGENTS.md`). Names: a piece of a split edge is named after the edge and the vertex it starts at, and the vertex after the edge, so names doubled per cut, and twenty cuts made names of tens of megabytes. An argument longer than `LONGEST_ARGUMENT` is now spelled by its digest. Rescans: the piercing search restarted from every edge by face pair after each split, until a pair found clean stayed clean (a piece of a curve crosses nothing the whole did not). Traced fits: an involute's curvature changes tenfold along the flank, and the legs had to be split finer while drift is what makes the curve wide.

**Lesson.** What is harmless once may grow with every operation. Look for a factor per cut, and test at many cuts.

---

## Case 10: The bracket at (10000, -5000, 2000)

**Symptom.** A base plate with a bolt hole, built at this offset as a part of a larger assembly, came out invalid: *face's normal points into solid rather than out of it*, on three pieces of the hole's wall. At the origin, at `(1000, 0, 0)` and at `(10000, 0, 0)` the same cut built valid. It needed the offset in all three coordinates. Regression test: `cylinder_cut_from_a_block_far_from_the_origin` in `ops/geop-ops-boolean/src/regression_tests.rs`.

**The tempting reading.** Numerics at large coordinates: what else could depend on the distance from the origin?

**Named chain.** The normals were right. The *check* was wrong, and not by numerics. Which side of a face the solid is on was decided by classifying a point a distance off the face, the distance halved from the shell's "extent" until the two sides disagreed. The extent took the least coordinate over all three axes (`-5000`, the `y`) and the greatest (`10080`, the `x`): 15080 for a plate 80 long. So the halvings stopped at 7, and the one probe that resolved, at 29, went from a bore's wall out through 18 of plate on one side and across the bore into plate on the other. Three pieces had normals towards a side 18 away, and those were exactly the three that failed.

**Fix.** The limit as the distance goes to zero is what the question means, and a ray leaving the face answers it exactly: the parity of its crossings past the face (`solid_contains_beside`). The point the ray leaves from is a free choice, and a point on the boundary elsewhere too is rejected (`on_solid_boundary_besides`).

**Dividend.** The ray's crossings are kept with the verdict and named in the error. That showed the subd mug's handle patch to be a ribbon folded onto itself, a ray leaving it crossing its other half `6e-7` on, which no probe had seen.

**Lesson.** A probe is a sample too. A distance that decides an answer is an epsilon. And the same test measures volume from the mesh, where a sum of tetrahedra from the origin weighed slivers by their distance from it: 5.5 for a bore of 142. A sum that is exact only for a closed figure is taken from a point of the figure (`enclosed_volume`).

---

## Case 11: A cylinder's caps that overlapped

**Symptom.** In fixed point, validation found a cylinder's two caps overlapping. A cap 50 along its axis came out spanning `[38, 94]` in height. A quarter arc at `(1000, -500)` gave a tangent 30 wide.

**Chain.** A rational spline evaluated at an interval parameter computes `$A/W$` from two enclosures interval arithmetic cannot correlate. The quotient's width then grows with `$|A|$`, with the patch's *distance from the origin*, not its size.

**Fix.** Surfaces, curves and pcurves evaluate relative to a sharp control point of the span (`spline::centered`), adding it back with one rounding.

**A twist.** That one rounding is not free near the origin. A STEP edge lay `2e-31` above its plane face. Evaluated relative to a point `1.8e-15` up, its enclosure took in the plane, the importer measured no gap and did not widen the edge, and validation found the face's points off it. So a curve evaluates both ways and intersects the two. Both are enclosures of one point, and each is the tighter one somewhere.

**Lesson.** In `f64` this hides behind the format's relative precision until the width is read. A test of geometry far from the origin must assert the width of the enclosure ([Lecture 11](./11-evaluate-locally.md)).

---

## Case 12: 21 of 175, then 1 of 175

**Symptom.** `validate_fast` failures across 21 of 175 scenes.

**Chain.** `NurbSurface::project` sharpened *every* iteration including the final one. For all but the last that is correct, as the value is only a seed. But the final iterate is the answer, and `du`, `dv` inherit the target's width through the residual: the honest statement of how precisely an uncertain target pins down a foot point. Sharpening returned one sharp `(u, v)` for a target that only determined a *range*, and every later comparison against anything else derived from the same uncertain point then failed.

**Fix.** Leave the last step unsharpened. That one fix took failures from 21/175 to 1/175.

**Lesson.** See how far one line reached: twenty scenes that looked unrelated. A wrong statement of uncertainty fails everywhere it is read.

---

## Case 13: Pinning geometry to the assertion

**Symptom.** `Model::split_edge_at_vertex` checked `curve.evaluate(edge_t).could_be_equal(&vertex_point)` and split, yet the split point was not the vertex.

**Chain.** The check ran on the *wide* `edge_t`, a fat box that genuinely contains the vertex, and then split at the *sharpened* one. The check answers "is the vertex somewhere along this stretch?", while the split asks "cut exactly here". The true crossing `$t^*$` sits somewhere in `edge_t`; `$C(t_\text{sharp})$` is a perfectly valid point of the curve and simply *a different point* from the vertex, off by `$|t_\text{sharp}-t^*|\,|C'|$`.

**What was tried.** Pinning the split curve's endpoint onto the vertex. It looked like it worked for the 3-D curve. Applied to the pcurve, whose `(u, v)` came from a `min_subdivision_size`-bounded search, it pinned a control point to a box so wide the pcurve had no usable tangent left ("Cannot normalize zero"), and took the sweep from 0/175 failing scenes to 12/175.

**Fix.** Fix the parameter, not the geometry: refine it until the identity you are about to assert is true, Newton on `$C(t)=$` vertex, and re-validate the value you use.

**Lesson.** Validate the value you are about to use, not a wider one. Do not reconcile a mismatch by moving geometry to match the assertion.

---

## Case 14: The pivot of `1e-16`

**Symptom.** A half circle tangent to two parallel sides could not be enclosed. *Nothing was wrong with the constraints.*

**Chain.** The sketch solver's elimination went column by column, taking the first column with an entry that cannot be zero. Rounding leaves entries of `1e-16` that, as intervals, cannot be zero. One became a pivot and determined a coordinate the constraints barely touch. That left the half circle with its sweep free and two tangencies over the same coordinate.

**Fix.** Complete pivoting: largest entry over all remaining rows and columns.

**Lesson.** "Any value that cannot be zero" makes a pivot *valid*, not *good*. A free choice still has to be a good one ([Lecture 23](./23-sketch-solver.md)).

---

## Case 15: A line that took 359 steps

**Symptom.** A splice failed on a "corner" it could not decide; the fitted curve's control points had width 111.

**Named chain.** The curve was a straight line, running along a cone's generator into its apex. `adaptive_step_size` shortened every marching step so a full turn of the tighter surface would take 64 of them. At an apex the tightest bend is the circle round the axis, which shrinks with the distance to the apex. So each step was a fraction of the remaining distance, and the trace never arrived: 359 steps, spaced geometrically, for a line. Interpolating through those points widened exponentially along the chain.

**Fix.** A surface that runs straight the way the curve does (`curvature_radius_along` reports none) bends it by nothing, and no longer limits the step. Going further was tried: sizing by the bend along the curve for every surface. It is wrong, since a surface may bend a curve a little here and sharply a stride on, and the pattern of `turbine_blade` left its patch (the slow `example_mass_properties_are_consistent` caught it).

**Lesson.** Look at the number of steps a trace took: a line that took hundreds went wrong before it was fitted. And note how a slow test in the ignored set stopped the over-generalization.

---

## Case 16: A body that could not turn half way

**Symptom.** A dragged arm's solver ran out of steps. Nothing was wrong with the mates.

**Chain.** A body's turn was the quaternion `(1, w / 2)` normalized, which reaches a half turn only as `$|w|$` goes to infinity. A link that had to turn nearly half way round ran its variables off to hundreds, each step turning it less.

**Fix.** A pose is stepped by a twist in its own frame, through the exponential map (`Pose::exp`, `Variable::retract`). Every step of the solve is taken from where the body is, so no turn is ever far from the variables' origin.

**Lesson.** Degenerate parametrizations are not only in surfaces. When a solve stalls, check that its variables can reach the answer at a finite, well-conditioned value ([Lecture 36](./36-mates.md)).

---

## Patterns

Sort the cases by what kind of failure of reasoning they were:

| Pattern | Cases | The question to ask |
|---|---|---|
| Blaming numerics for a logic error | 1, 10 | What did our own code choose? |
| Width compounding by iteration | 2, 9, 15 | What grows with each repetition? |
| Discarding the statement of uncertainty | 3, 12 | Is this value an answer or a free choice? |
| Asking a sample for a certainty | 4, 10 | Does the representation offer an exact test? |
| Fixing a symptom where it appears | 2, 4, 13 | Has this been traced to a line of arithmetic? |
| Reading an interval as a point, or the reverse | 7, 13 | Does the answer mean something for every point in this input? |
| Taking proximity for membership | 8 | What property am I about to assume? |
| Forgetting that coordinates are not geometry | 6, 15, 16 | Is this singular in the geometry, or in the parametrization? |
| A far-away crash | 5, 3 | Where is it really? Interrupt it. |

If you remember only one table of this book, it might as well be this one.

<div class="recap">

## Recap

- Isolate a reproducer, cap the run, add permanent context, name the entities, descend to arithmetic, fix the cause, keep the test.
- Most hard failures are logical or about honesty, not about rounding.
- A failure and its cause can be modules apart, and the honest enclosure is what lets the failure report itself.
- A fix that makes a symptom vanish without an account of the arithmetic is a hypothesis, not a fix.

</div>

## Exercises: what would you check next?

Each of these gives a symptom in the kernel's own vocabulary. The point is the next question you would ask, not the fix.

**1.** A boolean of a block with a sphere reports a vertex `4e-6` wide, whereas both inputs' vertices are about `1e-15` wide. A colleague says "the sphere is nearly tangent to the block's edge there". What do you do before believing that?

<details class="solution"><summary>Solution</summary>

Name the entities at the vertex with `names_mentioned`: which edge, which face? Then check against them: compute the actual distance and the crossing angle. Next ask whether our code chose anything there: is one of those entities a cap or a stop position placed by an operation, on a corner that already exists? Only then consider numerics: which parameter is wide, and what widened it? (Case 1.)

</details>

**2.** A sweep over 175 scenes goes from 4 failures to 3 after your change, and one of the three is new. Is that progress?

<details class="solution"><summary>Solution</summary>

You cannot tell from the count. Isolate the old failure as a standalone test, and the new one too. A fix that greens the reproducer while regressing the sweep is not a fix. Look at what connects the new failure to your change. (The method, step 1 and step 7.)

</details>

**3.** `meet` returns a point, and a test downstream fails with "vertex not on edge". You find the point is `1e-3` off one of the offset surfaces. Where do you add the permanent check, and what does the error report?

<details class="solution"><summary>Solution</summary>

In the function returning the point: check it against every condition it was asked to meet, and fail there, naming the surface it missed, with the numbers printed by `{:?}`. This is what `meet` already does since Case 6: if it has passed its own check, the problem is in what you asked it. Look for a dropped condition, or a clamped foot point.

</details>

**4.** After you add a cache to a function, `assert_round_trips` passes but `assert_incremental` fails on the second program of `edit_sequence`, with a difference in one surface's `aabb`, at the last bit. What are you looking for?

<details class="solution"><summary>Solution</summary>

Two ways of computing one value. The incremental path keeps a value derived by a shortcut (say, moved along with a translate) while the scratch path computes it from the points. The rule is one function for a cached value (`compute_aabb`), as told in [Lecture 38](./38-runner.md). Also ask whether your cache key misses something a make reads.

</details>

**5.** A curve-curve intersection of two traced edges returns `Coincident` where you can see they only cross. Trace what you would print. Why `{:?}`?

<details class="solution"><summary>Solution</summary>

The control points' interval bounds and knots of both curves, and the box sizes the search ended on, in `Debug`. `Display` shows a midpoint to 3 decimals and hides the width, which is the thing in question. A traced edge's control points might be very wide, so that within its width it does overlap the other: then ask why that curve is wide (its widest marched point against its widest true point, as `trace_one_side`'s context reports).

</details>

**6.** A user's part fails only when placed far from the origin in all three coordinates. List the first three hypotheses you would test, from what this lecture shows, and how each is tested.

<details class="solution"><summary>Hint</summary>

Case 10: a formula that mixes axes or distances (a scale computed over coordinates of the whole model), found by reading what decides the answer and printing it. Case 11: an evaluation that subtracts or divides large correlated values, tested by asserting the width of the enclosure at the far position. And: an operation's chosen position or padding that depends on the coordinates. Rebuild the part in Rust at the reported coordinates first, as the reports' regression tests do: a "clean" equivalent may not carry the same case at all.

</details>
