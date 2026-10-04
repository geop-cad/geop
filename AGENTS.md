# Agent rules for this repo

This project is a new CAD kernel. It's self contained, and backwards compatibility is not important. The code is written in Rust, and the goal is to be correct, robust, simple, and maintainable. For every change, ask how this fits nicely into the overall design, and whether it makes the code easier to understand and maintain. If it doesn't, consider a different approach. New helper functions need to be added in the correct place. If it makes sense to rename a method, unify some arguments, or change a type, do so, even if it break backwards compatibility. The goal is to have a clean, simple, and correct codebase, not to maintain a stable API. The same holds for saved files: when a format change breaks old `.geop` files, delete them rather than adding a compatibility reader, and keep the schema simple and consistent. For the same reason, never commit `.geop` files as test fixtures — rebuild a user's reproducing file in Rust instead (see `cad/geop-cad-base/src/operations/regression_tests.rs`), and confirm the Rust version still fails without the fix.

Do not introduce new helper function for a single use case or because another function is violating its abstraction or intention. Instead, fix the existing function to work as expected.

Think logical. If the code makes sense from a logical perspective, but doesn't work as intended, fix the underlying logical issue instead of just patching the symptom.

## Cap every run's memory and time

Every program you start — a test, the CLI, a scratch run — runs with a
memory cap and a timeout, e.g.
`timeout 600 prlimit --as=16000000000 cargo test ...` or
`timeout 120 prlimit --as=6000000000 target/release/geop compile ...`.
A geometric search or triangulation that runs away does not stop on its
own: it grows until the machine runs out of memory, and takes everything
else on it down with it. A capped run fails fast instead, with
"memory allocation failed" or a timeout, which is itself a finding — then
find out where it ran away (run it under `gdb` and interrupt it) rather
than raising the cap.

The runaway is often far from where the failure was reported. A revolve
that "crashed the boolean" was in fact the rasterizer: clipping a concave
hole of many corners split it into exponentially many pieces
(`subtract_convex` kept degenerate leftovers). Only interrupting the
capped run showed where it was.

When you need a process id, take it from the process you started (`$!`),
never by matching names: `pgrep geop` also finds the user's own editor and
servers.

## Debugging: use `with_context`, never `eprintln!`/temporary code

When you need to understand why something fails, **add context to the real
call stack** instead of writing throwaway debug code:

- Wrap fallible calls with `.with_context(&ctx)?`, where `ctx` is a small
  `|e: GeopError| e.with_context(format!("fn_name(arg={arg}, ...)"))`
  closure — see `Model::ker`, `Model::mve`, `Model::split_edge_at_vertex`,
  `remesh_vertices::find_coincident_vertex_pair` for the established style.
- When an error's root cause needs the actual numeric state (control points,
  knot vectors, domains, intervals), print it via `{:?}` (`Debug`), not `{}`
  (`Display`) — `Display` on interval scalars truncates to 3 decimals of the
  *midpoint*, which silently hides exactly the width you're trying to
  diagnose.
- This context is permanent, not a scratch artifact: it stays in the
  codebase and helps the *next* failure of the same kind self-report,
  instead of requiring another investigation from scratch.

Do not add `eprintln!`, ad hoc `#[test]` scratch harnesses, or temporary
print statements to trace a bug. If existing context isn't enough to
diagnose something, that's a sign more context is missing from the real
code path — add it there, permanently, rather than instrumenting around it
temporarily. (Exception: a genuinely new, durable regression test that
exercises the bug going forward is fine and encouraged — the thing to avoid
is throwaway tracing code.)

## Never use epsilons for comparisons

Every `Scalar` is (or should be treated as) an interval type that already
propagates numerical uncertainty through arithmetic. Correctness comes from
comparing intervals honestly (`could_be_equal`, `definitely_greater`, etc.),
not from picking a magic epsilon and hoping it's the right size for every
call site.

- Never write `(a - b).abs() < epsilon`-style comparisons, and never
  introduce a new epsilon constant to paper over a comparison that should
  instead rely on the scalar's own interval semantics.
- `max_nodes` / `min_subdivision_size` (or similarly named step sizes) are
  the only tunables allowed to affect *how hard the search tries* — more
  budget or a tighter tolerance changes runtime and how quickly a search
  converges, not what a converged answer's *correctness* means. A search
  that hasn't converged should return `None`/an error/a wide (unsharp)
  interval — never a value that's merely "close enough" per an epsilon.
- If a result comes out wider than expected, the fix is to understand why
  (e.g. slow parametrization amplifying a tolerance, or multiple genuinely
  separate solutions unioned together) — not to shrink an epsilon until the
  symptom disappears.

## Trace numerical bugs to their root cause, don't paper over them

When a search or comparison misbehaves (hangs, exhausts its node budget,
returns a too-wide or wrong result), resist the pull toward a local
workaround — trimming the input away from the trouble spot, padding a
result "to be safe," seeding the search with a known-good answer, adding a
special case for the one pair that's failing. These can look like fixes
(the symptom goes away) while leaving the actual defect in place, ready to
resurface in a slightly different shape somewhere else. Worse, some of
these (padding, seeding) directly conflict with "never use epsilons" above:
they smuggle a magic tolerance back in through a side door.

Instead, ask *why* the numbers are wrong in the first place, and keep
descending until the answer is a specific line of arithmetic, not a
vague "it's imprecise near shared vertices." Concretely, from one real
session: a `curve_curve_intersect` blowup was first "fixed" by trimming
curves near a shared vertex — worked, but only because it avoided the
region where the bug lived, not because it fixed anything. The actual
chain, found by tracing one level at a time: repeated `NurbCurve::split`
calls compounded interval width in a control point's weight (root cause 1,
fixed by a tighter `interpolate` formula) and in knot values divided by an
ever-shrinking span (root cause 2, fixed by sharpening a free-choice value
right after the operation that introduced rounding noise, not by padding
the result afterward). Only *after* both were fixed at the source did it
become clear a third, independent thing was needed — an additional
domain-width convergence signal — because the first two fixes revealed it
cleanly instead of burying it under noise.

A workaround is sometimes the pragmatic short-term call under real time
pressure — but say so explicitly, and prefer spending the extra time to
find the root cause over shipping a trim/pad/seed that merely relocates
where the bug shows up next.

## Name the entities before you name the cause

"Near-tangent crossing", "imprecise near a shared vertex": an explanation
like this sounds plausible, cannot be checked, and is often wrong. Before
you state why something failed, identify the exact entities at the failing
vertex or edge by name (`assert_builds_valid` in the
regression tests resolves the ids in validation errors to names via
`names_mentioned`), and check the explanation against them.

From one real session: a revolve cut "up to next" failed with a vertex too
wide to validate, and was explained to the user as a near-tangency. The user
looked at the scene: there was none. Naming the vertex showed it was where
the drilled hole's rim crossed the revolve's *end face*, and the end face
was there because the operation's own fallback had stopped the tool flat at
the first point of contact, which was an existing corner. The bug was a
logic error in our own code, not numerics.

So ask first whether something our code *chose* (a stop position, a cap,
a split point) put the entities there. Blame numerics only after that is
ruled out, and do not present an explanation as fact until it is traced to
specific entities.

### Never place a cut on a corner that already exists

That case generalizes. When an operation chooses where a new face goes, a
choice that lands it exactly on an existing vertex or edge of the other body
asks the boolean to cross an edge at its own end point, a degenerate
configuration no enclosure stays tight through. Choose positions the
geometry actually defines instead. For up to next, that means going up to
the next face from where the tool enters the target, or all the way round
or through all if part of it never meets the target again, rather than
stopping flat at a sampled contact point.

## Construction code is sensitive at the last bit

Shapes feed booleans, and booleans at tangent contacts depend on interval
widths down to the last ULP. Rewriting extrude/revolve as one sweep builder
broke six "inscribed cylinder" boolean tests with identical geometry: every
station row was multiplied by an exact weight of 1, and the outward rounding
widened every control point. Skipping that multiplication fixed all six.

- Never multiply or add by exact constants needlessly in construction code.
- When you change how a shape is built, diff old and new models entity by
  entity (build both from a `git worktree` of HEAD outside the repo),
  comparing interval bounds, not just midpoints.

## Combine two enclosures of the same value with `union`, never an average

When two independent computations each produce an enclosure of the *same*
underlying quantity — e.g. a point on a face x face intersection curve,
projected once onto each surface — combine them with `union` (the smallest
interval containing both), not by averaging them.

An average produces a single sharp value that claims more precision than
either input had, and it silently discards the most useful signal in the
pair: *how far apart the two answers were*. That gap is the honest measure
of how much the two computations disagree, and it is exactly what later
convergence and "have I arrived?" tests need to see. `union` keeps it;
averaging throws it away and replaces it with false confidence.

The same reasoning is why [`Scalar::intersect`] exists for the opposite
case: when two enclosures of one value come from formulas with different
numerical strengths (see `Scalar::interpolate`), their *intersection* is
still a valid enclosure and is tighter than either — there, keeping less
width is justified, because both bounds genuinely hold simultaneously.
Union when you must cover both possibilities, intersect when both
constraints must hold at once, average never.

Example: `splice_edge_into_face` pins each end of a new loop to the
arriving end's `(u, v)` *intersected* with the vertex's projection onto the
surface. Using the end alone let `interpolate_enclosing`'s width spread
around the loop until a face split came out degenerate. Both values enclose
the same point, so their intersection is honest and tighter.

## Sharpen only where the value is a free choice, never where it is an answer

Every `Scalar` is an enclosure, and the whole correctness story rests on
those enclosures staying honest. We are always seeking the mathematical
truth: bounds should be as tight as they can legitimately be made, but a
bound must never be narrowed past what the computation actually
established. Dropping uncertainty does not make a result more accurate, it
makes it *wrong in a way nothing downstream can detect* — and in a geometry
kernel the concrete consequence is missed intersections.

`sharpen` is the sharpest tool for violating this, so it gets a rule.
Sharpening is legitimate **only where the value being sharpened is a free
choice** — where any value inside the interval would serve equally well,
so collapsing to one of them loses no mathematical accuracy. Two places
qualify:

- **Subdivision parameters in the `contains` / `intersection` loops.** A
  search that splits an interval may split it *anywhere*; the split point
  is the algorithm's own free choice, not a claim about geometry. Sharpening
  it is not only safe but necessary — repeated `split` calls on an unsharp
  parameter compound interval width through Boehm insertion (a knot span
  divided by an ever-shrinking span) until the search is useless.
- **The predictor, before the corrector.** A predicted point is a seed and a
  bias for the damped Newton that follows; it can be chosen arbitrarily
  without affecting which fixed point the corrector converges to.

Everywhere else, sharpening a value that is an *answer* is a bug:

- **Inside the corrector loop.** The corrector seeks a fixed point that
  encloses the space of possible solutions (in the spirit of the Krawczyk
  fixed-point formulation). Its iterates are the result, not a seed.
- **The last iteration of an iterative refinement.** `NurbSurface::project`
  sharpened *every* iteration including the final one. For all but the last
  that is correct — the value is only a seed for the next step. The final
  iterate is the answer, and `du`/`dv` inherit the target's width through
  the residual: that width is the honest statement of how precisely an
  uncertain target pins down a foot point. Sharpening it away returned one
  sharp `(u, v)` for a target that only ever determined a *range* of them,
  and every later comparison against anything else derived from the same
  uncertain point then failed. Fixing just this took `validate_fast`
  failures across the scene sweep from 21/175 to 1/175.

### A sharpened answer lies to every interval test downstream

The sharpest illustration of the rule above, because the damage happened
three modules away from the cause. `predictor_corrector_step` sharpened its
iterate every Newton step *including the one it returned*. A traced
intersection curve that ran exactly along a face's trim boundary therefore
came back with a **sharp** `(u, v)` sitting `6e-17` off that boundary — and
`face_contains` classified it `Inside`, because `could_be_equal` had no width
to work with. The direction check that exists precisely to reject such a
curve waved it through; the curve was spliced in as a duplicate of the
boundary, carving a zero-area sliver; and that surfaced much later as a face
a boolean could not classify because it had no interior point.

Note what the sharpening cost was *not*: no geometric error. The point was
where it should be to within 6e-17. What it destroyed was the *statement of
uncertainty* that every three-valued comparison downstream depends on. A
value that is honestly `[0, 1.2e-16]` answers "could this be on the
boundary?" correctly; the same value sharpened to `6e-17` answers it wrongly
and with total confidence.

The fix generalizes, and it is worth reaching for whenever a loop needs sharp
seeds but produces an answer: **keep two copies**. Sharpen the one that seeds
the next iteration — that is a free choice, and without it 20 Newton steps
compound width until `evaluate` fails outright (measured: 36/175 scenes) —
and carry the unsharpened update alongside as what gets returned. The
returned width is then that of a *single* step from a sharp seed: bounded per
step rather than either compounded or discarded.

### Ask a topological question topologically

The same episode produced a second, independent cause of the same symptom,
and the contrast between the two fixes is the lesson.

Every intersection curve has two ends, and both are piercing points, so both
land in `find_tracing_start_points`' output — meaning every curve is traced
*twice*, once from each end. The second trace re-derives the same curve and
splices a duplicate edge between the same two vertices. That does not split
anything: it runs along the boundary the first splice just created, carving
off a zero-area sliver.

The tempting fix is geometric: sample the finished curve and require some
point of it to be strictly inside the face. That was tried. It works, and it
is wrong — it is a sampling heuristic standing in for a question with an
exact answer, it costs a `face_contains` per sample, and it papers over the
duplicate rather than recognising it. The right test is the one
`edge_is_boundary_of_face` already embodies: *is this curve already
represented here?* — answered by looking for an edge that already joins the
same two vertices on both faces. Exact, cheap, and it names the actual
condition.

What makes the topological test sufficient is a pipeline invariant, and it is
worth stating because it is easy to forget: `remesh_edges_x_edges` runs first
and has already put a vertex at every coincidence, so no curve can *partially*
overlap an edge. A curve either fully duplicates one or does not touch it.
Where an invariant like that holds, reach for the exact test it enables
instead of sampling geometry to rediscover it.

### Validate the value you are about to use, not a wider one

A specific trap this rule keeps producing. `Model::split_edge_at_vertex`
checked `curve.evaluate(edge_t).could_be_equal(&vertex_point)` on the *wide*
`edge_t` — a fat box that genuinely contains the vertex — and then split at
the *sharpened* `edge_t`. The check answers "is the vertex somewhere along
this stretch of curve?" while the split asks "cut exactly here." Those are
different questions, and the gap between them is real geometry: the true
crossing parameter `t*` sits somewhere in `edge_t`, the midpoint generally
is not `t*`, and `C(t_sharp)` is a perfectly valid point on the curve that
is simply *a different point* from `C(t*) = vertex_point`, off by
`|t_sharp - t*| x |C'(t)|`.

Note what is *not* wrong there: sharpening introduced no geometric error,
and the two halves are exact sub-arcs. What was wrong was asserting a
topological identity ("this split point is that vertex") that the numerics
never established. When you sharpen a parameter, re-validate the sharpened
one — or, better, refine it until the identity you are about to assert is
actually true (Newton on `C(t) = vertex_point`), rather than splitting at an
arbitrary interior point of a search box and reconciling afterwards.

Do not reconcile such a mismatch by moving geometry to match the assertion.
Pinning the split curve's endpoint onto the vertex was tried and looked like
it worked for the 3-D curve; applied to the pcurve — whose `(u, v)` came
from a `min_subdivision_size`-bounded containment search — it pinned a
control point to a box so wide the pcurve had no usable tangent left
("Cannot normalize zero"), and took the sweep from 0/175 failing scenes to
12/175. Fix the parameter, not the geometry.

## Subdivide to isolate, then Newton to refine

Subdivision and Newton are different tools and neither does the other's job.
Using one for both is what made split parameters wide enough to need
`sharpen` in the first place.

- **Subdivision is the global method.** It reliably finds *every* solution,
  separates them, and — through the leaf-count signal the intersection
  searches rely on — recognizes coincidence even for a partial overlap. But
  it converges one bit per split, so reaching machine accuracy takes ~50
  levels against the ~7 that isolate the solution. Worse, the coincidence
  signal *depends* on stopping early: at fine tolerance a transversal
  crossing also produces many leaves, and the contrast that made the signal
  meaningful disappears.
- **Newton is the local method.** It cannot find anything, but it polishes an
  isolated solution quadratically — a dozen iterations where subdivision
  would need forty more levels.

So `min_subdivision_size` is a *handoff threshold*, not an accuracy target:
subdivide until the solutions are separated, then refine each one. The
tolerance stops influencing the final accuracy at all, which is a stronger
form of the epsilon rule above than making the search itself tolerance-free.

Refinement must stay honest. Sharpen every iterate *except the last* (each is
only a seed for the next), leave the final step unsharpened so the returned
width states how well the data pins the answer down, and intersect the result
with the box subdivision proved the solution lies in — both are valid
enclosures of the same value. Make it infallible: a singular Jacobian at a
tangential crossing or a pole, an iterate leaving the domain, a refined box
disjoint from the original — all return the incoming box unchanged.
Refinement may only tighten, never fail, or callers start needing fallbacks
of their own.

Refine where the parameter is *used*, not everywhere. `curve_surface_intersect`
deliberately does not refine everything it returns: most callers only need to
know where and how many crossings there are, and refining changes results they
already agree with — doing it unconditionally broke `offset_sphere_is_valid`.
Refine when the parameter becomes a split point, where width genuinely matters.

One trap worth naming: a refinement is only as good as its *target*. Refining
a pcurve parameter against a `(u, v)` that is itself `min_subdivision_size`
wide gives a wide residual, a wide step, and a result no narrower than it
started — silently, since the fallback path looks identical to success. The
target has to be refined first (`NurbSurface::project` on the vertex point).
The same applies to a vertex built by evaluating a wide crossing parameter:
fix the parameter at the source, or everything downstream inherits its width.

## Proximity is not membership

A recurring shape of bug in this kernel: something is selected because it is
*near*, and then used as though it *belonged*. The two are not the same, and
the gap only shows up much later, somewhere else.

The clearest case: a traced intersection curve terminates when it reaches a
known vertex, and `candidate_within` chose the nearest vertex within one
marching stride. But a traced curve lies on both of the surfaces it is
tracing between, so its terminating vertex must lie on both too — and an
oversized or overhanging solid puts plenty of unrelated vertices within a
stride of the curve's end. Adopting one of those ended the edge at a point on
neither face. `fit_pcurve` then had nothing to project onto and silently
returned the nearest foot point instead, ~8e-3 away, which finally surfaced
as 32 `validate_fast` errors about pcurve endpoints not matching their 3-D
points. The fix was to require what the geometry actually asserts:
`surface_could_contain` on both surfaces.

The same shape appears in `find_coincident_pair` reading a solution count as
coincidence, and in `find_vertex_at_point` matching by distance. When you
select a candidate by proximity, ask what property the code is about to
*assume* it has, and test that property directly.

A diagnostic that pays for itself: when a check like this fails, report
whether the point lies on the surface **at all**. "The pcurve names the wrong
`(u, v)` for a point that is on the surface" and "the edge was spliced onto a
face it does not belong to" produce identical error text but need opposite
fixes, and one extra `surface_could_contain` in the error path separates them
immediately. It turned this bug from a guess into a measurement: 32 failures,
every one with exactly one endpoint on-surface and one off.

## Make the invariant a type, not a convention

`Face` used to hold a flat `Vec<BoundaryRef>` with "index 0 is the outer loop"
as an unwritten rule, plus a `face` back-pointer that could go stale. It is
now `outer: BoundaryType` and `holes: Vec<BoundaryType>`, and
`find_boundary_containing` returns a `BoundaryIndex` (`Outer` or `Hole(i)`)
rather than a bare position.

That change is not cosmetic — it is what makes the hard cases expressible. An
edge joining two points of a face's boundary means four different things
depending on which boundaries those points sit on: two points of one hole
divide it into two holes; two different holes merge into one; a hole bridged
to the outer loop stops being a boundary at all; and two points of the *outer*
loop cut the face itself in two. Only the last creates a face. With a flat
list, all four looked like "same index or different index" and were handled as
two cases, which silently produced faces carrying several disjoint patches.

Two consequences worth remembering:

- **A face always has exactly one outer boundary, so "no boundary" is not a
  state.** A face with no edges yet is bounded by a bare *vertex* — the state
  `mvfs` produces. That makes `mer` promote a bare-vertex boundary to the ring
  that arrives on it, and `ker` demote it back when the last loop leaves.
  `remove_boundary` refuses to delete an outer loop outright: an operation
  that genuinely consumes one is deleting the face and must say so.
- **Splitting a face invalidates "on this surface" as a membership test.**
  Both halves share one surface, so every edge already imprinted into the
  original is still on the surface of the half it does not belong to — and
  `find_coincident_pair` re-imprinted it, splitting again, forever. Proper
  face topology is also the fix: ask whether the edge lies inside *this face's
  trim* (`face_contains` at the curve's midpoint), which is a question a face
  with a real outer loop and real holes can answer and a bag of loops cannot.

## Isolate a reproducer before fixing anything

Every bug fix starts by pinning the failure into its own test, and ends
with both that test and the broader suite passing:

1. **Isolate first.** Pull the failure out of whatever sweep or pipeline
   surfaced it into a single standalone `#[test]` that fails for exactly
   that reason. A scene that fails as one line of a 175-scene log tells you
   almost nothing; the same scene as its own test can be run in a second,
   read in full, and reasoned about.
2. **Then fix.** With the reproducer red, you have an unambiguous signal
   for whether a change actually addresses the cause — as opposed to
   shifting the symptom somewhere else in the sweep, which is easy to
   mistake for progress when the only metric is an aggregate count.
3. **Then verify both.** The reproducer must go green *and* the full suite
   must stay green. A fix that greens the reproducer while regressing the
   sweep is not a fix; that has happened here and the aggregate numbers are
   what caught it.

Keep the reproducer afterwards — it is the durable value. Name it after the
scenario, not the bug, and if it must be committed while still failing,
mark it `#[ignore]` with a reason recording what is known and what has been
ruled out, so the next session starts from evidence instead of re-deriving
it. This is the one sanctioned exception to "no throwaway debugging code"
(see the `with_context` section): a durable regression test is the opposite
of throwaway. Over time the suite accumulates one test per real defect,
which is what makes the kernel get harder to break rather than merely
differently broken.
