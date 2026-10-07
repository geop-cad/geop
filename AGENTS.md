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

## Evaluate relative to the patch, not the origin

A rational spline evaluated at an interval parameter computes `A / W` from
two enclosures interval arithmetic cannot correlate. The quotient's width
then grows with `|A|`, that is with the patch's distance from the origin,
not with its size. In fixed point, a cylinder's cap 50 along its axis came
out spanning `[38, 94]` in height, and validation found the cylinder's two
caps overlapping. A quarter arc at `(1000, -500)` gave a tangent 30 wide.

Surfaces, curves and pcurves now evaluate and differentiate relative to a
sharp control point of the span (`spline::centered`) and add it back with
one rounding. Which point is a free choice. Wherever an interval formula
subtracts or divides two large correlated quantities, move it to a local
origin first. Test geometry far from the origin with both scalar types: in
`f64` the effect hides behind the format's relative precision, in fixed
point it does not.

That one rounding is not free near the origin. A STEP edge lay `2e-31`
above its plane face. Evaluated relative to a point `1.8e-15` up, its
enclosure took in the plane, the importer measured no gap and did not widen
the edge, and validation's clipping, which reads the control points, found
the face's points off it. A curve now evaluates both ways and intersects
the two. Both are enclosures of one point, and each is the tighter one
somewhere.

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

The invariant has one more condition: a trace must stop at every vertex on
its curve. `trace_one_side` chose its first direction by a trial step, and
when the half step towards a near vertex was rejected (the curve ran along a
boundary up to it), the full step landed past that vertex and was accepted.
The trace jumped the vertex and spliced a curve that *partially* duplicated
an existing edge, which the invariant says cannot happen. A step must never
decide a direction past a vertex known to lie on the curve.

The viewport's pick had the same shape of bug. An edge counted as hidden
when it lay more than one pointer reach behind the face hit, measured along
the ray. At a glancing look, an edge of that very face lies further along
the ray than that, so it could not be hovered. "Is this edge in front of
the face?" was standing in for "does this edge bound the face?", which the
topology answers exactly (`ViewEdge::faces`).

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

### Check an iterative answer against every condition it was asked to meet

An iteration that stops is not an iteration that converged. The shell's
`offset_vertex` moves a vertex onto the offsets of its faces by Gauss-Newton.
It returned a hemisphere's pole at `(0, 0.9, 0.1)`, on the offset plane but
off the offset sphere, and the failure surfaced two steps later as "not on
the circle". Two silent causes: a dropped condition, and a projection that
never moved (see "Degenerate parametrizations" below). Checking the returned
point against every surface it was asked to lie on turned that into an error
at the source, naming the surface it missed, with the numbers. Make that
check part of the function. It costs one projection per condition, and it is
what makes the next bug of the same kind take minutes instead of an
afternoon.

### Ask a question about a box only where its answer holds for a box

The converse trap. `face_interior_point_where` asks `face_contains` about a
box of `±epsilon` around a candidate, so the accepted point keeps clear of
the boundary. The vertex and pcurve checks answer that honestly for a box.
But the ray casting after them built its ray from the box, which turns the
ray into a strip. A strip that merely grazes a pcurve overlaps it the way a
crossing does, so it gives one hit, and the parity flips. A point 0.44 from
a hole's axis (the rim 0.33) was found "inside" the disc cut from the
plate's top, and the boolean kept that disc and left the result open.

Once the box is known to touch no boundary, all of it lies on one side,
and one sharp point of it decides which side. Before you run an
interval computation on a wide input, check that its answer still means
something for every point in that input. Parity counting does not:
counting crossings only makes sense for a ray from a single point.

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

## Degenerate parametrizations: the surface is fine, the coordinates are not

At a pole, where a row of control points collapses to one point (the apex of
a revolved face, a sphere's poles), the parametrization is singular. The
surface itself is perfectly smooth there. Three bugs came from treating the
coordinates as if they were the geometry:

- **A seed on the pole never leaves it.** `NurbSurface::project` skipped any
  Newton step whose Jacobian could be singular. From a seed on the pole every
  step is singular, so it returned the pole for every target. Only the
  collapsed parameter stops moving the point; the other one still moves it.
  So step along that one, down a meridian. Which meridian is a free choice,
  since at the pole the collapsed parameter says nothing: take the one that
  heads most towards the target and leads into the domain (`off_pole`). A
  pole can sit at either end of the domain, so do not assume the step is
  positive.
- **Decide tangency where the geometry is, not at moving foot points.** Two
  quarters of one sphere meet at its pole: one smooth surface, so one
  condition. `offset_vertex` compared normals at foot points that each patch
  clamps to its own domain. Those normals differed a little, the two quarters
  counted as two nearly equal conditions, and the system went singular.
  Group surfaces by tangency once, at the vertex, and measure each group from
  its nearest foot point, which is the foot point on the union of the
  pieces.
- **Straight in `(u, v)` is not preserved by offsetting.** The shell kept an
  inner trim curve straight whenever the original was straight. That holds
  on a plane (affine coordinates) and along an iso-line. It fails for a
  sphere's meridian cut by a plane: the inner copy is a small circle.

The same holds for the coordinates a solver moves in. A body's turn was
the quaternion `(1, w / 2)` normalized, which reaches a half turn only as
`|w|` goes to infinity. A link of a dragged arm that had to turn nearly
half way round ran its variables off to hundreds, each step turning it
less, until the solver ran out of steps. Nothing was wrong with the mates.
The turn is now given by modified Rodrigues parameters
(`geop-core-math/src/solvers/system/placed.rs`): rational, a half turn at `|w| = 4`,
singular only at a full turn. When a solve stalls, check that its
variables can reach the answer at a finite, well-conditioned value.

## A free choice still has to be a good one

"Any value that cannot be zero" makes a pivot *valid*, not *good*. The
sketch solver's elimination went column by column, taking the first column
with an entry that cannot be zero. Rounding leaves entries of `1e-16` that,
as intervals, cannot be zero. One of them became a pivot and determined a
coordinate the constraints barely touch. That left a half circle tangent to
two parallel sides with its sweep free and two tangencies over the same
coordinate, which could not be enclosed. Nothing was wrong with the
constraints. Complete pivoting (largest entry over all remaining rows and
columns) fixed it.

The same holds for every free choice the rules above allow: subdivision
points, Newton seeds, which meridian to leave a pole along, where to split a
curve at a matched point. Validity is the minimum. When a choice is free,
choose the well-conditioned one.

## Decide a rank over the box you prove, not at a point

`system::enclose` chose its independent constraint rows by eliminating the
Jacobian at the solved point. A redundant constraint (an arc slot's second
`Concentric`, a rectangle's fourth right angle) holds only *on* the
solution, and the solved point is a solution only to the solver's
tolerance; there its row looked independent by a pivot as small as that
tolerance. Taken as independent, it turned the solution into a curve, and
no Krawczyk box could be proven. The sketch tools had to avoid redundancy
by hand.

The rows are now chosen over a box around the point, widened from the
point itself through the solver's tolerance: a pivot that could be zero
anywhere in the box is no pivot. The rows left out are then checked over
the proven box, and one that does not enclose zero there is a conflict,
reported by name. Interval arithmetic can prove a conflict, never its
absence, so say "consistent to what the box resolves", not "implied".
The old code proved an "exact solution" for two lengths 1e-11 apart.

## Refuse what you do not support: early, by name, and say why

An operation that only handles some configurations must recognise the rest
before it starts building. Otherwise the unsupported case shows up deep in a
boolean as a degenerate splice, or worse, as an invalid model nobody notices.
The shell assumed a removed face's inner copy lies inside the face, which is
false where that face meets a kept face at an inward corner (a pocket's
floor, a step, the face under a boss). It failed in five different ways
before `inward_corner` refused those cases up front, naming the faces and
the edge. A refusal names the entities and the condition, and says what is
supported, so the user can change the model and the next session can extend
the code.

When adding an end condition or a special case, decide it where the
geometry actually differs. For blends, each end of an edge is decided on its
own: run out where the edge leaves the solid, stop flush at a wall, mitre
where two blended edges meet at an inward corner. One rule for the whole
edge was wrong for one of the cases in each combination.

## Test what the user does, not only what the arguments say

Four bugs passed every operation test because the tests built the arguments
directly:

- The shell's faces could never be picked: `EntityRef::lies_in` fell back to
  `self == scope` for every case it did not list, and a face never equals
  its solid. The shell tests set the face names directly.
- An offset plane's handle worked when the step was opened, and not while it
  was being created. The selection field was still waiting for picks, and a
  waiting field swallowed every pointer event, drags included.
- Seeking back on the timeline kept showing the last step's part, because
  `settle` ran the whole program after `rerun` and left the runner there.
- Fillet tool ends were tested per edge, and broke only when two blended
  edges met at a corner.

For anything a user reaches through the editor, write at least one test in
`editor_tests.rs` that drives it the way the front end does: `New`, `Click`,
`Hover`, `Drag`, `Seek`. Assert on what the editor sends back (the
presentation's `grab`, the scene's solids, the dialog's values). Be wary of
catch-all `_ =>` arms that answer a question: they give confident answers
for cases nobody thought about.

## Sweep the space, but keep the everyday suite fast

`stress_tests.rs` blends every edge, and shells every face, of boxes,
cylinders and spheres: whole, drilled, pocketed, stepped, cut off and with a
boss. Its first run found five shell bugs that no hand-written test had
reached. Sweeps like this tell a refusal apart from a failure, and require
that whatever is built is valid.

They are also slow. Bodies with circular edges take up to a minute each,
and the user noticed the suite slowing down. Keep a fast representative
subset in the default run. Mark the rest
``#[ignore = "slow: … — run with `cargo test -- --ignored`"]``, like the
boolean render sweep. Run the ignored set before changing blends, shells,
booleans or the solver. When a regression test needs a slow setup, split
it: a fast test of the actual defect, and an ignored test of the
combinations.

## Count the work, not the seconds

A drag in a 2000-part robot went from 0.25 s to minutes, and nothing
timed it. The editor builds every step's form on every change, for the
step list; the added part's form asked the whole assembly how free its
part was, a dense null space over every part's variables. Once per step,
that is cubic in the parts. What fixed it, and what to keep doing:

- A form shown in a list is of the step's arguments and the program's
  state. Questions about what was built, over the whole assembly, are for
  the one dialog open, once per edit.
- A system that splits into independent groups is solved group by group,
  and so is any question about it (`Assembly::freedom`, like the solve).
- Listing must not build: example programs and standard parts are listed
  by name, and made when opened or read.
- Guard scaling with a count (`geop_ops::assembly::mates_resolved`), not
  a time: a quadratic shows at 80 parts as clearly as at 2000.

Timings mislead. Two builds of the same code differ by up to 30% (how
code is split into codegen units), and other jobs on the machine slow
everything. Compare CPU time, run the old and new binaries interleaved,
and build both the same way. `perf` is not allowed here: profile by
interrupting a capped run under gdb at intervals and counting stacks.
`ptrace_scope` only lets gdb into a process it started: run the test under
`gdb -batch`, send the inferior `SIGUSR2` (handled `stop print nopass`)
and print the backtraces. To count, set breakpoints that only count —
on a line, or on a function with its caller checked — never prints.

The second round found the cost each time in work that was not needed,
not in slow arithmetic:

- **Where the representation says polynomial, integrate exactly.** Mass
  properties ran an adaptive 15-point rule inside another along every
  trim curve. Along a direction with equal weights a surface is a
  polynomial, its moments one of known degree, and a Gauss–Legendre rule
  of a few points has no truncation error at all: five times fewer
  surface points, and a proven inner integral instead of an estimated
  one (`mass.rs`).
- **A deterministic iteration that repeats a seed is done.** Newton with
  sharpened seeds rarely hits an exact fixed point; it cycles in the last
  bit and ran out its iterations. Once a seed repeats, the rest is
  periodic and the result known: the identical answer, in less than half
  the iterations (`NurbSurface::project`).
- **Ask the cheap question first.** A boolean cast a ray across the whole
  other solid for every interior point it classified; a point near one
  already classified is classified along a short path from it, which a
  box test rules out of nearly every face (`shell_contains_from`). A path
  between structured points is not generic, though: a straight segment
  between two points of one face lies in its plane, and two corners of a
  symmetric face can graze a cylinder. Go by a random point, as a ray
  takes a random direction.
- **An interval argument is no licence to extrapolate.** A spline
  evaluated over a parameter interval reaching past one knot span used
  that span's polynomial for all of it: 25 times too wide over a fitted
  curve, which a search then subdivided away at great cost. Curves now
  unite their spans' parts (`find_spans`); surfaces still extrapolate, and
  the tangent-branch search on blends pays for it.

## Working in parallel worktrees

Features developed side by side in separate worktrees (one agent each)
conflicted only in the shared lists every feature appends to:
`PartOperation` and its imports, the workspace and `geop-cad-base`
`Cargo.toml`, `set_tests.rs`'s list of operations, `web/src/icons.tsx` and
`DEVELOPERS.md`. Keep both sides, then read the merged result: a doc comment
listing crates, or a JSX entry cut mid-element, needs a hand edit. The merge
is not done until the full workspace suite and the web build pass on the
merged branch. Each branch passing its own tests says nothing about the
combination, or about paths that only the editor exercises (see above).

## An approximate tangency dips through the face

A surface that is only approximately tangent to a face — a rolling-ball
blend skinned through exact stations, tangent at each, interpolated between
— crosses the face as often as not between the stations, by a hair. A
boolean then has to cut the face along every such crossing: near-duplicate
vertices next to the contact, "nothing to split" errors. Tangency to within
the approximation's error is no tangency to the boolean. The blend in
`geop-ops-fillet/src/rolling.rs` leaves each face at a millionth of a radian
towards the ball, and checks between stations that it really leaves on
that side, so that it meets the face in its contact curve alone.

Where the approximated data is only C1 — a face's curvature jumping across
an edge, a radius changing its rate at a vertex — no number of stations
makes a cubic follow it to that accuracy. Put a station exactly there and
break the spline: the ball's contact on the edge between the two faces
(found on that edge, not near it), a station at the vertex where the radius
law kinks.

## A tool's own curves must not lie in the solid's faces

A blend tool meets the solid where the boolean finds it; any curve of the
tool lying *in* a face of the solid, crossing that face's boundary, asks the
boolean to resolve an overlap rather than a crossing, and it exhausts its
search. Three times in `geop-ops-fillet`:

- A rolled chamfer's run-out ended its last span with a station at the
  chain's end vertex, its section in the third face's plane, and its chord
  crossed that face's boundary there. The run-out is now part of the last
  span (a repeated knot), so the face is crossed by the span's interior.
- A straight chamfer built as a whole tool once used its chord's ends on the
  faces, exact lines lying in them. Extending the chord past the faces, as
  the swept chamfer always did, made it cross them instead. (A fillet's
  contact curves do lie on the faces, but tangentially and padded to
  enclose the true contact, which the boolean takes.)
- Mitring a rolled blend at an inward corner was tried, running it on
  straight from its station at the vertex. Where the walls stand square to
  the shared face, that station's section lies in the *other* wall's plane,
  its curves on that wall's face, and its contact on its own wall already on
  the mitre plane (a run-on side of no length). It failed in the boolean,
  and rolled blends meeting at an inward corner are refused instead.

## A hash map's order is no order

`Model`'s entities live in `HashMap`s, whose iteration order differs between
two models of the same part. The corner planner took a corner's edges in
that order, and the ball's patch began at a different contact on every
build; the example's round trip (build, save, load, build again) caught it.
Wherever an order reaches the result — which loop corner comes first, which
face is "first" — sort by the ids, which are given in creation order.

The kernel's own curves are such data. Its arcs and helices are rational
quadratics, only C1 where their spans meet. `fit_pcurve` fitted one C2
cubic through 48 samples of the whole curve. Across those joints its drift
shrank only as `h^2`, and a helix of pitch 1 came within 15% of the 1e-4
an entity may carry. The joints are known: they are the knots of the
source curve. So it now fits each smooth piece separately and joins them,
and only then adapts, sampling a piece more densely while the pcurve is
wider than the target. Use the structure you know before you sample
blindly, and let the sample count follow the width it produces.

## Many booleans on one body: what grows with every cut

A spur gear is a disc with a gap cut per tooth, twenty to a hundred
booleans on one body. Four things that are harmless once grew with every
cut, and each only showed up past some count of teeth:

- **Names.** A piece of a split edge is named after the edge and the
  vertex it starts at, and that vertex after the edge again, so the name
  doubled per cut: twenty cuts made names of tens of megabytes. A name's
  argument longer than `LONGEST_ARGUMENT` is now spelled by its digest.
- **Rescans.** The piercing search started over from every edge × face
  pair after each split. A pair once found clean stays clean (a piece of a
  curve crosses nothing the whole did not), so it is not asked again.
- **Width along a split edge.** An edge split again and again grows wider
  with each split, and its next crossing comes out wider still. A rim of
  four quarter arcs failed once a quarter was crossed by about fifteen
  gaps; drawn in twelve arcs it is not. Where an operation will cut one
  edge many times, draw it in pieces, ending where no cut will land.
- **Traced fits.** An involute's curvature changes tenfold along the
  flank, so the cubic through the march's points drifted a hundred times
  the usual; the legs are now split finer while the drift is what makes
  the curve wide.

The sketch solver is the other trap: a sketch whose arcs' sweeps must be
solved for, with points far from where they were drawn, can be sent off
to infinity. A sketch every point of which a formula places — lines,
Béziers, arcs whose sweep does not change — is placed in one step for any
size.

## A step sized by the tightest bend never arrives

`adaptive_step_size` shortened every marching step so a full turn of the
tighter surface would take 64 of them. At a cone's apex the tightest bend is
the circle round the axis, which shrinks with the distance to the apex. A
curve running along a generator into it, a straight line, took a step a
fraction of its distance to the apex each time, and never got there: 359
steps, spaced geometrically, for a line. The interpolation through those
points then widened exponentially along the chain, to 111 in the control
points, and the splice failed on a "corner" it could not decide.

A surface that runs straight the way the curve does (`curvature_radius_along`
reports none) bends it by nothing, and no longer limits the step. Going
further, and sizing the step by the bend along the curve for every surface,
was tried and is wrong: where the surfaces bend a curve a little here it may
bend sharply a stride on, and the pattern of `turbine_blade` left its patch
(the slow `example_mass_properties_are_consistent` caught it). Look at the
number of steps a trace took: a line that took hundreds went wrong before it
was fitted. A trace that records its widest marched point and its widest
true point between them (see the context in `trace_one_side`) tells where a
wide curve got wide.

## A pole is one vertex at many `(u, v)`

A face's loop passes a pole once at each corner of its collapsed row, and
along the row (a `CoedgeGeometry::Vertex` coedge), each at a `(u, v)` of its
own. Which pass an edge arrives at is told by *its* `(u, v)`, not by the way
it leaves the vertex, and not by pinning its end to the first coedge that
arrives there: that is a corner at the row's end, and the edge is bent to it.
`splice_edge_into_face` leaves the end of an edge at a pole unpinned, cuts
the row where the edge's pcurve ends and splices after the first half
(`split_pole_row`).

## Faces a union leaves in two are one wall

A plate standing flush with the side of what it stands on is two faces in
one plane, and a corner on that side has four faces, not three. What the
fillet needs of the third face — a plane, square to the edge — is true of
both. `tool_end` counts faces of one plane as one wall. A refusal says which
faces meet where (`corners_of`), so the next corner it cannot take is named.

## A reference direction is not left to noise

A joint measures its turns from a direction square to its axis, chosen from
the world's axes as the one the axis runs least along. An axis along `z` is
a hair off it in `x` and in `y` alike, so which of the two is the least was
decided by noise, differently for the two ends of a joint: a quarter turn
or none. `across` takes the first axis that no other component is
definitely less than. A frame put on a face or an edge gives the same
direction as the joint would have chosen for it (`axis_frame`), so frames
and the entities they are on are at a turn of zero alike.
