# Lecture 20: Validation

<div class="goals">

**After this lecture you can**

- say what it means for a B-rep to be valid, as a list of checkable properties, and for each one what breaks if it fails;
- choose between `validate`, `validate_fast` and `validate_manifold`, and read their reports;
- explain why the validator reports everything it finds and what order it runs in;
- distinguish a check that proves a defect from one that only fails to find it;
- use `surface_could_contain` as a diagnostic, and turn a validation failure with ids into one with names;
- describe the debugging habit AGENTS.md demands, naming the entities before naming a cause.

</div>

In a B-rep, geometry and topology are separate data that must agree. Most of those agreements cannot be enforced by a type. A pcurve *should* run where the edge runs, but nothing stops it from being anywhere. So any operation that edits a model can leave it subtly wrong in ways no subsequent code will notice until it fails somewhere else, much later, for what looks like an unrelated reason. Validation is what closes that distance.

This lecture is shorter on mathematics, and long on the way of thinking that makes a kernel like this possible at all.

## Why a bad model is worse than a crash

Consider this chain, from AGENTS.md. A curve traced along a face's trim boundary came back with a sharp `(u, v)` a tiny distance off the boundary. `face_contains` said `Inside`. The curve was spliced in, a duplicate of the boundary, carving a zero-area sliver. That face was "structurally perfect": its loop closed and its pcurves were continuous. It surfaced much later as a face a boolean could not classify because it had no interior point.

Note the distance between the cause (a sharpened value inside the corrector, [Lecture 16](./16-tracing-and-pcurves.md)) and the symptom (a failed classification in a later operation). Every piece of code in between was innocent. A bug report would have named the last of them.

Validation shortens this chain, ideally to one step. Its job is to fail right after the operation that made the mistake, and to say what is wrong in terms of entities.

## The properties of a valid model

Here is every check in `core/geop-core-topology/src/validation/`, ordered from cheapest to most expensive, with what each one is for.

### Pure structure

**Pointers** (`pointer_check.rs`, `check_pointers`). Every id referred to anywhere exists in its arena. This is the most basic sanity check, and the only one that makes walking the model safe at all.

**Two-way references** (`two_way_references.rs`, `check_two_way_references`). Whatever points one way is mirrored the other. `next` and `prev` are inverses. A face's `shell` lists that face. A coedge in a face's loop names that face, and that face's boundaries reach it. A wire owns its edges and vertices alone.

**The edge rule** (`manifold_check.rs`, `check_every_edge_has_paired_coedges`). Every edge of a solid is used by an even number of coedges, as many one way as the other: two where two faces meet, and four or more where a solid touches itself along an edge (two blocks sharing only an edge, a boss tangent to a wall along a line). An odd count is a hole in the shell. Unbalanced senses are "a torn seam rather than a shared one". A sheet's border and a wire's edges are exempt.

So "manifold" here is a generous notion: the validator allows a solid to touch itself along an edge. This is deliberate. Such solids genuinely arise, and containment does not care, because its rays graze no edge.

### Geometry against topology

**Curves meet their vertices** (`check_curves_and_surfaces_vertices.rs`). An edge's curve starts at `start_vertex` and ends at `end_vertex`. A coedge's pcurve, evaluated through its face's surface, starts and ends at those two points, in the order its `sense` implies. All by `could_be_equal` on enclosures.

**Loops close in `(u, v)`** (`pcurve_loop_continuity.rs`). Each coedge's pcurve ends where `next`'s starts, *in parameter space*. The check above does not imply this at a pole: a coedge ending at `$(1,0)$` and the next starting at `$(0,0)$` both map to the same point, and the loop then jumps the row. "That's invisible to any single-coedge check, but fatal to `contains::face::face_contains`'s ray-casting." ([Lecture 18](./18-poles-seams-splicing.md).)

**Pcurves trace their edges** (`curve_and_surface_sampling_check.rs`, `check_curve_and_surface_sampling`). Along the whole length: samples `sample_count` points along the pcurve, mapped through the surface, and along the edge, at matching fractional positions of each one's domain, and requires them to be `could_be_equal`. A `Vertex`-backed coedge's samples must all map to its vertex.

**Numerical accuracy** (`numerical_accuracy.rs`, `check_numerical_accuracy`) has its own section below.

### Relative position

**Holes are inside the outer loop** (`holes_inside_outer.rs`). A hole outside the outer loop subtracts nothing, and one straddling it subtracts a region that was never there. Both leave every structural invariant intact. One point per hole decides this, because `check_edges_disjoint` already rejects crossing loops.

**Winding** (`face_orientation.rs`, `check_loop_winding`). The outer loop runs counter-clockwise in `(u, v)`, every hole clockwise, by the exact `loop_signed_area`. "This is the convention the kernel *computes with*, not merely a tidiness rule." A dangling ring, whose every edge is traversed twice, encloses no area and is exempt (`traverses_each_edge_twice`).

**Normals point out** (`check_normals_point_outward`). At a point of each face, `solid_contains_beside` says which side of the face the solid is on ([Lecture 19](./19-classification.md)), and it must be opposite the normal. An inverted face "still validates structurally, still renders, and still has a perfectly good boundary".

### Interference within a body

Three checks look for places the topology failed to record where things meet. They are limited to entities *of one body*, because separate bodies may overlap as they please (the two halves of a split, a new body extruded into another). `body_groups` in `core/geop-core-topology/src/validation/mod.rs` does the grouping.

**Vertices are disjoint** (`disjointness_check.rs`, `check_vertices_disjoint`). No two distinct vertices of a body could overlap. "A shared position should be represented by reusing the same `VertexId`, not by two coincident ones."

**Edges meet only at shared vertices** (`check_edges_disjoint`). Every crossing found by `curve_curve_intersect` must be at a vertex, and coincident edges always fail.

**Edges pierce faces only at shared vertices** (`check_edges_and_faces_consistent`). Similarly, by `curve_surface_intersect`, skipping pairs where the edge is declared a boundary of the face. An edge entirely embedded in a surface is a continuum of coincidence, and the search would not settle. Those pairs are the sampling check's job.

**Faces do not meet away from edges** (`face_face_numerical_intersection.rs`). For each pair of non-adjacent faces, random starting pairs are put through alternating-projection Newton. A pair that converges is unremarkable: "two coplanar faces of one solid share their entire plane". The violation is a converged point strictly inside *both* trims, meaning an intersection is missing from the topology.

**Consistency of rays** (`check_ray_direction_consistency`). In `validate_manifold`, for each shell, random points are classified along random rays. The directions should not matter. They should all agree, with one dissenting vote tolerated, since near a boundary one numerically fragile ray is not a topology defect. This exercises the strategy of [Lecture 19](./19-classification.md) directly.

### One that only `validate_fast` runs

**Faces have interior** (`faces_have_interior.rs`). Every face has a point strictly inside its trim, using `face_interior_point`. That is exactly the sliver story above. It is "deliberately in `validate_fast` rather than the full `validate`, even though it casts rays: a degenerate face is a *construction* defect", and the sweep that produces one reports it at once.

## Three entry points

`core/geop-core-topology/src/validation/mod.rs` has two of these, and `manifold_check.rs` the third.

| Function | Runs | For |
|---|---|---|
| `validate_fast` | accuracy, pointers, two-way, curves and vertices, loop continuity, faces have interior | after every step of a long run or a sweep |
| `validate` | the above without faces-have-interior, plus sampling, disjointness, face-face, holes inside outer, winding, normals | a finished model |
| `validate_manifold` | `validate`, plus the edge rule and consistency of rays | a finished solid |

`ops/geop-ops-model/src/testing.rs` (`assert_valid`, `check_valid`) uses `validate_manifold`, and `validate` for models with open bodies (`assert_valid_sheets`). It is what a test in any op crate calls, and it uses the defaults in `ValidationParameters`. The notes there are worth reading (`core/geop-core-topology/src/validation/parameters.rs`): the pairwise searches need a far looser tolerance than a single containment query to converge within `max_nodes`, because a near-tangential pair's clipping barely discriminates, and each halving of the tolerance costs at least one extra level. So `min_subdivision_size` is `1e-4` there. This is the only kind of tunable allowed: how hard the search tries.

### Report everything

"Every check function takes `(params, errors, model)` and pushes its own violations onto `errors` rather than returning on the first one." The value of that shows up as soon as you read a real report, since the pattern of the errors says more than any one of them. AGENTS.md's fit_pcurve story turned on such a pattern: "32 failures, every one with exactly one endpoint on-surface and one off."

### Numerical accuracy runs first

`check_numerical_accuracy` requires that no stored coordinate has a width over `MAX_WIDTH`, `1e-4`. It is no comparison tolerance, and nothing is tested for equality against it. It is a bound on how much uncertainty an entity is allowed to *carry*. A vertex wider than a containment search's resolution cannot be located on anything, and a control point that wide makes a curve whose hull no longer pins it down.

It runs first because it explains the rest. "A too-wide entity does not fail in place: it fails somewhere downstream, as a containment search that finds nothing, a pcurve that will not match its edge, or an intersection that is missed entirely, and the report names that distant symptom rather than the cause." Its message suggests the likely cause: "something that produced it skipped a refinement".

The same bound is a minimum on how long an edge may be. "An edge shorter than the accuracy its own endpoints carry is not a feature of the model, it is noise." And the placeholder surface `NurbSurface::everything` is reported as an unbounded width, since a finished model must not contain one.

<div class="kernel">

**In the kernel.** `MAX_WIDTH` is public, because what builds an entity may need it. The boolean's exact arc is one of two enclosures of a curve, taken only where it fits the budget.

AGENTS.md's section on widths ([Lecture 11](./11-evaluate-locally.md)) tells how you check accuracy in a test: not just that an enclosure holds the right value, but `vertex.point[k].width().to_f64() < 1e-9`, because in `f64` an enclosure's growth hides behind the format's relative precision until the width is read.

</div>

## What a passing validator proves

Be honest about the logical status of each check. There are two kinds.

**Exact checks** are decisive either way: the pointers, two-way references, edge rule, loop closure as far as the enclosures say, and winding where the area's sign is decided.

**Searches and samples** are one-sided. If one finds a defect, that is a proof, and it names where. If it finds none, you know less: the sampling check looks at 17 points along each pcurve; the face-face check at 20 random pairs per pair of faces. AGENTS.md's rule "Do not decide from samples" is about deciding *answers*, and the kernel's decisions are not made here. But a validator that passes has not proved a model correct. `body_crossing`'s doc comment even says it directly: "`None` if no edge does, which does not rule out two faces crossing along a curve no edge of either reaches."

The practical stance is that of a test suite: it is a sieve, not a proof, and a defect found some other way should become a check or a reproducer. Several checks say in their doc comments which failure they are there to catch: `holes_inside_outer.rs` names `splice_edge_into_face`'s hole reclassification, and `faces_have_interior.rs` a face split along an edge that already coincides with a boundary.

## Proximity is not membership

Here is the validation story that gives this rule its name in AGENTS.md. Thirty-two `validate_fast` errors appeared, about pcurve endpoints not matching their 3-D points, on 4 faces.

Why? A traced curve ends at a vertex. `candidate_within` chose the nearest vertex within one marching stride. But an oversized solid puts plenty of unrelated vertices within a stride of a curve's end. Adopting one ended the edge at a point on neither face. `fit_pcurve` then had nothing to project onto, and silently returned the nearest foot point, about `8e-3` away.

The error text was identical to what you would see for an edge spliced to the right face with the wrong `(u, v)`. The distinction is what points to the fix:

- "The pcurve names the wrong `(u, v)` for a point that is on the surface": the numbers are at fault.
- "The edge was spliced onto a face it does not belong to": the logic is at fault.

One extra `surface_could_contain` (`core/geop-core-geometry/src/contains/surface.rs`) in the error path separates them. That turned "a guess into a measurement": every one of the 32 had exactly one endpoint on the surface and one off. The fix followed: a vertex must lie on *both* surfaces to end a trace.

So when you write a check, make its failure message answer the next question. A diagnostic that reports whether the point lies on the surface at all pays for itself the first time it fails.

## Name the entities before you name the cause

A failed validation mentions ids: `vertex 6123`, `FaceId(88)`. These mean nothing to a person. So `NamedModel::names_mentioned` (`ops/geop-ops-model/src/named_model.rs`) takes a report's text, finds each id mentioned, and returns `id = name`. The test helper `assert_valid` appends it ("where ...") through `named_report` in `ops/geop-ops-model/src/testing.rs`, and each `regression_tests.rs` that builds a user's reproducer has an `assert_builds_valid` calling it (for example `ops/geop-ops-extrude-revolve/src/regression_tests.rs`).

The names come from the operation: `extrude(hole,hole_sketch,c1,start)`, the rim arc at the start of the hole's wall. So a report says what is at the failing vertex, in the language of the user's own program. `combine` in `lib/geop-lib-booleans/src/boolean.rs` adds the same to any remesh error.

Why it matters, from a real session in AGENTS.md. A revolve cut "up to next" failed with a vertex too wide to validate, and it was explained to the user as a near-tangency. The user looked at the scene: there was none. Naming the vertex showed it was where the drilled hole's rim crossed the revolve's *end face*, and the end face was there because the operation's own fallback had stopped the tool flat at the first point of contact, which was an existing corner. The bug was a logic error in our own code, not numerics.

So, the habit:

1. Do not state why something failed until you have named the exact entities at the failing vertex or edge, by name, and checked your explanation against them.
2. Ask first whether something *our code chose* (a stop position, a cap, a split point) put the entities there.
3. Blame numerics only after that is ruled out.

"Near-tangent crossing" and "imprecise near a shared vertex" sound like explanations, and they cannot be checked. Lecture 42 collects more such cases; the same rule also applies to your explanations in your own head.

## Context in the real call stack

A failure far from the top of a call stack needs its context. AGENTS.md's way, which you see everywhere above: wrap fallible calls with `.with_context(&ctx)?`, `ctx` a small closure, as in `Model::split_edge_at_vertex`:

```rust,ignore
let ctx = |e: GeopError| {
    e.with_context(format!(
        "Model::split_edge_at_vertex(edge_id={edge_id}, edge_t={edge_t}, vertex_id={vertex_id}, ...)"
    ))
};
```

It stays in the code permanently. It is not a debugging print, for which AGENTS.md has an unambiguous rule: do not add `eprintln!`, ad hoc test harnesses, or temporary prints to trace a bug. If existing context is not enough, more belongs in the real code path, "and helps the *next* failure of the same kind self-report". Use `{:?}` for numbers in such messages, since `Display` on an interval scalar shows three decimals of the midpoint and hides exactly the width you are diagnosing.

## Reproduce first

The last piece of the method is what you do with a validation failure. From AGENTS.md's *Isolate a reproducer before fixing anything*:

1. **Isolate.** Pull the failure out of the sweep or pipeline into a standalone `#[test]` failing for exactly that reason.
2. **Fix**, with the reproducer red, so you can tell a fix from a moved symptom.
3. **Verify both**: the reproducer goes green and the broad suite stays green.

Keep the reproducer, named after the scenario, not the bug. A user's `.geop` file is rebuilt in Rust, in the `regression_tests.rs` of the crate whose operation failed, and confirmed to fail without the fix. Each regression test there is a defect the kernel can no longer have. `ops/geop-ops-extrude-revolve/src/regression_tests.rs` has many examples, with documentation explaining each case: read the doc comment above the test for a 5.6 by 3.7 plate in that file, as a model of what a good one says.

<div class="kernel">

**In the kernel.** Sweeps test validity at scale: `lib/geop-lib-booleans/src/scenes.rs` builds pairs of solids, and `lib/geop-lib-booleans/src/render_test.rs` runs each through remesh and every operator. The `stress_tests.rs` of `geop-ops-fillet` and `geop-ops-shell` blend every edge and shell every face of a set of bodies (`geop_lib_fixtures::bodies`). AGENTS.md: their first run found five shell bugs no hand-written test had reached. They tell a refusal apart from a failure, and require that whatever is built is valid.

</div>

<div class="recap">

## Recap

- Validity is a long list of agreements between geometry and topology, most of which no type can enforce. Each check in `validation/` guards one, for a reason you can name.
- `validate_fast` is structural and cheap, `validate` adds the pairwise searches, `validate_manifold` adds closure and ray consistency.
- Accuracy runs first, because a too-wide entity fails elsewhere; every check reports everything.
- An exact check is decisive. A search or a sample proves a defect when it finds one, and proves little by finding none.
- Name the entities first (`names_mentioned`), ask whether our own code chose them, and only then blame numbers.
- A durable reproducer goes with every fix.

</div>

## Exercises

**1.** A model passes `validate_fast` and fails `validate` with "edge X crosses face Y" at a point away from any vertex. What kind of operation produced this, and how would you begin?

<details class="solution"><summary>Hint</summary>

`check_edges_and_faces_consistent` found an intersection that the topology does not record. Something built or moved an edge without imprinting the crossing, or a remesh missed a piercing. Name the two entities with `names_mentioned`. Then find which step made them, and check whether it is within one body: remember that separate bodies may overlap. Only then look at why the search that should have found it did not.

</details>

**2.** Why does `validate` skip adjacent pairs in the face-face check? Give a concrete false positive it would produce.

<details class="solution"><summary>Solution</summary>

Two neighbouring faces' *surfaces* generically meet along the extension of the shared edge, beyond the edge itself. The alternating-projection search does not know about trims as it iterates, so it can converge on a point on that extension. For two adjacent planar faces of a box, each plane runs on past the edge; a converged point is on the line through the edge. The check then requires it to be strictly inside both trims, which would filter most of them, but diagonal quadrants of a revolved pole, which share only a vertex, are also skipped because of artifacts "of already-expected adjacency". The doc comment on `adjacent_face_pairs` says that shared edges are validated elsewhere.

</details>

**3.** Suppose you are told "a face has a loop of positive signed area but a hole which is counter-clockwise". Which checks fail? What does the same-hole splice do on such a face, and why would no structural check see it?

<details class="solution"><summary>Solution</summary>

`check_loop_winding` fails. No other check depends on winding: parity needs no orientation, and pointers and continuity are unaffected. A same-hole splice on it picks "the ring of positive area bounds material", so it would call the wrong ring a new face. The result would be a face whose outer loop goes around a void, and whatever asks which side the material is on gets the opposite answer. This is why winding is called a convention the kernel computes with.

</details>

**4.** Suppose a report says one of the following (both messages are made up for this exercise). What would you do first in each?

(a) `vertex 812's coordinate 2 is [..], 3e-3 wide`.

(b) `pcurve of coedge 901 ends at (0.25, 1.0), surface there is 2e-2 from vertex 77`, together with `surface_could_contain(vertex 77) = None`.

<details class="solution"><summary>Solution</summary>

(a) Find what produced the vertex: its name says which operation. A width that big is far above a Newton-refined crossing's, so "a missing refinement" comes first; read `check_numerical_accuracy`'s doc comment, and find the search whose raw enclosure was stored. (b) The vertex is not on the surface at all, so this coedge is on a face it does not belong to, or the vertex was wrongly adopted. It is logic, not numerics. Look at what chose that vertex: the `candidate_within` story. If `surface_could_contain` had returned a `(u, v)`, then the pcurve's numbers would be the suspect instead.

</details>

**5.** Propose a new validation check that decides whether a loop's pcurves are simple (no self-crossings) in `(u, v)`. What does it catch? What should it use, and what would it have to exempt?

<details class="solution"><summary>Hint</summary>

Use `curve_curve_intersect` over pairs of a loop's pcurves, as in `check_edges_of_body_disjoint`. It must allow a crossing at a shared end, and must exempt spurs and a loop running along a slit twice, where a pcurve coincides with its own reverse (compare `traverses_each_edge_twice`). A pole's row also appears here. Consider cost, and whether it goes in `validate_fast`. Ask too what real failure it would have caught: a check earns its cost with a reproducer.

</details>
