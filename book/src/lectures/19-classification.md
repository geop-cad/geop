# Lecture 19: Inside or outside

<div class="goals">

**After this lecture you can**

- prove the parity rule for points and closed curves, and say exactly when it needs a retry;
- describe how Geop classifies a `(u, v)` against a trimmed face and a 3-D point against a shell and a solid, with all the outcomes;
- explain why a ray from a box is wrong, and how to spot other computations whose answer means nothing for a wide input;
- compute a loop's signed area exactly by Green's theorem and Gauss–Legendre quadrature, and construct a case where a sampled polygon gets the sign wrong;
- decide which side of a face a solid is on without probing a distance off it.

</div>

A boolean's central question ([Lecture 22](./22-boolean.md)) is whether a point lies inside another solid. Face containment asks the same inside a surface's parameter plane, and face splitting, interior points, tracing, validation and picking all rest on it. So we should get it exactly right, including what "right" means when the point, the curves and the arithmetic are all enclosures.

## Parity

Let `$\Gamma$` be a closed curve in the plane without self-crossings. By the Jordan curve theorem, it divides the plane into an inside and an outside. Take a point `$q$` not on `$\Gamma$`, and a ray from `$q$` to infinity, and count the times the ray crosses `$\Gamma$`.

**Claim.** If every crossing is transversal, `$q$` is inside exactly when the count is odd.

*Proof.* A point on the ray far enough out is outside. Walking back towards `$q$`, each transversal crossing flips between inside and outside, and nothing else does. So `$q$` is outside after an even number of flips. `$\square$`

A face has several loops, one outer and some holes. The same argument applies to the union of the curves, the "inside" of the union being exactly the material: inside the outer loop and outside every hole. So you do not even have to treat holes separately, or know which loop is which. It needs no orientation at all.

The proof has one assumption, and everything interesting lives in it: **transversal**. Two things can go wrong.

- The ray passes through a *vertex* between two coedges. Does that count as one crossing, on either side of the vertex? Two, one on each? Zero, if the curve turns back there? The answer depends on the corner's shape.
- The ray is *tangent* to a curve. It touches, stays on one side, and does not flip, though a search sees one hit.

The textbook response is to classify these cases and count half-crossings correctly. In interval arithmetic that is hopeless. You often cannot tell whether a ray passes through a vertex or a hair beside it, and you cannot tell a tangent from two close crossings. Geop takes the far simpler view: **do not answer from that ray**. A direction was a free choice. If it is ambiguous, choose another.

A random direction misses any given vertex with probability 1. The chance of a wide-enough near-tangency is small. So a few directions suffice. Note how the rule "ask only clear questions" lets correctness rest on a much weaker requirement than "handle every degenerate case".

## Classifying against a face

`face_contains` (`core/geop-core-topology/src/contains/face.rs`) returns a `PointClassification`, one of `OnVertex`, `OnCoedge`, `Inside`, `Outside`. Its work is done in `pcurves_contain`, which works in order:

1. **On a vertex?** For each pcurve, `could_be_equal` of its start and the query. (A bare-vertex boundary is checked first, in `face_contains`.)
2. **On a pcurve?** `curve_could_contain` on each. Both are complete passes, so that `OnVertex` always beats `OnCoedge`, regardless of iteration order. Those two are *full* passes because a first match in iteration order would make the answer depend on the order.
3. **Otherwise**, the query touches nothing. Up to 64 times (`MAX_RAY_ATTEMPTS`), draw a direction from a seeded generator (`core/geop-core-topology/src/contains/rng.rs`), make a straight ray long enough to leave the domain, and:
   - reject the direction if it could pass through any pcurve's start;
   - run `curve_curve_intersect` against each pcurve, rejecting on budget exhaustion (a search that does not settle is itself evidence of a near-tangent or overlapping ray, and no count of it is reliable);
   - reject if a crossing is at a pcurve's end (a graze that slipped past the first test);
   - otherwise count.

The seed is a parameter and the result does not depend on it, if things are as they should be. Fixing it makes a run reproducible. If no direction works, it returns an error saying *why the last was rejected*, "since 'no clear direction' alone says nothing about the cause".

### A crossing right next to the query

The search may return a crossing it cannot place past the query. It might be a real one just beside it, and it might be the search reporting at its own resolution. The code's history is instructive. It used to discard every hit within `epsilon` of the query. That is an epsilon comparison, as forbidden by AGENTS.md, and it "turned a point `6e-5` outside a face into one inside it". Now the crossing is refined by Newton (`refine_curve_curve_crossing`). If it then lies past the query, it counts. If the refined crossing still cannot be told from the query, the point is on the boundary as far as the numbers can tell, and the answer is `OnCoedge`: "A point `4e-16` inside a face met its boundary so along every ray, and was never classified." Otherwise, the direction is rejected.

### Ask a question about a box only where its answer holds for a box

The query `(u, v)` may be an interval. Steps 1 and 2 handle that honestly: "could this box touch a vertex or a pcurve?" has a meaningful answer for a box. After them, the box is known to touch no boundary, and being connected, all of it lies on one side.

Now step 3. Suppose you cast a ray from the box as given. In interval arithmetic that ray is a *strip*. A strip that merely grazes a pcurve overlaps it, and overlap is what a crossing looks like, so a search returns one hit. A true ray from any single point of the box misses that pcurve or crosses it twice. Either way the parity is even, and the strip's parity is odd. Wrong.

That is not a thought experiment. It is AGENTS.md's case: a point `$0.44$` from a hole's axis, with a rim of `$0.33$`, was found inside the disc cut from a plate's top, and the boolean kept that disc and left the result open. `face_interior_point_where` had asked about a box of `$\pm\varepsilon$` around a candidate point, for the good reason that the accepted point should keep clear of the boundary. The fix is one line in `pcurves_contain`: a sharp `origin` at the box's midpoint.

Play with it below. The strip's hits are the circled ones.

<figure class="demo" data-demo="ray-parity">
<figcaption>Drag the white point and the hole, and turn the ray. The sharp ray's crossings are red, and their count's parity gives the answer. "Aim at a vertex" makes the direction go through a pcurve's start: the kernel's rule rejects it, and a new direction is drawn. "Graze the hole with a strip" sets up the bug: a sharp ray misses the rim, and the strip of half-width 0.06 overlaps it once. The search is the convex-hull subdivision of Lecture 12 and the loops are cubics, but the demo's tolerance and its comparison are plain floats, and its circle is the usual cubic approximation, not the kernel's exact arc.</figcaption>
</figure>

The general rule, as AGENTS.md states it: before you run an interval computation on a wide input, check that its answer still means something for every point in that input. "Could this touch that?" does. Parity does not: counting crossings only makes sense for a ray from a single point.

## Finding a point in a face

Classifying a *face* against another solid needs a point in it. It is harder than it sounds, because a face carved out by a boolean need not contain the midpoint of its surface's domain, and "very often does not".

`face_interior_point_where` starts at the boundary. For each coedge of the outer loop, take the middle of its pcurve, step along the two perpendiculars to its tangent in `(u, v)`, one of which points inside. Halve the step until a `face_contains` says `Inside`. It tries both perpendiculars rather than deriving which is inward from the loop's winding, "cheaper and more robust". It requires the candidate to be clear of the boundary by a margin that starts at the search's resolution and gets finer if needed (from `1e-4` to `1.5e-9` of the surface's parameters): a fillet's corner piece on a face a hundred times its size has interior points all the same.

The candidate is sharp: any point inside the face will do, which makes it a free choice.

The same function has a second lesson. `accept` lets the caller refuse a candidate and take the next. One coedge per candidate gives "two genuinely different points" even when one area of the face is awkward. A boolean uses that ([Lecture 22](./22-boolean.md)): a point on the other solid's boundary says nothing, so it asks for another.

## Classifying against a solid

`core/geop-core-topology/src/contains/shell.rs` is the 3-D analogue, with a module comment that sums it up. Coincidence is checked first, *vertex, then edge, then face*: each a full pass. Then a ray in a random direction, drawn from a seeded PRNG, retried whenever it grazes a vertex or an edge, "both ambiguous to count reliably: an edge is shared by two faces, a vertex by several edges", until its only crossings are clean face-interior hits. The parity then decides, needing no oriented normal.

Note the checks in `cast_ray`. It uses `curve_could_contain` on each vertex and `curve_curve_intersect` on each edge before it counts any face crossing.

A shell can answer alone (`shell_contains`). For a solid, `solid_contains` combines its shells. A point is inside the solid when it is inside an odd number of them: the shells of one solid never cross, so that is inside the outer shell and no void. Each shell is asked on its own, and the insides XOR together. A point on any shell's boundary returns at once.

### Being asked many times: a short path

A boolean classifies hundreds of points against one solid. Casting a ray across the whole solid for each is wasteful. AGENTS.md's second performance round found it: `shell_contains_from` asks instead about a *segment* from a point already classified. If `$p$` and `$r$` are joined by a path crossing the boundary an even number of times, they are on the same side. A short path rules out nearly every face of the shell with a box test, before any search runs. `Classified` is the memory of what has been decided; `solid_contains` takes one, and uses the nearest point in it.

There is a trap. A straight segment between two points of one face lies in that face's plane, and in any face flush with it. Two corners of a symmetric face can graze a cylinder, "a crossing that counts once and is none". A path between structured points is not generic. So the path goes by way of a random point `via`, "as far from the middle as the two are apart": two legs, each like a ray. "A random detour is as clear of these as a random ray is." A degenerate path falls back to full rays.

### Which side is the solid on?

Here is a question that looks as if it needs a distance, and does not. Given a face, which of its two sides is the solid on? (Validation asks this, to check that normals point out.)

The obvious way: move a little off the face along the normal and classify that point. *How little?* Any answer is a tunable that decides an answer, which is what AGENTS.md's *A probe is a sample too* forbids. It went wrong in the following way. A plate placed at `$(10000,-5000,2000)$` had its extent taken over all three axes, as 15080 for a plate 80 long. So the distance was halved only so far, and the one probe that resolved, at 29, went from a bore's wall out through 18 of plate on one side and across the bore into plate on the other. On a non-convex solid, a probe of any fixed size can reach through into another part of the solid.

The limit as the distance goes to zero is what the question means, and a ray *leaving the face* answers that limit exactly. Every point of the ray just past the face is on the normal's side. So `solid_contains_beside` draws a direction, turns it into the normal's half space (rejecting one that could run along the face), and counts the crossings past the face (`leaving` tells `cast_ray` not to count the start). Odd means the solid is on that side. No distance at all.

The start point is a free choice, and the first offered may be a poor one: the centre of a cube's face, where an inscribed sphere's pole lies, with no topology between them. `on_solid_boundary_besides` rejects a point that is on the boundary anywhere else as well. The answer, `Beside`, keeps the ray's crossings with the verdict, to be named in an error. AGENTS.md: that is what showed the subd mug's handle patch to be a ribbon folded onto itself, a ray leaving it crossing its other half `6e-7` on, which no probe had seen.

## Which way does a loop run?

Containment by parity did not need orientation, but other things do. The winding rule (outer counter-clockwise, holes clockwise) is what the same-hole splice computes with. So how do you decide which way a loop of NURBS curves runs?

The signed area enclosed by a closed curve is, by Green's theorem,

`$$A=\frac12\oint\bigl(u\,dv-v\,du\bigr)=\frac12\oint\bigl(u\,v'-v\,u'\bigr)\,dt,$$`

positive when counter-clockwise, and the sum over a loop of a term per pcurve.

### Exact, because it is a polynomial

On a span of a non-rational pcurve of degree `$p$`, `$u$` and `$v$` are polynomials of degree `$p$` and `$u',v'$` of degree `$p-1$`, so the integrand has degree `$2p-1$`. A Gauss–Legendre rule of `$m$` points integrates a polynomial of degree `$2m-1$` exactly. So **`$m=p$` points integrate it with no truncation error at all**. A cubic span needs 3 points. The only error left is rounding, and interval arithmetic already bounds that. The result is proven.

Here is what that looks like, for one cubic span with control points `$P_0,\dots,P_3$`, on `$t\in[0,1]$`. The rule's nodes are `$\tfrac12\mp\tfrac12\sqrt{3/5}$` and `$\tfrac12$`, its weights `$\tfrac5{18},\tfrac8{18},\tfrac5{18}$`:

`$$\int_0^1 f(t)\,dt\approx\tfrac{5}{18}f\!\left(\tfrac12-\tfrac12\sqrt{\tfrac35}\right)+\tfrac{8}{18}f\!\left(\tfrac12\right)+\tfrac{5}{18}f\!\left(\tfrac12+\tfrac12\sqrt{\tfrac35}\right).$$`

Exercise 3 checks it by hand.

A rational curve, an arc say, has a rational integrand, for which no finite rule is exact. Then adaptive Gauss–Kronrod estimates the truncation error, and it goes into the enclosure (`core/geop-core-math/src/quadrature.rs`).

### Why not a polygon?

The obvious alternative is to sample each pcurve at a few points and use the polygon's shoelace formula. AGENTS.md: "Which way a loop runs was decided from a polygon of eight points per pcurve, in five places with five sample counts: a pcurve fitted in fifty spans was looked at in eight of them." The polygon's area is that of a different curve. The gap between them is part of no enclosure. So a thin loop can come out with the wrong sign, *definitely*.

To construct that, take a sliver: a straight side along `$y=0$` from `$(0,0)$` to `$(1,0)$`, and a return along `$y=a+b\cos(16\pi x)$`. It is a wiggle with 8 periods, pinned to 0 at the ends. Its true mean height is `$a$`. Sampled at `$x=i/8$`, all of the interior samples read `$a+b$`. With `$a=0.02$` and `$b=-0.05$`, the true area is near `$+0.02$`, while the polygon's is near `$-0.026$`: it is the wrong way round, and the number gives no hint of that. This sliver is built to make the point, but its ingredient, a pcurve of many spans looked at in few samples, is what the kernel really had.

<figure class="demo" data-demo="loop-area">
<figcaption>Green's theorem against a polygon of samples. Drag the control points of the blob: both numbers agree better as the samples increase, the polygon's error dropping with the square of the count. Then choose the sliver, a fit of 64 cubic spans: at 8 samples per pcurve, the polygon has the wrong sign. Slide the samples up and watch it recover. All curves here are polynomial splines; the kernel's rational curves are integrated adaptively instead.</figcaption>
</figure>

<div class="kernel">

**In the kernel.** `nurb_curve::loop_signed_area` and `NurbCurve::area_term` are in `core/geop-core-geometry/src/nurb_curve/area.rs`, and `uv_loop::loop_signed_area` in `core/geop-core-topology/src/uv_loop.rs` runs it over a face's loop. `is_polynomial` chooses the exact rule: "the same enclosure, not merely overlapping ones". The integration is done with the curve moved to a chosen `origin`, as the terms of one loop must share one; compare [Lecture 11](./11-evaluate-locally.md).

The rasterizer is the one place polygons are right, because what it draws is a polygon: `sample_loop_to_polygon` in `uv_loop.rs`. "Ask the area which way a loop runs, never a sampled polygon." AGENTS.md's *Do not decide from samples* states the rule for the whole kernel. Mass properties in `core/geop-core-topology/src/mass.rs` integrate the same way, over the surfaces.

</div>

## Ask the question the topology can answer

One more story, from the viewport. The editor's picker decided whether an edge was hidden by casting a ray from the pointer: an edge counted as hidden when it lay more than one pointer reach behind the face hit, measured along the ray. At a glancing look, an edge of that very face lies further along the ray than that, so it could not be hovered. "Is this edge in front of the face?" stood in for "does this edge bound the face?", and the topology answers that exactly (`ViewEdge::faces`).

Classification is full of such substitutions. It is tempting to use a distance, a sample or a nearby point when an exact question is available. Before you measure, ask whether what you want to know is already stored.

<div class="recap">

## Recap

- Parity counts transversal crossings. Rather than handling a graze, reject the direction and draw another: it was a free choice.
- `face_contains` works in `(u, v)`, `shell_contains` in 3-D. Both check coincidence first, by full passes, then cast rays; a wide query is reduced to one sharp point, as parity means nothing for a strip.
- A path from an already classified point is cheaper than a ray, with a random detour to avoid structure.
- `solid_contains_beside` answers which side of a face a solid is on, as the limit a probe distance only approximates.
- Signed area comes from Green's theorem, exactly on polynomial spans by Gauss–Legendre, never from a sampled polygon.

</div>

## Exercises

**1.** A ray meets a pcurve at a point where it is tangent, with no vertex nearby. Does the kernel's parity go wrong? Trace what `pcurves_contain` does.

<details class="solution"><summary>Solution</summary>

`curve_curve_intersect` finds a cluster there, or exhausts its budget. If it runs out, the direction is rejected: "no answer at all". If it returns one solution, a graze would count once and flip the parity, and nothing in the test refuses it. So this is a real gap, made small only by the direction being random: an exact tangency of a random direction has probability 0. With intervals, the risk is a *near*-tangency within the search's tolerance, whose two crossings merge into one hit. Stating that plainly is better than pretending the rule is airtight.

</details>

**2.** Why does `face_contains` check vertices in a full pass before any pcurve? Construct an order in which a single pass would give a different answer.

<details class="solution"><summary>Solution</summary>

A query on a vertex is also on both pcurves ending there. If pcurve checks ran as you iterated and the vertex test for the second pcurve came after the pcurve test for the first, a query at the second pcurve's start would answer `OnCoedge` on the first pcurve. Which of the answers you got would depend on where the loop's anchor happens to be. With a full vertex pass first, it is always `OnVertex`.

</details>

**3.** Verify the 3-point rule on `$f(t)=t^5$`, whose integral over `$[0,1]$` is `$\tfrac16$`. Then explain why 2 points are not enough for the area term of a cubic span.

<details class="solution"><summary>Solution</summary>

With `$s=\tfrac12\sqrt{3/5}$`, the rule gives `$\tfrac5{18}\bigl((\tfrac12-s)^5+(\tfrac12+s)^5\bigr)+\tfrac8{18}\cdot\tfrac1{32}$`. Expanding, `$(\tfrac12-s)^5+(\tfrac12+s)^5=2\bigl(\tfrac1{32}+10\cdot\tfrac18s^2+5\cdot\tfrac12s^4\bigr)=\tfrac1{16}+\tfrac52s^2+5s^4$`. With `$s^2=\tfrac3{20}$`: `$\tfrac1{16}+\tfrac38+\tfrac9{80}=\tfrac{5+30+9}{80}=\tfrac{11}{20}$`. So the rule gives `$\tfrac5{18}\cdot\tfrac{11}{20}+\tfrac1{72}=\tfrac{11}{72}+\tfrac1{72}=\tfrac16$`. Exact. Two points integrate degree 3 exactly and no more, and the area term of a cubic has degree 5.

</details>

**4.** Show that the sum of `area_term`s over a closed loop does not depend on the `origin`, though each term does.

<details class="solution"><summary>Solution</summary>

Shifting the origin by `$o$` replaces `$u\,v'-v\,u'$` by `$(u-o_u)v'-(v-o_v)u'$`. The difference is `$-o_uv'+o_vu'$`, whose integral over the curve is `$-o_u\Delta v+o_v\Delta u$`, the change across it. Over a closed loop, `$\sum\Delta u=\sum\Delta v=0$`. So a loop's terms must share one origin, which then is a free choice: move it near the loop to tighten the enclosures (compare `book/scripts/demo_tests/loop-area.test.mjs`, `the_area_does_not_depend_on_the_origin`, which measures the plain floats' loss at distance 1000).

</details>

**5.** For the sliver in the demo with `$a=0.02,b=-0.05$`, estimate the polygon's area by hand at 8 samples per pcurve. How many samples are needed for the right sign, and what would a rule "just use more samples" achieve for a fit of 1000 spans?

<details class="solution"><summary>Solution</summary>

Eight points along the return pcurve, starting at `$(1,0)$` and with the end at `$(0,0)$` left out, are the start point and seven interior points, each at height `$a+b=-0.03$`, on a base of width 1. The polygon is a trapezoid, of area about `$-0.03\cdot\tfrac78\approx-0.026$`, the bottom line being in the loop's other pcurve. Any sample count not a divisor of the period resolves more; 64 samples, one per span, certainly do. "More samples" changes the sample count that fails: a thinner sliver, or a more wiggly pcurve, again aliases, and a sample count that grows with span count costs what the exact rule costs, with none of its proof.

</details>

**6.** Find another computation in this lecture whose answer does not hold for a box, and one whose answer does.

<details class="solution"><summary>Hint</summary>

The *signed area* of a pcurve with a wide control point is an enclosure and still means something, an interval of areas for the curves in it. A *crossing count* does not. "Is `t` definitely greater than 0" does. Being "the nearest vertex" does not: the rule of AGENTS.md's *Proximity is not membership*.

</details>
