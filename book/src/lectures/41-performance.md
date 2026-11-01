# Lecture 41: Counting work

<div class="goals">

**After this lecture you can**

- explain why a kernel's performance tests count operations and do not time them;
- spot a scaling bug from a count at a modest size, and write the test that keeps it out;
- exploit the structure of a problem: groups that do not interact, pendants, polynomial integrands, repeated seeds;
- design a benchmark that has the shape of the real thing, and say why the wrong shape proves nothing;
- profile a capped run with `gdb` when `perf` is not available, and keep a large test suite fast.

</div>

A CAD kernel fails its user in two ways. It can be wrong, which is what the rest of this book is about. And it can be slow in a particular way: not the slowness of a heavy computation, which a person accepts, but the slowness of an interaction that used to be instant and now is not. `AGENTS.md`'s section on this opens with a story that is a good specimen:

> A drag in a 2000-part robot went from 0.25 s to minutes, and nothing timed it. The editor builds every step's form on every change, for the step list; the added part's form asked the whole assembly how free its part was, a dense null space over every part's variables. Once per step, that is cubic in the parts.

Nothing was slow on its own. Each form was reasonable. Each dense null space was the right mathematical answer to its question. What was wrong was the *number of times the question was asked*. No profile of a small example would have found it, and no timing test would have caught it, because the test machine has its good days and because nobody writes a timing test with 2000 parts.

So this lecture has one thesis: **count the work, not the seconds.**

## Why seconds mislead

Timings are measurements of a system you do not control. `AGENTS.md` records what happens in practice:

- Two builds of the same code differ by up to 30%, from how the code is split into codegen units.
- Other jobs on the machine slow everything.
- A slow test is run less, so what it protects decays.

If you must compare times, compare CPU time, run the old and new binaries interleaved, and build both the same way. `ops/geop-ops-assembly/src/scale_tests.rs` has ignored tests that print CPU seconds for just that purpose (`cpu_seconds`), and they are run by hand.

A count has none of these problems. It is deterministic, it needs no large input, and the asymptotic shape is visible at once:

> Guard scaling with a count (`geop_ops_assembly::mates_resolved`), not a time: a quadratic shows at 80 parts as clearly as at 2000.

Let us see why. Suppose an edit does `$an^2+bn+c$` units of work. Measure at `$n$` and `$4n$`. For large `$n$`, the ratio approaches 16 for a quadratic and 4 for a linear law. Measuring a count, which is exact, you can distinguish them at quite small `$n$` as soon as the leading term dominates the others. Measuring seconds, you would need sizes where the difference exceeds the noise, which is to say slow tests.

Here is the real test, in full essence, from `ops/geop-ops-assembly/src/scale_tests.rs`:

```rust
#[test]
fn an_edit_resolves_each_mate_a_few_times() {
    let mut runs = Vec::new();
    for screws in [20, 80] {
        let resolved = |f: &mut dyn FnMut()| {
            let before = (crate::mates_resolved(), geop_lib_constraints::solve::runs());
            f();
            (
                crate::mates_resolved() - before.0,
                geop_lib_constraints::solve::runs() - before.1,
            )
        };
        ...
```

It builds a robot at 20 screws and at 80, drives real editor commands, and bounds the mates resolved and program runs per edit. The doc comment states the property: *four times the screws, the same runs.* It runs in the everyday suite. A timing test of a 2000-part robot could not.

`mates_resolved` is one counter of several. Pick the one closest to what grows:

| Counter | Counts | Defined in |
|---|---|---|
| `Runner::counts` (`Counts`) | makes, placed parts built, nodes restored | `core/geop-core-program/src/runner.rs` |
| `steps_run` | steps walked, whether or not they made anything | `core/geop-core-program/src/runner.rs` |
| `mates_resolved` | mates found and measured on the parts placed | `ops/geop-ops-assembly/src/mechanism.rs` |
| `solve::runs` | runs of the program a solve asked for | `lib/geop-lib-constraints/src/solve.rs` |

You can add one: a thread-local `Cell<usize>` is a few lines, as `MATES_RESOLVED` shows. It stays in the code. It is the next person's first instrument.

## What fixed the drag

The fixes are a catalogue of general principles, each drawn from a specific chase.

**Do not ask a question for something you are only listing.** The step list shows each step's form, so the editor builds them all on every change. *A form shown in a list is of the step's arguments and the program's state. Questions about what was built, over the whole assembly, are for the one dialog open, once per edit.*

**A system that splits into independent groups is solved group by group, and so is any question about it** (`geop_lib_constraints::solve::freedom`). If `$k$` independent groups of size `$s$` are solved jointly, the elimination costs `$(ks)^3$`, against `$k\,s^3$` apart. That is a factor `$k^2$`. A 2000-screw assembly on a fixed plate is 2000 tiny problems, not one huge one.

**Listing must not build.** Example programs and standard parts are listed by name, made when opened or read (`ExampleSet`, `geop_lib_stdlib`'s `FAMILIES`).

**What is answered as often as the pointer moves sends what changed.** This is `Changed::State` from [Lecture 39](./39-editor.md). An event of a drag sends the parts it moved. Every event used to send it all: 4.8 MB at 2000 parts, for an arm that moved.

**A drawing is of what it draws, and reads nothing else.** A placed part's drawing (`Drawn`, in `ops/geop-ops-model/src/placed/mod.rs`) drew the parts placed in it too, which the part placing it draws anyway, so it read where they are. Dragging a duck's mouth, placed in its head, placed in its neck, drew and meshed the head and neck again at every event, a second of CPU per event, the head being a subdivision surface, for a mouth that moved. The reuse rule of [Lecture 38](./38-runner.md) is exact, so an over-broad read is punished exactly. Nested benchmarks now have the nesting (`testing::duck`, in `ops/geop-ops-assembly/src/testing/duck.rs`).

## A benchmark has to have the shape of the real thing

Here is the subtlest of these. The 2000-part robot benchmark mated every screw to a fixed plate. The solver split it into 2000 groups of one screw, and drags were fast. A real robot mates screws to brackets, fastened to links that turn: *one* group of every part. The `robot_arm` benchmark (`ops/geop-ops-assembly/src/testing/robot_arm.rs`) has that shape, and a drag of only 101 of its parts ran for over fifty minutes without finishing: one dense system of 600 variables.

So everything looked fine on the benchmark while the actual product was unusable, and no amount of tuning the benchmark's numbers would have shown it. See this in the model below.

<figure class="demo" data-demo="scaling-counter">
<figcaption>Operation counts on log-log axes. The mate graph is generated and analysed for real: groups by union-find, pendants by peeling. Each dense system's cost is the operation count of Gaussian elimination, which the demo's test checks against an elimination that was actually run. Choose "screws on a fixed plate": grouping is everything. Choose "screws on a chain of links", each on a bracket on a link: grouping does nothing, the whole assembly is one group, and only pendants help. It is a model: the kernel's solver does more per system than one elimination, and six variables per body and six links are round figures.</figcaption>
</figure>

What fixed it is the notion of a **pendant**. A block whose constraints only hang it off others (they hold, and they constrain nothing of what they tie it to) is solved and enclosed after what it hangs off, alone. See `core/geop-core-math/src/solvers/system/structure.rs`:

> A screw mated to a bracket, a bracket fastened to a link: the screw's constraints read the screw and the bracket, and decide nothing about the bracket — whatever the bracket does, the screw can follow, and how it does is the screw's own business.

The arm's drag is then its links and joints, 42 variables however many parts hang off them (`AGENTS.md`). In linear-algebra terms, a pendant is a block in a block-triangular form of the Jacobian, and such a structure can be detected and exploited. [Lecture 37](./37-assembly-solver.md) treats it properly. Two lessons from the way there:

- *The rank test that decides a pendant was first asked at the point.* Rounding leaves a hair of a row that depends on the others, `1e-17`, definitely not zero, and nothing was a pendant off the axes. Asked over a box as wide as the solver's tolerance, it could be zero. "Decide a rank over the box" ([Lecture 24](./24-rank-and-proof.md)) holds for every rank decision.
- *Every target revised by every step (`CHAIN`) is a square in the steps.* The unknowns were one target, made again whenever a joint's parts moved, since a joint's coordinate is seeded where they are. A third of a drag of the 1000-part arm was there. A step's own target, gathered when read (`Unknowns::of`, like `Constraints::of`), costs what changed.

The second is the chain lesson of [Lecture 38](./38-runner.md) turned into a cost: `$n$` steps each revising one shared value, with `$n$` readers of it, is `$n^2$` work.

## The cost is in work not needed, not in slow arithmetic

The second round of the investigation, in `AGENTS.md`, found the same thing every time. Almost nothing was gained by faster arithmetic. Here are the four cases.

### Where the representation says polynomial, integrate exactly

Mass properties ran an adaptive 15-point rule inside another along every trim curve. The integral is turned into one along each face's trim curves, by the divergence theorem and Green's theorem, and at each point of that runs an inner integral along the surface. Look at that integrand. Along a direction with equal weights, a surface is a polynomial: any plane, an extrusion along its straight direction, a cylinder along its axis. Moments are then polynomials of known degree, and a Gauss–Legendre rule with `$n$` nodes integrates degree `$2n-1$` exactly.

So `$\lceil(d+1)/2\rceil$` points suffice, with **no truncation error at all**. That took five times fewer surface points, and a proven inner integral instead of an estimated one. See the "exact where it can be" section at the top of `core/geop-core-topology/src/mass.rs`, and `integrate_polynomial` in `core/geop-core-math/src/quadrature.rs`.

Notice that this improvement is *also a correctness improvement*. That is common. An estimate replaced by a proof usually gets cheaper too, since the structure that gives the proof also removes the need to hedge.

### A deterministic iteration that repeats a seed is done

Newton with sharpened seeds rarely hits an exact fixed point. It cycles in the last bit, and it ran out its iterations. But an iteration from a sharp seed is a deterministic function of that seed. If a seed repeats, the rest of the sequence is periodic and the result known. So `NurbSurface::project` stops at the first repeated iterate and returns the identical answer, in less than half the iterations (the logic is `iterate`, in `core/geop-core-math/src/solvers/iterate.rs`).

Notice what this is not. It is not a convergence tolerance. *Runs `iterations` times with no convergence tolerance, interval scalars can't judge "close enough"*, says the function's comment. The test is exact equality of seeds, and the answer is provably the one the loop would have given.

### Ask the cheap question first

A boolean cast a ray across the whole other solid for every interior point it classified. But a point near one already classified is classified along a short path from it, which a box test rules out of nearly every face (`shell_contains_from`, in `core/geop-core-topology/src/contains/shell.rs`).

There is a trap here, and it shows why this is not merely an optimization. A path between structured points is not generic. A straight segment between two points of one face lies in its plane, and two corners of a symmetric face can graze a cylinder. So the path goes by a random point, as a ray takes a random direction ([Lecture 19](./19-classification.md)).

### An interval argument is no licence to extrapolate

A spline evaluated over a parameter interval reaching past one knot span used that span's polynomial for all of it: 25 times too wide over a fitted curve, which a search then subdivided away at great cost. Curves now unite their spans' parts (`find_spans` in `core/geop-core-geometry/src/spline.rs`), and so do a surface's `partials`, which a proof over a box (`enclose_foot`) reads.

`NurbSurface::evaluate` still extrapolates, as the comment on `find_spans` says: uniting there left another test's boolean with a face straddling a cap, and something downstream leans on the width. This is honest engineering, and you should read it as such: not every instance is fixed, and the code says which is not and why.

This case shows numerics and performance as one subject. A too-wide enclosure costs subdivision steps. Every bit of honest width you remove saves search work. Much of the performance of Part III comes from this, not from tight loops.

## Finding where the work is

`perf` is not allowed on this machine. `ptrace_scope` only lets `gdb` into a process it started. So the method is sampling by hand:

1. **Cap the run.** Always (`AGENTS.md`):
   ```sh
   timeout 600 prlimit --as=16000000000 cargo test ...
   ```
   A geometric search that runs away does not stop on its own. It grows until the machine runs out of memory. A capped run fails fast, with "memory allocation failed" or a timeout, and *that is itself a finding*. Then find where it ran away instead of raising the cap.
2. **Interrupt under gdb.** Run the test under `gdb -batch`, send the inferior `SIGUSR2` (handled `stop print nopass`), and print the backtraces, at intervals. Ten samples often suffice. A function that holds half the time will be in about five of them. A thousand samples only sharpen a percentage you did not need.
3. **Count with breakpoints.** To count, set breakpoints that only count, on a line or on a function with its caller checked, never prints. This answers "how many times", the question that matters, without changing what you measure.
4. **Take the process id from the process you started** (`$!`), never by matching names: `pgrep geop` also finds the user's own editor and servers.

The runaway is often far from where the failure was reported. A revolve that "crashed the boolean" was in fact the rasterizer: clipping a concave hole of many corners split it into exponentially many pieces (`subtract_convex` in `core/geop-core-rasterize/src/clip.rs` kept degenerate leftovers). Only interrupting the capped run showed where it was. We retell this in [Lecture 42](./42-case-studies.md).

### Reasoning from a stack

Having found a place, ask of it the questions this lecture has been practising:

- *How often is this called, and should it be?* (the drag)
- *Does it know something it is not using?* (polynomial integrands, repeated seeds)
- *Does the question have a cheap conservative answer?* (box tests, hull tests)
- *Is it working on something too wide?* (extrapolation, compounding width)
- *Is it reading more than it needs?* (a drawing of what it does not draw)
- *Does the work split?* (groups, pendants)

Only after those, ask whether the arithmetic could be faster.

## Worked example: bounding a quadratic

Suppose a form calls `freedom` once for every step, and you suspect it. A plain test plan:

1. Pick a counter: `mates_resolved`, since `freedom` resolves mates.
2. Choose sizes where a quadratic and a linear law differ: 20 and 80.
3. Measure the delta over the operation in question, as in `an_edit_resolves_each_mate_a_few_times`.

Say you get 44 at 20 screws and 524 at 80. The ratio is 11.9, far above 4. Fit `$an^2+bn$`: `$400a+20b=44$` and `$6400a+80b=524$`, so subtract four times the first from the second: `$4800a=348$`, `$a\approx0.0725$`, `$b\approx0.75$`. At 2000 screws that predicts about 291 500. Once fixed, assert that the delta per mate is bounded by a small constant, independent of the size, at both sizes. Those numbers were invented for this exercise in arithmetic; what the real test asserts is in `ops/geop-ops-assembly/src/scale_tests.rs`.

## Sweep the space, but keep the everyday suite fast

The same reasoning applies to the tests themselves. The `stress_tests.rs` of `geop-ops-fillet` and of `geop-ops-shell` blend every edge, and shell every face, of boxes, cylinders and spheres: whole, drilled, pocketed, stepped, cut off and with a boss. Their first run found five shell bugs no hand-written test had reached. They are valuable, and slow: bodies with circular edges take up to a minute each, and the user noticed the suite slowing down. So:

- Keep a fast representative subset in the default run.
- Mark the rest ``#[ignore = "slow: … — run with `cargo test -- --ignored`"]``.
- Run the ignored set before changing blends, shells, booleans or the solver. CI runs them nightly (`DEVELOPERS.md`).
- When a regression test needs a slow setup, split it: a fast test of the actual defect, and an ignored test of the combinations.
- `scripts/test-affected.sh` tests only what a change can affect, from a map of which crates build on which. This is why dev-dependencies are chosen so carefully ([Lecture 40](./40-writing-an-operation.md)): each is an edge in that graph.

<div class="recap">

## Recap

- Count, do not time: a count is exact, cheap to get, and shows a law at a small size. Keep the counters in the code.
- Group what does not interact; treat what only hangs off the rest afterwards.
- Most cost is in work not needed: a question asked per step, a read wider than the drawing, a rule estimating what is exact, an iteration past its repeat.
- A benchmark must have the shape of the real thing, or it proves nothing about it.
- Cap every run. Sample by interrupting under `gdb`. Count with silent breakpoints. Then ask how often, what is known, what is too wide.

</div>

## Exercises

**1.** A Gauss–Legendre rule of `$n$` points integrates polynomials up to degree `$2n-1$` exactly. A bicubic plane patch's moment integrand along `$u$` has degree 6 in this direction. How many points do you need? Verify with a two-point rule on `$\int_{-1}^1 x^2\,dx$` and show that the same rule fails on `$x^4$`.

<details class="solution"><summary>Solution</summary>

`$2n-1\ge 6$` gives `$n=4$`. The two-point rule has nodes `$\pm 1/\sqrt3$` and weights 1. For `$x^2$` it gives `$2/3$`, which is exact. For `$x^4$` it gives `$2/9$`, whereas the integral is `$2/5$`. The degree-3 bound is sharp.

</details>

**2.** In the demo, on the robot shape at 2000 parts, compute by hand the ratio of "one dense system" to "groups and pendants", using `$m^3/3$`-order costs and the demo's six variables per body, six links.

<details class="solution"><summary>Solution</summary>

The demo's `denseOps` counts the multiplications and divisions of elimination and back substitution, `$\tfrac{m(m+1)(2m+1)}{6}+\tfrac{m(m-1)}{2}$`, with leading term `$m^3/3$`. Bodies: 2000 screws, 6 links, 6 brackets, so `$m=2012\cdot 6=12072$`, about `$5.9\times10^{11}$`. Pendants: a core of 6 links, `$m=36$`, costs 16 836; the other 2006 bodies are systems of 6 variables at 106 each, 212 636 in all. The sum is about `$2.3\times10^5$`, and the ratio about `$2.5\times10^6$`. Scale that by what one operation costs: it is the difference between hours and a frame.

</details>

**3.** The story of the duck: why did the drawing of the head read where the mouth is, and how would you have detected the over-read with a test, not a timer?

<details class="solution"><summary>Solution</summary>

It drew the parts placed in it, which requires their poses. The placing part draws them anyway, so that read was wasted, yet the reuse rule honoured it: a mouth's new pose invalidated the head's drawing. Test: drive a drag of the mouth, then count the makes of the `Drawn` target of the head (`Runner::counts`, or a `kept_makes` delta), and assert zero. Without nesting in the benchmark, this test could not have been written: a flat benchmark has no such read.

</details>

**4.** Newton from a sharp seed cycles in the last bit with period 2: `$a,b,a,b,\dots$`. The stop-on-repeat returns at the second `$a$`. The loop without the stop would have run `$N$` iterations. Prove the two return the same value only if a condition holds on `$N$`, and read how `iterate` in `core/geop-core-math/src/solvers/iterate.rs` handles this.

<details class="solution"><summary>Hint</summary>

If the iteration is `$x_{k+1}=f(x_k)$` and `$x_{i}=x_{i+p}$`, then `$x_{N}=x_{i+((N-i)\bmod p)}$`. So the final value depends on where in the cycle `$N$` lands. To return "the identical answer", one must compute the position. Read the code to see that it does so, and notice that the seed may be sharpened where the returned value is not.

</details>

**5.** Your new operation makes the fillet stress test take twice as long, and the fast subset is now 30 s. What do you do, in which order?

<details class="solution"><summary>Hint</summary>

First, find out why with a count or a stack sample, since it may be an actual regression, and you should not paper over it by ignoring tests. If it is inherent, pick the bodies that exercise distinct configurations for the fast subset and mark the rest `#[ignore = "slow: ..."]`. If a regression test needs the slow setup, split it into a fast test of the actual defect and an ignored one of the combinations.

</details>
