# Lecture 37: The assembly solver

<div class="goals">

**After this lecture you can**

- describe the loop that solves a program's state from outside the part, and why no step writes the state;
- compute a mechanism's freedom from the rank of its rows, and say where counting formulas go wrong;
- explain groups and pendants, why they are the difference between a small system and a cubic one, and how the rank test behind a pendant must be asked;
- tell a limit position from a singular configuration, in a real linkage;
- say what the editor does after every edit, what a conflict report costs, and why a value must be read where its read is checked.

</div>

[Lecture 36](./36-mates.md) wrote mates and joints as residuals. Now we solve them, while a user drags a link, in an assembly that may have 2000 parts. The numerics are exactly those of Part VI, since the same `System` in `core/geop-core-math/src/solvers/system/` does the work: `least_squares::minimize` steps, Krawczyk proofs, rank over a box. What is new is scale and structure. A dense solve is cubic in its variables, and 2000 parts have 12 000 of them. The lecture's main idea is the observation that makes this tractable: **a real assembly is nearly all parts that merely hang off a few that matter.**

## The loop outside the part

From [Lecture 2](./02-programs-of-steps.md): a program is a list of steps that builds a part. Where the placed parts are is the program's *state*, an input. A step declares what it requires of the state and which values a solve may move, in two targets of the part:

- `Constraint`s, residuals to vanish (or, for a limit, `Bound::AtMost`, to stay at most zero), with their derivatives. A step defines them through `define_constraints`, and `Constraints::of` gathers every step's. These are in `lib/geop-lib-constraints/src/constraint.rs`.
- `Unknown`s (`lib/geop-lib-constraints/src/unknown.rs`): a state value, its `Mobility`, a `seed` for where to start it, and `unless`, a constraint which, if it fails, says to start it from the seed. `Unknowns::of` gathers them.

`Mobility` says what a solve may do with a value:

| `Mobility` | Meaning | Example |
|---|---|---|
| `Fixed` | stays exactly as it is | a part with a `fixed` mate |
| `Held` | moves as far as the constraints need, and no further: held by `DAMPING` | a placed part's pose |
| `Follows` | moves as the constraints have it, held nowhere | a joint's coordinate |

So the solve happens outside the part. `solve` in `lib/geop-lib-constraints/src/solve.rs` runs the part at the state once, reads its constraints, and steps the state. At every state it tries, it does not run the program again. It makes only the constraints it asks again, each by its step's generator, of the part as built and the state tried (`Remade`). So the work of a solve step is that of the constraints it asks. No step writes the state, and every front end gets the same solver.

`Preference`s, which become the system's preferences as in the sketch's solve, express what a solve should favour among the states the constraints allow: a drag is `Pull::Point`, a point of a placed part pulled towards the pointer. Typed-in placement is `Pull::Pose`, and setting a joint is `Pull::Value`, which holds that coordinate at its value and lets the parts move to it (`Pull` in `core/geop-core-editor/src/editing/form.rs`).

## Freedom, by rank

Here is a planar four-bar linkage: a crank, a coupler and a rocker on two ground pivots. Each moving body has a pose (in the plane, `$x,y$` and an angle, three numbers). Each pin takes two rows. So 9 variables, 8 rows: `$9-8=1$`. One way to move. That is the classic *Grübler count*: 3 per body minus 2 per pin.

Grübler counts rows. The kernel uses their rank, for the reason you met in [Lecture 23](./23-sketch-solver.md). Take a parallelogram linkage, whose coupler always stays parallel to the ground, and add a third crank midway, parallel to the other two. It adds one body (3) and two pins (4): 12 variables, 12 rows, and Grübler says **0: it cannot move.** And yet it plainly moves, since the third crank follows the motion the first two already determined. The row that the third crank's closing pin adds is redundant: rank 11, freedom 1. Everywhere, not at one point.

<figure class="demo" data-demo="four-bar">
<figcaption>A real planar solve of a linkage, as the kernel solves one in miniature: each link a body with three variables, each pin two residuals, the step shortest over the pin rows, and the pull along their null space. Drag the three handles. Choose the third crank and compare counting with rank. "Rocker's limit" puts crank and coupler in line; "Flat parallelogram" lays the linkage flat. Simplified: planar, so a body's angle adds and no exponential map is needed; rank is decided by a tolerance on the pivot, not over a box.</figcaption>
</figure>

### Two kinds of "stuck"

The demo shows the two things that can look like lock-up. They are very different.

**A limit position.** In the crank-rocker linkage, press "Rocker's limit". Crank and coupler now lie on one line, and the rocker has reached the end of its swing. Read the numbers: the rank is still 8, the freedom still 1, regular. The linkage's one motion has the crank turning and the rocker standing still (the readout's rate goes to 0). If you drag the crank, it just keeps going. If you drag the rocker past its limit, it stops, at the end of its travel, as the pull's minimum among where the pins hold. Machinists call this a *dead point*, since a rocker driving the crank has no leverage there. For the solver nothing special has occurred.

**A singular configuration.** Press "Flat parallelogram". All four pivots lie on one line. The rank drops to 7, the freedom to first order is 2, and the smallest pivot is zero. There really are two ways out: staying a parallelogram, or folding into an *anti*-parallelogram with the coupler crossing. This is what a Krawczyk proof cannot pass: a singular point, like the tangent of [Lecture 24](./24-rank-and-proof.md). As `System::enclose` would put it, no true solution is proven there, and `Report` says so. A step of a drag lands a hair away from it on some branch, and the rank is back.

The constrained minimizer has seen this too: its curvature estimate `$B$` can change abruptly across such a configuration, so where a step it proposed is turned down, the step is tried again without it, and it is learned afresh if that does better (`core/geop-core-math/src/solvers/least_squares.rs`). It is also why no merit function is used: `$\Sigma r^2+\rho\lvert c\rvert$` would need `$\rho$` above the multipliers, and those grow without bound at a singular configuration, the module's comment says, two links in line for example.

## Scale: a benchmark has to have the shape of the real thing

From AGENTS.md's *Count the work, not the seconds*: the 2000-part robot benchmark mated every screw to a fixed plate, so the solver split it into 2000 groups of one screw, and drags were fast. A real robot mates screws to brackets fastened to links that turn: one group of every part. The `robot_arm` benchmark (`ops/geop-ops-assembly/src/testing/robot_arm.rs`) has that shape: a fixed base and a chain of links on revolute joints, brackets fastened to the links, screws mated to the brackets, motors, belted pulleys and a gripper. And a drag of only 101 of its parts **ran for over fifty minutes without finishing**: one dense system of 600 variables.

### Groups

The first step in `System::structure` (`core/geop-core-math/src/solvers/system/structure.rs`) is the obvious one. Blocks no term ties together are separate systems, found by union-find over which blocks each term reads (`grouped`). A plate with hundreds of screws is hundreds of systems of one screw. The answer is the same, as what a solve minimizes is a sum over the groups, and a group whose constraints hold and that nothing pulls is not solved at all. A question about the system splits likewise: `freedom` in `lib/geop-lib-constraints/src/solve.rs` measures group by group.

That helps the plate. It does not help the arm, in which everything is connected.

### Pendants

Think about a screw mated to a bracket. The screw's constraints read the screw and the bracket, and they decide nothing about the bracket. Whatever the bracket does, the screw can follow, and how it does is the screw's own business. Such a block is a **pendant** on the blocks its terms read besides it: it can be solved after them, alone, with them where they ended up, and enclosed after them, over their boxes. After taking the screw off the bracket, the bracket comes off its link the same way. What is left is the *core*: the links and joints.

A block `$b$` hangs off the blocks `$S$` its remaining terms read besides it when, where the blocks are:

1. **Nothing pulls it, and no limit reads it.** Either could hold back what it hangs off.
2. **Its terms hold.** It is where they put it, given `$S$`.
3. **Its rows have the same rank by its own steps as by all of `$S$`'s and its own.** They constrain nothing of `$S$`, to first order. A part mated to a link *and* to the ground is no pendant, since its rows also hold the link.
4. **It can be carried.** Held where it is, it can be moved along by one of `$S$` (`Variable::carried`, `$to\circ from^{-1}\circ self$` for a pose), which is what a solve starts it from. And every value of `$S$` that only `Follows` must still be read by some other term. Otherwise a joint coordinate only `$b$`'s joint reads would be held where it was, and `$b$` turned with the link, where the whole solve held `$b$` and turned the joint.

The third condition is a rank decision, and it has the same trap as in [Lecture 24](./24-rank-and-proof.md). From AGENTS.md: the rank test was first asked at the point. Rounding leaves a hair of a row that depends on the others, `$10^{-17}$`, definitely not zero, and nothing was a pendant off the axes. Asked over a box as wide as the solver's tolerance, as the enclosure asks it, it could be zero. "Decide a rank over the box" holds for every rank decision, not only the enclosure's.

The result is layers (`Structure::layers`). Each `Group` has blocks and the blocks `on` which it depends, in an earlier layer. The arm's drag is then its links and joints, **42 variables, however many parts hang off them** (AGENTS.md). The groups of a layer are worked on in *lockstep* (`core/geop-core-math/src/solvers/system/lockstep.rs`): one evaluation answers every group waiting, so the evaluations a solve takes do not grow with the number of groups.

What changes in the answer? Only what a block "held where it was" holds back. A pendant moves with what it hangs off, and of its own freedom stays as near where it was as its terms let it. But it no longer holds back what it hangs off: a link carrying a hundred screws is as easy to move as one carrying none. For a user that is simply better.

### Counting instead of timing

AGENTS.md tells the rest. A drag in a 2000-part robot went from 0.25 s to minutes, and nothing timed it. The editor builds every step's form on every change, for the step list; the added part's form asked the whole assembly how free its part was, a dense null space over every part's variables. Once per step, that is cubic in the parts. Now:

- A form shown in a list is of the step's arguments and the program's state. Questions about what was built, over the whole assembly, are for the one dialog open, once per edit.
- Guards are counts, not times. `geop_ops_assembly::mates_resolved` counts mates found and measured, and `ops/geop-ops-assembly/src/scale_tests.rs` asserts on it: building 30 screws on a plate resolves 31 mates, and a part moved resolves none, since "only screw7's constraint reads where it is". `work` in `core/geop-core-math/src/solvers/mod.rs` counts dense linear algebra as `$\text{rows}\times\text{columns}^2$` for each factorisation, so a test can bound a solve's cost. A quadratic shows at 80 parts as clearly as at 2000.
- An event of a drag changes the state alone (`Changed::State`). It sends the parts it moved, and the program once the drag is let go.
- A drag's events are a hair apart, so the structure found in one is reused for the next (`Problem::structure`, `System::solve_unproven_in`) rather than found again. If a pendant cannot follow after all, the check notices.

[Lecture 41](./41-performance.md) returns to all this.

## What the editor does

The assembly crate adds to the editor through one trait, `Settle` in `core/geop-core-editor/src/registry.rs`, which is what keeps the kernel from knowing about mates. After every edit, `Settler::settle` in `ops/geop-ops-assembly/src/settle.rs` does the following.

1. If something pulls, **pull first**: every part not fixed is free to give way. A linkage follows the link dragged.
2. Otherwise, if the mates hold, nothing moves at all.
3. Otherwise solve for **the step's own part first**, the others held: mating a new part moves it, not the part it is mated to. Only if that does not converge is every part not fixed set free.
4. Where the constraints still do not hold, **ask which conflict**.

A `Report` from a solve carries what holds, the limits stopped at, what moved, the minimizations and an `Enclosure` for every group, or why none: the constraints that conflict, by name, or what is singular.

**Limits** (`Bound::AtMost`) are kept by an active set: solved without the limits first, then every one the solve ends beyond is held at its bound, and the group is solved again from where it started.

**Conflicts.** `System::conflicting` returns a smallest set of constraints that cannot all hold together. A group that does not hold is enclosed where it ended, and the conflict the enclosure proves names the constraints. Where nothing is proven, in a singular configuration, each constraint of the group is left out in turn and kept out where the rest still cannot all hold. That is as many solves as there are constraints, so it is for a dialog to show, never for every drag.

**Freedom** in a part's dialog (`solving::freedom` in `ops/geop-ops-assembly/src/solving.rs`, shown by `JointsPanel`) is decided over the box the enclosure proved, not at the point: a redundant constraint takes no freedom away. It asks once per edit. A part fixed in a sub-assembly moves as the sub-assembly does, so it reads its freedom from the first pose round it that the solve frees.

## Sub-assemblies, and where a value is read

An assembly can be placed in another. Every value of the state of a program placed becomes one of the placing program's, named behind the step's id (`hinge/pin.pose`). Its constraints come up with it, through `Constraints::of` and `Unknowns::of`: residuals as they are, lengths within the instance, wherever it is.

That invites a subtle bug, which the incremental build of [Lecture 38](./38-runner.md) makes visible. A version of a computed value is reused when what it read did not change, so the place a value is read from and the place its read is checked must be the same. From AGENTS.md: a mate on a part of a placed sub-assembly read `/hinge:pin.pose` from the top part's own state, which the file never gave, and got no pose. The check looked in the hinge's part, where the hinge file's pose was, and saw nothing change. The mate kept a constraint differentiated by nothing, and the solve gave up at once. A value behind an instance's path is now read inside that instance's part, by `Reader::placed_part`, as the check does. When a read and its dependency take different routes to the same name, one of them is wrong.

<div class="kernel">

**In the kernel.** Generic algorithms: `core/geop-core-math/src/solvers/system/` (`mod.rs` for `System`, `Equations`, `Variable`; `driver.rs`; `structure.rs`; `enclose.rs`; `lockstep.rs`). The state's adapter: `lib/geop-lib-constraints/src/solve.rs` (`Problem`, `Run`, `solve`, `enclose`, `freedom`, `conflicting`). What the editor does with it: `ops/geop-ops-assembly/src/solving.rs` and `ops/geop-ops-assembly/src/settle.rs`. The drag tool: `DragTool` in `ops/geop-ops-assembly/src/drag_tool.rs`. Tests that count: `ops/geop-ops-assembly/src/scale_tests.rs`.

Not implemented: dynamics. There are no forces, masses in the solve, or contact. Motion studies (`Drive`, `MotionPanel`) drive joint coordinates by a time table and solve each frame from the one before; `check` asks the interference inspector about each frame, after the fact. `Urdf` writes the mechanism for simulators that do have those.

</div>

## Recap

<div class="recap">

- The solve runs outside the part: steps declare `Constraint`s and `Unknown`s, and `solve` makes only the constraints it asks again at each state it tries.
- Freedom is variables minus rank. Counting formulas fail exactly on redundancy.
- A limit position is regular: a motion remains, and one body stands still. A singular configuration loses rank: two branches meet and no proof can pass.
- Groups split what nothing ties together. Pendants take off what only hangs off the rest, layer by layer, leaving a small core. The test is a rank over a box.
- Count work with tests: `mates_resolved`, `work`. Keep whole-assembly questions to the one dialog open, once per edit.
- Read a value where its read is checked.

</div>

## Exercises

**1.** Apply Grübler's count to: a door on two coaxial hinges; a parallelogram with a third crank (as in the demo); a four-bar with a ball joint in place of every pin, in space. What does each count say, and what is the rank-based truth?

<details class="solution"><summary>Solution</summary>

Door: one body, 6 + 2 coordinates, 12 rows: count `$8-12=-4$`; true freedom 1, rank 7, five rows redundant. Parallelogram: count 0, truth 1, as above. Spatial four-bar with 4 balls: 3 bodies × 6 = 18, 4 × 3 = 12 rows, count 6, and the rank agrees. Of those 6, 3 are each bar spinning about its own length, which moves nothing a designer cares about. The rank is right and it is also not what the designer meant: one reason a `freedom` per part is shown.

</details>

**2.** A motor is `Fastened` to a link, and a screw is mated to the motor by a concentric and a plane coincident mate, leaving the screw a spin. Walk through the four pendant conditions for the screw and for the motor. In which order do they come off? What is the screw's freedom after the structure is applied?

<details class="solution"><summary>Solution</summary>

The screw: nothing pulls it and no limit reads it; its terms hold; its rows' rank by its own steps equals the rank by the motor's steps and its own (it constrains nothing of the motor); and it can be carried by the motor. So it comes off first. Then the motor: it has only the link's fastening left, whose rows it can satisfy alone, so it comes off the link. Its layer comes before the screw's, as `on` requires. The screw's freedom is its spin, 1, unchanged: pendants do not change what is free, only what holds back what.

</details>

**3.** Show directly for the four-bar why the rank is unchanged at the rocker's limit. Take the crank and coupler in one line, let `$\dot\theta_1,\dot\theta_2$` be the crank and coupler's rates. The point where the coupler meets the rocker has velocity along what direction? Why does this force the rocker to stand still, and leave one freedom?

<details class="solution"><summary>Solution</summary>

With crank and coupler collinear along direction `$e$`, the velocity of the coupler's end is `$(L_1\dot\theta_1+L_2\dot\theta_2)\,e^\perp$`. The rocker's end moves perpendicular to the rocker. Those directions differ unless the rocker is also along `$e$`, so both velocities must vanish: the rocker stands still, and `$L_1\dot\theta_1+L_2\dot\theta_2=0$`, one equation in two unknowns. One freedom, rank full. If all four pivots are in line, the directions coincide, the velocity need not vanish, and there is an extra freedom: that is the flat parallelogram.

</details>

**4.** A drag of a 1000-part arm is profiled, and a third of its time goes to re-making a target that gathers every step's unknowns, revised by each step in turn. Why is that a square in the steps, and how does gathering when read fix it? AGENTS.md has this story.

<details class="solution"><summary>Hint</summary>

Every target revised by every step (`CHAIN`) makes each step copy what the steps before accumulated: `$\sum_k k$` in the number of steps. A step's own target, gathered at read time by `Unknowns::of` and `Constraints::of`, only costs what changed. See `lib/geop-lib-constraints/src/unknown.rs`.

</details>

**5.** Why does `Settler::settle` try the step's own part first? Construct a scene where freeing every part in the first attempt gives a worse answer for the user, though every constraint holds.

<details class="solution"><summary>Hint</summary>

Mating a new bolt to an assembly: with only the bolt free, the bolt jumps to the hole. If everything were free, damping is the same per block, so the assembly's many parts would move too, a bit each, to meet the bolt half way. Every mate holds and the user's assembly has been disturbed.

</details>

**6.** Suppose a mate's residual were accidentally computed with a joint coordinate that nothing reads except that mate. What does `Mobility::Follows` guarantee? What goes wrong in a pendant test without the "still read by another term" condition?

<details class="solution"><summary>Solution</summary>

A `Follows` value is held nowhere, so nothing pulls against the constraints. Without the condition, `$b$` hanging off a link is solved after it, alone. The joint's coordinate, which only `$b$`'s joint reads, was treated as part of `$S$`, and solved in an earlier layer with no term telling it anything, so it stays where it was. Then `$b$` must satisfy its joint with that stale angle: it turns with the link. The whole solve would have held `$b$` and turned the coordinate instead. So different structure, different answer, which the structure must never produce.

</details>
