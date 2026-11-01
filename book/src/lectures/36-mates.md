# Lecture 36: Mates and poses

<div class="goals">

**After this lecture you can**

- represent a rigid motion by a quaternion and a position, and say what happens when the quaternion is off unit;
- explain why a solver must step a pose by a twist through the exponential map, not by adding to a global coordinate;
- write a mate or a joint as residuals, and count its rows and the freedom it leaves;
- explain why a joint's angle row is `$2\tan\frac{\varphi-\theta}{2}$`, not a sine;
- say what a mate means when the user picked a piece of a wall, and why a reference direction must not be left to noise.

</div>

A part is rarely alone. A hinge has two leaves and a pin; a robot has hundreds of parts. An assembly in Geop is a part in which other parts are *placed*, each another file's program, and *mated*: a statement about how a placed part sits relative to something else. "This shaft is concentric with that hole." "This link turns about that axis."

The sketch solver of [Lecture 23](./23-sketch-solver.md) moved numbers. Here the unknowns are positions and orientations of rigid bodies, and the same `System` will solve them ([Lecture 37](./37-assembly-solver.md)). What is new is that a rotation is not a number: it does not live in a vector space. This lecture is about how to make a Newton-type solver work on such a thing, and then how to state a mate.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"four_bar","example":"four_bar","yaw":0.3,"pitch":1.0}'>
<figcaption>The <code>four_bar</code> example, four files placed and mated, meshed by the real kernel. In the editor, drag a link and the others follow, solved live.</figcaption>
</figure>

## Where a part is

A placed part's location is a value of the program's **state**: not an argument of the step, and not something the step computes. In a saved program it looks like this:

```json
"state": { "pin.pose": { "position": [1.0, 1.0, 0.5], "rotation": [1.0, 0.0, 0.0, 0.0] } }
```

The step `AddPart` (`ops/geop-ops-assembly/src/add_part.rs`) places the part where that state says and adds its mates. **It solves nothing.** A solve outside the part moves the state until the constraints hold ([Lecture 37](./37-assembly-solver.md)). So a mate in a later step can move a part an earlier step placed, for every step.

That pose is `Pose` in `core/geop-core-math/src/primitives/pose.rs`: a point `$p$` of the part is at `$Rp+\text{position}$`, with `$R$` the rotation of a quaternion.

### Rotations by quaternions

A rotation by angle `$\theta$` about a unit axis `$n$` is the unit quaternion

`$$q=\Bigl(\cos\tfrac\theta2,\ \sin\tfrac\theta2\,n\Bigr),$$`

and it turns a vector `$v$` by the product `$qv\bar q$`, with `$v$` treated as the quaternion `$(0,v)$`. Composition of rotations is multiplication of quaternions. For example a quarter turn about `$z$`, `$q=(\tfrac{\sqrt2}{2},0,0,\tfrac{\sqrt2}{2})$`, sends `$(1,0,0)$` to `$(0,1,0)$`. Note that `$q$` and `$-q$` give the same rotation: the half angle means a full turn of the body goes half way round the quaternions.

Why quaternions, and not matrices or Euler angles? A rotation matrix has nine numbers and six conditions, which rounding violates. Euler angles have three numbers but go singular at gimbal lock; in Geop, angles are for showing a pose to people only (`Pose::euler_degrees`, `Pose::from_euler`). A quaternion has four numbers and one condition, and if the condition is off by a hair, it can be handled.

The kernel's choice is that **a quaternion need not be unit to stand for a rotation**: the rotation it makes divides by its norm squared (see `Pose::apply`). That is a deliberate decision and it pays off in two places.

**A value read from a file is kept as read.** `Pose`'s reader used to normalize the quaternion it read. Normalizing a sharp quaternion gives an interval. A file holds its midpoint, which reads back as another interval and is written as another midpoint. A program with a placed part's pose in its state was never the same program once saved and loaded, and the press example failed its round trip (AGENTS.md). So the reader keeps the pose exactly as read and refuses only what stands for nothing, the zero quaternion. Reading what was written must give back what was read.

**An enclosure must hold a rigid motion.** Where the proof needs a unit quaternion, it normalizes honestly, in intervals, as [Lecture 24](./24-rank-and-proof.md) told: `retract_over`.

## The coordinates a solver moves in

Newton's method needs an update `$x\leftarrow x+\delta$`. A pose has no plus. What should a step of a pose be?

The tempting answer, and the kernel's former one, is a global chart: a body's turn is described by a 3-vector `$w$` through the quaternion `$(1,w/2)$`, normalized. It is smooth, has three numbers, and is the identity at zero. It looks perfect. Take a turn about `$z$`: `$(1,w/2)$` normalized has half angle `$\arctan(w/2)$`, so the angle it stands for is

`$$\theta(w)=2\arctan\frac w2,\qquad\frac{d\theta}{dw}=\frac{1}{1+w^2/4}.$$`

Look at that derivative. A half turn is reached only as `$\lvert w\rvert\to\infty$`. To stand at `$170°$`, `$w=2\tan85°\approx22.9$`. At `$179.5°$`, `$w\approx458$`, where one unit of `$w$` turns the body by only `$2\times10^{-5}$` radians. The Jacobian's column has shrunk by that factor, and the solver's damping, which keeps a value near where it was, now weighs more than the motion.

From AGENTS.md: a link of a dragged arm that had to turn nearly half way round ran its variables off to hundreds, each step turning it less, until the solver ran out of steps. Nothing was wrong with the mates. The coordinate system simply could not reach the answer at a finite, well-conditioned value.

<figure class="demo" data-demo="pose-exp">
<figcaption>A model problem: one link turned to a target near a half turn, by one damped Gauss-Newton in each of two sets of variables, the same damping and the same step acceptance. Not the kernel's old solver, and not its current one, but the same effect. Orange is the old coordinate: at 179° it needs 47 steps, and at 179.5° it hits the cap of 200 still <code>2e-6</code> off, with <code>w</code> at 458. Blue steps a twist from where the body is: 11 and 12 steps. Slide the target and the damping.</figcaption>
</figure>

### Twists and the exponential map

The fix is to stop having a global coordinate at all. **Every step of the solve is taken from where the body is:**

`$$x\oplus\delta=x\circ\exp(\delta).$$`

Here `$\delta=(\omega,v)$` is a **twist**, six numbers in the body's own frame: turning at `$\omega$` while moving at `$v$`, for unit time. It is a tangent vector at `$x$`, and `$\exp$` takes it to the rigid motion it generates. Since `$\delta$` is measured from the current pose, every step's coordinates are close to zero, where the map is perfectly conditioned. No turn is ever far from the variables' origin. A half turn is just two quarter turns.

A solver usually calls this operation a *retraction*, which is the name in the kernel's trait: `Variable::retract`. The trait in `core/geop-core-math/src/solvers/system/mod.rs` is how the generic solver sees any value it moves: `dimension`, `retract` (where a step takes it, sharp, a seed), `retract_over` (every value a box of steps takes it to, an enclosure), `differential`, `stay`, `same`, `carried`. A number is one instance, with `retract` being addition. A pose is another, with dimension 6.

**The formula.** For `$\omega$` with `$\phi=\lvert\omega\rvert/2$`, `Pose::exp` computes the unit dual quaternion `$\exp(\hat\xi/2)$` of `$\hat\xi=\omega+\varepsilon v$`, which multiplied out gives

`$$\text{rotation}=(\cos\phi,\ s\,\omega),\qquad\text{position}=2s\cos\phi\,v+(s^2+2c\cos\phi)(\omega\cdot v)\,\omega+2s^2\,\omega\times v,$$`

with

`$$s=\frac{\sin\phi}{2\phi},\qquad c=\frac{\cos\phi-\sin\phi/\phi}{8\phi^2}.$$`

It is a screw motion along the axis of `$\omega$`: it turns by `$\lvert\omega\rvert$` radians. To first order, it is `$(1,\omega/2)+\varepsilon(0,v/2)$`. Near `$\phi=0$` the closed forms are `$0/0$`, so `Pose::exp` sums them as series in `$\phi^2$`, polynomials and smooth through zero. Each series alternates, and for `$\phi^2<1$` its terms shrink, so each sum is within its first omitted term, which is added as an honest enclosure. So the map is an enclosure for a twist that is a box around zero too, and so are its derivatives. Dual numbers can differentiate straight through it, as a solver does to linearize a pose (`Pose::exp_with_derivatives`).

**A worked example.** Let `$\omega=(0,0,\pi/2)$`, `$v=(1,0,0)$`: a quarter turn about `$z$`, moving along the body's own `$x$` throughout. Then `$\phi=\pi/4$`, `$s=\sin\phi/(2\phi)=0.4502$`, and `$\omega\cdot v=0$`:

`$$\text{position}=2s\cos\phi\,(1,0,0)+2s^2\,(0,\tfrac\pi2,0)=(0.6366,\ 0.6366,\ 0).$$`

Indeed: a point moving at speed one while turning through `$\theta=\pi/2$` traces a quarter circle of radius `$1/\theta$`, from the origin to `$(\sin\theta/\theta,\ (1-\cos\theta)/\theta)=(0.6366,0.6366)$`. The demo's test, `book/scripts/demo_tests/pose-exp.test.mjs`, checks this formula against numerically integrating the motion.

**The differential.** For [Lecture 24](./24-rank-and-proof.md)'s proof we need the Jacobian of a term by the step at every point of a box. That is the term's Jacobian by the steps at `$x$`, what `Equations` gives, times the differential of the retraction: for a pose, the right Jacobian of the exponential map (`Variable::differential`, by `Pose::exp_differential`). At a step of exact zeros, it returns `None`: the identity, exactly, with no rounding introduced.

**Staying put.** What holds a part where it was when nothing else decides? `Variable::stay` gives residuals: its position's difference from `at`, and its rotation's as the unit quaternions' difference, using the nearer of `$\pm$`, times `scale`, so that a turn counts as the distance it moves a point that far from its axis. Everything in a system of residuals is again a length.

## Mates as residuals

A `Mate` holds a `MateKind` between `MateEntity`s (`ops/geop-ops-assembly/src/mates.rs`), of four sorts:

- **Constraints** (`Kind`): `Coincident`, `Concentric`, `Parallel`, `Perpendicular`, `Distance`, `Angle`, `Tangent`, `Path`. Each takes a point, a line or a plane of each entity, where its part puts it.
- **Joints** (`JointKind`, `Joint`): how the second frame may move on the first. Revolute, slider, cylindrical, planar, pin in slot, ball, universal, fastened.
- **Couplings** (`CouplingKind`): a coordinate of one joint tied to one of another by a factor and an offset: gear, belt, rack and pinion, screw, linear. They are presets of one linear relation.
- **Fixed** (`Anchor::Fixed`): of one entity, or, in an `add_part` step, of none, meaning the part that step places. The part never moves in a solve.

Each is residuals. Take `residuals` in `ops/geop-ops-assembly/src/residuals.rs`, with everything computed in dual numbers so every residual carries its derivatives:

| Mate | Entities | Rows | Residual |
|---|---|---|---|
| Coincident | point, point | 3 | `$p-q$` |
| Coincident | point, plane | 1 | `$n\cdot(p-q)$` |
| Coincident | point, line | 2 | `$p-q$` along two directions across the line |
| Coincident | line, line | 4 | parallel (2), one point off the other line (2) |
| Coincident | plane, plane | 3 | parallel (2), one on the other (1) |

Note how "parallel" and "off a line" are always two rows *across* the direction. A vector cross product has three components and rank two, which would give a redundant row at every solution, right at the point the Krawczyk test wants independence. Choosing rows in a basis across the direction avoids having to detect it. Picking what you ask for is cheaper than a rank decision.

### One table of joints

`Joint::frees` is one table: how each joint lets the second frame turn, and the motions it frees as coordinates. Every one is held by one residual function, `joint_residuals` in `ops/geop-ops-assembly/src/joints.rs`. The second frame's origin must be at the first's, moved along its axes by the joint's slides. Its axes must be turned about the first's `$z$` by its angle, or kept square to it for a universal joint's cross, or free for a ball.

| Joint | Coordinates | Rows | Freedom |
|---|---|---|---|
| `Fastened` | none | 3 + 2 + 1 | 0 |
| `Revolute` | angle | 3 + 2 + 1 | 1 |
| `Slider` | slide | 3 + 2 + 1 | 1 |
| `Cylindrical` | angle, slide | 3 + 2 + 1 | 2 |
| `PinSlot` | angle, slide `$x$` | 3 + 2 + 1 | 2 |
| `Planar` | angle, slide `$x$`, slide `$y$` | 3 + 2 + 1 | 3 |
| `Universal` | none as a coordinate | 3 + 1 | 2 |
| `Ball` | none as a coordinate | 3 | 3 |

Count a revolute joint between a fixed body and a free one. The variables are the body's pose (6) and the coordinate `$\theta$` (1): seven. The rows are 3 for the origin, 2 for the axes parallel, and 1 for the turn: six. `$7-6=1$`. That is what a hinge should have.

**The coordinate is a variable.** `$\theta$` is a value of the state, named after the mate (`joint_parameter`: `add_part(arm2,m1).angle`), that can be set, held, or limited. Its `Mobility` is `Follows`: move it as the constraints have it, and hold it nowhere. It follows its parts, since held as well, it would pull against them.

A ball's and a cross's turn is no coordinate at all, because angles of them go singular. Do not make a value of something with no good global coordinate: that was the lesson of the demo.

### The angle row, and a free choice that matters

How should "`$b$`'s reference is turned from `$a$`'s by `$\theta$`" be written? Let `$u$` be `$a$`'s reference turned by `$\theta$`, let `$t$` be its direction a quarter turn on, and let `$r_b$` be `$b$`'s reference. The obvious candidates are bad:

- `$r_b-u$`, the difference of two unit vectors. The component along `$u$` vanishes to *second order*. At a solution reached to tolerance, this row has a slope as small as that tolerance and still counts as a constraint, pinning the joint. This is the "redundant row looks independent" of [Lecture 24](./24-rank-and-proof.md), self-inflicted.
- `$\sin(\varphi-\theta)=r_b\cdot t$`, with `$\varphi$` the true angle. One row of slope one, except that a sine is zero half a turn away as well. The solve can land on the wrong branch.

The kernel's row is

`$$2\tan\frac{\varphi-\theta}{2}=\frac{2\,r_b\cdot t}{1+r_b\cdot u}.$$`

Zero only at `$\varphi=\theta$`, one row, slope one there. (Check: `$r_b\cdot t=\sin(\varphi-\theta)$` and `$r_b\cdot u=\cos(\varphi-\theta)$`, and `$\tan(x/2)=\sin x/(1+\cos x)$`.) Half a turn away the denominator goes to zero and the row fails, honestly: infeasible, a step turned down. Then it is multiplied by the scale `$\ell$`, to be a length.

## What a pick means

Here is something a solver's author would never guess. A cam swept along a sketched circle is four faces, a quarter each, and a follower held tangent to the one picked ran to its end and stopped: the solve met the foot point's limit a quarter turn on. Nothing was wrong with the solve. What the user picked was the wall, and the face was only a piece of it (AGENTS.md, *A face is a piece of a wall*).

A `Tangent` mate keeps a circle (a roller's rim, a pin's) its radius off a face. A `Path` mate keeps a point on a curve. Where on it is the *foot point*, whose parameters are values of the state (`foot_parameter`: `add_part(follower,m2).u`), solved with the poses. Its residuals are the follower's point less the foot point.

Now `mate_entity::wall` in `ops/geop-ops-model/src/mate_entity.rs` takes a face and joins it with every face it runs on into across an edge, tangent to it, into one surface (`NurbSurface3D::joined`). The faces are taken in the order of their ids, since a hash map's order is no order. And `mate_entity::run` does the same for an edge and the edges it carries on into. A parameter along which the joined wall closes on itself wraps round instead of ending; one along which it does not is kept within its domain by limits. So a follower now rolls round every face of a cam.

Where a pick names a piece of something the geometry continues, ask whether the operation means the piece or the whole.

## A reference direction is not left to noise

A joint measures its turns from a direction square to its axis, chosen from the world's axes as the one the axis runs least along. An axis along `$z$` is a hair off it in `$x$` and in `$y$` alike. So which of the two is least was decided by noise, differently for the two ends of a joint: a quarter turn or none.

`Vector3::across` in `core/geop-core-math/src/vector/linalg.rs` takes **the first axis that no other component is definitely less than**. For an interval-valued axis this is stable: if `$x$` and `$y$` components could be equal, neither is definitely less, so the choice falls on the first one, at both ends. It is the interval version of "use a tie-breaker that does not depend on noise". And a frame put on a face or an edge, `frame_along` in `ops/geop-ops-model/src/geometry.rs`, gives the same direction the joint would have chosen. So frames and the entities they are on are at a turn of zero alike, and a joint clicked on a hole begins at angle zero.

<div class="kernel">

**In the kernel.** Poses: `core/geop-core-math/src/primitives/pose.rs` (`Pose::exp`, `Pose::exp_differential`, `impl Variable for Pose`). Mates: `ops/geop-ops-assembly/src/mates.rs`, `ops/geop-ops-assembly/src/joints.rs`, `ops/geop-ops-assembly/src/residuals.rs`. What a mate's steps declare: `ops/geop-ops-assembly/src/mechanism.rs` (constraints through `define_constraints`, unknowns through `declare_unknowns`). Elements: `MateEntity` in `ops/geop-ops-model/src/mate_entity.rs`. Limits are constraints of their own, `Bound::AtMost`.

Not implemented: the solver knows nothing of collision. Parts pass through each other unless mates forbid it. `MotionPanel` plays a motion study and can ask the interference inspector about every frame ([Lecture 37](./37-assembly-solver.md)), but no contact is solved other than a `Tangent` or `Path` mate.

</div>

## Recap

<div class="recap">

- A pose is a quaternion and a position. The quaternion need not be unit; the reader keeps what it read; an enclosure normalizes honestly.
- A global coordinate of a turn reaches half a turn only at infinity. Steps are twists from where the body is, through `Pose::exp`, and no turn is ever far from the origin of the variables.
- A mate is residuals, each a length. Rows are chosen in a basis across a direction, so that redundancy does not arise by construction.
- `Joint::frees` is one table, `joint_residuals` one function. The angle row `$2\tan\frac{\varphi-\theta}{2}$` is the one with slope one and a single zero.
- A pick may name a piece of a wall or a run: ask what the geometry continues. A direction chosen from data needs a tie-break the noise cannot flip.

</div>

## Exercises

**1.** Compute `$qv\bar q$` for `$q=(\cos\frac\theta2,0,0,\sin\frac\theta2)$` and `$v=(1,0,0)$`, and confirm the result `$(\cos\theta,\sin\theta,0)$`. Now scale `$q$` by 2 and apply `$qv\bar q$` without dividing. What comes out, and what must `Pose::apply` do?

<details class="solution"><summary>Hint</summary>

Expanding gives `$(\cos^2\frac\theta2-\sin^2\frac\theta2)\,e_x+2\sin\frac\theta2\cos\frac\theta2\,e_y$`. Scaled by 2, the result is 4 times longer: `$\lvert q\rvert^2$`. So applying a non-unit quaternion divides by its norm squared.

</details>

**2.** In the model problem, a pull with damping `$\mu$` and slope `$\kappa=d\theta/dw$` near the target gives the step `$\delta w=-\kappa r/(\kappa^2+\mu^2)$`, with `$r$` the angle error. Show that the angle error then shrinks per step by the factor `$\mu^2/(\kappa^2+\mu^2)$`. Evaluate it at `$179.5°$` with `$\mu=10^{-4}$`. How many steps to gain a factor of `$10^6$`?

<details class="solution"><summary>Solution</summary>

`$\delta\theta=\kappa\,\delta w=-\frac{\kappa^2}{\kappa^2+\mu^2}r$`, so the new error is `$\frac{\mu^2}{\kappa^2+\mu^2}r$`. At `$179.5°$`, `$w=458.4$`, `$\kappa=1/(1+w^2/4)=1.9\times10^{-5}$`, `$\kappa^2=3.6\times10^{-10}$`, `$\mu^2=10^{-8}$`. The factor is `$0.965$`. Then `$\ln10^6/\ln(1/0.965)\approx390$` steps, past the demo's cap of 200 as it shows. With a twist, `$\kappa=1$` always, and the factor is `$10^{-8}$` per step.

</details>

**3.** Evaluate `Pose::exp`'s position formula for `$\omega=(0,0,\pi)$`, `$v=(0,0,1)$`, and interpret the motion. What about `$\omega=(0,0,0)$`, `$v=(a,b,c)$`? Why can't the code use the closed form for `$s$` and `$c$` there?

<details class="solution"><summary>Solution</summary>

`$\phi=\pi/2$`, `$s=1/\pi$`, `$\cos\phi=0$`, `$c=(0-(1/(\pi/2)))/(8\pi^2/4)=-(2/\pi)/(2\pi^2)=-1/\pi^3$`. Then `$2s\cos\phi\,v=0$`, `$\omega\cdot v=\pi$`, `$(s^2+0)\,\pi\,\omega=(1/\pi^2)\pi(0,0,\pi)=(0,0,1)$`, and `$\omega\times v=0$`. Position `$(0,0,1)$`: half a turn while advancing by one, a screw. For `$\omega=0$`: the formula gives `$v$`, a pure translation, using the limits `$s=\frac12$`, `$\cos\phi=1$`. The closed form has `$\phi$` in the denominator, `$0/0$`, hence the series.

</details>

**4.** For a `Cylindrical` joint between a fixed body and a free one, count the variables and rows. Do the same for a `Ball`. What if two `Revolute` joints on parallel axes join the same two bodies?

<details class="solution"><summary>Solution</summary>

Cylindrical: 6 + 2 coordinates = 8 variables, 3 + 2 + 1 = 6 rows, freedom 2. Ball: 6 variables, 3 rows, freedom 3. Two revolutes, coaxial: 6 + 2 = 8 variables, 12 rows, rank 7: freedom 1, 5 rows redundant. If their axes are parallel and distinct: the rows cannot all vanish, since no pose has its axis on both. That is a conflict of the geometry, not of the arithmetic, which `conflicting` reports.

</details>

**5.** Verify that `$2\tan\frac{x}{2}=\frac{2\sin x}{1+\cos x}$` has slope one at `$x=0$` and no other zero in `$(-\pi,\pi)$`. Show that the component of `$r_b-u$` along `$u$` has zero slope in `$x$` at `$x=0$`, and explain with [Lecture 24](./24-rank-and-proof.md) what this would do to a rank decision.

<details class="solution"><summary>Solution</summary>

With `$x=\varphi-\theta$`, `$r_b\cdot u=\cos x$`, so the component of `$r_b-u$` along `$u$` is `$\cos x-1\approx-x^2/2$`: slope `$-x$`. At a solution to tolerance, `$x\approx10^{-9}$` gives a pivot of `$10^{-9}$`, small but maybe definitely not zero, so it counts as an independent row, and pins `$\theta$` to `$\varphi$` where there is no such constraint of its own beyond the single turn row. That is exactly how a redundant row looks independent at a point.

</details>

**6.** Why is a wall's seam, in `mate_entity::wall`, handled by wrapping a parameter, rather than by keeping the foot point on one face and switching faces when it reaches an edge?

<details class="solution"><summary>Hint</summary>

A solver's variables must be smooth: switching faces is a discontinuity in `$f$` and `$J$`, where no Newton step is valid and the rank test may disagree on each side. On one joined surface, with a parameter wrapping round, the foot point moves continuously across what were edges. It is also the same trick as `Coincident`: remove the case from the equations, by construction.

</details>
