# Lecture 24: Rank and proof

<div class="goals">

**After this lecture you can**

- derive Krawczyk's operator from the mean value theorem, and prove that `$K(X)\subseteq X$` with a norm bound gives exactly one solution in `$X$`;
- explain why a floating-point solution is not enough for a kernel, and what `Sketch::enclose` gives instead;
- separate a redundant constraint from a singular configuration and from a conflict, by what each does over a box;
- say why every rank decision must be made over a box, and which box;
- state what interval arithmetic can prove about constraints, and what it can never prove.

</div>

The last lecture ended with a sketch whose points the solver put somewhere so that every residual is below one part in a billion of the sketch's size. Why is that not good enough?

Take a rectangle extruded into a box. The solver reports its top line as `Horizontal` to the solver's tolerance: the two ends' heights differ by perhaps `$10^{-10}$`. Built as though exact, that line tilts a face by a hair where the designer meant square. It is a real feature, and a boolean with another box later finds it: a sliver of tiny faces where two planes that should be one cross at an angle of `$10^{-10}$`. Enclosures are supposed to save us from this, and an interval around "the height" could. But around which value, and how wide? The answer has to come from the constraints themselves.

So we ask for something stronger than a small residual: **a box of numbers, proven to contain a true solution of the constraints.** The geometry built from the box then carries exactly the uncertainty the solve left, and the interval tests downstream can see that the two faces may be one. This lecture is about how to obtain that proof.

## Newton's method, in intervals

We want to show that `$f(x)=0$` has a solution in a box `$X\subset\mathbb{R}^n$`, for `$f:\mathbb{R}^n\to\mathbb{R}^n$`. For the moment let the system be square and regular. The naive idea is to run Newton with interval numbers. It fails, because Newton divides by `$J$`, and `$J$` over a box is a whole family of matrices.

Krawczyk's idea (1969) is to not invert an interval matrix at all. Pick **any** matrix `$Y$`, a free choice that should be good: the inverse of the Jacobian at the middle of the box. Pick a sharp point `$\tilde x$` in `$X$`. Then define

`$$g(x)=x-Yf(x).$$`

A zero of `$f$` is a fixed point of `$g$`. Now apply the mean value theorem to each component of `$f$`: for any `$x\in X$`, there is a matrix `$\bar J$`, each row the gradient of that row of `$f$` at a point of the segment from `$\tilde x$` to `$x$`, with

`$$f(x)=f(\tilde x)+\bar J\,(x-\tilde x).$$`

Every row of `$\bar J$` lies in the row of the interval Jacobian `$J(X)$`, so `$\bar J\in J(X)$`. Substituting,

`$$g(x)=\tilde x-Yf(\tilde x)+(I-Y\bar J)(x-\tilde x)\ \in\ \underbrace{\tilde x-Yf(\tilde x)+\bigl(I-YJ(X)\bigr)(X-\tilde x)}_{K(X)}.$$`

`$K(X)$` is the **Krawczyk operator**. It is computed with plain interval arithmetic: one evaluation of `$f$` at a point, one of `$J$` over the box, and a matrix product. What we have is a theorem with three parts.

**(a) No solution is lost.** If `$x\in X$` has `$f(x)=0$`, then `$x=g(x)\in K(X)$`. So every root in `$X$` lies in `$K(X)$`. In particular, if `$K(X)\cap X=\emptyset$`, then `$X$` holds no root.

**(b) Existence.** If `$K(X)\subseteq X$`, then `$g$` maps the compact convex box `$X$` into itself, and it is continuous. Brouwer's fixed point theorem gives a fixed point: a zero of `$Yf$` in `$X$`.

**(c) Uniqueness and regularity.** Suppose moreover that `$\lVert I-YJ(X)\rVert_\infty<1$`. For every matrix `$A$` in `$J(X)$`, `$\lVert I-YA\rVert<1$` makes `$YA$` regular, hence `$Y$` and `$A$` regular. So the zero of `$Yf$` from (b) is a zero of `$f$`, and if `$x,y$` were two of them, then `$0=f(x)-f(y)=\bar J(x-y)$` with regular `$\bar J$`, so `$x=y$`. And "regular over the whole box" says the independent rows stay independent over the whole box.

So `$K(X)\subseteq X$` together with the norm bound proves that `$X$` contains **exactly one** solution of `$f$`. Rounding is no threat: with outward rounding, the computed `$K(X)$` contains the true one, so if the computed `$K(X)$` fits inside `$X$`, the true one does too. The proof runs on a computer.

Note what `$Y$` is. It is a free choice. It does not enter the conclusions at all; only whether the test passes depends on it. `$Y$` need not be accurate, and it should be computed once and carried as an honest enclosure, or as plain numbers treated as exact. And `$\tilde x$` is a free choice too.

### A worked example

Take the unit circle and the line `$y=0.6$`:

`$$f(x,y)=\begin{pmatrix}x^2+y^2-1\\ y-0.6\end{pmatrix},\qquad J(x,y)=\begin{pmatrix}2x&2y\\0&1\end{pmatrix}.$$`

Let `$\tilde x=(0.8,0.6)$`, a root, and take the box of half width `$0.1$`. Then

`$$J(\tilde x)=\begin{pmatrix}1.6&1.2\\0&1\end{pmatrix},\qquad Y=\begin{pmatrix}0.625&-0.75\\0&1\end{pmatrix},\qquad J(X)=\begin{pmatrix}[1.4,1.8]&[1.0,1.4]\\0&1\end{pmatrix}.$$`

Multiplying,

`$$I-YJ(X)=\begin{pmatrix}[-0.125,0.125]&[-0.125,0.125]\\0&0\end{pmatrix},$$`

with norm `$0.25<1$`. Since `$f(\tilde x)=0$`,

`$$K(X)=\tilde x+\bigl(I-YJ(X)\bigr)\begin{pmatrix}[-0.1,0.1]\\ [-0.1,0.1]\end{pmatrix}=\begin{pmatrix}0.8+[-0.025,0.025]\\0.6\end{pmatrix}.$$`

`$K(X)=[0.775,0.825]\times\{0.6\}$`, well inside `$X$`. The test passes. In this box there is one point on both the circle and the line. Notice how much it contracted, by a factor of 4 from the box: the bound `$\lVert I-YJ(X)\rVert$` is the contraction. It is proportional to the box's width, since `$J(X)$` deviates from `$J(\tilde x)$` by that much.

<figure class="demo" data-demo="krawczyk">
<figcaption>The test itself, in interval arithmetic rounded outwards. Drag the centre. Change the box's size and the line. With the centre off the root, a small box's image lies outside it, which proves that box empty (part a); a larger one proves one root; a very large one proves nothing. "Newton polish" and "Widen until proven" are the kernel's two moves. "Tangent" sets the line on the circle, where no box ever passes. Simplified: JavaScript cannot set the rounding mode, so each operation is widened by one unit in the last place.</figcaption>
</figure>

### How wide should the box be?

There is a Goldilocks zone. Too small a box and `$K(X)$` falls outside it, because `$\tilde x-Yf(\tilde x)$` is a whole Newton step away from `$\tilde x$`. Too big and the norm exceeds one, and the spread blows `$K(X)$` past the box. So: polish first and then widen. `Solving::prove` in `core/geop-core-math/src/solvers/system/enclose.rs` does exactly this:

1. **Newton first.** At most `POLISH_STEPS` (6) steps, each squaring the error, on the determined variables. Each iterate is sharp, being only a seed for the next. The last pass takes no step: it is the point the box is proven around.
2. **Start from what the step still reaches.** The first candidate half-width is twice `$\lvert Yf(\tilde x)\rvert$`, per variable.
3. **Grow only as needed.** If `$K\not\subseteq$` box, the new radius is what `$K$` reached. If the radius is no longer finite, the system is singular. At most `ENCLOSE_ATTEMPTS` (24) tries: how hard it tries, never what a verified box means.
4. **Check the norm.** If `$K\subseteq$` box but the norm bound fails, the rows are not proven independent, and that is an error, not a success.
5. **Return `$K\cap$` box.** Both enclose the solution, so their intersection is honest and tighter. The rule of [Lecture 5](./05-honest-numerics.md) again: intersect two enclosures of one value.

In the demo, after "Newton polish" with the line at height 0.6, "Widen until proven" reports a half-width of `$10^{-14}$` at the very first attempt. That width says how precisely the constraints pin down the solution at this scale: a few rounding errors. A well-conditioned sketch comes out with vertices of width near the machine precision; a badly conditioned one comes out wider, *honestly*.

Formally, the kernel writes the operator for a box `$\Delta$` of steps around `$\tilde x$`,

`$$K(\Delta)=-Yf(\tilde x)+\bigl(I-YJ(\Delta)\bigr)\Delta,$$`

the same thing with the origin moved to `$\tilde x$`. That move is not cosmetic. Part VIII needs it, since a pose's step `$\Delta$` is not added to the pose but retracted along its own tangent space ([Lecture 36](./36-mates.md)). `$J(\Delta)$` is the Jacobian at each value the box reaches, times the differential of the retraction there (`Variable::differential`). For a number the differential is the identity.

## Non-square systems: which rows, which variables?

So far `$f$` was square. A real sketch has `$m$` rows and `$n$` unknowns, usually not equal. Two things must be decided before the proof can start: **which rows are independent**, and **which variables they determine**. The rest of the variables are *free*, the designer's choice, and they stay exactly as they are. The rest of the rows are *left out*, and have to be checked afterwards.

`eliminate` in `core/geop-core-math/src/solvers/linear.rs` decides this by Gauss-Jordan with complete pivoting ([Lecture 23](./23-sketch-solver.md)). A pivot is the largest entry that is definitely not zero, among the rows and columns not used yet. Elimination stops when every candidate could be zero. Each pivot `(column, row)` assigns one variable to one row. The result is a square system on the determined variables, with the free ones held.

So everything hangs on a rank decision. And now comes the central difficulty of this lecture.

### A redundant row looks independent at a point

Take a rectangle's fourth right angle, or an arc concentric with a center it already has. These constraints *say what the others say*, but they say it on the solution only. Where the blocks are now, a solution merely to the solver's tolerance, the row's remainder after elimination is not exactly zero. It has a pivot about as small as the tolerance: say `$10^{-10}$`. As an interval that pivot may very well exclude zero.

Elimination at the point would then count the row as independent. It takes a variable for it. The square system now has too many rows for what the constraints really determine; its solution set is a curve rather than a point, and nothing can prove a curve is a point. The old code rejected such sketches, so the sketch tools had to avoid redundancy by hand. Worse, as AGENTS.md records, it proved an "exact solution" for two lengths `$10^{-11}$` apart.

The fix, from *Decide a rank over the box you prove, not at a point*: **the rows are chosen over a box around the point**, widened from the point itself through the solver's tolerance. A pivot that could be zero anywhere in the box is no pivot. Once the box reaches the true solution, a redundant row's pivot straddles zero, and the row is left out.

Concretely, `enclose_group` tries in order: the point itself; a box as wide as the tolerance (`tolerance`, one part in `RELATIVE_TOLERANCE` of the size); then twice as wide, and so on. Each distinct choice of pivots is put to the Krawczyk test. The first that passes wins.

Why try the point first? Because when no redundancy is present, the point's pivots are the best ones. And the box must not start too large, or a real pivot of a tiny feature straddles zero too.

### The rank must be checked again, over the proven box

Here is the trap in that fix, and it is a very good one. Take a point on the unit circle and on its tangent `$y=1$`:

`$$f(x,y)=\begin{pmatrix}x^2+y^2-1\\ y-1\end{pmatrix}.$$`

There is exactly one solution, `$(0,1)$`, and it is singular: at it, both rows have gradient `$(0,\cdot)$`, and `$J$` has rank 1. Let the solve end at `$x=1.9\times10^{-8}$`, as AGENTS.md records. Eliminating over a box as wide as the solver's tolerance: the circle row's `$\partial/\partial y=2$` pivots on `$y$`, and the tangent row's remainder is about `$-x$` in `$x$`, a pivot that could be zero over that box. So the row is left out, and `$x$` counts as **free** and is held where the solve left it. The box then proves `$y$` by the circle alone: a beautiful proof that `$y=\sqrt{1-x^2}$`, for the `$x$` we are holding.

And the left-out tangent row, checked over the box? Its value is about `$y-1\approx-x^2/2$`, of size `$2\times10^{-16}$`, less than the box's width. It encloses zero. Everything seems consistent, and a "true solution" `$(1.9\times10^{-8},1)$` is returned, *where there is none*.

The catch: the row's **derivative** over the proven box is about `$-1.9\times10^{-8}$` in `$x$`, a very small number that is definitely not zero. Over this box, unlike the wider one, the row *is* independent. A constraint the others truly imply leaves the rank as it is wherever they all hold. This one does not. So:

> **A left-out row must vanish over the proven box, and it must add nothing to the rank there.**

`Solving::verify` runs this second elimination over the proven box, with the rule that if there are more pivots than picked rows, then "the constraints are singular at their solution". The test `a_tangency_is_not_proven_in_every_scalar` in `core/geop-core-math/src/solvers/system/tests.rs` drives this at `$x=0.3,\ 10^{-4},\ 10^{-6},\ 10^{-8}$`: no box is claimed, and no conflict either.

The demo's "Tangent" button shows the deeper reason no box can pass. With `$\tilde x=(x,1)$`,

`$$J(\tilde x)=\begin{pmatrix}2x&2\\0&1\end{pmatrix},\quad Y=\frac{1}{2x}\begin{pmatrix}1&-2\\0&2x\end{pmatrix},\quad I-YJ(X)=\begin{pmatrix}\mp r/x&\mp r/x\\0&0\end{pmatrix}.$$`

(using a box of half-width `$r$`, whose `$2(x\pm r)$` and `$2(1\pm r)$` are the first row of `$J(X)$`). So `$\lVert I-YJ(X)\rVert=2r/x$`, and regularity requires `$r<x/2$`. But the only root is `$x$` away from `$\tilde x$`, so a box containing it needs `$r\ge x$`. Those are incompatible: however close the solve ends, **no box both contains the root and proves it regular**. The demo reproduces this: at `$x=0.01$`, half-width `$0.005$` gives a norm of `$1.00$`; half-width `$0.001$` passes the norm test, and correctly proves *that* box empty.

## Three things that look alike

We can now give precise meanings to the three cases that all present as "an extra row":

| | Row's value over the box | Row's rank contribution over the box | What it means | What is returned |
|---|---|---|---|---|
| **Redundant** | encloses 0 | none | says what the others say | a proof; freedom as if the row were not there |
| **Singular** | encloses 0 | adds rank | true at a point where independent rows lose rank | `EncloseError::Unproven`, "singular at their solution" |
| **Conflict** | definitely not 0 for a combination, over a box in every variable | n/a | no solution near here | `EncloseError::Conflict`, naming the constraints |

Getting a conflict right takes care of its own. A left-out row that does not vanish over the proven box is *not yet* a conflict, since the free variables were held: other values of them might satisfy it. So `Solving::conflict` looks for a real proof. The combination `$g$` of the rows that elimination reduced the row to vanishes wherever all the rows do. Over a box `$B$` **in every variable**, by the mean value theorem, `$g\in g(x)+\nabla g(B)\,B$`. If this is definitely not zero, no point of `$B$` is a solution. Only that gets the name of conflict, with the box's size stated: "cannot hold where the others do within `$\rho$` of here". The test `a_line_missing_a_circle_conflicts_in_every_scalar` shows the unit circle with `$y=1.5$`.

And here is the epistemic heart of the matter. **Interval arithmetic can prove a conflict, never its absence.** A row that encloses zero agrees with the others to every digit the box resolves. It might still be off by `$10^{-30}$`. So do not say "implied"; say "consistent to what the box resolves". This is not a weakness to engineer away. In a floating-point world, two lengths equal to the last digit and two lengths that differ in the 40th are the same input.

## Enclosing what a value stands for

When the variables are poses, a new trap opens. A pose is a quaternion and a position, seven numbers for six degrees of freedom. A solver's step sharpens both, so the quaternion comes out a hair off unit. AGENTS.md records what happened when a solved revolute chain was enclosed as `$\tilde x\circ\exp(\delta)$` for a box of twists `$\delta$`: every pose in the box kept that norm. Turning a point by a quaternion off unit scales the point by its norm squared. So *no pose in the box was a rigid motion*. The true solution was in none of them. A joint's redundant row came out `$10^{-15}$` off zero over the proven box: a "conflict" between constraints that agree.

`retract_over` in `core/geop-core-math/src/primitives/pose.rs` now makes the rotation unit honestly, by an interval normalization, *before* it steps. The general lesson: wherever a representation has more numbers than the thing it stands for (a quaternion for a rotation, homogeneous coordinates for a point), ask whether an enclosure covers the thing, not just the numbers.

<div class="kernel">

**In the kernel.** `System::enclose` in `core/geop-core-math/src/solvers/system/mod.rs` is the entry point, group by group, and its detailed module comment in `core/geop-core-math/src/solvers/system/enclose.rs` is this lecture in compressed form. Over the box it proved, `System::freedom` gives how many independent ways each block can move: the null space of the rows over that box (`null_space` in `core/geop-core-math/src/solvers/linear.rs`).

For sketches, `Sketch::enclose` in `core/geop-core-sketch/src/solve.rs` returns an `Enclosure`: every point, arc sweep and circle radius as a box around the exact solution, with free variables exactly as drawn. `ProfileLoop::to_nurbs` builds NURBS from it ([Lecture 25](./25-sketch-features.md)). A sketch whose constraints are not met is no solution of them and is built as drawn (`Enclosure::as_drawn`). One that meets them only at a singular configuration fails to enclose.

`SolveReport::unproven` carries the failure messages, and `SolveReport::dof` the rank decision. The `Sketch3d` of [Lecture 25](./25-sketch-features.md) and the assemblies of [Lecture 37](./37-assembly-solver.md) use all of this unchanged: the algorithm has no notion of what its variables mean, beyond `Variable`.

</div>

## Recap

<div class="recap">

- A small residual is no proof. `$K(X)=\tilde x-Yf(\tilde x)+(I-YJ(X))(X-\tilde x)$` turns the mean value theorem into one.
- `$K(X)\subseteq X$` gives existence by Brouwer; `$\lVert I-YJ(X)\rVert<1$` adds regularity and uniqueness; every root in `$X$` is in `$K(X)$`.
- Polish by Newton, then widen from what a Newton step still reaches. Return `$K\cap X$`.
- Every rank decision is made over a box: to choose the rows, around where the solve ended; to accept them, around the solution proven.
- Redundant, singular and conflicting differ in what the row does over the box. Interval arithmetic proves conflicts, never their absence.

</div>

## Exercises

**1.** For the line `$y=0.6$` and the unit circle, take `$\tilde x=(0.9,0.5)$` and the half-width `$r=0.2$`. Compute `$Y$`, `$f(\tilde x)$`, `$K(X)$` and the norm by hand, and confirm that the box is proven. Check it against the demo.

<details class="solution"><summary>Solution</summary>

`$f(\tilde x)=(0.81+0.25-1,\ 0.5-0.6)=(0.06,-0.1)$`. `$J(\tilde x)=\begin{pmatrix}1.8&1\\0&1\end{pmatrix}$`, so `$Y=\begin{pmatrix}1/1.8&-1/1.8\\0&1\end{pmatrix}$`. `$Yf=(0.16/1.8,\,-0.1)=(0.0889,-0.1)$`, so the point `$\tilde x-Yf=(0.8111,0.6)$`. `$J(X)$` has first row `$([1.4,2.2],\ [0.6,1.4])$`. `$I-YJ(X)$` has first row `$(1-[1.4,2.2]/1.8,\ -([0.6,1.4]-1)/1.8)=([-0.222,0.222],\ [-0.222,0.222])$` and a zero second row. The norm is `$0.444$`. So `$K_1=0.8111+2\cdot0.222\cdot0.2\cdot[-1,1]=[0.7222,0.9]$`, and `$K_2=0.6$`. Both lie in `$[0.7,1.1]\times[0.3,0.7]$`: proven.

</details>

**2.** Prove part (a) of the theorem without Brouwer: any root in `$X$` lies in `$K(X)$`. Why is this true even when `$Y$` is a terrible choice? What happens to the use of this fact when `$Y$` is terrible?

<details class="solution"><summary>Hint</summary>

`$f(x)=0$` implies `$g(x)=x$`, and `$g(x)\in K(X)$` by the mean value form, for any `$Y$`. A terrible `$Y$` makes `$K(X)$` enormous: it remains a valid enclosure, it merely proves nothing.

</details>

**3.** Two lines are constrained `Parallel` and then `Horizontal` each. At the sketch's solved point the parallel row's remainder is `$3\times10^{-11}$`. Describe what elimination at the point does, what elimination over the box of half-width `$10^{-9}\ell$` does, and which of them the Krawczyk test can accept.

<details class="solution"><summary>Solution</summary>

At the point, `$3\times10^{-11}$` may be definitely not zero, so the row is counted as independent and takes a variable it barely determines. The Jacobian of the square system is nearly singular, so the proof fails (as a norm too large). Over the box, the remainder is that small value plus a spread of about the box's width times the row's curvature, which encloses zero, so no pivot is taken. The row is left out and the square system has good pivots. The left-out row encloses zero over the proven box and adds no rank: it is redundant.

</details>

**4.** Why is a proof over the box of *steps* of a pose not simply a proof over a box of seven numbers (quaternion and position)? Give two reasons.

<details class="solution"><summary>Hint</summary>

Seven numbers for six freedoms: a box in seven numbers has a seventh direction, off the unit norm, along which every row is a function too, so the system is not square, and is not regular in that direction. And (see the section on what a value stands for) a box around a non-unit quaternion holds no rigid motion at all. The retraction parametrizes rigid motions directly.

</details>

**5.** Suppose a sketch's solve ends well, the Krawczyk test passes, and `SolveReport::dof` says 2. A user adds a dimension that is exactly redundant with those already there, as computed by hand in decimal. What should happen? Why can the solver not distinguish that from a dimension differing by `$10^{-20}$`?

<details class="solution"><summary>Hint</summary>

The row encloses zero over the proven box and adds no rank: redundant, `dof` still 2. A difference of `$10^{-20}$` is below what the box resolves, so the two inputs get identical treatment: "consistent to what the box resolves". Interval arithmetic can prove a conflict only down to its width.

</details>

**6.** In the demo, set the line to a height of `$1.2$`. No root exists. Which verdicts can you obtain by moving the centre and the box? Can it ever claim one root? Why not?

<details class="solution"><summary>Solution</summary>

"None" and "undecided" only. "Proven" would mean a true root exists in `$X$`, by the theorem (the arithmetic being outward rounded). The test is sound, so as there is no root, it can never pass: at some boxes `$K(X)$` lands outside `$X$` (proving it empty), at others it is simply too wide to tell.

</details>
