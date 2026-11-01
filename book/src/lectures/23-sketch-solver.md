# Lecture 23: Solving a sketch

<div class="goals">

**After this lecture you can**

- write any geometric constraint as a residual and count what it takes away;
- derive the step of Gauss-Newton on a system with more unknowns than equations, and say why the shortest step is the right one;
- explain why the kernel solves least squares by Householder reflections and pivots completely, and what went wrong when it did not;
- describe how a drag is a preference among the configurations the constraints allow, not a constraint itself;
- read a `SolveReport` and tell an under-constrained sketch from a redundant one and from a conflicting one.

</div>

Nearly every solid in a CAD program starts as a flat drawing. A designer draws a rough outline and then says what it means: these two lines are parallel, this one is 40 long, this arc is tangent to that line. The kernel's job is to find where the points must go for all those statements to be true, as the designer drags things around, many times a second, for a sketch that may have hundreds of variables.

This is the first place in the book where we solve nonlinear equations with no geometry to subdivide. Part III found every intersection of two curves by chopping up a parameter box. A sketch has hundreds of unknowns, and no subdivision search survives that: a box in 300 dimensions has `$2^{300}$` corners. We rely instead on a good starting point, which the designer's drawing provides, and on Newton's method. In [Lecture 24](./24-rank-and-proof.md) we then get back what we gave up by proving, in interval arithmetic, that there is a true solution next to the one Newton found.

## Entities and variables

A sketch in Geop holds *points* and *curves*. Points are the only entities with positions of their own; a curve refers to its points by id. So two lines that share an end share one `PointId`, and they cannot come apart, however the solver moves things. Making connectivity a fact of the data rather than a constraint removes a whole class of failures.

| Entity (`CurveKind`) | Defined by | Its own extra variable |
|---|---|---|
| `Line` | start, end | none |
| `Arc` | start, end, `sweep` | its half sweep |
| `Circle` | center, `radius` | its radius |
| `Spline` | control points, optional `SplineShape` | none |
| `Text` | anchor, string, height, angle | none (only the anchor is solved for) |

These live in `core/geop-core-sketch/src/sketch.rs`. A point is `x`, `y` and a flag `fixed`, which marks something *given* rather than solved for: the sketch's origin, or an edge of the part projected into its plane ([Lecture 25](./25-sketch-features.md)).

An arc is stored by its *sweep*, the angle it turns through, and not by a radius or a centre. That choice is not arbitrary. Take an arc from `$s$` to `$e$` with half sweep `$\theta$`. Its radius is `$L / (2\lvert\sin\theta\rvert)$` where `$L=\lvert e-s\rvert$`, which blows up as the arc straightens out. Its sweep goes smoothly through zero. And the sweep tells a minor arc from a major arc of the same radius, which a curvature cannot. So the variable the solver moves is the one that stays smooth in every configuration a drag can reach (`Arc` in `core/geop-core-sketch/src/geometry.rs`). Remember this: a solve stalls when its variables cannot reach the answer at a finite, well-conditioned value, a lesson we will meet again in [Lecture 36](./36-mates.md).

The solver sees none of this structure, only one long vector `$x$` of numbers. `Layout` in `core/geop-core-sketch/src/solve.rs` builds the map: each *class* of coincident points gets one `$(x,y)$` pair, then each arc its half sweep and each circle its radius. This gives us the first trick of the lecture:

> `Coincident` is not a residual. It removes two variables by construction.

If two points are said to coincide, they are simply given the same place in `$x$`. There is no equation to hold to a tolerance, so there is no way for it to hold only approximately. Only where both are `fixed` is it a real check, since neither can move.

## Constraints as residuals

A constraint is a function of `$x$` that vanishes exactly when the constraint holds. Here is the core of the catalogue, as `core/geop-core-sketch/src/solve.rs` writes it. With `$u_a,u_b$` the unit directions of lines `$a,b$`:

| Constraint | Residual(s) | Rows |
|---|---|---|
| `Horizontal` | `$e_y-s_y$` | 1 |
| `Vertical` | `$e_x-s_x$` | 1 |
| `Fix` | `$p_x-v_x,\ p_y-v_y$` | 2 |
| `Distance` | `$\lvert b-a\rvert-v$` | 1 |
| `Parallel` | `$(u_a\times u_b)\,\ell$` | 1 |
| `Perpendicular` | `$(u_a\cdot u_b)\,\ell$` | 1 |
| `Angle` | `$\sin(\angle(a,b)-v)\,\ell$` | 1 |
| `Tangent` (line, circle) | `$\lvert d(s,e;c)\rvert-r$` | 1 |
| `Tangent` (circles, outside) | `$\lvert c_a-c_b\rvert-(r_a+r_b)$` | 1 |
| `Concentric` | `$c_a-c_b$` | 2 |
| `Midpoint` | `$q-m$` | 2 |

Here `$\ell$` is the sketch's characteristic size, and `$d(s,e;c)$` the signed distance of `$c$` from the line through `$s,e$`. Notice what `$\ell$` is doing. **Every residual is a length.** An angle's sine is dimensionless, so it gets multiplied by `$\ell$`, the diagonal of the box around the sketch's points, or a circle's diameter if larger (`Problem::new`). Without this, a sketch drawn in millimetres would weigh angles against lengths differently from the same sketch in metres, and the convergence test would need one tolerance per kind of constraint. With it there is one test: a residual *holds* once it is within one part in `RELATIVE_TOLERANCE` (`$10^9$`, in `core/geop-core-math/src/solvers/system/driver.rs`) of the sketch's size.

A free choice shows up in the tangent row. Two circles can touch from outside or from inside. `Self::tangent_mode` decides which once per solve, from the starting configuration, so a solve never flips what the designer drew. The arc's radius constraint is written `$2R\lvert\sin\theta\rvert-L$`, not `$L/(2\lvert\sin\theta\rvert)-R$`: the same zero set, but finite for a nearly straight arc.

**Derivatives** are not approximated. Each residual is written once, generically over a `Number` type, and evaluated on `Dual` numbers (`core/geop-core-math/src/dual.rs`). A dual number carries a value and a gradient; every arithmetic operation applies the chain rule to them. This is forward-mode automatic differentiation. Each constraint touches only a few variables (`MAX_LOCAL_VARS`, 17 at most), so the gradient is short. The derivative is exact up to rounding, and when the scalar underneath is an interval, it is an enclosure of the true derivative. That will matter next lecture.

## Counting freedom

Let `$n$` be the number of variables and `$m$` the number of residual rows, and let `$J(x)$` be their `$m\times n$` Jacobian. To first order the configurations near `$x$` that keep the constraints are the null space of `$J$`. So the number of independent ways the sketch can still move is

`$$\text{freedom} = n-\operatorname{rank}J(x).$$`

It is the rank that counts, and not `$m$`. Here is a rectangle drawn as four points with shared corners: `$n=8$`. `Horizontal` on top and bottom and `Vertical` on the sides give four rows that are obviously independent, leaving `$8-4=4$`: position (2), width and height. Add `Fix` on a corner (2 rows) and `Distance` for width and height (2 rows), and `$n-\operatorname{rank}=0$`. The sketch is *fully constrained*.

Now add `Perpendicular` between two adjacent sides. It is true, but it says nothing new. The sides are horizontal and vertical already. Its row is a combination of theirs, and `$\operatorname{rank}J$` stays 8 with `$m=9$`. Such a constraint is **redundant**. Now instead add `Distance` of 1 along the diagonal of a rectangle 3 by 2. Again `$m=9$`, and now no point satisfies all the rows. Such a constraint is **conflicting**. Both add a row without adding rank; to tell them apart requires looking at whether the row can vanish, and doing so honestly is the subject of [Lecture 24](./24-rank-and-proof.md).

<figure class="demo" data-demo="sketch-solver">
<figcaption>A real, small solver: dual-number gradients, Householder reflections, a pull among where the constraints hold, as described below. Tick constraints and drag points. Watch the rows, the rank and the freedom, and the residual per step. "Implied" constraints add a row and no rank. "Diagonal = 2" conflicts with a width and a height, and the sketch settles for least squares. Two simplifications: it decides rank by a tolerance on the pivot where the kernel asks an interval (the next lecture is why that matters), and it has no line search or filter.</figcaption>
</figure>

## Gauss-Newton when the system is not square

We want `$c(x)=0$`, with `$c:\mathbb{R}^n\to\mathbb{R}^m$`. Linearize at the current point:

`$$c(x+\delta)\approx c(x)+J\delta.$$`

In general `$m\ne n$`. A typical sketch while it is being drawn has `$m<n$`, with infinitely many `$\delta$` that zero the linear model. Which one should we pick? The natural answer in an interactive program is: **the shortest**. The designer drew the sketch where it is, so change it as little as the constraints demand. The answer to

`$$\min_\delta\ \lvert\delta\rvert^2\quad\text{subject to}\quad J\delta=-c$$`

is `$\delta=-J^{+}c$` with `$J^{+}$` the pseudoinverse. It lies in the row space of `$J$`: every component along a direction the constraints do not care about is zero. And that gives the property we want: *a variable no residual depends on never moves.*

### A worked example

Fix `$A=(0,0)$` and let `$B=(x,y)$` start at `$(4,3)$`. Ask for `Distance(A,B)=5` and `Horizontal(A,B)`. Then

`$$c=\begin{pmatrix}\sqrt{x^2+y^2}-5\\ y\end{pmatrix}=\begin{pmatrix}0\\3\end{pmatrix},\qquad J=\begin{pmatrix}0.8&0.6\\0&1\end{pmatrix}.$$`

Solving `$J\delta=-c$` gives `$\delta_y=-3$` and `$0.8\delta_x+0.6(-3)=0$`, so `$\delta_x=2.25$`, and `$B=(6.25,0)$`. Here `$c=(1.25,0)$` and `$J=I$`, so the second step gives `$B=(5,0)$` exactly. The first step overshot along the circle because the linear model does not know the circle curves. Near a solution that error shrinks quadratically.

### Least squares by reflections, not by normal equations

Almost every textbook gets from `$\min\lvert A y-b\rvert$` to `$A^{\mathsf T}A\,y=A^{\mathsf T}b$`. That formula is correct and it squares the condition number. Consider the classic matrix

`$$A=\begin{pmatrix}1&1\\ \varepsilon&0\\ 0&\varepsilon\end{pmatrix},\qquad A^{\mathsf T}A=\begin{pmatrix}1+\varepsilon^2&1\\1&1+\varepsilon^2\end{pmatrix}.$$`

With `$\varepsilon=10^{-8}$`, `$A$` has full rank with a condition number around `$10^{8}$`, which doubles can handle. But `$1+\varepsilon^2$` rounds to exactly 1 in `f64`, so the normal matrix is singular. The information was there, and forming the product destroyed it.

Geop learned this the hard way. From AGENTS.md: the minimizer's step met its linearized constraints through the rows' coefficients `$R$` in an orthonormal basis, solved as `$R^{\mathsf T}Ry=-R^{\mathsf T}c$`. Two arcs of a trimmed sketch offset, held to one crossing by a tangency each, made `$R$` ill-conditioned; `$R^{\mathsf T}R$` squared it; and the sharpened `$y$` met none of the rows. The solve stopped flat at its first step with the dimension's residual untouched, though the configuration was proven.

The cure is `least_squares` in `core/geop-core-math/src/solvers/least_squares.rs`, by **Householder reflections**. A reflection `$H=I-2vv^{\mathsf T}/(v^{\mathsf T}v)$` is orthogonal, so it does not change any length and it does not amplify errors. Choose `$v=x-\alpha e_1$` with `$\lvert\alpha\rvert=\lvert x\rvert$` and `$H$` maps the column `$x$` onto `$\alpha e_1$`, zeroing everything below the diagonal. The sign of `$\alpha$` is a free choice, opposite to `$x_0$`, so that `$v_0=x_0-\alpha$` adds rather than cancels. Do this column by column and `$A$` becomes triangular; back-substitute. A column with nothing left of it is left out, its `$y$` zero. Nothing forms `$A^{\mathsf T}A$`.

## The kernel's step: constraints exact, preferences second

A sketch solve is more than Gauss-Newton on the constraints, because a drag wants something too. Suppose the designer is dragging a point of a rectangle. One could add "point = pointer" as another row of `$c$` with a big weight. That is a penalty method, and the kernel refuses it, for a reason stated at the top of `core/geop-core-math/src/solvers/least_squares.rs`: such a sum could buy itself a little lower with a little violation of `$c$`, and wherever a long lever multiplies that violation, that is a lot of motion.

So two kinds of term are kept apart: **constraints** `$c(x)=0$`, held exactly; and **preferences** `$r(x)$`, minimized among the points where the constraints hold. The solver, `least_squares::minimize`, is a constrained Levenberg-Marquardt: each step solves

`$$\min_\delta\ \lvert r+J_r\delta\rvert^2+\delta^{\mathsf T}(B+\lambda D)\delta\quad\text{subject to}\quad c+J_c\delta=0$$`

by the *null-space method*. Build an orthonormal basis `$Q$` of the constraint rows, and `$Z$` of the directions they leave free. Then

1. the shortest step meeting the linearized constraints is `$\delta_c=Qy$` with `$Ry=-c$`, the coefficients `$R$` of the rows in `$Q$`;
2. the remainder of the step lies along `$Z$`, and serves the preferences: it minimizes the model there from the curvature reduced to those directions, `$Z^{\mathsf T}HZ$`.

Here `$D$` is Marquardt's scaling, the diagonal of the Jacobians' normal matrix, and `$\lambda$` shrinks after every step taken and grows after every one turned down: Gauss-Newton near a solution, gradient descent far away. `$B$` estimates the curvature of the constraints as seen by the pull, which Gauss-Newton leaves out. The demo above runs steps 1 and 2 with `$B=0,\ \lambda=0$`.

A step is taken only if it *definitely* lowers either the sum or the constraints' violation, as interval comparisons, against the point it starts from and every point taken before (a filter). So the two cannot trade back and forth, and no merit function weighs one against the other, because such a weight would need to exceed multipliers that grow without bound at a singular configuration. And the minimizer needs no tolerance: it stops where no step can definitely improve on where it is. The rule from [Lecture 5](./05-honest-numerics.md) is in full force here. The step itself is a free choice, so it is sharpened (`choose`); whether to take the step is an answer, so it is not. `choose` adds a small subtlety: a slope that *could be* zero is taken as zero, not as its midpoint, since a tiny nonzero slope would couple variables that do not depend on each other and let a weakly held one wander.

What about the variables nothing pulls? They are held in place by a damping preference with a small weight, `DAMPING` (`$1:10\,000$` to a pull). That is what "changing the sketch as little as the constraints allow" means numerically. A `Variable` says how to stay near where it was (`Variable::stay`).

If the constraints cannot all hold, `core/geop-core-math/src/solvers/system/driver.rs` runs a second minimization which takes them into the sum too. The sketch ends at the nearest configuration in the least-squares sense, updated anyway, and the report says which constraints failed. One minimization is bounded by `MAX_ITERATIONS` (200): how hard it tries, never what an answer means.

## Dragging

`Sketch::solve_with_drag` does this through `Variables::dragged` in `core/geop-core-sketch/src/variables.rs`: each dragged coordinate becomes a pull towards its target. In the demo, with the three default constraints ticked, drag `P1` around. It slides along the horizontal line it must stay on and never leaves it, tracking the pointer as far as it can. The pointer is not a constraint; if it were, then a pointer outside what the sketch can reach would be a conflict.

The solved positions are then written back *sharpened*: where a solve puts the points is the sketch's state and a free choice. They are what is drawn and what a program saves. The exact geometry is the enclosure of [Lecture 24](./24-rank-and-proof.md).

## Three stories about digits

### Which pivot?

Elimination (`eliminate` in `core/geop-core-math/src/solvers/linear.rs`) decides which constraints are independent and which variables they determine. Its first version went column by column, taking the first column with an entry that cannot be zero. Rounding leaves entries of `$10^{-16}$` that, as intervals, cannot be zero. One of them became a pivot and determined a coordinate the constraints barely touch. That left a half circle tangent to two parallel sides with its sweep free and two tangencies over the same coordinate, which could not be enclosed. Nothing was wrong with the constraints. **Complete pivoting**, taking the largest entry over all remaining rows and columns, fixed it.

As AGENTS.md puts it, "any value that cannot be zero" makes a pivot *valid*, not *good*. The same holds for every free choice: validity is the minimum; choose the well-conditioned one.

### The norm

Here is a line whose innocent look hides a trap:

`$$\lvert v\rvert=\sqrt{\textstyle\sum v_i^2}.$$`

The square of a small number keeps half its digits. In fixed-point arithmetic with a resolution of `$2^{-32}$`, a residual of `$10^{-6}$` squares to `$10^{-12}$`, below that resolution. The norm came out `$[0,\,4.6\times10^{-5}]$`. The minimizer judges a step by whether the violation's norm *definitely* drops, and no step can definitely lower an interval reaching to zero, so a body pinned half a turn away stopped `$10^{-6}$` off, ten times the tolerance, "flat" (AGENTS.md, *A norm squares away half the digits*). In `f64` the same loss hides below `$10^{-150}$`.

The fix is in `norm` in `core/geop-core-math/src/solvers/least_squares.rs`: scale by the largest entry, a free choice and sharp, before squaring: `$\lvert v\rvert=m\sqrt{\sum (v_i/m)^2}$` with `$m=\max\lvert v_i\rvert$`.

### Where a formula will do, do not iterate

The standard parts are programs whose sketches must solve at every size. AGENTS.md notes that a sketch whose arcs' sweeps must be solved for, with points far from where they were drawn, can be sent off to infinity. A comment in `lib/geop-lib-stdlib/src/drive.rs` records the case: with arcs, a gear far from the one drawn did exactly that. Newton's basin simply does not reach that far.

The cure, in `lib/geop-lib-stdlib/src/drawing.rs`, is `Drawing`: a sketch in which every point is placed by two formulas of the parameters, its `$x$` and `$y$` from a fixed origin, so that "solving the sketch is solving a linear system with one answer". The solver places it in one step for any size. When you control the sketch, write the sketch that is easy to solve.

<div class="kernel">

**In the kernel.** Reading order: `Sketch::solve` and `Sketch::solve_with_drag` in `core/geop-core-sketch/src/solve.rs` set the problem up. `Planar` (an alias of `Variables` in `core/geop-core-sketch/src/variables.rs`) makes it a `System`. `System::solve` in `core/geop-core-math/src/solvers/system/mod.rs` splits it into independent groups and drives `least_squares::minimize` on each. The same `System` also solves assemblies ([Lecture 37](./37-assembly-solver.md)): a sketch is only one implementation of `Equations`.

`SolveReport` says `converged`, `max_residual`, `iterations`, the minimizations as `phases`, `dof`, `free_points`, `free_curves`, `failed_constraints`, and `unproven`, why no true solution was proven. The editor's dialog reports how many degrees of freedom are left or how many constraints conflict.

Not implemented: the sketch solver does not search globally. It finds the solution the solve reaches from where the sketch is. A sketch with a second solution (a triangle flipped across a side) stays on the branch it was drawn in.

</div>

## Recap

<div class="recap">

- Connectivity is shared point ids, and coincidence removes variables: the cheapest constraint is the one that is not an equation.
- Every constraint is a residual measured as a length, differentiated exactly by dual numbers.
- Freedom is `$n-\operatorname{rank}J$`. Extra rows are redundant if they can vanish, conflicting if they cannot.
- Constraints are held exactly, and preferences (a drag, staying put) choose among where they hold, by the null-space method.
- Solve least squares with orthogonal transformations, pivot completely, and scale a norm before squaring it.

</div>

## Exercises

**1.** A triangle has three points with shared corners. Count variables. Which set of constraints makes it fully constrained, using `Fix`, `Horizontal` and `Distance`? How many `Distance` constraints are needed?

<details class="solution"><summary>Solution</summary>

Six variables. `Fix` on one point takes 2 and `Horizontal` on one side leaving it takes 1. With position and direction gone, 3 numbers remain, the triangle's shape: three `Distance` constraints give `$2+1+3=6$`. Note that the triangle may still flip across its base, or the base point to the left or right: fully constrained means locally unique, not globally.

</details>

**2.** Two lines are `Parallel`, and each is `Horizontal`. Show that at a configuration where both are horizontal, of lengths `$a$` and `$b$` and running in the `$+x$` direction, the three rows have rank 2.

<details class="solution"><summary>Solution</summary>

Let the lines be `$(p,q)$` and `$(r,s)$`. For `$u=d/\lvert d\rvert$` at `$d=(a,0)$`, `$du_y=d(d_y)/a$` and `$du_x=0$`. Differentiating `$u_1\times u_2=u_{1x}u_{2y}-u_{1y}u_{2x}$` at `$u_1=u_2=(1,0)$` gives `$du_{2y}-du_{1y}=d(s_y-r_y)/b-d(q_y-p_y)/a$`. So the `Parallel` row, with its factor `$\ell$`, is `$\ell/b$` times the second `Horizontal` row minus `$\ell/a$` times the first: a combination. Rank 2 of 3 rows.

</details>

**3.** Show that the solution of `$\min\lvert\delta\rvert$` subject to `$J\delta=-c$` has no component in the null space of `$J$`, and that a variable that appears in no residual therefore does not move.

<details class="solution"><summary>Hint</summary>

Write `$\delta=\delta_r+\delta_n$` with `$\delta_r$` in the row space and `$\delta_n$` in the null space. `$J\delta=J\delta_r$`, and `$\lvert\delta\rvert^2=\lvert\delta_r\rvert^2+\lvert\delta_n\rvert^2$`. So `$\delta_n=0$`. A variable in no residual has a zero column in `$J$`, so `$e_i$` is in the null space, and the row space is orthogonal to it, so `$\delta_i=0$`.

</details>

**4.** Compute by hand a Householder reflection that maps `$x=(3,4)$` to a multiple of `$e_1$`. Which `$\alpha$` do you choose and why? What is `$v$`? Check that `$Hx=\alpha e_1$`.

<details class="solution"><summary>Solution</summary>

`$\lvert x\rvert=5$`. Pick `$\alpha=-5$`, opposite in sign to `$x_0=3$`, so that `$v=x-\alpha e_1=(8,4)$` involves no cancellation. `$v^{\mathsf T}v=80$`, `$v^{\mathsf T}x=40$`, so `$Hx=x-2\cdot\frac{40}{80}v=x-v=(-5,-0)$`, that is `$(-5,0)$`.

</details>

**5.** In the demo, tick only `P0 fixed`, `bottom horizontal` and `diagonal = 2`. How many rows, and what is the freedom? Now add `right vertical`, `width = 3.5` and `height = 2`, and read the residual. Why is the largest residual smaller than the diagonal's excess of about 2.03?

<details class="solution"><summary>Solution</summary>

Rows 4, rank 4, and `$11-4=7$` free, counting the 8 corner coordinates and the circle's 3. With a width of 3.5 and a height of 2, the true diagonal is `$\sqrt{3.5^2+2^2}\approx4.03$`, so 2 is off by about 2.03. Least squares distributes that excess over the conflicting rows: width, height and diagonal each take part of it, so none stays at the full excess. That is what "as near as they come" means, and also why a conflict should be reported rather than quietly accepted.

</details>

**6.** Why would a penalty `$w\lvert c\rvert^2+\lvert r\rvert^2$` with a large `$w$` handle a drag poorly at a long lever: a point dragged at the end of a long link hinged at its other end? Think about what a small violation of the hinge costs versus what it buys.

<details class="solution"><summary>Hint</summary>

A violation `$\varepsilon$` of the hinge costs `$w\varepsilon^2$`, a second-order amount, while it reduces the pull's term to first order in `$\varepsilon$`. So the minimum always has some violation, of order `$1/w$`, and raising `$w$` until it is negligible makes the matrix so ill-conditioned the digits are gone. The null-space method never trades the two.

</details>
