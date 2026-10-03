# geop-core-solve

> Brief overview only — full documentation is coming later.

Solving constraints: the one engine behind sketches, assemblies and the
parameters of programs.

## Systems

A `System` has **parameters** (`Param`): numbers, and poses of rigid
bodies — a unit dual quaternion and the point the body turns about. It has
**residuals** (`Residual`): functions of a few parameters that are zero
exactly when what they stand for holds. Solving moves the free parameters
until every residual vanishes, changing them as little as it can.

- **Residuals are lengths**, so no kind dominates another by its units,
  and one relative tolerance decides when a residual holds.
- **Residuals are constraints.** A solve minimizes its preferences (pulls,
  and staying put) among the configurations where every residual holds,
  by constrained Levenberg–Marquardt (`geop_core_math::least_squares`).
  The residuals hold exactly: no preference can trade a little violation
  of them for itself — which, behind a long lever, is a lot of motion.
- **Honest enclosures.** Every residual is computed as a `Dual` over
  `ScalInF64`, so its gradient is exact and its `sqrt`/`sin`/`PI` are
  enclosed; the minimizer takes a step only where the merit definitely
  drops, and stops where no step can tell.
- **Increments.** The variables are increments from where the parameters
  are when a solve starts. A pose's are a translation and a turn about the
  body's center (`Placed`: `T(c + dt) R(w) T(-c) q0`, with `R(w)` the
  quaternion `(1, w/2)` normalized). Turns are measured as lengths at the
  system's size, so a step is as long for a turn as for a move. Poses have
  no singular configurations.
- **Pulls** (`Pull`) are preferences: a number towards a value, a body
  towards a pose, a point of a body towards a point — a drag. Ranked below
  them, a dragged body would rather not turn, and below that everything
  else would rather stay where it is: from far off, a solve moves bodies as
  little as it can rather than turning them over.

`System::report` says which residuals hold, `free_variables` how many
degrees of freedom are left and which variables can still move, and
`enclose` proves an interval enclosure of the exact solution of a system
of numbers (Krawczyk).

## Mates

The `mates` module builds systems of rigid bodies: an `Assembly` of
`Body`s and `Constraint`s, each a `Kind` — `Coincident`, `Concentric`,
`Parallel`, `Perpendicular`, `Distance`, `Angle` — between two `Feature`s:
a point, line or plane attached to a body, or to the ground.
`geop-core-sketch` builds a sketch's system the same way, its constraints
residuals of its points' coordinates.
