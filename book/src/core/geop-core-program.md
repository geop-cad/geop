# geop-core-program

The program machinery every part is built by, independent of what is built:
a program is a list of steps, each an `Operation`; running it builds a
`Part`, target by target, reusing every version whose arguments and reads
did not change. The values a run is given are its inputs (parameters) and
its **state** — where its placed parts are, how its joints are turned —
which no step changes: a solver moves the state until the constraints the
steps define hold.

- `operation`: `Operation` — a step's arguments, how it runs, and what its
  editor shows (`Form`) and does with what the user did (`Event`).
- `part`: `Part`, its `View`s and the `Reader` a target's generator reads
  through, recording what it read. `State` is a value the state holds: a
  block of variables of the constraint systems of
  [geop-core-math](./geop-core-math.md) (`Variable`), stepped along its own
  tangent space — a number by adding to it, a pose by the exponential map.
- `constraint`: `Constraint` — residuals a step requires to vanish (or, for
  a limit, to stay at most zero), each a length, and their derivatives by
  the steps of the states they read.
- `program_runner`, `step_editor`, `editing`: running a program and editing
  its steps.

## Solving the state (`solve`)

A program's state is solved by the constraint systems of
`geop_core_math::solvers::system`, which hold every algorithm: this module
is only the adapter. Each value of the state a solve may move
(`Problem::free`) is a block of the system; each constraint the last run
defined is a term, brought up to date for every state tried
(`Part::update`) rather than the program run again; `Preference`s — a
drag's pulls — are the system's preferences.

- `solve`: where the free values have to be for every constraint to hold,
  as near where they were as the constraints allow and pulled as the
  preferences ask — group by group, limits kept by an active set. Its
  `Solved` carries the state, a `Report` (what holds, the limits stopped
  at, what moved, the minimizations, and how many times the constraints
  were made to solve and to enclose), and an `Enclosure` of a true
  solution near the state found for every group it solved: each value's
  box, enclosed in its own step coordinates around the solved point — or,
  for a group none is proven for, why: the constraints that conflict, by
  name, or what is singular. `geop_ops::solving`'s report passes both on
  (`enclosed`, `unproven`), as a sketch's does its `unproven`.
- `enclose`: the same enclosure of a state given.
- `freedom`: how many independent ways each value can still move, decided
  over the box the enclosure proved, not at the point — a redundant
  constraint takes no freedom away.
- `conflicting`: a smallest set of constraints that cannot all hold
  together, by name.

What a solve may move is declared in the program itself: a step defines an
`Unknown` (a target, named by `unknown`) for a value of the state it lets a
solve move — how, and where a solve starts it from where the state has
none, or where a given constraint does not hold — or that it holds fixed.
`unknowns` gathers them from a part, so a solve of any program is set up
from the part alone; `geop_ops::solving` is the editor's side of it.
