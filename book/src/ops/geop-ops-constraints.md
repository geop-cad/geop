# geop-ops-constraints

> Brief overview only — full documentation is coming later.

Constraints on a part's state, and the solver that moves the state until
they hold. Nothing here knows what the constraints are: mates and joints
are [geop-ops-assembly](./geop-ops-assembly.md)'s, which defines them
through this crate.

## What the steps declare

A program's steps say what they require of the state and which values of
it a solve may move. Each of the two is one target of the part, built up by
the steps one revision after another, so a solve of any program is set up
from the part alone:

- **Constraints** (`Constraint`, gathered in the target `constraints()`, a
  `Constraints`): residuals a step requires to vanish — or, for a limit
  (`Bound::AtMost`), to stay at most zero — each a length, and their
  derivatives by the steps of the states they read. A step defines its
  constraints with `define_constraints` in two targets: its own, made by its
  generator and rebuilt only when what that read changes, and its revision
  of `Constraints`, which adds them to those of the steps before. A solve
  runs the program at every state it tries; the split keeps a step's
  generator from running again only because a constraint of an earlier
  step moved.
- **Unknowns** (`unknown::Unknown`, gathered in `unknown::unknowns()`):
  a value of the state a step lets a solve move — how (`Mobility`), where a
  solve starts it from where the state has none (`seed`), or where a given
  constraint does not hold (`unless`: a joint's coordinate, measured from
  where its parts are unless the joint holds) — or that it holds fixed.
  Declared with `unknown::declare_unknowns`.

## Solving (`solve`)

A program's state is solved by the constraint systems of
`geop_core_math::solvers::system`, which hold every algorithm — the same a
sketch's numbers are solved by: this module is only the adapter. Each value
of the state a solve may move (`Problem::free`) is a block of the system,
stepped along its own tangent space (a pose by the exponential map of a
twist); each constraint the program defines is a term, made again for every
state tried by running the program again (`Run`) — what read no state that
changed is reused, not built again; `Preference`s — a drag's pulls — are
the system's preferences, and every value nothing pulls stays as near where
it was as the constraints let it.

- `solve`: where the free values have to be for every constraint to hold,
  group by group, limits kept by an active set. Its `Solved` carries the
  state, a `Report` (what holds, the limits stopped at, what moved, the
  minimizations, and how many times the constraints were made to solve and
  to enclose), and an `Enclosure` of a true solution near the state found
  for every group it solved: each value's box, enclosed in its own step
  coordinates around the solved point — or, for a group none is proven
  for, why: the constraints that conflict, by name, or what is singular.
  The state is moved even where the constraints cannot all hold, to where
  they come closest.
- `enclose`: the same enclosure of a state given.
- `freedom`: how many independent ways each value can still move, decided
  over the box the enclosure proved, not at the point — a redundant
  constraint takes no freedom away.
- `conflicting`: a smallest set of constraints that cannot all hold
  together, by name.

What an editor does with these, the same for every program — dragging,
setting a joint, listing what can be set — is `geop_ops_assembly::solving`
(see [geop-ops-assembly](./geop-ops-assembly.md#solving-the-state)).
