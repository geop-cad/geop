# geop-cad-base

> Brief overview only — full documentation is coming later.

The CAD engine: which operations the editor offers, editing a program of
them, and example programs.

## The editor's operations

`PartOperation` is the set of operations the editor offers (see
[operation sets](../ops/geop-ops.md#operation-sets)): `AddSketch`,
`Extrude`, `Revolve`, `Sweep`, `Loft`, `Boolean` and `AddDatum`, each defined
by the crate that builds it. `Program`, `ProgramRunner` and `Step` here are the program
types of [geop-ops](../ops/geop-ops.md#programs) over that set — what the
web editor edits and the command-line tool compiles.

## The editor

`Editor` is the whole of editing a program, as one state machine: it holds
the program, its undo and redo, how far it runs (the marker), and the step
being edited — a `StepEditor` of [geop-ops](../ops/geop-ops.md#operations)
and the part before the step, as drawn, for picking. An editor drives it
with `Command`s and draws the `Update` each answers, so it keeps no state of
its own beyond the camera:

- **Program commands:** `new` (a step of an operation, inserted where the
  program runs to), `open` (a step, to edit), `remove`, `move`, `seek`,
  `load`, `load_example`, `undo`, `redo`. Each is refused, changing
  nothing, while a step is being edited, or when it would leave the program
  invalid; an update says why.
- **Step commands:** `event` (what the user did to the step), `preview`
  (show the part the step builds, or the part before it), `commit` (put the
  step into the program — only if it builds) and `cancel`.

An update holds the program as a list of steps shows it — each step's
fields in one line and whether it failed — the part to draw, and the step
being edited with its presentation and whether it builds. What did not
change since the last update is left out: a hover sends only the step, and
the part is sent again only once something ran. The steps are run by one
`ProgramRunner` — up to and including the step being edited while there is
one, else as far as the program runs — which keeps the part after every
step, so the part before the step being edited is always at hand, and a
change to it replays only that step.

What the shown steps built on — their reference fields' values — is hidden
from the drawing: sketches and whole datums, since what was made from them
shows them now; none that something of could be picked while a field waits
for a pick. Every scene lists the part's structure beyond its faces — its
solids, sketches, datums, placed parts and their mates — each shown or not;
`Command::Visibility` shows or hides one by name over what the editor would
by itself, for as long as the same file is edited.

The program's parameters are edited as a whole (`Command::Parameters`),
also while a step is edited: everything reading one is built again — the
step being edited included, a sketch solved anew with its formulas'
values.

## Examples

`examples` holds programs written in Rust (`box_with_drill_hole`,
`bracket`, `cross_drilled_shaft`, `two_plates`, `boss_on_reference_plane`,
`handle_with_hole`, `luggage_tag`). The tests use them, and they serve as a
reference for writing new programs.
