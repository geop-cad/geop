# geop-cad-base

> Brief overview only — full documentation is coming later.

The CAD engine: which operations the editor offers and the programs written
in them, the part as it composes it from every crate, the workspace its
programs place parts from, editing a program — one step and the whole —
example programs, and the standard parts.

## The editor's operations

`operations::editors` is the set of operations the editor offers (see
[operation sets](../core/geop-core-editor.md#operation-sets)), made by the
`operations!` list: sketches, datums, extrude, revolve, sweep and loft,
holes and threads, fillets and chamfers, shells and drafts, booleans and
splits, patterns, mirrors and moves of bodies, surfaces, subdivision
surfaces, sheet metal, harnesses, STEP import, drawings and placed parts,
each defined by the crate that builds it. `infos` lists them for a front
end's toolbar, and `registry` is the registry its programs run with.
`Program` is the program file of
[geop-ops-parameters](../ops/geop-ops-parameters.md#the-program-file-programfile),
and `Runner` the runner of its steps — what the web editor edits and the
command-line tool compiles. `operations::build(program, file, workspace)`
builds a program outside the editor, as the file `file` of the workspace
it places parts from (`operations::workspace`, beside the standard parts).

`outputs` is, the same way, the set of exporters and inspectors the editor
offers (see [geop-ops-output](../ops/geop-ops-output.md)): STL and STEP
files, drawings, URDF robots, flat patterns and bills of materials;
measuring, mass properties, interference and the bill of materials as a
table.

These lists, `part`, the examples and the standard parts are the only
places this crate names an operation crate; the editor itself knows none.

## The part, composed

The `part` module. An operation crate plugs into the editor without the editor knowing it: it
defines its own targets and draws them, and `part` composes the part from
every crate's:

- `draw(reader)`: the part as the viewport draws it, a
  [`PartView`](../ops/geop-ops-view.md) — its model, its datums, its
  sketches, what its steps draw on it and the parts placed in it, in its
  colour (`geop_ops_assembly::draw`, which draws placed parts the same
  way).
- `structure(reader, hidden)`: what the part has beyond the faces of its
  solids — datums, sketches, solids, faces standing on their own, placed
  parts, and whatever its steps list, such as mates — each a
  `StructureItem`, shown or not.

## The workspace

The `workspace` module. `Workspace` is the
[workspace](../core/geop-core-program.md#the-workspace) of a set of program
files: the parts they build, by path, and the other files their steps
read. Files name each other by `/`-separated paths relative to the file
doing the naming — `../parts/bolt.geop` — so a folder of files can be
moved as a whole (`resolve`, `relative`). Each file is read once, until it
changes: a program is the same `Rc` while its file does not change, so a
part placed from it again carries over what it built. Every part placed is
a part of its own, kept by the part placing it (sharing the parts that
read no state between their instances is a TODO). A write changes its
revision, and what read its files is read again. The runner refuses a
placement that would make files place each other in a cycle, naming the
cycle: the files must form a DAG. Where the files come from is a
`FileStore`; `stdlib::WithStandardParts` adds the standard parts beside a
workspace's own files, as read-only files named `std:…`
(`std:iso4032_hex_nut.geop`), which mean the same file from anywhere.

## The editor

`Editor` is the whole of editing a program, as one state machine: it holds
the program, its undo and redo, how far it runs (the marker), and the step
being edited — a `StepEditor` (below) and the part before the step, as
drawn, for picking. An editor drives it with `Command`s and draws the
`Update` each answers, so it keeps no state of its own beyond the camera.
The program is the one of a file, among others it may place parts from:
the editor is told the file's path with the program (`Command::Load`), and
is sent the other files' programs whenever they change (`Command::Files`)
by the front end, which owns the files.

- **Program commands:** `new` (a step of an operation, inserted where the
  program runs to), `open` (a step, to edit), `remove`, `move`, `copy`,
  `paste`, `seek`, `load`, `load_example`, `undo`, `redo`. Each is refused,
  changing nothing, while a step is being edited, or when it would leave
  the program invalid; an update says why.
- **Step commands:** `event` (what the user did to the step), `preview`
  (show the part the step builds, or the part before it), `commit` (put the
  step into the program — only if it builds) and `cancel`.
- **The state:** `drag_tool` (with no step edited, dragging any placed part
  moves it as far as the mates let it) and `joint` (set a joint's
  coordinate; the parts move to it), both solved by
  `geop_ops_assembly::solving`.
- **Output commands**, which change nothing: `export` (write the part with
  an exporter, by its kind, in a format, told its own options: the update's
  `export`), `inspect` (ask an inspector about the part: the update's
  `inspection`, `{kind, report, shown}`: the answer as data, and as the
  inspector lays it out) and `pick_tool` (take an inspector that picks in
  hand: clicks pick what it takes, and every update carries its answer
  about the picks and what it draws). The program state lists the
  `exporters` and `inspectors` offered, so a front end's menus are made
  from them.

An update holds the program as a list of steps shows it — each step's
fields in one line and whether it failed — the part to draw, and the step
being edited with its presentation and whether it builds. What did not
change since the last update is left out: a hover sends only the step, and
the part is sent again only once something ran. The steps are run by one
`Runner` — up to and including the step being edited while there is one,
else as far as the program runs — which keeps the part after every step,
so the part before the step being edited is always at hand, and a change
to it replays only that step.

What the shown steps built on — their reference fields' values — is hidden
from the drawing: sketches and whole datums, since what was made from them
shows them now; none that something of could be picked while a field waits
for a pick. Every scene lists the part's structure (`part::structure`);
`Command::Visibility` shows or hides one entry by name over what the editor
would by itself, for as long as the same file is edited.

The program's parameters are edited as a whole (`Command::Parameters`),
also while a step is edited: everything reading one is built again — the
step being edited included, a sketch solved anew with its formulas'
values. `Command::RenameParameter` renames one and every formula reading
it.

## Editing one step

How a field is edited is the same for every operation, so it is not the
operations' but the `StepEditor`'s (`step_editor`), which edits one step.
It turns what the user did — a `StepEditEvent` (see
[geop-core-editor](../core/geop-core-editor.md#what-is-shown-and-what-reaches-it))
— into what the operation understands, an `Event`:

- **Reference fields** hold `Selectable` values: elements of the part of
  the kinds the field takes. A field arms on a press; the editor then finds
  what a click would pick on hover, lights it, and picks it on a click —
  once for a field of one element, again and again (each pick adding one
  or taking it out) for a field of several. Taking one out, or all, is the
  editor's too, as is saying what each element held is used as, or why it
  cannot be. The operation is only ever sent what the field holds now. A
  new step whose first reference field is still empty starts with it
  armed: what it is built on is what it needs first. One that already holds
  something — an extrude's newest sketch — waits for no click, so its
  handles can be dragged straight away. So does a field that appears empty
  as the step is edited — a mate just added waits for its entities — and a
  field that goes away stops waiting. While a field waits for a pick, the
  part is shown as it stands, not the plane the step works in. Picks test
  against the part before the step, or — for an operation whose references
  point into what it adds itself (`OperationInfo::picks_built`), a placed
  part's mates — against what it builds.
- **Handles** of number fields are drawn by the editor and dragged along
  their direction, the value following how far the pointer has moved along
  the handle's track, from the ray where the drag started to the ray where
  it is now, snapped to a hundredth (`DRAG_SNAP`).
- **Selecting** is the editor's: a click on a selectable visual selects it
  or takes it out again, a click on nothing clears the selection unless
  shift is held, and Escape clears it. The operation reads the selection in
  its form and may change it as its fields are used. The editor draws what
  is selected and what the pointer is over.
- **Dragging** a draggable visual is the editor's too: it is grabbed where
  the drag starts, and the operation is sent where to, in the plane worked
  in — or, with none, in the plane through where it was grabbed, facing the
  eye. With a tool in hand that strokes, a drag from anywhere else is the
  tool's.
- **The gizmo**, if the form has one, is drawn over everything and grabbed
  first; a drag of one of its parts is followed, snapped and sent to the
  step as it was when it was pressed, so each event applies the whole drag
  once.
- **Keys:** Escape, Enter, Delete and a secondary click mean the same in
  every tool (`CanvasEvent::Cancel`, `Confirm`, `Delete`); any other key is
  a tool's shortcut.

Whatever else the pointer and the keys do goes to the operation as a
`CanvasEvent`: clicks while it has a tool in hand, and those on nothing,
hovers, keys. Hit tests against visuals (`hit::hit_visuals`) and against
the part as drawn (`PartView::pick`, through the context's `Scene`) decide
what a pointer is over, so a pick cannot disagree with what is on screen.

## Examples and standard parts

`examples` holds programs written in Rust (`box_with_drill_hole`,
`bracket`, `cross_drilled_shaft`, `split_plate`, `handle_with_hole`,
`luggage_tag`, `patterned_plate`, `hole_plate`, ...) and workspaces of
several files (`workspaces`). The tests use them, and they serve as a
reference for writing new programs.

`stdlib` holds the standard parts — ISO screws, nuts, washers, dowel pins,
standoffs, ball bearings, T-slot extrusions, a NEMA 17 motor, gears and
their rack, pulleys, shaft collars, couplings, linear guides, hobby
servos — one program per family, generated in Rust, its sizes the rows of
a table parameter `size`. Every part turns around, or runs along, the `z`
axis, and carries named datums to mate it by (`axis`, and a plane for the
face it sits on). Listing them builds nothing: a family's program is
generated the first time its file is read.
