# geop-core-editor

What editing is. The vocabulary a step's editing is said in — dialogs,
visuals, gizmos, events, picks (`editing`) — numbers typed as formulas
(`EditForm::numeric`), what an operation shows and does with them
(`editable`, `part_operation`), how a reference field picks an element
(`selectable`), and the operations an editor offers (`Editors`). What an
editor draws of a part and picks in it is the `view`, and what is made of a
part rather than in it — files and answers — the `output`. The interaction
every operation shares, driving these, is [one step edited](#editing-one-step)
(`step_editor`), and the whole of editing a program is
[the editor](#the-editor) (`editor`), over what a
[registry](#the-registry-registry) offers: it names no operation, so an
operation crate can drive an editor of three operations in its own tests.
A program runs without any of it (see
[geop-core-program](./geop-core-program.md)).

## An operation (`PartOperation`)

Every operation of a part implements `PartOperation` and is made an
`Operation` and an `Editable` by `part_operation!(MyOp(MyOpArgs))`:

- `KIND`, the name its steps are stored under, and `INFO`, an
  `OperationInfo` — its label, a sentence of what it does, its
  `OperationGroup` (the toolbar's section) and `OperationTier` (how
  prominently it is offered there), its icon, the file extensions it reads
  beside programs (`reads`, a STEP file to import) and whether its picks
  are made in what it builds rather than the part before it
  (`picks_built`: a placed part's mates).
- `Args`, the arguments a step is given and saved with: numbers, choices,
  sketches, and references to elements by name.
- `new_args(context)`: the arguments of a new step, from what the part
  before it holds — an extrude takes the newest sketch.
- `run(step, runner, part, args)`: what it builds, and the runner running
  it, to run programs of its own. A step that builds into the
  part's model is a `ModelOperation` of
  [geop-ops-model](../ops/geop-ops-model.md) and runs `build_model`.
- `form(context, args, editing)`: the one form it is edited through, an
  `EditForm` (below). It never fails: an editor needs a dialog most exactly
  when the arguments do not build, so whatever is wrong is said in it.
- `Session`: what it keeps between events besides its arguments (`()` for
  most; the tool in hand and a half-drawn curve for a sketch), what is
  forgotten before each event (`begin_event`), and `canvas`, what it does
  with the pointer and keys beyond its fields — clicks while it has a tool
  in hand, and those on nothing, hovers, keys, moves of its draggable
  visuals.

Everything is edited against a `Context`: the step's id, the runner it is
run by (`runner`, which reads the files around its program — a picker of
files, a placed program's parameters), the part as it was
before the step (`before`, a `Reader`), the part as the step built it when
it last ran (`built`), and, for an event, the part as the editor draws it
(`scene`), so an operation picks in the editor's drawing rather than
drawing the part again for every move of the pointer.

## One form, with its setters

The `edit_form` module. An `EditForm` is a `Form` built together with what setting each of its
fields does. Each field is added once, with its setter — `number`,
`numeric`, `checkbox`, `select`, `actions`, `file`, `color`, and `pick` /
`pick_one` for reference fields — and an event is answered by the setter of
the field it sets (`EditForm::set`), never by matching keys. A setter gets
the whole `Edit`: the step's arguments and its session. It may capture what
the form was built from, for what setting one field implies for others:
another sketch picked brings its own axis, a selection picks the
construction it fits. Sub-forms shared by several operations add fields
with setters to an `EditForm` in the same way. An operation only ever
hands out the form (`EditForm::into_form`); the setters stay its own,
built again for each event.

## What is shown and what reaches it

The `editing` module.

- **A `Form`** is plain data: a `Dialog` — an ordered list of keyed
  fields that say what their values *mean*: headings and texts, actions
  (things to do or choose, grouped, one perhaps active, a disabled one
  saying why), checkboxes, numbers (with a `Unit`, a slider range and
  perhaps a handle to drag them by, a `Track`), selects, lists, references
  to elements of the part, a file to download (`Dialog::download`) — and
  `Visual`s to draw in the viewport: points, polylines, filled areas and
  labels, each perhaps selectable or draggable. Besides, a plane to work
  in (`focus`), a sheet of its own (a drawing's paper), what it has in
  hand (`InHand`), the visuals selected, a value asked for in place
  (`Prompt`), a `Gizmo`, and `pulls`: where it wants the state pulled
  (`Pull::Point` a drag, `Pull::Pose` a placement typed in, `Pull::Value`
  a joint's coordinate set). An operation never sets the state itself;
  the editor solves the program with its pulls, and the constraints have
  the last word.
- **An `Event`** is what reaches an operation once the editor has done
  what it does alike for every operation: a field set
  (`Event::Field`), the selection now (`Event::Select`), or what the
  pointer or a key did beyond that (`Event::Canvas`, a `CanvasEvent`).
  How a field was set — typed, picked for in the viewport, dragged by its
  handle — the operation never knows.
- **A `StepEditEvent`** is what the user did, as a front end sends it: a
  dialog field used, a hover, a click or a drag in the viewport, each a
  `Pointer` — the `Ray` from the eye through the cursor and its `Reach`,
  how far from the ray counts as under it (a cone from the eye in
  perspective, a tube in an orthographic view) — or a key.
- **A `Presentation`** is what an editor shows of a step: the form, what
  the reference fields hold and what a click would pick lit, what is
  selected and hovered, the handles, and what a click picks now (`grab`).
- **A `Gizmo`** (`editing::gizmo`) is the one way every operation offers a
  drag in three dimensions: arrows, planes and rings to move along, in and
  about its axes, and cubes to scale, laid out in the pointer's reaches so
  it keeps its size on screen. The editor draws it, follows a drag and
  snaps it, and sends what the drag did as a `GizmoDrag`, measured from
  where the drag started.
- **Hit tests** (`editing::hit`): which visual a pointer is over
  (`hit_visuals`), and the `Scene` an editor picks elements in. Near means
  within the pointer's reach, so it means near on screen however far away
  or zoomed in the view is; the ray geometry itself is
  `geop_core_math::primitives::Ray`'s.

So an editor renders primitives and forwards raw input, and every decision
— what a click picks, what a line snaps to, what a drag changes — is made in
Rust, where it can be tested.

## Picking references

The `selectable` module. A reference field holds `Selectable` values, one per pick. An
`ElementReference<T, K>` is one when its kind is a `SelectableElement`:
the kind says what a click on what the editor draws selects
(`PICKED_FROM`, the drawn kinds it takes a click on, and `select`) and what
of it to light while it is held (`drawn`). What an operation asks for may
be something else drawn as those: a solid picked by a face of it, a tangent
chain by an edge of it, a line by a straight edge. The crate defining the
kind says how; the editor stays the same for all of them.

Where an operation takes the same thing from several sources — a line from
a straight edge, a datum axis or a sketch line — its argument is an enum
declared with `one_of!`, each variant an element reference, all resolving
to one value:

```rust,ignore
one_of! {
    /// What a revolve turns about.
    pub enum AxisRef -> Axis<Design> {
        Edge(ElementReference<NamedModel<Design>, Line>),
        Datum(ElementReference<Datums, Line>),
    }
}
// args.axis.resolve(&reader)?  ->  Axis<Design>
// saved as {"edge": "extrude(box,K,c3)"} or {"datum": "origin:z"}
```

A click is tried against the variants in order. `selected` gives the
values a field holds after the editor sent it its picks — a click on a
value held already takes it out again — and `selectable::pick` picks
directly on the canvas, for a step that picks without a reference field.

## Numbers as formulas

A `NumericInput` ([geop-core-program](./geop-core-program.md#numbers-as-formulas))
is a number an operation is given, typed as a plain value or as a formula
of the program's parameters. An operation's arguments take one wherever
they take a number, and its form shows it with `EditForm::numeric`: a
field of `Number::numeric`, showing what it comes to as the part before
the step reads the parameters — or, where it does not evaluate, why —
with no slider and no handle while it follows a formula. Text typed sets
it, plain if it is a number and a formula otherwise, and a number slid or
dragged sets it plain (`Value::numeric_input`).

## Operation sets

`Editors` is the operations an editor offers, by kind (`with`): what gives
each step its arguments, form and events, and — through
`Editors::operations` — the `OperationRegistry` its programs run with, so
the two cannot disagree. `infos` lists them in the order they were added,
each a `Listed` (kind and `OperationInfo`), which is what a front end
builds its toolbar from. An editor's are its
[registry's](#the-registry-registry).

## The view (`view`)

`PartView`: a part as the viewport draws it, and picking elements of it
with a `Pointer`.

### Drawing

A part is drawn from its rasterization — sampled edges, triangulated faces,
outlined sketches (see [geop-core-rasterize](./geop-core-rasterize.md)) —
and a pick tests against exactly that, never re-deriving geometry of its
own, so a pick cannot disagree with what is on screen. The same goes for
what the viewer draws besides the part: the datums are laid out here, by
the sizes the viewer draws them with (a frame's axes are `FRAME` reaches
long), and picked by the same rules. Serialized, a `PartView` is what a
viewer draws, with the targets its model, datums and sketches are of: a
viewer lights an element the editor names (a `PickedElement`) by the
target, kind and name it carries, as a pick names it.

Each crate draws its own targets into a view — the model its vertices,
edges and faces, the sketches theirs, the datums theirs, the placed parts
theirs — by a `Drawer` of its registry, and the registry composes them
(`Registry::draw`: every drawer in turn, then what the steps annotate),
then `PartView::finish` measures the result. What each drawer drew is kept
by what it read (`Drawings`, `Reader::recorded`): the part shown again
with other sketches — a sketch opened, or closed — draws its sketches
again, never its faces. A face's triangles are one `FaceMesh`, known by a
hash of what a viewer is sent of it.

The parts placed in a part are drawn by reference: each placed part —
however deep — is listed once, with where it is (`ViewInstance`), and drawn
from its component's own view, built once and shared by every instance of
it. Moving a placed part changes where it is, not what is drawn.

### Picking

`PartView::pick(pointer, accepts)` finds what the pointer is over among
what is drawn of the kinds a field accepts (`drawn`: `vertex`, `edge`,
`face`, `sketch`, `sketch_point`, `sketch_curve`, `datum`, `frame`). Near
means within the pointer's reach. Frame datums are drawn on top of
everything, so they are picked first; otherwise the smallest element
under the pointer and not hidden behind a face wins — a vertex or a sketch
point over an edge or a sketch curve — and else the nearest along the ray
of the faces, sketches and other datums hit. A pick names what it hits as the crate that drew it
named it, a `PickedElement`, which the crate defining the element turns
into a typed reference (see
[Picking references](#picking-references)).

A pick moves the pointer into each placed part's own frame and names what
it hits as that part names it, its target behind the instance's name
(`Located`: `/bolt:model`). `pick_instance` and `part_to_drag` find the
placed part under the pointer, for dragging it. A `frame` pick (a
`FramePick`) puts a frame on what is picked — a face, an edge, a point, a
datum — at the corner or side of a face, or the end of an edge, that the
pointer is near: what a joint is made between (see
[geop-ops-assembly](../ops/geop-ops-assembly.md)).

### What steps add to the drawing and the structure

An editor knows no operation by name. What a step adds besides its
topology, it adds as a target of its own, which the editor collects without
knowing whose it is:

- **Annotations** (`annotations(step)`, an `Annotations`): what a step
  draws on the part besides its topology — a cosmetic thread's helix and
  designation (`geop_ops_model::draw_threads` is the example).
  `annotations_of` collects every step's.
- **Structure entries** (`listing(step)`, a `Listing` of `Listed`): what a
  step lists in the part's structure tree beside what every part has
  (datums, sketches, solids, faces standing on their own, placed parts): a
  kind (`mate`, the tree's group), a name and the elements it refers to,
  lit while it is pointed at. `listed_by_steps` collects every step's;
  `add_part` lists its mates this way.

`icons` holds the icons operations share.

## Outputs (`output`)

What is made *of* a part rather than in it: files written from what a
program built (`Exporter`), and questions answered about it (`Inspector`).
Neither is a step: exporting and inspecting change no program and are
nothing to undo. A crate that writes a format, or answers a question,
implements one of the traits on a unit struct, as an operation does; an
editor offers those registered in its `Outputs`, by their infos, and knows
nothing more of them. The options each takes are its own, sent as JSON.

### What they are given (`Output`)

An `Output` is what a program built, as an exporter or an inspector is
given it: the runner holding the part as far as the program runs, the parts
it places included (`part()`), the part before a step (`part_before`, what a
drawing step is of), the program file it ran (`file_name`), with its state —
to ask how its parts can move by running it again — the step being edited
and the date. What the program says of its part (its title, designation,
colour and material) is read from the part like any element of it
(`geop_ops_model::parameters::read_properties`, see
[geop-ops-model](../ops/geop-ops-model.md#properties)), and a
file written from it is named after the program's file
(`geop_ops_model::parameters::stem(output.file_name())`).

### Exporters

An `Exporter` is a kind of file written from what a program built. Its
`INFO` (`ExporterInfo`) gives its kind, label, a few words of what the file
is for, and the formats it writes by extension, the default first; its own
`Options` are deserialized from the JSON a front end sends (`Default` where
it sends none; `NoOptions` for one that takes none); and
`export(output, format, options)` returns an `Export` — a name, a media
type, and text or bytes (`Content`). A step's dialog offers a file of its
own with `Dialog::download(key, exporter, formats)`.

The STL exporter (`geop_ops_assembly::Stl`) writes the mesh the editor
draws; others write STEP files, drawings, URDF robots, flat patterns and
bills of materials, each in its own crate.

### Inspectors

An `Inspector` is a question answered about what a program built. Its
`INFO` (`InspectorInfo`) says whether it is asked of the whole part or of
picks (`Picks`: the drawn kinds it accepts, at most how many);
`inspect(output, picked, options)` returns its `Report` (any `Serialize`)
and the visuals to draw of it in the viewport, and `show(report)` lays the
report out for a front end as an `Answer` — text, lists, tables, groups,
buttons that ask again or save a file (`Block`, `AnswerAction`), and a
plane to cut a section with — so a front end shows every inspector's answer
alike, knowing none. An inspector that picks becomes the editor's pick
tool, asked again with every pick. `Bounded` is a number as a report shows
it: the midpoint of an enclosure and its half-width.

### Registering them

Exporters and inspectors are registered in an `Outputs` (`exporter`,
`inspector`; a kind twice is refused), which a
[registry](#the-registry-registry) holds beside its operations.

## The registry (`registry`)

Everything an editor offers is a `Registry`'s, one builder every operation
crate adds to with its one `register(r: Registry) -> GeopResult<Registry>`;
an application calls each crate's in turn (the editor's is
[geop-cad-base](../cad/geop-cad-base.md)'s `registry`). Each way of adding
something is a trait:

- **Operations** (`operation`, an `Editable`): offered within their group
  and tier in the order they were added; `operations()` gives the runner
  the same set.
- **`Drawer`**: draws the targets a crate defines into the part's view
  (`draw`), lists them in the structure tree (`structure`, each a
  `StructureItem` with the `kind_label` a front end groups it under, shown
  unless hidden) and says what it draws only to be picked from (`hidden`:
  the datums of placed parts, shown while a part is placed). A part is
  drawn by every drawer in turn (`Registry::draw`) and listed likewise
  (`Registry::structure`, then what the steps list).
- **Exporters and inspectors** (`exporter`, `inspector`), as above.
- **`Panel`**: program-level state a front end shows beside the steps,
  under its `KEY` in `ProgramState::panels` (`state`), the commands it
  takes as `Command::Panel { key, payload }` (`command`, its own
  `Command` type), and what it asks once an edit (`after_edit`): the
  parameters, the joints of an assembly.
- **`Tool`**: a mode in hand while no step is edited, taken in hand by its
  key (`Command::Tool`), offered as its `ToolInfo` says
  (`ProgramState::tools`); the events of the viewport are its (`handle`),
  it shows what it would take (`presentation`), and may answer about the
  part with every update (`inspection`). The pick tool
  (`tools::pick::PickTool`) is the core's, one per inspector that picks,
  by the key `pick:<inspector>`.
- **`Settle`** (at most one): the solve the editor runs after every edit,
  moving the program's state so the constraints its steps declare hold
  (`settle`, given what the step edited pulls), and naming the state it
  solves for (`solvable`). With none, the state is kept as given.
- **`ExampleSet`**: programs and workspaces of several files
  `Command::LoadExample` and `LoadWorkspaceExample` load, listed by name
  without being made.

A panel, a tool or a settler sees the editor through an `EditorContext`
(the program, the part as it last ran, the part as drawn, running it all
again, `Rerun`, moving the state, editing the program with the marker and
the step edited kept at their steps, one undo for a whole drag), so undo
and the marker stay right whatever an extension does.

## The editor

`Editor` (`editor`) is the whole of editing a program, as one state
machine: it holds the program, its undo and redo, how far it runs (the
marker), and the step being edited — a `StepEditor` (below) and the part
before the step, as drawn, for picking. An editor drives it with
`Command`s and draws the `Update` each answers (`handle`, or `handle_json`
on the wire), so it keeps no state of its own beyond the camera. The
program is the one of a file, among others it may place parts from: the
editor is told the file's path with the program (`Command::Load`), and is
sent the other files' programs whenever they change (`Command::Files`) by
the front end, which owns the files. It is made of a registry and a
`FileStore` the files it is sent are kept in (`Editor::new`).

- **Program commands:** `new` (a step of an operation, inserted where the
  program runs to), `open` (a step, to edit), `remove`, `move`, `copy`,
  `paste`, `seek`, `load`, `load_example`, `undo`, `redo`. Each is refused,
  changing nothing, while a step is being edited, or when it would leave
  the program invalid; an update says why.
- **Step commands:** `event` (what the user did to the step), `preview`
  (show the part the step builds, or the part before it), `commit` (put the
  step into the program — only if it builds) and `cancel`.
- **Extensions:** `panel` (a command to a panel, by key) and `tool` (take a
  tool in hand by its key, or put it down: with no step edited, the
  viewport's events are the tool's).
- **Output commands**, which change nothing: `export` (write the part with
  an exporter, by its kind, in a format, told its own options: the update's
  `export`) and `inspect` (ask an inspector about the part: the update's
  `inspection`, `{kind, report, shown}`: the answer as data, and as the
  inspector lays it out). The program state lists the `exporters` and
  `inspectors` offered, so a front end's menus are made from them.

After every command that changed something the editor settles the state
(the registry's `Settle`: while a step that pulls nothing is edited, only
the program up to it runs; once the whole program has run, state nothing
solves for goes), runs what is shown, and tells the panels once an edit.

An update holds the program as a list of steps shows it — each step's
fields in one line and whether it failed — the panels' states, the part to
draw, and the step being edited with its presentation and whether it
builds. What did not change since the last update is left out: a hover
sends only the step, and the part is sent again only once something ran.
The steps are run by one `Runner` — up to and including the step being
edited while there is one, else as far as the program runs — on one part,
a memo table of every step's nodes, so the part before the step being
edited is always at hand, and a change to it makes again only what reads
what changed. The builds of the last few files edited are kept, so opening
one again builds nothing. The parts placed in the part drawn are sent as
what changed since the last scene: their components once each, by key.
So are the faces' meshes, of the part and of the components: a face is
sent by its mesh's key, each mesh once, and the viewer forgets one when
told to (`forgotten_meshes`), once nothing drawn has used it for longest.

What the shown steps built on — their reference fields' values — is hidden
from the drawing: sketches and whole datums, since what was made from them
shows them now; none that something of could be picked while a field waits
for a pick. Every scene lists the part's structure (`Registry::structure`);
`Command::Visibility` shows or hides one entry by name over what the editor
would by itself, for as long as the same file is edited.

`testing` (feature `testing`) drives an editor as a front end does:
`editor_with(registry, program)`, `Stepping` (one step edited, gesture by
gesture), pointers, clicks and dialog fields.

## Editing one step

How a field is edited is the same for every operation, so it is not the
operations' but the `StepEditor`'s (`step_editor`), which edits one step.
It turns what the user did — a `StepEditEvent` (see
[What is shown and what reaches it](#what-is-shown-and-what-reaches-it))
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
