# geop-ops

> Brief overview only — full documentation is coming later.

The elementary structures the kernel's operations work with: parts,
operations, and programs made of operations, backed by the
`geop-ops-derive` companion crate. It defines what an operation is but no
operation itself: every other `geop-ops-*` crate builds on this one and
adds operations of its own. See the main
[README](https://github.com/geop-cad/geop#programs) for the JSON program
format itself.

## The part


The `part` module. A `Part<S>` holds a topology `Model`, the
`PlacedSketch`es and `Datum`s used to build it, and a `NameRegistry` mapping every vertex, edge, face,
solid, sketch and datum to exactly one name, and every name to exactly one
entity.

Its fields are private. Every Euler and edit operator of
[geop-core-topology](./geop-core-topology.md) has an identically named
method on `Part` that forwards to it, registers each entity it creates under
the name the caller supplies, and forgets the name of each entity it
deletes. The invariant checked by `Part::check_names` therefore holds by
construction, not by the diligence of each caller.

- **Sketches** are stored with the plane they were placed on. The plane is
  resolved once, when the sketch is added, so a sketch drawn on a face stays
  where that face *was*, even after a later operation reshapes or consumes
  the face.
- **Datums** are reference geometry — points, axes, planes and coordinate
  systems the part is built *with*, not *of* (the `Datum` of
  [geop-core-math](../core/geop-core-math.md#render-primitives)). Every datum
  carries a full right-handed frame, so anything built on it has axes to be
  built along. Every part has one no step adds: the coordinate system
  `origin` (`geop_ops::ORIGIN`), the world's origin and axes. Every other
  datum is the target `datum(name)` its step defines, resolved through
  `Lookup::datum` and drawn with the part. A coordinate system
  stands for its origin, its three axes and the three planes between them,
  each usable on its own (`DatumComponent`).
- **Lookups by name** (`vertex_id`, `edge_id`, `face_id`, `solid_id`,
  `sketch_id`, `datum_id`, and `coedge_id(edge, face)`) are what every
  operation that refers to existing entities is built on.
- **Instances** are other parts placed in this one — the part a program
  file builds, shared by every instance of it and drawn once, with the file
  it came from — at a `Pose`, free to be moved by the **mates**: constraints
  between two entities, a `fixed` one among them holding a part in place (see
  [geop-ops-assembly](./geop-ops-assembly.md)). A placed part is a
  part too, so a part is a tree. An entity of a placed part is named behind
  its instance's name — `bolt/extrude(head,end)` — and resolves where it is
  placed: `Aspects::of`, `resolve_plane` and `resolve_datum` move what they
  find by the instance's pose, so any operation can build on it.
- **State**: the parameters a program is built with, kept in the
  program's `state` apart from its steps. A step reads one by
  name, declaring it (`Part::pose_parameter`): `add_part` places its part
  where `<id>.pose` says. Solving the program's state (`solving`) gives
  new values for them, which the editor keeps — so every step sees every
  placed part where the constraints put it, however late in the program
  those constraints are.
- **`PartDescription::of`** describes the topology in names only, with no
  internal id anywhere, so two parts built by the same program describe
  identically and the description diffs well as text.

## Solving the state (`solving`)

What an editor does with a part's state, the same for every program, on
`geop_core_program::solve`: `solve` settles it so every constraint holds —
pulled by a drag (`Pull::Point`), a pose typed in (`Pull::Pose`), a value
set (`Pull::Value`, an angle driven there in steps) — `set` sets values a
user sets, `check` says which constraints hold, `freedom` how free each
placed part is, and `conflicting` which constraints cannot hold together.
Nothing here knows what the constraints are. Each placed part moves by its
pose state; a step declares any other value a solve moves, or holds one
fixed, as an `Unknown` (a joint's coordinate, measured from its parts where
the joint does not hold; the part a fixed mate holds), and the values a
user sets directly as `Settings` (a joint's coordinates and limits), which
`settings` lists for an editor. A solve's `Report` says which values no
true solution is proven for, and why.

## Topological naming


A name says *how* an entity came to be, never *when*. It is built only from
inputs that stay the same when a part is rebuilt after an upstream edit:
operation ids, sketch element ids, the names of the entities an operation
consumed, and positions counted along those. It never uses an internal id
or the order in which an algorithm happened to create things. A program that
refers to `extrude(box,end)` keeps meaning the same face when the box gets
taller.

Every name has the form `kind(operation,arg,...)`, built by a `Namer`:
`kind` is the operation that created the entity, `operation` the id of the
program step that ran it, and the arguments identify the entity within that
step. For example:

| Name                      | Entity                                                     |
| ------------------------- | ---------------------------------------------------------- |
| `extrude(E)`              | the solid built by extrude step `E`                        |
| `extrude(E,start)`, `extrude(E,end)` | its caps                                        |
| `extrude(E,K,c3)`         | the side face swept by line `c3` of sketch `K`             |
| `extrude(E,K,c3,end)`     | that face's edge on the end cap                            |
| `extrude(E,K,p1)`         | the edge swept by sketch point `p1`                        |
| `boolean(B,E1,E2,i,n)`    | the `i`-th of `n` points where edges `E1` and `E2` cross, counted along `E1` |

Operation ids may only contain ASCII letters, digits, `_`, `-` and `.`
(`validate_operation_id`), so the `(`, `)` and `,` that structure a name
stay unambiguous even when an argument is itself a name.

## Operations

Every operation is a unit struct implementing `Operation`, with an `Args`
struct of plain, serializable design data. Built, a step maps

```text
(part, args, library) -> part                                   apply
```

— the library being where it finds the parts it places, if it places any —
and edited, it shows a `Form` — fields and visuals — whose fields an editor
sets:

```text
(context, args)                 -> form                         form
(context, args, field, value)   -> args                         set
```

with the `Context` the part before the step, the step's id, the library,
the program's state, and what the step built when it last ran — and,
with an event, the part as the editor draws it (`Context::scene`), so an
operation picks in the editor's drawing rather than drawing the part
again for every move of the pointer. A
setter may also set a parameter (`Edit::state`), and a form may ask
for placed parts to be dragged (`Form::drags`): solving them is the
editor's.

`form` never fails: an editor needs a dialog most exactly when the arguments
do not build, so whatever goes wrong is said in it. Besides these, an
operation gives the arguments of a new step (`new_args`, from what the part
before it holds).

- **Arguments** are numbers, choices, sketches, and references to entities
  by name (`EntityRef`): what the program stores.
- **The form** is a `Dialog` — an ordered list of keyed fields that say
  what their values *mean*: headings and texts, actions (things to do or
  choose, grouped, one perhaps active, a disabled one saying why), checkboxes,
  numbers (with a unit, a slider range, and perhaps a handle to drag them
  by), selects, lists, and references to entities of the part — and
  `Visual`s to draw in the viewport: points, polylines, filled areas and
  labels, each perhaps selectable or draggable.
- **A field is set** by the dialog, by a pick in the viewport, or by
  dragging its handle. Which of these it was, the operation never knows.
  Each field is described once: the form (`Form`) holds, next to each
  control, the setter its value goes to, and `Operation::set` finds it by
  the field's key — an operation never matches keys itself. A setter may
  capture the context, for what setting one field implies for
  others: a selection picks the construction it fits, another sketch brings
  its own axis.

How a field is edited is the same for every operation, so it is not the
operations' but the `StepEditor`'s, which edits one step. It turns what the
user did — a `StepEditEvent`: a dialog field used, a hover, a click or a drag in
the viewport, each a `Pointer` (the `Ray` from the eye through the cursor,
and its `Reach`: how far from the ray counts as under it, a cone from the
eye in perspective, a tube in an orthographic view), or a key — into what
the operation understands:

- **Reference fields** hold entities that can fill one of the field's
  `Role`s — a point, a line, a plane, an edge, a circle, something round, a
  solid, a sketch — and, with a scope, are part of it: a revolve's axis is a
  line *of the sketch it revolves*. Which roles an entity fills is decided by
  its shape, not its kind (`Aspects`): a straight edge, a datum axis, a
  frame's axis and a sketch line are all lines. A field arms on a press; the
  editor then finds what a click would pick on hover, lights it, and picks it
  on a click — once for a field of one entity, again and again (each pick
  adding one or taking it out) for a field of several. Taking one out, or
  all, is the editor's too, as is saying what each entity held is used as,
  or why it cannot be. The operation is only ever sent what the field holds
  now. A new step whose first reference field is still empty starts with it
  armed: what it is built on is what it needs first. One that already holds
  something — an extrude's newest sketch — waits for no click, so its
  handles can be dragged straight away. So does a field that appears empty
  as the step is edited — a mate just added waits for its entities — and a
  field that goes away stops waiting. While a field waits for a pick, the
  part is shown as it stands, not the plane the step works in. Picks test
  against the part before the step, or — for an operation whose references
  point into what it adds itself (`OperationInfo::picks_built`), a placed
  part's mates —
  against what it builds.
- **Handles** of number fields are drawn by the editor and dragged along
  their direction, the value following how far the pointer has moved along
  the handle's track, from the ray where the drag started to the ray where
  it is now, snapped to a hundredth.
- **Selecting** is the editor's: a click on a selectable visual selects it
  or takes it out again, a click on nothing clears the selection unless
  shift is held, and Escape clears it. The operation reads the selection in
  its form and may change it as its fields are used. The editor draws what
  is selected and what the pointer is over.
- **Dragging** a draggable visual is the editor's too: it is grabbed where
  the drag starts, and the operation is sent where to, in the plane worked
  in — or, with none, in the plane through where it was grabbed, facing the
  eye.

An operation that draws in a canvas of its own — a sketch — also takes what
the editor passes on (`Operation::process_user_event`, an `Event::Canvas`):
clicks while it has a tool in hand, and those on nothing, hovers, keys, and
moves of its draggable visuals — with its `EditingState` for what it keeps
between them: the tool in hand, a half-drawn curve. An operation that keeps
nothing has `()`.
What the editor sends back is a `Presentation`: the form, what the
reference fields hold and what a click would pick lit, what is selected and
hovered, the handles, and what a click picks now.

So an editor renders primitives and forwards raw input, and every decision
— what a click picks, what a line snaps to, what a drag changes — is made in
Rust, where it can be tested. Hit tests against visuals (`hit::hit_visuals`)
and against the part as drawn (`PartView::pick`, an operation's through
its context's `Scene`) decide what a pointer is
over: near means within its reach, and the smallest entity under it wins —
a vertex over an edge over a face, a coordinate system's origin over its
axes over its planes. What the viewer draws at a constant size on screen —
handles, labels, coordinate systems — is laid out in reaches. `PartView` is
the part as the viewport draws it — its rasterization, sketch outlines and
datums, laid out by the same sizes the viewer draws them at, and serialized
as what a viewer draws — so a pick cannot disagree with what is on screen.
The ray geometry itself — how near a ray passes to a point, a segment, a
triangle, a plane — is `geop_core_math::primitives::Ray`'s, and all of it is
in the part's scalar type.

**Entity references.** A step that builds on existing geometry names it
with an `EntityRef`: a `Vertex`, `Edge`, `Face`, `Solid`, `Sketch` or
`Datum` of the part by name — for a coordinate system, perhaps only one of
its axes or planes — or a `SketchCurve` / `SketchPoint`, a curve or a point of a sketch by its
id. `resolve_plane` gives the plane one lies in, what a
sketch is placed on; what else an entity can be used as is for the
operations that build on it to say (see
[geop-ops-datums](./geop-ops-datums.md)).

No operation lives here. Each is a plugin in a crate of its own, built on
this one and using nothing the others cannot: `AddSketch` in
[geop-ops-sketch](./geop-ops-sketch.md), `AddDatum` in
[geop-ops-datums](./geop-ops-datums.md), `Extrude` and `Revolve` in
[geop-ops-extrude-revolve](./geop-ops-extrude-revolve.md), `Boolean` in
[geop-ops-booleans](./geop-ops-booleans.md).

## Operation sets

An operation is a unit struct implementing `geop_core_program::operation::Operation`:
its `INFO` (`OperationInfo`: the kind its steps are stored under, its label,
doc, group and tier), its `Args` and its `EditingState`. A program step
holds the kind and the arguments as JSON —
`{"id": "...", "operation": "extrude", "args": {...}}` — and a run looks the
kind up in a `Registry` of the operations the program may use. Which
operations an application offers is its own choice, so the registry is made
there: the editor's is `registry` in
[geop-cad-base](../cad/geop-cad-base.md).

## What an editor shows and makes of a part

An editor knows no operation by name. Whatever a crate adds to it beyond
its operations, it adds through one of these, and the editor collects or
offers it without knowing whose it is:

- **Annotations** (`ui::annotations(step)`): what a step draws on the part
  besides its topology — a cosmetic thread's helix and designation. A step
  defines its target of type `ui::Annotations`; `PartView::of` draws every
  step's, for the part and for each part placed in it. The hole crate's
  `draw_threads` is the example.
- **Structure entries** (`ui::listing(step)`): what a step lists in the
  part's structure tree beside what every part has (datums, sketches,
  solids, faces standing on their own, placed parts): a `ui::Listed` with a
  kind (`mate`, the tree's group), a name and the entities it refers to.
  A step defines its target of type `ui::Listing`, usually with the
  generator `ui::listed`; `ui::structure` lists them all. `add_part` lists
  its mates this way.
- **Exporters** (`output::Exporter`): a kind of file written from what a
  program built. A unit struct with its `INFO` (`ExporterInfo`: its kind,
  label, a few words of what the file is for, and the formats it writes
  by extension, the default first), its own `Options` (deserialized from
  the JSON a front end sends; `Default` where it sends none) and
  `export(output, format, options)`, which returns an `Export` (a name, a
  media type, text or bytes). `Output` is what it is given: the part as far
  as the program runs (the parts it places included, `part()`), what its
  program says of it (`about()`: its file, material, title and
  designation, the run's `placed::About` input), the step being edited and
  the date — and `freedom(held)`, how free its placed parts are with some
  values held, the one question that solves (the URDF export asks it with
  every joint held, to find parts mates leave loose). `output::Stl` writes
  the mesh the editor draws; drawings and flat patterns are laid out on an
  `output::sheet::Sheet` and written as SVG or DXF. What several outputs
  read of the part is shared: `solids` lists the solids a part shows,
  wherever they are placed, each of its own part's material, with their
  mass properties; `placed::owner` resolves an entity of a placed part. A
  step's dialog offers a file of its own with
  `Dialog::download(key, exporter, formats)`.
- **Inspectors** (`output::Inspector`): a question answered about what a
  program built. Its `INFO` (`InspectorInfo`) says whether it is asked of
  the whole part or of picks (`Picks`: the roles it picks, at most how
  many); `inspect(output, picked, options)` returns its `Report` (any
  `Serialize`) and what to draw of it in the viewport, and `show(report)`
  lays the report out for a front end as an `output::Answer` — text,
  lists, tables, groups, buttons that ask again or save a file, and a
  plane to cut a section with — so a front end shows every inspector's
  answer alike, knowing none. An inspector that picks becomes the editor's
  pick tool, asked again with every pick.

Exporters and inspectors are registered in an `output::Outputs` (`exporter`,
`inspector`; a kind twice is refused), the application's choice like the
operation registry: the editor's is `outputs` in
[geop-cad-base](../cad/geop-cad-base.md).

## Parameters

The `parameters` module. A program's `Parameters` are the named values its
design is given by, defined once in the program and read by name wherever a
value is typed — a sketch dimension of `width / 2`:

- **Numbers**, each a formula of the others, defined before it or
  after — `4`,
  `width / 2`, `sqrt(a^2 + b^2)` — with `+ - * / ^`, parentheses, `pi`
  and `sqrt abs sin cos tan asin acos atan round floor ceil min max`,
  angles in degrees. A `min` and `max` say what a slider offers when the
  part is placed.
- **Tables**: a family of variants — screw sizes, say — one row each, of
  which one is selected. `screw` is the selected row's name, and
  `screw.clearance` its value in the column `clearance`.
- **The part's colour**, `#rrggbb`, read as `color`.

What a parameter is defined as is the program's own; what it is *built
with* may be overridden by a program placing the part, by the same name,
through the state (see [geop-ops-assembly](./geop-ops-assembly.md)). So a
placed screw is made M5 by the program placing it, without touching the
screw's file. `Program::inputs` resolves the definitions with the state's
overrides into the values a build reads; `Part::evaluate` evaluates a
formula against them and declares what it read, so the `ProgramRunner`
reruns from the first step that read a parameter whose value changed — or
that failed, since what it would have read is not known — and nothing for
a change no step read, like the part's colour: the parts it keeps just take
the new values. A
parameter that does not resolve is left out — what reads it fails, saying
so — and the editor shows why.

## Programs

The `program` module. `Program` and `ProgramRunner` are generic over the
operation set a program is written in.

A `Program` is an ordered list of `Step`s, each an operation with its
arguments and an id. Everything a step creates is named after that id (see
[topological naming](#topological-naming)), and steps refer to what
earlier steps built only by name, never by internal id. A program therefore
means the same thing every time it runs, including after a round trip
through JSON: rebuilding a read-back program gives the same part, name for
name.

```rust,ignore
let mut program = Program::new();
program.push("outline", AddSketchArgs {
    plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
    sketch: solved(outline),
});
program.push("box", ExtrudeArgs {
    sketch: "outline".into(), distance: 1.0, symmetric: false,
    combine: Combine::NewBody,
});
program.push("hole_sketch", AddSketchArgs {
    plane: EntityRef::Face { name: "extrude(box,end)".into() },
    sketch: solved(hole),
});
program.push("hole", ExtrudeArgs {
    sketch: "hole_sketch".into(), distance: -0.5, symmetric: false,
    combine: Combine::Difference { target: "extrude(box)".into() },
});
```

`Program::build(library)` builds a part from scratch and checks after every
step that every entity has a name. The `Library` is where the program finds
the parts it places: `Workspace` is the library of a set of files, by path,
each program built with the `Scope` of its own file, so references resolve
relative to it. A workspace keeps what it built until its files change, and
refuses a placement that would make files place each other in a cycle,
naming the cycle: the files must form a DAG. `ProgramRunner` builds incrementally for an editor:
`run(program, stop, library)` runs the first `stop` steps, reuses whatever earlier
runs built that still applies, and stops at the first failing step, since
the steps after it would fail for lack of what it should have built; the
part after any number of them is `part_at`. `to_json` pretty-prints one
step per object, with every sketch entity keyed by its id, so edits show up
as small line diffs. How a program is edited is an editor's (see
[geop-cad-base](../cad/geop-cad-base.md)).
