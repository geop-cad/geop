# geop-core-editor

What an operation is to an editor: the vocabulary a step's editing is said
in — dialogs, visuals, gizmos, events, picks (`editing`) — numbers typed as
formulas (`numeric_input`), what an operation shows and does with them
(`editable`, `part_operation`), how a reference field picks an element
(`selectable`), and the operations an editor offers (`Editors`). A program
runs without any of it (see [geop-core-program](./geop-core-program.md));
the interaction every operation shares, driving these, is the editor's own
(the `StepEditor` of [geop-cad-base](../cad/geop-cad-base.md#editing-one-step)).

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
- `run(step, part, args)`: what it builds. A step that builds into the
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

Everything is edited against a `Context`: the step's id, the part as it was
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

The `numeric_input` module. A `NumericInput` is a number an operation is given — a length, an angle, a
count — typed as a plain value or as a formula of the program's
parameters: numbers, `+ - * / ^`, parentheses, `pi` and the functions
`sqrt abs sin cos tan asin acos atan round floor ceil min max`
(`FUNCTION_NAMES`), angles in degrees (`evaluate`). An operation's
arguments take one wherever they take a number, and its form shows it with
`EditForm::numeric`. A step resolves it where it builds
(`NumericInput::resolve`), reading each parameter the formula names as an
input of the part, so a change to one of them rebuilds the step and a
change to any other does not. Saved, a formula is told apart from every
other value in a step's arguments (`formulas_in`, `rename_parameter_in`),
so what reads a parameter is found, and renamed, without asking the
operation. What the parameters are is
[geop-ops-parameters](../ops/geop-ops-parameters.md)'.

## Operation sets

`Editors` is the operations an editor offers, by kind (`with`): what gives
each step its arguments, form and events, and — through
`Editors::operations` — the `OperationRegistry` its programs run with, so
the two cannot disagree. `infos` lists them, each a `Listed` (kind and
`OperationInfo`), which is what a front end builds its toolbar from. The
editor's set is `editors` in [geop-cad-base](../cad/geop-cad-base.md).
