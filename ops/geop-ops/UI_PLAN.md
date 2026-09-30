# Plan: one interaction model for every operation

## Goal

Every operation — sketch editing, reference constructions (datums), extrude,
revolve, booleans — returns a presentation generic enough that the editor can
give all of them the same good UX. An operation says *what its values mean*;
the editor decides *how they are shown and manipulated*. A pick, a selection,
a drag, Escape or Delete then behaves the same whatever is being edited, and a
new operation gets that behavior for free instead of reimplementing it.

## The problem

`StepEditor` (`src/ui/step.rs`) already owns the right things — arming a pick
field, hover, dragging handles, Escape — but its vocabulary is too small, so
operations fill the gaps themselves, each differently. The root issue is that
[`Control`](src/ui/dialog.rs) describes **widgets** (select, list, checkbox,
slider), so operations choose widgets. It should describe **meaning** — a
reference to a line, a length, a set of entities — and leave the widget and
the interaction to the editor.

### Where the UX already diverges

1. **Revolve picks its axis from a dropdown**
   (`geop-ops-extrude-revolve/src/operation/revolve.rs`, `form`). Not revolve's
   fault: `EntityRef` and `Target` cannot name a curve *inside* a sketch, so
   there is nothing to pick. The same gap means you cannot revolve around a
   datum axis or a straight edge, which a user would expect.
2. **Datums reimplement a multi-pick** (`geop-ops-datums/src/editor.rs`):
   add-or-remove toggling in its own `set`, a separate `List` control that
   duplicates the pick's value with `selection:{i}` keys parsed back from
   strings, its own "Clear" button, and hand-built per-item status ("not
   found", roles). Every future multi-pick would copy this.
3. **The sketch runs a parallel interaction system**
   (`geop-ops-sketch/src/editor/gestures.rs`): its own hit testing, hover,
   shift-toggle selection, drag state machine, Escape and Delete. Selecting in
   a sketch and picking for a step behave differently; handles are dragged by
   the editor, sketch points by the sketch.
4. **"Select things, then offer what fits" exists twice**: datum
   constructions that fit the selection, and sketch constraints that fit the
   selection. Same pattern, two implementations, two looks.
5. **Handles are coupled to fields by a string convention**: a handle
   `Visual`'s key must equal its `Number` field's key (`"param:distance"`),
   and `StepEditor::drag` looks the value up in the dialog. Nothing enforces
   it.
6. **Every field is described twice** — in `form` (key, control) and in `set`
   (key, parse the `Value`) — which is where drift creeps in (e.g. revolve's
   `"axis"` parsed as a `u64` out of a `Choice` string).

## Target design

The `Operation` trait keeps its shape — `new_args`, `apply`, `form`, `set`,
and `event` only for tools that draw. What changes is the vocabulary between
operation and editor, and with it how much code an operation needs.

### 1. References by role, not by widget or entity kind

Replace `Control::Pick` + `Target` with a semantic reference field:

```rust
Control::Reference {
    role: Role,                       // Point | Line | Plane | Solid | Profile | …
    value: Vec<(EntityRef, Status)>,  // per item: what it resolves to, or why it does not
    multiple: bool,
}
```

- `Role` is the classification that today lives in `geop-ops-datums`
  (`geometry::{Geometry, Role}`), moved into `geop-ops` next to the resolvers
  on `EntityRef` (`resolve_plane`, plus a new `resolve_line`, `resolve_point`).
  A role accepts everything that *is* such a thing geometrically: a `Line` is
  a straight edge, a datum axis, a frame axis or a sketch line.
- `EntityRef` gains sketch sub-entities (`SketchCurve { sketch, curve }`,
  `SketchPoint { sketch, point }`), and `PartView` picks them.
- The editor owns everything about the field: arming, toggling for
  `multiple`, removing an item, clearing, showing each item's status.
- Revolve's axis becomes `Reference { role: Line }`; the datum selection
  becomes `Reference { role: Any, multiple: true }` and its `List`, "Clear"
  and toggle code go away.

### 2. Handles belong to number fields

```rust
Control::Number { value, unit: Length | Angle | Count, handle: Option<Track> }
```

with `Track { at, direction }`. The editor draws the handle and drags it; the
key convention between visuals and fields disappears, and every length gets
the same drag, snap and typed entry. `Shape::Handle` goes away.

### 3. Commands offered for a selection

One primitive for "what can be done with what is selected":

```rust
Control::Commands { items: Vec<Command { key, label, group, enabled, reason }> }
```

Datum constructions and sketch constraints both become this, so both show
applicable commands, disabled ones with the reason ("needs a point and a
plane"), in the same way.

### 4. Interaction declared on visuals, owned by the editor

Visuals declare their affordances instead of operations handling raw events:

```rust
Visual { key, shape, style, selectable: bool, drag: Option<Drag> }
enum Drag { Along(Track), InPlane }
```

The editor owns hover, selection (click, shift-toggle, Escape clears, Delete
sends a delete), and dragging along a track or in the focus plane. The
operation receives high-level values: `Value::Selection(Vec<String>)`,
`Value::MovedTo { key, point }`. Hover and selection styling come from the
editor, not from each operation setting `Style::Hover` / `Style::Selected`.

The sketch's `event` shrinks to what is genuinely its own: drawing tools
consuming clicks in the focus plane. Selecting and dragging a sketch point
then works exactly like picking an entity or dragging an extrude's distance.

### 5. (Optional) Describe each field once

A `Form` builder where each field carries its setter, e.g.

```rust
d.length("distance", args.distance, |a, v| a.distance = v);
```

removes most `set` match arms. Cross-field consequences — `Combine::follow_sign`,
revolve resetting its axis when the sketch changes, datums keeping the
construction fitting the selection — move into one
`normalize(before, &mut args)` hook. This is the most invasive step and 1–4
already remove most of the drift, so it comes last and only if it still pays.

## Steps

Each step is independently shippable and keeps the full suite green.

1. **Roles and sketch sub-entities.** Move `Role`/`Geometry` into `geop-ops`;
   add `EntityRef::SketchCurve` / `SketchPoint` and picking them in
   `PartView`; replace `Pick`/`Target` with `Reference`/`Role`. Revolve's
   axis becomes a line reference (sketch line, edge, datum axis).
2. **The editor owns reference fields.** Toggle, remove, clear and per-item
   status in `StepEditor` and the dialog; delete the datum crate's `List`,
   "Clear" and toggling.
3. **Handles on `Number`.** Add `unit` and `handle`; drop `Shape::Handle` and
   the key convention; migrate extrude and datums.
4. **Editor-owned selection and drag.** Affordances on `Visual`,
   `Value::Selection` / `Value::MovedTo`; move the sketch onto them and delete
   its hit testing, hover, selection and drag code.
5. **Commands for a selection.** `Control::Commands`; datum constructions and
   sketch constraints both use it.
6. **(Optional) Fields with setters and a `normalize` hook.**

## Done when

- No operation hit-tests, tracks hover, or keeps selection or drag state of
  its own — only drawing tools consume raw events.
- No operation parses its own keys back out of strings (`"selection:3"`,
  `"param:x"`, ids in `Choice` values).
- Every reference to geometry is picked in the viewport, with the same arming,
  highlighting, removal and status.
- The sketch editor tests (`geop-ops-sketch/src/editor/tests.rs`) and the
  datum editor tests pass against the editor-owned interaction.
