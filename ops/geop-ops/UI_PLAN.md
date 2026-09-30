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

## Status

All six steps are done. What was built differs from the sketch above in a
few names and details:

1. **Roles and sketch entities.** `Aspects` (what an entity can be used as;
   the name avoids "geometry", which in this kernel means NURBS curves and
   surfaces) and `Role` live in `geop_ops::operation`. `EntityRef` has
   `SketchCurve { sketch, curve }` and `SketchPoint { sketch, point }`,
   resolved from the sketch's enclosed solution (`Sketch::enclose`).
   `PartView` draws sketch points and stores each drawn entity's roles, and
   `PartView::pick(pointer, roles, scope)` offers only what can fill one,
   within the scope if one is given. `Target` is gone.
   Revolve's axis is any `Role::Line` in the sketch's plane. A region
   touching the axis along an edge revolves as before; that needs the axis
   to be a line of the sketch, because only then do the constraints say
   which edges lie on it. A region clear of the axis revolves into a ring:
   `revolve_at_oriented` now takes closed profiles, building the genus with
   `mer` / `mekr`.
2. **The editor owns reference fields.** `Control::Reference` holds
   `Picked` entities, and the editor sets each one's detail and tone.
   `Value::RemoveAt` / `Value::Clear` come from the viewer; an operation's
   setter only ever receives the entities the field holds now. The datum
   crate's list, "Clear" button, toggling and `SelectionFit::roles` are
   deleted (`inspect_selection` became `fitting_constructions`).
3. **Handles on `Number`.** `Number { unit, range, step, handle:
   Option<Track> }`. The editor turns handles into `Shape::Handle` visuals.
   That shape still exists for the viewer to draw, but no operation
   produces it, and the key convention is gone.
4. **Editor-owned selection and drag.** `Visual::selectable` / `draggable`,
   `Form::tool`, and `StepEditor` owning `selection`, hover styling, the grab
   flag and plane drags. Operations receive `CanvasEvent`: hover, leave,
   clicks the selection did not take, `Move { key, from, to, done }`, and
   keys. The sketch lost its hit-tested selection, hover and grab state.
5. **Actions for a selection.** `Control::Actions` replaces `Buttons` and
   the grouped `Select`. Datum constructions, sketch constraints, sketch
   tools and sketch edits all use it. `Select` is now only a dropdown for
   plain enums.
6. **Fields with setters.** `Form` (`ui/form.rs`) builds each field
   together with its setter (`Form::number`, `Form::reference`,
   `Form::actions`, ...; `Form::on` for a list's items). `Operation::set`
   has a default implementation that runs the setter for the key, so no
   operation implements `set` any more. No separate `normalize` hook was
   needed: a setter may capture the part before the step, which covers
   every cross-field consequence (the datum construction following the
   selection, revolve's axis following its sketch, extrude's combine mode
   following the distance's sign).

## Done when

- [x] No operation hit-tests for selection, tracks hover, or keeps selection
  state of its own. The sketch still hit-tests its own visuals for *snapping*
  a drawn point, and remembers where a drag started and what it moves. Both
  are specific to drawing.
- [x] No operation parses dialog keys back out of strings. The sketch's
  constraint list registers one setter per item, capturing the item's id.
  Visual keys like `p3` / `c3` / `k3` are the sketch's own names for its
  entities, and it still reads them from the selection.
- [x] Every reference to geometry is picked in the viewport, with the same
  arming, highlighting, removal and status.
- [x] The sketch and datum editor tests pass against the editor-owned
  interaction. New regression tests: `revolve_axes_are_picked`,
  `selections_are_edited_in_their_field`, `sketch_lines`,
  `sketch_points_and_lines`, `fields_serialize_flat`, the ring revolves
  (`square_ring_is_valid`, `torus_is_valid`,
  `revolve_square_around_an_outside_axis_is_a_ring`, ...), and the sketch
  enclosure (`solutions_are_enclosed`, `free_choices_stay_sharp`,
  `outline_revolved_around_its_edge_joined_to_its_box`).
