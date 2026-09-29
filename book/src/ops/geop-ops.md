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


A `Part<S>` holds a topology `Model`, the `PlacedSketch`es and `Datum`s
used to build it, and a `NameRegistry` mapping every vertex, edge, face,
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
  built along.
- **Lookups by name** (`vertex_id`, `edge_id`, `face_id`, `solid_id`,
  `sketch_id`, `datum_id`, and `coedge_id(edge, face)`) are what every
  operation that refers to existing entities is built on.
- **`PartDescription::of`** describes the topology in names only, with no
  internal id anywhere, so two parts built by the same program describe
  identically and the description diffs well as text.

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
struct of plain, serializable design data and a `Session` holding the
temporary state of editing a step of it. Built, a step maps

```text
(part, args) -> part                                            apply
```

and edited, every user action maps

```text
(part, args, session, event) -> (args, session, presentation)   edit
```

— the editor reruns the program with the new arguments, shows the
presentation, and sends the next event. `edit` never fails: an editor needs
a dialog most exactly when the arguments do not build, so whatever goes
wrong is said in the dialog. Besides these, an operation gives the
arguments of a new step (`new_args`, from what the part before it holds), a
one-line `summary`, and the sketches and datums a step builds on
(`references`), which an editor hides once they are used.

- **Arguments** are numbers, choices, sketches, and references to entities
  by name (`EntityRef`): what the program stores.
- **The session** is everything else of an edit: the tool in hand, a
  half-drawn curve, which field waits for a pick, what the pointer is over.
  It is never saved, and starts afresh whenever an editor opens a step.
- **The event** is what the user did: a dialog control used, a hover, a
  click or a drag in the viewport — each a `Pointer`, the ray through the
  cursor with what a screen pixel measures along it — or a key.
- **The presentation** is what to show: a `Dialog` — an ordered list of
  keyed primitives: headings, texts, buttons, checkboxes, sliders, selects,
  lists — and `Visual`s to draw in the viewport: points, polylines, filled
  areas, labels and handles, each with a key and a style. It also says which
  entities of the part to light, what a click picks now, whether to work in
  a plane (head on, with a grid), and whether a press where the pointer is
  starts a drag.

So an editor renders primitives and forwards raw input, and every decision
— what a click picks, what a line snaps to, what a drag changes — is made in
Rust, where it can be tested. The `ui` module holds the helpers that make
operations answer consistently:

- **Hit tests** against visuals (`hit::hit_visuals`) and against the part
  as drawn (`PartView::pick`): tolerances in screen pixels, the smallest
  entity under the pointer winning — a vertex over an edge over a face, the
  origin over an axis over a base plane. `PartView` is the part as the
  viewport draws it — its rasterization, sketch outlines, datums and the
  origin's gizmo, laid out by the same sizes the viewer draws them at — so a
  pick cannot disagree with what is on screen.
- **`Picking`**: a dialog field waiting for an entity; a hover finds what a
  click would pick, a click picks it.
- **`Dragging`**: a handle dragged along its direction, the value following
  the pointer's projection onto the handle's track.

**Entity references.** A step that builds on existing geometry names it
with an `EntityRef`: the `Origin`, a world `Axis`, a base `Plane`, or a
`Vertex`, `Edge`, `Face`, `Solid`, `Sketch` or datum of the part by name.
What an entity *is* (its `Geometry`: a point, a line, a plane, an arc,
something round) is decided by its shape, not its kind, using
`geop-core-geometry`'s shape recognition: a straight edge is a line, a
circular one has a center and an axis, a flat face is a plane.

No operation lives here. Each is a plugin in a crate of its own, built on
this one and using nothing the others cannot: `AddSketch` in
[geop-ops-sketch](./geop-ops-sketch.md), `AddDatum` in
[geop-ops-datums](./geop-ops-datums.md), `Extrude` and `Revolve` in
[geop-ops-extrude-revolve](./geop-ops-extrude-revolve.md), `Boolean` in
[geop-ops-booleans](./geop-ops-booleans.md).

## Operation sets

A set of operations is an enum with one variant `Name(NameArgs)` per
operation, implementing `Operations` — what a program step holds, and what
serializes as `{"operation": "extrude", "args": {...}}`. Which operations
an application offers is its own choice, so the set is defined there: the
editor's is `PartOperation` in [geop-cad-base](../cad/geop-cad-base.md).

## The derive crate

`#[derive(Operations)]` (from `geop-ops-derive`) on an enum of operations
implements `Operations`: each variant `Name(NameArgs)` dispatches to the
unit struct `Name`, the doc comment describes the operation, and
`#[operation(label = "...")]` gives its short name. Sessions differ by
operation, so the set carries a step's session as JSON.

## Programs

`Program`, `ProgramEdit` and `ProgramRunner` are generic over the operation
set a program is written in.

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
    plane: EntityRef::Plane { normal: WorldAxis::Z },
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

A program changes only through `Program::update(ProgramEdit)`: `Insert`,
`Update`, `Remove`, `Move` or `Replace`. Edits address steps by id rather
than position, so an edit means the same thing however the steps around it
have moved. An edit that would leave the program invalid (an unknown step, a
duplicate or malformed id) is rejected and changes nothing. Whether the
steps still *build* is a separate question, answered by running them.
Editing lives here rather than in an editor, so that the browser UI, a
script, or any future editor all change programs the same way.

`Program::apply` builds a part from scratch and checks after every step that
every entity has a name. `ProgramRunner` builds incrementally for an editor:
`run(program, stop)` runs the first `stop` steps, reuses whatever earlier
runs built that still applies, and stops at the first failing step, since
the steps after it would fail for lack of what it should have built.
`to_json` pretty-prints one step per object, with every sketch entity keyed
by its id, so edits show up as small line diffs. `ProgramRunner` also
reports what the steps it built build on (`references`).
