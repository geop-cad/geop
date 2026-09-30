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
  built along. Every part starts with one: the coordinate system `origin`
  (`geop_ops::ORIGIN`), the world's origin and axes. A coordinate system
  stands for its origin, its three axes and the three planes between them,
  each usable on its own (`DatumComponent`).
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
struct of plain, serializable design data. Built, a step maps

```text
(part, args) -> part                                            apply
```

and edited, it shows a `Form` — fields and visuals — whose fields an editor
sets:

```text
(part, args)                 -> form                            form
(part, args, field, value)   -> args                            set
```

`form` never fails: an editor needs a dialog most exactly when the arguments
do not build, so whatever goes wrong is said in it. Besides these, an
operation gives the arguments of a new step (`new_args`, from what the part
before it holds).

- **Arguments** are numbers, choices, sketches, and references to entities
  by name (`EntityRef`): what the program stores.
- **The form** is a `Dialog` — an ordered list of keyed fields: headings,
  texts, buttons, checkboxes, sliders, selects, lists, and entities to pick
  — and `Visual`s to draw in the viewport: points, polylines, filled areas,
  labels, and handles, each bound to a number field.
- **A field is set** by the dialog, by a pick in the viewport, or by
  dragging its handle. Which of these it was, the operation never knows.

Picking and dragging are the same for every operation, so they are not the
operations' but the `StepEditor`'s, which edits one step. It turns what the
user did — an `Event`: a dialog field used, a hover, a click or a drag in
the viewport, each a `Pointer` (the `Ray` from the eye through the cursor,
and its `Reach`: how far from the ray counts as under it, a cone from the
eye in perspective, a tube in an orthographic view), or a key — into fields
set:

- **Pick fields** arm on a press; the editor then finds what a click would
  pick on hover, lights it, and sets the field on a click — once for a
  field of one entity, again and again for a field of several. A new step
  starts with its first pick field armed: what it is built on is what it
  needs first. While a field waits for a pick, the part is shown as it
  stands, not the plane the step works in.
- **Handles** are dragged along their direction, the value following how
  far the pointer has moved along the handle's track, from the ray where the
  drag started to the ray where it is now, snapped to a hundredth.

An operation that draws in a canvas of its own — a sketch — also takes the
pointer and key events the editor does not (`Operation::event`), with a
`Session` for what it keeps between them: the tool in hand, a half-drawn
curve. Every other operation's session is `()`. What the editor sends back
is a `Presentation`: the form, what the pick fields hold and what a click
would pick lit, and what a click picks now.

So an editor renders primitives and forwards raw input, and every decision
— what a click picks, what a line snaps to, what a drag changes — is made in
Rust, where it can be tested. Hit tests against visuals (`hit::hit_visuals`)
and against the part as drawn (`PartView::pick`) decide what a pointer is
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
its axes or planes. `resolve_plane` gives the plane one lies in, what a
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
operation, so the set carries a step's session boxed.

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

`Program::build` builds a part from scratch and checks after every step that
every entity has a name. `ProgramRunner` builds incrementally for an editor:
`run(program, stop)` runs the first `stop` steps, reuses whatever earlier
runs built that still applies, and stops at the first failing step, since
the steps after it would fail for lack of what it should have built; the
part after any number of them is `part_at`. `to_json` pretty-prints one
step per object, with every sketch entity keyed by its id, so edits show up
as small line diffs. How a program is edited is an editor's (see
[geop-cad-base](../cad/geop-cad-base.md)).
