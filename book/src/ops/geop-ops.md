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

An operation maps a part and its arguments to a new part, together with
what an editor needs to edit the step interactively:

```text
(part, args) -> (part, dialog, handles)
```

Every operation is a unit struct implementing `Operation`, with an `Args`
struct of plain, serializable design data, and the three results are its
three methods: `apply`, `dialog` and `handles`. They are separate because
they are needed separately — a script only builds, and an editor needs the
dialog most exactly when the arguments do not build yet, so `dialog` never
fails.

- **Arguments** are numbers, choices, sketches, and references to entities
  of the part by name. Their schema (below) says how each is entered — a
  slider, a choice, a pick in the viewport — and algebraic types such as a
  datum's `Construction` or an extrude's `Combine` make the form a dialog
  whose fields follow what was chosen.
- **The dialog** (`Dialog`) adds what only the part can tell, per argument:
  what each picked entity can be used as, which of a choice's options fit
  what is picked.
- **The handles** are the step's values placed in the 3-D view.

**Entity references.** A step that builds on existing geometry names it
with an `EntityRef`: the `Origin`, a world `Axis`, a base `Plane`, or a
`Vertex`, `Edge`, `Face` or datum of the part by name. What an entity *is*
(its `Geometry`: a point, a line, a plane, an arc, something round) is
decided by its shape, not its kind, using `geop-core-geometry`'s shape
recognition: a straight edge is a line, a circular one has a center and an
axis, a flat face is a plane.

**Handles.** An operation can describe where its adjustable values sit in
space: a `Handle` has a position, a motion (along a line or in a plane) and
the path of the argument it rewrites. An editor draws the handles it wants
and turns a drag into new argument values, so dragging edits the program
exactly as typing into a form does, and the editor needs to know nothing
about the operation. How the 3-D viewer draws and drags them is
described under [geop-cad-web](../cad/geop-cad-web.md#handles-in-the-3-d-viewer).

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

## Schemas and the derive crate


`OperationSchema` describes every operation and its arguments, so a UI can
build a form for any operation without knowing it by hand. A new operation
shows up in every editor without touching any of them. The schemas are
generated by `geop-ops-derive`:

- `#[derive(OperationArgs)]` on an arguments struct: each field carries
  `#[arg(<ArgKind>)]` saying what it holds and so how it is entered (e.g.
  `Number { default, min, max }`, `Sketch`, `Solid`, `Plane`, `Selection`),
  and its doc comment becomes its description.
- `#[derive(Operations)]` on an enum of operations, implementing
  `Operations`: each variant `Name(NameArgs)` dispatches to the unit struct
  `Name`, the doc comment describes the operation, and
  `#[operation(label = "...")]` gives its short name.

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
by its id, so edits show up as small line diffs.
`ProgramRunner` also reports the handles and the dialog of every step it
ran — the dialog of a failing step too, since that is what helps fix it.
