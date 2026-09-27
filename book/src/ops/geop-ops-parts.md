# geop-ops-parts

> Brief overview only — full documentation is coming later.

The program format: the operations a part is built from, and the code that
runs them, backed by its `geop-ops-parts-derive` companion crate. See the
main [README](https://github.com/geop-cad/geop#programs) for the JSON
program format itself.

## Programs

A `Program` is an ordered list of `Step`s, each an operation with its
arguments and an id. Everything a step creates is named after that id (see
[geop-core-part](../core/geop-core-part.md)), and steps refer to what
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

## Operations

Every operation is a unit struct implementing `Operation`, with an `Args`
struct of plain, serializable design data. `PartOperation` lists them all
and serializes as `{"operation": "extrude", "args": {...}}`:

| Operation   | Does                                                              |
| ----------- | ----------------------------------------------------------------- |
| `AddSketch` | places a sketch on a base plane, a planar face or a datum plane   |
| `Extrude`   | sweeps a sketch's regions along its plane's normal, by a signed `distance`, optionally `symmetric` |
| `Revolve`   | sweeps a sketch's regions a full turn around one of its lines     |
| `Boolean`   | unites, intersects or subtracts two solids                        |
| `AddDatum`  | adds a reference point, axis or plane                             |

Extrude and revolve take a `Combine`: keep the result as a `NewBody`, or
immediately unite, intersect or subtract it with a `target` solid.

**Entity references.** A step that builds on existing geometry names it
with an `EntityRef`: the `Origin`, a world `Axis`, a base `Plane`, or a
`Vertex`, `Edge`, `Face` or datum of the part by name. What an entity *is*
(its `Geometry`: a point, a line, a plane, an arc, something round) is
decided by its shape, not its kind, using `geop-core-geometry`'s shape
recognition: a straight edge is a line, a circular one has a center and an
axis, a flat face is a plane.

**Datums.** `AddDatum` builds reference geometry from a selection in one of
the ways CAD systems commonly offer (`Construction`):

- **points:** offset point, midpoint, point on edge, center, projection onto
  a plane or a line, line meets plane, lines meet, three planes meet;
- **axes:** through two points, along a line, axis of an arc or cylinder,
  two planes meet, perpendicular to a plane or a line, parallel through a
  point, angle bisector, tangent to an edge;
- **planes:** offset, midplane, through three points, at an angle, through a
  line and a point, through two lines, parallel through a point, normal to a
  line or an edge.

`inspect_selection` tells an editor which constructions fit a selection,
using exactly the matching a step applies.

**Handles.** An operation can describe where its adjustable values sit in
space: a `Handle` has a position, a motion (along a line or in a plane) and
the path of the argument it rewrites. An editor draws the handles it wants
and turns a drag into new argument values, so dragging edits the program
exactly as typing into a form does, and the editor needs to know nothing
about the operation. How the 3-D viewer draws and drags them is
described under [geop-cad-web](../cad/geop-cad-web.md#handles-in-the-3-d-viewer).

## Schemas and the derive crate

`OperationSchema` describes every operation and its arguments, so a UI can
build a form for any operation without knowing it by hand. A new operation
shows up in every editor without touching any of them. The schemas are
generated by `geop-ops-parts-derive`:

- `#[derive(OperationArgs)]` on an arguments struct: each field carries
  `#[arg(<ArgKind>)]` saying what it holds and so how it is entered (e.g.
  `Number { default, min, max }`, `Sketch`, `Solid`, `Plane`, `Selection`),
  and its doc comment becomes its description.
- `#[derive(Operations)]` on `PartOperation`: each variant `Name(NameArgs)`
  dispatches to the unit struct `Name`, the doc comment describes the
  operation, and `#[operation(label = "...")]` gives its short name.

## Examples

`examples` holds programs written in Rust (`box_with_drill_hole`,
`bracket`, `cross_drilled_shaft`, `two_plates`, `boss_on_reference_plane`,
`handle_with_hole`, `luggage_tag`). The tests use them, and they serve as a
reference for writing new programs.
