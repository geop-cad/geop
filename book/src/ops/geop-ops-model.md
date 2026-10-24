# geop-ops-model

> Brief overview only — full documentation is coming later.

A part's topology: the one `NamedModel` its steps build, revised by each in
turn — every vertex, edge, face and solid with a stable name, and the
features the steps made — the kinds of element a step refers to in it, and
what those are used as: points, lines, planes, circles, curves, bodies. A
step that builds into the model is a `ModelOperation`.

## The model (`NamedModel`)

A `NamedModel<S>` holds a topology `Model` of
[geop-core-topology](../core/geop-core-topology.md) and a `NameRegistry`
mapping every vertex, edge, face and solid to exactly one name, and every
name to exactly one entity.

Its fields are private. Every operation of the topology that creates or
deletes entities — `build_body`, `split_edge_at_vertex`,
`splice_edge_into_face`, `merge_vertex`, ... — has an identically named method on
`NamedModel` that forwards to it, registers each entity it creates under
the name the caller supplies, and forgets the name of each entity it
deletes. The invariant checked by `NamedModel::check_names` therefore holds
by construction, not by the diligence of each caller.

- **Lookups by name** (`vertex_id`, `edge_id`, `face_id`, `solid_id`) are
  what every operation that refers to existing entities is built on;
  `names_mentioned` turns the ids in an error back into names.
- **`PartDescription::of`** describes the topology in names only, with no
  internal id anywhere, so two parts built by the same program describe
  identically and the description diffs well as text.
- **Features** (`Feature`, `BooleanOp`, `ToolReach`): what each step that
  combined a tool with a solid did — a cut, a boss, a hole — kept as its
  tools and how each was combined, so a later step can do it again
  elsewhere (a pattern of features).
- **How a step meets a solid** (`Combine`, `SolidRef`, `Tool`, `Extent`):
  the vocabulary every feature that combines shares — a new body, or a
  join, cut or intersection with a solid of the part, each side going a
  length, up to the next face or through all — and its fields in a form.
  Applying it is the boolean's: `combine_tools` in
  [geop-lib-booleans](../lib/geop-lib-booleans.md).

The model is the part's target `model()`, a singleton: a reference into it
is saved as the element's name alone. A step reads it with
`read_model(&reader)`, which gives an empty model before any step built
one. A `ModelOperation` says how a step builds into it
(`ModelOperation::build(model, reader, step, args)`), and its run is
`build_model`, the step's revision of the model.

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
stay unambiguous even when an argument is itself a name. A piece of a
split edge is named after the edge and where it starts (`Namer::piece`);
an argument longer than a limit is spelled by its digest, so names do not
grow without bound when one body is cut many times.

## Elements and what they are used as

The kinds of element a step refers to in the model (`elements`) are
`Vertex`, `Edge`, `Face` and `Solid` — each an `ElementReference` into the
model by its stable name — and `EdgeChain`, the tangent chain of edges
through an edge. Each says how a dialog picks it: a vertex, an edge or a
face as itself, a solid by a face of it, an edge chain by an edge of it.
`reader.retrieve_element(&r)` gives a `ModelElement` (the model and the
id), or a `ChainElement`. `element::<K>(name)` makes a reference.

What an entity can be used as is decided by its shape, not its kind
(`geometry`), using the shape recognition of
[geop-core-geometry](../core/geop-core-geometry.md):

| Kind     | Resolves to                     | In the model                              |
| -------- | ------------------------------- | ----------------------------------------- |
| `Point`  | a point                         | a vertex                                  |
| `Line`   | an `Axis`                       | a straight edge                           |
| `Plane`  | a `CoordinateSystem`, `w` normal | a flat face                              |
| `Round`  | the `Axis` it turns around      | a circular edge, a cylinder, a cone       |
| `Circle` | an `Arc`                        | a circular edge                           |
| `Curve`  | a `Chain` of one curve          | an edge, of a face or of a wire           |
| `Path`   | a `Chain`                       | an edge of a face, not of a wire          |
| `Body`   | a solid, or a sheet by a face   | a solid, or faces standing on their own   |

Other targets have the same kinds where they have that shape — a datum
axis is a `Line` (see [geop-ops-datums](./geop-ops-datums.md)), a sketch
line too (see [geop-ops-sketch](./geop-ops-sketch.md)) — and an operation
that takes one from several of them holds a `one_of!` enum (see
[geop-core-editor](../core/geop-core-editor.md#picking-references)).
`frame_along` gives a frame on an axis whose `u` is where a joint measures
its turns from, so frames put on parallel axes line up.

## Curves, chains and what steps note

- **`Chain`**: curves each on `[0, 1]` and starting where the one before
  ends, with a stable name for every curve and every joint — what a
  profile (`Profile`), a path, a rail or a guide is, however it was drawn
  or picked. `tangent::tangent_chain` finds the `TangentChain` of edges
  through an edge.
- **Solid notes** (`solid_notes`, `notes_of`): what a step says of a solid
  beyond its shape — a solid only drawn, which is no body (a harness's
  swept bundle), a sheet-metal body's thickness, the wires ordered in a
  bundle's place — read by whoever weighs, checks or orders a part.
- **Cosmetic threads** (`threads`, `read_threads`): threads recorded on a
  round face rather than modelled, drawn as the annotations of the step
  that recorded them (`draw_threads`, see
  [geop-core-editor](../core/geop-core-editor.md#the-view-view)). A step that repeats features — a
  pattern, a mirror — copies the threads they carry.
- **`Records<T>`**: what a part records of some of its entities by name —
  the cable a solid is swept from, the sheet a sheet-metal body is bent
  from — as the value of a target the steps that make them revise.

The model draws itself into a `PartView` (`NamedModel::draw`): its
vertices, edges and faces, each by name, from its rasterization (see
[geop-core-rasterize](../core/geop-core-rasterize.md)).

`testing` (feature `testing`) holds what the tests of the operations share
about the model: checking it valid (`assert_valid`, `check_valid`), finding
its entities by where they are (`vertex_at`, `edge_at`, `face_towards`),
volumes, and building a part of a given model and one step
(`given_part`, `apply`).
