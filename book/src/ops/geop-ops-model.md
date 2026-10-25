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
axis is a `Line` (see [Datums](#datums-datums)), a sketch
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

## Datums (`datums`)

A part's datums: reference geometry it is built *with*, not *of* — a point,
an axis, a plane or a coordinate system, by name (each a `Datum` of
[geop-core-math](../core/geop-core-math.md#primitives), carrying a
full right-handed frame, so anything built on it has axes to be built
along). The operation that adds one is
[geop-ops-reference-geometry](./geop-ops-reference-geometry.md)'s; this crate holds only the
target (`geop_ops_model::datums`), so every operation that builds on a datum
depends on it alone.

### The target

All of a part's datums are one target, `datums()`, a `Datums` that the
steps adding datums revise, each adding its own by its step's id. The given
step of a run defines it (`define_datums`), so `ORIGIN` — the coordinate
system `origin`, the world's origin and axes, which no step adds — is there
before any step. A datum stays where it was when its step ran, whatever
later steps do to what it was built from. `read_datums(&reader)?.get(name)`
reads one; `Datums::draw` draws them all into a
[`PartView`](../core/geop-core-editor.md#the-view-view), by the sizes the viewer draws them at.

### Elements

The target is a singleton, so a reference into it is saved as the datum's
name alone — `"origin:z"`. A datum is an element of the target as a whole
(`AnyDatum`), and as what it is, the geometric kinds of
[the model's](#elements-and-what-they-are-used-as):
a point datum a `Point`, an axis a `Line`, a plane a `Plane`. A frame is a
point too, its origin, and its components are elements of their own, named
after it (`component_name`): `origin:z` its `z` axis, `origin:xy` the plane
normal to it. So a frame's axes and planes are each usable on their own,
and an operation that takes a plane from a face or a datum takes the
origin's planes the same way.

`datum::<K>(name)` makes a reference; `origin_plane(axis)` and
`origin_axis(axis)` are the origin's, typed as
`ElementReference<Datums, Plane>` and `ElementReference<Datums, Line>` —
what a sketch on the `xy` plane or a revolve about `z` is given.

## Parameters and properties (`parameters`)

What a part says of itself besides its shape: the parameters its design
is given by and its properties — title, designation, colour, material —
as steps of its program defining targets, which a program placing the part
sets through the run's inputs; and where the parts of its instances are
placed. A program, its file, its state and the values it is run with are
[geop-core-program](../core/geop-core-program.md)'s.

### A program's parameters

A [`Program`](../core/geop-core-program.md#operations-and-programs) holds
its parameters and properties in two of its steps. `push(id, args)`
appends a step for a program written in Rust, whose arguments name their
operation:

```rust,ignore
let mut program = Program::new();
program.push("outline", AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline)));
program.push("box", ExtrudeArgs {
    sketch: whole_sketch("outline"),
    extent: Extents::blind(1.0),
    output: Output::Solid(Combine::NewBody),
});
program.push("hole_sketch", AddSketchArgs::on(on_face("extrude(box,end)"), solved(hole)));
program.push("hole", ExtrudeArgs {
    sketch: whole_sketch("hole_sketch"),
    extent: Extents::blind(-0.5),
    output: Output::Solid(Combine::cut("extrude(box)")),
});
```

`parameters_of` and `set_parameters` read and write its parameters and
properties, as an editor lists them together (`Parameters`), in their
steps — the properties, the numbers, and a lookup step per table, before
the numbers whose formulas read its columns; `parameter_numbers` gives
what its parameters come to as defined,
`parameter_uses` what reads each, and `rename_parameter` renames a
parameter and every formula reading it.

An editor shows them in the parameters panel (`ParametersPanel`, key
`parameters`, see
[the registry](../core/geop-core-editor.md#the-registry-registry)): what
they are defined as and come to, why those that do not resolve fail, what
reads each and the materials a part's can be; it takes
`set_parameters` (allowed while a step is edited, which then sees them)
and `rename_parameter`. `PartColor` draws the part in its colour.

### Parameters

Parameters are the named values a program's design is given by, defined
by steps of the program as a target each (`parameter:<name>`), and read by
name wherever a number is typed (a
[`NumericInput`](../core/geop-core-program.md#numbers-as-formulas)): a
sketch dimension of `width / 2`. Every step defining parameters adds them
to the part's `ParameterValues`, a chain.

- **Numbers** (`DefineParameters`, the step `parameters`, of
  `NumberParameter`s), each a formula of the others, defined before it or
  after, and of the tables' columns — `4`, `width / 2`,
  `sqrt(a^2 + b^2)`. A `min` and `max` say what a slider offers when the
  part is placed.
- **Tables** (`LookupTable`, a step of its own per table): a part family —
  screw sizes, say — one row each. The row built is the one the run's
  text input of the table's name names (`size = "M4"`, from a program
  placing the part), else the table's own selection; a text naming no row
  fails, naming the input and the rows. `screw` is the row's name, and
  `screw.clearance` its value in the column `clearance`, each a parameter
  of its own.
What a parameter is defined as is the program's own; what it is *built
with* may be set by a program placing the part, by the same name: the run's
input of that name, a `Setting` (see
[geop-ops-assembly](./geop-ops-assembly.md#parameters)). So a placed screw
is made M5 by the program placing it, without touching the screw's file. A
parameter that does not come to anything fails — what reads it fails,
saying so — and the editor shows why. `read_parameter_values` (of
geop-core-program, which knows the target) lists what they all came to,
in order.

Since each formula is resolved where a step builds, and that read is
recorded, a run after a parameter changed rebuilds only from the steps that
read it.

### Properties

A step `PartProperties` (the step `properties`) defines what the part is
besides its shape (`Properties`, `read_properties`): its **title** and
**designation** — what a bill of materials lists it as, the designation
followed by its parameters' values (`ISO 4762 M4x12`) — its **colour**,
`#rrggbb`, which a program placing it may set by the input `color`, and
its **material** (`Material`, a name and a density in kg/m³, picked from a
list or given by its density; `MaterialOf` says what it is weighed as).
What a part is listed as without a title is the name of the file it is
placed from (`stem`).

`file_choices` lists what a field picking a file offers: the files around
the program, through the runner running it.

### Where instances are placed (`instance`)

A step that places a part (see
[geop-core-program](../core/geop-core-program.md#placed-parts)) says where
it goes with a target of its own (`define_instance`, an `Instance`): a
`Placement` — where a pose state of the part puts it, or, for a copy, a
fixed motion after another instance — and the file and inputs its part
was placed from, which a copy places again. `instance` reads one,
`instances` every one of the parts placed in the part
(`Reader::instances`) — each must be defined — and `placement` gives
where an instance is, however deep, as `Factor`s outermost first: the
values of the state that place it and the instances it is in, each read
by its path in the part asked (`/bolt:nut.pose`) and recorded, and the
fixed motions of copies. `compose` composes them into a pose, and
`located` gives the pose of the instance at a path.

## Sketches (`sketch`)

The sketch target: `sketch(name)` is a `PlacedSketch` — the sketch, drawn
with [geop-core-sketch](../core/geop-core-sketch.md), and the plane it lies
on — which is what later steps build on: read whole (`whole_sketch`, what
an extrude sweeps) or by its elements (`sketch_element`), which have the
geometric kinds [above](#elements-and-what-they-are-used-as) where they
have that shape — a sketch's line is a `Line`, its plane a `Plane`, its one
chain or loop a `Path`. `SketchPlane` is where a sketch is placed: a
planar face, a datum plane — the planes of the origin among them — or
another sketch's plane. The step that draws a sketch is
[geop-ops-sketch](./geop-ops-sketch.md)'s.

### Figures

`figures` draws what examples and tests write down by numbers: a closed
`polygon`, a fully constrained `rectangle` and `circle`, every number
design data (`n`), and a sketch `solved` once it is drawn. Every crate's
examples draw with it, rather than each keeping its own copy.
