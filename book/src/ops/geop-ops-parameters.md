# geop-ops-parameters

> Brief overview only — full documentation is coming later.

What a part says of itself besides its shape: the parameters its design
is given by and its properties — title, designation, colour, material —
as steps of its program defining targets, which a program placing the part
sets through the run's inputs; and where the parts of its instances are
placed. A program, its file, its state and the values it is run with are
[geop-core-program](../core/geop-core-program.md)'s.

## A program's parameters

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
properties, as an editor lists them together (`Parameters`), in their two
steps; `parameter_numbers` gives what its parameters come to as defined,
`parameter_uses` what reads each, and `rename_parameter` renames a
parameter and every formula reading it.

## Parameters

Parameters are the named values a program's design is given by, defined
by a step of the program (`DefineParameters`, the step `parameters`) as a
target each (`parameter:<name>`), and read by name wherever a number is
typed (a [`NumericInput`](../core/geop-core-program.md#numbers-as-formulas)):
a sketch dimension of `width / 2`.

- **Numbers**, each a formula of the others, defined before it or after —
  `4`, `width / 2`, `sqrt(a^2 + b^2)`. A `min` and `max` say what a slider
  offers when the part is placed.
- **Tables**: a family of variants — screw sizes, say — one row each, of
  which one is selected. `screw` is the selected row's name, and
  `screw.clearance` its value in the column `clearance`.
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

## Properties

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

## Where instances are placed (`instance`)

A step that loads the part of an instance (see
[geop-core-program](../core/geop-core-program.md#instances)) says where it
goes with a target of its own (`define_instance`, an `Instance`): a
`Placement` — where a pose state of the part puts it, or, for a copy, a
fixed motion after another instance — and the file and inputs its part
was loaded from, which a copy loads again. `instance` reads one, `instances` every one of the parts
the part has loaded (`Reader::subparts`) — each loaded part must be
placed — and `placement`
gives where an instance is, however deep, as `Factor`s outermost first:
the values of the state that place it and the instances it is in, read
and recorded, and the fixed motions of copies. `compose` composes them
into a pose, and `located` gives the pose of the instance at a path.
