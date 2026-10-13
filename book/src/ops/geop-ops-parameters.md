# geop-ops-parameters

> Brief overview only — full documentation is coming later.

What a part is given rather than built: the parameters its design is given
by, resolved to the numbers a run reads as its inputs; what its program
says of it besides — its file, colour, material, title, the rows of its
tables — and the files around it, as targets the run's given step defines;
its state; and the program as a file holds all of that. It also names the
types every operation of a part is written against: `BuildPart` and
`PartReader` (the [geop-core-program](../core/geop-core-program.md) part
and reader of a part's inputs and state), `StepContext` (what a step is
edited against) and `Registry`.

## The program file (`ProgramFile`)

A `ProgramFile` is a program as a `.geop` file holds it:
`{"parameters": {...}, "steps": [...], "state": {...}}` — its parameters,
its steps (a [`Program`](../core/geop-core-program.md#operations-and-programs))
and its state. `push(id, args)` appends a step for a program written in
Rust, whose arguments name their operation:

```rust,ignore
let mut program = ProgramFile::new();
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

`run(runner, overrides)` runs it on a `Runner` (the
[`ProgramRunner`](../core/geop-core-program.md#running-a-program)
of a part's inputs and state), with `overrides` taking the place of its
parameters' own values, and fails naming the first step that does.
`to_json` pretty-prints it, so edits show up as small line diffs.
`excerpt` and `paste` copy steps between programs, `rename_parameter`
renames a parameter and every formula reading it, and `validate` checks a
program read from a file.

## Parameters

`Parameters` are the named values a program's design is given by, defined
once in the program and read by name wherever a number is typed (a
[`NumericInput`](../core/geop-core-editor.md#numbers-as-formulas)):
a sketch dimension of `width / 2`.

- **Numbers**, each a formula of the others, defined before it or after —
  `4`, `width / 2`, `sqrt(a^2 + b^2)`. A `min` and `max` say what a slider
  offers when the part is placed.
- **Tables**: a family of variants — screw sizes, say — one row each, of
  which one is selected. `screw` is the selected row's name, and
  `screw.clearance` its value in the column `clearance`.
- **The part's colour**, `#rrggbb`, set as `color`; its **material**
  (`Material`, a name and a density in kg/m³, picked from a list or given
  by its density); its **title** and **designation**, what a bill of
  materials lists it as (`Parameters::designate`: `ISO 4762 M4x12`).

What a parameter is defined as is the program's own; what it is *built
with* may be overridden by a program placing the part, by the same name, as
`Settings` (see [geop-ops-assembly](./geop-ops-assembly.md#parameters)). So
a placed screw is made M5 by the program placing it, without touching the
screw's file. `Parameters::resolve` turns the definitions and the
overrides into what a run is given: the numbers, its inputs
(`ProgramFile::inputs`), and the rows and the colour chosen. A parameter
that does not resolve is left out — what reads it fails, saying so — and
the editor shows why.

Since each formula is resolved where a step builds, and that read is
recorded, a run after a parameter changed rebuilds only from the steps that
read it, and nothing for a change no step read, like the part's colour.

## What a run is given (`about`)

The given step of a run (see
[geop-core-program](../core/geop-core-program.md#running-a-program))
defines two targets from here:

- `about()`, an `About`: the program's file, its parameters and what was
  chosen of them — what a step or an exporter reads of the part's colour,
  material (`MaterialOf`), title and designation (`read_about`).
- `files()`, the `Files` around the program, which a step that reads a
  file beside programs — a STEP file to import — reads (`read_files`).

## The state (`state`)

A part's state holds `StateValue`s, by name: a number — a joint's
coordinate — or a `Pose` — where a placed part is, `<instance>.pose`
(`pose_state`). It is kept with the program (`ProgramFile::state`) between
runs, read by the steps like an input, and moved by a solve (see
[geop-ops-constraints](./geop-ops-constraints.md)): a number steps by what
is added to it, a pose by a twist in its own frame through the exponential
map. A value the run gives none of reads as the number zero.
