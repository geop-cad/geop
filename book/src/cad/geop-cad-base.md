# geop-cad-base

> Brief overview only — full documentation is coming later.

The CAD application, composed: what its editor offers, one line per crate,
an editor of all of it, the workspace its programs place parts from
beside the [standard parts](../lib/geop-lib-stdlib.md), and the example
programs. The editor itself is
[geop-core-editor](../core/geop-core-editor.md#the-editor)'s; the operations
are their crates'.

## The registry

`registry()` is the
[`Registry`](../core/geop-core-editor.md#the-registry-registry) the editor
offers: every crate's `register`, one line each — sketches, datums,
extrude, revolve, sweep and loft, holes and threads, fillets and chamfers,
shells and plastic features, booleans and splits, patterns, mirrors and
moves of bodies, surfaces, subdivision surfaces, placed parts, harnesses,
STEP import, drawings, URDF robots, sheet metal, inspecting, bills of
materials and the part's parameters — and the examples. Within a group and
tier the toolbar offers operations in the order they are registered;
exporters and inspectors come in that order too, and the first exporter's
first format, STL, is what a part is written in unless told otherwise. A
part is drawn and listed by the drawers in that order: solids and faces,
sketches, datums, placed parts.

This is the only place the crate names an operation crate, besides the
examples and the standard parts. `editor()` is an editor of all of it,
the standard parts beside the files it is sent; `outputs()` the exporters
and inspectors alone, as the command line picks them by name.

A [`Program`](../core/geop-core-program.md#operations-and-programs) is what
the web editor edits and the command-line tool compiles, run by a `Runner`
of these operations (`runner`). `build(program, file, workspace)` builds a
program outside the editor, as the file `file` of the workspace it places
parts from (`workspace`, beside the standard parts).

## The workspace

`Workspace` is the editor's workspace: the core's file-backed
[`Files`](../core/geop-core-program.md#the-workspace) of a set of program
files, the parts they build, by path, and the other files their steps
read, with the standard parts beside them. Files name each other by
`/`-separated paths relative to the file doing the naming —
`../parts/bolt.geop` — so a folder of files can be moved as a whole. Each
file is read once, until it changes: a program is the same `Rc` while its
file does not change, so a part placed from it again carries over what it
built. Every part placed is a part of its own, kept by the part placing it
(sharing the parts that read no state between their instances is a TODO).
A write changes its revision, and what read its files is read again. The
runner refuses a placement that would make files place each other in a
cycle, naming the cycle: the files must form a DAG.
Its files are wrapped in
[`WithStandardParts`](../lib/geop-lib-stdlib.md), which adds the standard
parts beside them as read-only files named `std:…`
(`std:iso4032_hex_nut.geop`), which mean the same file from anywhere.

## Tests

The tests of programs written in many operations, and of the editor driven
over all of them as a front end drives it, are here for now
(`operations/*tests*.rs`, their helpers in `registry::testing`); each moves
to the crate it tests.

## Examples

Each operation crate ships the examples that exercise it in its own
`examples.rs`, registered by its `register` (extrude and revolve's
`box_with_drill_hole`, `bracket`, `pipe`, `parametric_plate`, ...;
`split_plate` of [geop-ops-boolean](../ops/geop-ops-boolean.md);
`rounded_box` of `geop-ops-fillet`). `examples` holds the rest for now:
programs written in Rust (`patterned_plate`, `hole_plate`, ...) and
workspaces of several files (`workspaces`), offered as one `ExampleSet`
(`examples::set`). An editor lists them all through the registry
(`Registry::example_programs`); they serve as a reference for writing new
programs.

