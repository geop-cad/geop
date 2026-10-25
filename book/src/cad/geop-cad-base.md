# geop-cad-base

> Brief overview only — full documentation is coming later.

The CAD application, composed: what its editor offers, one line per crate,
an editor of all of it, and the workspace its programs place parts from
beside the [standard parts](../lib/geop-lib-stdlib.md). The editor itself is
[geop-core-editor](../core/geop-core-editor.md#the-editor)'s; the operations
and their examples are their crates'. It is three files: `lib.rs`,
`registry.rs` and `tests.rs`.

## The registry

`registry()` is the
[`Registry`](../core/geop-core-editor.md#the-registry-registry) the editor
offers: every crate's `register`, one line each — sketches, datums,
extrude, revolve, sweep and loft, holes and threads, fillets and chamfers,
shells and plastic features, booleans and splits, patterns, mirrors and
moves of bodies, surfaces, subdivision surfaces, placed parts, harnesses,
STEP import, drawings, URDF robots, sheet metal, inspecting, bills of
materials and the part's parameters — each with its examples. Within a group and
tier the toolbar offers operations in the order they are registered;
exporters and inspectors come in that order too, and the first exporter's
first format, STL, is what a part is written in unless told otherwise. A
part is drawn and listed by the drawers in that order: solids and faces,
datums, sketches, the part's colour, placed parts.

This and the composition's tests are the only places the crate names an
operation crate (`scripts/check-layering.sh` checks it). `editor()` is an
editor of all of it,
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

Only what needs every operation registered is tested here, in
`tests.rs`: every operation offered once, started from what the steps
before built and opened on any part (`set`); the editor of everything
driven as a front end drives it — commands that edit the program and only
what changed sent back, every operation offered group by group, every
example loaded by name, a program of unknown steps refused (`editor`);
every example built incrementally (`incremental`); and every example and
example assembly drawn in every view, weighed and halved (`examples`,
the slow ones ignored). A test of one operation or one example lives in
that operation's crate: its `program_tests.rs` and `editor_tests.rs` over
a registry of what they need (or `tests/` where they need a crate that
depends on it), with the helpers of `geop_core_editor::testing`.

## Examples

Each operation crate ships the examples that exercise it in its own
`examples.rs`, registered by its `register` when its feature `examples`
is on — which this crate asks for, since an example is written in other
operations' `Args` too and an operation itself depends on none (extrude
and revolve's
`box_with_drill_hole`, `bracket`, `pipe`, `parametric_plate`, ...;
`split_plate` of [geop-ops-boolean](../ops/geop-ops-boolean.md);
`rounded_box` of `geop-ops-fillet`; `horn` and `motor_flange` of extrude
and revolve too; the datum planes' `boss_on_reference_plane` and
`airfoil_wing` of
[geop-ops-reference-geometry](../ops/geop-ops-reference-geometry.md);
`patterned_plate` of `geop-ops-pattern`, `hole_plate` of `geop-ops-hole`,
`sheet_metal_bracket` of `geop-ops-sheetmetal`, `subd_mouse` of
`geop-ops-subd`, `curved_panel` of `geop-ops-surface`; the assemblies and
every example made of several files of
[geop-ops-assembly](../ops/geop-ops-assembly.md)). The examples drawn in
the editor are kept as the `.geop` text they saved, in the `src/drawn/` of
the crate whose operations they use (`edge_part` of extrude and revolve,
`conebox` of reference geometry, `turbine_blade` and `cool_flange` of
patterns). An editor lists them all through the registry
(`Registry::example_programs`); they serve as a reference for writing new
programs.

