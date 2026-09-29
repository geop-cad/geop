# geop-cad-base

> Brief overview only — full documentation is coming later.

What an interactive CAD editor needs from the kernel beyond the operations
themselves: which operations it offers, and example programs. (Picking
entities with a ray is `geop_ops::ui::PartView`'s.)

## The editor's operations

`PartOperation` is the set of operations the editor offers (see
[operation sets](../ops/geop-ops.md#operation-sets)): `AddSketch`,
`Extrude`, `Revolve`, `Boolean` and `AddDatum`, each defined by the crate
that builds it. `Program`, `ProgramEdit`, `ProgramRunner` and `Step` here
are the program types of [geop-ops](../ops/geop-ops.md#programs) over that
set — what the web editor edits and the command-line tool compiles.

## Examples

`examples` holds programs written in Rust (`box_with_drill_hole`,
`bracket`, `cross_drilled_shaft`, `two_plates`, `boss_on_reference_plane`,
`handle_with_hole`, `luggage_tag`). The tests use them, and they serve as a
reference for writing new programs.
