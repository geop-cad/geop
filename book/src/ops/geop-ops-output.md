# geop-ops-output

> Brief overview only — full documentation is coming later.

What is made *of* a part rather than in it: files written from what a
program built (`Exporter`), and questions answered about it (`Inspector`).
Neither is a step: exporting and inspecting change no program and are
nothing to undo. A crate that writes a format, or answers a question,
implements one of the traits on a unit struct, as an operation does; an
editor offers those registered in its `Outputs`, by their infos, and knows
nothing more of them. The options each takes are its own, sent as JSON.

## What they are given (`Output`)

An `Output` is what a program built, as an exporter or an inspector is
given it: the runner holding the part as far as the program runs, the parts
it places included (`part()`), the part before a step (`part_before`, what a
drawing step is of), the program file it ran, with its state — to ask how
its parts can move by running it again — what the program says of its part
(`about()`: its file, material, title and designation, see
[geop-ops-parameters](./geop-ops-parameters.md#what-a-run-is-given-about)),
the step being edited and the date.

## Exporters

An `Exporter` is a kind of file written from what a program built. Its
`INFO` (`ExporterInfo`) gives its kind, label, a few words of what the file
is for, and the formats it writes by extension, the default first; its own
`Options` are deserialized from the JSON a front end sends (`Default` where
it sends none; `NoOptions` for one that takes none); and
`export(output, format, options)` returns an `Export` — a name, a media
type, and text or bytes (`Content`). A step's dialog offers a file of its
own with `Dialog::download(key, exporter, formats)`.

The STL exporter (`geop_ops_assembly::Stl`) writes the mesh the editor
draws; others write STEP files, drawings, URDF robots, flat patterns and
bills of materials, each in its own crate.

## Inspectors

An `Inspector` is a question answered about what a program built. Its
`INFO` (`InspectorInfo`) says whether it is asked of the whole part or of
picks (`Picks`: the drawn kinds it accepts, at most how many);
`inspect(output, picked, options)` returns its `Report` (any `Serialize`)
and the visuals to draw of it in the viewport, and `show(report)` lays the
report out for a front end as an `Answer` — text, lists, tables, groups,
buttons that ask again or save a file (`Block`, `AnswerAction`), and a
plane to cut a section with — so a front end shows every inspector's answer
alike, knowing none. An inspector that picks becomes the editor's pick
tool, asked again with every pick. `Bounded` is a number as a report shows
it: the midpoint of an enclosure and its half-width.

## Registering them

Exporters and inspectors are registered in an `Outputs` (`exporter`,
`inspector`; a kind twice is refused), the application's choice like the
operation registry: the editor's is `outputs` in
[geop-cad-base](../cad/geop-cad-base.md).

## Sheets

Files of paper — drawings, flat patterns — are laid out on a
`sheet::Sheet`: plain 2-D strokes and labels in millimetres with `y` up,
each on a `Layer` defined by who draws on it, and written in a `Format`,
SVG (`to_svg`) or DXF (`to_dxf`). The kernel's curves become lines, arcs
and circles where they are those exactly, and polylines through points of
them otherwise.
