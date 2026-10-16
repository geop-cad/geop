# geop-lib-stdlib

> Brief overview only — full documentation is coming later.

The standard parts — ISO screws, nuts, washers, dowel pins, standoffs,
ball bearings, T-slot extrusions, a NEMA 17 motor, involute spur gears and
their rack, GT2 pulleys, shaft collars, flange couplings, MGN linear
guides, hobby servos — one program per family, generated in Rust and
written in the arguments of the operations that build them (sketch,
datums, extrude and revolve, patterns, parameters). A family's
sizes are the rows of its table parameter `size`, its dimensions read from
that row by the formulas of its sketches (`Drawing`), so a program placing
one picks its size like any parameter of a placed part.

Every part turns around, or runs along, the `z` axis, and carries named
datums to mate it by: `axis`, and a plane for the face it sits on
(`StandardPart::base`). The faces a thread goes on are named in
`StandardPart::threaded`.

`WithStandardParts` is a `FileStore` that adds the standard parts beside a
workspace's own files, as read-only files named `std:…`
(`std:iso4032_hex_nut.geop`), which mean the same file from anywhere;
[geop-cad-base](../cad/geop-cad-base.md) wraps the editor's workspace in
it. `parts()` and `part(file)` give the families themselves. Listing them
builds nothing: a family's program is generated the first time its file is
read.
