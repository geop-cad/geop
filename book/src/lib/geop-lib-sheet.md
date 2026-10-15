# geop-lib-sheet

> Brief overview only — full documentation is coming later.

Files of paper — drawings, flat patterns — are laid out on a
`Sheet`: plain 2-D strokes and labels in millimetres with `y` up,
each on a `Layer` defined by who draws on it, and written in a `Format`,
SVG (`to_svg`) or DXF (`to_dxf`). The kernel's curves become lines, arcs
and circles where they are those exactly, and polylines through points of
them otherwise.

Drawings (`geop-ops-drawing`) and flat
patterns (`geop-ops-sheetmetal`) are written on it. `xml_escape` makes text
fit for XML, for SVG sheets and any other XML a writer makes (a URDF robot).
