# geop-lib-shell

> Brief overview only — full documentation is coming later.

Shelling, built directly rather than cut out by a boolean: `shell` hollows
a solid of a `NamedModel` out to walls of one thickness, open where the
given faces were. Every face kept is offset inward, a vertex moves to where
the offsets of its faces meet, an edge to where the offsets of its two
faces meet, and where a face is taken away a rim joins the outer side to
the inner one. The inner side has exactly the solid's topology, so nothing
is intersected.

A sheet is shelled the same way: `thicken` turns it into a solid, a wall
along each free edge, and `offset_faces` copies faces along their normals
into a sheet of their own.

Others that rebuild a solid face by face (a draft, `geop-ops-plastic`) use
its pieces: `extended` gives a surface room around a face, `pcurve` a
coedge's pcurve on a moved surface, and `surface_missed` checks that a
curve built for an edge lies on its faces.

What it takes are models, faces and solids by id, and a `Namer`; the
`Shell` operation of `geop-ops-shell` resolves a step's references and
calls it.
