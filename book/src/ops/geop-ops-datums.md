# geop-ops-datums

> Brief overview only — full documentation is coming later.

Reference geometry as an operation (see [geop-ops](./geop-ops.md#operations)).

`AddDatum` builds reference geometry from a selection in one of
the ways CAD systems commonly offer (`Construction`):

- **points:** offset point, midpoint, point on edge, center, projection onto
  a plane or a line, line meets plane, lines meet, three planes meet;
- **axes:** through two points, along a line, axis of an arc or cylinder,
  two planes meet, perpendicular to a plane or a line, parallel through a
  point, angle bisector, tangent to an edge;
- **planes:** offset, midplane, through three points, at an angle, through a
  line and a point, through two lines, parallel through a point, normal to a
  line or an edge;
- **coordinate systems:** through three points — at the first, x towards
  the second, the xy plane through the third.

A coordinate system is used as a point, its origin, and by its axes, like
the origin itself: a point offset from it goes along them.

What a selected entity can be used as — its `Geometry`: a point, a line, a
plane, an arc, something round — is decided by its shape, not its kind,
using `geop-core-geometry`'s shape recognition: a straight edge is a line, a
circular one has a center and an axis, a flat face is a plane.

`inspect_selection` says which constructions fit a selection, using
exactly the matching a step applies. `AddDatum`'s dialog is built on it:
the selection is picked in the viewport — anything but solids and
sketches — each entity listed with what it can be used as; only the
constructions that fit can be chosen, and a construction that stops fitting
gives way to the first that does. Offsets are handles: an offset plane's
distance along its normal, an offset point's `x`, `y`, `z` along its axes.
