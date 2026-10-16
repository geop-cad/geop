# geop-ops-reference-geometry

> Brief overview only — full documentation is coming later.

Reference geometry as an operation (see
[geop-core-editor](../core/geop-core-editor.md#an-operation-partoperation)).
`AddDatum` builds a point, an axis, a plane or a coordinate system from
elements picked in the part, in one of the ways CAD systems commonly offer
(`Construction`), and adds it to the part's datums (see
[geop-ops-datums](./geop-ops-datums.md)) under the step's id:

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
the origin itself: a point offset from it goes along them. Like a sketch's
plane, a datum is built when the step runs, from the part before it, and
stays where it was whatever later steps do to what it was built from.

## The selection

The step's selection is a list of `Selected`, a `one_of!` enum (see
[geop-core-editor](../core/geop-core-editor.md#picking-references)) of
anything that can be an input of some construction: a vertex, an edge or a
face of the model, a datum, a sketch's point or curve. What each selected
element can be used as — its `Shape`: a point, a line, a plane, a curve, an
arc, something round, a frame — is decided by its shape, not its kind,
using [geop-core-geometry](../core/geop-core-geometry.md)'s shape
recognition: a straight edge is a line and a curve, a circular one a
circle, a curve and something round, a flat face a plane. Each
construction takes its inputs as `Role`s, one selected element per input,
in any order. Any operation that needs to know "what is this thing" of a
polymorphic selection can follow the same pattern.

`fitting_constructions` says which constructions fit a selection, using
exactly the matching a step applies. `AddDatum`'s dialog is built on it:
the selection is a reference field of several elements, which the editor
lists with what each can be used as; the constructions are actions, only
those that fit enabled, and a construction that stops fitting gives way to
the first that does. Offsets are handles and their values numbers or
formulas: an offset plane's distance along its normal, an offset point's
`x`, `y`, `z` along its axes.
