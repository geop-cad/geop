# geop-ops-extrude-revolve

> Brief overview only — full documentation is coming later.

Extrude, revolve, sweep and loft operations. They resolve what their
arguments refer to — the sketch's areas as profiles
(`geop_ops_sketch::shape_profiles`), the axis, the path — and sweep them
with [geop-lib-sweep](../lib/geop-lib-sweep.md), which describes the sweeps
and how what they build is named. With the `testing` feature, `testing`
draws the sketches and pushes the extrusions other crates' program tests
start from.

## The operations

`Sweep` sweeps one sketch's area along another sketch's one chain or loop
of curves, starting at the end (or joint) of the path nearest the profile;
`Loft` lofts through several sketches' single loops, in order. Both keep a
new body or take a `Combine`, and as a `face` build faces standing on
their own.

`Extrude` and `Revolve` are the operations a program uses (see
[geop-core-editor](../core/geop-core-editor.md#an-operation-partoperation)):
they sweep a sketch of the part into a solid, and name everything after the sketch's elements. A sketch is one
area to sweep — an outer loop and its holes; a sketch of several separate
areas is refused.

How far they go is an `Extents`: the first side a length — an angle, for
a revolve — `UpToNext` or `ThroughAll`, along the plane's normal or, if
`reversed`, against it; `symmetric`, the same the other way (half a length
each way); or else an optional second side on its own terms. Up to next and
through all go as far as the target solid. Through all is just a length
reaching past all of it, so sides without up to next build one sweep between
their ends. A side going up to the next face is built on its own from the
sketch's plane, long enough to reach past the target — for a revolve, short
of a full turn, which needs the target clear of the axis — and combined
with `boolean_up_to_next` (see [geop-lib-booleans](../lib/geop-lib-booleans.md)):
a join grows up to the target, a cut or an intersection takes the first
stretch of the target the profile passes through, and a new body — which
goes up to the next face of any solid of the part — is that piece alone,
the solids left as they are. If the target stops only part of the profile
and the rest goes on past, the side goes exactly as far as the profile
first meets the target (the nearest corner of where the two overlap) — or,
for a cut or an intersection meeting nothing up to there, on to where it
next meets it. The second side built so is named within `side2`.

As a `face`, they sweep the sketch's curves — its area's outline, or one
open chain of curves enclosing nothing — into faces standing on their own,
part of no solid: what `Split` cuts a solid with. A face going up to the
next face of a solid picked for it is trimmed where it meets it
(`trim_up_to_next`), leaving the solid as it is.

Editing one, the sketch is picked in the viewport and the extrude's lengths
are handles. A revolve's axis is any line in the sketch's plane — a line of
the sketch, a datum axis, a frame's axis, a straight edge: the area,
holes and all, lies on one side of it, and touches it only where the
constraints put it on it, which needs the axis to be a line of the sketch
itself. Dragging or typing the first length across the sketch plane turns a
join into a cut and back; on either side, how to combine stays what was
chosen — an extrude may cut upwards into a solid above its sketch. Both
take a `Combine` from [geop-ops-model](./geop-ops-model.md): keep the
result as a `NewBody`, or immediately unite, intersect or subtract it with a
`target` solid — which is why this crate depends on the booleans, not the
other way round.
