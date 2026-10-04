# geop-ops-extrude-revolve

> Brief overview only — full documentation is coming later.

Extrude and revolve operations, plus basic shapes (cube, sphere,
cylinder, ...) for tests to be written against.

Both are *sweeps* (`sweep`): a planar profile carried along a `Path` —
straight for an extrude, around an axis for a revolve — into a solid, or,
without one, into a sheet of faces standing on their own. A sweep is
described whole, as a `BodySpec` of vertices, edges and faces, and built in
one go by `Part::build_body` (see [geop-core-topology](../core/geop-core-topology.md)),
which checks it is consistent first and names every entity it builds.

## Profiles

A `Profile` is a chain of 2-D NURBS curves with a name for every curve and
every joint. The swept faces, edges and vertices are named after them: for a
sketch these are its element ids, for a shape built in code their positions
in the chain (`Profile::closed`). A `SweepLoop` is a profile to sweep, and
says which of its joints lie on a revolve's axis (*poles*) and which of its
curves lie along it.

`common` holds the helpers the constructors share: `line2`/`line3`,
`polygon` and `polyline` profiles, `bilinear` patches, exact rational
quarter arcs (`sqrt2_over_2`), and `embed_curve`, which places a 2-D curve
on a plane in 3-D exactly, as an affine map of its control points.

## Sweep

A sweep is a grid. Every profile joint sits at every *station* of the path
(a vertex), every curve lies at every station (an edge), every joint travels
along every *span* between two stations (a lateral edge), and every curve
sweeps a *wall* through every span. A solid swept along an open path is
closed by two flat caps, `N(start)` and `N(end)`, spanning the profile's
bounding box; along a closed path — a full turn — it needs none. A wall is
the tensor product of its span (`u`: degree 1 for a line, a rational
quadratic for an arc) and its curve (`v`).

A revolve's axis makes two things degenerate: a pole is a single vertex at
every station, the walls next to it closing over it with a degenerate
coedge, and a curve along the axis sweeps no wall — with caps, it is the
edge both caps share.

The walls point outwards when the profile runs counter-clockwise in its
stations' `(e1, e2)` and the path runs against `e1 × e2`, or clockwise and
along it; `extrude` and `revolve` orient the loops they are given — the
outer loop counter-clockwise, holes clockwise — accordingly.

| Entity                                  | Name                                   |
| --------------------------------------- | -------------------------------------- |
| wall swept by curve `X` through span `q` | `N(X,q)`, or `N(X)` for an extrude    |
| curve `X` at station `s`                | `N(X,s)`; `N(X)` on the axis            |
| lateral edge swept by joint `P`         | `N(P,q)`, or `N(P)` for an extrude      |
| vertex of `P` at station `s`            | `N(P,s)`; `N(P)` for a pole             |
| caps                                    | `N(start)` / `N(end)`                   |

The solid's shells are its connected sets of faces: a full turn of a region
with a hole sweeps the hole into a void, a shell of its own.

## Extrude

`extrude` sweeps loops from one height to another along a plane's normal:
stations `start` and `end`, one span between them.

## Revolve

`revolve` sweeps `(r, z)` loops from one angle to another, either way round
and up to a full turn, around an axis. A partial turn is cut into equal spans
of at most a quarter turn, its stations `a0, a1, ...` and its spans `q0,
q1, ...`. A full turn always has four spans, run backwards from `a0` — `q3`
first — so that `a1` lies a quarter turn on from `a0`. `revolve_at_oriented`
revolves a "top-down" profile a full turn the way the basic shapes are
built: an open chain from pole to pole, or a ring clear of the axis.

## Basic shapes

The `shapes` module builds basic solids directly. It is only compiled for
tests: this crate's own, and other crates' through the `test-shapes`
feature, which their dev-dependencies turn on.

| Function                          | Shape                                                    |
| --------------------------------- | -------------------------------------------------------- |
| `cube_solid`                      | an axis-aligned box, extruded from its top face          |
| `sphere_solid`                    | an exact sphere of 8 rational octant faces               |
| `revolved_cylinder`, `revolved_cylinder_along_axis` | an exact cylinder, revolved around z, x or y |
| `extruded_cylinder`               | an `n`-gon prism approximating a cylinder                |
| `figure8_profile`                 | a dumbbell outline with two holes, extruded              |

## The operations

`Extrude` and `Revolve` are the operations a program uses (see
[geop-ops](./geop-ops.md#operations)): they sweep a sketch of the part into
a solid, and name everything after the sketch's elements. A sketch is one
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
with `boolean_up_to_next` (see [geop-ops-booleans](./geop-ops-booleans.md)):
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
take a `Combine` from [geop-ops-booleans](./geop-ops-booleans.md): keep the
result as a `NewBody`, or immediately unite, intersect or subtract it with a
`target` solid — which is why this crate depends on the booleans, not the
other way round.
