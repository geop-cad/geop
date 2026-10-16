# geop-lib-sweep

> Brief overview only — full documentation is coming later.

The sweep algorithms: extrude, revolve, screw, sweep along a path and
loft, and the skinned grid all of them (and a blend's tool) are built as;
plus basic shapes (cube, sphere, cylinder, ...) for tests to be written
against.

An algorithm here takes geometry — `Profile`s drawn in a plane's
coordinates, `Chain`s of 3-D curves, planes and axes — and builds into a
`NamedModel`. Resolving a sketch, an axis or a picked edge into that
geometry, and combining what is built with a solid, is the operations' part
(see [geop-ops-extrude-revolve](../ops/geop-ops-extrude-revolve.md)); the
sketch's areas become profiles by `geop_ops_sketch::shape_profiles`.

All are *sweeps* (`sweep`): a planar profile carried along a `Path` —
straight for an extrude, around an axis for a revolve, along a chain of
curves for a sweep — or a loft's several profiles skinned together, into a
solid, or, without one, into a sheet of faces standing on their own. A sweep is
described whole, as a `BodySpec` of vertices, edges and faces, and built in
one go by `NamedModel::build_body` (see [geop-ops-model](../ops/geop-ops-model.md) and
[geop-core-topology](../core/geop-core-topology.md)),
which checks it is consistent first and names every entity it builds.

## Profiles

A `Profile` is a chain of 2-D NURBS curves with a name for every curve and
every joint. The swept faces, edges and vertices are named after them: for a
sketch these are its element ids, for a shape built in code their positions
in the chain (`Profile::closed`). A `SweepLoop` is a profile to sweep, and
says which of its joints lie on a revolve's axis (*poles*) and which of its
curves lie along it.

`polygon`, `polyline` and `circle_at` make the simplest profiles. The
curves are the kernel's own (exact rational quarter arcs, `sqrt2_over_2`;
a curve is placed on a plane in 3-D by `NurbCurve::embed`, see
geop-core-geometry).

## Sweep

A sweep is a grid. Every profile joint sits at every *station* of the path
(a vertex), every curve lies at every station (an edge), every joint travels
along every *span* between two stations (a lateral edge), and every curve
sweeps a *wall* through every span. A solid swept along an open path is
closed by two flat caps, `N(start)` and `N(end)`, spanning the profile's
bounding box; along a closed path — a full turn — it needs none. A wall is
the tensor product of its span (`u`: degree 1 for a line, a rational
quadratic for an arc, a cubic B-spline through approximating sections for
any other curve) and its curve (`v`).

`skin` is the same grid with a section of its own at every station — what
a loft passes through: the `i`-th curves of all sections compatible (one
degree, one knot vector, see `NurbCurve::compatible`), a wall running from
one section's curve to the next one's, everything named after the first
section.

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

## Sweep along a path

`sweep_along` carries a profile, from where it is drawn, along a
`Chain` (`geop_ops_model::Chain`) of 3-D curves by the rigid motion that keeps it square to the
path — a rotation-minimizing frame, which for a planar path keeps the
path's plane normal fixed. Every joint of the path is a station, every curve
a span: a line moves the profile straight on and an arc turns it about the
arc's axis, both exactly; any other curve is approximated by sections at
samples along it — frames carried by the double reflection method —
skinned by cubic Hermite interpolation of the frames. Where two lines meet
at an angle the profile is mitred, projected onto the plane bisecting the
corner; any other corner is refused. Stations are named after the path's
joints, spans after its curves.

## Loft

`loft` skins two or more `Section`s — profiles in planes of their
own — with ruled walls between consecutive ones. The sections are made to
correspond first: each turned to wind the same way seen along the loft
(mirroring its frame where its plane faces the other way), a section of
fewer curves has its longest ones halved, a closed section starts at the
joint lining it up best with the one before, and the curves are made
compatible. Closed loops loft into a solid capped by the first and last
sections; open chains into sheets — between two curves, the ruled surface
joining them.

## Basic shapes

The `shapes` module builds basic solids directly. It is only compiled for
tests: this crate's own, and other crates' through the `testing` feature,
which their dev-dependencies turn on. The rasterizer's tests of curved and
swept solids live here too, with the shapes they draw.

| Function                          | Shape                                                    |
| --------------------------------- | -------------------------------------------------------- |
| `cube_solid`                      | an axis-aligned box, extruded from its top face          |
| `sphere_solid`                    | an exact sphere of 8 rational octant faces               |
| `revolved_cylinder`, `revolved_cylinder_along_axis` | an exact cylinder, revolved around z, x or y |
| `extruded_cylinder`               | an `n`-gon prism approximating a cylinder                |
| `figure8_profile`                 | a dumbbell outline with two holes, extruded              |
