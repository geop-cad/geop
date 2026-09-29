# geop-ops-extrude-revolve

> Brief overview only — full documentation is coming later.

Extrude and revolve operations, plus the sample primitive shapes (cube,
sphere, cylinder, torus, ...) built from them.

Every solid here is built entirely from the Euler operators of
[geop-ops](./geop-ops.md) `Part` (`mvfs`, `mve`, `mef`, `mer`,
`replace_face`, ...), so it is valid by construction and every entity gets a
stable name as it is created.

## Profiles

A `Profile` is a chain of 2-D NURBS curves with a name for every curve and
every joint. The swept faces, edges and vertices are named after them: for a
sketch these are its element ids, for a shape built in code their positions
in the chain (`Profile::closed`).

`common` holds the helpers the constructors share: `line2`/`line3`,
`polygon` and `polyline` profiles, `bilinear` patches, exact rational
quarter arcs (`sqrt2_over_2`), and `embed_curve`, which places a 2-D curve
on a plane in 3-D exactly, as an affine map of its control points.

## Extrude

`extrude` sweeps a closed profile — a counter-clockwise outer loop plus any
number of clockwise holes — one unit along `w` of a coordinate system.
Every profile curve must be clamped, have domain `[0, 1]`, and start where
the previous one ends.

The coordinate system must be **left-handed**, because every side wall is
parametrized `(height, profile)` and its normal points outward only under
that combination. With a right-handed basis the solid comes out
inside-out: still structurally valid, but rejected by the orientation
check. `extrude_from_plane` takes a right-handed plane and a signed distance
and arranges this itself.

The construction:

1. `grow_ring` builds the outer ring on a placeholder face whose surface is
   `NurbSurface3D::everything()`, and `mef` splits it off as the bottom cap,
   leaving the same ring, reversed, on the placeholder.
2. Each hole is grown on the bottom cap (`mvr`, then `mve`), and `mer`
   moves its other side onto the placeholder too.
3. `build_side_walls` advances every ring by `w`, closing one side wall per
   curve.
4. `replace_face` gives the placeholder its real top surface, holes
   included.

Everything is named after the profile curve `X` or joint `P` that swept it
(`ExtrudeNames`):

| Entity                              | Name                        |
| ----------------------------------- | --------------------------- |
| side face swept by curve `X`        | `N(X)`                      |
| edge along `X` on the start/end cap | `N(X,start)` / `N(X,end)`   |
| edge swept by joint `P`             | `N(P)`                      |
| vertex at `P` on the start/end cap  | `N(P,start)` / `N(P,end)`   |
| start/end cap                       | `N(start)` / `N(end)`, or `N(start,R)` / `N(end,R)` when one step extrudes several regions |

## Revolve

`revolve_at_oriented` sweeps an open `(r, z)` profile 360° around an axis.
The first and last points must lie on the axis (`r = 0`) and become poles.
Walked from first to last point, the region bounded by the profile and the
axis must lie on the right, or the solid comes out inside-out.

The solid is built one 90° column at a time rather than one row at a time,
and the last column closes back onto the first meridian. That way no
zero-area cap is left behind at the final pole. A leftover placeholder would
be worse than untidy: its `ENTIRE` surface would `could_be_equal` every
point and absorb every containment query aimed at it, and a zero-area face
has no interior point for a boolean to classify.

`revolve_at` revolves around the z-axis through a given origin, and
`revolve` around the z-axis itself. Names follow the quadrants `q0..q3` and
angles `a0..a3`: `N(X,q)` for the face swept by curve `X` through quadrant
`q`, `N(X,a)` for a meridian edge, `N(P)` for a pole.

## Basic shapes

| Function                          | Shape                                                    |
| --------------------------------- | -------------------------------------------------------- |
| `cube_solid`                      | an axis-aligned box, extruded from its top face          |
| `sphere_solid`                    | an exact sphere of 8 rational octant faces               |
| `revolved_cylinder`, `revolved_cylinder_along_axis` | an exact cylinder, revolved around z, x or y |
| `extruded_cylinder`               | an `n`-gon prism approximating a cylinder                |
| `figure8_profile`                 | a dumbbell outline with two holes, extruded              |

The torus (a 4×4 grid of NURBS patches) exists in `torus.rs` but is not
currently compiled.

## The operations

`Extrude` and `Revolve` are the operations a program uses (see
[geop-ops](./geop-ops.md#operations)): they sweep a sketch of the part into
a solid, and name everything after the sketch's elements. Editing one, the
sketch is picked in the viewport and the extrude's distance is a handle on
the end cap; until the user chooses how to combine, a positive distance
joins and a negative one cuts. Both take a `Combine` from
[geop-ops-booleans](./geop-ops-booleans.md): keep the result as a
`NewBody`, or immediately unite, intersect or subtract it with a `target`
solid — which is why this crate depends on the booleans, not the other way
round.
