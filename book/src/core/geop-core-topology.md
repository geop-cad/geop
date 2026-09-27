# geop-core-topology

> Brief overview only — full documentation is coming later.

B-rep (boundary representation) data structures, Euler operators, and
edit/validation logic for solids built from
[geop-core-geometry](../core/geop-core-geometry.md) primitives.

## The model

A `Model<S>` holds every entity in its own arena (`HashMap`), referenced by
typed ids (`VertexId`, `EdgeId`, `CoedgeId`, `FaceId`, `ShellId`,
`SolidId`):

```text
Solid ──▶ Shell(s) ──▶ Face(s) [+Surface] ──▶ Coedge(s) [+Curve2] ──▶ Edge [+Curve3] ──▶ Vertex
```

| Entity   | Holds                                                                  |
| -------- | ---------------------------------------------------------------------- |
| `Vertex` | a 3-D point                                                            |
| `Edge`   | a `Curve3` trimmed so its domain is exactly the edge, and its start and end vertex |
| `Coedge` | one side of an edge on one face: `sense`, a `Curve2` pcurve on the face's surface, `next`/`prev` in its loop |
| `Face`   | a `NurbSurface3D`, one `outer` boundary and a list of `holes`          |
| `Shell`  | a connected set of faces                                               |
| `Solid`  | its shells, outer shell first                                          |

Geometry is owned by the entity it belongs to, not shared through another
arena, because nothing references it on its own. A loop is not an entity
either: a face's boundary is a `BoundaryType`, either a `Loop` anchored at
one coedge whose `next`/`prev` cycle traces it, or a bare `Vertex`.

Some choices here encode invariants in the types:

- A face has exactly **one outer boundary**, so it is a field rather than
  "index 0 of a list". `BoundaryIndex` (`Outer` or `Hole(i)`) says which kind
  of boundary a lookup found. That distinction decides what joining two
  boundaries means: two points of one hole split it into two holes, two
  holes merge into one, a hole bridged to the outer loop stops being a hole,
  and only two points of the outer loop cut the face in two.
- "No boundary" is not a state. A face without edges is bounded by a bare
  vertex, which is what `mvfs` creates.
- A coedge is backed either by an `Edge` (shared with exactly one opposite
  coedge on the neighbouring face) or by a `Vertex`. The latter is a
  degenerate segment that lets a loop run along a surface row collapsed to a
  point, such as a sphere's pole.
- Orientation has no flag. It lives in the surface's parametrization, so
  `reverse_face` mirrors the surface in `u` and applies the same mirror to
  the face's pcurves.

## Euler operators

Every structural change goes through an Euler operator, which keeps the
Euler–Poincaré formula `V − E + F − L = 2(S − G)` true. Each checks its own
arguments (`argument_validation`: pcurves and curves start and end where
claimed, coedges lie on the same or on different loops as required) before
mutating anything.

| Make   | Kill   | Effect                                                              |
| ------ | ------ | ------------------------------------------------------------------- |
| `mvfs` | `kvfs` | a new solid with one face and one vertex                            |
| `mve`, `mve_from_vertex` | `kve` | a new vertex and an edge to it                      |
| `mef`  | `kef`  | an edge across a loop, splitting off a new face                     |
| `mer`  | `ker`  | like `mef`, but the split-off ring becomes a new boundary of an existing face |
| `mekr` | `kemr` | an edge joining two loops of one face into one                      |
| `mvr`  | `kvr`  | a bare vertex as a new boundary of a face                           |

`add_vertex_coedge` / `kill_vertex_coedge` splice a vertex-backed coedge in
and out of a loop. They are not Euler operators, since they touch neither
`V`, `E` nor `F`. `replace_face` and `replace_pcurve` swap geometry in
place; typically a face is built on the `everything()` placeholder surface
and gets its real surface once its boundary is complete. Both check that
every pcurve still lands on its edge's 3-D end points before changing
anything, so a rejected swap leaves the model untouched.

## Edit operations

`edit` restructures what already exists, and is what the boolean operations
are built from:

- `split_edge_at_vertex` splits an edge, and every coedge tracing it, at an
  existing vertex. The split parameter is refined with Newton until the
  split point really is the vertex, rather than splitting at an arbitrary
  point of a search box.
- `splice_edge_into_face` imprints an existing edge into a face. Depending on
  which of its end vertices already lie on the face's boundaries, it creates
  a hole, a spur, divides a hole, merges two holes, absorbs a hole into the
  outer loop, or splits the face.
- `merge_vertex`, `merge_edge` merge duplicates (vertex points are combined
  with `union`).
- `reverse_face` flips a face's material side without touching the
  neighbouring faces.
- `assemble_solid` builds the result of a boolean: one new solid made of the
  faces to keep, deleting everything no longer reachable from them.

## Containment

`contains::face::face_contains` classifies a `(u, v)` point against a face's
trim as `OnVertex`, `OnCoedge`, `Inside` or `Outside`, by casting a ray in
parameter space. `contains::shell` does the same in 3-D for a point against
a solid. Both first check for coincidence with the boundary. They then pick
ray directions from a seeded PRNG and retry until a ray crosses the boundary
only at clean interior points (a graze through a vertex or edge cannot be
counted reliably), so the parity of the crossing count gives the answer.

## Validation

`validation::validate` runs every whole-model check and returns *all*
violations at once rather than stopping at the first:

- `numerical_accuracy` runs first, since an entity that carries more
  uncertainty than the searches assume explains failures anywhere else;
- pointer validity and two-way references (`next`/`prev`, back-pointers);
- vertices lie on their curves and surfaces, and pcurves agree with their 3-D
  curves;
- pcurve loops are continuous, holes lie inside the outer loop, loops wind
  the right way and normals point outward;
- vertices and edges are disjoint, edges and faces are consistent, and no
  two faces intersect away from their shared edges.

`validate_fast` runs only the structural checks and the single-pass geometric
ones, skipping the pairwise intersection searches, for use after every step
of a long sequence. `validate_manifold` checks that a shell encloses a
volume consistently. The budgets for all of this are in
`ValidationParameters`.
