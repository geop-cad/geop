# Direct modeling, deformation, healing, stitching: SMLib vs geop

Scope: SmFeatureExecutive (defeature), RemoveFaces / DeleteFace / DeleteFilletFace,
ReplaceSurface / LocalOperation (move or replace face), ExtendSurfaces, TrimSurfaces,
Simplify, topological-edge removal (merging faces on one surface), taper and draft,
Bend / Unbend / Twist volumes with SpaceBend / SpaceUnbend, HealBrep, Stitch, Gap, Merge
post-processing, TrackTopologyChanges, Mirror / Transform, mesh to B-rep, CopyBrepMap,
and attributes.

SMLib root: `/home/tobiasjacob/usd-brep/source/SMLib`. geop root: `/home/tobiasjacob/geop`.
Line numbers are in `src/` unless a path is given.

**Summary.** SMLib is broad here but shallow. Many entry points are stubs, disabled,
or contain obvious bugs; they are listed so nobody ports them as written.

geop's few edits are deeper and more principled than SMLib's:
- STEP healing, draft and shell rebuild a solid face by face, moving vertices onto
  where the surfaces meet.
- Results are enclosures, never "average and grow the tolerance".
- Refusals say what they refuse.

geop is missing almost the whole *direct-edit* toolbox:
- delete face with heal, move or offset face (push-pull), replace face, defeature;
- merging faces that lie on one surface after a boolean;
- stitching with gaps or partial edge overlaps;
- global deformation;
- scaling;
- mesh to B-rep.

---

## 1. Feature table

| Capability | SMLib (file:function) | geop (crate::function) | Verdict |
|---|---|---|---|
| Delete faces, leave a hole (sheet) | `SmBrep.cpp:26172 RemoveFaces` → `DeleteFace` 7072 | `ops/geop-ops-edit/src/delete.rs DeleteBody` deletes whole bodies only; `ExtractFace` copies a face | **Missing**: no "delete face → open sheet" |
| Delete face and heal (extend the neighbours and re-intersect) | `SmFeatureExecutive.cpp:1922 ExtSurfaceRebuild` (G1-extend the neighbour surfaces, XOR boolean, trim, stitch); `SmFilletRemoval.cpp:124 RemoveFace` for fillets | missing | **Missing**, high value |
| Fill a hole with a planar cap | `SmFeatureExecutive.cpp:1865 PlanarCapRebuild` (`CreatePlanarFaceWith3DCurves`) | missing as an edit. Pieces exist: Coons patch in `ops/geop-ops-step/src/import/body.rs rebuilt_surface`, boundary surface in `ops/geop-ops-surface/src/boundary.rs` | **Missing** |
| Defeature: partition along a loop of edges, rebuild base and feature, restore | `SmFeatureExecutive.cpp:725 CollectFaces`, 852 `PartitionFeature`, 892 `BuildPatches`, 1119 `ApplyPatches`, 1184 `RestoreFeature`, 1217 `JoinPatches`, 1735 `ValidatePatch` | parametric history only: features are recorded (`ops/geop-ops-model/src/feature.rs`) and a step can be removed. Nothing for imported (STEP) solids | **Missing** for dumb solids |
| Remove a fillet face (restore the sharp edge) | `SmFilletRemoval.cpp:1535 SmBrep::DeleteFilletFace` | missing | **Missing** |
| Replace a face's surface (same topology) | `SmBrep.cpp:26240 ReplaceSurface` (drops pcurves, rebuilds by projection) | internal only: draft and shell rebuild via `BodySpec` (`ops/geop-ops-plastic/src/draft.rs`, `lib/geop-lib-shell/src/shell.rs`) | **Missing** as an op; geop's mechanism is better (vertices re-met, not just re-projected) |
| Move or offset faces in place (push-pull, LocalOperation) | `SmBrep.cpp:11923 LocalOperation` → `SmOffsetExecutive::DoExtendedOffset` with `SetExtendConvexEdges(TRUE)` | missing. `offset_faces` in `geop-lib-shell` makes a *separate* sheet; draft tilts planes only | **Missing**, highest value |
| Draft on planar faces next to planes or cylinders | (no direct-edit draft in SMLib; only `SmPrimitiveCreation::CreateTaperExtrude` 3604 at construction) | `ops/geop-ops-plastic/src/draft.rs draft` | **geop better** |
| Draft on non-planar faces, next to fillets or free-form, tangent neighbours | — (not in SMLib either) | refused (`draft.rs:194-258`) | gap in both; geop's refusals are good |
| Tapered extrude (draft angle at construction) | `SmPrimitiveCreation.cpp:3604 CreateTaperExtrude` (offset profile, corners as linear extension or G1 fillet) | missing on extrude; sweep has taper and twist (`program_tests.rs:868,903` in extrude-revolve) | **Missing** on extrude |
| Extend a surface | `SmBSplineSurfaceNLib.cpp:4469 CreateExtendedSurface` (G1/C2 via `N_SrfExtendByDist`); `SmBrep.cpp:4028 ExtendSurfaces` is **broken** (see §2.13) | `NurbSurface3D::extended` (polynomial continuation, `core/geop-core-geometry/src/nurb_surface/patch.rs:266`); `ExtendSurface` op (`ops/geop-ops-surface/src/extend.rs`): a lone four-sided face, one edge | geop's math is better (exact continuation); the op is narrow |
| Trim surfaces by curves on them | `SmBrep.cpp:28988 TrimSurfaces` (`MergeCurvesOnSurface` + keep / delete / split by reference point) | `TrimSurface` (`ops/geop-ops-surface/src/trim.rs`): by a face, keep a side; `ProjectCurve` imprints a sketch | Comparable. Missing: trim by curves plus a pick point, and split-only |
| Merge faces that lie on one surface (remove "topological" edges and vertices) | `SmEdge.cpp:4986 IsTopological`, `SmBrep.cpp:26158 RemoveTopologicalEdgesAndVertices`, `SmMerge.cpp:827 BooleanPostProcess`, `SmBrep.cpp:7601 DeleteTopologicalVertex` | **missing**. AGENTS.md "Faces a union leaves in two are one wall": fillet works around it | **Missing**, high value |
| Squeeze a degenerate face or edge | `SmBrep.cpp:27469 SqueezeFace`, 27526 `SqueezeFaceIfDegenerate`, 26985 `SqueezeEdge` | none; validation's `faces_have_interior` detects | **Missing** (as healing) |
| Healer pipeline | `SmHealData.cpp:1716 HealTgts` (24 steps, 5 of them stubs, `Fix_BadLoops` disabled) | STEP import `heal` (`ops/geop-ops-step/src/import/body.rs:2918`): vertices re-met where the surfaces meet, edges re-traced, contradictory small faces rebuilt as Coons patches, edges widened to the file's uncertainty | **geop better in kind** (moves geometry onto the surfaces; SMLib only grows tolerances). Missing: SMLib's topological cleanups (degenerate edges and faces, coincident vertices, poles, unclosed loops) |
| Grow tolerance to cover gaps | `SmHealData.cpp:2801 Fix_TolSizes`, `inc/SmTol.h:610-688` | `widened` (`body.rs:3590`) unions the enclosure up to `ACCURACY` | geop better (honest enclosure, bounded) |
| Gap measurement | `SmGap.cpp:1620 BuildSampleSet` (samples, plus mid-knot samples) | `edge_gap` (`body.rs:3438`) | Both sampled. geop should move to a `nearest::least_distance` enclosure (AGENTS "Do not decide from samples") |
| Stitch faces into a shell or solid with a tolerance | `SmStitch.cpp:1086 DoStitching`, `SmBrep.cpp:26555 StitchFaces`, 26661 `StitchAndOrient`, `SmStitch.cpp:2659 StitchIntoSolid` | `Knit` (`ops/geop-ops-surface/src/knit.rs`): vertex enclosures overlap, edges with equal ends | geop's topological test is exact. **Missing**: gaps beyond the enclosure, T-junctions (partial overlap; refused), sliver and short-edge squeeze |
| Orient a closed shell outward | `SmBrep.cpp:24120 OrientTrimmedSurfaces` (ray votes), `SmStitch.cpp:2512 UnifyNormals` | `knit.rs` turns faces for consistency plus `normal_points_outward` (validation `face_orientation.rs`) | geop fine (exact parity per AGENTS) |
| Seam, pole and periodic repair | `SmHealData.cpp:4239 Fix_MoveSeam`, 4443 `Fix_SplitEdgesAtSeam`, 5320 `Fix_SplitFaceAtSeams`; `MakeVertexLoop` at a missing pole (disabled) | STEP import: `cut_wrapping_face`, `cut_tube_wrapping_face`, `cut_whole_torus`, `cut_helical_face`, `drop_doubled_edges`, `split_edge` of closed edges, `poles_of` | Equivalent; geop's NURBS patches never wrap, so most of SMLib's seam work does not exist in geop |
| Sheet that encloses two shells (mislabelled) | `SmHealData.cpp:4732 Fix_BadSheets` | n/a (geop's solids are built whole) | N/A |
| Mirror a body | `SmBrep.cpp:23449 Mirror` (curves and surfaces mirrored, uses reversed, pcurves rebuilt) | `Model::transform_body` (`core/geop-core-topology/src/edit/transform_body.rs`) plus `Mirror` op (`ops/geop-ops-pattern/src/mirror.rs`) | geop better (pcurves kept; faces reversed exactly) |
| Rigid move or copy | `SmBrep.cpp:28777 Transform` | `MoveBody` (`ops/geop-ops-pattern/src/move_body.rs`) | parity |
| Uniform scale | `Transform(..., cpOptScale)` isotropic | **missing** (`Motion` has no scale: `core/geop-core-math/src/primitives/pose.rs:714`) | **Missing** |
| Non-uniform scale (to B-splines) | `Transform` non-isotropic branch (28800+): everything to NURBS | missing | **Missing** (exact for NURBS: an affine map of control points) |
| General 4x4 transform | `SmBrep.cpp:28975 Transform(matrix)` | missing | low priority |
| Bend about an axis (neutral radius, K-factor) | `SmBendVolume.cpp` (math `h:40-75`), `SmSpaceBend.cpp:161 DoSpaceBend` | `ops/geop-ops-pattern/src/deform_body.rs DeformBody` (bend), `core/geop-core-geometry/src/deform.rs AxialMap::bend` | **Done**: zone imprinted, rest carried on rigidly, proven bound, pcurves kept |
| Unbend (flatten about an axis) | `SmUnbendVolume.cpp`, `SmSpaceUnbend.cpp:183` | `FlatPattern` for sheet-metal bodies only (`ops/geop-ops-sheetmetal/src/flat_pattern.rs`) | Partial (sheet metal only). Not an `AxialMap`: its map takes a point by its angle about the bend's centre, `atan2` of two coordinates, which no map affine across the axis composes on the control points; it would need a fit of the composition with a proven bound |
| Twist about an axis | `SmTwistVolume.cpp` (no B-rep driver!) | `DeformBody` (twist), `AxialMap::twist` | **Done**, either hand |
| Taper, pinch, shear (axial shape function) | `NL_Geom.cpp:4740 N_TransformPtWithShapeFuncAndScale` (points only; curve and surface versions compiled out) | `DeformBody` (taper, pinch, shear), `AxialMap::taper` (a pinch is a taper one way across), `AxialMap::shear` (affine: `AxialMap::as_motion`, no zone division) | **Done**, all exact |
| Set deform points (free-form: move control points) | `SmBrep.cpp:9324 GetDeformPoints` / 9382 `SetDeformPoints` | missing (subd has a cage editor) | low priority |
| Mesh (triangles) to B-rep | `SmBrep.cpp:3128 CreateFromTriangleBag` (one planar face per triangle, then stitch), 3073 `CreateFromPolyBrep` (`MakeFaceFromPolyFace`) | `ImportStl` (`ops/geop-ops-step/src/stl.rs`), ASCII or binary: `BodySpec::of_triangles`, coplanar triangles merged. Binary files travel as `data:` URLs of their bytes (`geop_core_program::workspace::content`) | **Done** |
| Split a face at C0/C1 discontinuities of its surface | `SmBrep.cpp:18366 SubdivideFaceAtDiscontinuities`, 17702 `FixAllFaceDiscontinuities` | a sweep cuts each profile curve at its corners, C0 knots where it turns (`SweepLoop::at_corners`), so no swept face is creased; a creased wall had the shell fail to offset it. Not yet for a surface a STEP file gives creased; fillet puts stations at kinks | Partial |
| Simplify (knot removal, refit) | `SmBrep.cpp:3946 Simplify` → `Refine3DCurves` (the comment says it "causes bad problems"), `RemoveKnots` | missing | Low priority; SMLib's is unreliable |
| Recognise analytic surfaces in B-splines | `SmBrep.cpp:2303 ConvertCurvesToAnalytics`, `CopyAndAddAnalytics`, `RefitSurfOfRevolutionSurfaces` | `NurbSurface3D::as_plane` / `as_cylinder` (used by draft); STEP import keeps analytic kinds | Partial |
| Topology change tracking | `SmTrackTopologyChanges.cpp:189` (Notify callback: removed, added, changed lists) | stable names (`Namer`, `lib/geop-lib-booleans/src/naming.rs` edge_split / face_split / trace / piercing) | **geop better** for its purpose (persistent naming); see §2.16 for what to borrow |
| Attributes propagated through split, merge, copy | `inc/SmAttribute.h:226-520` (`SM_AB_COPY`/`REFERENCE`/`TEMP`; Split, Merge, Coincident, Copy callbacks) | names only; colour is a part property (`ops/geop-ops-model/src/parameters/steps.rs:617`); no per-face attributes | Missing: per-entity user attributes (colour, material, tags). Round 3 judged it not small: a face's colour needs a field in the scene protocol and the web renderer, a step to set it, and its inheritance read from a name's arguments, which a long name spells by its digest (`LONGEST_ARGUMENT`) and so loses (see §2.15) |
| Copy a B-rep with an original↔copy map | `SmCopyBrepMap.cpp` | `extract.rs`, pattern copies named `move(M,X)` / `mirror(M,X)` | parity via names |
| Validate a patch before applying | `SmFeatureExecutive.cpp:1735 ValidatePatch` | `validate` / `validate_fast` (`core/geop-core-topology/src/validation/mod.rs`) after every build | geop better |

---

## 2. Special cases and features to port (prioritised by how often a user hits them)

### 2.1 Move / offset face in place (push-pull), the core direct edit — P0

**SMLib.** `SmBrep.cpp:11923 SmBrep::LocalOperation(faces, newSurfaces)`:
- Each picked face gets a new surface. A reversed one is flipped by comparing normals
  at the domain centre (a sample: do not copy that).
- Unpicked faces are copied with `CopyAndAddAnalytics`.
- Everything goes through `SmOffsetExecutive(SM_OO_LOCAL_OPERATION)`.
  `SetExtendConvexEdges(TRUE)` means a gap opened at a convex edge is closed by
  extending and intersecting neighbours, not by a fillet.
- `DoExtendedOffset` then re-meets the topology and the old faces are swapped for the
  new ones.

Hard cases SMLib leaves to the offset executive:
- New gaps at convex edges: extend and intersect.
- Overlaps at concave edges: trim.

It has no handling of topology changes: a face that vanishes, or a step that becomes
deeper than a neighbour.

**geop today.** The machinery exists, private to two callers:
- `geop_lib_shell::{extended, pcurve, surface_missed}` and `geop_core_geometry::meet::{meet, tangent_groups, Offset}`.
- Draft (`ops/geop-ops-plastic/src/draft.rs`) already does "replace the face's surface,
  keep the topology, move every vertex to where its faces' surfaces meet, rebuild each
  edge where its two faces meet". It does this for planes next to planes or cylinders.
- Shell does it for offsets.

**Port.**
- New lib entry `geop_lib_shell::replace_faces(part, faces: &[(FaceId, NurbSurface3D)]) -> BodySpec`.
  This generalises draft's builder; the module docs already say "what rebuilding a solid
  face by face needs".
- Three operations in a new op crate `geop-ops-direct` (or `geop-ops-edit`), each a
  `PartOperation`:
  - `MoveFace`: translate or rotate the surface (`NurbSurface::transform`).
  - `OffsetFace`: `NurbSurface3D::offset`, as shell uses.
  - `ReplaceFace`: the surface of another face or sheet.
- Vertices: `meet` with one `Offset` condition per tangent group, from the old vertex as
  the seed. Check the returned point against every surface
  (AGENTS "Check an iterative answer against every condition").
- Edges:
  - between two planes: a line between the moved vertices;
  - plane and cylinder: draft's arc / plane-section code;
  - general: trace where the two surfaces meet between the two vertices, using the
    boolean's tracer (`predictor_corrector_step`).
- Refuse up front, by name (`inward_corner`-style):
  - an edge whose two new surfaces no longer meet between its vertices (the face
    vanishes, or an edge turns around, as draft already detects);
  - a tangent neighbour (fillet): refuse, or move the fillet along with it, as phase 2.
- Phase 2: topology change. When a moved face passes a neighbour's far edge, fall back
  to a boolean: extend the moved face's surface (`NurbSurface3D::extended`), build a
  tool, and cut or unite.
  - Do not stop the face on an existing corner (AGENTS "Never place a cut on a corner
    that already exists").
- Names: keep every name (draft does).
- Editor: a drag gizmo along the face normal (like `MoveBody`'s `Gizmo`), plus an
  `editor_tests.rs` test that drives it.
- Stress sweep over `geop_lib_fixtures::bodies`: move every planar face ±d, as the
  shell sweep does.

### 2.2 Delete face and heal (close the gap by extending neighbours) — P0

**geop (round 2).** `geop_lib_shell::delete_faces` heals a gap of 3 to 8 sides, left by one face
or a chain of them (a fillet chain): the faces around the gap close it as some triangulation of
its polygon of sides, each three meeting at a corner, each two sharing two corners along a new
edge; every triangulation is tried through `rebuilt` and the first it accepts is kept. No branch
on the number of sides or on surface kinds.

**SMLib.** `SmFeatureExecutive.cpp:1922 ExtSurfaceRebuild`:
- Collect the faces along the loop edges.
- If no loop edge lies on the face's outer loop (the hole is an inner loop of that face):
  copy the face with only that inner loop, flipped to become the outer loop. That cap
  is the patch (1990-2030), e.g. a through hole's rim face simply gets its hole filled.
- Otherwise, the hole crosses the face's outer loop:
  - `CreateExtendedSurface(dExt, SM_CT_G1)`, where `dExt = 2 × the loop's bbox max
    length` if not given (1970);
  - a face over the whole extended domain, with the original face XOR'ed out
    (`ManifoldBoolean(SM_BO_EXCLUSIVE_OR)`);
  - all such faces `PiecewiseMerge`d;
  - `sm_TrimPatches` (31): delete patch faces that use no loop-edge curve (checked by
    midpoint `DropPoint`) or are contained in an existing face (one point classified);
  - `StitchAndOrient`, `ShrinkGeometry`.
- `ValidatePatch` (1735) checks that:
  - the patch exists;
  - its only lamina edges are loop edges (else the patch is "too small");
  - face × face intersections with the target have no curve inside a face (the patch
    must not pierce the body).
- `JoinPatches` (1217): when each loop alone fails (a fillet has two loops, one patch),
  merge the patches of all loops and validate again.
- `SM_RB_FROM_OTHER_BREP`: build the feature's patch from the base's extended surfaces
  (the cut-off feature becomes a closed solid).

`SmFilletRemoval.cpp` (`DeleteFilletFace`, 124-1535) is the specialised version for a
4-sided fillet:
- **Topology** (`CollectFilletTopology` 208): two tangent rails and two end edges, by
  `IsTangentEdge(2°)`.
- **New corner** (`CalcNewCornerVertices` 465): surface-surface-surface intersection of
  the end face with both tangent faces (`LocalSurfaceSurfaceIntersect`), domains
  expanded ×2, out-of-bounds evaluation enabled. The vertex is the **average** of the
  three evaluations, tolerance their max spread (AGENTS: use the union, never average).
- **Side edges** (`FindSideEdgeExtensions` 581): if the side edge's curve does not reach
  the new corner, extend it with the end-face × tangent-face intersection and join.
  With more than 3 edges at a corner, the end edge's geometry is replaced instead.
- **The new sharp edge** (`IntersectTangentSurfaces` 812): the intersection of the two
  extended tangent surfaces.
- **Edit** (`DoRemoval` 1015): move vertex and edge onto the adjacent face
  (`MoveVtxAndEdgeToAdjacentFace` 905).

**geop today.** Missing. `DeleteBody` deletes whole bodies only.

**Port** (an op crate `geop-ops-direct`, algorithm in a lib):
- `DeleteFace { faces, heal: Heal }`, where `Heal` is `Open | Cap | Extend`.
  - `Open`: delete the faces, leave a sheet. Model edits only; validation of a sheet.
  - `Cap`: if the hole's loop is planar (enclosure: every vertex and edge on one
    `Plane`, via `as_plane` of a fit, never a sample), build a planar face. Otherwise
    build a Coons / boundary patch from `ops/geop-ops-surface/src/boundary.rs`; that is
    G0 only, say so.
  - `Extend`: the neighbours' surfaces continued (`NurbSurface3D::extended`; exact for
    analytic and NURBS), then re-met exactly as in 2.1.
- `Extend` has two topological cases; decide each by topology:
  - **(a) Inner-loop case.** The deleted faces all lie inside one hole of a neighbour
    (a boss, a pocket, a hole's wall). The heal is "remove that hole loop from the
    neighbour face" (`Model::remove_boundary` on a `BoundaryIndex::Hole(i)`). No
    geometry at all; cheap, exact, and the most common case (delete a hole or a boss).
    Check that what is left of the solid closes: the deleted faces form exactly the
    region bounded by that loop.
  - **(b) Crossing case** (a chamfer, a fillet, a step face). The deleted face's
    neighbours across its edges meet each other once it is gone:
    - every edge between two surviving neighbours across the gap is rebuilt where their
      extended surfaces meet;
    - every vertex is re-met with `meet`.
    For a 4-sided face between two rails (a fillet or chamfer), this is SMLib's
    `DeleteFilletFace`:
    - the corner vertex is `meet` of the end face and the two rail faces;
    - with the enclosure kept, `union` the three foot points, never the average;
    - side edges are extended (`NurbCurve::extended`) up to the new corner, then
      re-trimmed, or rebuilt as a trace of end face × rail face.
- Refuse, naming faces, when:
  - the extended neighbours do not meet within the face's bounding box grown by its
    size (a free choice, as `extended`'s "room");
  - more than three surfaces meet at a new corner without a common tangency group;
  - the new edge would cross an existing face (validate with `validate_fast`;
    `ValidatePatch` equivalent: a lamina edge outside the loop means the patch is too
    small).
- Multiple loops of one feature (a fillet chain whose loops form one patch): handle
  them as one region; that is SMLib's `JoinPatches`.
- Regression tests:
  - delete the blind hole of `geop_lib_fixtures::drilled_box` (inner-loop case);
  - delete a fillet face from a filleted box (crossing case);
  - delete a chamfer;
  - delete a boss on a plate (inner loop on the top face).

### 2.3 Merge faces that lie on one surface ("topological edges") — P0

**SMLib.** `SmEdge.cpp:4986 SmEdge::IsTopological`. An edge can be removed when all of
these hold:
- (1) it is manifold, 4 edgeuses;
- (2) it is G1: `|n1·n2| ≥ 1 − sqrt(eps)` at samples 0.37, 0.57, 0.82;
- (3) the surfaces coincide at those samples within the cross-section tolerance;
- (4) it is not a seam edge: on a closed direction, an iso-curve on the domain boundary;
- (5) either the same surface on both sides, or both surfaces analytic or planar and
  `sm_CoverOtherCoincidentFace` shows one surface can be extended over the other
  face's trim. For planes the tolerance is divided by 10000.

Then:
- `DeleteEdge(edge, surfToKeep)` merges the two faces into one, with the merged UV
  domain;
- `DeleteTopologicalVertex` (`SmBrep.cpp:7601`) joins two edges at a vertex where
  exactly two edges meet and their curves join smoothly. It deletes the shorter of the
  two.

This runs after every boolean (`SmMerge.cpp:827 BooleanPostProcess`, gain 10) and on
request (`RemoveTopologicalEdgesAndVertices`, gain 1).

**geop today.** Missing.
- A box united flush with a box leaves two coplanar faces and a seam edge.
- A pattern of blocks leaves a grid of edges.
- Fillet, draft and shell then see two faces where the user sees one. AGENTS.md
  ("Faces a union leaves in two are one wall") documents a workaround in `tool_end`.
- The STEP round-trip bugs in `ops/geop-ops-step/src/regression_tests.rs:81,113`
  (a linear pattern of overlapping blocks; a mirrored L bracket) involve exactly such
  faces.

**Port** (a lib function in `geop-lib-booleans`, called at the end of `boolean()` and
offered as a `Simplify` / "Merge faces" op):
- Same surface, decided exactly:
  - geop's faces each carry their own NURBS patch, so "same surface" is "the
    recognised analytic shapes are equal": `as_plane` / `as_cylinder` /
    revolved-shape equality with `could_be_equal` on their parameters. A NURBS patch is
    equal only when its control net is identical after aligning knots.
  - Do **not** sample normals (SMLib's step 2): two planes are one plane or not.
- Seam rule: geop patches do not wrap, so a cylinder in two halves must stay two faces
  if the union would wrap past the patch. Merge only if one patch can carry both faces:
  - a `NurbSurface3D::extended` patch over the union of the two `(u, v)` boxes;
  - for revolved faces, the sweep must stay below the patch's limit (cut wrapping faces
    as STEP import's `cut_wrapping_face` does).
- Topology: removing the edge between two faces of one solid joins their outer loops.
  If the edge is a bridge (the same face on both sides after an earlier merge), the
  loop splits into an outer loop and a hole. This is the `BoundaryType` case analysis
  in AGENTS "Make the invariant a type".
- Then join edges at vertices of degree 2 whose curves are the same line or circle
  (exact shape equality), with `NurbCurve` concatenation. That is SMLib's
  `DeleteTopologicalVertex`; it does not decide smoothness from samples.
- Names: the merged face keeps the older id's name (ids are in creation order;
  AGENTS "A hash map's order is no order"). Record the absorbed name as an alias so
  references to either resolve. Needs a `Namer` alias facility.
- Tests:
  - two blocks united flush give 6 faces;
  - a linear pattern of 4 overlapping blocks, then STEP round trip (un-ignore
    `regression_tests.rs:81` if fixed);
  - a cylinder made of two halves stays two faces on one seam if it would wrap.

### 2.4 Knit / stitch with gaps, T-junctions, slivers — P1 (STEP, IGES, imported sheets)

**SMLib.** `SmStitch.cpp:1086 DoStitching`.

Pre-passes:
- delete lamina faces that are degenerate within tol/2 (1158-1176);
- squeeze lamina edges shorter than the tolerance (1233-1333).

Then up to 10 rounds of:
- **I. Split edges at nearby vertices** (`sm_TestAndSplitEdge` 518):
  `tol = max(stitchTol, vtxTol + edgeTol)`. The interior test `sm_CheckInteriorEdge`
  (453) requires the parameter at least 20·tol/|C'| from the ends and the point not
  within tol of the end vertices. This is how T-junctions get handled.
- **II. Merge vertices.** Sort along the (1,1,1) line, sweep a window, and squeeze a
  short shared lamina edge (length < 2·distance) rather than glue.
- **III. Merge edges** with the same two vertices, midpoint within tol. Orientation
  from the tangent dot product. Max distance from 5 samples, lines skipped.

Tolerances and orientation:
- Tolerance: `StitchAndOrient` uses min(2 × the B-rep tolerance, 100 × 1e-5), then a
  second pass at ×20 if more than one lamina edge remains.
- `StitchIntoSolid` (2659) doubles the tolerance from tol/10 up to diagonal/200 until
  no lamina edges remain.
- Orientation: `OrientTrimmedSurfaces` (`SmBrep.cpp:24120`) with ray votes from
  interior points, skewed ray `(1.23, 2.35, 3.46)`.

`SelectTopologyToKeep` (54) keeps the vertex with more edges, and the edge closer to
the other side's faces.

Bug: `SmStitch.cpp:1423` never resets the candidate flags.

**geop today.** `Knit` (`ops/geop-ops-surface/src/knit.rs`):
- vertices join when their enclosures overlap (`UnionFind`);
- edges join when they have the same vertex pair and `curve_could_contain` the other's
  middle;
- a partial overlap is an open edge, refused if a solid is asked;
- ambiguity (two vertices of one sheet in one group) is refused (`knit.rs:285, 317`).

This is exact and honest, but sheets from another system with a 1e-4 gap never knit.

**Port** (in the order a user hits them):
1. **T-junctions.** For each open edge, `curve_surface`-style search for the other
   sheet's open vertices that `curve_could_contain` (enclosure, no epsilon). Split the
   edge there with `Model::split_edge_at_vertex`, refining `t` by Newton on
   `C(t) = vertex point` before splitting (AGENTS "Validate the value you are about to
   use"). Then knit as today.
   - Interior test: the parameter enclosure must exclude both end parameters,
     topologically, and the point must not overlap an end vertex's enclosure.
   - This replaces SMLib's 20·tol/|C'| margin.
2. **Gap tolerance as an explicit argument.** `Knit { gap: Option<Length> }`, which
   widens every open vertex and open edge enclosure by `gap` before matching (as STEP
   import's `widened`). The user states the uncertainty; nothing is guessed. Report
   the largest gap actually closed (like `Healing::summary`).
   - Never auto-double the tolerance until it closes (`StitchIntoSolid`); that is a
     tunable deciding the answer.
3. **Re-meet instead of growing tolerance.** After joining at a stated gap, move the
   joined vertices to where their faces' surfaces meet (`meet`) and re-trace the
   edges, as STEP `heal` does. The width then comes back down to the surfaces'
   agreement. Share `heal` by moving it from `ops/geop-ops-step/src/import/body.rs`
   into a lib (`geop-lib-heal`, or `geop-lib-shell`), since Knit and STEP both need it.
4. **Short edges and slivers.** An open edge whose two vertices could be equal
   (enclosures overlap) is squeezed (`merge_vertex` and delete the edge). A face whose
   whole boundary lies within the gap of one edge is deleted. Name both in the report.
5. **Orientation.** Keep geop's exact consistency flood plus `normal_points_outward`.
   SMLib's ray votes are samples; do not port them.

### 2.5 Healing pipeline for imported B-reps (topological steps geop lacks) — P1

SMLib's healer (`SmHealData.cpp:1716`; full step list in the stitch survey) does these
steps that geop's STEP `heal` does not:
- **Degenerate edges** (step 1/9, `SmEdgeProps.cpp:482`, `Fix_DegenEdges` 3706): squeeze
  an open zero-length edge onto its start vertex, delete a closed one.
  - geop: STEP import splits closed edges; a zero-length open edge goes through as an
    edge between two vertices whose enclosures overlap. Port: in `normalize`, merge the
    vertices and drop the edge when the vertex enclosures overlap and the curve's hull
    lies within their union (exact, topological).
  - **Done (round 2):** `Builder::join_coincident_vertices` makes an edge's two ends one
    where they and the curve's hull lie within the file's uncertainty, and drops the edge;
    the vertex is then re-met where its faces meet by `heal`, nothing squeezed. Likewise
    where one coedge of a loop ends at a vertex and the next starts at another within the
    uncertainty. Vertices joined by no loop or edge stay two (proximity is not membership).
- **Coincident vertices** (steps 5/6, `CombineCoincidentVertices`, `SmBrep.cpp:1266`):
  every branch (no edges / shared short edge → squeeze / shared face → temp edge and
  squeeze / nothing shared → glue).
  - geop: vertices are taken from the file's ids, so two file vertices at one point
    stay two. Port: a voxel or bounding-box tree over vertex enclosures; overlapping
    ones of the same body merged with `Model::merge_vertex` (which already unions the
    points). Refuse when two vertices of one face's loop would merge non-adjacently
    (that is a pinch, not a duplicate).
- **Degenerate or sliver faces** (steps 7/8, `SmFace::IsDegenerate`, `SmFace.cpp:14964`;
  `SqueezeFace`, `SmBrep.cpp:27469`). Definition: every point of the face is within
  tolerance of some boundary; equivalently, every pair of non-degenerate edges
  coincides.
  - geop: `rebuilt_surface` rebuilds contradictory small faces, but a true sliver
    (zero area) gets a degenerate Coons patch.
  - Port: detect topologically and by enclosure. All of the face's edges pair up as
    `curve_could_contain` each other, so the face's two sides are one curve. Collapse
    the face: merge each coincident edge pair (`merge_edge`; its TODO to union the
    curves must be done first), delete the face. Report by name.
- **STEP sphere or torus given as one vertex loop** (`SmHealData.cpp:3526-3620`): rebuild
  the face over the natural domain.
  - geop: check `cut_wrapping_face` / `cut_whole_torus` cover the "vertex loop only"
    shape. Add a regression test of a STEP sphere written as `VERTEX_LOOP` + `SPHERICAL_SURFACE`.
- **Missing pole vertex** (`SmFaceProps.cpp:924-962`, fix in disabled `Fix_BadLoops`
  6171-6250): add a vertex at a pole inside a no-area loop.
  - geop: `poles_of` handles collapsed rows. Make sure a STEP cone or sphere face whose
    loop does not visit the apex gets a pole vertex: the apex is where `Vertex`-coedges
    go (AGENTS "A pole is one vertex at many (u, v)").
- **Loop orientation and order** (stage 3, `SmFaceProps.cpp:1527-1700`): flip loops with
  the wrong orientation; pick the outer loop; split a face with several outer loops into
  faces (`MakeFaceFromLoopOnFace`).
  - geop: orientation is integrated exactly (`nurb_curve::loop_signed_area`). Port the
    *multi-outer-loop split*: a STEP face with two loops that are both outer (an IGES
    habit) should become two faces, decided by `pcurves_contain` (one loop inside the
    other or not), not by the file's `FACE_OUTER_BOUND` flag. Today
    `holes_inside_outer` validation would reject it.
- **Gaps between edges and faces larger than tolerance.** SMLib only grows tolerance
  (`Fix_TolSizes` 2801); `Fix_BadGaps` 4042 is an empty stub; `SmGap::RefineGeometry`
  (217, edge rebuilt as surface×surface, vertex at surface³ corner) exists but the
  healer never calls it.
  - **geop already does the right thing** (`heal` re-meets vertices and edges). Keep it;
    extend it to the knit (2.4.3).

Do not port:
- the stubs (CoinEdges, MissedEdgeXSects, BadGaps, BackPointers, UncontainedEdges);
- SMLib's model-size → tolerance bands (`SmTol.h:567`), which are exactly the epsilon
  AGENTS forbids. geop's tolerance is the file's stated uncertainty.

### 2.6 Defeature (feature recognition + partition + rebuild) for dumb solids — P1

**SMLib.** `SmFeatureExecutive`.
- The user gives a feature by faces, or by its boundary edge loops.
- `CollectFaces` (725) marks the loop edges and flood-fills faces from a seed without
  crossing them.
  - It asserts every loop edge has exactly 2 faces, one in the feature and one in the
    base ("Loop does not bisect the brep").
- `PartitionFeature` (852): copy the faces to a feature B-rep, remove them from the base.
- `BuildPatch` (989), per loop × {base, feature}, with `SmRebuildBrepType`:
  - `NONE` (keep it open);
  - `PLANAR_CAP`;
  - `EXTEND_SURFACE`;
  - `FROM_OTHER_BREP` (the feature's patch is the base's extended surfaces, which makes
    the cut-off feature a closed solid).
- Patches are cached so one is not built twice.
- `m_bTriedExtSurf` stops a known failure being retried.
- `ApplyPatches` (1119): copy patch faces, `StitchAndOrient`.
- `RestoreFeature` (1184): put the faces back if the rebuild failed.
- N-sided patch rebuild (2223) is "In development. Does not work."

**geop today.** Parametric suppression works for features made in geop (delete the
step); nothing works for imported solids.

**Port.**
- `Defeature { faces | loops, rebuild_base, keep_feature }`. This is 2.2 over a region
  of faces instead of one face.
- Region: flood fill over faces not crossing the picked loop edges (`solvers::walk::flood`).
  Refuse by name if a loop edge has both faces on one side ("the loop does not cut the
  solid in two").
- `keep_feature`: also output the feature as a solid, its opening closed by the base's
  extended surfaces turned around. This is `FROM_OTHER_BREP`, useful for "extract the
  boss as a body".
- Feature recognition, later and separate:
  - a hole = cylindrical faces forming a hole loop in one face;
  - a fillet = a face tangent along two rails (topologically: shares G1 edges, decided
    by surface equality of tangent planes along a recognised rolling-ball shape, not by
    a 2° angle sample as `CollectFilletTopology` does);
  - a boss or pocket = faces inside one hole loop.
  Offer "pick one face, select the whole feature".

### 2.7 Global deformation: bend, unbend, twist, taper — P2

**SMLib** (survey of `SmVolume` stack; `NL_VolumeAdv.cpp` and `SmBSplineVolume.cpp`
are dead code):
- **Bend** (`SmBendVolume.cpp` 1261 `EvaluateSimple`):
  `P = C + u cos(v/d)·Mid + u sin(v/d)·Bin + w·Axis`.
  - The neutral radius `d` preserves arc length at radius `d`.
  - Singular axis `u < singTol`: only warns.
  - Multivalued at ±π: two solutions (2771-2797).
  - Domain `|v/d| ≤ π`: a bend past 360° is unsupported.
- **Unbend**: the inverse, `r = √(u²+v²)`, `θ = atan2(v, u)`, `(r, dθ, w)`. No `r = 0`
  guard.
- **Twist**: rotation by `2πw/p`.
  - Bugs: every left-hand twist is clamped (292-297); wrong sign of `Duw` (1045).
  - No B-rep driver.
- **Taper / pinch / shear**: `N_TransformPtWithShapeFuncAndScale` (`NL_Geom.cpp:4740`),
  points only.
- **B-rep driver** (`SmSpaceBend.cpp:161 DoSpaceBend`):
  - (1) imprint the bend zone's boundaries by a boolean with a box (start and stop faces
    picked by hard-coded indices 1283);
  - (2) flood-fill the before / inside / after regions;
  - (3) inside: wrap curves and surfaces lazily as `SmCrvInVolume` / `SmSrfInVolume`
    (exact composition, chain rule to 3rd derivatives), then `ApproximateWithBSplines`
    (bicubic patches, 5 subdivision levels; failure silently ignored);
  - (4) before / after: rigid motions, so C1 at the zone boundary comes from the bend
    map's own derivative at the boundary (`TestBendTransforms` 1575);
  - (5) special-case exact output: a plane ⟂ the axis stays a plane; a plane ⟂ Mid
    becomes a cylinder; a line along Bin becomes a circle (sampled checks).
- Pcurves are deleted and re-projected.

**geop today.** Done (taper, twist, bend; unbend not): `AxialMap` in `core/geop-core-geometry/src/deform.rs` composes each Bézier piece with the map on its control points (Bernstein products; the trigonometric coefficients replaced by Chebyshev cubics with a proven bound that widens the result), `Model::transform_body` takes any `Deformation`, and `DeformBody` imprints the zone ends first. Sheet metal still builds bends and flat patterns by construction.

**Port** (a lib `geop-lib-deform` and an op crate `geop-ops-deform`):
1. **Split** the solid at the zone's planes with `geop_lib_booleans::split`.
   - The zone boundaries are planes ⟂ the bend direction at `±d·θ/2`, named by the
     user's datum, never chosen on an existing vertex (AGENTS "Never place a cut on a
     corner that already exists").
   - If a zone plane passes through an existing vertex, refuse by name or shift the
     zone, a user choice.
2. **Before and after regions**: `Model::transform_body` with exact rigid `Motion`s.
3. **Inside region**, exact where geop can be exact:
   - Planes ⟂ the axis stay planes (rotated). Planes ⟂ the bend direction become
     cylinders about the axis (`shape::Cylinder`, exact rational NURBS).
   - Planes ⟂ the radius stay radial planes. Lines become lines, arcs or helices.
   - Decide these by the face's recognised shape (`as_plane`, normal enclosure vs the
     axis), not by 6×6 samples.
   - Otherwise fit: map each surface by fitting the composed map `B∘S` on the patch's
     own knot grid, refined while the enclosure of `B∘S − fit` is wider than the target
     (AGENTS "let the sample count follow the width it produces"). The error bound must
     be an enclosure: evaluate `B` with interval arithmetic over each fitted span
     (`bend` is analytic: cos and sin of an interval), and **fail by name** if not
     reached (SMLib silently keeps the bad fit).
4. **Vertices** are the exact map of their enclosures (interval cos and sin).
   **Edges**: the same treatment as surfaces. **Pcurves** do not change for exact
   compositions, since a face's patch is reparametrised by the same `(u, v)`:
   `S'(u, v) = B(S(u, v))` keeps every pcurve valid. **This is the big win over SMLib**:
   no re-projection, so pcurves and their names keep.
5. **Refuse**:
   - material reaching the axis (`u` enclosure reaching ≤ 0: SMLib only warns);
   - a bend angle with `|θ| ≥ 2π` (self-overlap);
   - a bend at angle 0 (no-op, not an error);
   - a twist period of 0;
   - self-intersection: `validate_fast` after the build, naming the faces.
6. **Twist and taper.** The same driver with no zone split (or one zone). A taper with a
   linear scale law `s(w)` maps planes ⟂ the axis to scaled planes, exactly. Twist about
   the axis maps a cylinder about the axis to itself and a plane through the axis to a
   helicoid (fit).
7. **Unbend** of a general solid about a user axis (the inverse map), for non-sheet-metal
   flattening. Lower priority; sheet metal's `FlatPattern` covers the common case.

### 2.8 Scale (uniform and non-uniform) — P1 (cheap)

**SMLib.** `SmBrep.cpp:28777 Transform(..., cpOptScale, bScaleTol)`:
- isotropic scaling keeps the analytic types and scales the tolerances;
- non-isotropic scaling converts everything to B-splines first (28800+);
- `Transform(double[4][4])` (28975) is a general affine map.

**geop today.** `Motion` (`core/geop-core-math/src/primitives/pose.rs:714`) is an
isometry only. There is no scale op (common for STEP units and for moulds: shrinkage
compensation).

**Port.**
- Generalise `Motion` to an affine map. A NURBS curve or surface under an affine map is
  exact: map the control points, keep the weights.
- Uniform scale keeps every recognised analytic shape. Non-uniform scale turns
  circles into ellipses: the NURBS stays exact, but `as_cylinder` no longer recognises
  it, which is correct.
- A negative determinant reverses faces (as the mirror already does).
- Pcurves stay as they are, since the parametrisation is unchanged (as in
  `transform_body`).
- Interval widening comes from the rounded matrix, as for rotations.
- Op: `ScaleBody { bodies, about: Point, factors: [Scalar; 3] }` in
  `ops/geop-ops-pattern` beside `MoveBody`. A STEP import "units" option reuses it.
- Refuse a zero factor.

### 2.9 Mesh (STL / triangle bag) to B-rep — P2

**SMLib.**
- `SmBrep.cpp:3128 CreateFromTriangleBag`: one planar face per triangle (3 line
  B-splines, `CreatePlanarFaceWith3DCurves`); a degenerate triangle is silently skipped
  (`continue`). Then `StitchFaces(tol)`.
- `CreateFromPolyBrep` (3073) builds one face per poly face (`MakeFaceFromPolyFace`
  13910), then stitches.
- There is no merging of coplanar triangles and no surface fitting.

**geop today.** Missing; only STL export (`core/geop-core-rasterize`).

**Port** (`ops/geop-ops-step` sibling, or `geop-ops-mesh`: `ImportStl` reading by
extension, `OperationInfo::reads`):
- Build the topology from the mesh's own indexing, not by stitching geometry:
  - weld vertices by exact coordinate equality (STL repeats exact floats);
  - each triangle edge is shared by exactly two triangles, else refuse with the
    triangle numbers (non-manifold or open).
- Each triangle is a planar face with line edges, built with `BodySpec`, exact.
- Degenerate (zero-area) triangles: collapse the edge (squeeze) topologically, report
  the count.
- Then 2.3 merges coplanar neighbours, so a mesh box becomes 6 faces. Coplanarity is
  decided exactly on the vertices' enclosures (the 4th vertex `could_be_equal` to the
  plane). That gives a usable B-rep for booleans.
- Later: fitting cylinders and free-form patches to regions; reverse engineering is out
  of scope.

### 2.10 Replace face with another surface or a sheet — P2

**SMLib.** `ReplaceSurface` (`SmBrep.cpp:26240`):
- swaps the surface, sets the face domain to the new surface's natural domain, deletes
  the pcurves, rebuilds them by projection (`CreateUVTrimCurves`);
- vertices and edges are **not** moved. If the new surface does not pass through the old
  edges, the B-rep is silently invalid.
- `LocalOperation` (2.1) is the version that re-meets.

**Port.** `ReplaceFace { face, with: Face of a sheet }` via the 2.1 builder:
- extend the replacement (`NurbSurface3D::extended`);
- re-meet vertices and edges;
- refuse if a neighbour's edge loses its meeting curve.

### 2.11 Trim surfaces by curves on them (+ split-only) — P3

**SMLib.** `SmBrep.cpp:28988 TrimSurfaces`:
- per surface, `MergeCurvesOnSurface` splits the faces along the 3-D curves (`TRUE` =
  projection for tolerance);
- then `SM_TT_SPLIT` returns; otherwise find the face nearest a reference point
  (`BrepPointSolve`, which may land on a vertex or edge and takes `sFaces[0]`, an
  ambiguous choice);
- collect the connected faces without crossing the new edges;
- keep or delete them.

**geop.** `TrimSurface` (by a face) and `ProjectCurve` (sketch along a normal).

**Port.** Add an "imprint 3-D curves (from a 3-D sketch) onto a sheet" mode to
`ProjectCurve`, keeping the piece a picked face is in:
- choose the piece by the picked *face* (a topological pick), not by the nearest point,
  which avoids SMLib's vertex/edge ambiguity;
- refuse curves that do not lie on the face (`surface_could_contain` at curve points as
  a check, with an enclosure-based check of the curve's hull).

### 2.12 Extend a face of a sheet: generalise `ExtendSurface` — P2

**SMLib.** `CreateExtendedSurface(dist, SM_CT_G1 | C2 | G1R)` via
`N_SrfExtendByDist` (`SmBSplineSurfaceNLib.cpp:4469-4850`). It retries with a smaller
distance while the extension fails (`while(0 != N_SrfExtendByDist(...))` 4568).

**geop.** `ExtendSurface` (`ops/geop-ops-surface/src/extend.rs`) refuses:
- a face of a sheet with other faces (`:172`);
- a face that is not its whole patch (`:179`), e.g. any trimmed face, the common case;
- edges with two faces.

**Port.**
- (a) Trimmed faces. Extend the patch (`NurbSurface3D::extended` on the side the edge
  is nearest in `(u, v)`), then move the picked edge outward. The new boundary is the
  old edge's pcurve offset along the surface: build it as the iso-line at `u + Δ`
  only when the edge is an iso-line; otherwise refuse by name until a
  geodesic-offset pcurve exists.
- (b) Several faces: extend the edges of a chain of free edges together, the shared
  vertices moving along the shared side.
- (c) Linear (ruled) extension as an alternative to natural continuation, for G1 only:
  continue along the cross-boundary derivative. Useful when the natural extension of a
  free-form patch curls over.

### 2.13 Things NOT to port (SMLib bugs and stubs found)

- `SmBrep::ExtendSurfaces` (`SmBrep.cpp:4028`): the condition
  `GetType() != BSpline || GetType() != Offset` is always true, so **every surface is
  skipped and the function is a no-op.**
- `SmBrep::Simplify` / `Refine3DCurves` (3946/3986): its own comment says it "sometimes
  causes bad problems".
- `NSidedPatchRebuild` (`SmFeatureExecutive.cpp:2223`): "Does not work".
- `DeleteFilletFace`: 2° tangency sample (`SmFilletRemoval.cpp:224`), corner
  averaging (522).
- Heal stubs and the disabled `Fix_BadLoops`; `Fix_UncontainedEdges` edits a cache only.
- Bend / twist volume bugs: §2.7.
- `CreateFromTriangleBag` silently skipping degenerate triangles.
- Stitch's sticky candidate flags (`SmStitch.cpp:1423`); `SmGap.cpp:274/246/282`.

### 2.14 Split a face at its surface's C0/C1 discontinuities — P3

**SMLib.** `SubdivideFaceAtDiscontinuities(face, SM_CT_C0|C1|C2)` (`SmBrep.cpp:18366`)
splits faces along the iso-lines at multiple knots.
`FixAllFaceDiscontinuities` (17702) / `FixAllRepeatedEndControlPoints`.

**geop.** Fillet puts stations at kinks; STEP import takes B-splines as they are. A
face with a C0 crease inside it confuses tangency questions (fillet, `meet`'s
`tangent_groups`).

**Port.** A topology helper `Model::split_face_at_knots(face, continuity)`:
- knots of multiplicity ≥ degree (C0) are found exactly from the knot vector;
- split along the iso-line with `splice_edge_into_face`; the iso-line pcurve is exact.

Run it in STEP import for faces whose patch has interior C0 knots.

### 2.15 Per-entity attributes propagated through edits — P3

**SMLib.** `SmAttribute` (`inc/SmAttribute.h:213-520`) has behaviours `COPY`,
`REFERENCE`, `STANDALONE_*` and `TEMP`. Callbacks on Split, Merge, Coincident, Copy
and Edit are driven by `Notify` in every edit function (a table of 17 events).

**geop.** Stable names already propagate through split, merge and trace (`naming.rs`);
colour is a part-level property.

**Port.** Attributes keyed by *name* in `NamedModel` (face colour, material, tags,
STEP `STYLED_ITEM` colours), resolved through the same name origins (`face_origin`,
`edge_origin`), so a split face's halves inherit their parent's attributes. No
callback zoo is needed: inheritance falls out of naming. Read colours on STEP import.

**Round 3: skipped, not small.** `face_origin` and `edge_origin` live only for the
length of one boolean (`BooleanNaming`); afterwards a face's ancestry is only in the
arguments of its name, and a name's argument longer than `LONGEST_ARGUMENT` is spelled
by its digest, so a face cut often enough loses it. Doing it properly means: a target of
attributes by name, a step to set them (a picker and a colour field), ancestry recorded
at the split rather than parsed back out of names, a colour per face in the scene
protocol (`ViewFace`) and the web renderer's materials, and STEP `STYLED_ITEM` read on
import. Each is modest; together they are a feature of their own.

### 2.16 Topology change tracking — borrow one idea

`SmTrackTopologyChanges` keeps removed, added and changed lists across one atomic edit
(`SmTrackTopologyChanges.cpp:189`). SMLib's healer uses it to re-heal **only new
entities** (`UpdateTargetLists`, `SmHealData.cpp:1067`).

geop's booleans already avoid rescans ("a pair once found clean stays clean"). For the
healing in 2.4 and 2.5, return from each fix the list of entities it created (as
`naming.rs` already knows them) and run the next checks only on those.

---

## 3. Missing whole features (summary work order)

1. **Push-pull / move / offset face in place** (`MoveFace`, `OffsetFace`): 2.1.
   Machinery exists in draft and shell; generalise it into a lib.
2. **Delete face with heal** (open / cap / extend; inner-loop case plus crossing case,
   fillet and chamfer removal): 2.2.
3. **Merge faces that lie on one surface, and join smooth edges after booleans**
   (simplify): 2.3. It also fixes the AGENTS-documented workaround in fillet.
4. **Knit with an explicit gap, T-junction splitting, sliver and short-edge squeeze,
   and re-meeting**: 2.4. Move STEP's `heal` into a lib to share it.
5. **Import healing: duplicate vertices, zero-length edges, sliver faces, multi-outer-loop
   faces, missing pole vertices**: 2.5.
6. **Defeature regions of dumb solids** (and extract the feature as a solid): 2.6.
7. **Scale bodies** (uniform and non-uniform; STEP unit conversion): 2.8.
8. **Global deformation: bend, twist, taper, unbend about an axis**, exact where the
   shapes allow, with pcurves kept: 2.7.
9. **Mesh (STL) to B-rep**, topology from indices, coplanar merge: 2.9.
10. **Replace face**: 2.10. **Extend trimmed and multi-face sheets**: 2.12.
11. **Tapered (drafted) extrude**: `CreateTaperExtrude` equivalent, with corners as
    linear extension or a G1 fillet. Today only sweep tapers.
12. **Draft beyond planes next to planes or cylinders**: drafting cylinders and cones
    (draw a cone), drafting next to fillets (move the fillet: the rolling ball re-rolled
    between the tilted faces), faces parallel to the neutral plane (parting-line draft).
    SMLib has none of this; it is listed for completeness.
13. **Split a face at C0 knots** (2.14) and **per-entity attributes** (2.15).

### Where geop is already better

- STEP healing moves geometry onto where the surfaces meet and keeps honest enclosures.
  SMLib only grows tolerances, and its geometric `RefineGeometry` is never called.
- Draft and shell rebuild with exact topology and names. SMLib's `ReplaceSurface`
  leaves vertices where they were.
- Mirror keeps pcurves exact (`transform_body`). SMLib re-projects them.
- Knit's edge matching is a topological test (same vertices) plus curve containment.
  SMLib uses 5-sample max distance and orientation from one tangent dot product.
- Validation after every operation. SMLib's healer silently succeeds on failed fits,
  and `DoSpaceBend` ignores the approximation status.
- Refusals name entities and conditions; SMLib asserts or returns `SM_ERR`.

### Open `#[ignore = "bug: ..."]` tests relevant to this area

- `ops/geop-ops-step/src/regression_tests.rs:81` (linear pattern of overlapping blocks,
  STEP round trip) and `:113` (mirrored L bracket with a slot). Both probably involve
  faces a union left in two on one plane; 2.3 likely helps.
- `ops/geop-ops-shell/src/tutorial_regression_tests.rs:59`: the shell refuses the fillet's
  traced quarter-circle edges. Same family as drafting next to fillets: direct edits need
  arcs recognised (`as_circle`), not traced.
- `ops/geop-ops-boolean/src/regression_tests.rs:123`: tangent block on the cylinder's
  seam gives an edge with 4 coedges. A merge of faces on one surface (2.3) must respect
  geop's seam rule.
- `ops/geop-ops-extrude-revolve/src/regression_tests.rs:842`: a non-manifold result
  instead of a refusal. A knit or heal step must never produce 4-coedge edges either.
