# Booleans and what lies under them: SMLib vs geop

Scope: surface/surface intersection (SSI), topological intersection/imprint, classification, boolean assembly, post-processing, cutting/splitting/sectioning.

Paths:
- SMLib: `/home/tobiasjacob/usd-brep/source/SMLib/src` (abbreviations: **SI** `SmSurfaceIntersector.cpp`, **ADV** `SmAdvSurfaceIntersector.cpp`, **TI** `SmTopologyIntersector.cpp`, **PT** `SmProtoTopology.cpp`, **MG** `SmMerge.cpp`, **BR** `SmBrep.cpp`).
- geop: `/home/tobiasjacob/geop`. **XF** = `lib/geop-lib-booleans/src/remesh/remesh_edges_x_faces.rs`, **BO** = `lib/geop-lib-booleans/src/boolean.rs`.

## 0. How the two are built, in brief

**SMLib** runs every NURBS boolean through `SmMerge::ManifoldBoolean` (MG:2510–5063):
1. `IntersectInsertRelate` (TI:1092) runs a full SSI for every face pair (`FaceIntersect`). The SSI finds start points on the boundary iso-curves and closed loops by subdividing into Bezier leaf pairs; it classifies tangent and singular points by Dupin indicatrices; and it dispatches to analytic routines for plane, cylinder, cone, sphere, extrusion and revolution.
2. Results are collected as "ProtoTopology" without editing the Breps. Clusters are then resolved by tolerance capture and imprinted.
3. Gaps in intersection loops are repaired by tolerance (`CloseIntersectionLoop`, `FixGaps`).
4. Faces are classified: radial sectors at shared edges, then a flood fill of connected face sets with one ray per set. Regions get void/solid flags, so nested shells and lumps come out right.
5. `BooleanPostProcess` removes "topological" edges and vertices, merging faces that share a surface.

Everything is tolerance-driven: capture distance max(0.01, 2000·tol) at TI:1154, 5·tol combine factors at PT:6372, 20° angle tolerances, a 5° skip for near-tangent interior seeds, 6 axis-aligned rays.

**geop** has no standalone SSI. `remesh` (`lib/geop-lib-booleans/src/remesh/mod.rs:142`) works in increasing dimension:
1. vertex×vertex merge;
2. tangent branch points on edges;
3. vertex×edge;
4. edge×edge;
5. edge×face, in this order: split piercings (XF:304), imprint coincident edges (XF:554), find start points at piercing vertices (XF:717), trace face×face curves from those vertices by predictor–corrector (XF:1270) and fit a cubic (XF:1741). Plane × surface-of-revolution about its normal is the only case that gives an exact arc (`circle_through`, `core/geop-core-geometry/src/shape.rs:695`).

Then `classify_face` (BO:657) tests every interior point it is offered against the other solid by ray parity (`core/geop-core-topology/src/contains/shell.rs`). A keep/drop table follows (BO:83), then `check_closed` (BO:583), then `assemble_solid`, which puts **one shell** around all kept faces (`core/geop-core-topology/src/edit/assemble_solid.rs:53–97`). There is no post-processing.

Everything is an interval enclosure; nothing is captured by distance.

Consequence: everything SMLib finds from a **face pair** alone — interior loops, isolated singular points, curves along which nothing pierces — geop can only find if an **edge** leads to it.

---

## 1. Feature table

| Capability | SMLib (file / function) | geop (crate / function, or "missing") | Verdict |
|---|---|---|---|
| Standalone surface/surface intersection API | `SmSurface::GlobalSurfaceIntersect` (SmSurface.cpp:20375) → `SmAdvSurfaceIntersector::DoIntersection` (SI:6106) | missing: only marching from edge piercings, inside `remesh` (XF:1134 `trace_from_start_point`) | SMLib better |
| Start points on patch boundaries | `FindBoundaryStartPoints` (SI:1078), boundary iso-curves × surface (SI:1147–1346) | trimmed edges × faces (`find_piercing_crossing`, XF:304), then a vertex scan (XF:717) | par: geop uses the trimmed boundary, which is the right one for B-reps |
| Closed loops that touch no boundary | ADV `FindInteriorCurves` (ADV:1139): leaf-pair seeds; skips pairs whose normals are within 5° | **missing** | SMLib better |
| Exclusion test that proves no loop exists (Gauss map / normal cone) | `BranchIsGaussSeparate` (ADV:1713), dead code (ADV:1383/1401) | missing | both missing |
| Tangent-point / tangent-curve / singular-point classification | ADV `ComputeSingularityPoint` (ADV:116–966): principal curvatures, Dupin conics, 0/2/4+ directions | only along an existing edge (`remesh_tangent_branches.rs`, mean-curvature difference); interior start with parallel normals is skipped (XF:1338) | SMLib better |
| Curves crossing each other (branch nodes) | `SplitIntersectingProtoEdges` (PT:6656) | only where a node falls on an existing vertex or seam | SMLib better |
| Pole handling in SSI | start points on poles skipped (SI:1464–1478); `StepBackFromSingularity` (SI:2322) | `split_pole_row`, `NurbSurface::project` `off_pole`, `circle_through` refuses the axis | geop better |
| Cone apex as a trace start | skipped like a pole | skipped, but traced from the other end (XF:1316–1328) | geop better (fails only if both ends are apexes) |
| Analytic: plane∩plane | `SmPlane::IntersectWithPlane` (SmPlane.cpp:2095) → line | traced and fitted with a cubic | SMLib better |
| Analytic: plane∩cylinder/cone (lines, circle, ellipse) | `SmCone::IntersectWithPlane` (SmCone.cpp:3219) | circle only when the plane is ⟂ to the axis of a surface the net shows to be a revolution (`circle_through`); lines and ellipses are traced | SMLib better |
| Analytic: sphere∩plane/sphere/coaxial cone | SmSphere.cpp:1425/1613/2381 | sphere∩plane gives a circle via `circle_through`; sphere∩sphere is traced | SMLib better |
| Analytic: cyl∩cyl with parallel axes → lines; equal-radius crossing → ellipses | `SmCone::IntersectWithCone` (SmCone.cpp:2604) | traced | SMLib better |
| Analytic: extrusion∩extrusion (parallel directions), coaxial revolutions | SmSurfOfExtrusion.cpp:3314, SmSurfOfRevolution.cpp:4032–4107 | traced | SMLib better |
| Output curve | piecewise C0 cubic Hermite, 3D + 2 UV curves with **shared knots** (`sm_FlushCrv` SI:5576) | interval cubic fit, width-adaptive (`fit_traced_branch` XF:1741); pcurves fitted afterwards | geop better (honest enclosure, drift controlled) |
| Step control | trial step ×1.5 / ÷1.5 with chord and angle check (SI:2937, 3433, 5134–5389) | curvature-limited stride + width-driven leg refinement (XF:156) | par |
| Point × Brep, curve × Brep, isolated shell vertices, wire edges | `sm_IntersectShellVertices` (TI:1354), `sm_IntersectWireLaminaEdges` (TI:1447) | `remesh` takes `Body` (solid or sheet), not wires | SMLib better |
| Edge×edge coincidence and crossing | ProtoEdge overlap groups (PT:6995) | `remesh_edges_x_edges.rs` (merge/split) | par |
| Edge lying in face (imprint) | `EdgeFaceCoincidence` (TI:7752), `MergeCoincidentSurfaces` (TI:4347) | `find_coincident_pair` / `imprint_coincident_edge` (XF:554/646) | par |
| Coincident faces, same or opposite normal | sector classification + region void flags (MG:3230–3320), `FixFoundFaceCoincidence` (MG:2129) | `OnSameNormal` / `OnOppositeNormal` from normals at an interior point (BO:657–738) | par (geop simpler and exact) |
| Same curve found from several face pairs (dedupe) | `FindProtoEdge` / `sm_AreSameCurve` (PT:4843, PT:279), `CanCombineEdgeDefinitions` (PT:6509) | `edge_is_boundary_of_face` + "an existing edge joins the same two vertices"; **fails along a cylinder seam** (ignored test) | SMLib better |
| Degenerate (zero-length) intersection curve → vertex | PT:4751; short closed ProtoEdge dropped (PT:3405) | not handled explicitly; a trace shorter than a stride is handled by a half step (XF:1352) | SMLib better |
| Isolated touching point → shared vertex | PT TI:1972–2021 makes a ProtoVertex | no vertex; classification tolerates it (`touching_spheres_union` etc.) | par for solids; SMLib better if a non-manifold vertex is wanted |
| Gap repair in intersection loops | `CloseIntersectionLoop` (TI:6108), `FixGaps` (TI:632) by tolerance | none; `check_closed` errors with names (BO:583) | geop better (no tolerance squeeze); missing an early loop-parity check |
| Face classification | radial sector at shared edges (SmEdge.cpp:1789 `FindRadialSector`), flood fill of connected sets (MG:3330…), one ray per set along ±X/±Y/±Z (BR:24687) | every interior point of every face by random-ray parity, with a nearest-classified short path (`shell_contains_from`); disagreement is an error | geop more robust; SMLib does less work |
| Lumps / voids / nested shells | regions with void flags, ancestry (MG:4455–4915), `SetRegionIsVoidFlagsForNestedSolids` (BR:9190) | **one shell** for all kept faces (assemble_solid.rs:86–97) | SMLib better |
| Non-manifold results | `NonManifoldBoolean` (MG:2401), spine edges allowed; edges left without faces become wires (TI:456) | `check_closed` accepts 4 uses (BO:577); `validate_fast` does not run the manifold check; full validation rejects n≠2 (manifold_check.rs:23) | SMLib better (geop is inconsistent) |
| Post-process: remove edges/vertices between faces on one surface | `BooleanPostProcess` (MG:827), `SmEdge::IsTopological` (SmEdge.cpp:4986), `RemoveTopologicalEdgesAndVertices` (BR:25658) | **missing** | SMLib better |
| Far from the origin | `SmTemporaryBrepTransform` (MG:2338): moves to the origin if \|centre\|²>1e6 | evaluation relative to the patch (`spline::centered`) | geop better |
| Parallelism / acceleration | surface trees, TBB (TI:1128, 1657), bounding-box culls (TI:4898) | per-pair `curve_could_meet_aabb` (curve_surface.rs:337); no spatial index; O(V²), O(E·F) rescans | SMLib better |
| Registered or suppressed SSI pairs | `RegisterExistingSSI` (TI:265) | missing (geop does have `settled` pairs inside one piercing phase, XF:304) | SMLib better (minor) |
| Union / intersection / difference | `SM_BO_UNION/INTERSECTION/DIFFERENCE` | `BooleanOp` (`ops/geop-ops-model/src/feature.rs:33`) | par |
| XOR | `SM_BO_EXCLUSIVE_OR` | missing | SMLib |
| Imprint body on body (keep both, only split) | `SM_BO_IMPRINT`, `m_bImprinting` (MG:3104) | `imprint` (`lib/geop-lib-booleans/src/imprint.rs`): `remesh` alone, both bodies kept, each its own copy of the curves; `imprint_on_face` for one face, any body as the tool; the `Imprint` op (`ops/geop-ops-boolean`) | par (geop splits both bodies; SMLib only A) |
| Imprint and classify (report what a boolean would drop) | `SM_BO_IMPRINT_CLASSIFY` (MG:4060) | missing | SMLib |
| Merge (cellular, keep every piece) | `SM_BO_MERGE`, `PARTIAL_MERGE`, `PiecewiseMerge` (MG:341) | missing | SMLib |
| Extract separate pieces | `SM_BO_EXTRACT_SEPARATE` (MG:5005), `CutBreps` (BR:38618) | `split` (split.rs) for a sheet tool only | SMLib better |
| Slice / split by a sheet | `SM_BO_SLICE`, `SmSurfaceBiLinearCutter` (SmCutter.cpp:726) | `split` (split.rs:52), **fails on a ring section** (ignored test) | par (both have bugs) |
| Cut by a plane | `SmPlaneCutter` (SmCutter.cpp:47), `SmApiCut` | `section_part` in the drawing (box difference, `ops/geop-ops-drawing/src/section.rs`); `split` with a planar sheet | par |
| Section curves (Brep ∩ Brep as wires) | `SmBrep::BrepIntersect` (BR:890) | `section`, `section_by_plane` (`lib/geop-lib-booleans/src/section.rs`): copies remeshed, the shared edges built as a wire; the `Section` op, body × body or body × datum plane; a wire edge picked as a path is its run (`Model::wire_chains`) | par |
| Sheet × sheet keeps the wire | MG:4689–4712 | `trim_surface` uses `remesh` between sheets (`ops/geop-ops-surface/src/trim.rs:203`); the `Section` op of two sheets gives the wire | par |
| Wire × solid boolean | `ProcessWiresAndShellVertices` (MG:7239) | missing | SMLib |
| N-ary / CSG tree boolean | `merge_breps` (MG:1343), `BooleanTrees` (MG:1820), `BooleanLists` (MG:1431) | `combine_tools` (combine.rs) folds tools one after another; `Boolean` op is two solids | SMLib better |
| Cookie cutter (remove A only, do not add B) | `m_bCookieCutter` (MG:4226) | missing | SMLib |
| 2D coplanar sheet boolean | `Boolean2D` (SmPrimitiveCreation.cpp:7588) | not in booleans (sketch regions are a separate system; not surveyed) | n/a |
| Up to next | — | `boolean_up_to_next`, `piece_up_to_next`, `trim_up_to_next` | geop only |
| Persistent naming of the result | relations / attributes (`Relate`, TI:5275) | full naming scheme (`naming.rs`, `Origins`) | geop better |

---

## 2. Special cases to port, by how often a user hits them

### P1 — users hit these in ordinary modelling

**1. Closed intersection loops that no edge pierces**
- **SMLib:**
  - ADV:1139 `SmAdvSurfaceIntersector::FindInteriorCurves`, with `SolveBranch` (ADV:1380) and `LocalSolve` (ADV:1515).
  - Both surfaces are subdivided into Bezier leaf pairs. Each intersecting pair is seeded from the patch centres with `LocalSurfaceIntersect` and traced both ways (SI:6058).
  - Leaf pairs whose normals are within 5° are skipped (ADV:1175–1186), so near-tangent small loops are missed by design.
  - The basic variant is SI:5935.
- **geop today:**
  - Start points come only from vertices where an edge of one body lies on a face of the other (`find_tracing_start_points`, XF:717). The module doc says outright that tracing "does not create new ones" (XF:218–229). I checked: `geop-core-geometry/src/intersection/` has curve×curve and curve×surface only, no surface×surface.
  - So a bump on a lofted, subd, boundary or network face poking through a plane, or two freeform faces meeting in an oval inside both, is never imprinted. `classify_face` then reports "straddles" (BO:704).
  - The ignored `rounding_the_trailing_edge_of_a_lofted_blade` (`ops/geop-ops-fillet/src/regression_tests.rs:468`) fails with exactly that error ("misses an intersection curve on the first span's face"). It is worth checking whether that is this case.
  - Analytic shapes mostly escape it only because geop splits periodic faces into sectors (quarter cylinders; the STEP import removes seams, `ops/geop-ops-step/src/import/body.rs:9`), so a seam edge nearly always pierces.
- **How to do it in interval style:**
  - Every closed loop of `S1(u,v)=S2(s,t)` has an extreme point in any fixed direction `d`. At that point `(n1×n2)·d = 0`.
  - Solve the square system `S1−S2 = 0` (3 equations) plus `(n1×n2)·d = 0` (1 equation) in 4 unknowns over the face pair. Use the existing subdivision + Newton/Krawczyk machinery: isolate by subdivision, refine by Newton, keep the unsharpened last step.
  - Every loop then provably yields ≥2 isolated points. A box where the system cannot be proven regular is a tangency (case 4) and must be reported, not skipped as SMLib does.
  - Choose `d` as a free choice that keeps the system well conditioned, e.g. away from both surfaces' average normal.
  - Exclude a box pair cheaply first: disjoint AABBs, then disjoint normal cones (SMLib's dead Gauss-map test, ADV:1713). If both normal cones exclude ±d-extremes, the pair has no loop extreme.
  - Insert each found point as a vertex in the interior of both faces. Splicing a closed loop needs `splice_edge_into_face` to accept a curve from a face-interior vertex back to itself as a new hole, or two vertices (the `d`-min and `d`-max), giving two edges. Then trace from those vertices with the existing `trace_one_side`.
  - Only run it for pairs where some edge×face test did not already produce a curve through the box, so the cost is limited to the pairs that need it.

**2. Exact curves for analytic pairs (lines, circles, ellipses)**
- **SMLib** dispatches before the marcher:
  - `SmPlane::GlobalSurfaceIntersect` (SmPlane.cpp:1942): plane∩plane is a line (2095).
  - `SmCone::IntersectWithPlane` (SmCone.cpp:3219): lines for a plane ∥ axis, a circle for a plane ⟂ axis, an ellipse from extremum points.
  - `SmCone::IntersectWithCone` (2604): parallel cylinders give lines; equal radii with crossing axes give ellipses.
  - `SmSphere` 1425/1613: sphere∩plane and sphere∩sphere give circles.
  - `SmSurfOfExtrusion` 1870/3164/3314 and `SmSurfOfRevolution` 3020–4107: coaxial revolutions give circles.
  - Coincident regions come back as a REGION_BOUNDARY outline.
- **geop today:**
  - Only `circle_through` (shape.rs:695) is exact: a plane × a surface the net shows to be a revolution about the plane normal. Every other pair is a marched cubic (XF:1741).
  - Two ignored tests show what this costs downstream:
    - `diameter_of_a_hole_extruded_through_all` (`ops/geop-ops-drawing/src/regression_tests.rs:42`): the rims are traced, so the drawing's diameter dimension refuses them.
    - `shell_of_a_box_with_rounded_vertical_edges` (`ops/geop-ops-shell/src/tutorial_regression_tests.rs:59`): the quarter circles are traced, so the shell cannot offset them.
  - Width also compounds along traced edges (AGENTS "Many booleans").
- **How:**
  - Extend `circle_through`'s pattern into one `exact_branch(surf_a, surf_b, point) -> Option<NurbCurve>`, using the shape recognisers already in shape.rs (`as_plane`, `as_cylinder`, `axis_of_revolution`, `as_line`, `as_arc`), each decided from the control net:
    - plane∩plane → a line along `n1×n2` (`Plane::intersect_plane` already exists, shape.rs:388);
    - plane ∥ cylinder axis → a generator line;
    - plane oblique to a cylinder → an ellipse as a rational quadratic, from the axis, radius and plane;
    - two coaxial revolutions, or sphere∩sphere → a circle about the shared axis;
    - parallel cylinders → lines ∥ axis.
  - Accept the curve only if the net-recognition is certain under the intervals. Keep the trace's job of deciding *where* the branch ends; the exact curve replaces only the fit.
  - Verify that both ends lie on the curve (`curve_could_contain`) rather than trusting the formula. That is the same contract as `circle_through`.

**3. Merging faces that share a surface after a boolean (redundant edges and vertices)**
- **SMLib:**
  - `SmMerge::BooleanPostProcess` (MG:827, on by default, checks intersection edges only, MG:4971).
  - `SmEdge::IsTopological` (SmEdge.cpp:4986): a manifold edge, tangent normals, not a seam, one surface able to cover the other. The edge is then deleted and the faces merged; degree-2 vertices are removed (`DeleteTopologicalVertex`).
  - Standalone: `RemoveTopologicalEdgesAndVertices` (BR:25658).
- **geop today:** missing; I found no face-merge or edge-dissolve in `core/geop-core-topology/src/edit/`.
  - The union of two boxes sharing a face (`cubes_sharing_a_face_union`, BO:1413), or a boss on a plate flush with a side, leaves the flush side split into two faces along an edge that bounds nothing.
  - Each later boolean pays for those faces and edges again. That is quadratic growth in faces for patterns and gears, and the split edges widen (AGENTS "Width along a split edge").
- **How (exact and topological, no tolerance):**
  - `Origins` (`remesh/mod.rs:202`) already records which original face every piece came from.
  - Two kept faces sharing an edge can be merged without any test when (a) they have the same origin face, so the *identical* surface object, and (b) they have the same orientation after `Keep::Reversed`. Delete the edge, join the loops, keep the surface.
  - Across two different source faces, merge only if their surfaces are provably the same surface: identical control nets after a knot-compatible comparison, or one covering the other via `surface_could_contain` of the other's net with a covering domain. Otherwise leave the edge. Never merge on "tangent within tolerance".
  - Afterwards, a vertex with exactly two edges, both pieces of the same origin edge, and the same two faces on both sides, can be dissolved by restoring the origin curve's sub-range. Do not re-fit.
  - This reduces width rather than adding it.
  - Put it in `lib/geop-lib-booleans` as a final step of `combine`, before `assemble_solid`. Naming: the merged face keeps the name of the earlier piece.

**4. A curve found twice along a tangent periodic seam (four coedges on one edge)**
- **SMLib:**
  - `FindProtoEdge` (PT:4843) with `IsSameProtoEdge` (PT:2889) / `sm_AreSameCurve` (PT:279), comparing end ProtoVertices plus a midpoint drop.
  - `CanCombineEdgeDefinitions` (PT:6509) keeps one definition and prefers the lower-dimensional source (an existing edge over a face curve).
  - `ResolveOverlappingProtoEdges` (PT:6995) rebuilds overlapping groups.
- **geop today:**
  - Ignored test `union_of_a_cylinder_tangent_to_a_block_along_its_seam` (`ops/geop-ops-boolean/src/regression_tests.rs:123`): the seam `cylinder(boss,p3)` ends up with 4 coedges.
  - Related: ignored `cut_tangent_to_a_side_face_is_not_refused` (`ops/geop-ops-extrude-revolve/src/regression_tests.rs:842`).
  - Here the block face is tangent to the cylinder along the seam line. The seam edge lies *in* the block face, so it is imprinted as coincident (XF:554). The block face is also split, and classification keeps both sides' copies.
- **How:**
  - The seam lies in the plane, and both quarters `c2` and `c3` touch the plane along it from the same side. The plane faces on either side of the imprinted seam therefore see the cylinder on one side only.
  - The defect is in the keep table: an edge that is an imprint of a tangent contact (the other body's faces on both sides of it are tangent to this face) must not be used by kept faces of *both* bodies.
  - Concretely:
    - after remesh, for each imprinted coincident edge, record whether its own adjacent faces lie tangentially on the face it was imprinted into (normals parallel over the edge, decided as intervals);
    - in `check_closed` and assembly, treat a 4-use edge whose four faces pair up as two tangent pairs as a non-manifold *touch*;
    - either refuse it by name (AGENTS "Refuse what you do not support") or, for a union, keep one manifold pairing: plane face with the cylinder face on its side, chosen by the radial order of the faces around the edge. That radial order is SMLib's `FindRadialSector` (SmEdge.cpp:1789) with `RadialSectorClassify` (SmEdge.cpp:1050).
  - In geop, radial order is an exact question once the faces' normal-plane directions around the edge are enclosed: sort the tangent half-planes by angle, and where two are tangent compare curvatures. That is the same quantity `remesh_tangent_branches.rs` computes.

**5. Split or section through a hole: a section with several loops**
- **SMLib:**
  - Imprinting is per curve (`EdgeDefinition::Imprint` PT:1369 → `MakeEdgeInFace`). After a face split, pending curves are re-sorted into the child faces (`UpdateFaces` PT:5287, `sm_ClassifyParamOnFaces` PT:189). A ring section is simply two closed edges in one face.
- **geop today:**
  - Ignored test `split_a_block_across_its_through_hole` (`ops/geop-ops-boolean/tutorial_regression_tests.rs:50`): `split` takes the inner loop for "the face ends inside the solid".
- **How:** in `split.rs`, decide "ends inside the solid" per *edge of the cut face* by whether the solid's boundary runs along it. Every edge where the sheet crosses the solid is shared with a solid face, so an edge of the inner loop is such an edge too. This is topological and needs no geometry. The pieces are then the union-find over face sides already described in split.rs:1–17.

**6. Lumps and voids: one shell is not a solid**
- **SMLib:**
  - Regions carry void/solid flags with ancestry from both inputs (MG:4455–4606). The new infinite region is chosen from regions whose sources were both infinite (MG:4621–4672). Void/solid is recomputed per op (MG:4805–4892).
  - `SetRegionIsVoidFlagsForNestedSolids` (BR:9190) alternates void and solid by nesting.
  - `CreateOneBrepPerBody` and `EXTRACT_SEPARATE` give one Brep per connected set.
- **geop today:**
  - `assemble_shell` puts every kept face into one `Shell` (assemble_solid.rs:86–97), although `Solid` documents "subsequent shells are void shells" (`core/geop-core-topology/src/solid.rs:6`).
  - A difference that cuts a bar in two gives one solid whose single shell has two components. A cube minus an inner sphere gives one shell with the outer and inner faces mixed. Downstream (mass, shell, fillet, pattern, naming of pieces) sees one body.
- **How:**
  - After the keep decisions, take the connected components of kept faces (union-find over shared edges, as in split.rs).
  - Nest them exactly: for components P and Q, P is inside Q iff a vertex of P classifies `Inside` Q's shell (`shell_contains`). Vertices are free choices; take one not on Q.
  - Outer shells, those inside no other or inside an even number, start a solid each. A shell inside exactly one outer shell is a void of it.
  - Return `Vec<SolidId>` (lumps) from `boolean`, named `boolean(B)`, `boolean(B,1)`, …, ordered by smallest kept face name, as split.rs does.

**7. Non-manifold contacts (shared edge or vertex) must be one policy**
- **SMLib:** `ManifoldBoolean` vs `NonManifoldBoolean` (MG:2401); the API dispatches on `IsManifoldSolid()` (`SM_API/src/SmApiBrep.cpp:208–237`). Spine edges are allowed in non-manifold mode. Faces deleted next to a kept wire edge leave it as a wire (TI:456).
- **geop today:**
  - `check_closed` explicitly accepts an edge used 4 times (BO:577–581). `cubes_sharing_an_edge_union` (BO:1421) passes because `validate_fast` does not run `manifold_check` (`core/geop-core-topology/src/validation/mod.rs:151–160`). The full validation would reject the same result (manifold_check.rs:23).
  - So the result is accepted or refused depending on which validator runs.
- **How:** decide once.
  - Either refuse by name in `combine` (edge, the four faces, "the bodies touch along this edge; geop builds manifold solids only"),
  - or make the 4-use edge a supported non-manifold edge. Then `shell_contains` has to count such an edge consistently: a ray through it is already retried, so it is mostly the validator's rule.
  - Given AGENTS' preference, refuse first, early, by name, and make case 4 use the same check.

### P2 — common in mechanical parts with tangencies and freeform shapes

**8. Interior tangent and singular points of face×face (branch nodes)**
- **SMLib:**
  - ADV:116 `ComputeSingularityPoint` classifies each surface point as planar, elliptic, hyperbolic or cylindrical (ADV:150–155):
    - plane vs cylindrical → TANGENT_CURVE along the zero-curvature direction (ADV:229);
    - plane vs saddle → SINGULARITY with 4 asymptotic directions (ADV:251);
    - umbilic → coincidence or tangent curve (ADV:375–441);
    - general case: Dupin conics of both surfaces intersected (ADV:604–685); 0 directions = tangent point, 2 = tangent curve, more = singular (ADV:731–966).
  - Crossing curves are split at their crossing (`SplitIntersectingProtoEdges` PT:6656).
  - Known gap: only one branch is traced from a singular start point (ADV:868, SI:1499–1521, the torus–cylinder example with 6 branches).
- **geop today:**
  - A start point where `n_a × n_b` cannot be normalised returns without tracing (XF:1331–1340).
  - The branch-point theory exists only *along an existing edge* (`remesh_tangent_branches.rs:1–22`).
  - Two equal-radius cylinders with crossing axes work (`steinmetz_cylinders_*`, BO:1689) only because the crossing points fall on the quarter seams (the same holds for `cube_minus_two_inscribed_bores`). Rotate one cylinder 45° about its axis and the node lies inside a face. The march then reaches a point where the corrector is singular, and nothing inserts a vertex there.
- **How:**
  - Find the nodes first, as vertices: the points of S1=S2 where `n1 ∥ n2`. That is 3 equations from S1=S2 plus 2 from n1×n2 expressed in a tangent frame of S1, in 4 unknowns. It is over-determined, so use subdivision to isolate the boxes where all five can vanish, then Newton on `S1−S2=0, (n1×n2)·t1=0` with `t1` a free tangent of S1 (choose the well-conditioned one).
  - Insert the nodes as vertices before tracing. `candidate_within` then stops traces at them automatically.
  - From each node, leave along the directions where the difference of the second fundamental forms (`II_2 − II_1` in the common tangent plane) vanishes. Solve the quadratic form exactly: two directions if indefinite, a tangent curve if rank 1, an isolated touch if definite.
  - Trace every direction, not one: that is SMLib's admitted gap. A direction whose sign is undecidable under intervals is an error naming the node. Do not guess.

**9. Tangential edge×face contact that is still a crossing**
- **SMLib:** `ClassifyBoundaryStartPointIntersection` (SI:8127). If the boundary is tangent to the other surface within 2° (SI:8198–8236), it walks the gap function on both sides with ×5 steps (SI:8393) and tests zero, monotonic, diverging or open. Diverging on both sides means a touching point; otherwise it is traced. Boundary-tangent start points are traced in both directions (SI:6540–6567).
- **geop today:** a crossing whose curve tangent could be ⟂ to the face normal anywhere in the search box is skipped as "not a piercing" (XF:348–375, the `could_be_equal(Scalar::ZERO)` check). The coincidence phase handles it only when the edge lies *in* the face, and tangent branches only handle the along-edge case. An edge tangent to a face at an isolated point *where the faces still cross*, an odd-order contact, is lost: an S-shaped edge of a freeform face kissing a plane, or a circle tangent to a plane boundary while its face crosses the plane.
- **How:** for such a box, evaluate the signed distance of the edge from the surface, `d(t) = (C(t) − S(foot))·n`, on sharp parameters just outside the box on both sides. Those parameters are free choices, so sharpening them is legitimate.
  - Signs definitely opposite: an odd-order crossing. Refine `t` by bisection on the sign change (`solvers::bisect::sign_changes` is already used in `remesh_tangent_branches.rs`), split there, and record it as a start point. Leave the direction to case 8's node logic when the normals are parallel.
  - Same sign: a touch, skip it as today.
  - Undecidable: an error naming the edge and face.

**10. Zero-length and sub-stride intersection curves**
- **SMLib:**
  - A degenerate classification curve becomes a ProtoVertex (PT:4751).
  - A closed ProtoEdge shorter than 2·tol is dropped; a long one whose ends are >2·tol apart gets a second vertex (PT:3405–3467).
  - An interval whose start, mid and end classify to one vertex is dropped (PT:4833).
  - Degenerate point curves on other curves are removed (SI:6637–6673).
- **geop today:** the half step for a vertex nearer than a stride (XF:1352–1365); no general rule. The ignored fillet tests in `.claude/worktrees/...` (faces with no interior point, `fillet_of_a_hole_rim_tiny_against_its_face`) are in the same family on the main branch's sibling worktrees.
- **How:** not by length thresholds.
  - A traced curve whose two end vertices are the same vertex (`could_be_equal` points merged by `remesh_vertices`) is either a closed loop or nothing. Decide by whether any marched point is definitely off the vertex.
  - A curve between two distinct vertices is always kept. If a face then has no interior point, that is a width problem to trace (AGENTS rules), not to filter.

**11. Check that intersection loops close, right after remesh**
- **SMLib:** single-vertex detection: count shared edges per shared vertex, a closed edge counting twice (MG:2909–3002); `AreIntersectionLoopsClosed` (TI:6485); `m_bIntersectionLoopClosed` status (MG:3044–3096). It then repairs with tolerance (`FixGaps`); **do not port the repair.**
- **geop today:** a missing curve surfaces only later, as `classify_face` "straddles" (BO:704) or `check_closed` (BO:583).
- **How:** at the end of `remesh_edges_x_faces`, for every vertex created as a piercing, or lying on faces of both bodies, count the remesh-created edges at it per face pair. An odd count means a trace was lost there. Error with the vertex name, the two faces, and the traced edges. This is a pure topology check, costs nothing, and names the place.

**12. Coincident faces whose interior point lands on the other's boundary curve**
- **SMLib:** sector classification at shared edges (SmEdge.cpp:1789–1940, 10 sample params) and `FixFoundFaceCoincidence` (MG:2129). A tangent face diverging from a sector boundary is classified by how it diverges (SmEdge.cpp:1781–1786).
- **geop:**
  - `classify_face` sets aside on-boundary points and decides coincidence by normals (BO:686–737).
  - It errors when normals are near perpendicular (BO:727). That happens where a face lies on the other body's boundary *only along a curve* at every point tried, i.e. a sliver.
  - It errors when no normal exists at any point (pole-only patches).
- **How:** for a face whose every offered interior point is on the other's boundary, use `face_interior_point_where` with a finer margin first (already done in 4132088). If still undecided, classify by the radial-order test of case 4 at an edge of the face, which needs no interior point. This is SMLib's approach and fits geop exactly, since the edge is shared after remesh.

### P3 — performance and rarer configurations

**13. Spatial culling and settled sets (work, not seconds)**
- **SMLib:** surface trees precomputed in parallel (TI:1128); bounding-box cull per face pair expanded by tolerance (TI:4898); registered-SSI table (TI:184–265); flood-fill classification, one point per connected face set (MG:3330, 3563, 3722, 3925).
- **geop:**
  - `find_coincident_vertex_pair` is O(Va·Vb) and restarts after every merge (`remesh_vertices.rs:17`).
  - `find_coincident_pair` runs `curve_surface_intersect` over all E×F and **restarts from scratch after every imprint**, with no `settled` set unlike `find_piercing_crossing` (XF:554 vs XF:304).
  - `find_tracing_start_points` is O(E·F) `surface_could_contain` (XF:717).
  - Classification is one point set per face.
- **How:**
  - Sort-and-sweep or BVH over interval AABBs for vertex×vertex, edge×face and face×face candidate pairs. This is a free-choice structure with no effect on answers.
  - Give `find_coincident_pair` the same `settled` invariant: an imprint splits a face, but a pair found "not coincident-inside" stays so for the face piece holding the old id. The new face's pairs need asking.
  - Classification by flood fill: faces joined by an edge that is *not* a remesh-created edge, and not a coincident imprint, are on the same side. Classify one face per component and copy the result, keeping one confirming point per component for the straddle diagnosis.
  - Guard with a count test (`mates_resolved`-style): number of `curve_surface_intersect` calls for a gear of N teeth.

**14. Multi-branch start points and curves with both ends at apexes**
- **SMLib:** cannot do this either (ADV:868).
- **geop:** an apex start is skipped and relies on the other end (XF:1316–1328). A generator shared by two cones with a common apex, or a curve from apex to apex (two coaxial cones touching), has no traceable end.
- **How:** case 8's node machinery: an apex is a node whose branch directions are found from the cone's generators lying on the other surface. Use `curve_surface_intersect` of the generator lines through the apex, i.e. the net's spoke curves, against the other surface.

**15. Isolated touch points as shared vertices (only if non-manifold is supported)**
- **SMLib:** a ProtoVertex for touching points, so shells touching at a point get a common vertex (TI:1972–2021).
- **geop:** none needed for manifold solids; classification copes (`touching_spheres_union` BO:1914, `cube_union_sphere_touching_a_face` BO:1736). Port only together with case 7's non-manifold decision.

**16. Points pre-registered or suppressed per surface pair**
- **SMLib:** `RegisterExistingSSI` (TI:265); a NULL entry suppresses all of a surface's intersections. Used by offset and by `PiecewiseMerge`.
- **geop:** nothing. Useful for pattern / up-to-next to skip the pairs known disjoint by construction. Low priority: the AABB culling of case 13 gives most of the benefit honestly.

### Things SMLib does that geop should NOT port

- Capture distance and averaging of vertex positions (PT:84 `sm_FindPositionFromPCs`, PT:2478, TI:1154) replace interval coincidence with a tolerance. geop's `could_be_equal` plus refine is better.
- Gap repair by squeezing vertices (`FixGaps` TI:632, `CheckForSqueezeVertices` TI:331, `GlueCoincidentBrepVertices` PT:5854). geop's named error is better. Port only the detection (case 11).
- Skipping near-tangent interior seeds (ADV:1175–1186) and the 2° boundary-tangent heuristic (SI:8198). In geop these must be decided or refused.
- Rays only along ±X/±Y/±Z (BR:24687). geop's random rays with retries are better.
- `IsTopological` "tangent within tolerance" face merging. geop should merge only provably identical surfaces (case 3).
- "Confused keep/del list ... continuing" (MG:3973–4056) and silently skipping unclassifiable faces (MG:3789, 3917). geop errors.

---

## 3. Whole features geop does not offer

1. **Surface/surface intersection as an API and as an op** (curves between two faces or sheets): SMLib `SmSurface::GlobalSurfaceIntersect`, `SmBrep::BrepIntersect` (BR:890). In geop it belongs in `core/geop-core-geometry/src/intersection/surface_surface.rs`, with the interior-loop and node search of cases 1 and 8. `remesh` would then call it per face pair instead of discovering curves only from edges.
2. ~~**Section curves op**~~ done: `Section` (body × body, body × datum plane) builds a wire; a sweep takes a run of it as a path.
3. ~~**Imprint op (body onto body)**~~ done: `Imprint`, both bodies kept and divided, or only one face of the first.
4. **Imprint and classify:** `SM_BO_IMPRINT_CLASSIFY` (MG:4060), for previewing what a boolean would remove. It is cheap to expose from `combine`'s `decisions` map.
5. **XOR:** `SM_BO_EXCLUSIVE_OR`. The keep table needs only `Outside` from both, plus `Inside` from both reversed; `OnSame` from both is dropped and `OnOpposite` kept from both.
6. **Merge / partition (cellular, non-manifold):** `SM_BO_MERGE`, `SM_BO_PARTIAL_MERGE`, `PiecewiseMerge` (MG:341). These need non-manifold topology (case 7).
7. **Extract separate / multi-lump results:** `SM_BO_EXTRACT_SEPARATE`, `CutBreps` (BR:38618), `CreateOneBrepPerBody`. In geop this comes for free with case 6.
8. **N-ary boolean / CSG tree:** `merge_breps` (MG:1343), `BooleanTrees` (MG:1820), `BooleanLists` (MG:1431). geop's `Boolean` op takes two solids; `combine_tools` folds tools in turn. A multi-body union in one step, with one remesh of all against all and one classification, would also avoid compounding width.
9. **Boolean of solid with sheet (cookie cutter / keep the sheet's part inside):** `m_bCookieCutter` (MG:4226), `SM_BO_SLICE`. geop has `split` and `trim_up_to_next`, but no "solid minus sheet-bounded half-space" or "sheet ∩ solid → trimmed sheet" op. The latter is a one-liner on top of `trim.rs`'s classification.
10. **Wire and point bodies in booleans:** `ProcessWiresAndShellVertices` (MG:7239), `sm_IntersectShellVertices` (TI:1354). geop's `remesh` takes solids and sheets only.
11. **Sheet × sheet intersection keeping the wire:** MG:4689–4712.
12. **Boolean with the intersection curves returned:** `SmApiBooleanWithCurves`. geop could return the remesh-created edges from `Origins`/naming.
13. **Planar 2D region boolean on sheets:** `Boolean2D` (SmPrimitiveCreation.cpp:7588). This may be covered by the sketch's regions (not surveyed).
