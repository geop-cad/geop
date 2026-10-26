# Offsets, shells, thicken, hollow: SMLib vs geop

Scope: SMLib `SmOffsetExecutive`, `SmOffsetGeometryCreation`, `SmOffsetSurface`,
`SmOffsetCurve`, `NL_SrfOffset`, `NL_CrvOffset`, the per-surface
`CreateOffsetSurface` overrides, and `SmCompositeCurve::CreateOffsetsOfManyCurves`.
geop: `lib/geop-lib-shell/src/shell.rs` (`shell`, `thicken`, `offset_faces`, `Hollow`),
`core/geop-core-geometry/src/nurb_surface/offset.rs` (`NurbSurface3D::offset`),
`core/geop-core-geometry/src/meet.rs`, `ops/geop-ops-shell`, `ops/geop-ops-surface`
(`offset.rs`, `thicken.rs`), and `core/geop-core-sketch/src/offset.rs`.

All SMLib paths are under `/home/tobiasjacob/usd-brep/source/SMLib/src/`. All geop
paths are relative to `/home/tobiasjacob/geop/`.

## 0. How the two designs differ

**SMLib** (`SmOffsetExecutive::DoSolidOffset`, SmOffsetExecutive.cpp:379, and
`DoExtendedOffset`, :1062) builds a *soup* and then resolves it:

1. Every face gets an offset surface (`FaceuseOffset`, SmOffsetGeometryCreation.cpp:118).
   That surface is exact for analytic types and implicit (`SmOffsetSurface`) for
   B-splines. The face's own UV trim curves are re-used on it. Faces in the
   "shell faces" list are copied, not offset.
2. Gaps at convex edges are filled in one of two ways. With `bDoExtendedOffset=FALSE`
   it makes rolling-ball cap faces at edges (`EdgeuseOffset`, :2205) and sphere caps
   at vertices (`VertexOffset`, :273). With `TRUE` it extends both offset surfaces
   and intersects them (`MateEdges`, :1802; `ExtendTrimEdges`, :1384;
   `TrimExtendedEdge`, :878).
3. `PiecewiseMerge` (SmOffsetExecutive.cpp:2236) intersects every pair of faces,
   except pairs already known to share an edge (the `sNoSSI` relation,
   :2303-2412). Then `MakeManifold` runs, regions are labelled, and
   `RemoveExcessFaces` (:2042) deletes every face that has an edge whose midpoint is
   closer than `0.95*d` to the original body.
4. A shell is `ShellBrep` (:190). It makes the offset body, then a manifold
   **boolean difference**: original minus inset, or offset minus original (:332,
   :342). For a sheet it instead merges the sheet with its offset and stitches them
   (:299-320).

**geop** (`lib/geop-lib-shell/src/shell.rs`) builds the hollow *directly*. The
topology does not change. The inner side is the body's own BodySpec, face for
face:

- Each vertex is moved by Gauss-Newton `meet` onto the offsets of its faces.
- Each edge's inner copy is the edge's shape between the moved vertices
  (`shaped_like`: a line or a coaxial arc only). It is then verified with
  `surface_missed`.
- An opening is a rim: the removed face minus its inner copy (`Hollow::rims`).
- A sheet gets ruled walls along its free edges.

Anything outside these assumptions is refused, and the refusal names the entity.

**Where geop's approach is better (keep it):**

- There is no boolean of coplanar faces. SMLib's open shell copies the removed faces
  unchanged into the offset body, then boolean-subtracts. That puts the removed
  face exactly on the original, which is the degenerate configuration AGENTS.md
  warns about.
- Names persist entity by entity: `shell(S,X)`, `shell(S,E,start)`. SMLib only
  propagates attributes through `Notify`, and its own comment calls that
  "awkward" (:1934).
- The surfaces are exact (planes and revolutions), with no `ReplaceImplicitOffsets`
  approximation step.
- Every built curve is verified to lie on both inner surfaces (`surface_missed`).
  `meet` checks every condition it was asked to meet.
- Poles and apices are handled exactly. The cone's apex goes to the offset apex
  (`offset_revolution`). SMLib uses `sm_FindNonsingularStepoff`, a 0.001/0.01/0.1
  stepoff with linear extrapolation (SmOffsetSurface.cpp:858).
- Tangent faces are grouped once, at the vertex (`tangent_groups`). SMLib uses a
  `2°` tangency tolerance (`m_dTangencyTolRadians`) and an `85°` cutoff
  (`TrimExtendedEdge`).
- Failures are named refusals, not silent `SM_SUCCESS` fallbacks. Examples of those
  fallbacks: `FillGapEdgeuses` returns success when it cannot find 2 candidate
  edges (:3183), and the pipe-sweep fallback is used "when bProblems" (:3086).

The port should keep geop's direct construction as the main path. Where the topology
really does change (vanishing faces or edges, self-intersection), add a resolution
stage. Do not switch to soup-plus-boolean.

---

## 1. Feature table

| Capability | SMLib (file / function) | geop (crate / function) | Verdict |
|---|---|---|---|
| Offset plane | `SmPlane::CreateOffsetSurface` (SmPlane.cpp:261) | `NurbSurface3D::offset` → translate | parity |
| Offset cylinder / cone / sphere / torus | `SmCone::CreateOffsetSurface` (SmCone.cpp:533), `SmSphere` (:259), `SmTorus` (:240) | `offset_revolution` (meridian line/arc) | parity for the normal case |
| Cone offset past its apex (one radius < 0) | SmCone.cpp:592-660: makes a truncated cone at the new apex and trims the domain | refused: "takes the surface of revolution across its axis" | **missing** |
| Sphere / torus / cylinder shrunk to nothing | the surface is skipped (`dNewRadius < tol → return SUCCESS, NULL`, SmSphere.cpp:320, SmTorus.cpp:298), then `DoSolidOffset` `continue`s (SmOffsetExecutive.cpp:470) | refused: "shrinks the meridian's arc ... past its center" | **missing** (vanishing face) |
| Spindle torus (major < offset minor) | `EdgeuseOffset` self-intersecting torus case (SmOffsetGeometryCreation.cpp:2636-2710) | refused (r < 0 check) | missing |
| Revolution with free-form meridian | `SmSurfOfRevolution::CreateOffsetSurface` (SmSurfOfRevolution.cpp:1244): offsets the generator curve and trims it at the axis (:1403) | refused: "meridian is neither a line nor a circular arc" | **missing** |
| Extrusion of any planar profile | `SmSurfOfExtrusion::CreateOffsetSurface` (:1208): offsets the profile with `CreateTrimmedOffsets`, trim=TRUE | refused (not a revolution) | **missing** |
| Free-form B-spline offset | `SmOffsetSurface` (implicit exact evaluator, derivatives up to 3rd, pole stepoff); `ReplaceImplicitOffsets` (SmOffsetExecutive.cpp:1971) → `ApproximateOffsetSurface` / `CreateOffsetWithPolygon` (SmBSplineSurface.cpp:1659, subdivides to tolerance); `N_SrfOffset` (NL_SrfOffset.cpp:678: exact for quadrics, generator offset for rev/extrusion, otherwise sampling + interpolation + knot removal) | refused | **missing** |
| Variable-distance surface offset | `N_SrfOffsetFunc` (NL_SrfOffset.cpp:81): `S + d(u,v)·V/|V|`, not wired to Brep | missing | missing in both at the Brep level |
| Per-face thickness | none (one `m_dOffsetDistance`) | none, but `meet` already takes a per-condition `offset` | cheap win for geop |
| Inward offset / shell | `ShellBrep(dist<0)` → original − inset | `shell` (inward only) | parity |
| Outward shell (walls grow outward) | `ShellBrep(dist>0)` → offset − original | missing (`ShellArgs.thickness` is "measured inward") | missing |
| Offset whole solid (no hollow) | `OffsetBrep` (SmOffsetExecutive.cpp:100) | `offset_faces` makes a sheet copy only, not a solid | partial |
| Convex edge gap: extend and intersect | `DoExtendedOffset` + `MateEdges` (SSI of extended surfaces, picks the branch nearest the old midpoint, :1953-1965) | implicit: `meet` + `shaped_like` (line/arc only) | geop lacks general edges |
| Convex edge gap: rolling-ball round | `EdgeuseOffset` caps: torus (arc rails, :2636), cylinder/cone (:2713), circular cross-section sweep (`SmCircularCrossSectionFSG`, :2751); vertex sphere from 3 arcs (`CreateSphereFromArcs`, :614) | missing | missing (option) |
| Edge offset where the rails coincide (tangent edge) | no cap when rails < 10·tol (:2496) | `tangent_groups` at vertices; tangent edges are plain line/arc | parity |
| Tangent-sector handling | prepass `IsTangentSector` (SmOffsetExecutive.cpp:1443-1510); `TrimExtendedEdge` extends UV across a tangent sector (:1081-1214); `sNoSSI` across tangent edges (:2372-2396) | `tangent_groups` (exact grouping) | geop better for vertices |
| General (non-line/arc) edges | handled through SSI of the extended offset surfaces | **refused**: `shaped_like` "the edge is neither straight nor circular" | **missing, top priority** |
| Inner surface extension past the face | `GetCreateExtendedSurface` (:2111): any surface G1, `5·d`; the face picks the extended surface only if its trims need it (:1613-1703) | `extended`: only degree-1 directions, not past a pole | partial |
| Self-intersection of a single offset surface | `FaceuseOffset` `GlobalSurfaceIntersect(self)` (SmOffsetGeometryCreation.cpp:217-259); `MergeSelfIntersectionCurves` into the face (SmOffsetExecutive.cpp:3661) | not checked | missing |
| Global self-intersection (inner side runs into itself) | `PiecewiseMerge` + `MakeManifold` + region labelling + `RemoveExcessFaces` | **not noticed** ("that the inner side runs into itself otherwise is not noticed", shell.rs:48) | **missing (silent wrong result)** |
| Vanishing edge (short edge eaten) | merge + `sm_RemoveSmallStuff` (<10% of length, :2571) + `sm_ExtractGaps` (:2623) + `FillGapEdgeuses` (3-sided corner blend, :3106) + pipe-sweep fallback (:3086) | refused for straight edges only ("inner copy ... turns around", shell.rs:427-444); arcs unchecked | missing |
| Vanishing face (narrow face, small fillet) | NULL offset → skipped; merge closes it; `RemoveExcessFaces` | refused | **missing, high priority** |
| Offset domain smaller than face (singular parts cut away) | `BuildOffsetOfFaceuse` 2-D boolean of the face's UV region with the offset domain (SmOffsetExecutive.cpp:3765-3928) | n/a (only exact same-param offsets) | missing (needed with truncated cones) |
| Open faces (shell openings) | "shell faces" copied, then boolean difference; at an inset that hits a shell face, `MateLaminaEdgeToSurface` extends the side face to it (SmOffsetGeometryCreation.cpp:2318-2365) | `Hollow::rims`, direct | geop better in principle |
| Opening at an inward corner (pocket floor, step, boss base) | works (boolean) | **refused**: `inward_corner` (shell.rs:222) | **missing, high priority** |
| Opening a face tangent to a kept face | works via the tangent-sector logic and the boolean | **refused**: "a face taken away is tangent to one kept" (shell.rs:406) | missing |
| Opening part of a revolved face at its pole | works | refused (shell.rs:389, :845) | missing (low) |
| Vertex of ≥4 faces whose offsets are not concurrent | merge creates the new edges | `meet` fails ("not X from surface k") | missing |
| Solid that already has a void | per-faceuse offset handles any number of shells | refused (shell.rs:110) | missing (low) |
| Sheet thickening with side walls | `bCreateOffsetSolid` → `BuildSideWallOfEdgeuse` ruled (:2789); `StitchIntoShell` | `thicken`, ruled walls | parity; geop's is exact topology |
| Free-edge (lamina) offset of a curved sheet | `ExtendTrimEdges` handles lamina edges by extension and trim against neighbours | `shaped_like` → line/arc only | missing for spline borders |
| Thicken on both sides / symmetric | (caller's job) | `ThickenSide::Both` (offset half, then thicken) | geop has it |
| Non-manifold / lamina faces removed when too close | `BuildOffsetOfFaceuse` deletes a face whose lamina edge is closer than `d − 2·tol` to the original (:3694-3761) | n/a | missing (part of self-intersection) |
| Local operation (replace a face's surface, re-extend neighbours) | `SmBrep::LocalOperation` (SmBrep.cpp:~11960-12040) with `SM_OO_LOCAL_OPERATION` and `SmLocalOperationGeometryCreation` | missing (no move / offset / replace face) | **missing feature** |
| Planar curve offset, lines/arcs | `SmCompositeCurve::CreateOffsetsOfManyCurves` (SmCompositeCurve.cpp:2586) | `Sketch::offset` (core/geop-core-sketch/src/offset.rs:177), constraint-linked | geop better (associative) |
| Spline curve offset | `SmBSplineCurve::CreateSimpleOffset` (:2475); `N_CrvOffset` / `N_CrvOffsetApprox` / `N_CrvOffsetPtSampling` (NL_CrvOffset.cpp:897-1217) | refused ("the offset of a spline is no spline") | missing |
| Variable-distance curve offset | `N_CrvOffsetFuncConstantDir` / `VariableDir` (NL_CrvOffset.cpp:88, :493) | missing | missing |
| Corner types | `SM_OC_LINEAR_EXTENSION`, `SM_OC_FILLET_CORNER`, `SM_OC_LINEAR_CHAMFER` (SmCurveTypes.h:62) | `Corners::Round`, `Corners::Extend` | chamfer missing |
| Fallback when extension does not meet | connect by the tangent-line intersection, else a line (SmCompositeCurve.cpp:~2885-2940, [B632]) | refused: "do not meet: round their corner instead" | geop stricter |
| Global trimming of a self-intersecting offset | `IntersectAndTrimOffsets` (:3218) + "forbidden zone" `sm_CurveClassify` (:2051) + `sm_CheckBowTies` (:2352) | refused: "would cross itself: X runs into Y" | **missing** |
| Many loops offset together (outer + holes interacting) | `CreateOffsetsOfManyCurves` with per-loop direction (1/2/3 = left/right/both) | single chain per call | missing |
| Implicit exact offset curve object | `SmOffsetCurve` (SmOffsetCurve.h:29) | n/a | not needed |

---

## 2. Special cases to port (work order, most frequently hit first)

### 2.1 Edges that are neither straight nor circular (inner copy by SSI)

- **SMLib:** `SmOffsetGeometryCreation::MateEdges` (SmOffsetGeometryCreation.cpp:1802-2167) does four things:
  - It extends both faces' offset surfaces (`GetCreateExtendedSurface`, SmOffsetExecutive.cpp:2111).
  - It intersects them with `GlobalSurfaceIntersect` (:1862).
  - It trims each intersection branch with `TrimExtendedEdge` (:878). That function takes the trim points from the neighbouring edges' extended surfaces, or from UV extension at tangent sectors. Its fallback is to drop the vertex onto the curve (:1310).
  - If several branches survive, it keeps the one whose midpoint is nearest the original edge's midpoint (:1953-1965).

  For lamina (free) edges, `ExtendTrimEdges` (:1384) prefers the more iso-parametric UV curve of the two faces. It extends that curve 10× (:1186), lifts it through the extended surface, drops it onto the other surface (several pieces → keeps the shortest drop, :1598-1618), and trims.
- **geop today:** `shaped_like` (shell.rs:1033) returns `"the edge is neither straight nor circular: offsetting it is not supported"`. Every edge produced by a boolean SSI trace hits this. Examples:
  - a cylinder through a cylinder (`pipe_tee` in the stress sweep);
  - an angled hole (an ellipse);
  - a spline-profile extrusion's edges;
  - fillet edges the boolean traced. The ignored test `ops/geop-ops-shell/src/tutorial_regression_tests.rs:59`, `#[ignore = "bug: shelling a box whose vertical edges were rounded is refused — ... traced curves, which the shell cannot offset"]`, is this case.
- **How in geop:** keep `meet` for the vertices. For an edge whose shape is not line/arc:
  1. The two inner surfaces are known exactly (the faces' offsets), and so are the two moved vertices (`inner_points`). The inner edge is the branch of `inner_surfaces[f] ∩ inner_surfaces[g]` that runs from `inner_point(start)` to `inner_point(end)`.
  2. Trace it with the booleans' face×face tracer (`trace_one_side` / the predictor-corrector in `geop-core-geometry`). Seed it at the start vertex. Give it a direction from the original edge's tangent, carried to the offset (the tangent of the offset curve is parallel to `n_f × n_g` at the moved point). Stop at the end vertex, which is a known vertex on the curve ("A step must never decide a direction past a vertex known to lie on the curve").
  3. Choose the branch topologically: the branch that ends at the other moved vertex, *not* the nearest-midpoint heuristic SMLib uses. Use `surface_could_contain` on both surfaces for the end vertex ("Proximity is not membership").
  4. A closed edge (one vertex) gets the same treatment: trace until back at the start vertex.
  5. Fit pcurves with `fit_pcurve` (piecewise by the source curve's knots). Validate with `surface_missed` as now.
  6. For a free edge (sheet border) there is no second face. The inner copy is the image of the edge under `p ↦ p + d·n_f(p)` on that one surface, since the offset is exactly parametrised (`offset` keeps `(u,v)`). So the inner pcurve is the *same* pcurve on the inner surface. The 3-D curve is the inner surface evaluated along it, fitted per span with the width checked. This needs no SSI, and it also gives the ruled wall a proper rail.
  7. Put this in `shaped_like`'s caller (`Hollow::inner_curve`). Make `shaped_like` a fast path, not the only path.

### 2.2 Offset of surfaces that are not plane or line/arc revolution

- **SMLib, in priority order:**
  1. `SmSurfOfExtrusion::CreateOffsetSurface` (SmSurfOfExtrusion.cpp:1208) offsets the planar generator with `CreateTrimmedOffsets(..., bTrimResults=TRUE, ...)` (:1245-1257) and re-extrudes it.
  2. `SmSurfOfRevolution::CreateOffsetSurface` (SmSurfOfRevolution.cpp:1244) offsets the generator in its plane. It uses `CreateSimpleOffset` to keep lines and circles. It trims the offset generator at the axis, keeping the larger part (:1403-1430).
  3. General B-splines: an implicit `SmOffsetSurface`, replaced at the end by `ApproximateOffsetSurface`, which subdivides spans until within tolerance (`CreateOffsetWithPolygon`, SmBSplineSurface.cpp:1659-1931). `N_SrfOffset` (NL_SrfOffset.cpp:678) samples, interpolates and removes knots.
- **geop today:** `NurbSurface3D::offset` (core/geop-core-geometry/src/nurb_surface/offset.rs:58) refuses: `"only planes and surfaces of revolution with a straight or circular meridian have an exact offset"`. That refuses fillet/rolling-ball blends from `geop-ops-fillet`, lofts, network/boundary surfaces, STEP B-spline faces, and spline extrusions.
- **How in geop (interval style, no epsilon):**
  - **Extrusion (exact where its profile is):** recognise it like `extended` does: degree 1 in one direction, two rows that are translates with equal weights. Offset the profile in the plane square to the direction. That gives a planar curve offset (2.12 below), which is exact for line/arc profiles. Re-extrude with the same knots so `(u,v)` keep meaning.
  - **Revolution with a free-form meridian:** the same, with the meridian's planar offset (`offset_revolution` already has the `(h, r)` frame).
  - **Free-form (approximate, but honest):** fit a NURBS `Q` to `S + d·N` with the same `(u,v)` parametrisation. Insert knots of `S`, plus extra ones where the width demands it. Then compute an **enclosure of the error** over each span by branch and bound: interval-evaluate `Q(u,v) − (S(u,v) + d·N(u,v))` over subdivided boxes, like `nearest::least_distance`. Widen `Q`'s control points by that bound, so the patch encloses the true offset. Refine the knots while the bound exceeds the entity tolerance an edge may carry (AGENTS.md cites `1e-4`; aim for a fraction of it). The number of spans then follows the width it produces ("let the sample count follow the width it produces"), and no tolerance decides correctness.
  - **Refuse by name** where `|d|·κ_max` could reach 1 (the offset passes a focal point and folds over). Bound the principal curvatures over each span with intervals: `1 + d·κ_i` must be definitely `> 0`. Otherwise the offset self-intersects. Report it, and in a shell treat that face as "vanishing locally" (2.5).

### 2.3 Vanishing faces: fillet / round smaller than the wall

The case: a solid edge rounded with radius `r < t`, then shelled. This is the first thing every enclosure tutorial does with a thick wall.

- **SMLib:**
  - `SmSphere::CreateOffsetSurface` returns no surface if `dNewRadius < tol` (SmSphere.cpp:320). `SmTorus` does the same for the minor radius (SmTorus.cpp:298), and `SmCone` does when both radii are negative (SmCone.cpp:592).
  - `DoSolidOffset` `continue`s past a NULL offset (SmOffsetExecutive.cpp:470).
  - The neighbours' offsets then meet each other in `PiecewiseMerge`, and `RemoveExcessFaces` drops the leftover pieces.
- **geop today:** `offset_revolution` errors `"shrinks the meridian's arc of radius ... past its center"` / `"takes the surface of revolution across its axis"` (offset.rs). The shell stress `is_refusal` list even contains `"shrinks the meridian"`.
- **How in geop:** this is a topology change confined to the face.
  1. In `Hollow::new`, classify each kept face by its offset. A face whose offset radius is definitely `< 0`, or could be `0`, **collapses**: an inward offset of a convex cylinder or torus blend, a sphere corner patch, or a cone tip.
  2. A collapsed face has no inner copy.
  3. If its neighbours `g, h` are tangent to it along its two long edges (a fillet), the inner copies of those edges merge into one inner edge, `inner(g) ∩ inner(h)`. For a constant-radius blend between planes, that is a line parallel to the blend's axis. The inner vertices at the ends are `meet` of the remaining faces (drop the collapsed face from the `Offset` list at those vertices).
  4. A vertex-blend (sphere corner) collapses to a vertex, `meet` of the three walls.
  5. Decide collapse with `Scalar` comparisons on `scaled` (arc) and on `r_j + dr` (line). If `could_be_equal(0)`, the face collapses *exactly* to its axis or center. The result is the same, so no epsilon is needed.
  6. Names: the collapsed inner face has none. The merged inner edge is named after the face (`shell(S,F)`), so it stays stable.
  7. The edges of the collapsed face between neighbours disappear on the inner side. The rest of `result` iterates `spec.edges`, so `inner_curves[e] = None` must be allowed for non-open edges, and the inner loops of the neighbours must be rewired to the merged edge.
  8. Add the reproducer first: a block with one rounded upright edge (`ops/geop-ops-shell/src/regression_tests.rs:58` builds it), `r = 0.5`, shell `t = 1`.

### 2.4 Opening at an inward corner (pocket floor, step riser, face a boss stands on)

- **SMLib:** a shell is offset-body minus original, so this works by construction. At the edge between an open face and an offset face, `EdgeuseOffset` (SmOffsetGeometryCreation.cpp:2318-2365, "we are insetting and hit the shell face, extend and intersect the side face") calls `MateLaminaEdgeToSurface`. That extends the offset side face to the open face's surface.
- **geop today:** `inward_corner` (shell.rs:222) refuses: `"face {f} is taken away where it meets face {g} at an inward corner ... its opening would reach past it, which is not supported"`. The module docs (shell.rs:44-47) explain why.
- **What the result is (worked out for a step):** take a lower top `F` (z=1, x∈[0,1]), a riser `G` (x=1, z∈[1,2]), and the material under both. Open `F`, keep `G`. The wall behind `G` is the slab x∈[1,1+t], and it stands on F's *extended* plane. The result has the rim `F \ F'`, plus a **new downward face on F's surface beyond e** (x∈[1,1+t]): the underside of G's wall. So the opening is not "F less its inner copy". It is "the hollow's section by F's surface". Near `e`, that section reaches past `F`.
- **How in geop:**
  1. Build the rim on `extended(F.surface)` instead of `F`.
  2. At an inward-corner edge `e` (removed `f`, kept `g`), the rim's boundary does not run along `e` then back along `inner(e)`. Instead it runs:
     - along `e` (shared with `g`, which stays),
     - then along the wall end: `g`'s inner surface ∩ F's surface, a line or arc for the supported surface pairs (plane ∩ plane, plane ⟂ axis ∩ cylinder), otherwise traced as in 2.1,
     - then on to the inner copies of the edges that continue.
  3. The vertex where `g`'s inner copy meets F's surface and the next wall's inner copy is a `meet` with F's surface at offset `0` (as removed faces already are: `Offset{offset: ZERO}` on `inner_surfaces[f]` = F's own).
  4. This needs the rim's loop assembly in `rims` to accept these extra coedges. The "pieces" machinery already does the analogous thing for edges between two removed faces.
  5. Keep the refusal for the sub-case where the wall end would cross another face of the solid. That is a global interference, 2.5.

### 2.5 The inner side runs into itself (global self-intersection) — currently silent

- **SMLib:** `PiecewiseMerge` (SmOffsetExecutive.cpp:2236-2478) intersects all faces pairwise except the `sNoSSI` neighbour pairs, then calls `SmMerge::PiecewiseMerge(&sNoSSI, ...)`. `MakeManifold` follows, and every finite region is marked solid (:913-932, :1836-1862). `RemoveExcessFaces` (:2042-2104) deletes any face bounding an edge whose midpoint is `< 0.95·d` from the original (`BrepPointSolve` MINIMIZE). A per-face self-intersection pass also exists (`FaceuseOffset` :217 + `MergeSelfIntersectionCurves` :3661), and so does lamina-face pruning (:3694-3761, threshold `d − 2·tol`).
- **geop today:** shell.rs:48-49 says: "Walls so thick that a straight edge's inner copy turns around are refused; that the inner side runs into itself otherwise is not noticed." Examples:
  - a thin rib or a narrow slot with `t > width/2` on the non-straight parts;
  - two bosses closer than `2t`;
  - a hole closer than `t` to an outer wall;
  - an arc edge whose inner copy turns around (only straight edges are checked, shell.rs:427-444).

  Today these produce an invalid model, or a "valid" one that intersects itself.
- **How in geop:**
  1. **Detect first (cheap, do now).** After building the inner side, run the boolean's face×face piercing search between non-adjacent inner faces, and between inner faces and kept outer faces. The pairs that share an edge or vertex are known topologically, which is exactly SMLib's `sNoSSI` list and is free here. Any crossing is a refusal naming the two faces: "the walls of face A and face B run into each other: walls too thick for this part".
  2. **Resolve later.** Imprint all crossings (the boolean imprint already exists). Classify each resulting region of the inner side by its **least distance to the original boundary**, using `nearest::least_distance`: an enclosure, not a sample. Keep a region if the distance definitely equals `t` (encloses `t`). Drop it if it is definitely `< t`. This replaces SMLib's `0.95·d` magic factor with an interval test. Apply the same to the outer side when shelling outward.
  3. The resolved inner side may be several shells (the void splits). The `BodySpec` output must allow any number of inner shells.

### 2.6 Opening a face tangent to a kept face

The case: a box with filleted top edges, top removed; or a cup whose rim fillet is tangent to the open top.

- **SMLib:** tangent sectors are pre-processed (`DoExtendedOffset`, SmOffsetExecutive.cpp:1443-1510). In `TrimExtendedEdge` (SmOffsetGeometryCreation.cpp:955-1214), the UV trim curve is extended 10× across a tangent or lamina neighbour and intersected. Neighbours count as tangent within 1° (`AreEdgesTangent(...,1.0)`).
- **geop today:** shell.rs:406: `"at vertex {}, a face taken away is tangent to one kept: the walls cannot end on a smooth surface"`. The module doc lists "take away part of a smooth surface".
- **How in geop:**
  - The wall ends on the removed face's surface `F` (at offset 0), exactly as for any removed face.
  - The difficulty is that the kept face `g` is tangent to `F` along `e`. The inner copy `g'` is tangent to `F'`, not to `F`, and may not reach `F` at all. A fillet offset to `r − t` sits a depth `r` below the plane and never touches it.
  - So the wall end is `extended(g') ∩ F`. That needs `g'` extended G1 past its boundary (2.10). The extension's intersection with F is a transversal crossing whenever `t > 0`.
  - At the vertices, `meet` with F (offset 0) and `extended(g')` gives the inner vertex. `tangent_groups` then stops grouping `F` with `g`: one is at offset 0, the other at offset `t`. The current test "removed and kept in one tangent group → refuse" becomes "split the group".
  - Reproducer: a block with all four top edges filleted, shelled open at the top.

### 2.7 Edges that vanish (inner copy has no length or turns around)

- **SMLib:**
  - `SyncronizeOffsetEdgeuses` (SmOffsetExecutive.cpp:2685) pairs the two sides' offset edge pieces.
  - `sm_RemoveSmallStuff` (:2571) drops pieces shorter than 10% (an epsilon; do not port it).
  - `sm_InsertBreaks` (:2513) splits each side at the other's breaks.
  - `sm_ExtractGaps` (:2623) finds the unmatched pieces.
  - `FillGapEdgeuses` (:3106) caps each gap with a 3-sided `CreateCornerBlend`. If that fails, it sweeps a pipe along the edge (:3086-3095).
- **geop today:** for a straight edge, shell.rs:439 refuses with `"the walls are too thick: the inner copy of edge {} has no length or turns around"`. For arcs and other edges there is no check (part of 2.5).
- **How in geop:** this is the edge analogue of 2.3, the straight-skeleton event.
  1. When an inner edge's direction definitely reverses or could be zero, the edge collapses.
  2. Its two end vertices become one inner vertex, `meet` of the union of both vertices' faces. If those are 4 faces and not concurrent, that is 2.8.
  3. The two faces that met along it no longer share an inner edge on that span.
  4. The test is topological once the sign is known, so use `definitely_greater` / `definitely_less` on `(end−start)·along`. "Could be zero" means it collapses exactly, which is handled the same way.
  5. Apply the same check to arcs: the signed sweep of the inner arc against the original's.

### 2.8 Vertex of ≥ 4 faces whose offsets are not concurrent

- **SMLib:** `BuildOffsetOfVertex` (SmOffsetExecutive.cpp:3227) makes a cap only when every edge has a cap (rounded mode). In extended mode the merge produces the extra edges.
- **geop today:** `meet` (core/geop-core-geometry/src/meet.rs:169) Gauss-Newton gives a least-squares point. The final check errors with `"the point ..., found from ..., is not ... from surface k of n"`.

  Examples: a pyramid with a non-symmetric quadrilateral base, a hip-roof apex, a drafted pocket corner where 4 drafted faces meet, a vertex created by 2.7.
- **How in geop:**
  1. When `offsets.len() ≥ 4` after `tangent_groups`, check concurrency first. Solve exactly on every 3-subset (for planes: the 3×3 linear system), and test pairwise with `could_be_equal`.
  2. If they are not concurrent, the inner vertex splits into a chain of vertices joined by new inner edges: the straight skeleton of the cone of faces around the vertex. For planes, the new edges are lines `inner(f_i) ∩ inner(f_j)` for the face pairs that become adjacent. Which pairs is decided by the order of the faces around the vertex and by which 3-plane points are "inside" the other half-spaces (sign tests, exact for planes).
  3. Start with: refuse with a named message ("the offsets of the N faces at vertex V do not meet in one point: a new edge would be needed — not supported"). The current error comes from `meet` and reads as an internal failure.

### 2.9 Round (rolling-ball) gap filling at convex edges: an option

- **SMLib:** `bDoExtendedOffset=FALSE` (the default in `OffsetBrep` / `ShellBrep`).
  - `EdgeuseOffset` (SmOffsetGeometryCreation.cpp:2205) builds the cap between rails = the two offset edges, around the centre line = the original edge:
    - two arcs plus a circular centre line → a torus (`CreateSurfOfRevolution`, :2659-2710, with the spindle case);
    - a line plus an arc → a cylinder or cone (`CreateLinearSweep`, :2713-2745);
    - otherwise a circular cross-section sweep (`SmCircularCrossSectionFSG`, :2751).
  - The rails are trimmed so their end gap to the centre line equals `d` (:2526-2602).
  - Concave sectors get no cap (:2304).
  - `VertexOffset` (:273) builds a sphere from 3 arcs (`CreateSphereFromArcs`, :614). Its special cases:
    - 1 bounding edge with a seam → none (:310-322);
    - 2 coincident edges → none (:324-355);
    - a shell vertex with 2 lines and an arc → a non-tangential ruled blend (`bShelling`, :446-516);
    - arcs that do not lie on the planes → rebuilt from intersection points (:517-610).
- **geop today:** only "sharp" (an extended intersection by `meet`). `offset_faces` with a positive distance on a convex body, and `ThickenSide::Along` on a convex sheet, give sharp corners. That is a valid offset, but not the true (Minkowski) offset.
- **How in geop:**
  - Add `Corners { Sharp, Round }` to `offset_faces`, `thicken` and an outward shell (2.13). For `Round`, a convex edge `e` (`n_f × n_g · along > 0` in the offset direction, the same test as `inward_corner`) gets a cap face:
    - a straight edge → a cylinder of radius `|d|` about it;
    - a circular edge → a torus about the arc's axis. That is exact as a NURBS revolution, but it is the spindle case if the arc radius < `|d|`; refuse that.
    - other edges → a pipe surface, which is not exact. Use 2.2's fitting-with-enclosure, or refuse.
  - A convex vertex whose edges all got caps gets a sphere patch bounded by the caps' end arcs.
  - The caps are tangent to both offset faces by construction, so the vertices on them are exact, not `meet`-solved.
  - This reuses the fillet crate's rolling-ball corner code only for the vertex patch. Move what is shared into a lib (AGENTS.md: no op→op dependency).

### 2.10 Extension of inner surfaces that are not straight in a direction

- **SMLib:** `GetCreateExtendedSurface` (SmOffsetExecutive.cpp:2111) extends *any* surface by `5·d`, G1 (`SM_CT_G1R`). The face is then switched back to the unextended surface if its trims fit (:1613-1703). `SmOffsetSurface` extends lazily on evaluation outside its domain, up to 10× the domain size (SmOffsetSurface.cpp:499-569).
- **geop today:** `extended` (shell.rs:1181) extends only directions of degree 1 with 2 rows. It does not extend past a pole. Sphere and torus faces, and the arc direction of a cylinder, are never extended. So an inner copy that reaches past a curved face (needed by 2.4 and 2.6, and at inward corners between curved faces) fails with `"its curve ... is not on the offset of face ..."`.
- **How in geop:**
  - For revolutions: extend the *angular* direction by turning further (an exact NURBS arc, up to < a full turn). Extend the meridian direction by extending the meridian (a line extends linearly; an arc extends round its circle up to the poles).
  - For free-form: G1 linear extension of the boundary rows, as `extended` does for degree 1. With intervals, an exact polynomial extension by knot reparametrisation keeps the parametrisation.
  - How much to extend is a free choice. The face's own size, or the thickness times a factor, both work, because extension changes no answer, only whether a solution is in range.

### 2.11 Cone or revolution offset across the axis / past its apex

- **SMLib:**
  - `SmCone::CreateOffsetSurface` (SmCone.cpp:592-660): if only one radius goes negative, it builds a cone truncated at the new apex. The face's UV region is then intersected with that smaller domain by a 2-D boolean (`BuildOffsetOfFaceuse`, SmOffsetExecutive.cpp:3765-3928).
  - `SmSurfOfRevolution` trims the offset generator at the axis (SmSurfOfRevolution.cpp:1403-1430, "take the largest portion").
  - The torus spindle case is at SmOffsetGeometryCreation.cpp:2688-2705.
- **geop today:** offset.rs refuses `"takes the surface of revolution across its axis"` whenever any control point's `r_j` is definitely negative. A typical case is a tapered pin or countersink tip shelled thicker than the tip radius.
- **How in geop:** the offset meridian line crosses the axis at `h* = h_j − d / line.direction[1]` (that formula is already used for the apex).
  1. Cut the meridian there. The offset surface becomes the cone from `h*` (a pole) to the far end.
  2. The face's inner copy then loses its far loop: the small cap's inner copy vanishes, which is case 2.3 for the cap.
  3. The inner vertex goes to the pole, as the apex already does (`pole_at` path in `Hollow::new`).
  4. The face's inner trim is the image of its `(u,v)` loop under the reparametrisation `v ↦ (v − v*)/(1 − v*)`. That is an affine map of the meridian parameter, exact in intervals. No 2-D boolean is needed, because the cut is iso-parametric.

### 2.12 Sketch (2-D curve) offsets

- **SMLib:** `SmCompositeCurve::CreateOffsetsOfManyCurves` (SmCompositeCurve.cpp:2586-3210):
  - It offsets every segment of several loops at once, each loop to the left, the right or both.
  - It skips degenerate offsets (:2725-2780).
  - It fills each convex corner per `SmOffsetCornerType`: linear extension, fillet arc or chamfer (:2840-3150). When all of those are degenerate, it falls back to the tangent-line intersection or a straight line ([B632], ~:2885).
  - With `bTrimResults`, it intersects every offset piece with every other, including itself (`IntersectAndTrimOffsets`, :3218). Overlap ranges at ends are trimmed off ([B127]), and parameters are clamped into the domain ([B386]).
  - It classifies each piece as kept or dropped by whether it lies in the "forbidden zone" closer than `d` to *any original curve* (`sm_CurveClassify`, :2051, with `dCutoff = d(1 − SM_EFF_ZERO)`).
  - It handles "incomplete bow-ties" at the ends of open chains (`sm_CheckBowTies`, :2352; `sm_FixBowTie`).
  - Splines are offset by `CreateSimpleOffset` (SmBSplineCurve.cpp:2475) or `N_CrvOffset` / `N_CrvOffsetApprox` (NL_CrvOffset.cpp:897, :1059), and variable-distance by `N_CrvOffsetFunc*` (:88, :493).
- **geop today:** `Sketch::offset` (core/geop-core-sketch/src/offset.rs:177) handles one chain of lines, arcs and circles. It is **constraint-linked**: the offsets follow edits, which SMLib cannot do. It refuses splines, "fold back", "do not meet: round their corner instead", "too short for an offset", and "would cross itself: X runs into Y".
- **How in geop:**
  - (a) **Global trim instead of refusal.** When pieces of the offset path cross (`crossings`), split them there and keep a piece iff its least distance to the chain is not definitely less than `|d|`. For lines and arcs this is closed-form (point–segment, point–arc), so do it exactly. The kept pieces become separate chains, constrained like the rest: each kept piece stays `Parallel` / `Concentric` to its source, and the new corners are `Coincident` intersection points. This is what a user expects when offsetting inward past a notch.
  - (b) **Several loops at once** (outer and holes): offset each with its own side, then trim across loops by the same distance rule.
  - (c) `Corners::Chamfer`: a line between the two offset ends. The result is closer than `d`, so skip the distance trim for chamfer segments, as SMLib requires `bTrimResults=false`.
  - (d) **Splines:** the offset of a spline is not a spline. Add an *unconstrained* (or "fixed") fitted offset that is honest about its width, as in 2.2, or keep the refusal. Low priority for sketches; higher for 2.2's extrusion profiles, which do not need constraints.
  - Note: the sketch offset decides in `f64` (`if radius <= 0.0`, `dot(ti,tj) < 0.0`) plus `RELATIVE_TOLERANCE`. Moving those decisions to `Scalar` would align it with the epsilon rule. That is a separate cleanup.

### 2.13 Outward shell and solid offset

- **SMLib:** `ShellBrep(dist>0)` = offset − original (SmOffsetExecutive.cpp:342). `OffsetBrep` returns the grown or shrunk solid itself (:100).
- **geop today:** the shell only goes inward (`ShellArgs.thickness` "measured inward"). `offset_faces` returns a sheet copy only.
- **How in geop:**
  - Shell outward: the same `Hollow`, with the roles swapped. The original faces become the inner side (turned around), and the outward offsets the outer side. Convex edges then need `meet` (sharp) or caps (2.9), and the inward-corner refusal applies to the opposite edge class.
  - Offset solid: the inner side alone, closed (`inner_side` with `BodySpec::new(true)`), replacing the solid. Both are small once 2.3, 2.5 and 2.7 exist.

### 2.14 Per-face thickness (multi-thickness shell)

- **SMLib:** no (one distance). `N_SrfOffsetFunc` has a distance function at surface level only.
- **geop:**
  1. Easy and frequently asked for. `Hollow::new` offsets each face by its own `t_f`.
  2. `meet` already accepts a per-condition offset (`Offset { offset }`). Pass the original surfaces with `offset: −t_f`, or keep the per-face inner surfaces at offset 0, as now.
  3. Straight edges stay straight: two offset planes still meet in a line.
  4. A cylinder offset by `t1` and a plane ⟂ to its axis offset by `t2` still meet in a coaxial circle. For other plane/cylinder pairs, `shaped_like`'s circle check fails and the case falls to 2.1.
  5. Add `ShellArgs.faces_thickness: Vec<(face, NumericInput)>`.

### 2.15 Solids with voids, and pole cases in openings

- **SMLib:** a void shell is just more faceuses, offset the other way relative to the material.
- **geop:** refuses `"the solid already has a void"` (shell.rs:110). It also refuses `"faces taken away and faces kept meet at a pole"` (:389) and `"face ... is taken away at vertex ..., where its surface comes to a point"` (:845).
- **How:**
  - Voids: `body_spec` over all shells. Each shell's faces offset into the material. Without openings, the result is outer, outer′, void′ and void shells. Because the topology is per face, this is mostly bookkeeping.
  - Poles: lower priority. They need `split_pole_row`-style handling of the rim at the pole.

---

## 3. Missing whole features

1. **Offset of free-form surfaces** (B-spline, lofts, blends, network/boundary surfaces, STEP faces), approximated with an *enclosed* error (2.2). Without it, shell, thicken and offset surface are refused on every fillet-blended, lofted or imported body.
2. **General edge inner copies via SSI** (2.1). This is the single biggest blocker. It includes the free sheet borders that are splines (thicken of surfaces).
3. **Topology-changing offsets:** vanishing faces (2.3), vanishing edges (2.7), non-concurrent vertices (2.8), cones past the apex (2.11).
4. **Global self-intersection detection and resolution of the offset side** (2.5). Today it is silent.
5. **Rounded (rolling-ball) offset mode** with edge caps and vertex sphere caps (2.9), the true Minkowski offset.
6. **Outward shell, solid offset (grow/shrink a body), and per-face (multi-)thickness** (2.13, 2.14).
7. **Local operation / replace face / move face / offset face** (SMLib `SmBrep::LocalOperation`, SmBrep.cpp ~11960-12040). It replaces chosen faces' surfaces and re-extends and intersects the neighbours (`DoExtendedOffset` with `SM_OO_LOCAL_OPERATION`, `SmLocalOperationGeometryCreation`). geop has no direct-edit face operations. Once 2.1 and 2.10 exist, geop's per-face rebuild (`body_spec` → move vertices by `meet` → rebuild edges, the same skeleton the draft in `ops/geop-ops-plastic/src/draft.rs` uses) gives this almost for free.
8. **Variable-distance offsets**: curves (`N_CrvOffsetFunc*`) and surfaces (`N_SrfOffsetFunc`). Low priority, though useful for variable thickness and for draft-like operations.
9. **Sketch offset**: global trimming, several loops at once, chamfer corners, spline offsets (2.12).
10. **3-D curve offsets** (an offset of an edge curve along a direction field, or on a face): `N_CrvOffsetFuncVariableDir`. geop has none. This would serve lips, split lines and "offset edge on face".

## 4. Suggested order of work

1. Implement 2.1. Reproducers: the ignored `shell_of_a_box_with_rounded_vertical_edges`, `pipe_tee`, an angled hole.
2. Add 2.5 detection (a refusal) at the same time, so nothing silent remains.
3. Implement 2.3 + 2.7 (vanishing faces and edges). They share the "drop a face or edge, re-`meet`" machinery. Reproducer: a rounded block shelled thicker than its radius.
4. Implement 2.4 + 2.10 (inward-corner openings and general extension), then 2.6 (tangent openings).
5. Implement 2.2: extrusion and revolution with planar-offset profiles first, then enclosed free-form fits.
6. Add 2.14, then 2.13.
7. Add 2.9 (round mode) and 2.8.
8. Do 2.12, 2.11 and 2.15, then local operations (§3.7).

## 5. Status after round 2

- **One offset (2.2, 2.11 at an apex).** `NurbSurface3D::offset` is the fitted
  offset alone: tried on the surface's own net first, so planes,
  cylinders, cones, spheres and tori keep their degrees and layout and come
  out exact to the rounding of the bound (about `1e-13`). The bound uses
  coefficient ratios and pads each control point only by the boxes its
  basis function covers. An apex is closed generally (cut back to, or
  carried on through the apex to, where the offset's row closes);
  `offset.rs`'s revolution path is gone.
- **2.4 inward-corner openings:** the solid less its inner side closed into
  a solid, by the existing boolean (`Hollow::subtracted`). Outward walls
  opened at an inward corner stay refused.
- **2.8:** a vertex of four or more surfaces whose offsets do not meet is
  refused naming the faces; poles apart (a pyramid's walls ruled into its
  tip) no longer stop a vertex that the surfaces place.
- **Widths:** `NurbSurface3D::evaluate` over a box intersects the mean value
  form; an edge's copy is now about as wide as the edge.
- **Still open, with ignored reproducers in `lib/geop-lib-shell`:** 2.3/2.7
  for a face whose offset does not collapse but whose copy inverts
  (`a_chamfer_narrower_than_the_walls`), 2.11 past the apex inside a face
  (`a_cone_cut_off_short_of_its_tip_shelled_past_it`). 2.15 voids are
  refused saying why (the void's wall would float free). 2.12 is unchanged.
