# Geometry, construction and queries: SMLib vs geop

Paths: SMLib = `/home/tobiasjacob/usd-brep/source/SMLib` (`src/`, `inc/`), geop = `/home/tobiasjacob/geop`.
All SMLib line numbers are `src/<file>:<line>` of the function definition, unless noted.
Read-only survey. Nothing in either repo was changed.

SMLib scale for context: the files in this area add up to about 380k lines. Most of it is the
NLib function library (`NL_*`, one C function per capability, each with many flags), and on top of
that the `Sm*` B-rep wrappers (`SmPrimitiveCreation`, `SmTopologySweep`, `SmTess`, `SmPoly*`).
geop's equivalent is about 40k lines: `core/geop-core-geometry` (15.7k), `lib/geop-lib-sweep` (7.7k),
`core/geop-core-rasterize` (2.5k), `ops/geop-ops-surface` (4.7k) and others.

---------------------------------------------------------------------------------------------------

## 1. Feature table

Verdicts:
- **better (geop)**: geop is better.
- **par**: roughly equal.
- **worse**: geop has it, but with fewer cases or options.
- **missing**: geop does not have it.

### 1.1 Curve library

| Capability | SMLib | geop | Verdict |
|---|---|---|---|
| Evaluation, derivatives, rational basis | `NL_CrvNurb.cpp` `N_CrvEval`, `N_CrvDerivs`, `N_CrvRatBasisDerivs` | `nurb_curve/evaluate.rs`, `tangent.rs`, `spline.rs` (interval, evaluated relative to the patch via `spline::centered`) | better (geop): honest enclosures |
| Curvature, Frenet frame, torsion, curvature derivative | `N_CrvEvalCurvature`, `N_CrvEvalFrenetFrame`, `N_CrvGetCurvatureDeriv` (`NL_CrvNurb.cpp`) | none for 3-D curves. The sketch has its own (`geop-core-sketch/src/geometry.rs:46`) | missing (small) |
| Knot insertion and refinement | `N_CrvInsertKnot`, `N_CrvRefine` (`NL_CrvTool.cpp`) | `knot_insertion.rs`, `nurb_curve/refine.rs` | par |
| Split, sub-curve, Bezier decomposition | `N_CrvSplit`, `N_CrvDecomposeBez`, `N_CrvExtractCrvSeg` | `nurb_curve/split.rs` (`split`, `sub_curve`, `bezier_pieces`) | par |
| Degree elevation | `N_CrvElevateDegree` | `compatible.rs:46` `elevate_degree` | par |
| **Knot removal (to tolerance)** | `N_CrvRemoveKnots` `NL_CrvTool.cpp:2286`, `N_CrvRemoveKnot`, `N_CrvRemoveKnotMaxErr`, `N_FitRemoveKnots*` | none | **missing** |
| **Degree reduction (to tolerance)** | `N_CrvReduceDegree` `NL_CrvTool.cpp:4186`, `N_BezReduceDegree` | none | **missing** |
| Make curves compatible (exact) | `N_CrvsMakeCompatible` | `compatible.rs:140` `compatible`, `join` | par |
| **Make compatible by approximation (keep control-point counts low)** | `N_CrvsMakeCompatibleApprox` `NL_CrvAdv.cpp:5733`, `...KnotRemove`, `...Constraints` | none: exact merging only, so control-point counts grow with every curve added | **missing** |
| Reverse, transform, reparametrize to an interval | `N_CrvReverse`, `N_CrvTransform`, `N_CrvReparamToInterval` | `reverse.rs`, `translate.rs`, `compatible.rs:24` `with_unit_domain`, `reparametrized` | par |
| Rational reparametrization (end weights to 1), rational-to-polynomial pruning | `N_CrvReparamWeights`, `N_CrvPruneRat` | `compatible.rs:102` `with_unit_end_weights`. No pruning | worse (small) |
| Arc length, point at length fraction | `N_CrvArcLength`, `N_CrvPercentageAlongPt`, `N_CrvEvalEvenSpacedPts` | `length.rs` `length`, `parameter_at_length_fraction` | par |
| Arc-length reparametrization | `N_CrvRemoveAllKnotsArcLen`, `N_SrfReparamArcLength` `NL_SrfGeom.cpp:2045` | none | missing |
| Extend by distance / to a point, with a choice of continuity | `N_CrvExtendByDist` `NL_CrvAdv.cpp:630` (NL_CMAX or NL_G1), `N_CrvExtendToPt` `:1656` | `nurb_curve/extend.rs:42` `extended(at_end, by)`: by a parameter amount, natural polynomial extension only | worse |
| Closest point, all local minima, projecting many points | `N_CrvClosestPt`, `N_CrvClosestPtMultiple` `NL_CrvGeom.cpp:949` | `nurb_curve/closest.rs` and `nearest.rs` (branch and bound, then Newton) | better (geop): proven global minimum |
| Interpolation through points (global) | `N_FitCrvInterp`, `N_FitCrvInterpGivenParams` | `interpolate.rs` `interpolate`, `cubic_spline` (end tangents), `interpolate_homogeneous` | par |
| Interpolation with tangents or derivatives at every point | `N_FitCrvTangents`, `N_FitCrvDerivs`, `N_FitHermite`, `N_FitCrvHighDerivs` | end tangents only (`cubic_spline`) | worse |
| **Approximation to a tolerance (least squares + knot removal)** | `N_FitCrvApprox` `NL_CrvFit.cpp:4482`, `N_FitCrvApproxTol`, `N_FitCrvApproxLstSq`, `N_FitCrvWeightedLstSq` | none: interpolation only, one control point per sample | **missing** |
| Closed / periodic fit | `N_FitCrvApproxClosed` `NL_CrvFit.cpp:14027`, `N_FitPeriodicCubic`, `N_FitCrvWeightedLstSqPeriodic` `:12603` | none: closed traced curves are open interpolants with a C2 break at the seam | missing |
| Approximating a procedurally defined curve to a tolerance | `N_ApproxProcCrvWithCrv` `NL_CrvApprox.cpp:3897` | `interpolate_enclosing` (widened to enclose, but not to a tolerance and not reduced) | worse |
| Approximating with arcs / biarcs (CAM, DXF) | `N_FitBiArc` `NL_CrvFit.cpp:767`, `N_ApproxCrvWithArcs` `NL_CrvApprox.cpp:83`, `N_FitCrvArcs`, `N_CrvSplitIntoLinesAndArcs` `NL_CrvTool.cpp:580` | none. DXF R12 writes splines as sampled polylines (`lib/geop-lib-sheet/src/dxf.rs:113`) | missing |
| Conic arcs: ellipse, parabola, hyperbola, general conic, conic type | `NL_CrvCommon.cpp` `N_CreateConicArc:1269`, `N_CreateEllipticalArc:971`, `N_ConicGetType`, `N_FitConicToPts` `NL_CrvFit.cpp:16966` | only inside the STEP importer (`ops/geop-ops-step/src/import/geometry.rs:653-706`), not a kernel constructor. The sketch has no conics | worse |
| Arcs of degree 3, 4, 5 (for degree compatibility) | `N_CreateCubicSemiCircle`, `N_CreateQuarticArc`, `N_CreateQuinticArc` | none (degree is elevated instead) | par |
| **Planar offset of free-form curves** | `N_CrvOffset` `NL_CrvOffset.cpp:1059` (exact for line and circle, approximated otherwise), `N_CrvOffsetApprox:897`, `N_CrvOffsetPtSampling:1217` | `geop-core-sketch/src/offset.rs`: lines and arcs only, splines refused | **worse** |
| Offset along a variable direction (3-D, on a surface) | `N_CrvOffsetFuncVariableDir` `NL_CrvOffset.cpp:88`, `N_CrvOffsetFuncConstantDir:493` | none | missing |
| Helix | `N_CrvApproxSpiral` `NL_Spiral.cpp:88` (cubic interpolation, radius r0 to r1) | `nurb_curve/helix.rs:161` exact on the cylinder, height within 0.53 % of the pitch | better (geop) for constant radius |
| **Tapered / conical / flat spiral** | `N_CrvApproxSpiral` (rad0 ≠ rad1), `N_CreateSpiralSrfTaper` `NL_CrvNurb.cpp:6829` | none (`helix` takes one radius) | **missing** |
| Curve decomposition at G1 / C0 discontinuities | `N_CrvDecomposeAtG1Continuity` `NL_CrvTool.cpp:1195`, `N_CrvDecomposeContinuity`, `N_CrvGetG1Segs` | `breakpoints()` exists, but nothing splits at creases | worse |
| Cleaning: degenerate spans, duplicate control points | `N_CrvRemoveDegenSegs` `NL_CrvTool.cpp:4518`, `N_CrvRemoveDuplicateCPts`, `N_tooCrvCleanSpans` | none | missing (affects imports) |
| Shape editing (move a point, warp, bend, flatten) | `NL_CrvShape.cpp` `N_CrvShapeModifyCPts`, `N_CrvShapeWarp:3808`, `N_CrvShapeBend`, `N_CrvShapeFlatten` | none | missing (low priority) |
| Symbolic products (dot, cross, derivative curves) | `N_CrvDotCrv`, `N_CrvCrossMultiplyCrv`, `N_CrvMakeDerivCrv`, `NL_FuncsAdv.cpp` | none (stationary points use subdivision instead) | par: not needed |
| Recognising lines, arcs and planar curves | `N_CrvIsLine`, `N_CrvIsPlanar`, `N_IsPtSet1dCircular` | `shape.rs` `as_line`, `as_arc` (three-valued, on the control net) | better (geop) |

### 1.2 Surface library

| Capability | SMLib | geop | Verdict |
|---|---|---|---|
| Evaluation, normals, poles | `N_SrfEvalPt*`, `N_SrfEvalPtNormalAtPole` `NL_SrfNurb.cpp:5418` | `nurb_surface/evaluate.rs`, `normal.rs` (`pole_at`, `normal_at`) | better (geop) |
| Principal / Gaussian curvature and directions | `N_surfJetPrinCurvature`, `N_SrfEvalPtCurvature` | `curvature.rs`: `curvature_radius`, `curvature_radius_along`, `mean_curvature` only | worse |
| Knot insertion, split, sub-patch | `N_SrfInsertKnot(s)`, `N_SrfSplit`, `N_SrfExtractPatch` | `nurb_surface/split.rs`, `knot_insertion.rs` | par |
| Degree elevation | `N_SrfElevateDegree` | `patch.rs:209` | par |
| **Knot removal and degree reduction for surfaces** | `N_SrfRemoveKnots`, `N_SrfRemoveKnotsKeepBoundaries`, `N_SrfReduceDegreeToTol` `NL_SrfTool.cpp:5987` | none | **missing** |
| Making surfaces compatible, joining surfaces | `N_MakeSrfsCompatible(UV)`, `N_SrfJoin` `NL_SrfTool.cpp:6504` | none | missing |
| Iso-curves and boundary curves | `N_SrfExtractIsoCrv`, `N_SrfExtractBoundaryCrvs` | via `boundary_curve` (`contains/surface.rs:79`) and rows | par |
| Removing degenerate patches, singularity tests | `N_SrfRemoveDegenPatch` `NL_SrfTool.cpp:6833`, `N_SrfIsSingular` `NL_SrfBasic.cpp:2858` | pole handling in `normal.rs` and `project.rs` | par |
| Extending a periodic surface | `N_ExtendPeriodicSurface` `NL_SrfBasic.cpp:6113` | none (no periodic concept) | missing |
| Extend by distance / to a curve, with a choice of continuity | `N_SrfExtendByDist` `NL_SrfAdv.cpp:661`, `N_SrfExtendToCrv:2711` (NL_CMAX or NL_G1) | `patch.rs:266` `extended` (by parameter, natural only). The op `extend.rs` works only on a whole untrimmed patch | worse |
| Ruled surface | `N_CreateRuledSrf` `NL_SrfCommon.cpp:846` | `patch.rs:58` `ruled` | par |
| **Ruled surface between a curve and a point (cone to an apex)** | `N_CreateRuledSrfBetweenCrvAndPt` `NL_SrfCommon.cpp:2171` | refused (`boundary.rs`: "two edges sharing an end — the ruled face would come to a point") | **missing** |
| Coons patch | `N_CreateCoonsSrf`, `N_CreateCoonsSrfTwist` | `patch.rs:127` `coons` | par |
| **Coons with cross-boundary derivatives (G1 to curved neighbours)** | `N_CreateCoonsSrfCrossBoundaryDerivs` `NL_SrfAdv.cpp:22012`, `SmTangentField` | `patch.rs` `tangent_to`: exact to flat neighbours; to curved ones leaving them at `TILT` (1e-6 rad) towards their normal, collocated at the Greville abscissae, and *proven* over the whole side (`strays`: mean value form of `into·n` over stretches, the face's feet enclosed by Krawczyk, `enclose_foot`), refined while it strays | better (SMLib's tangency is to an angle, unchecked between) |
| Gordon surface | `N_CreateGordonSrf` | `gordon.rs:172` | par |
| **N-sided G1 hole fill** | `N_FillNSidedHole` `NL_SrfAdv.cpp:18585` (k+1 patches, cross-derivative curves, central tangent plane) | `nurb_surface/fill.rs` `NurbSurface3D::fill(sides, neighbours)`: a quad per side about a flat inner polygon, every edge whole. G1 across the spokes *exactly* (each side's cross derivative `α V + β T` in one polynomial plane field), tangent to the polygon with flat inner corners (degree-5 spokes osculating it), tangent to the faces along the sides (exact to flat ones, `TILT` to curved, the corner twist set from how the face's normal turns, spans halved where it strays) | better (exact G1 inside; SMLib's is to an angle) |
| Skinning (loft surface): degree, closed C1, approximation tolerance, given params/knots | `N_CreateSkinSrf` `NL_SrfAdv.cpp:7899` (flags `cld`, `deg`, `knotTol`), `...Approx`, `...ApproxTol`, `...Params`, `...Interp` | `lib/geop-lib-sweep/src/loft.rs` (`through` is a cubic Bessel spline), `skin.rs` | worse (see §2) |
| **Skin with boundary continuity (G1 to adjacent surfaces at the first and last section)** | `N_CreateSkinSrfBoundaryContinuity` `NL_SrfAdv.cpp:13262`, `N_CreateSkinSrfBoundaryDerivs` | none | **missing** |
| Skin along a spine | `N_CreateSkinSpine` `NL_SrfAdv.cpp:10401` | none (guides at most 3, affine) | missing |
| Sweep surface: frame law, scale law, tolerance | `N_CreateSweepScale` `NL_SrfAdv.cpp:15353` (`curZ` frame curve, `curS` 3-axis scale curve, `tflg` interpolate trajectory, tolerance E), `N_CreateSweepSrf:11450` | `path_sweep.rs`: rotation-minimising frame or fixed normal, linear twist and scale, rails | worse (see §2) |
| Translational sweep, swung surface | `N_CreateTransSweepSrf`, `N_CreateSwungSrf` `NL_SrfAdv.cpp:7744` | `extrude.rs`, `revolve.rs` | par / missing (swung) |
| Revolved surface | `N_CreateRevolvedSrf` `NL_SrfCommon.cpp:1398` | `nurb_surface/revolve.rs:28`, `lib/geop-lib-sweep/src/revolve.rs` | par |
| Quadrics: sphere, torus, cone, ellipsoid, paraboloid, hyperboloid, elliptic cylinder and cone | `NL_SrfCommon.cpp` `N_CreateSphere`, `N_CreateTorus:1697`, `N_CreateEllipsoid`, `N_CreateParaboloid`, `N_CreateHyperboloid`, `N_CreateEllipticalCylCone`, `N_ConeEllipticEnds` | sphere (`sphere.rs`), torus, cylinder and cone via revolve. No ellipsoid, paraboloid or hyperboloid | worse (low priority) |
| **Free-form surface offset** | `N_SrfOffset` `NL_SrfOffset.cpp:678`, `N_SrfOffsetFunc:81`, `N_ApproxNormalSrfWithSrf` `NL_SrfApprox.cpp:2066`, `SmOffsetSurface` (evaluator) | `nurb_surface/offset.rs:58`: planes and revolutions with a line or arc meridian only. Everything else refused | **worse** |
| Approximating analytic surfaces with NURBS | `N_ApproxSphereWithSrf`, `N_ApproxTorusWithSrf`, `N_ApproxConeWithSrf`, `N_ApproxRevolvedSrfWithSrf` | exact rational constructions | better (geop) |
| **Surface fitting to points** (grid interpolation, scattered least squares, with boundary, periodic) | `N_FitSrfToPts` `NL_SrfFit.cpp:341`, `N_FitSrfApproxTol:6545`, `N_FitSrfToPtsAndBoundary:5576`, `N_FitRandomPN` `NL_SrfLstSqFit.cpp:561`, `N_FitSrfLstSqPeriodic` | none | **missing** |
| Projecting points, closest point on a surface | `N_SrfProjectPts` `NL_SrfGeom.cpp:2368`, `N_SrfGetClosestPt:67`, `SmSurfacePointSolve` | `nurb_surface/closest.rs` (branch and bound plus Newton), `project.rs` (Newton, steps off a pole) | better (geop) |
| Shape editing (region warp, bends) | `NL_SrfShape.cpp` `N_SrfShapeRegionWarp:5445`, `N_SrfShapeAxialBend` | none | missing (low priority) |
| Splitting a surface into faces at C0/C1 discontinuities | `SmBrep::CreateFacesFromSurface` `SmBrep.cpp:4762` (`SM_CT_C0`, `C1`, `C2`) | none | missing |
| Recognising surfaces of revolution, planes, cylinders | `SmSurfOfRevolution` helpers (`inc/SmSurfOfRevolution.h:529`) | `shape.rs` `as_plane`, `axis_of_revolution`, `as_cylinder` | better (geop) |

### 1.3 Solids, sweeps and primitives

| Capability | SMLib | geop | Verdict |
|---|---|---|---|
| Extrude a planar profile along its normal | `SmPrimitiveCreation::CreateLinearSweep` `SmPrimitiveCreation.cpp:1150` | `lib/geop-lib-sweep/src/extrude.rs:54`, op `extrude` (blind, symmetric, two sides, up to next, through all) | better (geop) for features |
| **Extrude along an oblique vector; extrude non-planar curves into sheets; extrude existing faces or edges** | `CreateLinearSweep` (faces, `:2029`, `:2106`), `SmTopologySweep::DoSweep` (vertices to edges, edges to faces, faces to solids) | sketch only, along the plane normal only | **missing** |
| **Taper (draft) extrude** with linear or round outer corners, an offset curve trimmed away to a point | `CreateTaperExtrude` `SmPrimitiveCreation.cpp:3604`, `CreateDraftSweep:3090`, `OffsetProfile:4491` | none (`geop-ops-plastic` drafts existing faces) | **missing** |
| Repetitions (sweep n times end to end) | `SmTopologySweep::SetRepetitions` `SmTopologySweep.cpp:60` | none | missing (low priority) |
| Splitting profile curves at C1 discontinuities before sweeping | `CreateLinearSweep(..., bTestContinuity)`, `CreateRotationalSweep(..., bTestContinuity)` | sketch curves are already pieces, but a NURBS spline with a C0 knot sweeps into a creased wall | worse (small) |
| Revolve; a profile on the axis (pole); full and partial turns | `CreateRotationalSweep` `:2565`, `:2799` | `revolve.rs` (poles, edges on the axis, crossing refused) | better (geop) |
| **Revolve: a curve touching the axis in its interior** | `SmRotationalSweepGeometry::CheckCurveSweep` `SmRotationalSweepGeometry.cpp:334` (`SM_SC_ROT_ON_CURVE`, axis intersections not at endpoints) | not detected (see §2 item 1) | **worse** |
| Revolve: self-intersecting swept surface check | same, `SM_SC_SELF_INTERSECT` (lines 440-760) | the axis-crossing check covers a planar profile. Non-planar input never happens | par |
| Translational sweep: a curve tangent to the sweep direction | `SmTranslationalSweepGeometry::CheckCurveSweep` `SmTransSweepGeometry.cpp:628` (`SM_SC_SWEEP_ALONG_LENGTH` :692, :782) | profile ⟂ direction in extrude. The path sweep refuses a path in the profile plane at the start (`path_sweep.rs:321`) | par for planar profiles, missing for sweeping 3-D edges |
| Sweep along a path: rotation-minimising frame | `CreateCurveSweep` `:4882`, `N_CreateSweepScale` (recursive Z0) | `path_sweep.rs` (double reflection, exact lines and arcs) | better (geop): exact on lines and arcs |
| **Sweep frame modes**: binormal law curve `curZ`, translational, fixed direction | `N_CreateSweepScale` `curZ`, `SmSweepOptions::bTranslationalSweep` (`inc/SmPrimitiveCreation.h`) | `Orientation::{FollowPath, FixedNormal}` | worse (no fixed-binormal or law mode) |
| **Scale and twist laws** | `curS` (3-axis scale vector as a function of t), `pScaleReference` / `pScaleCurve` in `CreateCurveSweep` | linear twist and `end_scale` only, or rails | worse |
| Sweep tolerance (approximation error control) | `E` in `N_CreateSweepScale`, `dAppoxTol3d` | `SECTIONS_PER_KNOT_SPAN = 4` (`path_sweep.rs:117`) fixed, no error measured | worse |
| **Path corner at an arc or spline (mitre)** | `CreateSweepAlongPlanarPath` `SmPrimitiveCreation.cpp:5722`: sweeps each segment extended past the corner, cuts both at the bisector plane, using a brep intersection when they are not lines | each wall on the surface it sweeps along its own curve, the body rebuilt where they cross (`path_sweep.rs` `sweep_along`); off a planar path a joint of the profile is two vertices, one per side (`apart_at_corner`). A circle round a corner out of the plane is an ignored reproducer: its walls touch inside a station edge | par |
| Profile wider than the path's bend | none (merge option `m_bDoMerge`) | `clear_of_bends` refuses it by name (`path_sweep.rs:1271`) | better (geop) |
| Pipe sweep (circle of radius r) | `CreatePipeSweep` `:6853` | sweep with a circle profile | par |
| Loft (skin primitive): degree, approximation tolerance, caps | `CreateSkinPrimitive` `:6290`, `:6323`, `CreateSkinFromFaces:6711` | `loft.rs` (matched points, corners, guides, ruled for 2, cubic for 3+) | geop better at matching. Options worse |
| **Loft to a point / from a point** | `N_CreateRuledSrfBetweenCrvAndPt`; skins allow degenerate sections | `Section` needs a `Profile` | **missing** |
| **Closed (periodic) loft** | `N_CreateSkinSrf(cld = NL_YES)` C1 closure when `cur[0] = cur[k]` | none | **missing** |
| Loft end tangency (to a direction, a weight, or adjacent faces) | `N_CreateSkinSrfBoundaryContinuity`, `N_CreateSkinSrfBoundaryDerivs` | none | missing |
| Blend primitive (blend surface between two curves on surfaces) | `CreateBlendPrimitive` `:7342`, `:7533`, `SmBlendSurface` (G0 to G4 cross-boundary) | fillet crate only | missing as a surfacing tool |
| Box, cylinder, cone, sphere, torus primitives; partial angles; cylindrical box; sphere without poles | `CreateBox:270`, `CreateCylindricalBox:424`, `CreateCone:647`, `:738`, `CreateSphere:854`, `:894`, `CreateTorus:992`, `:1038`, `CreateSphereNoPole:6911`, `CreateConeEllipticEnds:7235` | `lib/geop-lib-sweep/src/shapes` (test helpers), `geop_lib_fixtures::Primitive` (test-only). Users build them from sketches | par: a feature modeller does not need them |
| A 3-D curve from two 2-D views (cross curves) | `SmBrep::CreateFromCrossCurves` `SmBrep.cpp:3247` | none | missing |
| Planar faces from unordered 3-D curves, with holes and several regions | `CreatePlanarFacesWith3DCurves` `SmBrep.cpp:3733` | `boundary.rs` flat fill of one closed loop | worse |
| Faces from curves on a surface | `CreateFacesWithCurves` (`inc/SmBrep.h:435`) | `trim.rs`, `project_curve`, imprint | par |

### 1.4 Queries: projection, sections, silhouettes, inversion

| Capability | SMLib | geop | Verdict |
|---|---|---|---|
| Parallel projection of a curve onto faces | `SmBrep::CreateParallelProjectionCurves` `SmBrep.cpp:4928`, `SmSurface::CreateParallelProjectionCurves` | `ops/geop-ops-edit/src/project.rs`: sketch curves along the sketch normal only (extrude, then imprint) | worse |
| **Normal (closest-point) projection of a 3-D curve onto faces** | `SmBrep::CreateNormalProjectionCurves` `SmBrep.cpp:5033`, `SmSurfaceDropCurve` (`DropAndTrimCurve`) | `ops/geop-ops-edit/src/project.rs` `Along::FaceNormal`: a sheet through the normal lines at the curve's nearest points (`closest_point`), its rails padded by a *proven* bound on what they miss of the normal lines all along (`rail_miss`, by `foot_moving`), imprinted: the projection is an enclosure. The sheet takes any 3-D chain; the operation's argument still only a planar sketch | par (3-D sketch curves not yet pickable) |
| Perspective and rotational projection of curves | `SmProjectedCurve` (`SM_PT_PERSPECTIVE`, `SM_PT_ROTATION`) | none (`view.rs` is parallel only) | missing (low priority) |
| Planar section curves of a body, without a boolean | `SmBrep::CreatePlanarSectionCurves` `SmBrep.cpp:5169`, `SmSLC` | `ops/geop-ops-drawing/src/section.rs` subtracts a box (a full boolean) | worse (cost, solids only) |
| Silhouette curves, parallel view | `SmSurfaceSilhouette`, global solver plus tracer | `ops/geop-ops-drawing/src/silhouette.rs` (exact zero set, Newton tracing) | worse at seeding (see §2) |
| Silhouette curves, perspective view | `SmSurfaceSilhouette(bPerspective, crEye)` | none | missing |
| 2-D hidden-line visibility of curves | `SmDirectedSilhouette` | `hidden_lines.rs` (exact split, ray-classified) | better (geop) |
| Point inversion and closest point on a trimmed face | `SmSurfacePointSolve`, `SmLocalSolve1d/Nd`, `SmGlobalSolver` | `nearest.rs`, `geop-core-topology/src/distance.rs` (branch and bound over trimmed faces and edges) | better (geop) |
| Projecting onto a closed (seam) surface across the seam | `SmPeriodicExtent1d`, periodic domains | `project.rs` clamps to the domain. `closest_point` is global, so correct, but the seeded `project` stalls at the seam | worse (small) |

### 1.5 Tessellation and meshes

| Capability | SMLib | geop | Verdict |
|---|---|---|---|
| Absolute chord-height tolerance | `SmCurveTessDriver::m_dChordHeight`, `SmSurfaceTessDriver::m_dChordHeight` (`inc/SmTess.h:302`, `:368`) | relative: `size / (4 q²)` per edge or face (`rasterize/src/lib.rs`), no absolute value | worse |
| Angle tolerance (normal or tangent turning) | `m_dAngTolDeg` (`SmTess.h:303`, `:369`) | none | **missing** |
| Maximum and minimum edge length, aspect ratio, minimum segment count | `m_dMaxEdgeLength3D`, `m_dMinEdgeLength3D`, `m_dMaxAspectRatio`, `m_lMinSegNumber` | none | missing |
| **Crack-free (watertight) mesh: each edge tessellated once and shared by both faces** | `SmTess::Phase1SetupBrep` `SmTess.cpp:11118` (edges split once in 3-D, refined to node size and against polyline crossings), `SplitFaceEdgesToUVSize:3218`, `SmPolyBrep` adjacency | each face samples its own pcurves, a fixed `n` per coedge (`grid.rs:288`, `uv_loop.rs:29`). The STL docs admit it is "as watertight as those samplings agree" (`stl.rs`) | **worse** |
| Adaptive (quadtree) surface subdivision | `SmTessSrfCache::BuildTree`, `CheckAgainstSurfaceCache` | uniform grid, doubled up to 128 (`grid.rs:80`) | worse |
| Trim handling robustness | `CheckPolygonIntersections:3754`, `CheckPolygonNesting:4123` | exact convex clipping per cell (`clip.rs`) | better (geop) per face |
| View-dependent refinement at silhouettes | `SmViewBasedTessDriver` (`SmTess.h:419`) | none | missing (low priority) |
| Smoothing; advancing front | `SmSmoothingData`, `TriangulateFaceAF:10366` | none | missing (low priority) |
| Mesh decimation (quadric) | `SmPolyDecimate` (`SmPolyDecimate.cpp`, 4.3k lines) | none | missing |
| Mesh booleans, merging, intersection | `SmPolyIntersector`, `SmPolyMerge`, `SmPolySolver` | none | missing |
| Brep from a mesh / triangle bag | `SmBrep::CreateFromPolyBrep`, `CreateFromTriangleBag` | none | missing |
| Tessellating edges only | `SmBrep::TessellateBoundaries(dAngleTolDeg)` | `sample_curve` (`rasterize/src/lib.rs`) | par |

### 1.6 Mass properties

| Capability | SMLib | geop | Verdict |
|---|---|---|---|
| Volume, area, centroid, inertia of solids | `SmBrep::ComputePreciseProperties` (`inc/SmBrep.h:334`, relative accuracy, Romberg / Simpson `SmIntegrator`), `SmFace::ComputeProperties` (tessellation-based) | `geop-core-topology/src/mass.rs` (divergence + Green, exact Gauss–Legendre on polynomial directions, enclosed error), `ops/geop-ops-inspect/src/mass.rs` | **better (geop)** |
| Properties of sheets with a thickness (thin shell) | `ComputePreciseProperties(..., dFaceThickness, ...)` (`SmFace.h`) | solids only (`mass_properties(solid)`) | missing |
| Area moments of faces and sheets (first and second moments) | `rMoments[0..3]` area moments | `face_area` only | missing |
| Wire / curve properties (length, centroid, inertia) | edge properties | length via `measure.rs` | worse (small) |

### 1.7 Exchange and deformation

| Capability | SMLib | geop | Verdict |
|---|---|---|---|
| IGES entities 100/102/104/106/108/110/112/114/118/120/122/124/126/128, clamping periodic knots, validation | `NL_Iges.cpp` (`N_Iges126Crv`, `N_Iges128Srf`, `N_IgesNonPeriodicCrv:3738`, `N_IgesValidate`), `SmCurveBoundedSurface` (type 143/144 trimmed surfaces) | none | **missing** |
| STEP: reading analytic and B-spline types | (not SMLib) | `ops/geop-ops-step/src/import/geometry.rs` (exact for all analytic types, unclamped knots) | geop |
| STEP export of analytic types and pcurves | n/a | exports B-splines only: no `PLANE`, `CYLINDRICAL_SURFACE`, `CIRCLE`, `LINE`, `PCURVE` (`export.rs`) | worse for receivers (no hole recognition) |
| B-spline volumes, free-form deformation of curves and surfaces | `SmBSplineVolume` (3.8k lines), `NL_VolumeAdv.cpp`, `SmSrfInVolume`, `SmCrvInVolume` | none | **missing** |
| Bend / twist / space-bend volumes | `SmBendVolume`, `SmTwistVolume`, `SmSpaceBend`, `SmSpaceUnbend` | none (sheet metal has its own unfold) | missing |

---------------------------------------------------------------------------------------------------

## 2. Special cases to port (work order)

Items are ordered by how often a real CAD user would hit them. Every item follows the AGENTS.md
rules: interval enclosures, no epsilons, a refusal by name where something is unsupported, a
reproducer first, then the fix.

### Tier A: everyday modelling

**1. Revolve: a profile curve touching the axis in its interior.**
- **SMLib:** `SmRotationalSweepGeometry.cpp:334` `CheckCurveSweep` intersects each profile curve with
  the axis line (`IntersectWithLine`, about :390). A solution that is not a curve end gives
  `SM_SC_ROT_ON_CURVE` ("BowTie or ConeApex surface"). The edge is then split there or refused
  (`SmTransSweepGeometry.cpp:261`).
- **geop today:** `ops/geop-ops-extrude-revolve/src/revolve.rs:304-347` classifies each piece at its
  ends and at its stationary points across the axis. A stationary point *on* the axis (an arc tangent
  to the axis at an interior point, or a spline grazing it) gives a side value that only
  `could_be_equal(0)`. That flags neither `left` nor `right`, so nothing is refused. The joint check
  at :392 only looks at joints. The wall then gets an interior pole line that no `SweepLoop.poles`
  entry describes.
- **Port:** in the same loop, an extreme whose side value `could_be_equal(0)` and which is not a
  constrained on-axis joint must be refused by name: "the curve X touches the revolve axis at an
  interior point; split it there, or constrain the point onto the axis". Refuse rather than silently
  make a pole: a pole has to be a joint (see "A pole is one vertex at many (u, v)"). An extension
  would split the curve there with `refine_parameter_at_point` (Newton on `r(t) = 0`, intersected
  with the subdivision box) and mark the new joint a pole. Only do that when the touch is
  *definitely* a tangency, that is when `r'(t)` could be zero there. A transversal crossing is
  already refused.
- **Test:** an arc tangent to the axis, revolved. Assert the refusal names the curve.

**2. Taper (draft) extrude.**
- **SMLib:** `SmPrimitiveCreation.cpp:3604` `CreateTaperExtrude`:
  - the profile is split at G1 breaks and chained;
  - it is offset in its plane by `h·tan(angle)`; convex corners are joined by a line extension
    (`SM_OC_LINEAR_EXTENSION`) or a round arc (`SM_OC_FILLET_CORNER`), and concave corners are cut
    where the offsets cross;
  - each original/offset pair becomes a ruled face;
  - an original curve whose offset was *trimmed away entirely* (a small arc at an inner corner) gets
    a ruled face to a pivot point, which is a cone;
  - caps are built at both ends;
  - self-intersection of the offset returns 1007.
- **geop today:** no taper extrude. Users must extrude and then apply a plastic draft, which fails on
  the same corner cases.
- **Port:** in `lib/geop-lib-sweep`, build the walls directly:
  - each line wall is a plane at the draft angle (exact, ruled);
  - each arc wall is a cone (an exact rational ruled surface between two concentric arcs);
  - a spline wall is the ruled surface between the spline and its offset. Offsetting splines needs
    item 9; until then, refuse splines by name.
  - Reuse the sketch's offset rules (`geop-core-sketch/src/offset.rs`, corners `Round` / `Extend`).
  - An arc whose offset radius reaches zero or below (`radius - d` could be ≤ 0) becomes the cone to
    its apex point. That is a pole at the far cap, and the skin grid already has the pole machinery.
  - A curve too short for its corners: refuse, naming it, as the sketch offset does.
- **Test:** a rectangle with rounded corners whose fillet radius is smaller than the inset, drafted
  inward: the corners must turn into cones to a point.

**3. Path sweep corner at an arc or spline.** Done: walls on their own swept surfaces, rebuilt
(`geop_lib_shell::rebuilt`), sharp or smooth profiles, joints taken apart off a planar path. Left:
a circle whose walls touch inside a station edge (`sweep_a_pipe_round_a_corner_out_of_the_plane`).
- **SMLib:** `SmPrimitiveCreation.cpp:5722` `CreateSweepAlongPlanarPath`:
  - extends every path segment past the corner and sweeps each one;
  - cuts both bodies by the bisector plane, whose normal is the average of the two tangents, flipped
    for an inner corner;
  - when the neighbours are not lines, finds the true cutting curve by intersecting with the bodies;
  - unions the pieces; a closed path gets its last-to-first corner trimmed too.
- **geop today:** line-line mitres only. Other corners are refused (`path_sweep.rs:21`, test
  `corner_at_an_arc_is_refused`).
- **Port:**
  1. Extend each span past the corner exactly: a line by a line, an arc by the same circle
     (`NurbCurve::extended` or a longer arc), a spline by `extended`.
  2. Sweep each span with its frames. The mitre plane is the bisector of the tangents at the corner:
     a free choice of where the two walls meet, and an exact plane.
  3. Trim each wall by that plane with the boolean's imprint (`geop_lib_booleans::imprint`), not by
     projecting the profile. Projecting along the tangent is exact only for straight walls, which is
     why geop limits itself to lines today.
  4. Name the mitre edges after the corner station.
  - Refuse an inner corner whose extended walls reach past the neighbour's other end, naming the
    corner and the span.

**4. Loft to or from a point, and lofts that close on themselves.**
- **SMLib:** `N_CreateRuledSrfBetweenCrvAndPt` `NL_SrfCommon.cpp:2171` (cone to an apex). `N_CreateSkinSrf`
  `NL_SrfAdv.cpp:7899` with `cld = NL_YES` makes a C1 closed skin when `cur[0] = cur[k]`.
- **geop today:** `Section` needs a `Profile` (`loft.rs:88`). `boundary.rs` refuses two edges
  sharing an end. Nothing makes a loft closed in the loft direction.
- **Port:**
  - Add a `Section::Point { point, name }`. Its skin row collapses to the point. Use the existing
    pole machinery (`SweepLoop::poles`, `CoedgeOn::Vertex` in `sweep.rs`), the same way a revolve
    pole is handled.
  - The tangent at the apex is a free choice: Bessel from the neighbours. For a smooth dome, accept a
    direction from the user (then every curve leaves the point along that plane).
  - Closed loft: when the first and last sections are the same sketch, use periodic Bessel tangents
    in `through` (`loft.rs:1100`): the tangent at section 0 comes from sections k-1 and 1. Build
    without caps, and close the skin with the topology the full-turn revolve uses (shared station).

**5. Silhouettes missed between seed grid lines.**
- **SMLib:** `SmSurfaceSilhouette` is an `SmSurfaceTracer : SmGlobalSolver`. Start points come from a
  global subdivision solve on the surface tree, including boundary solutions
  (`FindSolutionsOnBoundaryCurve`) and closed-loop detection (`SmSurfaceTracer.cpp:648`).
- **geop today:** `ops/geop-ops-drawing/src/silhouette.rs:57-62` samples the sign of `g = n·d` on
  12 grid lines with 32 samples each. A closed silhouette loop that fits between two grid lines is
  never seeded: a bump, a small dimple, a fillet patch seen end-on. That is "deciding from samples"
  (AGENTS.md), and the drawing silently loses outlines.
- **Port:** replace seeding with subdivision over `(u, v)` patches of the enclosure of
  `g(u, v) = (S_u × S_v)·d`:
  - drop a patch whose `g` enclosure excludes 0;
  - on each surviving leaf at the handoff size, find points of `g = 0` on its boundary with 1-D
    subdivision along the four sides;
  - a leaf with no boundary zero but a sign change inside holds a closed loop; seed it at a critical
    point of `g` along u (`g_u = 0`, 1-D subdivision on the iso line through the leaf).
  - Keep the tracing as is. Mark traced leaves.
  - Perspective (`m_bPerspective`, `m_vEye`): `g = n·(S - eye)`, the same machinery.

**6. Crack-free tessellation (STL, viewer, picking).**
- **SMLib:** `SmTess::Phase1SetupBrep` `SmTess.cpp:11118` tessellates every edge once in 3-D (to the
  chord, angle and length tolerances), refines it to the face subdivision node size
  (`SplitFaceEdgesToUVSize` :3218), and splits further where polylines cross in UV
  (`CheckPolygonIntersections` :3754). Each face's boundary polygon is those shared points mapped to
  its UV, and `SmPolyBrep` keeps the radial edge adjacency.
- **geop today:** `grid.rs:288` samples each face's loop from its pcurves with `n` points per coedge,
  independently of the edge's own polyline (`rasterize/src/lib.rs:sample_curve`). Two faces' meshes
  disagree along every curved edge. The STL is not watertight (`stl.rs` docs), and the drawn edge is
  not the mesh boundary.
- **Port:**
  1. Sample each `EdgeId` once with `sample_curve` (already adaptive) and keep the 3-D points.
  2. For each coedge, take the face's `(u, v)` at each edge sample by Newton on its pcurve
     (`refine_parameter_at_point`) or by `NurbSurface::project` seeded from the pcurve's
     proportional parameter. These are free choices for rendering, so plain f64 is fine as in
     `clip.rs`.
  3. Clip the grid against *that* polygon, and output the edge's 3-D points (not `S(u, v)`) as the
     boundary vertices, shared by index between the two faces.
  4. In `RasterizedModel`, key boundary vertices by `(EdgeId, i)` so the STL writer can weld them.
  - A pole coedge (`CoedgeGeometry::Vertex`) maps to the vertex point.
- **Test:** an STL of a drilled box has every edge used by exactly two triangles.

**7. Tessellation tolerances: absolute chord height, angle, maximum edge length; adaptive refinement.**
- **SMLib:** `inc/SmTess.h:296-305` for curves and `:366-373` for surfaces (chord height, angle in
  degrees, max and min edge length, max aspect ratio, minimum segment count). The surface cache is a
  subdivision tree (`SmTessSrfCache::BuildTree`).
- **geop today:** one quality `n`. The tolerance is relative to each entity's own size
  (`size / (4 n²)`), so a 1 m plate and a 1 mm fillet get the same *relative* error: large absolute
  error on big faces, waste on small ones. The grid is uniform per face, capped at 128 (`grid.rs:80`):
  a long thin swept wall is either coarse along its length or over-fine across it. There is no normal
  angle criterion, so shading on large-radius faces is poor.
- **Port:** a `TessellationTolerance { chord: f64, angle: f64, max_edge: Option<f64> }`. Refine a cell
  while the midpoint deviation is above `chord`, the corner normals turn more than `angle`, or a side
  is longer than `max_edge`. Use per-cell (quadtree) refinement instead of a uniform grid. Clipping
  stays per cell, so it carries over. Curves use the same three tests. Rendering is the sanctioned
  sampling exception, so f64 is fine.

**8. Free-form surface offset (thicken, shell, offset surface on NURBS).**
- **SMLib:** `N_SrfOffset` `NL_SrfOffset.cpp:678` and `N_ApproxNormalSrfWithSrf` `NL_SrfApprox.cpp:2066`
  make a global approximation to a tolerance. `SmOffsetSurface` is an exact evaluator-based surface.
- **geop today:** `nurb_surface/offset.rs` refuses everything but planes and revolutions with a line
  or arc meridian. Thicken, offset surface and shell fail on any lofted, swept or imported free-form
  face.
- **Port** (the interval way, mirroring `NurbCurve::interpolate_enclosing`):
  1. Sample the offset `S + d·n` on a grid refined per knot span, adaptively while the fit is wider
     than the target, as `fit_pcurve` does.
  2. Fit by tensor-product interpolation: rows with `interpolate_homogeneous`, then columns.
  3. Measure the deviation at a true offset point inside every cell, and widen the control points by
     it (a `widen` for surfaces), so the enclosure holds the true offset.
  4. Keep `(u, v)` correspondence (same parametrisation), so trims transfer.
  - Refuse where `|d| · κ_max` could be ≥ 1 (the offset folds), using `curvature_radius` bounds over
    each patch, naming the face.
  - Item 12's knot removal then keeps the result small.

**9. Planar offset of spline curves (sketch offset, taper extrude, thicken of open sketches).**
- **SMLib:** `N_CrvOffset` `NL_CrvOffset.cpp:1059` is exact for lines and circles and approximates
  everything else to the tolerances `Eg` / `Ep`, by interpolation, least squares and knot removal.
  `N_CrvOffsetPtSampling` `:1217` is the sampling variant.
- **geop today:** `geop-core-sketch/src/offset.rs` refuses splines.
- **Port:** a `NurbCurve2D::offset(d)` in core-geometry. Sample `C + d·N` per knot span (start from
  the source's structure, as `fit_pcurve` does) and use `interpolate_enclosing` with true offset
  points between the samples, so the result *encloses* the true offset.
  - Refuse where `d·κ` could reach 1, a cusp, naming the curve and the parameter. Find that by
    subdivision on the curvature enclosure, never by samples.
  - The sketch can then offset a spline as a driven (non-solved) curve.

**10. Approximation of traced and fitted curves to a tolerance (control-point counts).**
- **SMLib:** `N_FitCrvApprox` `NL_CrvFit.cpp:4482`: interpolate, remove knots, least-squares fit at a
  higher degree, adjust the parameters, repeat. `N_ApproxProcCrvWithCrv` `NL_CrvApprox.cpp:3897` does
  the same for a procedurally evaluated curve.
- **geop today:** every traced intersection curve, silhouette, `fit_pcurve` result and spline sweep is
  a global interpolant with one control point per sample (`interpolate.rs`). Booleans on many-cut
  bodies then carry very heavy edges (see "Many booleans on one body").
- **Port:** after `interpolate_enclosing`, run interval knot removal (item 12). A knot may be removed
  when the removal error's enclosure, plus the existing width, still encloses the true-point checks.
  The width grows by the removal error, honestly. That gives the same enclosure guarantee with a
  fraction of the control points. Stop removing when the width would exceed the entity budget
  (`numerical_accuracy.rs`).

### Tier B: common surfacing and construction

**11. Coons / loft G1 to *curved* neighbouring faces.** *Done*, and the tangency is proven, not
sampled: over a stretch `I` of the side, `f(I) ⊆ f(c) + f'(I)(I − c)` for `f = into·n`, with `n`
and `n'` (`normal_turning`) over the box of `(u, v)` Krawczyk proves the face's foot points lie in
(`enclose_foot`, on the face carried past its edges). The bound's width goes as `|I|²`, so a
millionth of a radian is shown over stretches of about `1e-4`: some thousands of stretches per
side, a second or so. A second-order form (needing the face's third partials) would cut that by
a few. At a corner where the side meets another in the tangent plane, `f` is zero, and the proof
takes `f'` from the corner instead. Two things it needed first: `partials` over a box reaching
over a knot carried one span's polynomial over the next, which encloses nothing there (it now
unites the spans, as curves do); and the corner's twist had to take in how the face's normal
turns along the side (`twist` in `fill.rs`), or the tilt dipped and bulged just off the corner,
shrinking only as `h²` with the spans there. Original plan, for reference:
- **SMLib:** `N_CreateCoonsSrfCrossBoundaryDerivs` `NL_SrfAdv.cpp:22012` and `SmTangentField`
  (`inc/SmTangentField.h`): the cross-boundary tangent along an edge is interpolated between its end
  tangents and projected into the neighbour's tangent plane at `uvcurve(t)`.
  `N_CreateSkinSrfBoundaryContinuity` `:13262` does the same for a loft's first and last section.
- **geop today:** `patch.rs:339` `tangent_to_planes` handles flat neighbours only, and boundary
  surfaces refuse tangency to a non-flat face.
- **Port:** generalise `tangent_to_planes` to a cross-derivative *curve* per side. For each side,
  sample (per knot span of the side) the neighbour's normal at the edge's pcurve points, build the
  in-plane direction square to the edge, and fit it (`interpolate_enclosing`). Set the second control
  row to `side + λ·cross` with compatible knots (degree elevate as today). The twist vectors at the
  corners are the compatibility condition: refuse by name where the two neighbours' cross derivatives
  at a corner are inconsistent (their normals could differ). G1 then holds to the fit's enclosure,
  which validation can check.

**12. Knot removal and degree reduction, for curves and surfaces.**
- **SMLib:** `N_CrvRemoveKnots` `NL_CrvTool.cpp:2286` (Tiller's algorithm, error-bounded),
  `N_CrvReduceDegree` `:4186`, `N_SrfRemoveKnots*`, `N_SrfReduceDegreeToTol` `NL_SrfTool.cpp:5987`,
  `N_CrvsMakeCompatibleApprox` `NL_CrvAdv.cpp:5733`.
- **geop today:** none. `compatible` merges knot vectors exactly, so a loft or Gordon surface through
  many fitted curves carries the union of all their knots.
- **Port:** core-geometry `NurbCurve::remove_knot(t, budget) -> Option<(NurbCurve, Scalar)>`, which
  returns the curve and the removal error as an enclosure. Follow Piegl & Tiller A5.8, computing the
  two candidate control points from either side. Their difference *is* the error, and they are two
  enclosures of the same new point when removal is exact, so *intersect* them; when inexact, *union*
  them (AGENTS: union when both possibilities must be covered). Then:
  - `remove_knots_within(budget)`;
  - `compatible_within(budget)`: exact compatible, then remove knots on all curves together, a knot
    removed only if it is removable from every curve;
  - degree reduction via Bezier pieces (`bezier_pieces`) with the standard bounded error.
  - Surfaces: row by row with a shared decision.

**13. Normal projection of 3-D curves onto faces (wrap / project curve), and parallel projection of
any curve along a vector.** *Normal, done as an enclosure:* the sheet through the normal lines at
the curve's feet, rails fitted through them, is padded by what the rails miss of the lines' ends,
proven over every stretch of the curve by the mean value form (the foot's and the normal's motion
over the box Krawczyk proves the feet in, `NurbSurface3D::foot_moving`); the feet are found more
densely while a cheap look between them sees the rails miss by more than `RAIL_DRIFT`. Left: the
foot followed between the feet found first is the one they continue into, proven unique near, not
proven the nearest (a jump to another branch is not looked for); picking 3-D sketch curves in the
operation. Original plan, for reference:
- **SMLib:** `SmBrep::CreateNormalProjectionCurves` `SmBrep.cpp:5033`, `SmSurfaceDropCurve` (tracer,
  C0 surfaces and curves), `CreateParallelProjectionCurves` `:4928` (any vector, optional distance and
  angle tolerance), `SmProjectedCurve`.
- **geop today:** `ops/geop-ops-edit/src/project.rs` projects only a sketch's curves along its plane
  normal, via extrude and imprint.
- **Port:**
  - *Parallel:* generalise the existing route. Extrude the curve, now a 3-D edge or sketch3d curve,
    along any direction into a sheet with `extrude` over a `Path` along `dir`, then imprint. It is
    exact and already works for faces folding back.
  - *Normal:* trace the foot-point curve on the surface. `(u(t), v(t))` minimises
    `|S(u, v) - C(t)|`, so `F = (S - C)·S_u = (S - C)·S_v = 0`. Use a predictor-corrector like the
    silhouette tracer, with steps sized by the source curve's knot spans. Start points come from a
    global closest point (`least_distance`) at each end and at every place where the foot point
    jumps: subdivide in `t` on where `closest_point` changes face or patch.
  - Where the Jacobian could be singular (the curve passes through a centre of curvature), refuse by
    name.
  - Fit the `(u, v)` pcurve with `fit_pcurve`-style enclosure, and lift the 3-D curve by evaluation.

**14. Tapered helix / conical spiral / flat spiral (springs, tapered pipe threads such as NPT).**
*Done for pipe threads:* `Helix::taper`, the tapered screw, and NPT 1/16 to 2 inch in the hole
operation (`ops/geop-ops-hole/src/pipe.rs`): a tapped hole reamed to the 1:16 taper, its cosmetic
thread following it, a modelled thread swept on the tapered screw. Walls are recognised as any
revolution of a straight profile (`NurbSurface3D::as_round_wall`, taper 0 for a cylinder), never
as "a cone". A flat spiral is not done.
- **SMLib:** `N_CrvApproxSpiral` `NL_Spiral.cpp:88` (radius r0 to r1, turns, handedness),
  `N_CreateSpiralSrf` and `N_CreateSpiralSrfTaper` `NL_CrvNurb.cpp:6829` (a profile swept along a
  spiral).
- **geop today:** `helix.rs:161` has a constant radius, and screw sweeps (`revolve.rs:151` `screw_path`)
  use it.
- **Port:** extend `helix_rows` with a radius that grows linearly with the angle. Per quarter-turn
  span the radius changes linearly, so the planar spiral is no longer a circle. Keep the rational
  quadratic shape scaled by the span's mean radius and measure the radial error the same way the
  0.53 % pitch error is measured today. State it in the docs, or enclose it by widening, which is
  better. Extend `screw` to take a taper angle and refuse a taper that reaches the axis within the
  turns.

**15. Sweep frame and law options.**
- **SMLib:** `N_CreateSweepScale` `NL_SrfAdv.cpp:15353`:
  - `curZ`: an explicit binormal (local Z) as a function of t;
  - `curS`: a vector scale (x, y, z) as a function of t;
  - `tflg`, `tc`: interpolate the trajectory exactly, so the path is an iso-curve of the wall;
  - `E`: an approximation tolerance.
  `SmSweepOptions` (`inc/SmPrimitiveCreation.h`): `bTranslationalSweep`, `bMoveProfileCenterToPath`,
  `bOrientProfilePerpendicularToPath`.
- **geop today:** `Orientation::{FollowPath, FixedNormal}`, linear `twist` and `end_scale`, rails.
- **Port:**
  - `Orientation::FixedBinormal(direction)`: the frame's e2 is kept square to the path *and* in the
    plane holding `direction` (pull direction). Refuse where the path tangent could be parallel to
    `direction`, naming the parameter.
  - `Orientation::NormalToFace(face)`: e2 is the face normal at the path's foot points. The path must
    lie on the face, checked with `surface_could_contain`.
  - Law curves for twist and scale: a 1-D NURBS of the path's normalised length.
  - A fixed binormal is exactly what a helical path needs to keep a thread profile radial. Today only
    `screw` does that.

**16. Sweep approximation error control.**
- **SMLib:** `E` in `N_CreateSweepScale`, `dAppoxTol3d` in `CreateCurveSweep`.
- **geop today:** `path_sweep.rs:117` `SECTIONS_PER_KNOT_SPAN = 4` and `REFLECTION_STEPS = 8`, fixed.
  The body is consistent (the frames define it), but how far the frames are from a true
  rotation-minimising frame, or from the rails, is never measured. A tight spline path twists the
  profile visibly.
- **Port:** after building a spline span, measure the frame at the midpoints between sections against
  the double-reflection frame computed finer. Halve the span (more sections) while the angle exceeds
  a step-size tunable. That is allowed: it is "how hard the search tries", not correctness. Record the
  section count in context, as `trace_one_side` does.

**17. N-sided G1 hole fill.** *Done* (`NurbSurface3D::fill`, used by `boundary.rs`): the inner
polygon kept, which keeps every edge of the loop whole, unlike SMLib's split at the edges'
midpoints. What made it converge, for the next one: the corner where two quads and the polygon
meet is a flat point (degree-5 spokes, field constant to first order), and the cross derivatives
either side of a spoke are made in one plane field rather than each made tangent to it. Along a
curved neighbour the tilt has to start from nothing at the corner with a slope that takes in how
the face's normal turns (`Neighbour::turning`); collocation alone left a bump there that uniform
refinement shrank only as `h²`, and halving only the spans that stray fixes it in a few rounds.
Original plan, for reference:
- **SMLib:** `N_FillNSidedHole` `NL_SrfAdv.cpp:18585`: k+1 patches meeting at a centre point `CI`
  with a tangent plane `NI` (computed or given), interpolating the boundary curves and the
  cross-derivative curves `der` to within `eps` degrees.
- **geop today:** `boundary.rs` (3 or ≥ 5 edges): Coons quads about a flat inner polygon, C0 across
  the spokes.
- **Port:**
  1. Choose the centre and its tangent plane (free choices: the average of the boundary midpoints,
     the plane fitted through the cross tangents).
  2. Build the spokes as cubics leaving the boundary midpoints along the cross-derivative (item 11)
     and arriving in the centre plane.
  3. Make each quad patch with item 11's G1 Coons, using shared cross-derivative curves along each
     spoke, so neighbours are G1 across spokes. Corner twists must agree at the centre: choose them
     from the tangent plane.
  - Refuse when the loop's cross tangents at a corner do not lie in one plane (a non-G1 boundary),
    naming the vertex.

**18. Extend surface: by true distance, to a curve, tangent (ruled) option, trimmed faces.**
- **SMLib:** `N_SrfExtendByDist` `NL_SrfAdv.cpp:661` (recursive until the distance is met),
  `N_SrfExtendToCrv` `:2711` (a given curve becomes the new boundary), continuity `NL_CMAX` or `NL_G1`.
  `N_CrvExtendByDist` `NL_CrvAdv.cpp:630`, `N_CrvExtendToPt` `:1656`.
- **geop today:** `ops/geop-ops-surface/src/extend.rs` extends naturally by a *parameter* amount
  ("more or less" the distance), only for a face that is its surface's whole patch.
- **Port:**
  - The distance: solve for the parameter `by` with `length` along the cross iso-curve at the edge's
    middle, by 1-D Newton on arc length (`parameter_at_length_fraction` exists).
  - A tangent extension: ruled from the edge along the cross tangent (degree 1 in the extension
    direction, joined C1), with `ruled` and `join`.
  - Trimmed faces: extend the underlying surface, keep the trim except along the chosen edge, and
    re-splice the edge's offset copy (`splice_edge_into_face`).
  - Refuse by name when the extended patch would run into a pole (degenerate row).

### Tier C: occasional or specialist

**19. Planar faces from unordered curves with holes and several regions.**
- **SMLib:** `SmBrep::CreatePlanarFacesWith3DCurves` `SmBrep.cpp:3733` chains, finds nesting, and
  makes one face per region with holes.
- **geop:** `boundary.rs` fills one loop.
- **Port:** chain by shared vertices (topologically, as `knit` does), classify nesting with
  `loop_signed_area` and `face_contains`, and build the faces with `BodySpec`.

**20. Splitting at C0 / C1 creases (imported B-splines, swept splines).**
- **SMLib:** `CreateFacesFromSurface` `SmBrep.cpp:4762` (`SM_CT_C0`, `C1`, `C2`),
  `N_CrvDecomposeAtG1Continuity` `NL_CrvTool.cpp:1195`, and `bTestContinuity` in
  `CreateLinearSweep` / `CreateRotationalSweep`.
- **geop:** a full-multiplicity interior knot leaves a crease inside a face. Its normal is undefined
  along the crease, which upsets silhouettes, fillets and tangency.
- **Port:** in the STEP importer and before sweeping, split at knots of multiplicity ≥ degree, where
  the control points on either side are definitely not collinear, which is a topological test on the
  control net. That gives two faces and a sharp edge.

**21. Degenerate spans and duplicate control points in imported curves.**
- **SMLib:** `N_CrvRemoveDegenSegs` `NL_CrvTool.cpp:4518`, `N_CrvRemoveDuplicateCPts`,
  `N_SrfRemoveDegenPatch` `NL_SrfTool.cpp:6833`.
- **geop:** none. A zero-length span gives a zero tangent and stalls `refine_crossing`.
- **Port:** at import, remove spans whose control points could all be equal by knot removal (exact
  there), and refuse a curve that is entirely degenerate.

**22. Planar sections without a boolean.**
- **SMLib:** `SmBrep::CreatePlanarSectionCurves` `SmBrep.cpp:5169` returns 3-D and UV section curves
  per face.
- **geop:** the drawing subtracts a box (`section.rs:28`).
- **Port:** a `section_curves(model, plane)` in `geop-lib-booleans` that runs the face × plane
  intersection tracer and the edge × plane crossings only. It works for sheets too, and is the basis
  for slicing and measurement.

**23. Perspective projection and silhouettes** (`SmProjectedCurve SM_PT_PERSPECTIVE`,
`SmSurfaceSilhouette(bPerspective)`). Needed only for perspective drawing views. Add with item 5.

**24. Biarc / arc approximation for DXF and CAM.**
- **SMLib:** `N_FitBiArc` `NL_CrvFit.cpp:767`, `N_ApproxCrvWithArcs` `NL_CrvApprox.cpp:83`.
- **geop:** splines go to DXF as polylines (`geop-lib-sheet/src/dxf.rs:113`); laser cutters prefer
  G1 arcs.
- **Port:** biarcs between points with tangents, refined while the deviation (measured with
  `least_distance`, not samples) exceeds the export tolerance.

**25. Mass properties of sheets and wires.**
- **SMLib:** `ComputePreciseProperties(..., dFaceThickness, ...)` and area moments.
- **geop:** solids only.
- **Port:** the same Green integration with `F` such that the integrand is the area element times
  1, x, x², and so on. That is `|S_u × S_v|`, which is not polynomial, so use adaptive Gauss–Kronrod
  as for spheres. For a sheet of thickness t: mass = ρ·t·area, centroid and inertia from the area
  moments.

**26. Closed-surface projection across the seam.** `project.rs` clamps `(u, v)`. On a closed surface,
detected by its first and last row being able to coincide, wrap the iterate instead. `closest_point`
is already global, so this only matters to seeded callers.

---------------------------------------------------------------------------------------------------

## 3. Whole features missing

1. **IGES import and export.** `NL_Iges.cpp` (4.4k lines) covers entity types 100, 102, 104, 106,
   108, 110, 112, 114, 118, 120, 122, 124, 126, 128, plus `SmCurveBoundedSurface` for 143/144
   trimmed surfaces, periodic-to-clamped knots (`N_IgesNonPeriodicCrv:3738`) and validation. Many
   legacy and CAM workflows still exchange IGES. A geop port would reuse the STEP importer's
   structure: plain `f64` definitions, then exact NURBS (`import/geometry.rs`), then the shared body
   builder.
2. **Surface and curve fitting to scattered and grid points** (`NL_SrfFit.cpp`, 19k lines;
   `NL_SrfLstSqFit.cpp`, 9k; `NL_CrvFit.cpp`, 20k): interpolation of grids, least squares to a
   tolerance, fitting with a boundary, periodic fitting, fitting points with normals. This is reverse
   engineering and point-cloud surfacing. geop has only global interpolation through ordered points.
3. **Knot removal, degree reduction, and approximation to a tolerance** (items 10 and 12). This is
   the missing "reduce" half of geop's curve and surface toolkit, and it limits every fitted, traced
   and lofted entity.
4. **Mesh pipeline**: `SmPoly*`, about 40k lines. It covers a polygonal B-rep with adjacency, mesh
   booleans (`SmPolyIntersector`, `SmPolyMerge`), quadric decimation (`SmPolyDecimate`), smoothing,
   brep from a mesh (`CreateFromPolyBrep`, `CreateFromTriangleBag`), and mesh mass properties.
   geop has a render-only triangle list.
5. **Free-form deformation**: B-spline volumes (`SmBSplineVolume`, `NL_VolumeAdv.cpp`) with curves
   and surfaces mapped through them (`SmCrvInVolume`, `SmSrfInVolume`), and analytic bend, twist and
   space-bend volumes (`SmBendVolume`, `SmTwistVolume`, `SmSpaceBend`/`Unbend`). These are flex and
   twist features and morphing.
6. **Generic topology sweep**: `SmTopologySweep::DoSweep` sweeps any vertices, edges or faces of a
   B-rep into higher dimensions, with repetitions and an envelope sweep (silhouette imprinted, then
   swept; `DoAdvSweep`). In geop, extrude, revolve and sweep take only sketches. Extruding a face of a
   solid, or edges into a sheet, is missing.
7. **Shape-editing tools for curves and surfaces** (`NL_CrvShape.cpp`, `NL_SrfShape.cpp`): moving a
   point to pass through a target, warps, bends and flattening. These are direct-modelling helpers.
8. **Surfacing constructions without a geop counterpart**: swung surfaces (`N_CreateSwungSrf`),
   skinning along a spine (`N_CreateSkinSpine`), blend surfaces between curves on two surfaces with
   G0 to G4 cross-boundary continuity (`SmBlendSurface`, `CreateBlendPrimitive`), tensor-product
   surfaces from four boundary and cross-derivative curves (`N_CreateTensorProductSrf`), and
   ellipsoids, paraboloids and hyperboloids.
9. **Curve-from-views**: `SmBrep::CreateFromCrossCurves` makes a 3-D curve as the intersection of two
   extruded 2-D curves. Composite curves (`SmCompositeCurve`) as a curve type.
10. **Analytic STEP export**: not SMLib-specific, but in scope for `geop-ops-step`. geop writes only
    B-splines and no pcurves, so receivers cannot recognise holes or cylinders. Use `shape.rs`'s
    `as_line`, `as_arc`, `as_plane` and `as_cylinder` to write `LINE`, `CIRCLE`, `PLANE` and
    `CYLINDRICAL_SURFACE` (and cones, spheres and tori via `axis_of_revolution`), and add
    `SURFACE_CURVE` with `PCURVE`s.

## 4. Where geop is already better (keep, do not port)

- **Mass properties.** Divergence and Green's theorem over exact trimmed faces, an exact
  Gauss–Legendre inner integral where the surface is polynomial, and the error enclosed. SMLib uses
  Romberg / Simpson with a relative accuracy (`SmIntegrator`), or tessellation sums.
- **Closest point and least distance.** A branch and bound over control-hull bounds proves the global
  minimum, including on trimmed faces (`nearest.rs`, `distance.rs`). SMLib's local and global solvers
  rely on tolerances.
- **Path sweep frames.** Lines and arcs are carried exactly (torus walls), corner mitres are exact,
  and a bend tighter than the profile is refused by name.
- **Loft correspondence.** Corners, matched points, splitting of runs, guides up to affine.
  `N_CreateSkinSrf` only makes the curves compatible.
- **Revolve classification.** Poles from constraints, curves on the axis, and crossing refused, all
  three-valued. Only the interior-touch case (item 1) is missing.
- **Shape recognition** on the control net with three-valued answers (`shape.rs`).
- **Hidden-line removal.** An exact split at projected crossings and stationary points, then ray
  classification. That is more complete than `SmDirectedSilhouette`.
- **Per-face trim clipping in the rasterizer.** Exact convex clipping per cell, so there are no
  reconstruction errors. The weaknesses are only crack-freeness and the tolerances (items 6 and 7).
- **Exact rational helix on its cylinder.** SMLib's is a cubic interpolation.
