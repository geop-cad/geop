# geop feature-gap analysis (October 2026)

Against Fusion 360 (F), Creo (C), Onshape (O), NX (N), SolidWorks (S) and the
SMLib kernel (K). Read-only survey of `/home/tobiasjacob/geop` at
`7ca6ec82` (branch `sm-lib-port` merged), `ROADMAP.md`, `DEVELOPERS.md`,
`docs/smlib-comparison/*.md`, every op crate's operation `INFO`, and the
assembly / solver / cache / view code. Vendor columns are from the products'
published feature sets (2025–26 releases), not from a hands-on audit; the
"user hits it" column is for the ROADMAP's yardstick: an engineer designing
a robot (structure, drive train, enclosures, wiring, fasteners).

Constraints every recommendation respects (AGENTS.md): every curve and
surface is a NURBS, no analytic type dispatch; one general mechanism rather
than a feature per vendor checkbox; an operation is a `PartOperation` in its
own op crate, depending only on `core/`, `lib/` and `geop-ops-model`;
algorithms two ops share go in a `geop-lib-*`; what ops share (targets,
references) is vocabulary in `geop-ops-model`; no epsilons, refusals by name;
scaling guarded by counts, not timings.

---

## 1. Capability table

Legend — vendors: F Fusion, C Creo, O Onshape, N NX, S SolidWorks, K SMLib
(kernel only, so "—" for app features). geop: **done** / **partial** /
**missing**. Hit: how often the robot engineer reaches for it (H/M/L).

### 1.1 Sketching and reference geometry

| capability | vendors | geop | hit |
|---|---|---|---|
| 2-D sketch: lines, arcs, circles, splines, slots, polygons, fillets/chamfers | F C O N S | done (`core/geop-core-sketch`, `ops/geop-ops-sketch`) | H |
| Geometric + dimensional constraints, DOF/conflict report | F C O N S | done (Coincident…Angle, Radius, Symmetric, Fix; conflicts named) | H |
| Sketch offset, mirror, linear/circular sketch pattern, project/convert edges | F C O N S | done (offset of splines and several chains; project curve) | H |
| Ellipse, partial ellipse, conic, parabola | F C O N S | **missing** (`CurveKind` = Line, Arc, Circle, Spline) | M |
| Equation-driven / parametric curve (x(t), y(t)) | F C N S | missing | L |
| Sketch text (fonts) for emboss/engrave/labels | F C O N S | missing | M |
| Sketch blocks / reusable sketch library | C N S | missing | L |
| Image / DXF / SVG underlay or import into sketch | F C O N S | missing (DXF/SVG only exported) | M |
| 3-D sketch, helix/spiral | F C O N S | done (`geop-ops-sketch3d`; helix exact, tapered) | M |
| Datums: planes, axes, points, frames (mate connectors) | F C O N S | done (`geop-ops-reference-geometry`) | H |
| Curve through points / XYZ table, composite curve, intersection curve | F C N S K | partial (section curves as wire via imprint; no point-table curve) | M |
| Spiral (planar), variable-pitch helix | C N S | partial (helix with taper; no variable pitch, no flat spiral) | L |

### 1.2 Part features

| capability | vendors | geop | hit |
|---|---|---|---|
| Extrude / revolve / sweep / loft (guides, twist, to point) | F C O N S K | done (`geop-ops-extrude-revolve`, `geop-lib-sweep`) | H |
| Extrude end conditions: blind, through, up to next/face/body, offset-from-face, symmetric, draft | F C O N S | done mostly (up-to-next, through all, draft); up-to-surface-with-offset partial | H |
| Thin-feature extrude/revolve (wall from open profile) | F C O N S | missing (thicken exists for sheets only) | M |
| Holes (ISO/NPT, cbore/csink/tapped, threads) | F C O N S | done (`geop-ops-hole`) | H |
| Slot holes, hole series through assembly, hole tables | C O N S | missing | M |
| Fillets: constant, variable, chains, corners, chamfers | F C O N S K | done; gaps: rollover over crease, corners of different radii smooth, setbacks, face fillet, full round, conic/curvature-continuous section (`docs/smlib-comparison/fillets.md`) | H |
| Shell (inward, outward, multi-thickness, voids) | F C O N S K | done mostly; free-form edge copies, global self-intersection detection still open (`offsets.md` §6) | H |
| Draft (planar), rib, lip & groove | F C O N S | done (`geop-ops-plastic`, experimental) — draft only on planar faces; parting-line / step draft missing | M |
| Linear / circular / mirror patterns of bodies and features | F C O N S | done (`geop-ops-pattern`) | H |
| Curve-driven, sketch-point-driven, fill, table-driven patterns | F C O N S | **missing** | H (bolt circles done; hole patterns from sketch points are everyday) |
| Pattern skip instances / vary dimension per instance | F C O N S | missing | M |
| Emboss / engrave / wrap onto face | F C O N S | missing (project curve exists) | M |
| Coil / spring / thread (modelled) | F C N S | done via helix sweep + modelled ISO/NPT threads | M |
| Variable-section sweep (section driven by parameter along path) | C N S | missing | L |
| Split body/face, imprint, section curves | F C O N S K | done (`geop-ops-boolean`: split, imprint, section) | M |
| Booleans incl. XOR, lumps/voids, multi-body | F C O N S K | done; N-ary, cellular merge, wire×solid missing (`booleans.md`) | H |
| Direct edit: move/offset/replace/delete face, defeature, merge faces | F C O N S K | done (`geop-ops-edit`, `geop-lib-shell` rebuild) | H |
| Scale / deform body (taper, twist, bend, pinch, shear) | F C N S K | done (`geop-ops-pattern`) | L |
| Suppress / unsuppress a feature, conditional features | F C O N S | **missing** (Seek is a rollback bar, not suppression) | H |
| Feature reorder, rollback, copy/paste steps, undo | F C O N S | done (`Command::Move/Seek/Copy/Paste/Undo`) | H |
| Weldments / frames (structural members along a skeleton, end trims, cut list) | F(frame) C N S O(frame) | **missing** (T-slot std parts exist but placed one by one) | H for robot frames |
| Mold tools (parting line/surface, core/cavity) | C N S F | missing | L |
| Subdivision (T-spline-like) modelling | F C(Freestyle) N | done (`geop-ops-subd`) | L |
| Mesh → B-rep, STL import | F N S K | done (`ImportStl`, `BodySpec::of_triangles`) | L |
| Multi-body parts, body folders, combine | F C O N S | done | H |

### 1.3 Surfacing

| capability | vendors | geop | hit |
|---|---|---|---|
| Boundary / fill (N-sided, tangent), UV network, ruled | F C O N S K | done (`geop-ops-surface`) | M |
| Offset, thicken, knit, trim, extend surface | F C O N S K | done; free-form offset with proven bound done; variable offset missing | M |
| G2/G3 continuity control, blend surfaces, match | C N F(alias) K | missing (G1 only) | L |
| Curvature comb, zebra, draft analysis, deviation | F C O N S | **missing** (no analysis inspectors) | M |
| Untrim / replace face by surface | F C N S K | partial (ReplaceFace onto another face's surface) | L |

### 1.4 Sheet metal

| capability | vendors | geop | hit |
|---|---|---|---|
| Base flange, edge flange, hem, corners, cuts across bends, flat + DXF | F C O N S | done (`geop-ops-sheetmetal`) | H |
| Sketched bend, jog, lofted bend, convert solid → sheet metal | F C O N S | **missing** | M |
| Forming tools (louvers, dimples, bridges), tabs & slots | C N S O | missing | L |
| Gauge/bend tables, K-factor per material | F C O N S | partial (K-factor; no gauge tables) | M |
| Unfold / refold to add features in flat | C N S | missing | L |

### 1.5 Assemblies

| capability | vendors | geop | hit |
|---|---|---|---|
| Multi-file assemblies, sub-assemblies (rigid/flexible), instancing | F C O N S | done (`geop-ops-assembly`, `core/geop-core-program/placed`) | H |
| Constraint mates: coincident, concentric, parallel, perpendicular, distance, angle | F C O N S | done (`residuals.rs::Kind`) | H |
| Joints: revolute, slider, cylindrical, fastened, limits | F C O N S | done (`joints.rs`) | H |
| Ball, planar, pin-slot, universal, tangent/cam-follower, path mate, width/symmetric | F C O N S | **missing** | H (ball, planar, pin-slot, cam), M (others) |
| Gear, rack & pinion, screw couplings | F C O N S | done | M |
| Belt/chain/cable coupling, linear-linear coupling | C N S O | missing | M |
| Part patterns (linear/circular, pattern-driven by a feature) | F C O N S | partial (`Part pattern`; feature-driven pattern of fasteners missing) | H |
| Smart fasteners (populate holes with screws) | C S N(reuse) O(partial) | missing | H |
| Mirror component, derived part (copy with mods) | C N S O | missing | M |
| Top-down: in-context editing, skeleton models, external references | C N S O F | **missing** (a part cannot read a sibling's geometry) | H |
| Interference / clearance check | F C O N S | done (`geop-ops-inspect/interference.rs`; O(n²) pairs, see §3) | H |
| Collision while dragging, motion envelope | F C N S | missing | M |
| Exploded views (steps, trails) | F C O N S | **missing** (roadmap Next) | M |
| Motion study / animation by joint drivers, plots | F C O N S | missing (joint values set by hand, URDF export) | M |
| Display states, hide/isolate, appearance per instance | F C O N S | partial (colour per part via input; no per-instance display states) | M |
| BOM flat/indented, CSV, balloons | F C O N S | done (`geop-ops-bom`) | H |
| Configurations (part families) chosen per instance | F C O N S | partial (`LookupTable` + `size` input; standard parts use it; no user-level configuration table with suppression, see §2 WP-C) | H |
| Large-assembly modes (lightweight, LOD, SpeedPak, simplified reps) | C N S O(server) | **missing** (§3) | H at 1000+ parts |
| Standard parts library | F C O N S | done (`geop-lib-stdlib`, 16 families) | H |
| Wire harness routing | C N S F | done (`geop-ops-harness`); collision checks missing | M |
| URDF export | (none built-in; plugins) | done — geop ahead | M |

### 1.6 Drawings and MBD

| capability | vendors | geop | hit |
|---|---|---|---|
| Projected, section views, hidden lines, title block, BOM table, balloons | F C O N S | done (`geop-ops-drawing`) | H |
| Detail, auxiliary, broken-out, crop, alternate-position views | F C O N S | **missing** | H (detail), M (others) |
| Ordinate dimensions, tolerances (±, fits), GD&T frames, datum symbols | F C O N S | missing (roadmap Next) | H |
| Hole tables, revision tables, weld / surface finish symbols | C N S O | missing | M |
| PDF export, sheet templates/formats | F C O N S | missing (SVG, DXF only) | M |
| Model-based definition (PMI in 3-D, STEP AP242 PMI) | C N S O F | missing | L–M |

### 1.7 Inspection

| capability | vendors | geop | hit |
|---|---|---|---|
| Measure, mass properties (exact), section view | F C O N S K | done (`geop-ops-inspect`, `web/src/section.ts`) — geop ahead on exactness | H |
| Min distance / clearance between bodies | F C O N S K | done in kernel (`nearest::least_distance`); as user inspector only via interference "touch" | M |
| Curvature, draft, thickness, undercut analysis | F C O N S | missing | M |
| Compare two versions of a part (geometry diff) | C N S O | missing | L |

### 1.8 Data exchange

| capability | vendors | geop | hit |
|---|---|---|---|
| STEP AP214 import/export, assemblies as products | F C O N S K | done (`geop-ops-step`; 322/361 corpus valid) | H |
| STEP assembly import as placed parts | F C O N S | missing (roadmap Next) | H (vendor CAD of motors, gearboxes) |
| STEP AP242 (PMI, colours, layers) | F C O N S | missing | M |
| IGES | F C N S K | missing | L |
| Parasolid x_t / JT / native formats | F C O N S | missing (Parasolid read is licensed; JT later) | M |
| STL import/export | F C O N S | done | M |
| 3MF / OBJ / glTF export | F O S N | missing | M (3MF for 3-D printing robot parts is H for hobby robotics) |
| DXF / SVG import to sketch | F C O N S | missing | M |
| URDF / SDF / MJCF export | (plugins) | URDF done; SDF/MJCF missing | M |

### 1.9 Platform, collaboration, scale

| capability | vendors | geop | hit |
|---|---|---|---|
| Text programs in git; branch/merge/diff | O (built-in versions/branches) | done by design (programs are text, VS Code extension) — geop ahead on openness | M |
| Real-time multi-user editing | O F(partial) | missing | L |
| Release / revision / PDM-lite (part numbers, states) | O F C N S | partial (`PartProperties`) | L |
| Scripting / API | F(Python) O(FeatureScript) N(NXOpen) S(API) C(Toolkit) | done in kind (Rust ops, MCP agent tools `mcp__geop__*`, `geop` CLI) | M |
| Multi-threaded rebuild | C N S O(server) | **missing** (kernel single-threaded, `Rc` throughout; rayon only in tests) | H at scale |
| Persistent build cache across sessions | S (rebuild files), O (server) | partial (`.geopc` only for STEP imports, `core/geop-core-program/src/cache.rs`) | H at scale |
| CAM, FEA, rendering, generative | F C N S O(apps) | missing — out of scope per ROADMAP; note FEA-lite (linear static of a part) and rendering export (glTF) as cheap bridges | L |

---

## 2. Prioritised work list

Ordered by (frequency of use for the robot engineer) × (how blocking) ÷
size. Size: S ≈ days, M ≈ 1–2 weeks of agent rounds, L ≈ several rounds.
"Mechanism" names the one general thing to build; each list of vendor
features it covers is what *falls out* of it.

| # | item | mechanism (fits the architecture) | crates | size | deps |
|---|---|---|---|---|---|
| 1 | **Rigid clusters in the assembly solve** | Before solving, union-find blocks tied by `Fastened` joints and fully-constraining mate sets into one rigid block (one pose); solve the reduced system; expand poses after. General: a term whose rows remove all 6 DOF between two blocks merges them. Proven: the enclosure is of the reduced system, expanded exactly by composition | `core/geop-core-math/solvers/system` (`groups`, a `merge` pass), `lib/geop-lib-constraints/solve.rs`, `ops/geop-ops-assembly/mechanism.rs` | M | — |
| 2 | **Sparse / block solve inside a group** | Block-sparse Jacobian (`Linearized` is already per-block); Householder per group stays, but on a block-sparse structure (or Schur complement over tree-shaped chains). Keep complete pivoting rule | `core/geop-core-math/solvers/least_squares.rs`, `linear.rs`, `system/enclose.rs` | L | 1 (measure first) |
| 3 | **Persistent build cache for part programs** | Generalise `Cache::made_of` from "STEP text → bodies" to "(program text digest, inputs, kernel build) → built `NamedModel` + `PartView`"; `.geopc` beside every `.geop`; `Runner` asks it before building a placed part | `core/geop-core-program/cache.rs`, `runner.rs`, `placed.rs`, `cad/geop-cad-cli`, `cad/geop-cad-web` (store) | M | — |
| 4 | **Parallel builds of distinct components** | The runner builds placed parts' programs as independent jobs (distinct components share nothing); requires `Send` values at the part boundary (serialised `NamedModel` from #3 is the natural seam) and wasm threads (`SharedArrayBuffer`) in the web worker | `core/geop-core-program/runner.rs`, `web/src/kernel.worker.ts` | L | 3 |
| 5 | **Level-of-detail views of placed parts** | `PartView` per component at two tessellation tolerances + bounding box; the scene sends the coarse one for parts small on screen, and a box-only "lightweight" component until opened; picking uses a per-component BVH | `core/geop-core-rasterize`, `core/geop-core-editor/view`, `web/src/placed3d.ts` | M | — |
| 6 | **Configurations: named input sets + suppression** | A program's configurations are a `Table` step (generalise `LookupTable`) whose rows set parameters *and* a boolean `active` formula per step; the runner skips a step whose `active` is false (and its dependents refuse by name). `AddPart` picks a configuration by row key like `size` today. Covers configurations, design tables, suppression, part families | `core/geop-core-program/program.rs` (step `active`), `runner.rs`, `ops/geop-ops-model/parameters`, `ops/geop-ops-assembly/add_part.rs` (choose row) | M | — |
| 7 | **More joint kinds from one frame-pair formulation** | Every joint is "frame B relative to frame A lies in a subgroup of SE(3)": revolute, slider, cylindrical, fastened today; add planar, ball, universal, pin-slot (point on curve), as residual sets over the relative pose, coordinates as `Unknown`s. One `JointKind` table, not per-kind code | `ops/geop-ops-assembly/joints.rs`, `residuals.rs` | M | — |
| 8 | **Curve/point constraints between parts (tangent, cam, path)** | Mates whose entity is a face or curve, not its axis/plane: residual = signed distance via `NurbSurface::project`/curve projection, with the foot point as an `Unknown`. Gives tangent mate, cam follower, path mate, pin-in-slot along any curve | `ops/geop-ops-assembly/residuals.rs`, `ops/geop-ops-model/mate_entity.rs` | M | 7 |
| 9 | **Belt/chain/linear couplings** | Generalise `CouplingKind` to "linear relation between joint coordinates" (ratio, offset); gear/rack/screw/belt are presets | `ops/geop-ops-assembly/mates.rs` | S | — |
| 10 | **In-context references (top-down design)** | A step of part P may read an element of a sibling instance through the parent assembly: `ElementReference` with a `Located` path up (`/..:/bracket:face`) resolved by the runner in the parent's frame, recorded as a dependency so edits rebuild P. Skeleton models then are just parts others read | `core/geop-core-program/placed.rs`, `reader.rs`, `element.rs`; `ops/geop-ops-model/placed` | L | 3 (to keep rebuilds cheap) |
| 11 | **Feature-driven patterns: from sketch points, along a curve, by table** | One `Spacing` source = "a list of frames": from sketch points, a curve (equal arc length, tangent-aligned), a table of offsets, a fill region; linear and circular become presets. Used by body, feature and part patterns alike | `ops/geop-ops-model/spacing` (vocabulary), `ops/geop-ops-pattern`, `ops/geop-ops-assembly/part_pattern.rs` | M | — |
| 12 | **Smart fasteners (populate holes)** | A part pattern whose frames are the hole features of a placed part (holes already carry ISO size); picks the `std:` screw family row by hole size and depth. Falls out of #11 (frames from features) + #6 (configuration row by size) | `ops/geop-ops-assembly/part_pattern.rs`, `ops/geop-ops-hole` (read features), `lib/geop-lib-stdlib` | M | 6, 11 |
| 13 | **Frames / weldments** | Structure op: a 3-D sketch wire + a profile (sketch or `std:` T-slot section) → per segment a `geop-lib-sweep` sweep, ends trimmed/mitred by the boolean `trim up to next face` already in `geop-lib-booleans`; each member its own body with length property → BOM cut list | new `ops/geop-ops-frame`, uses `geop-lib-sweep`, `geop-lib-booleans`, `geop-ops-model` (sketch3d target) | M | — |
| 14 | **Exploded views** | An exploded state is a named set of pose offsets per instance (state values, not geometry) stored by a step `Explode`, applied on top of solved poses; drawings and the scene read it as an alternative pose map. Trails are lines between the two | `ops/geop-ops-model/placed` (vocabulary: exploded poses), `ops/geop-ops-assembly` (step + drag), `ops/geop-ops-drawing` (view option) | M | — |
| 15 | **Motion: drivers, sweep, collision while moving** | A `Motion` step = time table of joint coordinates (or a driver formula); the editor plays it through `Settle`; interference evaluated per frame with box pruning (#21) gives collision detection and envelope | `ops/geop-ops-assembly` (step), `ops/geop-ops-inspect/interference.rs` | M | 21 |
| 16 | **Drawing views: detail, auxiliary, broken, crop** | A view is a (camera frame, clip region, scale) on the same hidden-line pipeline: detail = circle clip + scale, auxiliary = camera normal to a picked edge/face, broken = several clip slabs shifted together | `ops/geop-ops-drawing/view.rs`, `section.rs` | M | — |
| 17 | **Tolerances, GD&T, ordinate & hole tables** | Annotation vocabulary: a dimension carries a tolerance (±, limits, ISO 286 fit from a table like hole tables); feature-control frames and datum symbols as annotations attached to model elements by stable names (so the same object can later be PMI, #26) | `ops/geop-ops-drawing/annotation.rs`, `ops/geop-ops-model` (if PMI shared) | M | — |
| 18 | **PDF export of sheets** | `geop-lib-sheet` already writes SVG/DXF from one sheet model; add a PDF writer (vector, no deps) | `lib/geop-lib-sheet` | S | — |
| 19 | **STEP assembly import as placed parts** | Each STEP product → a workspace file (`.step` part read through the cache) and the assembly → a generated `.geop` placing them with `Fastened`/fixed poses | `ops/geop-ops-step/import` | M | 3 helps |
| 20 | **Mesh export: 3MF, OBJ, glTF of part and assembly** | One `Exporter` per format over the rasterized `PartView` + instances (glTF instancing maps 1:1 to `ViewInstance`) | `ops/geop-ops-step` (or a new `geop-ops-mesh-export`), `core/geop-core-rasterize` | S | — |
| 21 | **Interference at scale** | Replace O(n²) pair loop by sweep-and-prune over instance boxes; memoise a pair by (component A, component B, relative pose) so 2000 identical screws on one plate are one check; touch via `least_distance` | `ops/geop-ops-inspect/interference.rs` | S | — |
| 22 | **Geometry analysis inspectors** | One inspector family over a face's NURBS: curvature (principal, from derivatives), draft angle vs a pull direction (classify faces positive/negative/vertical by intervals, not samples), wall thickness by `least_distance` to opposite faces, deviation between two bodies; drawn as a per-face colour map | `ops/geop-ops-inspect` | M | — |
| 23 | **Sketch entities: ellipse/conic, text, point-table curve, DXF/SVG import** | Each new `CurveKind` is a rational NURBS with solved parameters (ellipse = centre, axis, ratio; conic = rho); text = glyph outlines (TrueType quadratic Béziers are exact NURBS) as a non-constrained block; DXF/SVG import produces a sketch | `core/geop-core-sketch`, `ops/geop-ops-sketch` | M | — |
| 24 | **Emboss / engrave / wrap** | Project curve already maps a sketch onto a face; wrap = map by surface arc length (developable faces exact, others with a bound); emboss = sweep the wrapped region along the face normal and boolean. One `Wrap` op on `geop-lib-sweep` | `ops/geop-ops-edit` (project), `lib/geop-lib-sweep` | M | 23 (text) |
| 25 | **Thin features & sketched/jog bends & convert to sheet metal** | Thin extrude = offset the open profile (sketch offset exists) then extrude; sheet metal: sketched bend/jog = bend region about a line on an existing flat, convert = pick faces of a shelled solid → base flange + bends | `ops/geop-ops-extrude-revolve`, `ops/geop-ops-sheetmetal` | M | — |
| 26 | **STEP AP242: colours, PMI, layers** | Read/write presentation (colours already a part input) and semantic PMI from #17's annotation vocabulary | `ops/geop-ops-step` | M | 17 |
| 27 | **Fillet & offset kernel gaps (SMLib parity)** | Continue the comparison's open lists: rollover over creases, smooth corners of different radii (ignored reproducer exists), setbacks, face fillet / full round, conic section; shell: free-form edge copies, global self-intersection detected and refused | `ops/geop-ops-fillet`, `lib/geop-lib-shell` | L | — |
| 28 | **Surface evaluation over interval spanning knots** | Unite span parts as curves already do (`find_spans`) — ROADMAP Next; speeds every boolean/fillet stress test | `core/geop-core-geometry` | M | — |
| 29 | **Mirror component & derived part** | Mirror component = place a part with a reflected pose (`ScaleBody` with −1 axis exists; poses are rigid, so a mirrored instance is a *derived part*: a generated program "read file, mirror body") | `ops/geop-ops-assembly/add_part.rs`, `ops/geop-ops-pattern/mirror.rs` | S | 10 helpful |
| 30 | **Display states & per-instance visibility/appearance** | A step-free view setting (like the camera) per program: hidden/isolated instances, colour overrides; sent as part of `ViewInstance` | `core/geop-core-editor/view`, `web/src/placed3d.ts` | S | 5 (same files: sequence) |

Lower priority, noted: IGES import/export (K has it; `geometry.md`),
Parasolid/JT (licensing), G2 surfacing, mold tools, forming tools, real-time
collaboration, PDM states, CAM, FEA-lite (linear static, could reuse the
exact B-rep + a tet mesher), rendering (glTF from #20 is the bridge),
SDF/MJCF export next to URDF.

### 2.1 Work packages (parallel, disjoint files)

Each package owns its files; the shared lists (workspace `Cargo.toml`,
`cad/geop-cad-base/src/registry.rs`, `DEVELOPERS.md` table,
`web/src/icons.tsx`, protocol samples) are merged as AGENTS.md "Working in
parallel worktrees" says.

**WP-A Solver at scale** — items 1, 2, 9.
Owns `core/geop-core-math/src/solvers/**`, `lib/geop-lib-constraints/**`,
`ops/geop-ops-assembly/src/{mechanism,solving,settle}.rs`,
`ops/geop-ops-assembly/src/scale_tests.rs`. Starts with the counting
benchmark of §3 (shared fixture lives here). No feature work.

**WP-B Build, cache, view at scale** — items 3, 4, 5, 30.
Owns `core/geop-core-program/src/{cache,runner,placed}.rs`,
`core/geop-core-rasterize`, `core/geop-core-editor/src/view/**`,
`web/src/placed3d.ts`, `web/src/kernel.worker.ts`, `cad/geop-cad-cli`,
`cad/geop-cad-web` store. Item 4 after 3; 30 after 5.

**WP-C Configurations and top-down** — items 6, 10, 29.
Owns `core/geop-core-program/src/{program,reader,element}.rs` (step
`active`, upward references), `ops/geop-ops-model/src/parameters/**`,
`ops/geop-ops-assembly/src/add_part*.rs`. Coordinates with WP-B on
`runner.rs` (WP-C adds the skip-if-inactive check; agree the seam first,
or land WP-C's runner change before WP-B #4).

**WP-D Mechanisms and motion** — items 7, 8, 14, 15.
Owns `ops/geop-ops-assembly/src/{joints,residuals,mates}.rs` (WP-A does not
touch these; #9 couplings is a few lines in `mates.rs` — give it to WP-D
instead if WP-A is late), new `explode.rs`/`motion.rs` in assembly,
`ops/geop-ops-model/src/{mate_entity,placed}`. Depends on WP-E #21 for
collision during motion.

**WP-E Patterns, fasteners, frames, inspection** — items 11, 12, 13, 21, 22.
Owns `ops/geop-ops-model/src/spacing*`, `ops/geop-ops-pattern/**`,
`ops/geop-ops-assembly/src/part_pattern.rs`, new `ops/geop-ops-frame`,
`ops/geop-ops-inspect/**`. #12 waits on WP-C #6 (configuration row).

**WP-F Drawings and exchange** — items 16, 17, 18, 19, 20, 26.
Owns `ops/geop-ops-drawing/**`, `lib/geop-lib-sheet/**`,
`ops/geop-ops-step/**`. Exploded drawing view consumes WP-D #14 last.

**WP-G Sketch and part features** — items 23, 24, 25.
Owns `core/geop-core-sketch/**`, `ops/geop-ops-sketch/**`,
`ops/geop-ops-extrude-revolve/**`, `ops/geop-ops-sheetmetal/**`,
`ops/geop-ops-edit/src/project.rs` (+ new wrap).

**WP-H Kernel parity (SMLib)** — items 27, 28.
Owns `ops/geop-ops-fillet/**`, `lib/geop-lib-shell/**`,
`core/geop-core-geometry/**` (evaluation). Runs the ignored stress sweeps
before merging (AGENTS.md).

Dependency graph across packages: WP-B#3 → WP-B#4, WP-C#10, WP-F#19;
WP-C#6 → WP-E#12; WP-E#21 → WP-D#15; WP-F#17 → WP-F#26; WP-G#23 → WP-G#24;
WP-D#14 → (drawing exploded view in WP-F, last). Everything else is
independent.

---

## 3. Large-assembly performance

### 3.1 What exists

- `ops/geop-ops-assembly/src/scale_tests.rs`: the "robot" workspace (plate,
  screws, standoffs, boards of modules, `robot(boards, screws)`; 44 parts
  per board) with counting tests — `removing_a_screw_sends_no_other_part_again`,
  `moving_a_part_builds_again_only_what_reads_where_it_is`,
  `an_edit_resolves_each_mate_a_few_times` (counts `mates_resolved()` and
  `geop_lib_constraints::solve::runs()`),
  `dragging_one_of_many_placed_parts_sends_only_it` — and an ignored
  `robot_timings` (500 and 2000 parts: load, leaf edit, unchanged save,
  top-level edit, hover, drag, update sizes). ROADMAP: a drag in 2000 parts
  ≈ 0.25 s.
- Groups of blocks no term ties are solved apart and in lockstep
  (`System::groups`, `solvers/system/lockstep.rs`), so runs do not grow
  with the number of groups.
- Equal builds are one (`placed.rs`: "every screw of a size"), components
  drawn once and instanced (`ViewInstance.view: Rc<PartView>`,
  `web/src/placed3d.ts` `InstancedMesh` per component), the scene sends
  changes only.

### 3.2 Known bottlenecks from the code

1. **One dense system per connected group.** `System::groups`
   (`core/geop-core-math/src/solvers/system/mod.rs:276`) unions blocks
   through any term. In the test robot every screw mates to the fixed
   plate, so groups are one screw each. In a real robot fasteners mate to
   moving links, brackets are fastened to links, links are jointed: the
   whole arm is **one group** of 6·k pose unknowns, solved with dense
   `Vec<Vec<Scalar>>` interval Householder (`least_squares.rs`) and a dense
   Krawczyk enclosure (`system/enclose.rs`) — O((6k)³) per step, k in the
   hundreds. Fastened parts carry no motion, so rigid-cluster reduction
   (#1) collapses that to 6 × (number of moving links). This is the first
   thing a real robot will hit; the current benchmark does not exercise it.
   *Done (WP-A):* the `robot_arm` benchmark measured it — a drag of its
   101-part form ran for over 50 minutes. Blocks that only hang off others
   (fastened brackets, mated screws, motors, pulleys) are now pendants,
   solved and enclosed layer by layer after what they hang off
   (`System::structure`): the arm's largest group is its links and joints,
   42 variables, at 101, 281, 1013 and 1973 parts alike.
2. **Every evaluation reruns the program.** `Run::at`
   (`lib/geop-lib-constraints/src/solve.rs`) clones the state and runs the
   whole program (memoised, so cost ∝ steps, not geometry); per solve
   iteration that is O(placed parts) even when one group moves. Fine at
   2000 with few iterations; grows with iteration count in big groups.
   Counted already by `solve::runs()`; add a count of *steps visited*.
3. **No persistent build of `.geop` parts.** `Cache`
   (`core/geop-core-program/src/cache.rs`) persists only what a file's
   *text was made into* for imports (`.step.geopc`). Opening a workspace
   rebuilds every distinct component program (booleans, fillets, threads)
   every session and in every process (CLI, web worker, VS Code host).
4. **Single-threaded kernel.** `Rc`/`RefCell` throughout `core-program`;
   distinct components are independent but built serially; rayon is only a
   dev-dependency of `geop-lib-booleans` tests. In the browser the kernel
   is one wasm worker (32-bit address space: 4 GB ceiling for all built
   models and views).
5. **Full-resolution views only.** Each component has one `PartView`
   (rasterised at one tolerance); a 2000-part scene of detailed std parts
   (threads, gears with 120 teeth) ships all triangles. ROADMAP Next notes
   a committed STEP import resends its whole mesh (114 MB for 886 faces).
   No LOD, no lightweight/box mode.
6. **Picking is linear in triangles per candidate instance.**
   `PartView::layers` culls by each instance's bounding sphere (`near`),
   then `pick_face` tests every triangle of the component; no BVH. Hover
   on a dense component cluster costs the sum of their triangles.
7. **Interference is O(n²) pair loop with a box test**
   (`ops/geop-ops-inspect/src/interference.rs:166`), then a boolean per
   overlapping pair with no memo by (component pair, relative pose): 2000
   screws in 2000 tapped holes are 2000 booleans of the same pair.
8. **`measure()` of the scene** walks every vertex/edge/face point plus 8
   corners per instance on each redraw of the top view — linear, fine, but
   recomputed per update.
9. **Groups found per solve by evaluating every term**
   (`System::groups` evaluates all terms at the start), O(mates) per drag
   step — fine now, but means a drag cannot be cheaper than the whole
   mate set; caching the group partition between drag steps (it changes
   only when mates change) removes it. *Measured (WP-A):* the structure
   takes two of a drag's ~96 program runs — one at the point, which the
   memoised run answers cheaply, and one over a box — so it is not cached
   yet. What a drag of the 1973-part arm costs instead: ~96 program runs
   (a run re-makes every moved part's constraints, and walks every step),
   and the editor rebuilding the forms of every placed standard part's
   sketches for the step list on every event (about a third of it).

### 3.3 What to measure

| quantity | how (counts, not seconds — AGENTS.md "Count the work") |
|---|---|
| parts placed / distinct components / mates / joints | from the built part (`placed_parts`, `targets_of::<Resolved>`) |
| largest group: blocks and rows | new counter in `System::groups` → `largest_group()` |
| solve work per drag step | `solve::runs()`, new `steps_visited()` in the runner, flops proxy = Σ (rows × cols²) per factorisation |
| builds per edit / per load / per reopen | `Cache::made()` and a new "components built" counter in `placed.rs` |
| scene bytes | `size(&update)` (exists) split into components / instances / triangles |
| triangles tested per hover | counter in `pick_face` |
| interference pair checks / booleans | counters in `interference.rs` |
| peak memory | `prlimit --as` cap as a pass/fail bound in the ignored timing test; resident size of the built `Part` via a `size_of`-style walk of `PartView` triangle counts |
| load / save round trip | `assert_round_trips` on the generated workspace; text size of the files |

### 3.4 Proposed benchmark: `robot_arm` (2000 parts, mechanism-shaped)

Add to `ops/geop-ops-assembly/src/scale_tests.rs` (WP-A owns it) a second
generator next to `robot()`:

- `robot_arm(axes = 6, brackets_per_link = 4, screws_per_bracket = 40)`:
  a fixed base; `axes` links in a chain, each `Revolute` to the previous
  with limits; on each link `brackets_per_link` brackets *fastened* to the
  link (`Fastened`), each with `screws_per_bracket` screws mated
  (coincident + concentric) to the bracket — not to the base; a gripper
  sub-assembly placed flexible at the end; a `std:` NEMA 17 + GT2 pulley
  pair per axis with a belt coupling (#9). With the defaults:
  6 × 4 × 40 = 960 screws + brackets, links, motors, pulleys, std
  bearings ≈ 1100 parts; `screws_per_bracket = 80` gives ≈ 2050.
- Plus the existing `robot(boards = 40, screws = 240)` (≈ 2000 parts,
  trivially grouped) as the "flat" case.

Counting tests (fast, default run, at two sizes each so growth shows —
"a quadratic shows at 80 parts as clearly as at 2000"):

1. `a_fastened_cluster_solves_as_one_body` — after #1, the largest group
   of `robot_arm(6, 2, 5)` has ≤ 6 × (axes + 1) unknowns, independent of
   screws.
2. `drag_work_grows_with_the_moving_links_not_the_screws` —
   `solve::runs()` and the factorisation work counter for a drag of link 3
   are equal (runs) / bounded (work) for `screws_per_bracket` 5 vs 20.
3. `reopening_builds_nothing` — build the workspace with a `CacheStore`
   (in-memory shelf, as `cache.rs` tests do), start a fresh `Cache` over
   the same store, rebuild: components built = 0 (after #3).
4. `a_hover_tests_triangles_of_the_parts_under_the_pointer` — triangles
   tested per hover bounded by a constant × triangles of hit components
   (after #5's BVH).
5. `interference_checks_each_component_pair_pose_once` — screws in
   identical holes: booleans run ≤ distinct (component, component,
   relative pose) triples (after #21).
6. `scene_of_small_parts_is_sent_coarse` — update bytes for the 2000-part
   arm below a bound per instance when the camera frames the whole (after
   #5).
7. Keep `an_edit_resolves_each_mate_a_few_times` and extend it to the arm.

Ignored timing test `robot_arm_timings` (like `robot_timings`): load cold,
load warm (cache), leaf edit (a screw's length), top-level edit, hover,
drag of the wrist, joint value set, interference report, URDF export,
memory under `timeout 600 prlimit --as=16000000000`. Run old/new binaries
interleaved, CPU time, as AGENTS.md asks; profile by interrupting under
`gdb -batch` with `SIGUSR2`, never `perf`.

Targets to set once the counters exist (indicative, from vendors' "large
assembly" positioning): drag ≤ 50 ms for a 2000-part arm (interactive),
warm load ≤ 2 s, cold load bounded by distinct components (not instances),
scene ≤ ~30 MB for 2000 parts at full-frame, kernel heap ≤ 2 GB (wasm
headroom).
