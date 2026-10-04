# Roadmap: what geop still needs to design a robot

The yardstick is SolidWorks, Onshape, Fusion 360, Rhino, Creo and NX, but only
the features an engineer reaches for when designing a real machine such as a
robot: its structure, drive train, enclosures, wiring and fasteners. Breadth
for its own sake (CAM, CAE, rendering, PDM) is out of scope.

## What geop has (October 2026)

- 2-D constraint sketches: lines, arcs, circles, splines; 20 constraint kinds,
  projections of the part, trim.
- Extrude and revolve (blind, up to next, through all, symmetric, two sides),
  sweep along a sketch path, loft (with matching points).
- Booleans (union, intersection, difference) and split by a face.
- Fillet and chamfer on straight and circular edges, mitred at corners.
- Shell, delete body, extract face, project a curve onto a face.
- Datums: points, axes, planes, coordinate systems.
- Parameters and formulas, multi-file workspaces, placed parts with mates
  (coincident, concentric, parallel, perpendicular, distance, angle), drag.
- STL export; web editor (wasm) and VS Code extension (native `geop serve`).

## Missing, by area

Each item names the workstream that delivers it (see below). Items marked
*later* are deliberately left out of this round.

### Sketching and reference geometry
- 3-D sketches: points, lines and interpolating splines in space, snapping to
  model vertices and datums, usable as sweep paths and guide rails.
  — `sketch3d`
- Helix and spiral curves (pitch, turns, taper) as paths. — `hole-thread`
- Sketch patterns and offset curves. — *later*

### Part features
- Hole feature: simple, counterbore, countersink, tapped, clearance sizes from
  ISO tables, placed at sketch points on a face. — `hole-thread`
- Threads: cosmetic (recorded, drawn) and modelled helical threads. —
  `hole-thread`
- Linear and circular patterns, mirror, move/copy body. — `pattern`
- Rib, lip/groove (enclosure joints), draft on planar faces. — `plastic`
- Fillets on arbitrary (free-form) edges, variable radius. — `fillet-general`
- Sheet metal: base and edge flanges, bends with K-factor, flat pattern. —
  `sheetmetal`

### Sweeps and surfacing
- Sweep with guide rails (profile scaled and oriented by rails), twist and
  orientation control; loft with guide curves. — `sweep-guides`
- Surfaces: boundary (Coons) and N-sided fill, offset surface, thicken, knit
  faces into a solid, trim and extend. — `surfacing`
- Subdivision surfaces: Catmull–Clark cage editing (extrude face, crease),
  converted to a NURBS solid. — `subd`

### Assemblies
- Mates for mechanisms: revolute and slider with limits, gear, rack and
  pinion, screw. Patterns of placed parts. — `assembly`
- Smarter caching: invalidate only the files a change reaches, share meshes
  between instances, build independent parts in parallel; measured on an
  assembly of hundreds of instances. — `caching`
- Standard parts: ISO metric screws, nuts, washers, bearings, dowel pins,
  standoffs, 20-series aluminium extrusions; placed like any other file. —
  `stdlib`
- Wire harness: routes through clips and connectors in an assembly, bend
  radius checked, bundle diameter swept, cut lengths reported. — `harness`

### Inspection and output
- Measure (distance, angle), mass properties (volume, area, centre of mass,
  inertia), interference check, section view. — `inspect`
- STEP AP214/AP242 import and export of B-rep solids, validated against a
  downloaded public corpus. — `step`
- Drawings: projected views with hidden lines, dimensions, export to SVG and
  DXF. — `drawings`
- URDF export of an assembly for robot simulation: links from rigidly
  held parts, revolute/continuous/prismatic joints, mimics from couplings,
  inertia, meshes; closed loops refused. — `urdf`
- Configurations, design tables, BOM tables, PDM, rendering, CAM, FEA. —
  *later* or out of scope

## Testing policy for new features

Every feature gets a fast test that it still works, run by `cargo test`, and,
where depth is needed, validation tests marked
``#[ignore = "slow: … — run with `cargo test -- --ignored`"]``. Anything a user
reaches through the editor also gets a test in `editor_tests.rs` that drives
it the way the front end does.
