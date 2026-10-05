# Roadmap: what geop needs to design a robot

The yardstick is SolidWorks, Onshape, Fusion 360, Rhino, Creo and NX, but only
the features an engineer reaches for when designing a real machine such as a
robot: its structure, drive train, enclosures, wiring and fasteners. Breadth
for its own sake (CAM, CAE, rendering, PDM) is out of scope.

In October 2026 geop had 2-D sketches, extrude/revolve/sweep/loft, booleans,
fillets and chamfers on straight and circular edges, shell, datums,
parameters, multi-file assemblies with simple mates, and STL export. The list
below is what was missing then, and where each item stands now.

## Done

### Sketching and reference geometry
- 2-D sketches: slots, arc slots, polygons, sketch fillets and chamfers;
  offset, mirror, linear and circular patterns tied to their source by
  constraints; auto-tangent; redundant but consistent constraints proven,
  conflicting ones named.
- 3-D sketches: points, lines, 3-point arcs, splines; snapping to the model;
  usable as sweep paths and rails.
- Helices (exact on their cylinder) as thread and sweep paths.

### Part features
- Holes: simple, counterbore, countersink, tapped; ISO 273/261/2306 tables;
  blind with drill point, up to next, through all.
- Threads: cosmetic (drawn in the view and in drawings) and modelled.
- Linear and circular patterns, mirror, move/copy body.
- Rib, lip and groove, draft on planar faces.
- Fillets on any edge (rolling ball), tangent chains, variable radius,
  rounded corners where three fillets meet; chamfers on any edge.
- Sheet metal: base and edge flanges, hems, open and closed corners, cuts
  after flanging (across bends), exact flat pattern, DXF for laser cutting.
- Operation dimensions as formulas of parameters; parameters renamed
  everywhere they are read.

### Sweeps and surfacing
- Sweeps with one or two guide rails, twist, end scale, fixed or following
  orientation; lofts with up to three guide curves.
- Ruled, boundary (Coons, tangent to adjacent planes) and N-sided fill
  surfaces, offset surface, thicken, knit into solids, trim, extend.
- Subdivision surfaces: Catmull–Clark cage editing with creases and mirror,
  converted to a valid NURBS solid.

### Assemblies
- Joints: revolute, slider, cylindrical, fastened, with limits; gear, rack
  and pinion and screw couplings; degrees of freedom and conflicting mates
  reported; part patterns.
- Caching for large assemblies: incremental workspace, instanced drawing,
  independent groups solved apart (a drag in 2000 parts takes about 0.25 s).
- Standard parts: ISO screws, nuts, washers, dowel pins, standoffs, bearings,
  T-slot extrusions, NEMA 17 steppers, involute spur gears and racks, GT2
  pulleys, shaft collars, flange couplings, MGN rails and carriages, hobby
  servos (`std:` files), with datums for mating and joints, designations and
  materials.
- Wire harness routes: connectors and clips, bend radius checked, cut lengths.

### Inspection and output
- Measure, mass properties from the exact B-rep, interference, section view.
- Bills of materials, flat or indented, CSV, `geop bom`.
- Drawings: projected views with hidden lines and silhouettes, sections,
  dimensions, threads, title block; assembly drawings with balloons and a
  BOM table; SVG and DXF.
- STEP AP214 import and export; assemblies exported as product structure;
  318 of 361 public corpus files import valid (`scripts/fetch_corpus.sh`).
- URDF export of jointed assemblies for robot simulators.
- STL download in the web app.

### Editor and front ends
- Web app and VS Code extension: every operation, inspect and BOM panels,
  joint values, every export; camera framing; a scrolling toolbar; the
  kernel restarted with the program after a crash.
- A 3-D transform gizmo for every drag in space: arrows, plane squares and a
  free ball to move, rings to turn, cubes to scale along an axis or evenly;
  world or local axes, constant size on screen, snapping to round grid
  steps and 15° (shift for none), the distance or angle shown while
  dragging. SubD selections, 3-D sketch points, move body, offset datum
  points and placed parts are dragged by it.
- End-to-end checks: `npm run e2e` and `npm run e2e:vscode` in `web/`.

## Next

- Booleans and geometry: surface evaluation over an interval parameter
  still uses one knot span (curves now unite all spans), the main remaining
  cost in the slow stress tests; an arc split many times grows wide (so
  gears stop at 80 teeth); a
  fixed-point cylinder far from the origin exhausts its search budget.
- Fillets: rolling over a crease, mitres for rolled fillets, corners where
  fillets arrive with different radii.
- STEP: rebuild edges that disagree with their faces from the kernel's own
  intersections (most of the remaining corpus failures); assembly import as
  placed parts.
- Drawings: exploded views; parts that pass through each other.
- Assemblies: actuator effort and velocity for URDF; collision checks for
  harness routes.
- Configurations, design tables beyond table parameters, PDM, rendering, CAM,
  FEA: later, or out of scope.

## Testing policy

Every feature gets a fast test that it still works, run by `cargo test`, and,
where depth is needed, validation tests marked
``#[ignore = "slow: … — run with `cargo test -- --ignored`"]``. Anything a user
reaches through the editor also gets a test in `editor_tests.rs` that drives
it the way the front end does, and the flows that matter most are checked
end to end in a browser by `npm run e2e`.
