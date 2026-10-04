
Things I tried that didn't work:
- Geop 1: Using lines, arcs, spheres, tori, spirals etc. as primitives with explicit intersection formulas. Didn't scale as equations became too complex. Interval arithmetic was nice though and 2d booleans worked reliably.
- Geop 2: Went deep into algebra, symbolic math, ideals, Groebner bases, etc. to solve the intersection problems. Too slow and too much overhead. Decided to use only numerical methods from now on.
- Geop 3: Focused on getting the numerical methods right, first POCs for intersection of NURBS and nonlinear equation solving. Finally found something that could do the job. Decided to use dynamic ndarrays to represent multi-dimensional Bernstein polynomials from now on.
- Geop 4: Got stuck in numerical solvers and topology. Booleans looked promising but tracing was too difficult, too many edge cases.
- Geop 5: Based only on subdivion again, but instead of using convex hulls, approximate short segments with fat lines and fat surfaces. Interface for curve only provides subdivision, midpoint, tangent, and fat line approx. Interface for surfaces only provides subdivision, midpoint, normal, and fat surface approx. Intersection curves are never traced explicitly, they are just stored as a set of surface patches that can be subdivided further if needed. Intersections are trivial like this. Containment is done by ray shooting. We do not yet store any topology information, just a bag of surface bounded surface patches. The surface patches are also bounded by 3d curves, not in their parametric space. 

### Development

The kernel is a Cargo workspace of small crates, each depending only on the
ones before it; `web/` is a React front end that loads the top of that chain
compiled to WebAssembly and renders its output with `three.js`, and
`geop-cad-cli` puts the same chain on the command line.
`cloudflare-functions/` holds the Cloudflare Pages Functions deployed with
the web app (bug reports).

```
core/geop-core-math          scalars, interval arithmetic, linear algebra,
                              convex hulls, error type, frames, rays, datums,
                              poses (dual quaternions), constrained least
                              squares and dual numbers for the solvers
core/geop-core-geometry       NURBS curves/surfaces, helices, compatible
                               curves for lofts, containment, intersection
core/geop-core-topology       B-rep structures, Euler operators, edit/validation
core/geop-core-solve          the constraint solver every system shares:
                               parameters, residuals, pulls, enclosure;
                               rigid bodies and mates, joints with limits
                               and the couplings between them, solved in
                               groups no mate ties together
core/geop-core-sketch         2-D constraint sketches: entities, constraints
                               as residuals, profile extraction
ops/geop-ops                  parts (topology, sketches, datums, placed parts
                               and their mates, every entity with a stable
                               name), what an operation is and how it is
                               edited (events, dialogs, visuals, hit tests),
                               parameters and the formulas reading them,
                               programs, the files they place parts from, and
                               running them — a workspace rebuilds only what
                               a changed file reaches, and a scene sends
                               placed parts as changes (+ `geop-ops-derive`)
ops/geop-ops-sketch           the sketch operation: drawing and constraint
                               tools, snapping, reference geometry and
                               projections of the part
ops/geop-ops-datums           the datum operation: reference points, axes,
                               planes and coordinate systems
ops/geop-ops-booleans         3-D boolean operations (union/intersection/diff),
                               the boolean operation
ops/geop-ops-edit             edits of existing bodies: delete a body,
                               extract a face, project a sketch onto a face
ops/geop-ops-extrude-revolve  extrude/revolve; sweeps along paths, with guide
                               rails, twist and orientation; lofts, with
                               guide curves and matched points; their
                               operations, and basic shapes for tests
ops/geop-ops-rasterize        turns a Model into a triangle mesh, writes it
                               as STL, and renders it for debugging
ops/geop-ops-sketch3d         the 3-D sketch operation: points, lines, arcs
                               and splines in space, placed on the part and
                               constrained — paths and rails for sweeps
ops/geop-ops-fillet           fillets and chamfers on straight and circular
                               edges, cut or filled in with a boolean
ops/geop-ops-shell            shelling: a solid hollowed to walls of one
                               thickness, open at picked faces, the shell
                               operation
ops/geop-ops-pattern          linear and circular patterns, mirrors and
                               moves of bodies, copied as new bodies or
                               combined with a solid; their operations
ops/geop-ops-hole             holes from ISO tables (simple, counterbore,
                               countersink, tapped) and threads, cosmetic or
                               modelled along a helix; the hole and thread
                               operations
ops/geop-ops-plastic          housings: ribs grown up to the walls, lips and
                               grooves along a rim, drafts on planar faces
ops/geop-ops-surface          surfacing: boundary (ruled, Coons, filled)
                               surfaces, offset, thicken, knit, trim and
                               extend of faces standing on their own
ops/geop-ops-subd             subdivision surfaces: a control cage shaped in
                               the editor, built as its Catmull-Clark limit
                               surface, a solid of B-spline faces; the subd
                               operation
ops/geop-ops-sheetmetal       sheet metal: base and edge flanges with bends
                               and reliefs, the flat pattern; the sheet model
                               recorded on the body and thickened into it
ops/geop-ops-harness          wire harnesses: wires routed through connectors
                               and clips as lines and arcs, bend radius
                               checked, the bundle swept, cut lengths; the
                               route operation
ops/geop-ops-assembly         the part operation: place another file's part,
                               mate it, joint it, drag it; patterns of
                               placed parts
ops/geop-ops-inspect          inspecting a part without changing it: mass
                               properties of its solids and placed parts,
                               measurements of picked entities, interference
cad/geop-cad-base             the operations the editor offers, the editor engine,
                               example programs, the standard parts (below)
cad/geop-cad-web              the wasm bindings the web app loads (crate `geop`)
cad/geop-cad-cli              the `geop` command-line tool; `compile` meshes
                               each placed component once
```

The web app draws placed parts batched per component — one
`InstancedMesh` each (`web/src/placed3d.ts`) — and takes the scene's
changes rather than the whole scene. Its section view (`section.ts`)
clips the part and the placed parts and caps what it cuts open from
stencil counters drawn with them.

Standard parts (ISO screws, nuts, washers, dowel pins, standoffs, ball
bearings, T-slot extrusions, a NEMA 17 motor) live in
`geop-cad-base/src/stdlib`: one program per family, generated in Rust,
its sizes the rows of a table parameter `size`. Every workspace reads them
as read-only files named `std:…` (`std:iso4032_hex_nut.geop`) through
`stdlib::WithStandardParts`, the one place they come from; the editor's
part picker lists them, and `geop compile std:…` meshes one.