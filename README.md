# Geop CAD-Kernel

This repository is the beginning of an open-source CAD revolution. I am tired of paying insane amounts of money for CAD software that looks like it was designed in the 90s, and I want to build a modern CAD kernel that is open-source, easy to understand, and can be used as a foundation for research and development in geometric modeling.

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

```
core/geop-core-math          scalars, interval arithmetic, linear algebra,
                              convex hulls, error type, render primitives
core/geop-core-geometry       NURBS curves/surfaces, containment, intersection
core/geop-core-topology       B-rep structures, Euler operators, edit/validation
core/geop-core-sketch         2-D constraint sketches: entities, constraints,
                               BFGS solver, profile extraction
core/geop-core-part           a part: topology and sketches, every entity
                               with a stable name
ops/geop-ops-extrude-revolve  extrude/revolve and the sample shapes (cube,
                               sphere, cylinder, torus, ...)
ops/geop-ops-booleans         3-D boolean operations (union/intersection/diff)
ops/geop-ops-rasterize        turns a Model into a triangle mesh, and writes
                               it as STL
ops/geop-ops-parts            programs: the operations a part is built with,
                               and running them (+ `geop-ops-parts-derive`)
cad/geop-cad-base             ray-based picking
cad/geop-cad-web              the wasm bindings the web app loads (crate `geop`)
cad/geop-cad-cli              the `geop` command-line tool
```

**Kernel (Rust):**
```sh
cargo test        # run the whole workspace's test suite
cargo build        # native build, e.g. for the examples under each crate's examples/
```

**Web app:**

Requires `wasm-pack` (`cargo install wasm-pack`) and the
`wasm32-unknown-unknown` target (`rustup target add wasm32-unknown-unknown`),
plus Node 20+.

```sh
cd web
npm install         # once
npm run dev          # rebuilds the wasm bindings, then starts the Vite dev server
```

Then open the printed `localhost` URL. `npm run dev` (and `npm run build`)
automatically rebuild the wasm package first via `npm run build:wasm`, so
Rust changes anywhere in the workspace show up on the next dev-server
reload/restart. See `web/README.md` for more on how the wasm bridge
(`cad/geop-cad-web/src/lib.rs`) is wired up.

### Sketches

`core/geop-core-sketch` is a 2-D constraint sketcher: points, lines, arcs
(start, end and curvature), circles and splines, the usual CAD constraints
(coincident, horizontal/vertical, parallel, perpendicular, collinear,
tangent, equal, concentric, midpoint, symmetric, point-on-curve, fix,
distance, length, radius, angle), and a BFGS solver over the sum of squared
residuals. Coincidence is not a residual: coincident points share one pair
of variables, so connectivity is structural rather than a matter of
tolerance. The solved sketch is turned into closed regions (an outer loop
and its holes) and those into NURBS curves that `extrude` and `revolve`
consume, so both now take curved profiles, not just polygons.

In the web app, "Sketch" opens the 2-D editor (line, rectangle, arc, circle,
spline) on a base plane, a picked planar face or a reference plane — the camera turns to face
that plane, the editor is drawn over the 3-D view and pans and zooms it, and
leaving the sketch reverses the move; "Extrude" and "Revolve" then build a solid from it, either as
a new body or joined/cut/intersected with an existing one in the same step.
Revolving needs the profile to touch its axis along an edge constrained onto
that axis.

### Rendering

Tessellation is driven by curvature, not by a fixed sample count: an edge or
a face that is straight/flat gets the coarsest approximation (a line is two
points, a planar face four cells), and a curved one is refined — each
parametric direction on its own, so a cylinder wall is refined around its
circumference and not up its axis — until it is within a tolerance derived
from the face's own size. The mesh also carries the kernel's *surface*
normal at every triangle corner, so a curved face shades as the surface it
approximates instead of as the facets it was cut into.

### Programs

A part is described by a *program*: an ordered list of steps, each an
operation (`add_sketch`, `extrude`, `revolve`, `boolean`, `add_datum`) with its arguments.
Steps refer to what earlier steps made only by stable names — `extrude(box)`
is the solid the step `box` extruded, `extrude(box,end)` its end cap — never
by a kernel id, so a program is self-contained and rebuilds the same part,
name for name, every time. It is stored as JSON, one step per object and
every sketch entity keyed by its id, so edits show up as small diffs:

```json
{
  "steps": [
    { "id": "outline", "operation": "add_sketch",
      "args": { "plane": { "type": "Plane", "normal": "Z" }, "sketch": { "points": { ... }, "curves": { ... }, "constraints": { ... } } } },
    { "id": "box", "operation": "extrude",
      "args": { "sketch": "outline", "distance": 1.0, "symmetric": false,
                "combine": { "mode": "new_body" } } },
    { "id": "hole_sketch", "operation": "add_sketch",
      "args": { "plane": { "type": "Face", "name": "extrude(box,end)" }, "sketch": { ... } } },
    { "id": "hole", "operation": "extrude",
      "args": { "sketch": "hole_sketch", "distance": -0.5, "symmetric": false,
                "combine": { "mode": "difference", "target": "extrude(box)" } } }
  ]
}
```

Whatever a step builds on directly — a sketch's plane, what a reference is
built from — is an *entity reference*: `{"type": "Origin"}`, `{"type":
"Axis", "axis": "X"}`, a base plane `{"type": "Plane", "normal": "Z"}`, or a
`Vertex`, `Edge`, `Face` or `Datum` of the part `{"type": "Face", "name":
"..."}`.

"Save" in the web app downloads the program as `part.program.json`, and
"Load" reads one back. `outputs/parts/` has the examples of
`ops/geop-ops-parts/src/examples.rs` as program files.

### Reference geometry

"Reference" (`add_datum`) adds a datum — a reference point, axis or plane,
named after its step — built from entities picked in the viewport: vertices,
edges, faces, other datums, and the origin, axes and base planes of the
origin gizmo. What a selection can be built into depends on the *shape* of
what is selected, which the kernel recognizes from the NURBS
(`core/geop-core-geometry/src/shape.rs`: `as_line`, `as_arc`, `as_plane`,
`axis_of_revolution`): a straight edge is a line, a circular one has a center
and an axis, a flat face is a plane, a cylindrical, conical or spherical face
has an axis. The form lists every construction and greys out those that
don't fit the selection:

| builds | constructions |
|---|---|
| point | offset from a point (x/y/z, along its own axes for a datum or the origin), midpoint of two points, point on an edge (0–1), center of an arc, projection onto a plane or a line, where a line meets a plane, where two lines meet, where three planes meet |
| axis | through two points, along a line, axis of an arc or round face, where two planes meet, perpendicular from a point onto a plane, parallel to a line through a point, perpendicular from a point onto a line, angle bisector of two lines, tangent to an edge |
| plane | offset plane (distance), midplane of two planes (halfway, or halving their angle), through three points, at an angle through a line, through a line and a point, through two lines, parallel through a point, normal to a line through a point, normal to an edge |

```json
{ "id": "lifted", "operation": "add_datum",
  "args": { "selection": [{ "type": "Face", "name": "extrude(box,end)" }],
            "construction": { "method": "offset", "distance": 0.5 } } }
```

A sketch can then be placed on it with `{"type": "Datum", "name": "lifted"}`
(see the `boss_on_reference_plane` example). A datum is built once, when its
step runs, and the viewer hides one — like a sketch — once a later step has
used it.

### Command line

`geop compile` builds a program and writes the part it makes as an STL mesh
— the same mesh the web app draws:

```sh
cargo run --release -p geop-cad-cli -- compile outputs/parts/box_with_drill_hole.program.json
# 4 steps, 1 solid, 748 triangles -> outputs/parts/box_with_drill_hole.stl
```

or install it once and call it directly:

```sh
cargo install --path cad/geop-cad-cli
geop compile part.program.json                  # writes part.stl (binary)
geop compile part.program.json -o out/part.stl  # choose where
geop compile part.program.json --ascii          # plain-text STL
geop compile part.program.json -s "extrude(hole)" -s "revolve(shaft)"
                                                # only these solids
geop compile part.program.json -q 64            # smoother curved faces
```

| Option | |
|---|---|
| `-o, --output <FILE>` | Where to write the mesh. Defaults to the program's path with `.program.json` (or `.json`) replaced by `.stl`. |
| `-s, --solid <NAME>` | Write only this solid, by name; repeat for several. Every solid of the part by default. An unknown name is an error that lists the part's solids. |
| `--ascii` | ASCII STL instead of binary. |
| `-q, --quality <N>` | How finely curved faces are meshed (at least 2, default 24, what the web app uses). Flat faces are meshed exactly whatever it is. |

If a step fails, nothing is written: the error names the step and why, and
the command exits with status 1. Facets are wound and their normals point
out of the solid. The mesh is not yet watertight: each face is meshed on
its own, so two faces sharing an edge each sample it separately — their
triangles meet at T-junctions rather than shared vertices, and along a
curved edge may leave hairline gaps. Fine for viewing and for most
slicers, not for tools that require a closed manifold.

### Todos
- Fix the rasterization engine to render faces properly.
- Revolve a region that does not touch the axis (a torus: needs a handle in
  the topology, `mekr`/`kemr`, not just `mef`/`mer`).

- Go over all methods again and ensure tightest numerical bounds possible. Especially for the intersection methods where we switch from global solvers to local solvers.
- Implement tracing for face x face curves that don't intersect any existing edge (e.g. two sphere just touching in a sliver).

