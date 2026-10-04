
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
core/geop-core-geometry       NURBS curves/surfaces, containment, intersection
core/geop-core-topology       B-rep structures, Euler operators, edit/validation
core/geop-core-solve          the constraint solver every system shares:
                               parameters, residuals, pulls, enclosure;
                               rigid bodies and mates
core/geop-core-sketch         2-D constraint sketches: entities, constraints
                               as residuals, profile extraction
ops/geop-ops                  parts (topology, sketches, datums, placed parts
                               and their mates, every entity with a stable
                               name), what an operation is and how it is
                               edited (events, dialogs, visuals, hit tests),
                               parameters and the formulas reading them,
                               programs, the files they place parts from, and
                               running them (+ `geop-ops-derive`)
ops/geop-ops-sketch           the sketch operation: drawing and constraint
                               tools, snapping, reference geometry and
                               projections of the part
ops/geop-ops-datums           the datum operation: reference points, axes,
                               planes and coordinate systems
ops/geop-ops-booleans         3-D boolean operations (union/intersection/diff),
                               the boolean operation
ops/geop-ops-edit             edits of existing bodies: delete a body,
                               extract a face, project a sketch onto a face
ops/geop-ops-extrude-revolve  extrude/revolve, the extrude and revolve
                               operations, and basic shapes for tests
ops/geop-ops-fillet           fillets and chamfers on straight and circular
                               edges, cut or filled in with a boolean
ops/geop-ops-assembly         the part operation: place another file's part,
                               mate it, drag it
ops/geop-ops-rasterize        turns a Model into a triangle mesh, writes it
                               as STL, and renders it for debugging
cad/geop-cad-base             the operations the editor offers, the editor engine,
                               example programs
cad/geop-cad-web              the wasm bindings the web app loads (crate `geop`)
cad/geop-cad-cli              the `geop` command-line tool
```