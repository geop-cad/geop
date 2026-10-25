# Introduction

Geop is a Cargo workspace in four tiers. `core/` defines what things are:
math, geometry and topology; a program, the part it builds, and how it is
edited. `lib/` holds the algorithms two operations share, taking geometry
and models. `ops/` has one crate per family of operations, each with its
targets, forms, examples and tests. `cad/` composes them — one `register`
line per crate — and carries them to the browser and the command line.
`web/` is a React front end that loads the composition compiled to
WebAssembly and renders its output with three.js.

This book gives an overview of each crate: what it is for, its main types
and operations, and the design decisions behind them. It does not document
every API in depth — see [docs.geop-cad.dev](https://docs.geop-cad.dev) for
the generated rustdoc reference, and expect these chapters to grow over
time.

```text
core/geop-core-math          scalars, intervals, linear algebra, errors,
                              frames, poses, solvers
core/geop-core-geometry      NURBS curves and surfaces, containment,
                              intersection
core/geop-core-topology      the B-rep, building, editing, validation
core/geop-core-rasterize     a model as triangles, STL
core/geop-core-sketch        2-D constraint sketches
core/geop-core-program       a program, its values, the part it builds as a
                              memo table of nodes, placed parts, the runner
core/geop-core-editor        what an operation is and how it is edited, what
                              an editor draws, exporters and inspectors, the
                              editor over a registry
lib/geop-lib-sheet           sheets of paper (SVG, DXF)
lib/geop-lib-booleans        booleans, split, imprint, a step's `Combine`
lib/geop-lib-sweep           extrude, revolve, sweeps, lofts, skinning
lib/geop-lib-shell           shelling, thickening, offsetting
lib/geop-lib-stdlib          the standard parts beside every workspace
ops/geop-ops-model           the model target: topology with stable names
lib/geop-lib-constraints     the constraints of a part's state, solved
ops/geop-ops-sketch          the sketch operation
ops/geop-ops-sketch3d        the 3-D sketch operation
ops/geop-ops-reference-geometry  the datum operation
ops/geop-ops-extrude-revolve extrude, revolve, sweep and loft
ops/geop-ops-hole            holes and threads
ops/geop-ops-fillet          fillets and chamfers
ops/geop-ops-shell           the shell operation
ops/geop-ops-plastic         ribs, lips and grooves, drafts
ops/geop-ops-boolean         booleans and splits
ops/geop-ops-pattern         patterns, mirrors and moves
ops/geop-ops-subd            subdivision surfaces
ops/geop-ops-surface         surfacing
ops/geop-ops-edit            editing bodies and faces
ops/geop-ops-assembly        placed parts, mates, joints, the drag tool
ops/geop-ops-harness         wire harnesses
ops/geop-ops-step            STEP import and export
ops/geop-ops-drawing         2-D drawings
ops/geop-ops-sheetmetal      sheet metal and flat patterns
ops/geop-ops-inspect         mass properties, measurements, interference
ops/geop-ops-bom             bills of materials
cad/geop-cad-base            composition: every crate's `register`, one line
                              each, and the composition's tests
cad/geop-cad-web             the wasm bindings the web app loads (crate
                              `geop`), the protocol checked against `geop.ts`
cad/geop-cad-cli             the `geop` command-line tool
```
