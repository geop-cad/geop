# Introduction

Geop is a Cargo workspace of small crates, each depending only on the ones
before it. `web/` is a React front end that loads the top of that chain
compiled to WebAssembly and renders its output with three.js, and
`geop-cad-cli` puts the same chain on the command line.

This book gives an overview of each crate: what it is for, its main types
and operations, and the design decisions behind them. It does not document
every API in depth — see [docs.geop-cad.dev](https://docs.geop-cad.dev) for
the generated rustdoc reference, and expect these chapters to grow over
time.

```text
core/geop-core-math          scalars, interval arithmetic, linear algebra,
                              error type, frames, rays, datums, poses,
                              constraint systems
core/geop-core-geometry      NURBS curves/surfaces, containment, intersection
core/geop-core-topology      B-rep structures, Euler operators, edit/validation
core/geop-core-sketch        2-D constraint sketches: entities, constraints
                              as residuals, profile extraction
core/geop-core-program       programs and the parts they build: targets,
                              typed references to their elements, running
                              a program
core/geop-core-editor        what an operation is and how it is edited:
                              forms, dialogs, visuals, events, picks,
                              numbers as formulas
ops/geop-ops-model           the model a part's steps build: topology with
                              stable names, features, geometric kinds
ops/geop-ops-parameters      a program file: parameters, state, what it
                              says of its part, running it
ops/geop-ops-view            what an editor draws of a part, and picking
ops/geop-ops-output          exporters, inspectors, sheets (SVG, DXF)
ops/geop-ops-constraints     constraints and unknowns of a part's state,
                              and solving them
ops/geop-ops-datums          the part's datums: the origin, and those steps
                              add
ops/geop-ops-sketch          the sketch operation
ops/geop-ops-add-datum       the datum operation: reference points, axes,
                              planes and coordinate systems
ops/geop-ops-booleans        3-D boolean operations (union/intersection/diff),
                              the boolean operation
ops/geop-ops-extrude-revolve extrude/revolve, sweeps along paths, lofts;
                              their operations, and basic shapes for tests
ops/geop-ops-assembly        placed parts, mates, joints, solving them; the
                              library, the solids a part shows, STL export
ops/geop-ops-rasterize       turns a Model into a triangle mesh, writes it
                              as STL, and renders it for debugging
cad/geop-cad-base            the operations the editor offers, the part as
                              composed from every crate, the workspace, the
                              editor engine, examples, standard parts
cad/geop-cad-web             the wasm bindings the web app loads (crate `geop`)
cad/geop-cad-cli             the `geop` command-line tool
```
