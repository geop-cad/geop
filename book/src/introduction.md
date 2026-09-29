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
                              convex hulls, error type, render primitives
core/geop-core-geometry      NURBS curves/surfaces, containment, intersection
core/geop-core-topology      B-rep structures, Euler operators, edit/validation
core/geop-core-sketch        2-D constraint sketches: entities, constraints,
                              BFGS solver, profile extraction
ops/geop-ops                 parts (topology, sketches, datums, every entity
                              with a stable name), what an operation is and
                              how it is edited (events, dialogs, visuals, hit
                              tests), programs and running them (+ geop-ops-derive)
ops/geop-ops-sketch          the sketch operation
ops/geop-ops-datums          the datum operation: reference points, axes,
                              planes and coordinate systems
ops/geop-ops-booleans        3-D boolean operations (union/intersection/diff),
                              the boolean operation
ops/geop-ops-extrude-revolve extrude/revolve and the sample shapes (cube,
                              sphere, cylinder, torus, ...), the extrude and
                              revolve operations
ops/geop-ops-rasterize       turns a Model into a triangle mesh, and writes
                              it as STL
cad/geop-cad-base            the operations the editor offers, example programs
cad/geop-cad-web             the wasm bindings the web app loads (crate `geop`)
cad/geop-cad-cli             the `geop` command-line tool
```
