# Introduction

Geop is a Cargo workspace of small crates, each depending only on the ones
before it. `web/` is a React front end that loads the top of that chain
compiled to WebAssembly and renders its output with three.js, and
`geop-cad-cli` puts the same chain on the command line.

This book gives a brief, one-paragraph overview of each crate. It does not
yet document their APIs in depth — see [docs.geop-cad.dev](https://docs.geop-cad.dev)
for the generated rustdoc reference, and expect these chapters to grow over
time.

```text
core/geop-core-math          scalars, interval arithmetic, linear algebra,
                              convex hulls, error type, render primitives
core/geop-core-geometry      NURBS curves/surfaces, containment, intersection
core/geop-core-topology      B-rep structures, Euler operators, edit/validation
core/geop-core-sketch        2-D constraint sketches: entities, constraints,
                              BFGS solver, profile extraction
core/geop-core-part          a part: topology and sketches, every entity
                              with a stable name
ops/geop-ops-extrude-revolve extrude/revolve and the sample shapes (cube,
                              sphere, cylinder, torus, ...)
ops/geop-ops-booleans        3-D boolean operations (union/intersection/diff)
ops/geop-ops-rasterize       turns a Model into a triangle mesh, and writes
                              it as STL
ops/geop-ops-parts           programs: the operations a part is built with,
                              and running them (+ geop-ops-parts-derive)
cad/geop-cad-base            ray-based picking
cad/geop-cad-web             the wasm bindings the web app loads (crate `geop`)
cad/geop-cad-cli             the `geop` command-line tool
```
