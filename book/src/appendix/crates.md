# C. Map of the crates

Geop is a Cargo workspace in four tiers. `core/` defines what things are: math, geometry and topology; a program, the part it builds, and how it is edited. `lib/` holds the algorithms two operations share, taking geometry and models. `ops/` has one crate per family of operations, each with its forms, examples and tests, over the targets `geop-ops-model` defines. `cad/` composes them, one `register` line per crate, and carries them to the browser and the command line. `web/` is a React front end that loads the composition compiled to WebAssembly and renders its output with three.js.

The rules between the tiers are checked by `scripts/check-layering.sh`, and are explained in [Lecture 40](../lectures/40-writing-an-operation.md). This table lists every crate in the workspace's `Cargo.toml`, with the reference page where there is one and the lectures that teach it.

## `core/`: what things are

| Crate | What it is | Reference | Taught in |
|---|---|---|---|
| `core/geop-core-math` | the interval scalar and the number trait it shares with dual numbers, linear algebra, errors, frames, rays, datums, poses, solvers | [page](../core/geop-core-math.md) | [4](../lectures/04-intervals.md), [5](../lectures/05-honest-numerics.md), [24](../lectures/24-rank-and-proof.md), [37](../lectures/37-assembly-solver.md) |
| `core/geop-core-geometry` | NURBS curves and surfaces, their constructors, compatible curves, Coons and Gordon patches, containment and intersection | [page](../core/geop-core-geometry.md) | [6](../lectures/06-bezier.md) to [16](../lectures/16-tracing-and-pcurves.md) |
| `core/geop-core-topology` | the B-rep, building, editing and validation, wires | [page](../core/geop-core-topology.md) | [17](../lectures/17-brep.md) to [20](../lectures/20-validation.md) |
| `core/geop-core-rasterize` | a model as a triangle mesh, STL, debug renderings | [page](../core/geop-core-rasterize.md) | [39](../lectures/39-editor.md), [42](../lectures/42-case-studies.md) |
| `core/geop-core-sketch` | 2-D constraint sketches: entities, constraints as residuals, profiles, dimensions in a plane | [page](../core/geop-core-sketch.md) | [23](../lectures/23-sketch-solver.md) to [25](../lectures/25-sketch-features.md) |
| `core/geop-core-program` | a program, its values and the part it builds: a memo table of nodes, placed parts, the runner, the workspace and the cache | [page](../core/geop-core-program.md) | [2](../lectures/02-programs-of-steps.md), [38](../lectures/38-runner.md) |
| `core/geop-core-editor` | what an operation is and how it is edited, what an editor draws, exporters and inspectors, the editor over a registry | [page](../core/geop-core-editor.md) | [39](../lectures/39-editor.md), [40](../lectures/40-writing-an-operation.md) |

## `lib/`: algorithms operations share

| Crate | What it is | Reference | Taught in |
|---|---|---|---|
| `lib/geop-lib-sheet` | the sheets drawings and flat patterns are written on: SVG, DXF | [page](../lib/geop-lib-sheet.md) | [35](../lectures/35-step-and-drawings.md) |
| `lib/geop-lib-booleans` | booleans and the remesh under them, split, imprint, trim up to the next face, a step's `Combine` | [page](../lib/geop-lib-booleans.md) | [21](../lectures/21-remesh.md), [22](../lectures/22-boolean.md) |
| `lib/geop-lib-sweep` | extrude, revolve, screw, sweeps along paths, lofts, the skinned grid blends share | [page](../lib/geop-lib-sweep.md) | [26](../lectures/26-sweeps.md) to [28](../lectures/28-lofts-and-fills.md) |
| `lib/geop-lib-shell` | rebuilding a solid face by face, shelling, thickening and offsetting faces | [page](../lib/geop-lib-shell.md) | [30](../lectures/30-one-rebuild.md), [31](../lectures/31-offsets.md) |
| `lib/geop-lib-fixtures` | test-only steps: a solid or a datum at given coordinates; the stress sweeps' bodies | [page](../lib/geop-lib-fixtures.md) | [40](../lectures/40-writing-an-operation.md) |
| `lib/geop-lib-stdlib` | the standard parts, one program per family, and the store that puts them beside a workspace's files | [page](../lib/geop-lib-stdlib.md) | [38](../lectures/38-runner.md) |
| `lib/geop-lib-constraints` | the constraints and unknowns of a part's state, and the solver over them | [page](../lib/geop-lib-constraints.md) | [37](../lectures/37-assembly-solver.md) |

## `ops/`: one crate per family of operations

| Crate | What it is | Reference | Taught in |
|---|---|---|---|
| `ops/geop-ops-model` | the vocabulary every operation shares: the model, sketch, datum, parameter and placed-part targets and the references into them | [page](../ops/geop-ops-model.md) | [2](../lectures/02-programs-of-steps.md), [40](../lectures/40-writing-an-operation.md) |
| `ops/geop-ops-sketch` | the sketch operation, its tools, snapping and references | [page](../ops/geop-ops-sketch.md) | [23](../lectures/23-sketch-solver.md), [25](../lectures/25-sketch-features.md) |
| `ops/geop-ops-sketch3d` | the 3-D sketch operation: wires in space, paths and rails | | [25](../lectures/25-sketch-features.md) |
| `ops/geop-ops-reference-geometry` | the datum operation: points, axes, planes and frames from what is picked | [page](../ops/geop-ops-reference-geometry.md) | [2](../lectures/02-programs-of-steps.md) |
| `ops/geop-ops-extrude-revolve` | extrude, revolve, sweep and loft | [page](../ops/geop-ops-extrude-revolve.md) | [26](../lectures/26-sweeps.md) to [28](../lectures/28-lofts-and-fills.md) |
| `ops/geop-ops-hole` | holes from ISO and NPT tables, threads on straight or tapered walls, cosmetic or modelled | | [32](../lectures/32-patterns-holes-threads.md) |
| `ops/geop-ops-fillet` | fillets and chamfers, rolling-ball blends of tangent chains | | [29](../lectures/29-fillets.md) |
| `ops/geop-ops-shell` | the shell operation | | [30](../lectures/30-one-rebuild.md) |
| `ops/geop-ops-plastic` | ribs, lips and grooves, drafts | | [33](../lectures/33-sheet-metal-plastic.md) |
| `ops/geop-ops-boolean` | the boolean and split operations | [page](../ops/geop-ops-boolean.md) | [22](../lectures/22-boolean.md), [40](../lectures/40-writing-an-operation.md) |
| `ops/geop-ops-pattern` | patterns, mirrors and moves of bodies and features; bodies scaled, tapered, twisted or bent | | [32](../lectures/32-patterns-holes-threads.md) |
| `ops/geop-ops-subd` | subdivision surfaces: a control cage, box modelling, its Catmull–Clark limit | | [34](../lectures/34-subd.md) |
| `ops/geop-ops-surface` | boundary and UV surfaces, offset, thicken, knit, trim and extend | | [28](../lectures/28-lofts-and-fills.md), [31](../lectures/31-offsets.md) |
| `ops/geop-ops-edit` | delete a body, extract a face, project a sketch; direct edits: move, offset, replace and delete faces | | [30](../lectures/30-one-rebuild.md) |
| `ops/geop-ops-assembly` | placing parts, mates and joints, part patterns, the joints panel, the drag tool, the settler, STL and URDF export | [page](../ops/geop-ops-assembly.md) | [36](../lectures/36-mates.md), [37](../lectures/37-assembly-solver.md) |
| `ops/geop-ops-harness` | wire harnesses routed through connectors and clips | | not taught |
| `ops/geop-ops-step` | STEP import and export, assemblies as products and occurrences; STL meshes imported as B-reps | | [35](../lectures/35-step-and-drawings.md) |
| `ops/geop-ops-drawing` | 2-D drawings: views, hidden lines, sections, dimensions, title block, parts list | | [35](../lectures/35-step-and-drawings.md) |
| `ops/geop-ops-sheetmetal` | flanges, bends, hems, cuts, the flat pattern and its DXF | | [33](../lectures/33-sheet-metal-plastic.md) |
| `ops/geop-ops-inspect` | mass properties, measurements, interference | | [19](../lectures/19-classification.md), [41](../lectures/41-performance.md) |
| `ops/geop-ops-bom` | bills of materials, an inspector and a CSV exporter | | not taught |

## `cad/`: composition

| Crate | What it is | Reference | Taught in |
|---|---|---|---|
| `cad/geop-cad-base` | composition: `registry`, one `register` line per crate, the editor of all of it beside the standard parts, and the composition's tests | [page](../cad/geop-cad-base.md) | [39](../lectures/39-editor.md) |
| `cad/geop-cad-web` | the wasm bindings the web app loads (package `geop`), and the check of `web/src/geop.ts` against the protocol | [page](../cad/geop-cad-web.md) | [39](../lectures/39-editor.md) |
| `cad/geop-cad-cli` | the `geop` command line: `compile`, `examples`, `serve`, `open`, `app`, `mcp` | [page](../cad/geop-cad-cli.md) | [39](../lectures/39-editor.md) |

Outside the workspace: `web/` is the React front end, `vscode-extension/` the VS Code extension, `homepage/` the landing page, `cloudflare-functions/` the Cloudflare Pages Functions deployed with the web app, and `scripts/` the release, layering and affected-test scripts.

## Which page should I read?

A lecture tells you why, and a crate page tells you what is there. Reference pages cover each crate's types and decisions in a compact form, without exercises, and do not repeat the lectures' first principles. They were written alongside the code and are older than the lectures; if the two disagree, trust the code first and the lectures second.
