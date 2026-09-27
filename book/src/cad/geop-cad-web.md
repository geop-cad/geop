# geop-cad-web

> Brief overview only — full documentation is coming later.

The WebAssembly bindings the web app loads, published as crate `geop`.
Compiled with `wasm-pack` and consumed by the React front end in `web/`.

```sh
cd web
npm run build:wasm   # wasm-pack build ../cad/geop-cad-web --target web ...
```

The package is named `geop` rather than `geop-cad-web` so that the output
files (`geop.js`, `geop_bg.wasm`) match what `web/src/geop.ts` imports.

## The editor's view of the kernel

The browser edits a `Program` (see
[geop-ops-parts](../ops/geop-ops-parts.md)), and only through this module.
Everything crosses the boundary as JSON: programs and edits in their own
serde format, and scenes as flat number arrays that a three.js viewer
consumes directly. Entities are named, never numbered: a pick returns the
name of what was hit.

| Export                  | Does                                                             |
| ----------------------- | ---------------------------------------------------------------- |
| `init_panic_hook`       | forwards Rust panics to the JS console with a stack trace        |
| `operation_schemas`     | every operation and its arguments, for building forms            |
| `example_programs`      | the built-in example programs                                    |
| `program`               | the program being edited                                         |
| `update_program(edit)`  | applies a `ProgramEdit`, the only way the program changes        |
| `run_program(stop)`     | builds the first `stop` steps; returns `{results, scene, solids, sketches}` |
| `preview_program(edit, stop)` | builds the program as if `edit` were applied, without applying it |
| `pick_ray`              | the named entity under a ray, matching a filter                  |
| `sketch_plane(plane)`   | the frame of a sketch plane in the current part                  |
| `inspect_selection_fit` | what each selected entity can be used as, and which datum constructions fit |
| `solve_sketch(sketch, drags)` | solves a sketch, optionally dragging points, and returns it with its `SolveReport` and region outlines |

The module keeps its state in a few thread-local slots:

- **`PROGRAM`** — the program, changed only by `update_program`.
- **`COMMITTED`** — the `ProgramRunner` that builds it. Picks and sketch
  planes resolve against its last run and never against a preview, so
  nothing can refer to geometry that only a not-yet-made edit would
  create.
- **`VIEW`** — the rasterization of the committed part, sampled once per run.
  Every pick tests against it, so picks are cheap enough for hovering and hit
  exactly what is on screen.
- **`PREVIEW`** — a second runner with its own cache, so re-previewing while
  a slider moves only replays the edited step.

Both runners only rebuild from the first step that changed since their last
run.
