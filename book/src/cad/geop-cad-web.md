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
[geop-ops](../ops/geop-ops.md#programs) and
[geop-cad-base](./geop-cad-base.md)), and only through this module.
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
| `run_program(stop)`     | builds the first `stop` steps; returns `{results, scene, solids, sketches, datums, handles}` |
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

## Handles in the 3-D viewer

A handle is a value of a step that can be adjusted by dragging in the 3-D
view, such as an extrude's distance. The kernel describes the handles and
the browser draws them. Neither side knows anything about the operation the
handle belongs to.

**Where handles come from.** Every operation can implement
`Operation::handles(before, args)`, which is given the part as it was
*before* the step. Each `Handle` it returns has a `label`, a `position` in
world space, a `group` (`Feature` or `Sketch`), and a `motion` saying how it
moves and which arguments it writes:

- `Linear { direction, arg, value, scale }` — slides along a unit vector.
  Moving it `d` world units changes the number at `arg` from `value` by
  `d / scale`.
- `Planar { u, v, x, y, value }` — slides in the plane spanned by `u` and
  `v`. Moving it by `a·u + b·v` changes the numbers at `x` and `y` by `a`
  and `b`.

An argument is addressed by an `ArgPath`, the field names of the arguments
as they serialize: `["distance"]`, or `["sketch", "points", "7", "x"]`.

| Operation   | Handles                                                            |
| ----------- | ------------------------------------------------------------------ |
| `Extrude`   | `distance`: at the centre of the end cap, sliding along the sketch plane's normal; `scale` is 0.5 for a symmetric extrude, whose cap moves half as far |
| `AddDatum`  | an offset plane's `distance`, along its normal; an offset point's `x`, `y` and `z`, one handle per axis, placed a little out along that axis so each can be grabbed |
| `AddSketch` | every sketch point, `Planar` in the sketch plane (`Sketch` group)  |

`ProgramRunner::handles` collects the handles of every step that succeeded.
Each is computed against the part its own step saw and tagged with the
step's id (`StepHandle`). `run_program` and `preview_program` return them
all, and the editor chooses which to show.

**What the editor shows.** `App.tsx` shows handles only while a step's form
is open, and only that step's `Feature` handles, taken from the *preview*
run. Sketch points are edited in the sketcher, not in the 3-D view.

**Dragging** (`web/src/handles3d.ts`):

1. `HandleLayer` draws each handle as a ball (with two arrow cones for a
   linear one) on top of everything, at a constant size in pixels. Because
   handles are drawn on top, hover and click test them before any model
   entity.
2. On pointer down, `startDrag` records where on its track the handle was
   grabbed: for a linear handle, the parameter of the point on its line
   closest to the pointer ray; for a planar one, where the ray meets its
   plane. The drag takes the pointer, so orbiting and picking don't see it.
3. As the pointer moves, `dragEdits` measures the movement along the same
   track relative to the grab point and turns it into new values:
   `value + Δt / scale` for linear, `value + moved·u` and `value + moved·v`
   for planar. Values snap to 0.01, since dragging is for rough shaping and
   the form is for exact values. At most one set of edits is sent per frame.
4. `App.tsx` writes the new values into the open form's arguments at their
   paths, exactly as typing into the form would. The form change triggers
   `preview_program`, and the `PREVIEW` runner replays only the edited step,
   so the part and the handle's new position update as the drag goes.

A drag therefore never changes geometry directly. It edits an argument of a
program step, and the program is rebuilt from it.
