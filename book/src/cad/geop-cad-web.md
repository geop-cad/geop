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
| `operation_infos`       | every operation the editor offers: `{kind, label, doc}`          |
| `example_programs`      | the built-in example programs                                    |
| `program`               | the program being edited                                         |
| `describe_program`      | every step as a list shows it: `{id, kind, label, summary}`      |
| `update_program(edit)`  | applies a `ProgramEdit`, the only way the program changes        |
| `run_program(stop)`     | builds the first `stop` steps; returns `{results, scene, datums, extent, references}` |
| `preview_program(edit, stop)` | builds the program as if `edit` were applied, without applying it |
| `edit_step(request)`    | edits a step: `{operation, session, event}` in, `{operation, session, presentation}` out |

The module keeps its state in a few thread-local slots:

- **`PROGRAM`** — the program, changed only by `update_program`.
- **`COMMITTED`** — the `ProgramRunner` that builds it. A step is edited
  against its last run — the editor runs it up to the step being edited —
  and never against a preview, so nothing can refer to geometry that only
  a not-yet-made edit would create.
- **`VIEW`** — the committed part as drawn (`geop_ops::ui::PartView`), made
  once per run. The scene is built from it and every pick tests against it,
  so picks are cheap enough for hovering and hit exactly what is on screen.
- **`PREVIEW`** — a second runner with its own cache, so re-previewing while
  a slider moves only replays the edited step.

Both runners only rebuild from the first step that changed since their last
run.

## Editing a step

The editor holds a step being edited as three things: the step (its
operation and arguments), the session `edit_step` last returned, and the
presentation it came with. Everything the user does goes to `edit_step` as
an event, and what comes back replaces all three:

- **Dialog controls** send `{type: "dialog", key, value}` with the key of
  the control used. `web/src/DialogView.tsx` renders any dialog from its
  primitives alone.
- **The viewport** (`web/src/SceneViewer.tsx`) sends hovers (once a frame),
  clicks — primary or secondary, double or not — and drags, each as a
  `Pointer`: the ray through the cursor, the screen's right and up, and what
  a pixel measures along the ray. A press starts a drag only where the last
  presentation offered a `grab`; anywhere else it moves the camera.
- **Keys** go to the step unless a field is being typed into.

The viewer draws the presentation's visuals (`web/src/visuals3d.ts`) over
the model at the same on-screen sizes the kernel hit-tests them at, lights
its highlights, fades reference geometry that cannot be picked, and — when
the presentation names a plane to work in — turns to face it, stops
orbiting (dragging pans) and draws a grid on it.

Every change of the arguments reruns the program up to and including the
step with `preview_program`, which says whether the step builds — only then
can it go into the program — and shows what it builds.
