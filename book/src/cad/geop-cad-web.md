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

The browser edits a program through one `geop_cad_base::Editor` (see
[geop-cad-base](./geop-cad-base.md#the-editor)), and only through it:

| Export                  | Does                                                             |
| ----------------------- | ---------------------------------------------------------------- |
| `init_panic_hook`       | forwards Rust panics to the JS console with a stack trace        |
| `handle(command)`       | applies a `Command`, and returns the `Update` to show            |

Everything crosses the boundary as JSON, entities by name, never by number.
`web/src/App.tsx` keeps the last update and nothing else of the program:

- **Dialog fields** send `{type: "dialog", key, value}` events with the key
  of the field used. `web/src/DialogView.tsx` renders any dialog from its
  primitives alone.
- **The viewport** (`web/src/SceneViewer.tsx`) sends hovers (once a frame),
  clicks — primary or secondary, double or not — and drags, each as a
  `Pointer`: the ray from the eye through the cursor, and how far from it
  the pointer reaches — `REACH_PX` pixels, as a cone from the eye in
  perspective or a tube in an orthographic view. A press starts a drag only
  where the last presentation offered a `grab`; anywhere else it moves the
  camera.
- **Keys** go to the step unless a field is being typed into.

The viewer draws the part (a serialized `geop_ops::ui::PartView`), lights
the presentation's highlights, draws its visuals (`web/src/visuals3d.ts`)
over the model at the same on-screen sizes the kernel hit-tests them at,
fades reference geometry that cannot be picked, and — when the presentation
names a plane to work in — turns to face it, stops orbiting (dragging pans)
and draws a grid on it.
