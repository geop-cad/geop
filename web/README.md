# geop web

A React + Vite front end that loads the `geop` Rust crate compiled to
WebAssembly (via `wasm-bindgen`/`wasm-pack`) and renders its output with
`three.js`. This is the first slice of a browser-based CAD UI: it proves the
crate can build a solid, rasterize it, and hand the mesh to JS end-to-end.

## Layout

- `../cad/geop-cad-web/src/lib.rs` — the `#[wasm_bindgen]` surface exposed to
  JS (crate name `geop`, the one `build:wasm` compiles): the program, running
  it into scenes, and `edit_step`, through which every step is edited.
- `src/wasm/pkg/` — generated `wasm-pack` output (JS glue + `.wasm` binary).
  **Not committed** — rebuilt by `npm run build:wasm` (also runs
  automatically before `dev`/`build`).
- `src/geop.ts` — thin typed wrapper around the generated bindings.
- `src/App.tsx` — the editor: the program, the step being edited, and
  sending what the user does to `edit_step`.
- `src/DialogView.tsx` — renders a step's dialog from its primitives.
- `src/SceneViewer.tsx` — renders a `Scene` with `three.js`, draws what the
  step being edited shows (`src/visuals3d.ts`, `src/planeGrid.ts`), and sends
  hovers, clicks and drags as rays.

## Requirements

- `wasm-pack` (`cargo install wasm-pack`) and the `wasm32-unknown-unknown`
  target (`rustup target add wasm32-unknown-unknown`).
- Node 20+.

## Commands

```sh
npm install        # once
npm run dev         # rebuilds the wasm pkg, then starts the Vite dev server
npm run build        # rebuilds the wasm pkg, then produces a production build in dist/
```
