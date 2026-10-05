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
- `src/geop.ts` — thin typed wrapper around the kernel.
- `src/backend.ts` — where the kernel runs: the wasm module in a worker
  (`src/kernel.worker.ts`), replaced by a fresh one, given the program the
  app holds, if it crashes. `src/backend.vscode.ts` replaces it in the VS
  Code build.
- `src/App.tsx` — the editor: the program, the step being edited, and
  sending what the user does to `edit_step`.
- `src/OperationRibbon.tsx` — the operations as a ribbon of the groups the
  editor sends them in (`#[operation(group = ..., primary)]` on
  `PartOperation`): big buttons for the primary ones, small stacked ones
  for the rest, giving way to menus where the window is narrow; a section
  per group on a phone. A new operation needs no change here.
- `src/selection.ts` — a drag selects text only inside the dialog or panel
  it starts in; the app's chrome is never selectable.
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

## End-to-end checks

`web/e2e/` drives the real app in a headless Chrome through
`playwright-core`, which downloads no browser: it uses `$CHROME`, or a
Google Chrome or Chromium installed in the usual places, and skips (exit
code 0, with a message) if there is none. WebGL is rendered in software.
Cap their memory with a cgroup (`systemd-run --user --scope -p
MemoryMax=20G npm run e2e`): Chrome reserves far more address space than
it uses, so `prlimit --as` kills it.

```sh
npm run e2e          # the web app
npm run e2e:vscode   # the VS Code extension's page against a real `geop serve`
```

- `npm run e2e` rebuilds `dist/` if any source is newer (`npm run build`),
  serves it with `vite preview` on a free port, and checks that every
  operation is in reach of the toolbar — a button, or an entry of a menu of
  it — at 1400, 1100 and 900 px wide, with nothing scrolled (screenshots
  of the bar at 1400, 1100 and 800 px land in `e2e/out/toolbar-*.png`);
  that a drag over the 3-D view selects no text, and one in a dialog or
  the program panel selects only there; that every
  example of the File menu builds without an error shown or a failed step;
  that every operation opens on an empty part and on a part, and cancels;
  that a rectangle sketched by clicks extrudes, and the inspect panel weighs
  it; that a kernel that panics is restarted with the program, and says
  so; that a SubD face and a 3-D sketch point are dragged by their gizmos'
  arrows, lit where the kernel says the pointer is (with `--shots`, a
  screenshot half way through each drag, `gizmo-*.png`); and that STEP, STL, SVG, DXF, URDF and the BOM's CSV download as
  non-empty files of their kind.
- `npm run e2e:vscode` builds the release CLI and the extension's page
  (`build:vscode`), writes example workspaces with `geop examples
  --out-dir`, and opens them through `e2e/bridge.mjs`, which stands in for
  VS Code exactly as `vscode-extension/src/geopEditor.ts` talks to the page,
  running the kernel with the extension's own `GeopServer` (compiled with
  `npm run compile` there):
  single parts, an assembly with standard parts (`bolted_plate`) and a
  jointed one (`arm`). An edit, its undo and a moved joint must be written
  back to the document, exports must reach VS Code to be saved, and a
  `geop serve` that panics must be restarted with the document.

Both exit non-zero if any check fails: a page error, an error shown, a
step that fails, an example that does not build. A failing check leaves a
screenshot in `e2e/out/` (gitignored). Flags: `--build` rebuilds even if
the build looks current, `--only <text>` runs the checks whose name
contains it, `--shots` keeps a screenshot of every check.
