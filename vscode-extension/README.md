# Geop for VS Code

Opens any `.geop` file in VS Code as the geop CAD editor — the same web
frontend as [app.geop-cad.dev](https://app.geop-cad.dev), but the kernel runs
as a native process on your machine instead of as WebAssembly.

```
VS Code ── webview (web/, built with `--mode vscode`)
              │  postMessage
          extension host (src/)
              │  one JSON line per command / update, over stdio
          `geop serve` (cad/geop-cad-cli) ── geop_cad_base::Editor
```

- One `geop serve` process is started per open editor and ends with it; the
  process holds the editing state (undo, the step being edited).
- The text document stays the source of truth. Every change to the program is
  written into it as an ordinary text edit, so saving, VS Code's undo/redo,
  dirty markers, diffs and source control all work. A `.geop` file is JSON;
  "Reopen Editor With… → Text Editor" shows it.
- An empty `.geop` file is an empty program.
- A program places the parts other `.geop` files build ("Part" operation), by
  their paths relative to its own — `bolt.geop`, `../parts/bolt.geop`. The
  page is sent every `.geop` file of the document's workspace folder, open
  ones as edited (saved or not), and again whenever one changes, so an
  assembly follows its parts live. Files must not place each other in a
  cycle.
- What a file is made into — a STEP file's bodies — is kept beside it, as
  `name.STEP.geopc` (`geop serve --cache-root <folder>`), so the next editor
  opened, or the next session, reads it from there instead of reading the
  file again. It is only read back by the same build of the kernel. A
  `.geopc` file is a cache: delete it freely, and keep it out of source
  control (`*.geopc` in `.gitignore`).

## Build

Needs Node, Rust and the repo's `web/` dependencies (`npm install` there).

```sh
npm install
npm run build        # page -> media/, kernel -> bin/, TypeScript -> out/
```

Then press F5 in this folder to start an Extension Development Host, or
`npm run package` for a `.vsix`. The kernel is native code, so a `.vsix`
runs only on the platform it was built on.

`geop.serverPath` points the extension at a different `geop` executable
(e.g. `target/debug/geop` while working on the kernel).
