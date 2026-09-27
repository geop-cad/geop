# Geop CAD-Kernel

Geop is a CAD kernel written in Rust based on NURBS.

- [geop-cad.dev](https://geop-cad.dev) — project landing page
- [app.geop-cad.dev](https://app.geop-cad.dev) — the web app
- [book.geop-cad.dev](https://book.geop-cad.dev) — a short, crate-by-crate tour
- [docs.geop-cad.dev](https://docs.geop-cad.dev) — generated rustdoc reference

If Geop is useful to you, consider [sponsoring on Ko-fi](https://ko-fi.com/tobiasjacob)
— it funds the time that goes into the kernel.

## License

Geop is licensed under the [Business Source License 1.1](LICENSE.md), converting
to Apache-2.0 on 2030-09-27 (each release gets its own 4-year clock). In short:

- ✅ **Free for basically everything** — using it in your own product or
  infrastructure, research, embedding it in a commercial app, a business
  building a parts configurator on their site, a contractor building one for
  a client, self-hosting it, forking and modifying it.
- ✅ **Free to build CAD tools with it**, as long as they're free for end users
  and run entirely client-side (e.g. compiled to WebAssembly in the browser).
- ❌ **Not free** if you want to sell a general-purpose, hosted CAD design
  service where users design and upload arbitrary parts online. That's the
  one thing reserved for commercial licensing from the Geop maintainers,
  since it's the product we intend to build ourselves.
- 🕓 Every released version becomes fully open source (Apache-2.0) 4 years
  after its release, so nothing is locked away forever.
- ⚠️ The code is provided **as is, with no warranty of any kind** — see the
  disclaimer in [LICENSE.md](LICENSE.md) for the full terms.

Need a commercial license for a competing hosted CAD service, or unsure which
bucket your use case falls into? Reach out to the maintainers.

## Contributing

Contributions require agreeing to the [Contributor License Agreement](CLA.md),
which assigns copyright in your contribution to the maintainer (with a
license granted back to you for your own use) so the project can keep
relicensing and selling commercial licenses without needing to track down
every past contributor for permission. See CLA.md for how to sign it.


**Kernel (Rust):**
```sh
cargo test        # run the whole workspace's test suite
cargo build        # native build, e.g. for the examples under each crate's examples/

cargo install --path cad/geop-cad-cli
geop compile part.program.json -o out/part.stl  # compile to stl
```

**Web app:**

Requires `wasm-pack` (`cargo install wasm-pack`) and the
`wasm32-unknown-unknown` target (`rustup target add wasm32-unknown-unknown`),
plus Node 20+.

```sh
cd web
npm install         # once
npm run dev          # rebuilds the wasm bindings, then starts the Vite dev server
```

### Programs

A part is described by a *program*: an ordered list of steps, each an
operation (`add_sketch`, `extrude`, `revolve`, `boolean`, `add_datum`) with its arguments.
Steps refer to what earlier steps made only by stable names — `extrude(box)`
is the solid the step `box` extruded, `extrude(box,end)` its end cap — never
by a kernel id, so a program is self-contained and rebuilds the same part,
name for name, every time. It is stored as JSON, one step per object and
every sketch entity keyed by its id, so edits show up as small diffs:

```json
{
  "steps": [
    { "id": "outline", "operation": "add_sketch",
      "args": { "plane": { "type": "Plane", "normal": "Z" }, "sketch": { "points": { ... }, "curves": { ... }, "constraints": { ... } } } },
    { "id": "box", "operation": "extrude",
      "args": { "sketch": "outline", "distance": 1.0, "symmetric": false,
                "combine": { "mode": "new_body" } } },
    { "id": "hole_sketch", "operation": "add_sketch",
      "args": { "plane": { "type": "Face", "name": "extrude(box,end)" }, "sketch": { ... } } },
    { "id": "hole", "operation": "extrude",
      "args": { "sketch": "hole_sketch", "distance": -0.5, "symmetric": false,
                "combine": { "mode": "difference", "target": "extrude(box)" } } }
  ]
}
```

Whatever a step builds on directly — a sketch's plane, what a reference is
built from — is an *entity reference*: `{"type": "Origin"}`, `{"type":
"Axis", "axis": "X"}`, a base plane `{"type": "Plane", "normal": "Z"}`, or a
`Vertex`, `Edge`, `Face` or `Datum` of the part `{"type": "Face", "name":
"..."}`.

"Save" in the web app downloads the program as `part.program.json`, and
"Load" reads one back. `outputs/parts/` has the examples of
`ops/geop-ops-parts/src/examples.rs` as program files.
