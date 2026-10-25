# geop

**Parametric CAD for designing real machines: an exact, interval-arithmetic NURBS kernel in Rust, driving parts and assemblies written as programs with typed references.**

[![CI](https://github.com/geop-cad/geop/actions/workflows/ci.yml/badge.svg)](https://github.com/geop-cad/geop/actions/workflows/ci.yml)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)
[![Discord](https://img.shields.io/badge/Discord-join%20the%20chat-5865F2?logo=discord&logoColor=white)](https://discord.gg/btRDgrcwJ5)
[![Web app](https://img.shields.io/badge/app.geop--cad.dev-open%20in%20the%20browser-0a7d5c)](https://app.geop-cad.dev)

![A part sketched, extruded and filleted in the geop web app, then rebuilt with a new distance](docs/demo.gif)

*New part, a rectangle sketched by clicks, extruded, an edge filleted, and the extrusion's distance changed: the fillet follows.*

## Try it

- **In the browser:** [app.geop-cad.dev](https://app.geop-cad.dev). Nothing to install; files stay in your browser.
- **On the command line:**
  ```sh
  cargo install --path cad/geop-cad-cli
  geop examples --out-dir examples   # every built-in example as a .geop program and its STL
  geop compile part.geop -o part.step
  ```
- **From Claude Code (or any MCP client):**
  ```sh
  claude mcp add geop -- geop mcp
  ```
  Then ask for a part: the agent sketches, extrudes, fillets, inspects and exports through the same editor the web app runs. See [docs/mcp.md](docs/mcp.md).
- **In VS Code:** the extension in [`vscode-extension/`](vscode-extension/README.md) opens any `.geop` file as the editor, with the kernel running natively.

## What works

Taken from [ROADMAP.md](ROADMAP.md), which also says what is next.

- **Sketching:** 2-D constraint sketches with slots, polygons, fillets, offsets, mirrors and patterns, redundant constraints proven and conflicts named; 3-D sketches as wires; exact helices.
- **Part features:** extrude, revolve, sweep (with rails and twist) and loft; holes from ISO tables, cosmetic and modelled threads; rolling-ball fillets on any edge with variable radius and rounded corners, chamfers; shell, rib, lip and groove, draft; linear and circular patterns and mirrors of bodies and features; parameters as formulas.
- **Surfacing:** ruled, boundary, N-sided and Gordon (UV) surfaces, offset, thicken, knit, trim and extend; Catmull–Clark subdivision surfaces converted to valid NURBS solids.
- **Sheet metal:** base and edge flanges, hems, corners, cuts across bends, exact flat patterns as DXF.
- **Assemblies:** multi-file assemblies placed by mates; revolute, slider, cylindrical and fastened joints with limits, gear, rack-and-pinion and screw couplings; standard parts (ISO fasteners, bearings, extrusions, steppers, gears, pulleys, rails, servos); wire harnesses with bend radii and cut lengths; a drag in 2000 parts in about 0.25 s.
- **Inspection and output:** measure, mass properties from the exact B-rep, interference, section views; bills of materials; drawings with hidden lines, sections, dimensions and balloons as SVG and DXF; STEP AP214 import and export (322 of 361 public corpus files import valid); URDF for robot simulators; STL.
- **Front ends:** the web app, the VS Code extension, the `geop` CLI and the MCP server all drive one headless editor over one JSON protocol.

## How it is built

- Every number is an interval: comparisons are three-valued (`definitely`, `could be`), searches subdivide to isolate and Newton to refine, and no epsilon decides an answer. [AGENTS.md](AGENTS.md) records the rules and the bugs that made them.
- A part is a *program*: an ordered list of steps, each an operation with its arguments, referring to what earlier steps built by stable names (`extrude(box,end)` is the end cap of the step `box`), never by a kernel id, so it rebuilds the same part name for name.
- The workspace has four tiers: `core/` (math, NURBS, the B-rep, programs, the editor), `lib/` (algorithms two operations share), `ops/` (one crate per family of operations, each with its forms, examples and tests) and `cad/` (the composition, one `register` line per crate, carried to the browser, the command line and MCP).
- The [book](https://book.geop-cad.dev) tours the crates; [docs.geop-cad.dev](https://docs.geop-cad.dev) is the rustdoc; [DEVELOPERS.md](DEVELOPERS.md) has the crate table, scripts and conventions.
- The landing page is [geop-cad.dev](https://geop-cad.dev).

<details>
<summary>A program, as it is saved</summary>

```json
{
  "steps": [
    { "id": "outline", "operation": "add_sketch",
      "args": { "plane": { "type": "Datum", "name": "origin", "component": { "plane": "z" } },
                "sketch": { "points": { ... }, "curves": { ... }, "constraints": { ... } } } },
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

Whatever a step builds on is an *entity reference*: a `Vertex`, `Edge`, `Face`,
`Solid`, `Sketch` or `Datum` by name. Every part starts with the datum
`origin`; one of its axes or planes is `{"type": "Datum", "name": "origin",
"component": {"plane": "z"}}`. A program places the parts other files build
with `add_part` and mates their entities (`pin/extrude(pin,start)`); where
each placed part is, is the program's `state`, solved after every edit.
`geop compile assembly.geop` reads the placed files next to it.
</details>

## Contributing

Issues and pull requests are welcome; see [CONTRIBUTING.md](CONTRIBUTING.md)
for how the code is laid out and tested, and come talk on
[Discord](https://discord.gg/btRDgrcwJ5).

If geop is useful to you, consider [sponsoring on Ko-fi](https://ko-fi.com/tobiasjacob):
it funds the time that goes into the kernel.

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at
your option. The code is provided as is, with no warranty of any kind.
Unless you state otherwise, a contribution you submit is licensed the same
way, with no further terms.
