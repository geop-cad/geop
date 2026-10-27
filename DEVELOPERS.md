
Things I tried that didn't work:
- Geop 1: Using lines, arcs, spheres, tori, spirals etc. as primitives with explicit intersection formulas. Didn't scale as equations became too complex. Interval arithmetic was nice though and 2d booleans worked reliably.
- Geop 2: Went deep into algebra, symbolic math, ideals, Groebner bases, etc. to solve the intersection problems. Too slow and too much overhead. Decided to use only numerical methods from now on.
- Geop 3: Focused on getting the numerical methods right, first POCs for intersection of NURBS and nonlinear equation solving. Finally found something that could do the job. Decided to use dynamic ndarrays to represent multi-dimensional Bernstein polynomials from now on.
- Geop 4: Got stuck in numerical solvers and topology. Booleans looked promising but tracing was too difficult, too many edge cases.
- Geop 5: Based only on subdivion again, but instead of using convex hulls, approximate short segments with fat lines and fat surfaces. Interface for curve only provides subdivision, midpoint, tangent, and fat line approx. Interface for surfaces only provides subdivision, midpoint, normal, and fat surface approx. Intersection curves are never traced explicitly, they are just stored as a set of surface patches that can be subdivided further if needed. Intersections are trivial like this. Containment is done by ray shooting. We do not yet store any topology information, just a bag of surface bounded surface patches. The surface patches are also bounded by 3d curves, not in their parametric space. 

### Development

The kernel is a Cargo workspace in four tiers: `core/` defines what things
are (math, geometry, topology; a program, a part, an editor), `lib/` holds
algorithms two operations share, `ops/` has one crate per family of
operations, and `cad/` composes them and carries them to the web app and
the command line. `web/` is a React front end that loads `cad/geop-cad-web`
compiled to WebAssembly and renders its output with `three.js`;
`cad/geop-cad-desktop` is the native one, the same editor linked in.
`cloudflare-functions/` holds the Cloudflare Pages Functions deployed with
the web app (bug reports). `scripts/check-layering.sh` keeps the tiers
apart (see Scripts).

| crate | what it is |
|---|---|
| `core/geop-core-math` | the interval scalar and the number trait it shares with dual numbers, linear algebra, the error type, frames, rays, datums, poses, solvers |
| `core/geop-core-geometry` | NURBS curves and surfaces, their constructors, compatible curves, Coons and Gordon patches, containment and intersection |
| `core/geop-core-topology` | the B-rep, building, editing and validation, wires; faces on one surface merged (`BodySpec::merged`) |
| `core/geop-core-rasterize` | a model as a triangle mesh, STL, debug renderings |
| `core/geop-core-sketch` | 2-D constraint sketches: entities, constraints as residuals, profiles, dimensions in a plane |
| `core/geop-core-program` | a program, its values and the part it builds: a memo table of nodes made by `StepContext::produce`, placed parts, the runner, the workspace of files and the `Cache` of what they were made into and what their programs built (`.geopc` beside a file, given a `CacheStore`; `kept`: a node is read back where every read it was made from is the same now); feature `testing`: `assert_incremental`, `assert_round_trips`, `assert_reopens`, `Shelf` |
| `core/geop-core-editor` | what an operation is and how it is edited (`PartOperation`, `EditForm`), what an editor draws (`view`), exporters and inspectors (`output`), the editor engine over a `Registry`; feature `testing`: an editor driven as a front end drives it |
| `lib/geop-lib-sheet` | the sheets drawings and flat patterns are written on: SVG, DXF |
| `lib/geop-lib-booleans` | booleans and the remesh under them, split, imprint, trim up to the next face, a step's `Combine` |
| `lib/geop-lib-sweep` | extrude, revolve, screw, sweeps along paths, lofts, the skinned grid blends share; feature `testing`: basic shapes |
| `lib/geop-lib-shell` | rebuilding a solid face by face (draft, faces moved, replaced, deleted and healed over), shelling, thickening and offsetting faces |
| `lib/geop-lib-fixtures` | test-only steps a program can hold: `Primitive` (a block, prism, cylinder or sphere at given coordinates, kept or combined) and `GivenDatum`; the stress sweeps' bodies; a dev-dependency only |
| `lib/geop-lib-stdlib` | the standard parts, one program per family, and `WithStandardParts`, the `FileStore` that puts them beside a workspace's files |
| `lib/geop-lib-constraints` | the constraints and unknowns of a part's state, and the solver over them |
| `ops/geop-ops-model` | the vocabulary every operation shares, the one `ops/` crate an operation depends on: the model target (topology with stable names, features, element kinds, threads, solid notes, `Combine`, `Extent`), the sketch and 3-D sketch targets and the references into them, the datums, a part's parameters and properties (their steps), what is picked (`Selected`, `MateEntity`), placed parts with their solids and bill of materials, `Spacing`; their drawers |
| `ops/geop-ops-sketch` | the sketch operation, its tools, snapping and references |
| `ops/geop-ops-sketch3d` | the 3-D sketch operation: wires in space, paths and rails |
| `ops/geop-ops-reference-geometry` | the datum operation: points, axes, planes and frames from what is picked |
| `ops/geop-ops-extrude-revolve` | extrude, revolve, sweep and loft; feature `testing`: the sketches and extrusions other crates' tests start from; feature `examples`: its examples |
| `ops/geop-ops-hole` | holes from ISO and NPT tables, threads on straight or tapered walls, cosmetic or modelled |
| `ops/geop-ops-fillet` | fillets and chamfers, rolling-ball blends of tangent chains |
| `ops/geop-ops-shell` | the shell operation |
| `ops/geop-ops-plastic` | ribs, lips and grooves, drafts |
| `ops/geop-ops-boolean` | the boolean and split operations |
| `ops/geop-ops-pattern` | patterns, mirrors and moves of bodies and features; bodies scaled, evenly or along each axis, and tapered, pinched, sheared, twisted or bent about an axis |
| `ops/geop-ops-subd` | subdivision surfaces: a control cage, edited with the box modelling tools (extrude, inset, bevel, loop cut, slide, merge, dissolve, bridge, fill, sharp and soft creases, a mirror) and built as its Catmull–Clark limit surface |
| `ops/geop-ops-surface` | boundary and UV surfaces, offset, thicken, knit, trim and extend |
| `ops/geop-ops-edit` | delete a body, extract a face, project a sketch onto a face; direct edits: move, offset (push-pull), replace and delete faces (healed over or left open), the solid rebuilt around them; faces on one surface merged |
| `ops/geop-ops-assembly` | placing parts, mates and joints, part patterns, the joints panel, the drag tool, the settler, STL and URDF export |
| `ops/geop-ops-harness` | wire harnesses routed through connectors and clips |
| `ops/geop-ops-step` | STEP import and export, assemblies as products and occurrences; STL meshes (ASCII or binary) imported as B-reps |
| `ops/geop-ops-drawing` | 2-D drawings: views, hidden lines, sections, dimensions, title block, parts list |
| `ops/geop-ops-sheetmetal` | flanges, bends, hems, cuts, the flat pattern and its DXF |
| `ops/geop-ops-inspect` | mass properties, measurements, interference |
| `ops/geop-ops-bom` | bills of materials, an inspector and a CSV exporter |
| `cad/geop-cad-base` | composition: `registry()`, one `register` line per crate, the editor of all of it beside the standard parts, and the composition's tests |
| `cad/geop-cad-web` | the wasm bindings the web app loads (package `geop`), and the check of `web/src/geop.ts` against the protocol |
| `cad/geop-cad-cli` | the `geop` command line: `compile`, `examples`, `serve`, `open`, `mcp`; `desktop` with the feature `desktop` |
| `cad/geop-cad-desktop` | the desktop app (`geop-desktop`, `geop desktop`): the editor in-process on a thread of its own, drawn with egui and a wgpu viewport; its controller runs without a window, and its tests drive both |

#### How an operation plugs in

An op crate plugs into the editor without the editor knowing it. It
exports one `pub fn register(r: Registry) -> GeopResult<Registry>` that
adds what it contributes to the `geop_core_editor::registry::Registry`,
and `cad/geop-cad-base/src/registry.rs` calls every crate's in turn, one
line each, in ribbon order. What a crate can add:

- an operation (`Registry::operation`): a `PartOperation` made an
  `Editable` by `part_operation!`, edited through its one `form`, its
  icon the SVG of `OperationInfo::icon`;
- a `Drawer`, which draws the targets the crate defines into the part's
  `PartView` and lists them in the structure tree (`StructureItem`, with
  the `kind_label` its group is shown under); placed parts are drawn as
  `ViewInstance`s of a component sent once, never a mesh per placement;
- an `Exporter` (a file written from the part) or an `Inspector` (a
  question about it, answered as an `output::Answer`);
- a `Panel`: program-level state sent under its key in
  `ProgramState::panels`, taking `Command::Panel { key, payload }` — the
  parameters (`parameters`) and the joints (`joints`);
- a `Tool`: a mode in hand while no step is edited, taken by its key with
  `Command::Tool { key }` — the drag tool (`drag`); the pick tool of an
  inspector that picks is `pick:<inspector>`. A tool is no operation: the
  front ends offer it beside the inspectors, never among the operations;
- the one `Settle`: the solve the editor runs after every edit, moving the
  program's state so its constraints hold — with none, state is kept as
  given;
- an `ExampleSet`: programs `Command::LoadExample` loads and workspaces of
  several files `Command::LoadWorkspaceExample` loads.

An op crate keeps all of itself: `src/lib.rs` is its public surface
(`register`, target types, reference constructors, `read_*`, `*Args`);
`src/examples.rs` its examples, Rust programs written against its `Args`
(or a program drawn in the editor, kept as its `.geop` text in
`src/drawn/` and read with `include_str!`); `src/program_tests.rs` builds
programs in a registry of the operations they need, each example with a
round trip (`assert_round_trips`) and an incremental build
(`assert_incremental`); `src/editor_tests.rs` drives
`geop_core_editor::editor::Editor` as the front end does
(`geop_core_editor::testing`: `editor_with`, `Stepping`, pointers and
clicks); `src/regression_tests.rs` rebuilds the user reproducers of its
bugs; `src/stress_tests.rs` holds ignored sweeps; feature `testing` what
other crates' tests need of it. An operation is a leaf: its dependencies
are `core/`, `lib/` and `geop-ops-model`. Its form says where its dialog
is shown (`Form::layout`): beside the viewport by default, or as a wide
popup over it (`Layout::Popup`) for a form of tables (`EditForm::table`,
whose every edit reaches the operation as one `TableEdit`); the front
ends draw both layouts from the same field components. Its examples (`src/examples.rs`)
are written in other operations' `Args` too, so they sit behind the
feature `examples`, those dependencies optional; `geop-cad-base` turns the
feature on, and so does the crate's own test build (`cfg(test)`) and any
test that reads its examples (`features = ["examples"]` on the
dev-dependency). An op crate whose tests need a crate that
depends on it puts those tests in `tests/` (as `geop-ops-sketch` does):
in `src/` Cargo would link two copies of it. `cargo test -p <crate>` must
pass on its own.

What a step draws on the part is its `geop_core_editor::view::annotations`
target, what it lists in the structure tree its
`geop_core_editor::view::listing` target. `geop compile` picks the exporter
by the output's extension and passes it the options given as
`-O key=value`. The web app builds its export menu, and the inspect panel's
buttons — the tools and the inspectors — from the lists the program state sends, shows every
inspector's answer as the inspector lays it out, groups the structure by
the kernel's labels, lights what the kernel names by the target, kind and
name of each element, and shows each panel by the component its key maps
to (`web/src/ProgramPanels.tsx`, the one place it names a panel; the
desktop app's is `cad/geop-cad-desktop/src/ui/panels.rs`): no front
end knows an operation, an exporter or an inspector by name. What a set
of picked elements lights of a drawn part is the kernel's
(`view::lit::Lit`), which parts a gizmo shows (`GizmoView::parts`) and
what the structure says beside a placed part (`ProgramState::details`)
too: a front end only draws them. The files an
operation reads beside programs — a STEP file to import — it names by
extension (`OperationInfo::reads`), and the front ends keep those in a
workspace.

The protocol types of the web app (`web/src/geop.ts`) are checked against
the kernel's: `cargo test -p geop protocol`
(`cad/geop-cad-web/src/protocol_tests.rs`) drives the editor by JSON,
requires the kernel to read every command sampled, and keeps what it
answers, reduced to its shapes, in `web/test/protocol.samples.ts`, which
`tsc -b` checks against `geop.ts`. A change to what the editor sends — a
field, a dialog's shape — fails that test until the samples are written
again (`GEOP_WRITE_PROTOCOL_SAMPLES=1 cargo test -p geop protocol`), and
the web build until `geop.ts` agrees.

STEP files are tested against a downloaded corpus of public files (NIST
MBE PMI models, the FreeCAD parts library, OCCT/CadQuery/build123d
samples): `ops/geop-ops-step/scripts/fetch_corpus.sh` fetches it into
`target/step-corpus/` (never committed), and
`cargo test -p geop-ops-step corpus -- --ignored --nocapture` imports every
file and prints what passed, what was healed and the causes of what failed
(`STEP_CORPUS_FULL=1` validates fully, `STEP_CORPUS_ONLY=<name>` imports
the files matching a name and prints their errors in full).

The web app draws placed parts batched per component — one
`InstancedMesh` each (`web/src/placed3d.ts`) — and takes the scene's
changes rather than the whole scene. Its section view (`section.ts`)
clips the part and the placed parts and caps what it cuts open from
stencil counters drawn with them.

Standard parts (ISO screws, nuts, washers, dowel pins, standoffs, ball
bearings, T-slot extrusions, a NEMA 17 motor, involute spur gears and
their rack, GT2 pulleys, shaft collars, flange couplings, MGN linear
guides, hobby servos) live in `geop-lib-stdlib`: one program per family,
generated in Rust, its sizes the rows of a `LookupTable` step looked up by
`size`. Every workspace reads them as read-only files named `std:…`
(`std:iso4032_hex_nut.geop`) through `WithStandardParts`, the one place
they come from; the editor's part picker lists them, and
`geop compile std:…` meshes one. Listing them builds nothing: a family's
program is generated the first time its file is read (`FAMILIES`).

End-to-end checks drive the real app in headless Chrome (`web/e2e/`, see
`web/README.md`): `npm run e2e` in `web/` loads every example from the
File menu, opens and cancels every operation, sketches and extrudes a
part by clicks, weighs it, and downloads every export; `npm run
e2e:vscode` opens generated example workspaces in the VS Code
extension's page against a real `geop serve` and checks that edits are
written back; `npm run demo` records the README's `docs/demo.gif` the same
way (a rectangle sketched, extruded, filleted and resized, a screenshot
per state, assembled by `ffmpeg`). Logic of the front end that needs no browser lives in a
module of its own and is unit tested with `npm test` in `web/` (Node's
own runner, `web/test/`), as the click and double-click rules are
(`web/src/gestures.ts`). A bug they find in Rust gets a Rust test as well,
in the crate of what failed.

### Continuous integration

`.github/workflows/ci.yml` runs on every pull request the layering check,
`cargo fmt --check`, `cargo clippy --workspace --all-targets -- -D
warnings`, `scripts/test-affected.sh` against `main`, and the web build and
tests; on a push to `main` the whole suite; and every night the whole
suite with the ignored sweeps (`cargo test --workspace -- --ignored`).
Every run is capped with `timeout` and `prlimit`, as `AGENTS.md` asks.

### Releases

`scripts/release.sh <version>` sets the workspace version, commits, tags
`v<version>` and pushes the tag (the VS Code extension's
`package.json` is set to the same version). The tag runs
`.github/workflows/release.yml`: it builds the two pages for a native
kernel once (`web` in modes `open` and `vscode`), the `geop` CLI with the
`open` page built in (feature `frontend`) for Linux (x86_64, aarch64
through `cross`), macOS (x86_64, aarch64) and Windows (x86_64), one VS
Code extension per platform with that platform's `geop` in its `bin/`
(`geop-vscode-<version>-<linux-x64|linux-arm64|darwin-x64|darwin-arm64|win32-x64>.vsix`),
zips the web app's static build (`web/dist`), writes `SHA256SUMS` and
publishes them as a GitHub release with generated notes. A version with
a `-` (`0.2.0-rc1`) is published as a pre-release.

Users install from a release with `homepage/install.sh` and
`homepage/install.ps1`, served at `geop-cad.dev/install.sh` and
`/install.ps1` (as text, see `homepage/_headers`) by the landing page's
deploy. Both, like `geop vscode`, refuse any file whose SHA-256 does not
match `SHA256SUMS`. To try them without a release, point
`GEOP_RELEASE_BASE_URL` at a folder laid out like one
(`file:///…/v<version>/SHA256SUMS`, …) and set `GEOP_VERSION`.

### The command line and MCP

`cad/geop-cad-cli` is the `geop` binary: `compile` builds a program and
writes it with an exporter, `examples` writes every built-in example,
`serve` runs the editor for a front end over stdin/stdout (the VS Code
extension), and `mcp` runs it as a Model Context Protocol server for an
agent — `claude mcp add geop -- geop mcp` or `codex mcp add geop -- geop mcp`, tools in `docs/mcp.md`, the
protocol in `src/mcp.rs`, the end-to-end session in `tests/mcp.rs`.

`geop open [DIR | program.geop]` serves the web editor to a browser, its
kernel the `geop` process and its workspace `DIR` (`src/open.rs`). The
page is the web app built with `npm --prefix web run build:open`
(`web/dist-open`, its backend `web/src/backend.local.ts`), embedded with
`cargo build -p geop-cad-cli --features frontend`; without the feature
`geop open` says how to build it. The browser speaks the protocol of
`geop serve` over a WebSocket (`/ws`, one command per message, answered
in order by one update; both run a `src/session.rs` `Session`), and
reads and writes the folder's programs and data files through
`GET /api/files`, `PUT`/`DELETE /api/file?path=` — written as they are
edited, a `.geopc` cache beside each like `serve --cache-root`. It binds
127.0.0.1 only; every request must give `127.0.0.1:<port>` or
`localhost:<port>` as its `Host` (no DNS rebinding) and, if it has an
`Origin`, that origin; the WebSocket and the file API take a token made
afresh each run and given only in the printed URL's fragment; and a file
path must be relative, of an extension the explorer shows, and resolve
(links followed) inside `DIR`. One editor session serves every
connection (`SessionHandle`): `geop mcp --attach` would be another sender
of commands to it.

`geop vscode [--editor CMD]` installs the extension of the CLI's own
version and platform from its release (`src/vscode.rs`, file names and
checksums in `src/release.rs`) with `code`, `cursor` or `codium`.

### Scripts

- `scripts/release.sh <version>` cuts a release (see Releases).

- `scripts/test-affected.sh [--dry-run] [base-ref] [-- args]` tests only
  what a change can affect: the files changed against `base-ref` (default
  `origin/main` when that ref exists, otherwise `HEAD~1`; uncommitted and
  untracked files count too) map to their crates, and those crates, every
  crate building on them, and every crate whose tests use them run
  `cargo test -p …` under `timeout 1800 prlimit --as=16000000000`. A change
  under `core/` or to the workspace's `Cargo.toml`/`Cargo.lock` runs the
  whole workspace; files in no crate (scripts, docs, `web/`) run nothing.
  `--dry-run` prints the crates; arguments after `--` go to `cargo test`.
- `scripts/check-layering.sh` checks the dependency rules between the
  tiers (`ARCHITECTURE_PLAN.md` §2.1): core depends on core only, and
  `geop-core-program` on `geop-core-math` only; `geop-ops-model` on core
  only; a `geop-lib-*` on core, `geop-ops-model` and libs; an op crate on
  core, libs and `geop-ops-model` only, another op reached only by an
  optional dependency of its `examples` or `testing` feature (printed as a
  warning, like a dev-dependency); no ops or lib crate on `geop-cad-*`; no
  source under `core/` names an ops, lib or cad crate; `geop-cad-base`
  names ops crates only in `src/registry.rs` and `src/tests.rs`. The
  allow-list `scripts/layering-allowed.txt` is empty: a violation fails
  the check.
- `scripts/baseline-<date>.txt` records the lines and tests per crate and
  the results of the fast suite, as its header says how; a change that
  loses tests says which and why.
