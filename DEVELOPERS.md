
Things I tried that didn't work:
- Geop 1: Using lines, arcs, spheres, tori, spirals etc. as primitives with explicit intersection formulas. Didn't scale as equations became too complex. Interval arithmetic was nice though and 2d booleans worked reliably.
- Geop 2: Went deep into algebra, symbolic math, ideals, Groebner bases, etc. to solve the intersection problems. Too slow and too much overhead. Decided to use only numerical methods from now on.
- Geop 3: Focused on getting the numerical methods right, first POCs for intersection of NURBS and nonlinear equation solving. Finally found something that could do the job. Decided to use dynamic ndarrays to represent multi-dimensional Bernstein polynomials from now on.
- Geop 4: Got stuck in numerical solvers and topology. Booleans looked promising but tracing was too difficult, too many edge cases.
- Geop 5: Based only on subdivion again, but instead of using convex hulls, approximate short segments with fat lines and fat surfaces. Interface for curve only provides subdivision, midpoint, tangent, and fat line approx. Interface for surfaces only provides subdivision, midpoint, normal, and fat surface approx. Intersection curves are never traced explicitly, they are just stored as a set of surface patches that can be subdivided further if needed. Intersections are trivial like this. Containment is done by ray shooting. We do not yet store any topology information, just a bag of surface bounded surface patches. The surface patches are also bounded by 3d curves, not in their parametric space. 

### Development

The kernel is a Cargo workspace of small crates, each depending only on the
ones before it; `web/` is a React front end that loads the top of that chain
compiled to WebAssembly and renders its output with `three.js`, and
`geop-cad-cli` puts the same chain on the command line.
`cloudflare-functions/` holds the Cloudflare Pages Functions deployed with
the web app (bug reports).

```
core/geop-core-math          scalars, interval arithmetic, linear algebra,
                              error type, frames, rays, datums, poses (a
                              unit quaternion and a position), constrained
                              least squares and dual numbers for the solvers,
                              plain-`f64` vectors for free choices (`plain`)
core/geop-core-geometry       NURBS curves/surfaces and their simplest
                               constructors (lines, arcs, bilinear and ruled
                               patches), helices, compatible
                               curves for lofts, Coons and Gordon patches,
                               containment, intersection
core/geop-core-topology       B-rep structures, Euler operators, edit/validation;
                               wires: edges and vertices bounding no face
core/geop-core-rasterize      turns a Model into a triangle mesh, writes it
                               as STL, and renders it for debugging
core/geop-core-sketch         2-D constraint sketches: entities, constraints
                               as residuals, profile extraction, geometry
                               made from geometry (mirror, patterns, offset),
                               how a dimension is drawn in a plane
core/geop-core-program        what a program is (`Program`: steps and state,
                               ids, copy and paste, checked against its
                               operations), the values it is run with
                               (`Design`, `Setting`, `StateValue`), the
                               part it builds: a memo table of nodes, one
                               per step and target, each made by a plain
                               make and reused while its arguments are
                               equal and what it read holds (`Part`,
                               `StepContext::produce`, `Reader`, `Read`),
                               chain targets several steps produce, typed
                               references to their elements
                               (`ElementReference`), placed parts — built
                               from their program, inputs and state slice,
                               equal builds shared (`placed`) — the
                               operations a program is written in, running
                               it up to a step (`Runner`), the workspace of
                               files it reads (`Files`, `FileStore`), and
                               numbers as typed or formulas of its
                               parameters (`NumericInput`,
                               `ParameterValues`); feature `testing`:
                               `assert_incremental`
core/geop-core-editor         what an operation is and how it is edited
                               (`PartOperation`, `EditForm`): events,
                               dialogs, visuals, hit tests, the gizmo every
                               drag in space goes by; picking typed
                               references (`Selectable`, `one_of!`); the
                               field of a number typed or a formula
                               (`EditForm::numeric`, `Number::numeric`);
                               what an editor draws of a part (`view`:
                               `PartView`, picking by drawn kind,
                               annotations and the structure entries steps
                               list); the exporter and inspector interfaces
                               (`output`); the whole editor engine — one
                               step edited (`step_editor`), the program
                               edited by `Command`s answered by `Update`s
                               (`editor`), the pick tool (`tools::pick`)
                               — over a `Registry` of what it offers
                               (operations, `Drawer`s, exporters,
                               inspectors, `Panel`s, `Tool`s, one `Settle`,
                               `ExampleSet`s), none of which it names;
                               feature `testing`: an editor over a
                               registry of a few operations, driven as a
                               front end drives it (`editor_with`,
                               `Stepping`, pointers, clicks)
lib/geop-lib-sheet            the sheets drawings and flat patterns are
                               written on (SVG, DXF, each layer defined by
                               who draws on it), and `xml_escape`
ops/geop-ops-model            the model a part's steps build: topology with
                               stable names, the features their steps
                               combined, geometric kinds of its elements
                               (point, line, plane, round, curve, body),
                               cosmetic threads, solid notes; how it is
                               drawn. Its names are built from step ids in
                               the program's id alphabet
ops/geop-ops-parameters       a part's parameters (numbers, tables, colour,
                               material) as steps defining them
                               (`DefineParameters`, `LookupTable` for a part
                               family's table, `PartProperties`), read,
                               set and renamed in a `Program`
                               (`parameters_of`, `set_parameters`,
                               `rename_parameter`, `parameter_uses`); where
                               placed parts are (`Instance`, `placement`,
                               `located`); the editor's parameters panel,
                               and the part drawn in its colour
ops/geop-ops-constraints      the constraints and unknowns of a part's state
                               (mates, joints) — its own and its instances',
                               behind their names — and the solver
                               re-running the program over them
ops/geop-ops-datums           the part's datums: the origin, and those steps
                               add, picked as points, axes and planes,
                               drawn and listed
ops/geop-ops-sketch           the sketch operation: drawing and constraint
                               tools, snapping, reference geometry and
                               projections of the part
ops/geop-ops-add-datum        the datum operation: reference points, axes,
                               planes and coordinate systems built from what
                               is picked
ops/geop-ops-booleans         3-D boolean operations (union/intersection/diff),
                               the boolean operation
ops/geop-ops-edit             edits of existing bodies: delete a body,
                               extract a face, project a sketch onto a face
ops/geop-ops-extrude-revolve  extrude/revolve; sweeps along paths, with guide
                               rails, twist and orientation; lofts, with
                               guide curves and matched points; their
                               operations, and basic shapes for tests
ops/geop-ops-sketch3d         the 3-D sketch operation: points, lines, arcs
                               and splines in space, placed on the part,
                               snapped along x, y, z and onto points, and
                               constrained — built as a wire whose edges and
                               vertices are picked like any; paths and rails
ops/geop-ops-fillet           fillets and chamfers on straight and circular
                               edges, rolling-ball fillets of any other edge
                               and tangent chain, radius varying along them,
                               cut or filled in with a boolean
ops/geop-ops-shell            shelling: a solid hollowed to walls of one
                               thickness, open at picked faces, the shell
                               operation
ops/geop-ops-pattern          linear and circular patterns, mirrors and
                               moves of bodies, copied as new bodies or
                               combined with a solid; patterns and mirrors
                               of features (a cut, a boss, a hole and its
                               thread) from the tools their steps recorded;
                               their operations
ops/geop-ops-hole             holes from ISO tables (simple, counterbore,
                               countersink, tapped) and threads, cosmetic or
                               modelled along a helix; the hole and thread
                               operations
ops/geop-ops-plastic          housings: ribs grown up to the walls, lips and
                               grooves along a rim, drafts on planar faces
ops/geop-ops-surface          surfacing: boundary (ruled, Coons, filled)
                               and UV (Gordon, through a network of
                               curves) surfaces, offset, thicken, knit,
                               trim and extend of faces standing on their
                               own
ops/geop-ops-drawing          2-D drawings: projected views with hidden
                               lines and silhouettes, sections, title block,
                               bill of materials, SVG and DXF; dimensions,
                               notes and centre marks picked on the sheet
                               (drawn like a sketch's, `geop_core_sketch::
                               dimension`); the drawing step
ops/geop-ops-subd             subdivision surfaces: a control cage shaped in
                               the editor, built as its Catmull-Clark limit
                               surface, a solid of B-spline faces; the subd
                               operation
ops/geop-ops-sheetmetal       sheet metal: base and edge flanges with bends,
                               reliefs and open or closed corners, hems, cuts
                               across bends unrolled, the flat pattern and its
                               DXF for cutting (an exporter); the sheet model
                               recorded as a target of the part, laid out
                               flat and folded into it
ops/geop-ops-harness          wire harnesses: wires routed through connectors
                               and clips as lines and arcs, bend radius
                               checked, the bundle swept, cut lengths; the
                               route operation
ops/geop-ops-assembly         the part operation: place another file's part,
                               mate it, joint it, drag it; patterns of
                               placed parts; the library parts are placed
                               from, the solids a part shows wherever they
                               are placed, and the STL export; placed parts
                               drawn as instances of their components, the
                               joints panel, the drag tool, and the settler
                               solving the mates after every edit
ops/geop-ops-inspect          inspecting a part without changing it: mass
                               properties of its solids and placed parts,
                               measurements of picked entities, interference;
                               inspectors
ops/geop-ops-urdf             URDF export of an assembly: the tree of links
                               and joints, mimics, inertia, STL meshes, a
                               stored ZIP writer; an exporter (`geop compile
                               robot.geop -o robot.zip`)
cad/geop-cad-base             composition: what the editor offers, one
                               `register` line per crate (`registry.rs`),
                               an editor of all of it (`editor()`), the
                               workspace of the editor (the core's `Files`
                               beside the standard parts), example
                               programs, the standard parts (below)
cad/geop-cad-web              the wasm bindings the web app loads (crate `geop`)
cad/geop-cad-cli              the `geop` command-line tool; `compile` meshes
                               each placed component once
ops/geop-ops-step             STEP (ISO 10303-21) exchange: reading the
                               B-rep solids and sheets of a file into exact
                               NURBS bodies (an assembly flattened), healed
                               where the file disagrees with itself, writing
                               a part as one — an assembly as products and
                               occurrences; the import operation
ops/geop-ops-bom              bills of materials: placed parts grouped by
                               file and parameter values, flat or indented,
                               with designations (a part's own, see
                               `Parameters::designation`), materials, masses, sheet
                               thickness and wires cut to length, as the
                               steps noted them on their solids;
                               an inspector, and a CSV exporter
```

An operation crate plugs into the editor without the editor knowing it:
it exports one `pub fn register(r: Registry) -> GeopResult<Registry>`
that adds what it contributes to the `geop_core_editor::registry::Registry`,
and `geop_cad_base::registry` calls every crate's in turn. What a crate
can add:

- an operation (`Editable`, usually a `PartOperation`);
- a `Drawer`, which draws the targets it defines into the part's
  `PartView` and lists them in the structure tree (`StructureItem`, with
  the `kind_label` its group is shown under); placed parts are drawn as
  `ViewInstance`s of a component sent once, never a mesh per placement;
- a `geop_core_editor::output::Exporter` (a file written from the part)
  or `Inspector` (a question about it);
- a `Panel`: program-level state the editor sends under its key in
  `ProgramState::panels`, and the commands it takes as
  `Command::Panel { key, payload }` — the parameters (`parameters`) and
  the joints (`joints`);
- a `Tool`: a mode in hand while no step is edited, taken in hand by its
  key with `Command::Tool { key }` and offered as its `ToolInfo` says in
  `ProgramState::tools` — the drag tool (`drag`); the pick tool of an
  inspector that picks is `pick:<inspector>`;
- the one `Settle`: the solve the editor runs after every edit, moving
  the program's state (placed parts) so its constraints hold — with none,
  the state is kept as given;
- an `ExampleSet` of programs `Command::LoadExample` loads.

What a step draws on the part is its `geop_core_editor::view::annotations`
target, what it lists in the structure tree its
`geop_core_editor::view::listing` target. `geop compile` picks the exporter
by the output's extension and passes it the options given as
`-O key=value`. The web app builds its export menu, inspect buttons and
tool buttons from the lists the program state sends, shows every
inspector's answer as the inspector lays it out (`Inspector::show`, an
`output::Answer` of text, lists, tables, groups and buttons), groups the
structure by the kernel's labels, and shows each panel by the component
its key maps to (`web/src/ProgramPanels.tsx`, the one place it names a
panel): no front end knows an operation, an exporter or an inspector by
name. The files an operation reads beside programs — a STEP file to
import — it names by extension (`OperationInfo::reads`), and the front
ends keep those in a workspace.

An op crate's tests can drive the editor the way the front end does over
a registry of only the operations they need: `geop_core_editor::testing`
(feature `testing`) has `editor_with(registry, program)`, `Stepping` and
the pointer and click helpers.

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
guides, hobby servos) live in
`geop-cad-base/src/stdlib`: one program per family, generated in Rust,
its sizes the rows of a table parameter `size`. Every workspace reads them
as read-only files named `std:…` (`std:iso4032_hex_nut.geop`) through
`stdlib::WithStandardParts`, the one place they come from; the editor's
part picker lists them, and `geop compile std:…` meshes one. Listing them
builds nothing: a family's program is generated the first time its file
is read (`stdlib::FAMILIES`).
End-to-end checks drive the real app in headless Chrome (`web/e2e/`, see
`web/README.md`): `npm run e2e` in `web/` loads every example from the
File menu, opens and cancels every operation, sketches and extrudes a
part by clicks, weighs it, and downloads every export; `npm run
e2e:vscode` opens generated example workspaces in the VS Code
extension's page against a real `geop serve` and checks that edits are
written back. Logic of the front end that needs no browser lives in a
module of its own and is unit tested with `npm test` in `web/` (Node's
own runner, `web/test/`), as the click and double-click rules are
(`web/src/gestures.ts`). A bug they find in Rust gets a Rust test as well
(see `set_tests.rs`, `editor_tests.rs`).

### Scripts

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
  `geop-core-program` on `geop-core-math` only; a `geop-lib-*` on core,
  `geop-ops-model` and libs; no ops or lib crate on `geop-cad-*`; no source
  under `core/` names an ops, lib or cad crate; `geop-cad-base` uses ops
  crates only in `src/registry.rs`. Violations known today are listed in
  `scripts/layering-allowed.txt`; a new one fails the check, and one that
  is gone is reported so its line can be deleted. Nothing adds lines to
  that list.
- `scripts/baseline-2026-10.txt` records the lines and tests per crate and
  the test results before the refactor; a phase that loses tests says
  which and why.
