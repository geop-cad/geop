# Architecture plan: a small trusted core, independent operation crates, a thin CAD shell

This is the plan for turning the workspace into an architecture where

- `geop-core-*` is small, trusted, and defines *what things are*: math,
  geometry, topology; a program, an operation, a part, a workspace; and how
  a step is edited;
- every operation lives in a crate of its own that defines its targets, how
  other operations retrieve them, how it is edited, and carries **all** of
  its tests, examples and regression tests;
- `geop-cad-*` is a composition point and a transport layer: it lists the
  crates, hosts the wasm and CLI bindings, and contains no operation
  specific code and no operation specific tests;
- changing an operation requires testing only that crate and the crates
  that depend on it (`cargo test -p <crate>` plus reverse dependencies),
  never the whole workspace.

Backwards compatibility is not a constraint. Saved `.geop` files, crate
names, module paths and APIs change where it makes the result simpler.

The plan has three parts: what is wrong today (§1), the target architecture
(§2 to §6), and the migration phases written so that each can be handed to
an agent (§7 to §9).

---

## 1. Where we are

### 1.1 Sizes (October 2026)

| tier | crate | lines | tests |
|---|---|---|---|
| core | geop-core-math | 12 270 | 150 |
| core | geop-core-geometry | 14 779 | 185 |
| core | geop-core-topology | 11 549 | 79 |
| core | geop-core-sketch | 8 686 | 61 |
| core | geop-core-program | 3 473 | 36 |
| core | geop-core-editor | 4 837 | 23 |
| ops | geop-ops-model | 3 843 | 15 |
| ops | geop-ops-booleans | 8 094 | 83 |
| ops | geop-ops-extrude-revolve | 10 760 | 99 |
| ops | geop-ops-step | 9 195 | 27 |
| ops | geop-ops-drawing | 7 518 | 14 |
| ops | geop-ops-fillet | 6 922 | 41 |
| ops | geop-ops-sketch | 6 376 | 8 |
| ops | geop-ops-sheetmetal | 6 356 | 15 |
| ops | geop-ops-assembly | 5 886 | 19 |
| ops | 22 further ops crates | ~43 000 | ~310 |
| cad | geop-cad-base | **26 504** | **345** |
| cad | geop-cad-web + geop-cad-cli | 870 | 1 |
| web | web/src (TypeScript) | 7 670 | |

### 1.2 What blocks the goal

1. **`geop-cad-base` is the largest crate and most of it is operation
   specific.** Of its 26 500 lines, the genuine "base" is the editor engine
   (`editor.rs` 1 770, `step_editor.rs` 1 130, `inspect.rs`, `workspace/`):
   about 4 000 lines. The rest is operation specific test code (17 files,
   ~15 000 lines, 345 tests: sketch, assembly, drawing, urdf, bom, harness,
   sheetmetal, step, stress), example programs (`examples.rs`, 2 600) and
   the standard parts library (`stdlib/`, 3 800). A change to the fillet
   dialog rebuilds and retests all of it.

2. **The editor engine hard-wires the assembly solver and the parameters
   panel.** `editor.rs` references `geop_ops_assembly::solving` (settle,
   drag tool, joints, freedom, conflicting) and `geop_ops_parameters`
   (materials, parameter resolution, rename) in 82 places.
   `ProgramState` carries `joints`, `freedom`, `conflicting`, `materials`,
   `parameters`, `parameter_uses`, `drag_tool` as fixed fields; the web app
   has bespoke `JointsPanel.tsx` and `ParametersPanel.tsx` for them. A new
   operation that needs a program level panel or a tool has to edit the
   engine.

3. **Operation crates depend on each other for the wrong reasons.** The
   import survey (non-test code) shows four kinds of unwanted fan-in:
   - *shared vocabulary*: `geop_ops_booleans::{Combine, Extent, solid}`
     is imported by extrude-revolve, fillet, hole, pattern, plastic, shell,
     subd, edit, surface, harness. `Combine` is "how a tool meets the
     solid", not the boolean algorithm.
   - *algorithms living in operation crates*: the boolean algorithm
     (`boolean`, `remesh`, `split`, `imprint`, `trim`, `naming`, ~7 000
     lines), extrude-revolve's `skin`, `sweep`, `extrude`, `revolve`,
     `shape_loops` (used by fillet, hole, sheetmetal, plastic, surface,
     edit, rasterize), `shell::shell`/`offset_faces`/`thicken` (used by
     plastic, surface). Changing the *dialog* of Extrude retests Fillet.
   - *test fixtures*: `geop_ops_extrude_revolve::{shapes, testing}` is a
     dev-dependency of nearly every crate. Acceptable as dev-dep, but
     `shapes` is geometry and belongs below the operations.
   - *type aliases*: every crate imports
     `geop_ops_parameters::{Setting, BuildPart, Runner, PartReader,
     StateValue, StepContext}` only to name the concrete part type. The
     concrete input, state and design-scalar types are the program's and
     belong in the core.

4. **The core cannot own what a program is, because the concrete types are
   in ops crates.** `geop-core-program` is generic in `<I, S>`; the real
   `Setting`, `StateValue`, `ProgramFile` (steps + state, excerpt, paste,
   fresh ids) live in `geop-ops-parameters`; `Design = ScalInF64` and
   `validate_operation_id` (the id alphabet every stable name relies on)
   live in `geop-ops-model`; the file-backed `Workspace`, `resolve`,
   `relative`, `is_shared` live in `geop-cad-base`. The file format, the id
   rules and the workspace are not operation concerns.

5. **`geop-ops-model` depends on the viewport.** It imports
   `geop-ops-rasterize` and `geop-ops-view` to draw itself. The model is
   the base of every operation; the view is a presentation concern.
   Rasterization is a kernel algorithm over `Model`.

6. **Composition is by hand.** `geop-cad-base/src/part.rs::structure`
   lists datums, sketches, solids, faces, placed parts explicitly, and
   `geop_ops_assembly::draw` composes the drawing. Adding a new kind of
   target means editing `cad-base`.

7. **Editor-driven tests can only be written in `cad-base`**, because the
   `Editor` type lives there. That is why `editor_tests.rs` (69 tests),
   `sketch_editor_tests.rs` (43), `assembly_tests.rs` (31) pile up there.
   AGENTS.md rightly demands editor-driven tests for every operation; the
   layout makes them land in the wrong crate.

What is already right and must be kept: the `Part`/`Target`/`Reader`
dependency tracking with fn-pointer generators, typed `ElementReference`
and `HasElements`, `PartOperation` + `EditForm` with setters, `Selectable`
and `one_of!`, the `Exporter`/`Inspector` registries, the `Answer` block
vocabulary that keeps the front end ignorant of inspectors, instances as
sub-parts with state behind their path, and the testing feature pattern
(`geop_ops_model::testing`, `geop_ops_extrude_revolve::testing`).

---

## 2. Target architecture

### 2.1 Tiers and the dependency rule

```
core/   geop-core-math        scalars, intervals, linear algebra, error type, frames, poses
        geop-core-geometry    NURBS curves/surfaces, intersection, containment
        geop-core-topology    B-rep, Euler operators, validation
        geop-core-sketch      2-D constraint sketches (pure geometry, no program notion)
        geop-core-rasterize   Model -> triangles, STL bytes             (moved from ops)
        geop-core-program     what a program, operation, part, workspace is (§3)
        geop-core-editor      what editing a step and a program is (§4)

ops/    geop-ops-model        the model target, stable names, element kinds, drawing of
                              solids, and the solid vocabulary every feature shares
        geop-lib-*            shared algorithm libraries over the model, no operations
        geop-ops-*            one crate per operation family: targets, retrieval,
                              editing, tests, examples, regression tests

cad/    geop-cad-base         composition: the list of crates, nothing else
        geop-cad-web          wasm bindings
        geop-cad-cli          command line
```

Dependency rules, enforced by `scripts/check-layering.sh` (§8):

- `core/*` depends only on `core/*`. `geop-core-program` depends on
  `geop-core-math` only (error type, poses); `geop-core-editor` on
  `geop-core-math` and `geop-core-program`. Nothing in `core/` names an
  operation, a target type of an operation, or a crate outside `core/`.
- `geop-lib-*` depends on `core/*` and `geop-ops-model`, never on a
  `geop-ops-*` crate other than model. **A lib takes geometry and models;
  an op resolves references.** An algorithm takes `Profile`s, `Model`s,
  `NamedModel`s, `Pose`s, never a `PlacedSketch` or an `ElementReference`.
- `geop-ops-*` depends on `core/*`, `geop-ops-model`, `geop-lib-*`, and on
  other `geop-ops-*` crates **only to read their targets** (the sketch
  target to extrude it, the datums target to pick a plane). Depending on
  another op crate for its `Args` type, a helper, or a test fixture is a
  layering error; the thing being imported belongs in a lib crate or in
  model.
- `cad/*` depends on everything and defines nothing but composition.
- `web/` knows the protocol types of `geop-core-editor` only. It maps
  panel keys to components; it never branches on an operation kind.

### 2.2 Size targets

| crate | today | target |
|---|---|---|
| geop-core-program | 3 473 (1 900 without tests) | ≤ 3 500 with its tests, ≤ 2 500 without |
| geop-core-editor | 4 837 | ≤ 10 000 (absorbs view ~1 100, step editor ~1 100, editor engine ~1 500, output traits ~500) |
| geop-cad-base | 26 504 | ≤ 400 |
| any geop-ops-* | — | no upper bound, but its tests live in it |

---

## 3. `geop-core-program`: what a program, operation, part and workspace is

### 3.1 Decisions

**D1. The core owns the concrete value types, and the generics go.**
`Part<I, S>`, `Reader<'a, I, S>`, `ProgramRunner<I, S>`,
`OperationRegistry<I, S>`, `Context<'a, I, S, N>` become non-generic. The
core defines:

```rust
/// The scalar a program holds its design data in.
pub type Design = geop_core_math::scalars::ScalInF64;

/// What a run is given by name, up front: a parameter's setting.
pub enum Setting { Number(f64), Text(String) }

/// What the state holds by name: what a solve moves between runs.
pub enum StateValue { Number(Design), Pose(Pose<Design>) }
```

Why: the file format is `{steps, state}`, and the state's value type *is*
the file format; the inputs are what one program hands another when it
places it. Both are program concepts, not operation concepts. Every op
crate currently carries five aliases to say `Part<Setting, StateValue>`;
removing the generics removes that noise from ~30 crates and lets the core
own `Workspace`, which has to deserialize state. Cost: a new kind of state
value is a core change. That is correct: it is a file format change.

(Alternative kept in mind: keep `<I, S>` and add a tiny `geop-core-values`
crate. Rejected: one more tier for two enums.)

**D2. One `Program` type.** `Program { steps: Vec<Step>, state:
BTreeMap<String, StateValue> }` replaces `Program` + `ProgramFile` +
`Loaded`. `excerpt`, `paste`, `fresh_id`, `index_of`, `validate` (ids
unique and in the id alphabet, every operation known to a registry, every
step's args deserialize) move into the core. `parameters_of`,
`set_parameters`, `parameter_uses`, `rename_parameter` stay in
`geop-ops-parameters` as free functions over `&Program` / `&mut Program`:
they read steps of two particular operations.

**D3. The id alphabet is the core's.** `validate_operation_id` moves from
`geop-ops-model::names` to `geop-core-program::program`. The model's
`Namer` imports it.

**D4. The workspace is the core's.** `Workspace` trait, the file-backed
implementation (`FileStore`, `FileStoreMut`, per-file caching, revision
counter), `Location`, `resolve`, `relative`, `is_shared` move from
`geop-cad-base/src/workspace` into `geop-core-program::workspace`. The
`std:` scheme stays a generic "shared library" scheme; the standard parts
`FileStore` decorator moves to `geop-lib-stdlib`.

**D5. Numbers as formulas are a program concept.** `NumericInput` ("a
number as typed, or a formula of the part's parameters") and the
`ParameterValues` target type with `read_parameter_values` move from
`geop-core-editor::numeric_input` / `geop-ops-parameters::steps` to
`geop-core-program::numeric`. The step that *defines* parameters
(`DefineParameters`) stays an operation in `geop-ops-parameters`. This is
the same pattern as the datums' origin: the core knows the target type and
its default when no step defines it; an op defines it.

### 3.2 The run model (D6 to D10): one kind of node, one rule for reuse

The current runtime has more concepts than it needs: step *positions* as
identity, two revisions per part (`run` and `built`) plus `carry`, a
`Same` flag, `RevisionedValue` for inputs and state, `define_target`
versus `revise_target`, the `sees` rule, a `Kept` marker target per
instance, `load_subpart` with its own run-revision coupling, and
`Reader::inside`. It is hard to say when something runs. The replacement
is a memo table, in the style of a build system, with one kind of node.

**D6. A run is cheap declarations plus memoized computations.** A run
walks the program's steps in order and calls each operation's `run`. That
body must be cheap and pure in the sense that it only *declares* what the
step produces:

```rust
ctx.produce(&target_ref, args, make)   // make: fn(&Reader, &Args) -> GeopResult<Out>
```

`produce` computes the node at once (so a later node of the same step can
read it) unless it can be reused. A step runs no geometry outside a
`make`. The runner records nothing else. So the answer to "when does this
run?" is always: a `make` runs when its node is new, its arguments
differ, or something it read changed; nothing else ever runs.

**D7. Nodes are keyed by step id and target name, not by position.** A
node is `(StepId, TargetName)`. The producer a reader sees for target `T`
is the latest step *in the current program's order* before the reading
step that produces `T`. A step that produces a target an earlier step
already produces revises it: its `make` gets the earlier version through
`reader.previous::<T>()` (or `T::default()` if there is none). There is
one `produce`; "define" versus "revise" is not a distinction the API
makes. Whether a target type may have several producers is a property of
the type (`trait Target { const CHAIN: bool; }`, true for the model and
other accumulated targets, false for a sketch): a second producer of a
non-chain target is a validation error naming both steps. Inserting,
removing or reordering steps changes which producer a reader sees, and
that is caught by the dependency check, not by positions shifting.

**D8. Reuse is a check of recorded reads against the present.** Every
read through a `Reader` is recorded on the node being made:

```rust
enum Read {
    Target { name, producer: StepId, version: u64 },  // which version of which producer
    Absent(name),                                     // that no producer was visible
    Input(name, Option<Setting>),                     // the value read, itself
    State(name, Option<StateValue>),                  // the value read, itself
    File(path, revision: u64),                        // a workspace file
    Inside { instance: StepId, read: Box<Read> },     // a read inside a placed part
}
```

A node is reused iff its arguments are equal (`PartialEq`) and every
recorded read resolves to the same thing now: the same producer is
visible and its node has the same `version`; the name is still absent;
the input or state value is equal; the file revision is equal; the inner
read holds inside the placed part. Otherwise `make` runs again. A node's
`version` is bumped only when the new value differs from the old by
`PartialEq`, so an unchanged rebuild leaves dependents reusable. Inputs
and state carry no revisions: recording the value read is simpler and
exact.

Values are stored as `Rc<dyn Any>`, so a node's value, a reader's borrow
and a cache entry share one allocation.

**D9. A placed part is an ordinary node whose value is a built `Part`,
and equal builds are shared.** `AddPart` does
`ctx.produce(&placed("bolt"), PlacedArgs { file, inputs }, build_placed)`
where `build_placed` is a core-provided `make`. It reads the program file
through the reader (`Read::File`), the state slice under the instance's
path (`bolt/…` as `Read::State`), and asks the workspace's **build
cache** for the part:

```
key = (program: Rc<Program> identity + content hash, inputs, state slice)
cache: HashMap<key, Rc<Part>>   kept on the Workspace, swept at the end of each run
```

- A thousand screws of size M4 with no state have one key: one build,
  a thousand `Rc` clones. Nothing per instance.
- A sub-assembly with internal state (a jointed arm placed twice at
  different angles) has a key per distinct state slice. When the solver
  moves one arm's joint, the new key misses the cache; the build starts
  from a clone of that instance's previous `Part` (its memo table, cheap
  because values are `Rc`) and recomputes only the nodes that read the
  moved state. The result is a new immutable `Rc<Part>` under the new
  key; the old one is dropped at the sweep unless another instance still
  uses it.
- A part's state behind an instance path lives in the parent's state map
  (`bolt/b.pose`) as today; the placed node reads exactly the slice it
  passes down, so a change anywhere else in the parent's state touches no
  placed node.

Reading an element of a placed part (`/bolt:model`) goes through the
placed node's value and records `Inside { instance, Target{...} }`, so a
mate that reads the arm's second link is stale only when that link's
model version changed, not when anything in the arm changed.

Cycle detection stays: the build context carries the stack of files being
built, and a file placing one on the stack is refused with the chain.

This removes `Kept`, `subpart_target`, `load_subpart`, `Subpart`,
`Built`, `Same`, `carry`, `last_built`, `begin_run_as`, `run_as`, the
`built` revision, `Reader::inside`, and `Dependency::behind`. The
TODO in `part.rs` about sharing equal instances is the design, not a
future optimization.

**D10. The editor's part is one memo table, run to a marker.**
`runner.run(part, program, upto: Option<StepId>)` runs the steps before
the marker; nodes of steps after it are kept for when they run again;
nodes of steps no longer in the program are dropped at the end of the
run. `part.before(step_id)` gives the reader a step is edited against,
`part.current()` the finished part. Failed nodes hold `Err(message)`;
a node reading one fails with that message in context. The solver moves
state values and re-runs: only nodes whose recorded `Read::State` differ
recompute, inside placed parts too.

### 3.3 Module layout

```
geop-core-program/src
  lib.rs
  values.rs        Design, Setting, StateValue, Settings, StateValues
  program.rs       Program { steps, state }, Step, StepId, ids (alphabet, uniqueness, fresh_id),
                   excerpt/paste, validate(&registry): unknown op, bad id, args, chain rule
  operation.rs     Operation, StepArgs, ErasedOperation; OperationRegistry
  target.rs        Target trait (CHAIN), TargetReference, Node, Read
  part.rs          Part (memo table), produce, reuse check, sweep, before()/current()
  reader.rs        Reader: get, previous, retrieve_element, retrieve_placed, input, state,
                   file, program, targets_of, failed_targets_of; recording
  element.rs       ElementReference, HasElements, Kind, PickedElement (unchanged)
  placed.rs        placed node: PlacedArgs, build_placed, instance paths, build cache key
  runner.rs        Runner: registry, workspace, file stack; run(part, program, upto)
  workspace.rs     Workspace trait, Files (file-backed impl), FileStore, Location,
                   resolve/relative/is_shared, the build cache
  numeric.rs       NumericInput, ParameterValues target, formula evaluation
  testing.rs       feature `testing`: run steps by hand, assert_incremental
```

### 3.4 Invariants the core enforces

- A node is produced once per run; a non-chain target has one producer;
  a chain target's producers are visible in program order.
- `make` is a plain `fn`: a node depends only on its arguments and its
  recorded reads. A read not through the `Reader` cannot happen because
  the `Reader` is the only access to the part.
- Reuse iff arguments equal and every recorded read holds (D8).
- Step ids unique and in the id alphabet; files never place themselves.
- A placed part is built from exactly (program, inputs, state slice):
  equal keys share one build.
- `Program::validate(&registry)` at load, paste and commit.
- `testing::assert_incremental(registry, workspace, programs)`: runs an
  edit sequence incrementally on one part and each program from scratch,
  and asserts every visible node value equal. Every op crate runs it on
  its examples. This is what lets caching be trusted across crates that
  never see each other.

### 3.5 Inputs, strings and part families

Inputs are `Setting::Number` or `Setting::Text`. A part family is a
program whose sketches read `size.d`, `size.p` and the like, and whose
placer passes `size = "M4"`. Today that is `ParameterKind::Table` inside
`DefineParameters`. It becomes an operation of its own in
`geop-ops-parameters`: `LookupTable { input: "size", columns, rows }`
produces entries of the `ParameterValues` target (`size`, `size.d`,
`size.p`, …) from the row the input names, and fails naming the input and
the rows if the text matches none. `DefineParameters` keeps numbers as
formulas. The core knows only that an input is a number or a text and
that `ParameterValues` is a chain target several steps may add to.

### 3.6 Worked examples of what runs

- *Edit a fillet radius in a 30-step part.* The fillet node's arguments
  differ: its `make` runs. Every later node that read `model` sees the
  same producer with a new version: their `make`s run. Sketches, datums
  and nodes before the fillet are reused. Placed parts are untouched.
- *Drag a joint of a sub-assembly placed in a robot with 2000 parts.* The
  solver changes `arm/j1.angle` in the parent's state. The `arm` placed
  node's `Read::State` differs: its build cache key misses; the arm's
  part is cloned and only its nodes reading `j1.angle` recompute. Mates
  in the robot that read `arm/link2/model` see a new inner version and
  recompute; the other 1999 placed nodes read state slices that did not
  change and are reused without being looked at beyond the check.
- *Place the 1000th screw.* `produce` with the same key as the other 999:
  cache hit, one `Rc` clone, no build.
- *A cable follows the assembly.* A sketch in the assembly file projects a
  vertex of `arm/link2` (resolved with `retrieve_placed`, so the sketch
  crate must accept placed elements where it projects) and a sweep runs a
  cable along it. The sketch node recorded `Inside { arm, link2/model }`
  and the poses along the path; a drag changes those reads, the sketch
  recomputes, the sweep follows. Geometry *driven* by state is just a node
  that read state. Geometry that the solver then constrains against would
  close a loop whose derivative the solver cannot see: a constraint whose
  residual reads a target that itself read state is refused by name.
- *Reorder two steps.* The nodes of both are found by id; the reads whose
  visible producer changed fail the check; everything else is reused.

### 3.7 What the core deliberately does not do

- It does not know the model, sketches, datums, or any target type. Those
  are ops.
- It does not solve state. It offers a hook (§4.4) and the `StateValue`
  type.
- It does not draw. Drawing vocabulary is the editor's (§4).

## 4. `geop-core-editor`: what editing is

### 4.1 Scope

Today: the editing vocabulary (dialog, events, visuals, gizmo, hit
testing, forms), `PartOperation`, `Editable`, `Editors`, `Selectable`,
`one_of!`.

Target: additionally the **whole editor engine** and the **viewport
model**, generic over registered extensions, so that an operation crate
can instantiate an `Editor` with a registry of three operations and drive
it in its own tests.

Moves in:

| from | to |
|---|---|
| `geop-cad-base/src/step_editor.rs` | `geop-core-editor::step_editor` |
| `geop-cad-base/src/editor.rs` (Command, Update, ProgramState, SceneState, StepState, undo/redo, marker, builds cache, run_to, shown_part) | `geop-core-editor::editor`, with the assembly and parameters specifics removed behind hooks (§4.4) |
| `geop-cad-base/src/inspect.rs` (PickTool) | `geop-core-editor::tools::pick` (it is generic over inspectors) |
| `geop-ops-view` (PartView, pick, ViewInstance, drawn kinds, Annotations, Listing, icons) | `geop-core-editor::view` |
| `geop-ops-output::{export, inspect}` (Exporter, Inspector, Answer, Block, Outputs) | `geop-core-editor::output` |
| `geop-ops-output::sheet` (SVG/DXF sheets) | `geop-lib-sheet` (it is a writer library, not editor vocabulary) |

Drawing must stay instanced: a `Drawer` sends components by key and
instances by pose, never a mesh per placement, and `SolidMeshes` is a
chain target with one node per solid, recorded against that solid's
geometry, so one changed body re-meshes one body and a dragged placed part
re-meshes nothing. (A body as `geometry + pose` inside one part, so that
`MoveBody` and patterns also send a transform instead of a mesh, is the
extension this makes possible; it touches every consumer of the model and
is out of scope here.)

`PartView` keeps its structure; its `drawn::*` kinds become plain strings
supplied by whoever draws, and `ViewSketch`/`ViewDatum` are renamed to the
neutral things they are (`ViewCurveSet` with regions, `ViewFrame` with a
kind) in a second pass. First pass: move as is.

### 4.2 Module layout

```
geop-core-editor/src
  lib.rs
  editing/        dialog, event, form, gizmo, hit, visual    (unchanged)
  edit_form.rs    EditForm, FormState, setters                 (unchanged)
  part_operation.rs, editable.rs, selectable.rs                (unchanged)
  view/           PartView, pick, ViewInstance, annotations, listing, icons
  output/         Exporter, Inspector, Answer blocks, Outputs
  registry.rs     Registry: every extension point in one builder (§4.3)
  step_editor.rs  one step edited: arming fields, picks, drags, gizmo
  editor/         Editor, Command, Update, ProgramState, SceneState, StepState,
                  undo/redo, marker, builds, settle skeleton
  tools/          Tool trait; PickTool
  testing.rs      feature `testing`: Stepping, pointer, from_above, click_at, dialog,
                  handled, loaded, editor_with(registry, program)
```

### 4.3 Extension points and the registry

Every way an operation crate contributes to the editor is a trait with a
registration method. One builder, one `register` function per crate:

```rust
pub struct Registry { /* operations, drawers, exporters, inspectors, panels, tools, settler, examples */ }

impl Registry {
    pub fn operation<O: Editable>(self, op: O) -> GeopResult<Self>;
    pub fn drawer<D: Drawer>(self, d: D) -> GeopResult<Self>;
    pub fn exporter<E: Exporter>(self, e: E) -> GeopResult<Self>;
    pub fn inspector<I: Inspector>(self, i: I) -> GeopResult<Self>;
    pub fn panel<P: Panel>(self, p: P) -> GeopResult<Self>;
    pub fn tool<T: Tool>(self, t: T) -> GeopResult<Self>;
    pub fn settler<S: Settle>(self, s: S) -> GeopResult<Self>;   // at most one
    pub fn examples(self, set: ExampleSet) -> GeopResult<Self>;
    pub fn operations(&self) -> OperationRegistry;               // for the runner
}
```

Each op crate exports exactly one `pub fn register(r: Registry) ->
GeopResult<Registry>` that adds everything it contributes. `geop-cad-base`
is then:

```rust
pub fn registry() -> GeopResult<Registry> {
    let r = Registry::default();
    let r = geop_ops_parameters::register(r)?;
    let r = geop_ops_datums::register(r)?;
    let r = geop_ops_sketch::register(r)?;
    // ... one line per crate, in ribbon order
    Ok(r)
}
```

Ribbon order within a group comes from registration order, then
`OperationInfo::tier`, as today.

The extension traits:

- **`Editable`** (exists): an operation and its form.
- **`Drawer`**: `fn draw(&self, reader: &Reader, view: &mut PartView,
  hidden: &[String]) -> GeopResult<()>` and `fn structure(&self, reader:
  &Reader) -> GeopResult<Vec<StructureItem>>`. The crate that defines a
  target draws it and lists it. Replaces `geop-cad-base/src/part.rs` and
  `geop_ops_assembly::draw` composition; the assembly's drawer emits
  `ViewInstance`s. `StructureItem` gets a `kind_label` so the web's
  `StructurePanel` stops mapping `"sketch" -> "Sketches"` itself.
- **`Exporter`**, **`Inspector`** (exist): move the traits, keep the
  semantics.
- **`Panel`**: a program-level view model and the commands it accepts.
  ```rust
  pub trait Panel: 'static {
      const KEY: &'static str;                      // "parameters", "joints"
      fn state(&self, ctx: &EditorContext) -> GeopResult<serde_json::Value>;
      fn command(&self, ctx: &mut EditorContext, payload: serde_json::Value) -> GeopResult<Changed>;
  }
  ```
  `ProgramState.panels: BTreeMap<String, Value>`, `Command::Panel { key,
  payload }`. The parameters panel (defined parameters, resolved values,
  uses, materials, rename) becomes a `Panel` in `geop-ops-parameters`; the
  joints panel, freedom and conflicting report become a `Panel` in
  `geop-ops-assembly`. The fixed fields `parameters`, `parameter_uses`,
  `materials`, `joints`, `freedom`, `conflicting` leave `ProgramState`.
- **`Tool`**: a mode in hand while no step is edited.
  ```rust
  pub trait Tool: 'static {
      const KEY: &'static str;                      // "drag", "pick:<inspector>"
      fn handle(&mut self, ctx: &mut EditorContext, event: &StepEditEvent) -> GeopResult<Changed>;
      fn presentation(&self, ctx: &EditorContext) -> Presentation;
  }
  ```
  The drag tool (grab a placed part, pull it, solve) moves to
  `geop-ops-assembly`; the pick tool stays in core since it is generic over
  inspectors. `Command::DragTool`/`Command::PickTool` become
  `Command::Tool { key: Option<String> }`.
- **`Settle`**: the state solver hook, called by the editor after every
  edit with the pulls the open step's form reported.
  ```rust
  pub trait Settle: 'static {
      fn settle(&self, rerun: &mut Rerun, state: &StateValues, pulls: &[Pull],
                own: Option<&str>) -> GeopResult<Settled>;   // moved state, converged, conflicting
      fn solvable(&self, reader: &Reader) -> GeopResult<Vec<String>>;
  }
  ```
  The editor keeps the policy that is about editing (run to the open step
  or the whole program; keep only solvable state once complete; undo
  granularity of a drag). `geop-ops-assembly::solving` implements the
  trait. With no settler registered, state is kept as given, which is what
  a test of a fillet wants.
- **`ExampleSet`**: named example programs and multi-file workspace
  examples. Each op crate ships the examples that exercise it;
  `Command::LoadExample` looks them up in the registry.

### 4.4 What stays in the engine

Undo/redo of the program and of the open step; the marker and `run_to`;
the builds cache per file (`KEPT_BUILDS`); scene diffing (`Shown`,
instances sent as changes); `StepEditor` interaction (arming fields,
picks, selections, drags, gizmo, keys); `settle` policy skeleton; JSON
transport (`handle_json`); the `Crash` command. None of it names an
operation.

---

## 5. Operation crates

### 5.1 The shape of an op crate

```
ops/geop-ops-<name>/
  Cargo.toml          features: testing (fixtures for dependents)
  src/lib.rs          the public surface, enumerated: register(), target types,
                      reference constructors, read_* functions, *Args types
  src/targets.rs      the targets it defines: types, HasElements/Kind/Selectable impls
  src/<op>.rs         one PartOperation per file: Args, run, form, new_args, canvas
  src/draw.rs         Drawer impl (if it defines a drawn target)
  src/icons.rs
  src/examples.rs     ExampleSet: programs that exercise this crate
  src/testing.rs      feature `testing`: what dependents' tests need
  src/program_tests.rs    builds programs with a registry of what it needs
  src/editor_tests.rs     drives geop_core_editor::Editor the way the front end does
  src/regression_tests.rs rebuilt user reproducers (AGENTS.md), named by scenario
  src/stress_tests.rs     ignored sweeps
```

Public surface rule: an op crate's `lib.rs` re-exports everything another
crate may use; nothing else is `pub`. If a dependent wants something that
is not there, either it belongs in a `geop-lib-*` crate or the dependent is
reaching into an operation it should only read targets of.

### 5.2 Splits: libraries out of operation crates

| today | algorithm → `geop-lib-*` | operation stays in `geop-ops-*` |
|---|---|---|
| geop-ops-booleans | `geop-lib-booleans`: boolean, remesh, split, imprint, trim, naming, scenes, render tests | `geop-ops-boolean`: Boolean, Split operations and their forms |
| geop-ops-extrude-revolve | `geop-lib-sweep`: common, extrude, revolve, sweep, path_sweep, loft, skin, plain, shapes (test solids) | `geop-ops-extrude-revolve`: Extrude, Revolve, Sweep, Loft operations; `testing` (sketch + extrude steps) |
| geop-ops-shell | `geop-lib-shell`: shell, offset_faces, thicken, extended, pcurve | `geop-ops-shell`: Shell operation |
| geop-ops-output | `geop-lib-sheet`: Sheet, Layer, to_svg, to_dxf, xml_escape | (traits go to core-editor; crate disappears) |
| geop-cad-base/stdlib | `geop-lib-stdlib`: standard part families, tables, `WithStandardParts` FileStore | — |
| geop-ops-rasterize | `core/geop-core-rasterize` (depends on math/geometry/topology only) | — |
| geop-ops-add-datum | — | rename to `geop-ops-reference-geometry`; `Selected` documented as its public pick vocabulary (it sits above every crate it picks from) |

Lib crates take kernel and model types. Where an algorithm today takes a
`PlacedSketch` (`extrude.rs`, `shape_loops`), the lib takes `Profile`s and
`Pose`s and the operation resolves the sketch reference first.

### 5.3 Vocabulary into `geop-ops-model`

Move to `geop-ops-model` (it already holds `BooleanOp`, `Feature`,
`FeatureTool`, `ToolReach`):

- `Combine` (new body / join / cut / intersect with a solid) and
  `SolidRef`, `Tool`, `solid()` from `geop-ops-booleans::operation`;
- `Extent` (blind / up to next / through all / up to face) from
  `geop-ops-booleans::extent`, since every feature that cuts needs it;
- `Extents`/`Output` from extrude-revolve if they are the two-sided form of
  the same thing (decide while moving; one type, not two).

Applying a `Combine` needs the boolean algorithm, so `geop-ops-model`
would depend on `geop-lib-booleans`, which depends on model: a cycle.
Resolution: the *types* live in model; the function that applies a
`Combine` to a `NamedModel` lives in `geop-lib-booleans`
(`combine(model, tool, &Combine)`). Operations call that.

`geop-ops-model` drops its dependency on the view: its `draw.rs` becomes a
`Drawer` implemented in the same crate but depending on
`geop-core-editor::view` (core, allowed) and `geop-core-rasterize` (core,
allowed). `Design` and `validate_operation_id` are imported from the core.

### 5.4 Expected dependency graph after the split

```
geop-ops-model        -> core
geop-lib-booleans     -> model
geop-lib-sweep        -> model, lib-booleans (to combine)
geop-lib-shell        -> model, lib-booleans
geop-lib-sheet        -> core-math
geop-ops-parameters   -> model (its Panel, DefineParameters, PartProperties)
geop-ops-datums       -> model
geop-ops-sketch       -> model, datums
geop-ops-sketch3d     -> model, datums, sketch (reads sketch target for projection)
geop-ops-add-datum    -> model, datums, sketch
geop-ops-extrude-rev. -> model, lib-sweep, sketch, sketch3d, datums
geop-ops-boolean      -> model, lib-booleans
geop-ops-fillet       -> model, lib-booleans, lib-sweep
geop-ops-hole         -> model, lib-booleans, lib-sweep, sketch, datums
geop-ops-shell        -> model, lib-shell
geop-ops-plastic      -> model, lib-booleans, lib-sweep, lib-shell, sketch
geop-ops-pattern      -> model, lib-booleans, sketch, datums
geop-ops-constraints  -> core
geop-ops-assembly     -> model, constraints, datums, add-datum (Selected), core-rasterize
geop-ops-drawing      -> model, lib-sheet, assembly (placed parts), bom
geop-ops-inspect      -> model, assembly
geop-ops-bom, -urdf, -step -> model, assembly, lib-sheet/rasterize as needed
geop-lib-stdlib       -> the ops whose Args it writes programs in
geop-cad-base         -> everything
```

`geop-ops-fillet` no longer depends on `geop-ops-extrude-revolve`;
`geop-ops-edit` no longer depends on the boolean operation crate. Changing
the Extrude dialog retests Extrude, stdlib and cad-base. Changing the
boolean algorithm retests, correctly, everything that cuts.

Known remaining op→op edges that are legitimate (reading a target):
sketch (every feature on a sketch), datums (planes and axes), add-datum's
`Selected` (what a mate picks), assembly's placed parts (drawing, BOM,
inspect, STEP export).

---

## 6. Thin `cad/`

- `geop-cad-base`: `registry()` (one line per crate), `Editor::new()`
  convenience, the `std:` library wired in as the workspace's shared
  `FileStore`. Target ≤ 400 lines. Tests: `every_operation_is_offered`,
  `every_operation_opens_on_any_part`, every example loads and round-trips
  (build, save, load, build again, compare), the JSON protocol smoke test.
  Nothing else.
- `geop-cad-web`, `geop-cad-cli`: unchanged in role; the CLI's `compile`,
  `bom`, `serve` use the registry's exporters and inspectors by name as
  they do today; its hand-written tests that build parts move to the
  crates they test (step import, assembly solve).
- `web/`: `ProgramState.panels` rendered by a `PanelView` that maps panel
  keys to components (`ParametersPanel.tsx`, `JointsPanel.tsx` stay as
  components, but are looked up by key, never by inspecting the program);
  `StructurePanel` groups by `kind_label` from the kernel; tools are
  toggled by key. The web app must not contain an operation name outside
  that key map. `geop.ts` protocol types regenerate from the Rust types
  (consider `ts-rs` or a generated `protocol.ts`; today they are kept in
  sync by hand).

---

## 7. Migration phases

Phases 1, 1b and 2 are serial and touch every crate; run them one agent
at a time on `refactor`. Phases 3 and 4 are per crate and run in parallel
worktrees. Phase 5 closes.

Every phase ends with: `timeout 1800 prlimit --as=16000000000 cargo test
--workspace` green (fast suite), `cargo clippy --workspace`,
`scripts/check-layering.sh` green, `npm run build && npm test` in `web/`.
Record the test count per crate before and after; a phase that loses
tests has to say which and why.

### Phase 0: baseline and tooling (1 agent, small)

- Add `scripts/check-layering.sh`: reads `cargo metadata`, asserts the
  rules of §2.1 (core→core only; lib→core+model; ops→no cad; web protocol
  types only). Initially allowed to fail with a known-violations list that
  phases burn down.
- Add `scripts/test-affected.sh [base-ref]`: changed files → crates →
  reverse dependencies via `cargo metadata` → `cargo test -p …`. Document
  in DEVELOPERS.md; use in CI for pull requests.
- Record per-crate line and test counts (§1.1) in the PR description.

### Phase 1: the core owns the program (1 agent, serial)

1. `geop-core-program`: add `values.rs` (D1), merge `Program`/`ProgramFile`
   (D2), move `validate_operation_id` (D3), move `workspace` (D4), move
   `NumericInput` and `ParameterValues` (D5). Drop the `<I, S>` generics.
2. Mechanical sweep of all crates: replace the `Setting/StateValue/
   BuildPart/PartReader/Runner/Registry/StepContext` imports with the core
   types; `ProgramFile` → `Program`; `Loaded` → `Rc<Program>`.
3. `geop-ops-parameters` shrinks to: `DefineParameters`, `PartProperties`
   operations, `Parameters`/`ParameterKind`/`Material`, the free functions
   over `Program`, `instance.rs`/`state.rs` helpers (`located`,
   `pose_state`). Decide whether `located`/`placement` belong to assembly
   (they describe where a placed part is) and move them there if so.
4. Add `Program::validate(&registry)` and call it on `Command::Load`,
   `Paste`, `Commit`. Add `testing::assert_incremental`.
5. Core tests: rewrite `test/` and `test2/` on the real value types.

Acceptance: `geop-core-program` depends on `geop-core-math` only; no crate
outside core defines a state or input type; `rg 'ProgramFile|BuildPart|
PartReader'` finds nothing.

### Phase 1b: the run model (1 agent, serial, after phase 1 lands)

Implements §3.2 to §3.6 on top of phase 1's concrete types:

1. `target.rs`/`part.rs`/`reader.rs`: `Node`, `Read`, `produce`, the
   reuse check, versions, `before()`/`current()`, sweep. Delete
   `revise_target`, `define_target`, `RevisionedValue`, `Same`, `carry`,
   `run_as`, `begin_run_as`, the `built` revision, `sees`.
2. `placed.rs`: `PlacedArgs`, `build_placed`, the build cache on the
   workspace, `Inside` reads. Delete `Kept`, `subpart_*`, `load_subpart`,
   `Subpart`, `Reader::inside`, `Dependency::behind`.
3. `Target` trait with `CHAIN`; `Program::validate` checks the chain rule.
4. Sweep: every `define_target`/`revise_target` call becomes `produce`;
   `build_model` in `geop-ops-model` becomes a one-line `produce` of the
   `model` chain; `AddPart` produces a placed node; every
   `reader.inside(path)` becomes `retrieve_placed`.
5. `geop-ops-parameters`: `LookupTable` operation; `ParameterKind::Table`
   removed; the standard parts and examples that used tables rewritten
   (they are Rust, not files).
6. Tests: `assert_incremental` over every example; a test that a thousand
   placed equal parts build once (count `make` runs through a test hook on
   the registry, not a timer); a test that a two-instance sub-assembly
   with one joint moved rebuilds one instance; the reorder test.
7. The assembly solver (`geop-ops-assembly::solving`, `Rerun`) adapts to
   `runner.run(part, program, upto)` with state in the program.
8. `geop-ops-constraints`: a constraint whose residual reads a target
   that recorded a `Read::State` is refused, naming the constraint, the
   target and the state (see §3.6, the cable example).

Acceptance: the fast suite green; `assert_incremental` passes on every
example; the slow `example_mass_properties_are_consistent` and the
assembly scale tests (`assembly_scale_tests.rs`, a drag in 2000 parts)
are not slower in CPU time than the baseline; `rg 'revise_target|
define_target|load_subpart|Kept|inside\(' --type rust` finds nothing.

### Phase 2: the core owns the editor (1 agent, then 1 agent, serial)

2a. Move `geop-ops-view` into `geop-core-editor::view`; move
    `geop-ops-output::{export,inspect}` into `geop-core-editor::output`;
    create `geop-lib-sheet` from `geop-ops-output::sheet`. Move
    `geop-ops-rasterize` to `core/geop-core-rasterize` (its tests that
    need swept shapes move with the shapes to `geop-lib-sweep` in phase 3,
    or use the topology crate's cube fixture). `geop-ops-model` now draws
    through core types only.
2b. Move `step_editor.rs`, `editor.rs`, `inspect.rs` into
    `geop-core-editor`. Introduce `Registry`, `Drawer`, `Panel`, `Tool`,
    `Settle`, `ExampleSet` (§4.3). Extract from the moved `editor.rs`:
    - parameters panel state and commands → `geop-ops-parameters::panel`;
    - joints/freedom/conflicting panel, drag tool, settler →
      `geop-ops-assembly::{panel, drag_tool, settle}`;
    - `part.rs::structure` and `assembly::draw` → `Drawer` impls in
      model, sketch, datums, assembly, view-listing crates.
    Add `geop-core-editor::testing` (from `cad-base/operations/testing.rs`).
    `geop-cad-base` keeps a `registry()` built from every crate's
    `register` and re-exports `Editor`; its tests stay where they are for
    now (phase 4 moves them).

Acceptance: `geop-cad-base/src` is `lib.rs`, `registry.rs`, `operations/*
tests*` and `examples.rs`, `stdlib/`; `rg geop_ops_ core/geop-core-editor`
finds nothing; `web/` compiles against the new `ProgramState.panels` and
`Command::Panel`/`Command::Tool` with the panels looked up by key.

### Phase 3: libraries out of operations (parallel, one agent per row of §5.2)

Each agent, in its own worktree:

1. Create the `geop-lib-*` crate, move the algorithm modules, make them
   take geometry/model types (no `PlacedSketch`, no `ElementReference`).
2. Leave the operation crate with operations, forms, `register`, and its
   `testing` feature.
3. Move `Combine`, `Extent`, `SolidRef`, `Tool`, `solid()` into
   `geop-ops-model` (booleans agent does this first; other agents rebase).
4. Rewire dependents' `Cargo.toml` and imports.
5. Run `scripts/test-affected.sh` and the ignored sweeps of the touched
   areas (`stress_tests`, boolean render sweep).

Conflict hot spots: workspace `Cargo.toml` members and dependencies,
`geop-cad-base/src/registry.rs`, DEVELOPERS.md crate table. Merge booleans
first, then sweep, then shell, then stdlib.

### Phase 4: tests, examples and standard parts leave `cad-base` (parallel, one agent per op crate)

For each op crate, the agent:

1. Moves the tests of its operations out of `geop-cad-base/src/operations/`
   into the crate: `*_editor_tests.rs` → `editor_tests.rs` driving
   `geop_core_editor::Editor` with a registry of the operations the test
   needs (`testing::registry()` pattern of `geop-ops-extrude-revolve`);
   `*_tests.rs` → `program_tests.rs`; its entries of
   `regression_tests.rs` and `stress_tests.rs` → its own files.
   Assignment of the generic files: `tests.rs` (extrude/revolve/up-to-next)
   → extrude-revolve and lib-booleans; `regression_tests.rs` (torus,
   spindle, ring on plate) → extrude-revolve / lib-booleans by what
   failed; `view_tests.rs` → `geop-core-editor::view` tests plus model /
   sketch / datums drawer tests; `program_tests.rs` → core-program
   (runner reuse) and parameters (formulas); `set_tests.rs` stays in
   cad-base.
2. Moves the examples that exercise it from `examples.rs` into its
   `examples.rs` and registers them in `register`. Multi-file workspace
   examples go to the assembly crate.
3. Adds a round-trip test for each example (build, save, load, build,
   compare entity by entity) and `assert_incremental` over one edit
   sequence per example.
4. Deletes what it moved from `cad-base`.

The stdlib agent creates `geop-lib-stdlib` from `cad-base/stdlib` with its
tests, and the assembly, drawing and bom tests that place standard parts
depend on it as a dev-dependency.

Acceptance: `geop-cad-base/src` has `lib.rs`, `registry.rs`, and one
`tests.rs` of composition tests; test count across the workspace is not
lower than the baseline.

### Phase 5: thin shell and rules (1 agent)

- Web: `PanelView` keyed by panel key; `StructurePanel` by `kind_label`;
  tools by key; protocol types generated or checked against Rust.
- CLI: tests that build parts move to the crates they exercise.
- DEVELOPERS.md: rewrite the crate table (§2.1) and the "how an operation
  plugs in" section around `register`.
- AGENTS.md: add the rules of §10.
- CI: pull requests run `scripts/test-affected.sh`; `main` runs the whole
  suite nightly plus the ignored sweeps.
- `scripts/check-layering.sh` runs with an empty allow-list.

---

## 8. Scripts to add

`scripts/check-layering.sh`

```
cargo metadata --format-version 1 --no-deps | jq …
rules:
  geop-core-*  deps ⊆ geop-core-*
  geop-core-program deps ⊆ {geop-core-math}; geop-core-editor deps ⊆ {geop-core-math, geop-core-program}
  geop-lib-*   deps ⊆ geop-core-* ∪ {geop-ops-model} ∪ geop-lib-*
  geop-ops-*   deps ∌ geop-cad-*
  geop-cad-base src: no `use geop_ops_` outside registry.rs
  core/: `rg 'geop_ops_|geop_lib_|geop_cad_' core/` is empty
```

`scripts/test-affected.sh [base]`

```
changed crates = crates whose directory contains a file in `git diff --name-only base`
affected = changed ∪ reverse-deps(changed)  (cargo metadata resolve graph)
timeout 1800 prlimit --as=16000000000 cargo test $(printf -- '-p %s ' $affected)
```

Changes under `core/` affect everything and the script says so rather than
listing 40 crates.

---

## 9. Working as a swarm

- One worktree per agent (`git worktree add ../geop-<task>`), branch per
  task, rebased on `main` before the merge. Phases 1 and 2 are not
  parallel; do not start phase 3 agents before 2b is on `main`.
- Each agent's brief: the §-numbers of this plan it implements, the crates
  it may touch, the acceptance checks, and the instruction to run
  `scripts/test-affected.sh` plus the ignored sweeps of what it touched.
- Shared files everyone edits (keep both sides, then read the merge):
  workspace `Cargo.toml`, `geop-cad-base/src/registry.rs`, DEVELOPERS.md's
  crate table, `web/src/icons.tsx`. Each agent appends; the integrator
  reorders once.
- A merge is done when the full workspace suite and the web build pass on
  the merged branch, not when each branch passes alone.
- No agent adds compatibility shims (`ProgramFile` alias, old field names
  in `ProgramState`, a reader for old `.geop` state). Delete and rebuild.

---

## 10. Rules to add to AGENTS.md once this lands

- **Where things go.** Geometry with no notion of a program: `core/`. What
  a program, part, operation, workspace, editor *is*: `geop-core-program`,
  `geop-core-editor`. A target type, its element kinds, how it is drawn
  and listed, the operations that define it, their forms, tests, examples
  and regressions: that operation's crate. An algorithm two operations
  share: a `geop-lib-*` crate taking geometry and models, never
  references. Composition: `geop-cad-base/src/registry.rs`, one line.
- **A lib takes geometry; an op resolves references.** A `geop-lib-*`
  function never takes an `ElementReference`, a `PlacedSketch` or a
  `Reader`.
- **An op crate's public surface is its `lib.rs`.** Another crate may use
  only what it re-exports: `register`, target types, reference
  constructors, `read_*`, `*Args`. Wanting more means something is in the
  wrong crate.
- **Every op crate tests itself.** Program tests, editor-driven tests,
  regression tests, examples with a round trip and an `assert_incremental`
  pass. `cargo test -p <crate>` must pass alone; `scripts/test-affected.sh`
  is the pre-merge check.
- **The front end knows keys, not operations.** Panels, tools, structure
  groups and inspectors reach the web app by key and by the kernel's
  labels. An operation name in `web/src` outside the key map is a bug.
- **No fixed fields for one operation in the protocol.** What one
  operation family needs to show at program level is a `Panel`; what it
  needs the pointer for while no step is open is a `Tool`; what it needs
  after every edit is the `Settle` hook.

---

## 11. Decisions taken here, open to veto

| decision | alternative | why this one |
|---|---|---|
| D1 concrete `Setting`/`StateValue`/`Design` in core, generics dropped | keep `<I, S>` generics | the file format is the core's; 30 crates stop aliasing |
| D6–D9 one node kind keyed by step id, reads recorded by value, placed parts as nodes shared through a build cache | keep positions, revisions, `load_subpart` | fewer concepts, "what runs" answerable in one sentence, 1000 equal screws build once |
| tables become a `LookupTable` operation | keep `ParameterKind::Table` | a lookup is a step like any other; the core knows only numbers and text |
| booleans/sweep/shell split into lib + op crates | keep one crate each | directly serves "test only what depends on the change" |
| `geop-core-sketch` stays in core | move under ops | it is geometry with no program notion, used by five ops |
| rasterize moves to core | stays an ops lib | it is an algorithm over `Model` only |
| view and output traits move into `geop-core-editor` | separate `geop-core-view` crate | one editor crate under 10k lines is easier to hold than two |
| examples per op crate | one examples crate | an example is a test of its operations; it moves with them |
| panels keep bespoke TSX components, looked up by key | express panels as `Answer` blocks | the parameters panel is a real editor (tables, rename); UI code is allowed in the UI, logic is not |

---

## Status (2026-10-16)

Phases 0 to 5 have landed on `refactor`.

- `geop-cad-base/src` is `lib.rs`, `registry.rs` (117 lines together) and
  `tests.rs`, the composition's tests. The examples drawn in the editor
  live, as `.geop` text, in the crates whose operations they use.
- `scripts/check-layering.sh` passes with an empty allow-list; cad-base
  names op crates only in `registry.rs` and `tests.rs`.
- The CLI keeps only tests of its own; what it compiles is tested in the
  crate of each exporter.
- `web/src/geop.ts` is checked against the kernel: `cargo test -p geop
  protocol` keeps typed samples in `web/test/protocol.samples.ts`, which
  `tsc -b` checks. It found the viewer behind the protocol since elements
  became `PickedElement`s; `PartView` now sends the targets of its model,
  datums and sketches so the viewer lights elements by target.
- `LookupTable` has a form and an icon; `PartPattern` decides its copies in
  a node (`part_pattern:<step>`), every read recorded.
- Clippy passes with `-D warnings`; CI (`.github/workflows/ci.yml`) runs
  the checks and `scripts/test-affected.sh` on pull requests, the suite on
  `main`, and the ignored sweeps nightly.
- AGENTS.md has the rules of §10 (as "Where things go") and the lessons
  of the migration; DEVELOPERS.md and the book describe the new layout.
- The fast suite: 1455 passed, 0 failed, 87 ignored
  (`scripts/baseline-2026-10-16.txt`).

Open follow-ups:

- Placed copies of one component each build their own `PartView`,
  although they share its key (see
  `geop-ops-assembly`'s `draw_tests.rs`): equal components should be
  drawn once.
- `geop-ops-assembly` depends on `geop-lib-stdlib` for its
  `metric_plate` and `bolted_plate` examples, while `geop-lib-stdlib`
  dev-depends on `geop-ops-assembly`: a dev-dependency cycle of the kind
  AGENTS.md warns of. The examples that place standard parts belong in a
  crate above both, or the standard parts' tests in `tests/`.
- The protocol samples change whenever a dialog changes shape, so a change
  to an operation's form means writing them again; the check covers the
  shapes a session reaches, not every field an operation could show.
- `geop.ts` is still written by hand, only checked. Generating it (as
  `ts-rs` would) would remove the duplication, at the cost of deriving
  type information across every protocol type in core.
- The ignored sweeps pass but for `geop-ops-assembly`'s
  `a_joint_is_made_by_clicking_where_its_frames_go`, ignored as a known
  failure (a hovered frame pick is lit but not drawn as a triad), and the
  STEP corpus, which needs `fetch_corpus.sh`; the nightly CI skips both by
  name.
- Web e2e tests (`web/e2e/`) were not run here; the viewer's lighting by
  `PickedElement` is covered by the type check and the build, not yet by
  a browser test.
