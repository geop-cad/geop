# Lecture 39: The editor, its protocol and front ends

<div class="goals">

**After this lecture you can**

- explain the division of labour between the kernel's editor and its front ends, and why every decision lives in Rust;
- read and write the JSON protocol: `Command` in, `Update` out, and what an update leaves out;
- trace a click from a pixel to a typed reference: pointer, ray, reach, pick;
- say what a `Drawer`, a `Panel`, a `Tool` and a `Settle` are for, and why the protocol has no fixed fields for one operation;
- drive the real kernel by hand over `geop serve`, and over MCP.

</div>

So far a part is a program, and the runner builds it. A person, or an agent, has to make that program. CAD programs are usually built in the opposite order: a GUI full of operation logic with a kernel underneath. Geop inverts this. **The kernel contains the editor.** A front end renders primitives it is sent and forwards raw input. Every decision (what a click picks, what a line snaps to, what a drag changes) is made in Rust, where it can be tested without a browser.

This lecture follows one idea: the front end knows keys, not operations.

## Why put the editor in the kernel?

Geop has several front ends for one editor: the web app running the kernel as WebAssembly, the same page served by `geop open` with the kernel as a native process, the VS Code extension, `geop app` (a window of its own), a click-to-open desktop app (`docs/desktop.md`, feature `desktop`), and an agent over MCP. If each one interpreted a click, they would diverge, and each would need its own tests. If the logic is in the kernel, then:

- all of them behave alike (the rule is stated in `docs/ux.md`: "a front end only draws what it is sent and sends what the user did");
- a test of user behaviour needs only Rust (`geop_core_editor::testing`, in the next lecture);
- adding an operation needs no change to any front end.

The cost is that the protocol must be cheap, because everything crosses it: a hover is a message. We return to that below.

## One editor, as a state machine

`Editor` (`core/geop-core-editor/src/editor/mod.rs`) is the whole of editing a program. It holds the program, its undo and redo, how far it runs (the marker), and the step being edited, a `StepEditor` plus the part before that step as drawn, for picking. Everything goes through one function:

```rust
pub fn handle(&mut self, command: Command) -> Update
```

and, on the wire, `Editor::handle_json`, one JSON text in and one out. `Command` and `Update` are in `core/geop-core-editor/src/editor/protocol.rs`. The commands fall into groups:

| Group | Commands | Effect |
|---|---|---|
| The program | `New`, `NewIn`, `Open`, `Remove`, `Move`, `Copy`, `Paste`, `Seek`, `Load`, `Files`, `LoadExample`, `LoadWorkspaceExample`, `Undo`, `Redo` | change the program, refused while a step is being edited or if the result would be invalid |
| The step being edited | `Event`, `Preview`, `Commit`, `Cancel` | what the user did to it; `Commit` is refused unless it builds |
| Extensions | `Panel`, `Tool` | a command to a panel by key; a tool taken in hand by key |
| Display | `Visibility`, `Appearance`, `Isolate`, `Light` | what is shown: settings saved with the program, or lit, none of them a step |
| Outputs | `Export`, `Inspect` | write a file, answer a question; change nothing |
| Nothing | `Show` | what to show, all of it: how an editor starts |

After every command that changed something, the editor settles the state (below), runs what is shown, and tells the panels. The answer is an `Update`:

```text
error, program, scene, step, files, written, export, tool, inspection, lit, report
```

Every field but `error`'s meaning is the same: *what changed.* Fields that did not change are left out. A hover sends only the step; the part is sent again only once something ran. This is what makes it viable to send every pointer event across the protocol.

## A real session

Let us run this. `geop serve` is the editor as a host process: one JSON command per line on stdin, one JSON update per line on stdout. It is how the VS Code extension drives the kernel (`vscode-extension/src/server.ts`). The session (`cad/geop-cad-cli/src/session.rs`) answers each command with exactly one line. I sent these seven commands to the v0.12.0 binary, capped as `AGENTS.md` requires:

```sh
timeout 120 prlimit --as=6000000000 geop serve < commands.jsonl > answers.jsonl
```

```json
{"command":"show"}
{"command":"load_example","name":"box_with_drill_hole"}
{"command":"new","kind":"boolean"}
{"command":"cancel"}
{"command":"inspect","inspector":"mass_properties"}
{"command":"seek","marker":1}
{"command":"seek","marker":null}
```

Here is what came back, reduced to its keys and sizes. These are measured, from that one run:

| Command | Bytes of the answer | Fields present |
|---|---:|---|
| `show` | 28 881 | `program`, `scene`, `tool`, `lit` |
| `load_example` | 108 635 | `program`, `scene`, `tool` |
| `new` | 32 834 | `program`, `step`, `lit` |
| `cancel` | 30 854 | `program`, `tool`, `lit` |
| `inspect` | 2 431 | `tool`, `inspection` |
| `seek` to 1 | 32 372 | `program`, `scene`, `tool` |
| `seek` back to all | 63 026 | `program`, `scene`, `tool` |

Some things to read off this table.

*An empty editor already says 28 kB.* That is the `ProgramState`: its keys, as sent, are `program`, `path`, `steps`, `marker`, `can_undo`, `can_redo`, `tool`, `tools`, `operations`, `exporters`, `inspectors`, `examples`, `workspace_examples`, `panels`, `details` and `keys`. In this run `operations` listed every operation with its group, tier and icon; `exporters` began `stl`, `urdf`, `motion`, `step`, `drawing`, `flat_pattern`; `inspectors` were `measure`, `mass_properties`, `interference`, `clearance`, `bom`; `panels` were `configurations`, `joints`, `motion`. A front end builds its whole toolbar and all its menus from these lists. It has no list of its own.

*Loading the example answers with the program's steps:*

```json
{"id":"outline","kind":"add_sketch","label":"Sketch",
 "summary":"plane=plane origin:xy, project=","error":null,
 "runs":true,"suppressed":false,"editing":false,"placed":[]}
```

and then `box`, `hole_sketch`, `hole`, with the marker at 4 and `scene.part.solids` holding `extrude(hole)`. The name is the one [Lecture 2](./02-programs-of-steps.md) explained: the solid the last step left.

*`new` answers with the dialog.* `step` has `kind`, `label`, `doc`, `id` (null, not committed yet), `presentation`, `missing`, `error`, `preview`, `can_undo`, `can_redo`. Its `presentation.dialog` begins:

```json
[{"key":"a","type":"reference","label":"a","accepts":["face"],
  "value":[{"element":{"target":"/:model","kind":"solid","name":""},
            "detail":null,"tone":"normal"}],
  "multiple":false,"required":true,"armed":false}, ...]
```

The front end has never heard of a boolean. It sees a reference field keyed `a` that accepts clicks on faces and draws it. No `scene` was sent, because nothing ran.

*`inspect` answered the question and changed nothing.* The inspection reports the volume with an error estimate:

```json
{"kind":"mass_properties","report":{"bodies":[{"name":"extrude(hole)",
 "properties":{"volume":{"value":3.74867258787563,"error":4.667e-07}, ...}}]}}
```

We can check it by hand. The example is a 2 by 2 by 1 box with a blind hole of radius 0.4, half deep, so `$V = 4 - \pi\cdot 0.4^2\cdot 0.5 = 3.748672\ldots$`, and the kernel's answer agrees. It integrates over the exact trimmed faces, by the divergence theorem and Green's theorem (`core/geop-core-topology/src/mass.rs`), and it reports a value with an error, not a bare number.

*The last `seek` shows what is not sent again.* Seeking to the end returned 63 026 bytes where the first load took 108 635. The faces' meshes it had sent were not all sent again: a face is sent by a key, each mesh once, and the viewer forgets one only when told to (`forgotten_meshes` in the scene).

### The same over MCP

An agent speaks the same editor through `geop mcp`, a Model Context Protocol server over stdio (`cad/geop-cad-cli/src/mcp.rs`, documented in `docs/mcp.md`). It is one transaction per tool: no step is left open between calls. From another capped run, editing the `box` step to name a sketch that does not exist:

```json
{"name":"edit_step","arguments":{"id":"box","args":{"sketch":{"target":"/:sketch:nothing"}}}}
```

returns a status with `"failed": 3`, `"solids": []`, and at step `box`:

```text
RootError: no step before here produces target `sketch:nothing`
           (the targets before here: `sketch:outline`)
Context:
resolving sketch("/:sketch:nothing", "")
Argument: sketch
Context:
extrude(box, ExtrudeArgs { ... })
```

This is not a tool error. The step did not build, so the failure is in the status the tool returns. Three steps failed, since `hole_sketch` and `hole` depended on it: the runner's failure propagation from [Lecture 38](./38-runner.md), with the `with_context` chain of [Lecture 42](./42-case-studies.md) making the root cause readable. It also says *what the valid names are*, which is what lets an agent recover on its own. The argument tag (`Argument: sketch`) comes from `GeopError::concerning`; in the web app it is what puts the error under that field in the dialog (`docs/ux.md`, "An error stands beside its cause").

## What an operation gives the editor

Nothing in `Editor` names an operation. What it edits comes from a `Registry` (`core/geop-core-editor/src/registry.rs`), a builder that every op crate adds to with its `register` function. An application calls them in turn, and for the full application that is `cad/geop-cad-base/src/registry.rs`, one line per crate. The kinds of things a crate can add:

- **Operations**, each an `Editable`, in practice a `PartOperation` ([Lecture 40](./40-writing-an-operation.md)).
- **A `Drawer`** draws the targets a crate defines into the part's view, and lists them in the structure tree. `ModelDrawer`, `DatumsDrawer` and `SketchDrawer` in `geop-ops-model` are examples; the registry only composes them (`Registry::draw`, `Registry::structure`).
- **An `Exporter`** writes a file from what a program built; **an `Inspector`** answers a question about it. Neither is a step, so neither changes the program.
- **A `Panel`** is program-level state a front end shows beside the steps, sent under its `KEY` in `ProgramState::panels`, and taking `Command::Panel`: the joints (`JointsPanel`), motion (`MotionPanel`), configurations (`ConfigurationsPanel`).
- **A `Tool`** is a mode in hand while no step is edited: `DragTool`; and the pick tool of an inspector that picks (`PickTool`, by the key `pick:<inspector>`).
- **A `Settle`**, at most one: the solve the editor runs after every edit, moving the program's state so its constraints hold. `geop-ops-assembly` has the only one (`Settler`). With none, the state stays as given.
- **An `ExampleSet`**: programs listed by name without being made.

This design answers a question that comes up in every growing application. `ProgramState` once had fields named `parameters`, `joints`, `freedom` and `conflicting`, as `AGENTS.md` records: every front end then carried the assembly's concepts, whether it placed parts or not. The rule since: **no fixed fields for one operation in the protocol.** What one operation family needs to show at program level is a `Panel`. What it needs the pointer for while no step is open is a `Tool`. What it needs after every edit is the `Settle` hook.

The same holds in the other direction. Panels, tools, structure groups and inspectors reach the web app by key and by the kernel's labels. An operation's name in `web/src` outside the key map (`web/src/ProgramPanels.tsx`) is a bug: a branch the next operation silently falls through.

## From a pixel to a reference

Picking is where the kernel-side design is hardest and pays the most. A front end does not hit-test. It sends a **pointer**:

```rust
pub struct Pointer {
    pub ray: Ray,
    pub reach: Reach,
    pub cycle: usize,
}
```

(`core/geop-core-editor/src/editing/event.rs`). `ray` runs from the eye through the cursor. `reach` is how far from the ray counts as under it: a cone from the eye in perspective, a tube in an orthographic view, so that "near" means near *on screen* however far or zoomed the view is. `cycle` says which of what lies under the pointer it means, counted from the front.

`PartView` (`core/geop-core-editor/src/view/mod.rs`) is a part as the viewport draws it, and picking tests against exactly that, never re-deriving geometry of its own. So a pick cannot disagree with what is on the screen. The rules, from `docs/ux.md`:

- *Pick in what is seen.* What the scene hides is neither hit nor hides what is behind it.
- *Reach what is behind by stepping through it.* Everything under the pointer is a candidate, front to back. A right click lights the next. The kernel decides which, since it knows what is selectable, seen and held.
- *Light what a click holds.* While a field waits for a pick, a hover lights exactly what a click would put in the field, not the raw element under the pointer.

How the viewport's elements become typed references is `core/geop-core-editor/src/selectable.rs`. A reference field holds `Selectable` values. An `ElementReference<T, K>` is one when its kind `K` is a `SelectableElement`, which says what it can be picked from (`PICKED_FROM`) and how. A hit comes back as a `PickedElement`, an untyped kind, target and name as a front end names it. Only `ElementReference::from_picked` reads one, so an operation never matches on it.

A bug worth knowing from `AGENTS.md`, in the section *Ask a topological question topologically*: an edge counted as hidden when it lay more than one pointer reach behind the face hit, along the ray. At a glancing look, an edge *of that very face* lies further along the ray than that, so it could not be hovered. "Is this edge in front of the face?" was standing in for "does this edge bound the face?", which the topology answers exactly (`ViewEdge::faces`). Geometry was again answering what the topology already knew.

## The cost of every event

The editor answers as often as the pointer moves, so what happens at each event has a budget (see [Lecture 41](./41-performance.md)). Three mechanisms keep it small.

**`Changed`** (`core/geop-core-editor/src/editor/context.rs`) says what a command changed, and so what runs and what is sent. Its variants include `Nothing`, `Step`, `Run`, `State` and `Program`. An event of a drag gives `Changed::State`: only the program's state moved, so the update carries the parts it moved, and the program only once the drag is let go. From `AGENTS.md`: every event used to send all of it, 4.8 MB at 2000 parts, for an arm that moved.

**Drawings kept by what they read.** Each drawer's drawing is kept by what it read (`Reader::recorded`, then `Reader::reads_same`). The part shown again with other sketches draws its sketches again, never its faces. It is the runner's reuse rule applied outside the program.

**`begin_event`** (`PartOperation::begin_event`) is the place for what a session forgets before each event, so that no setter has to. A session is what an operation keeps between events besides its arguments, `()` for most, and only `add_part` overrides this hook among the operations (`ops/geop-ops-assembly/src/add_part.rs`).

One more rule makes the drag case work. An operation never sets the state itself. Its form lists `pulls`, where it wants the state pulled (`Pull::Point` for a drag, `Pull::Pose` for a placement typed in, `Pull::Value` for a joint's coordinate), and the editor solves the program with those pulls: *the constraints have the last word.* This is what separates "the user wants this part here" from "this part is here", the concern of [Lecture 37](./37-assembly-solver.md).

## The front ends

`web/` is a React front end. `web/README.md` documents the layout. Everything flows through a few files:

- `web/src/kernel.worker.ts`: the kernel in a worker. It imports the wasm module (`handle`, `agent_call`, `init_panic_hook`), says once whether it loaded, then answers each command with the update. A panic leaves the module unusable, so the page ends the worker and starts a fresh one with the program it holds (`web/src/backend.ts`).
- `web/src/geop.ts`: a thin typed wrapper, with the protocol types.
- `web/src/App.tsx`, `web/src/DialogView.tsx`, `web/src/OperationRibbon.tsx`, `web/src/SceneViewer.tsx`: the page, the dialog drawn from a step's primitives, the ribbon of operations by group and tier, and the scene drawn by three.js.

Where the kernel runs is one build-time choice: `web/src/backend.ts` for wasm, `web/src/backend.vscode.ts` for a native process on the VS Code host, `web/src/backend.local.ts` for `geop open`, which speaks the same protocol over a WebSocket.

The WebAssembly surface itself is tiny: `cad/geop-cad-web/src/lib.rs` keeps one editor, and exposes `handle`. Since it is the same `Editor`, a shared link such as [app.geop-cad.dev/?example=rounded_box](https://app.geop-cad.dev/?example=rounded_box) runs exactly the code this book discusses, in your browser.

### Keeping the protocol honest

A protocol between Rust and TypeScript can drift. `cargo test -p geop protocol` (`cad/geop-cad-web/src/protocol_tests.rs`) drives the editor by JSON, requires the kernel to read every command sampled, and keeps what it answers, reduced to its shapes, in `web/test/protocol.samples.ts`, which `tsc -b` checks against `web/src/geop.ts`. A change to what the editor sends fails that test until the samples are written again (`GEOP_WRITE_PROTOCOL_SAMPLES=1`), and the web build until `geop.ts` agrees. When two branches both change it, write the samples again on the merged branch, never merge them by hand.

### Panics

A panic in the kernel is a bug, but a CAD session is hours of a person's work. So every host treats it as survivable: `cad/geop-cad-cli/src/session.rs` answers `{"crashed": "..."}`, starts a fresh editor, and the front end gives it the program it holds. `geop mcp` does the same with the program as it was before the call. The state that matters is the program, which is plain data, so restarting costs one rebuild.

## An agent is a front end

MCP deserves a remark, because it is a very different user. The instructions the server sends say in a paragraph what a part is. The tools (`docs/mcp.md`) are `new_part`, `load_program`, `list_operations`, `add_step`, `edit_step`, `delete_step`, `run`, `list_elements`, `inspect`, `export` and `save`, plus `look` to see the part, `undo` and `redo`, and those that go through the editor as a person does (`open_step`, `step_event`, `commit_step`, `cancel_step`, `take_tool`, `panel`). This list is the one `docs/mcp.md` gives. The server reads its input on a thread of its own, so a client can cancel a long `inspect`, which answers with what it found so far, and is told how far it is (`Long checks` in that file; [Lecture 38](./38-runner.md) has what lies under it). `list_operations` given a `kind` returns its `default_args`, the JSON `add_step` takes, so an agent learns an operation's arguments the way a person learns a dialog: by opening it. What is easier done than written, such as drawing a sketch, is done through events, as a user does.

Everything this lecture stressed helps an agent as much as a front end: stable names rather than ids, errors that name their argument and the valid alternatives, a program that is plain data so an edit can be undone or diffed, and one source of truth.

<div class="recap">

## Recap

- The editor is in the kernel, and a front end draws what it is sent and forwards input. Decisions are made in Rust, where they are tested.
- `Editor::handle` takes a `Command` and returns an `Update`, holding only what changed. A hover sends the step alone.
- Operations, drawers, exporters, inspectors, panels, tools and the settler all come through a `Registry`; the protocol has no field for one operation; front ends know keys.
- A click is a `Pointer` (a ray and a reach), picked in what is drawn, turned into a typed reference by `Selectable`.
- `Changed` and kept drawings bound what an event costs. A drag changes state alone.
- `geop serve`, `geop open`, the wasm worker, VS Code and MCP are one editor, and the samples test keeps the protocol's two sides in agreement.

</div>

## Exercises

**1.** Why does `Commit` refuse a step that does not build, instead of committing it with an error? But a step *in* a program may fail, as the runner allows. What is the difference?

<details class="solution"><summary>Solution</summary>

`Commit` is a user's statement that this step is finished; refusing keeps them in the dialog with the error beside its cause while they can still fix it, and the program as it was. A step in a program may fail *later*, because an earlier edit broke what it reads, and then there is no dialog open: the program must still be representable, and the other steps must still build. So failure is a legal state of a program and an illegal result of a commit.

</details>

**2.** Run the session in this lecture yourself against `geop serve`. Then send `{"command":"event","event":{...}}` with a `dialog` event. Use `StepEditEvent` in `core/geop-core-editor/src/editing/event.rs` to find its JSON shape. Which keys does the answer hold, and how big is it compared with the `new` that started the step?

<details class="solution"><summary>Hint</summary>

Cap the process. The dialog's keys are in the `presentation.dialog` of the answer to `new`. Use a small python script to print each answer's non-null keys and length, as was done for the table above. `web/test/protocol.samples.ts` holds an example of every command the protocol test sends.

</details>

**3.** Suppose you need a "section view" feature in the viewport, with a plane the user can drag, that is not a step of the program. Is it a `Panel`, a `Tool`, an `Inspector`, a display setting, or does the front end own it? Argue from what each is for. Then check what the code does (`web/src/section.ts`, and the inspector's `Answer`).

<details class="solution"><summary>Hint</summary>

It changes no program and builds nothing. `DEVELOPERS.md` says the web app's section view clips the part from stencil counters, and `core/geop-core-editor`'s `Answer` can include a plane to cut a section with. Ask: must VS Code and the web app agree on it? Does a test need it? Does any kernel decision depend on it?

</details>

**4.** In the pick rules, why must a right click's stepping index travel in `Pointer::cycle` rather than be kept by the front end?

<details class="solution"><summary>Solution</summary>

Which candidates exist, in what order, and which of them count as one ("two faces of one feature are one feature") depends on what the field accepts, which only the kernel knows. Sending an index means a hover lights exactly what the click then takes, in every kind of pick: a field, a selection, the pick tool or an operation's own. A front end holding this would need to re-derive the candidates, which is the divergence the design avoids.

</details>

**5.** The update to `seek` back was smaller than the first load. Propose a test that would guard this, using counts instead of bytes, in the spirit of `AGENTS.md`'s *Count the work, not the seconds*.

<details class="solution"><summary>Solution</summary>

Drive an editor by `geop_core_editor::testing::handled`. Load a program, seek to 1, seek back. Assert on the update: the number of entries in `scene.meshes` the second time is at most the number of faces that were forgotten, or zero if none were; and `Runner::counts` shows no makes for the seek back, since the nodes of later steps were kept. A byte count would be brittle under protocol changes, whereas the count of meshes sent states the property itself: each mesh once.

</details>
