# Lecture 38: The runner and incremental builds

<div class="goals">

**After this lecture you can**

- describe what a `Program`, a `Part`, a node, a target and a `Reader` are, and how they relate;
- state the exact rule that decides whether a step's work is reused, and predict which steps run after an edit;
- explain why a value made again into an equal value stops a wave of rebuilding, and why a chain target behaves differently from a sketch;
- say why a read and its check must take the same route, using the kernel's own bugs as evidence;
- explain how a build survives between processes without ever trusting a stored record;
- write `assert_incremental` and `assert_round_trips` tests, and know what each one proves.

</div>

[Lecture 2](./02-programs-of-steps.md) said that a part is a program of steps. So far in the book we have looked inside the steps: how a boolean cuts, how a fillet rolls. Now we step back and ask how the steps are run. The question sounds like bookkeeping, and in most CAD systems it is treated that way, as an afterthought called "regeneration". Here it is an algorithm with a correctness property as sharp as anything in Part I, and it determines whether the editor feels instant or sluggish.

Here is the problem. A user drags the width of a sketch from 2.0 to 2.1. The program has forty steps, and one fillet among them takes a second and a half. Running everything from scratch at every pointer event is unusable. Running only "the steps after the edit" is wrong twice over: it does too much, because a later step may not depend on the edit, and in an editor that moves steps around, "after" is not even well defined from one run to the next. What we want is the classic property of a good build system:

> **A step runs when, and only when, something it actually read has changed.**

Everything in this lecture is how to make that sentence precise and true.

## The objects

A **program** (`core/geop-core-program/src/program.rs`) is what a `.geop` file holds: an ordered list of `ProgramStep`s plus the state it was saved with. Each step has an `id`, the kind of its `operation`, and its `args` as saved JSON:

```json
{
  "steps": [
    { "id": "box",  "operation": "extrude", "args": { "...": "..." } },
    { "id": "hole", "operation": "extrude", "args": { "...": "..." } }
  ]
}
```

A program is plain data. It can be copied for undo, compared for change, and saved unchanged. Steps refer to what earlier steps built only by name, never by an internal id, so a program means the same thing after a round through JSON.

A **part** (`Part`, in `core/geop-core-program/src/part.rs`) is what running a program produces. It is not a solid. It is a *memo table* with one kind of entry, a **node**: what one step produced of one target.

A **target** is a named, typed slot: `TargetReference<T>` is a stable name plus the Rust type of its value. The model is one target. Each sketch is another (`sketch:s1`). The datums are a third. Crucially, this crate knows none of them: `geop-core-program` depends only on `geop-core-math`, as `scripts/check-layering.sh` enforces. A model, a sketch and a datum are all defined by crates above it. The runner is a general incremental-computation engine that happens to be used for CAD.

A node is keyed by the pair (target name, step id), never by a position in the program. That one decision is what makes it possible to reorder steps.

## A step only declares

Here is the central interface. A step does no geometry directly. An operation's `run` receives a `StepContext` and *declares* what it produces:

```rust
ctx.produce(&target, &args, make)?;
```

where `make` is a plain function `fn(&Reader, &Args) -> GeopResult<T>`. "Plain function" is enforced by its type: it cannot capture anything from its environment, so whatever it depends on must be in `args` or be read through the `Reader`. This is the idea to hold on to. **A make is a pure function of its arguments and of what it reads, and everything it reads goes through one door.**

The `Reader` (`core/geop-core-program/src/reader.rs`) is that door, and it writes down everything that passes through it. Each thing a make reads is recorded on the node as a `Read` (`core/geop-core-program/src/target.rs`):

| `Read` variant | What it records | When it still holds |
|---|---|---|
| `Target` | the target's name, which step produced what was seen, and its version | the step visible now is the same producer, at the same version |
| `Absent` | that no earlier step produced this name | still no such producer |
| `Input` | the value of a run input | it is the same value |
| `State` | the value of a state variable | it is the same value |
| `File` | a workspace file's path and revision | same revision |
| `Listed` | which targets of one type existed, in order | the same list |
| `Inside` | a read made inside a placed part | that read holds in that part |

So the dependency graph is not declared by anyone. It is *observed*, at the finest grain available, every time a make runs. An operation author cannot forget a dependency, since a value that did not come through the reader could not have reached the make. Compare this with a system in which each feature lists its parents: such a list goes stale the first time someone adds a read and forgets to add a parent.

## The reuse rule

Now the rule. Look at `StepContext::produce` in `core/geop-core-program/src/part.rs`, stripped of its error handling:

```rust
let reusable = old.is_some_and(|node| {
    node.made_from::<T, Args>(args)
        && node.reads.iter().all(|read| part.holds(read, at, Some(name), ...))
});
```

A node is reused if and only if

1. a node for this (target, step) exists,
2. it was made from arguments equal to the ones given now, and
3. every read it recorded holds now, as `Part::holds` defines for each variant above.

Otherwise `make` runs, at once, so that a later node of the same step can read the result. That is the whole of it. In the words of the crate's documentation: *a make runs when its node is new, its arguments differ, or something it read changed; nothing else ever runs.*

### Versions and the early cut-off

How do we test "this read still holds" for a target whose value is a whole B-rep? Comparing models on every read would be absurd. Instead, every node carries a **version**, a stamp unique across all parts. A read records the version it saw, and holding means seeing that same version.

Now the subtle part. When a make runs again and its result *equals* the old value, by `PartialEq`, the node keeps its old version. Whoever read it then sees the same version, so their reads hold, and they are reused in turn. This is the *early cut-off*. It stops a wave of rebuilding at the first step whose output did not actually change. A sketch with a redundant formula, a parameter that was edited and then rounded: the rebuild stops right there.

Play with this model of the runner. It runs the same rule on arithmetic instead of geometry.

<figure class="demo" data-demo="incremental-build">
<figcaption>A model of the runner: the steps of a plate, where each "make" is arithmetic standing in for geometry. Green steps were reused, orange ones made again; purple ones were made again into an equal value, so their version stayed. Move <em>title</em>: only one step runs. Move <em>hole r</em>: the sketch and the cut run, the box does not. Move <em>width</em> slowly: the sketch rounds it, so most moves make the sketch again and stop there. After each run the demo builds the same program from scratch and compares, as <code>assert_incremental</code> does. It is a model: the rules are those of <code>StepContext::produce</code>, but it has no placed parts, files or listings.</figcaption>
</figure>

### Chains

Some targets are produced by exactly one step, a sketch for example. A second producer of such a target is an error that names both steps. The model is different: nearly every step transforms it. Such a target is a **chain**, and says so in its type (`Target::CHAIN`). Each producer starts from what the one before it left, by `Reader::previous`, and the producer a step sees is the latest one before it in the current order (`Part::visible`).

A chain has a consequence you must expect. Since every model step reads the previous model, editing a step rebuilds every model step after it, unless its value came out equal. I measured this on the real runner, in a scratch test over three steps that each build into the model, a block `base`, a plate `top`, and a block `side`, reading `Runner::counts`:

| What changed | `makes` that ran |
|---|---|
| first run | 3 |
| run again, nothing changed | 0 |
| edit `side`, the last | 1 |
| edit `top`, the middle | 2 |
| edit `base`, the first | 3 |
| swap `top` and `side` | 2 |

The swap is worth pausing on. Those two steps do not interact geometrically, yet both are made again, because each read *the model as the step before it left it*, and that changed. The rule is about reads, not about geometry. In contrast, two steps with independent targets, such as two sketches, are both reused when swapped, which the demo's "Swap the two sketches" shows. Inserting, removing or reordering steps needs no special handling at all: it changes which producer a reader sees, and the recorded reads catch that.

So the grain of what the runner can skip is the grain of the targets. That is why sketches, datums and parameters are targets of their own instead of being stored inside the model: a step that only reads one parameter (`NumericInput::resolve`, which records a read of each parameter a formula names) is not rebuilt when another parameter changes.

### One fast path

There is one optimization on top of the rule, at the start of `Runner::build` in `core/geop-core-program/src/runner.rs`: if the part ran with these same steps, inputs and state (`Part::ran_as`), and the files it read have not changed, the run returns at once. Walking the steps only to find every node reusable is the cost of a run, and an editor runs its program several times per event of a drag. This is not an exception to the rule; it is the rule's answer computed in one comparison. It is not taken for a part that read the part placing it (through `..`), since that part may have changed; we come to that below.

### Failure

A step that fails does not stop the run. `Runner::run` collects the errors in `RunOutput::step_errors`; later steps that do not depend on the failure still build, and a node reading a failed node fails too, with the message in context. For a user who has just broken step 5 of 40, this means steps that do not read it stay on screen, and the editor can say precisely what broke.

<div class="kernel">

**In the kernel.** `Runner` (`core/geop-core-program/src/runner.rs`) holds no part, only its operation registry, its workspace, its `Location` (the file it runs and the files being built around it) and the build cache it shares with the runners of the parts it places. `Runner::run(part, program, upto)` runs the steps before `upto`; the nodes of later steps are kept to be reused when they run again, and nodes of steps no longer in the program are dropped at the end of the run. This is how seeking back on the timeline and forward again costs nothing. `Runner::run_step` runs one step by hand, for a test of one operation. `Runner::counts` returns the `Counts`: makes, placed parts made and built, nodes restored. It is the instrument for [Lecture 41](./41-performance.md).

</div>

## A program does not know where it is

A runner knows its `Location`; a program does not. When a step reads another file, it goes through the runner (`Runner::program`, `Runner::read`) or its reader (`Reader::read_file`, `Reader::program`), which resolves the path relative to where the program runs, against a `Workspace`. A `Workspace` is where programs come from: a program by its file's path, a file's text, and a `revision(path)` counting how often the file changed. A node that read a file records its revision (`Read::File`), and a new revision makes it again.

`Runner::program` also refuses a file that places a program currently being built, however deep, and names the cycle. This is a nice example of refusing a configuration by name: the runner has the whole stack of files in its `Location`, so it can say exactly which files would place themselves, instead of overflowing a stack.

## Placed parts: a thousand screws, one build

A step that places another program's part is a node whose value is the built part, `PlacedPart`, an `Rc<Part>` (`core/geop-core-program/src/placed.rs`, whose make is `build_placed`). What such a build reads is the program file, the inputs given to it, and the slice of the placing part's state under the instance's name. Since that is *all* it reads, equal builds can be shared through the runner's build cache. A thousand screws of one size are one build and a thousand `Rc` clones. `Counts::builds` counts what was actually built, as opposed to `Counts::placed`, the makes of placed parts.

And where a placed part sits is no concern of its build: its pose is the placing part's state, read by the drawing and the mates, not built into its node. So **dragging a placed part builds nothing**. The whole of what makes interactive assemblies feasible hangs on this, as [Lecture 37](./37-assembly-solver.md) relies on.

A name behind an instance's path is spelled by `Located`: `/` between instances, `:` before the name, so `/asm/bolt:model` is the model of `bolt` inside `asm`, and `/:model` is the part's own.

### Read a value where its read is checked

Here is the first real bug. A version is reused when what it read did not change, so the place a value is read from and the place its read is checked must be the same. From `AGENTS.md`:

> A mate on a part of a placed sub-assembly read `/hinge:pin.pose` from the top part's own state, which the file never gave, and got no pose; the check looked in the hinge's part, where the hinge file's pose was, and saw nothing change. The mate kept a constraint differentiated by nothing, and the solve gave up at once.

Notice the failure mode. The read and the check *disagreed about what the same name refers to*. Nothing crashed at the read. The bug shows up far away, as a solver giving up. The fix: a value behind an instance's path is read inside that instance's part (`Reader::placed_part`), which is where `Part::holds` checks it. The lesson is general: when a read and its dependency take different routes to the same name, one of them is wrong.

### A read through `..` is checked where it was made

Parts placed in an assembly can read the parts placed beside them "in context", through the reserved path `..` (`geop_core_program::placed::OUTSIDE`): `/../bracket:model`. This means a placed part is no longer a function of its file and inputs alone. So its nodes record those reads inside `..`, and they are checked against the part placing it, as it was when the placing step built it. The placing node records them too, so a change in the sibling rebuilds the part, and a build is only shared where they hold.

One file is thus several programs. A part placed in two contexts, or placed with steps after its own (a mirrored component, `PlacedArgs::then`), builds differently from the same text. From `AGENTS.md`: what is kept of a file must be of *every* program it was built as since the cache was written. Noting only the last one's steps dropped what the other had kept, and the reopened assembly made the mirror again. A cache key that forgets one of the things the value depends on is the same bug as a forgotten dependency, only slower to notice.

## Between processes: a record is never trusted

Reopening a file should not rebuild everything. Yet a cache is a new way to be wrong: an entry from an earlier run could be stale in ways the current process cannot see. How do we reuse across processes and still keep the guarantee?

The versions are no use: they are stamps of this process. So a node also knows how it was made in terms that mean the same in any process. This is its `Derivation` (`core/geop-core-program/src/kept.rs`): a digest of its arguments (by their `Debug`, which writes every number exactly), and for each read, what it read and a digest of what that was: of the node it read, of an input's or state's value, of a file's text. The node's own `Digest` is a digest of that derivation. Two nodes of one digest hold one value in any process, *because makes are plain functions of their arguments and what they read*. Purity is what makes this sound, the same discipline again.

A target type with a `Keeper` (the model, `geop_ops_model`'s `NamedModel`, and a placed part's `Drawn`) has its nodes written to a file beside the program, `part.geop.geopc`, when the `Cache` is flushed (`Cache::flush`). A later run takes a node from there instead of making it only when its arguments and every read are the same now. To quote the module's documentation: *what a record says is never trusted, it is checked.* A cache file from another build of the kernel, or another `FORMAT`, is simply not read, since a value made by other code may differ. The `.geopc` file is a pure cache. Deleting it costs a make, and it belongs in no repository (it is in `.gitignore`). `Pool` writes values two records share only once, for instance the faces two revisions of a model have alike.

Counts tell the story here too: `Counts::restored` is the nodes taken from a cache, and `Counts::kept_makes` the makes of kept targets, which reopening an unchanged program must leave at zero. `assert_reopens` in `core/geop-core-program/src/testing.rs` checks exactly this: build in one process, build again in another from what the first kept, and assert that the second makes nothing kept and builds the same part.

### A cache derived from a value is derived one way

A cache file reads values back exactly, which is what exposed this bug. A curve or surface keeps the box of its control points (`aabb`), and compares it along with the rest. `translate` moved the box by the offset instead of boxing the points as moved. That was cheaper, and not the same enclosure, since moving a point rounds it. So a rounded corner patch, translated, compared unequal to itself as read back from the cache, whose reader boxes the points. Every reopened example built "otherwise", as `AGENTS.md` puts it. The box moved was also *narrower* than the points it claimed to hold, which is an honesty bug in the sense of [Lecture 5](./05-honest-numerics.md).

The rule: a cached value computed from others is computed by the one function every constructor uses (`compute_aabb`), never patched along by a shortcut of its own.

### A value read from a file is kept as read

The converse lesson, about the file format. `Pose`'s reader normalized the quaternion it read. Normalizing a sharp quaternion gives an interval. A file holds its midpoint, which reads back as another interval and is written as another midpoint. A program with a placed part's pose in its state was never the same program once saved and loaded, and the press example failed its round trip. Nothing needed the normalization: a quaternion need not be unit to stand for a rotation. So the reader keeps the pose exactly as read, and refuses only what stands for nothing, the zero quaternion.

The general rule, from `AGENTS.md`: reading what was written must give back what was read. A reader that "cleans up" a value makes a file that changes every time it is saved.

## A hash map's order is no order

All of the above assumes that a make is a function of its inputs. A subtle way to violate this without touching any global state is iteration order. `Model`'s entities live in `HashMap`s, whose order differs between two models of the same part. The corner planner of the fillet took a corner's edges in that order, and the ball's patch began at a different contact on every build. The result was *equally correct* each time, but not equal, and the example's round trip (build, save, load, build again) caught it.

So wherever an order reaches the result, sort by the ids, which are given in creation order. The pattern is worth remembering, since the failure looks nondeterministic: when two runs of the same input differ, look for a hash map first.

## What the tests prove

Reuse is an optimization that must not be observable. How can you test that for every operation, past and future, without knowing how any of them reuses anything? By comparing with the definition. `core/geop-core-program/src/testing.rs` provides:

- **`assert_incremental(registry, workspace, programs)`** runs a sequence of programs, one edited again and again, on one part; and each from scratch on a fresh part by a fresh runner. It asserts that both hold equal nodes of every target (`Part::differences`, by each target's `PartialEq`, inside placed parts too) and that steps fail alike. A dependency missed by a read would show as a difference.
- **`edit_sequence(original)`** supplies the edits, as a user makes them: the program as it is, its last step dropped, put back, its first two steps swapped (a failure, where the second reads the first), and put back.
- **`assert_round_trips(registry, workspace, program)`** builds a program, saves it, loads it, asserts the loaded program is equal and saves to the same text, builds what it loaded by a fresh runner, and compares the two parts node by node.
- **`assert_reopens`** as above, across processes.

These tests are generic in the operation, so each new operation gets them for almost nothing, as [Lecture 40](./40-writing-an-operation.md) shows. Note that they compare with equality on enclosures, so if an interval differs at the last bit between two builds, the test fails. That strictness is what caught the `translate` bug above and it is, by now, a large part of what keeps the kernel deterministic.

<div class="recap">

## Recap

- A program is plain data; a part is a memo table of nodes keyed by (target, step), never by position.
- A make is a plain function; everything it reads goes through a `Reader`, which records it. Dependencies are observed, not declared.
- A node is reused iff its arguments are equal and every recorded read holds now. A remake into an equal value keeps its version, which stops the wave.
- Chain targets rebuild what follows them; separate targets are what allow skipping.
- Placed parts are shared builds; dragging one builds nothing, because its pose is state, not part of its build.
- A record kept between processes is never trusted: it is checked, by digests of how it was made.
- A read and its check must take one route; a cached value is computed one way; a reader keeps what it read; an order reaching a result is sorted by id.

</div>

## Exercises

**1.** A program has steps `p` (a parameter `w`), `a` (a sketch reading `w`), `b` (a sketch reading nothing), `c` (extrude of `a`, into the model) and `d` (a cut from the model by `b`). Which makes run when you change `w`? When you change `b`'s arguments? If `a`'s value comes out equal after the change of `w`?

<details class="solution"><summary>Solution</summary>

Changing `w`: the node of `p` differs, so `a` is made; its new value differs, so `c` is made; `c` is a producer of the model chain, so `d`, reading the model, is made. `b` is reused. Changing `b`'s arguments: `b` is made (its arguments differ), then `d` reads a new version of `b`. `c` is reused: it never read `b`. If `a` came out equal, it keeps its version, so `c` and `d` are reused, the early cut-off.

</details>

**2.** In `Part::holds`, the `Target` case compares the producer *and* the version. Since versions are unique across all parts, why compare the producer at all? Find the case in which removing that comparison would change nothing, and say what the comparison documents.

<details class="solution"><summary>Hint</summary>

A version is unique, and a node's version belongs to one node, so an equal version implies an equal producer, unless a node is carried over between builds of a placed part, where "two builds share a version only where one carried the node over". Then read how `Read::Target` is used by `target_of` and in `Listed`: the producer names *who* was seen, which is what an error message and a reordering are about. Treat this as a question to investigate by reading `core/geop-core-program/src/part.rs`, not as one with a one-line answer.

</details>

**3.** Why is `make` a plain function pointer type and not a closure? Write the kind of bug a capturing closure would allow, and say which test would catch it.

<details class="solution"><summary>Solution</summary>

A closure could capture, say, a value computed in `run` from a read that was not recorded: `let w = ctx.reader().retrieve_input("w"); ctx.produce(&t, &(), move |_, _| make_it(w))`. The node records no read of `w` and its arguments do not include it, so changing `w` would reuse the old node. `assert_incremental` catches this, as soon as an edit in its sequence changes `w`: the incremental part differs from the scratch one. (Note that `StepContext::reader` is documented as recording nothing, for a step that declares one node per element of something.) With a plain function, the value has to travel through `args` or through the make's own reader, and both are visible to the rule.

</details>

**4.** A cache record stores a value and its `Derivation`. Explain what would go wrong if the digest of the arguments were computed from their `Display` instead of their `Debug`.

<details class="solution"><summary>Solution</summary>

The `Display` of an interval scalar prints a midpoint to three decimals (the warning in `AGENTS.md`'s debugging section), so two different arguments could have one digest. A run would then take a stale node from the cache: a wrong answer with no error at all. `Debug` writes every number exactly, so the digests differ whenever the arguments do.

</details>

**5.** You add an operation whose make iterates over `model.topology().faces` and takes the first face that passes a test as its "start". `assert_round_trips` fails intermittently. Name the cause and the fix.

<details class="solution"><summary>Solution</summary>

`HashMap` order differs between two models, so "the first" differs between the two builds. Sort by id before taking the first (ids are given in creation order), so that the choice is a function of the model and not of its hash seed. The same bug is told in "A hash map's order is no order" above.

</details>

**6.** Use the demo. Find a sequence of two edits that leaves the model unchanged yet makes more than one step run. Then explain, using the reuse rule, why it is still correct that those steps ran.

<details class="solution"><summary>Hint</summary>

Set the width to 3, then the depth to 1.5 and back. On the way back, `box` and `hole` read the input `depth`, whose value differs from the value the current node recorded, so they run; the values they produce differ from what the node held at depth 1.5, so they get new versions. The memo table holds one node per (target, step), not a history. The rule does not promise to find a value made two edits ago. In the kernel, the editor keeps "the builds of the last few files edited", not the last few edits of a file.

</details>
