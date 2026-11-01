# Lecture 40: Writing an operation

<div class="goals">

**After this lecture you can**

- write a new operation from scratch: its arguments, its build, its one form, its registration;
- say where each kind of code belongs, and what the tier rules forbid;
- refer to other steps' work with typed references, and accept one of several kinds of thing with `one_of!`;
- write the four kinds of test every op crate has, and say what each one catches that the others do not;
- explain why a test must drive the editor the way the front end does.

</div>

This lecture is a tutorial. By the end you will have written an operation that compiles against the real kernel. In fact the code below was compiled and its tests were run, against this book's revision of the kernel, in a throwaway crate with path dependencies. It is not part of the repository, because the operation itself is not useful: a square plate. The point is that everything a real operation has is here.

We then read a real one, `ops/geop-ops-boolean`, and see the same shape.

## The shape of an op crate

An operation is a leaf. From `AGENTS.md`'s *Where things go*: an op crate depends on `core/`, `lib/` and `geop-ops-model` only, never on another op crate, and `scripts/check-layering.sh` fails the build otherwise. Here is the rationale for the tiers, each rule named after what broke before it existed:

| Tier | What it knows | Rule |
|---|---|---|
| `core/` | what geometry, a program, a part and an editor are | names no operation |
| `lib/geop-lib-*` | algorithms two operations share | takes geometry, never an `ElementReference` or a `Reader` |
| `ops/geop-ops-model` | the vocabulary operations share: the model, sketch and datum targets and references into them | depends on `core/` only |
| `ops/geop-ops-*` | one family of operations | is a leaf; its public surface is its `lib.rs` |
| `cad/` | composition | one `register` line per crate |

Why so strict? `AGENTS.md` records the history: `geop-cad-base` once held 26 000 lines of every operation's tests and examples, so changing a dialog meant building and testing everything. Nineteen crates once depended on the extrude operation for `Extents` and `Output` that only its examples wrote, so a change to one operation retested 22 of 40 crates. Tier rules are not aesthetics. They decide what recompiles and what retests, and that decides how fast anyone can work.

And an op crate has a fixed layout, so that you always know where to look:

```text
ops/geop-ops-yours/
  src/lib.rs               its public surface, and `register`
  src/operation.rs         the operation
  src/examples.rs          its example programs, behind the feature `examples`
  src/program_tests.rs     programs built, each with a round trip and an incremental build
  src/editor_tests.rs      driven as the front end drives it
  src/regression_tests.rs  reproducers from bug reports, rebuilt in Rust
  src/stress_tests.rs      sweeps, mostly `#[ignore]`d
```

## Step 1: The arguments

Begin with what the user decides. Arguments are saved with the program, so they must be plain, serializable design data. They never hold internal ids, only names.

```rust
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Plate;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlateArgs {
    /// How long its sides are: a number, or a formula of the parameters.
    pub size: NumericInput,
    /// How thick it is.
    pub thickness: NumericInput,
    /// Kept as a new body, or combined with a solid of the part.
    pub combine: Combine,
}
```

`Plate` is a unit struct: the operation itself holds nothing. `NumericInput` (`core/geop-core-program/src/numeric.rs`) is a plain number or a formula of the part's parameters. Take one wherever you take a number, and the user can type `width / 2`. `Combine` (from `geop-ops-model`) is the shared answer to "what happens to the solid you built": kept as a new body, or united, intersected or subtracted with a target.

## Step 2: Declare the operation

`PartOperation` (`core/geop-core-editor/src/part_operation.rs`) is the one trait, and `part_operation!` turns it into both an `Operation` that a program can run and an `Editable` that an editor can open.

```rust
part_operation!(Plate(PlateArgs));

impl PartOperation for Plate {
    const KIND: &'static str = "plate";
    const INFO: OperationInfo = OperationInfo::new(
        "Plate",
        "A square plate on the origin plane.",
        OperationGroup::Solid,
        OperationTier::Menu,
    );
    type Args = PlateArgs;
    type Session = ();

    /// A unit plate a tenth thick, as a new body.
    fn new_args(_: &Context) -> PlateArgs {
        PlateArgs {
            size: NumericInput::Plain(1.0),
            thickness: NumericInput::Plain(0.1),
            combine: Combine::NewBody,
        }
    }

    fn run(ctx: &mut StepContext, args: &PlateArgs) -> GeopResult<()> {
        build_model::<Self>(ctx, args)
    }

    // form: below
}
```

`KIND` is what programs store the step under, forever. `INFO` is all a front end learns about the operation: its label, a sentence, its toolbar group and tier, and optionally `.shortcut("B")` and `.icon(...)` (see `ops/geop-ops-boolean/src/operation.rs`). `new_args` should produce sensible defaults from the part before the step; the boolean picks the two newest solids. `Session` is `()` unless you keep something between events besides the arguments (a sketch keeps the tool in hand and a half-drawn curve).

`run` declares what the step produces, as [Lecture 38](./38-runner.md) explained. Most operations build into the part's model, so they use `build_model` from `geop_ops_model`. It is short and worth reading:

```rust
pub fn build_model<O: ModelOperation>(ctx: &mut StepContext, args: &O::Args) -> GeopResult<()> {
    ctx.produce(&model(), &(ctx.step().to_string(), args.clone()), generate::<O>)
}
```

The make, `generate`, starts from `reader.previous()`, the model as the steps before it left it, and calls your build. The arguments include the step's id, because what a step makes is named after it.

## Step 3: Build it

```rust
impl ModelOperation for Plate {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &PlateArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("plate", step)?;
        let size = args.size.resolve(reader).concerning("size")?;
        let thickness = args.thickness.resolve(reader).concerning("thickness")?;
        // Refuse early, by name, what no solid can be.
        if !(size > 0.0) {
            return Err(
                GeopError::new(format!("a plate's size must be positive, not {size}"))
                    .concerning("size"),
            );
        }
        if !(thickness > 0.0) {
            return Err(GeopError::new(format!(
                "a plate's thickness must be positive, not {thickness}"
            ))
            .concerning("thickness"));
        }
        let corner = |x: f64, y: f64| Vector2::from_array([x, y].map(Scalar::from_f64));
        let outline = [
            corner(0.0, 0.0),
            corner(size, 0.0),
            corner(size, size),
            corner(0.0, size),
        ];
        let [x, y, z] = [0, 1, 2].map(Vector3::axis);
        let origin = Vector3::from_array([Scalar::ZERO; 3]);
        let plane = CoordinateSystem::try_new(origin, x, y, z)?;
        let loops = [SweepLoop::plain(Profile::closed(polygon(&outline)?))];
        let solid_name = args.combine.built_name(&namer);
        let built = extrude(
            model,
            &namer,
            Some(&solid_name),
            &plane,
            &[Scalar::ZERO, Scalar::from_f64(thickness)],
            &loops,
        )?;
        let solid = built
            .solid
            .ok_or_else(|| GeopError::new("extruded as a solid, but none came out"))?;
        combine_tools(model, &namer, step, &[Tool::whole(solid)], &args.combine)?.stopped()?;
        Ok(())
    }
}
```

A number of rules of the kernel are visible in these few lines.

**Everything goes through the reader.** `NumericInput::resolve` takes the reader, so reading a parameter in a formula is a recorded read. The step is rebuilt when, and only when, that parameter changes.

**Names, not ids.** `Namer::new("plate", step)` names everything the operation creates: the solid `plate(p)`, its top `plate(p,end)`, its sides after the profile's curves. A later step can refer to `plate(p,end)`, and the name still means the same thing after an edit of the size. This is what `extrude` names; see [Lecture 26](./26-sweeps.md).

**Refuse what you do not support, early, by name, and say why.** An invalid value never reaches the boolean. And `.concerning("thickness")` tags the error with the field's key, so the dialog shows it under that field ("An error stands beside its cause", `docs/ux.md`). Note that `!(size > 0.0)` is deliberate: it also refuses `NaN`, which a formula can produce.

**Scalars are made with care.** `Scalar::from_f64` is a sharp enclosure of the number. `AGENTS.md`'s *Construction code is sensitive at the last bit* warns against needless arithmetic in code like this: rewriting extrude and revolve as one sweep builder broke six tests through a multiplication by an exact weight of 1. So we do not, for example, build a unit square and scale it.

**A lib does the geometry.** `extrude` is `geop_lib_sweep`'s, and `combine_tools` is `geop_lib_booleans`'s. They take geometry, profiles and planes, and a model. *Resolving references is the operation's job; the algorithm then serves any caller.* This rule exists because the extrusion's algorithm once took a placed sketch and lived with the extrude operation, so the fillet, which sweeps its blend tools, depended on the extrude operation to reach it.

## Step 4: One form, each field with its setter

An `EditForm` (`core/geop-core-editor/src/edit_form.rs`) is a `Form`, built together with what setting each of its fields does.

```rust
fn form<'a>(
    context: &'a Context<'a>,
    args: &PlateArgs,
    _: &FormState<()>,
) -> EditForm<'a, PlateArgs> {
    let before = &context.before;
    let mut f = EditForm::<PlateArgs>::default();
    let size = Number::numeric(&args.size, "size", before, Unit::Length);
    f.numeric("size", size.range(0.0, 10.0), |edit, size| {
        edit.args.size = size
    });
    let thickness = Number::numeric(&args.thickness, "thickness", before, Unit::Length);
    f.numeric("thickness", thickness.range(0.0, 1.0), |edit, t| {
        edit.args.thickness = t
    });
    args.combine
        .fields(&mut f, before, |args, combine| args.combine = combine);
    f
}
```

The key `"size"` is written once, and the setter sits next to it. Compare the design this replaced, which `AGENTS.md` names: a form builder plus a separate `match` over event keys that drifts from the form it answers. `EditForm` makes the drift impossible. The field methods are `number`, `numeric`, `checkbox`, `select`, `actions`, `file`, `color`, `table`, and `pick` and `pick_one` for reference fields.

`form` never fails: its signature does not return a `Result`. An editor needs a dialog most exactly when the arguments do not build, so whatever is wrong is said in it. `context.before` is a `Reader` of the part as it was before the step: that is what a formula's current value and a list of solids come from.

Sub-forms shared across operations follow the same pattern. `Combine::fields` adds the combine fields, with their setters, to any `EditForm`, given a way to set the result into your arguments. So every operation that combines has the same fields for free. This is the sense in which `AGENTS.md` says "never a `show`/`set` pair".

## Step 5: Register

```rust
/// Adds what this crate offers an editor to `r`.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(Plate)
}
```

For a real crate, one line in `cad/geop-cad-base/src/registry.rs` then composes it with the rest (`let r = geop_ops_plate::register(r)?;`), and one entry in the crate table of `DEVELOPERS.md`. No front end changes at all: the web app receives the label, group, dialog and icon from the protocol, as [Lecture 39](./39-editor.md) showed. That the toolbar, palette and dialog appear with no front-end work is a good test of whether you followed the rules.

## Referring to what other steps built

The plate takes no references, since it builds from numbers. Most operations act on something. Look at `BooleanArgs` from `ops/geop-ops-boolean/src/operation.rs`:

```rust
pub struct BooleanArgs {
    pub a: SolidRef,
    pub b: SolidRef,
    pub op: BooleanOp,
}
```

and `SplitArgs`:

```rust
pub struct SplitArgs {
    pub solid: SolidRef,
    pub face: ElementReference<NamedModel, Face>,
}
```

An `ElementReference<Target, Kind>` (`core/geop-core-program/src/element.rs`) names the element of kind `Kind` called `name` in the target `Target`. The kind is a unit struct defined by the crate defining the target, so a reference of the wrong kind does not compile. An operation's form then picks it, with no further code:

```rust
f.pick_one("a", "a", before, Some(&args.a), |edit, a| edit.args.a = a)
 .pick_one("b", "b", before, Some(&args.b), |edit, b| edit.args.b = b)
 .select("op", "op", args.op.value(), BooleanOp::choices(), false, |edit, choice| { ... });
```

The kind says what a click selects, through `SelectableElement` (`core/geop-core-editor/src/selectable.rs`). A solid can be picked by clicking a face of it. Nothing about the boolean is in the picker.

The rule from `AGENTS.md`: a step's arguments refer to what other steps built by typed references, never by bare strings. A saved reference to the model is the element's name alone, `"extrude(box,end)"`, since a part has one model (`HasElements::SINGLETON`).

For a field that accepts one of several kinds of thing, use `one_of!`. It derives how the field is saved, picked and resolved. The book's page on the editor crate has the canonical example, an axis that can be a straight edge or a datum axis:

```rust
one_of! {
    /// What a revolve turns about.
    pub enum AxisRef -> Axis<Design> {
        Edge(ElementReference<NamedModel<Design>, Line>),
        Datum(ElementReference<Datums, Line>),
    }
}
```

The `build` of an operation that can only use the part's own reads with `retrieve_element`, which refuses the rest by name. One that can use an element of a placed part reads it with `retrieve_placed`, and places it where `geop_ops_model::parameters::located` says. And the build of the boolean above reads model names directly: `model.solid_id(args.a.name())?`, since it builds into that same model.

<div class="kernel">

**In the kernel.** Read `ops/geop-ops-boolean/src/operation.rs` in full next: 433 lines, four operations (`Boolean`, `Split`, `Imprint`, `Section`). Its `ModelOperation::build` for `Boolean` is five lines: a `Namer`, two ids, one call to `geop_lib_booleans::boolean`. An operation is mostly form and references. The algorithm is the library's.

</div>

## The tests

Here is the part most tutorials skip, and here it is half the work. The kernel's reliability comes from accumulated tests of four kinds, each with a job. `AGENTS.md`'s *Every op crate tests itself* requires that `cargo test -p <crate>` pass on its own.

### Program tests: what it builds

Build the program, check validity and volume. `geop_ops_model::testing` supplies helpers: look there before writing an `assert_valid` of your own.

```rust
/// The operations these programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?.with(Plate)
}

#[test]
fn a_plate_cut_from_a_block_leaves_the_rest() {
    let mut program = Program::new();
    program.push("box", PrimitiveArgs::block([0.0; 3], [2.0, 2.0, 1.0]));
    program.push(
        "p",
        PlateArgs {
            combine: Combine::cut("block(box)"),
            ..plate(1.0, 0.5)
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_volume(&part, "plate(p)", 4.0 - 0.5, 1e-3);
}
```

Notice what the test starts from: `PrimitiveArgs`, a step of `geop_lib_fixtures`, a block at given coordinates. It does not sketch and extrude. This is deliberate and it matters. A dev-dependency on the sketch or the extrude operation would make this crate's tests run again whenever those operations change, for no reason. `AGENTS.md`: a test that only needs a solid does not sketch and extrude it. `Primitive` is a real, serializable step, so it can be held in a program, whereas a `NamedModel` cannot. (A test of a single step on a model can skip programs altogether: `given_part` and `run_step` in `geop_ops_model::testing`, with `geop_lib_sweep::shapes`.)

A refusal is a behaviour too, and deserves a test:

```rust
#[test]
fn a_flat_plate_is_refused_by_name() {
    let mut program = Program::new();
    program.push("p", plate(1.0, 0.0));
    let error = program.build_model(registry).unwrap_err().to_string();
    assert!(error.contains("thickness must be positive"), "{error}");
}
```

And every program gets the two generic tests from [Lecture 38](./38-runner.md):

```rust
#[test]
fn plates_round_trip_and_build_incrementally() {
    let mut program = Program::new();
    program.push("box", PrimitiveArgs::block([0.0; 3], [2.0, 2.0, 1.0]));
    program.push("p", plate(1.0, 0.5));
    let registry = Rc::new(registry().unwrap());
    assert_round_trips(registry.clone(), None, &program);
    assert_incremental(registry, None, &edit_sequence(&program));
}
```

These two lines catch a forgotten read, a hash-order dependency, and a saved value that reads back differently, for an operation that does not even know the tests exist.

### Editor tests: what the user does

`AGENTS.md` has a section called *Test what the user does, not only what the arguments say*, and its first example is bracing:

> The shell's faces could never be picked: `EntityRef::lies_in` fell back to `self == scope` for every case it did not list, and a face never equals its solid. The shell tests set the face names directly.

Four such bugs passed every operation test because the tests built the arguments directly. So every operation that a user reaches through the editor gets at least one test driving it as the front end does:

```rust
/// What these tests edit: given solids, and plates.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_lib_fixtures::register)
        .and_then(crate::register)
        .expect("the operations each have a name of their own")
}

/// New, a number typed into a field, committed: the step holds the number,
/// and the scene draws the solid.
#[test]
fn a_new_plate_takes_a_typed_size() {
    let mut program = Program::new();
    program.push("box", PrimitiveArgs::block([5.0; 3], [6.0; 3]));
    let mut editor = editor_with(registry(), program);
    handled(&mut editor, Command::New { kind: "plate".into() });
    handled(&mut editor, dialog("size", Value::Number(3.0)));
    let update = handled(&mut editor, Command::Commit);
    assert!(update.program.is_some(), "the program changed");
    let plate: PlateArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(plate.size, NumericInput::Plain(3.0));
    let scene = (editor.handle(Command::Show).scene).expect("the scene is sent");
    assert_eq!(scene.part.solids.len(), 2, "{:?}", scene.part.solids);
}
```

A test with clicks, from `ops/geop-ops-boolean/src/editor_tests.rs`, does the same with pointers:

```rust
handled(&mut editor, Command::New { kind: "imprint".into() });
handled(&mut editor, dialog("face", Value::Press));
handled(&mut editor, click_at(from_above(0.25, 0.25)));
committed(&mut editor);
```

`from_above(x, y)` is a pointer straight down at `(x, y)`, and `click_at` makes it a click event: exactly what the viewport sends. Assert on what the editor sends back: the presentation's `grab`, the scene's solids, the dialog's values. `geop_core_editor::testing` also has `Stepping` for one step edited gesture by gesture. The registry here is built of only the crates the test needs, each crate's `register`, as the real editor's is built.

### Regression tests: a real bug's program, for ever

When a user reports a failure, the first job is to rebuild their case in Rust, in the `regression_tests.rs` of the crate whose operation failed. Never commit a `.geop` file as a fixture: a format change breaks old files, and the rule then is to delete them rather than add a compatibility reader. Confirm that the Rust version still fails without the fix. The doc comment is the permanent record of what was learned. Read the top of `ops/geop-ops-boolean/src/regression_tests.rs`: its comment on `cylinder_cut_from_a_block_far_from_the_origin` states the coordinates, the exact error text, which placements failed and which did not, and the root cause, in some thirty lines. We read that whole story in [Lecture 42](./42-case-studies.md).

### Stress tests: sweep the space

The sweep files (`ops/geop-ops-shell/src/stress_tests.rs`, `ops/geop-ops-fillet/src/stress_tests.rs`) blend every edge and shell every face of a set of bodies (`geop_lib_fixtures::bodies`): boxes, cylinders and spheres, whole, drilled, pocketed, stepped, cut off and with a boss. They distinguish a refusal from a failure (`is_refusal` in the shell's file), and require that whatever is built is valid. `AGENTS.md`: their first run found five shell bugs that no hand-written test had reached. They are slow, so a fast representative subset runs by default and the rest are `#[ignore = "slow: …"]`. Run the ignored set before changing blends, shells, booleans or the solver.

### Tests that need dependents

One trap. The sketch crate's tests extrude what they sketch, and `geop-ops-extrude-revolve` depends on `geop-ops-sketch`. As a unit test, with extrude as a dev-dependency, Cargo links two copies of the sketch crate, and their target types are *different types*: the extrusion cannot read the sketch the test built, and every typed target fails to match. Such tests go in `tests/` (an integration test links one copy), as `ops/geop-ops-sketch/tests/` does. A dependency cycle through a dev-dependency is that copy waiting to happen.

## Examples

An example is a Rust program written against the operations' `Args`. Since it writes other operations' `Args`, it compiles only where they are dependencies, and an operation must not depend on another. So an op crate's `examples.rs` sits behind its feature `examples`, those operations optional dependencies of that feature, as the `Cargo.toml` of `ops/geop-ops-boolean` shows:

```toml
[features]
examples = ["dep:geop-ops-extrude-revolve", "dep:geop-ops-sketch"]
```

and `register` adds the set under `#[cfg(any(test, feature = "examples"))]`. `geop-cad-base` turns the feature on for every crate. `ops/geop-ops-fillet/src/examples.rs` is the smallest example of an example: `rounded_box`. Each one gets a round trip and an incremental build (`ops/geop-ops-boolean/src/example_tests.rs`), which is how a user's first experience stays working.

## A check list

1. Does an existing operation do this, if generalized? `AGENTS.md`: a change that adds a feature should, where it can, make an existing function more general; a change that deletes code for the same behaviour is the better one.
2. Does it branch on surface kinds? It must not: *everything is a NURBS.*
3. Are arguments plain data, with typed references and `NumericInput`?
4. Is what it reads all through the reader?
5. Is what it refuses named, early, with `.concerning(key)` where a field is at fault?
6. Is an order that reaches the result sorted by id?
7. Did you add program, editor, regression and stress tests as they apply, and do they start from `Primitive`?
8. Does `scripts/check-layering.sh` pass? Does `cargo test -p <crate>` pass alone? `scripts/test-affected.sh` is the check before a merge.
9. Did you run every test capped? (`timeout 600 prlimit --as=16000000000 cargo test ...`)

<div class="recap">

## Recap

- An operation is `PartOperation`: `KIND`, `INFO`, `Args`, `new_args`, `run`, one `form`, made an `Operation` and an `Editable` by `part_operation!`. A model step adds `ModelOperation::build`.
- A form's field carries its setter, so a key is written once. Shared sub-forms add fields with setters.
- References are typed and name things; numbers are `NumericInput`; algorithms live in `lib/`.
- `register` is the whole connection to the editor. No front-end code changes.
- Four kinds of test: program (with round trip and incremental), editor (what the user does), regression (a reproducer in Rust), stress (a swept space, mostly ignored).
- The tiers, checked by a script, decide what rebuilds and retests.

</div>

## Exercises

**1.** Extend `Plate` with a boolean field `centred`, which puts the plate's centre at the origin. Write the field, its setter and a test. How many places did you have to edit for the field's key?

<details class="solution"><summary>Solution</summary>

Add `#[serde(default)] pub centred: bool` to the arguments, so a program saved earlier reads as not centred (or leave off the default: this project does not keep old files compatible, per `AGENTS.md`). In `form`: `f.checkbox("centred", "centred", args.centred, |edit, b| edit.args.centred = b);`. In `build`, shift the corners by `-size/2` when set. A test asserts `classify(&part, "plate(p)", [0.0, 0.0, 0.05])` is `PointClassification::Inside`, away from where the uncentred plate's corner would be on its boundary. The key appears once.

</details>

**2.** `Plate` builds at the origin. Make it sit on a datum or planar face instead. Which type do you add to the arguments? Which crate provides it? Which operation's form would you read as a model, and why can your crate not depend on that operation?

<details class="solution"><summary>Hint</summary>

`SketchPlane` is in `geop_ops_model::sketch` (see `ops/geop-ops-sketch` and `book/src/ops/geop-ops-sketch.md`). It is defined in the vocabulary crate precisely so that a crate that reads a plane needs no sketch operation. Read how `ops/geop-ops-reference-geometry` and the sketch operation resolve it. A dependency on another op is what `scripts/check-layering.sh` fails.

</details>

**3.** Which of the four kinds of test would catch each of these bugs? (a) `form`'s setter for `thickness` writes into `size`. (b) `build` reads a parameter by calling a global, not through the reader. (c) The plate with `Combine::cut` on a block whose top is flush with the plate's top leaves a zero-area sliver. (d) `build` chooses the first solid in `HashMap` order as its target.

<details class="solution"><summary>Solution</summary>

(a) An editor test: a program test sets arguments directly, past the setter. (b) `assert_incremental`, once a parameter changes in the sequence. If `edit_sequence` does not change it, add a program to the sequence that does. (c) A program test with `assert_valid`, once written, and a stress sweep over bodies to find it before a user does. (d) `assert_round_trips`, intermittently, since the second build may take another order. If that is how you found it, sort by id, and add the regression test.

</details>

**4.** Explain in terms of Cargo why `geop-ops-sketch`'s tests that extrude must live in `tests/`, and then find one in `ops/geop-ops-sketch/tests/`.

<details class="solution"><summary>Hint</summary>

With a unit test, `src/` is compiled with `cfg(test)` as one crate, while the dev-dependency `geop-ops-extrude-revolve` links the normal library build of `geop-ops-sketch`. These are two crates to rustc, so a target type in one is not the same type as in the other. An integration test links only the library.

</details>

**5.** Read `impl Inspector for MassProperties` in `ops/geop-ops-inspect/src/lib.rs`. An inspector is not an operation. List three things in the `PartOperation` trait that it has no analogue for, and say why.

<details class="solution"><summary>Hint</summary>

It has no `Args` saved with a program, no `run` producing targets and no `form` with setters: exporting and inspecting change no program and are nothing to undo. It has `INFO`, its own `Options`, `inspect` and `show`, as in `core/geop-core-editor/src/output/inspect.rs`.

</details>
