# Plan: `geop-ops` as a framework, features as plugins

Backwards compatibility is not a goal (see `AGENTS.md`); the examples and the
full suite must keep working after every step.

## Target

`geop-ops` is the framework relating four things, and knows no feature:

| Thing | Is |
|---|---|
| `Part` | The cached result of running a program: topology, names, the parts placed in it (recursively, each at a 6-D pose), state, parameters, `body_data`, an extension map, and what is worked out of it (view, bounds, mass) |
| `Program` | Takes inputs and state, produces a `Part` |
| `ProgramRunner` | Keeps the `Part` after each step, replays from the middle, skips steps whose inputs did not change |
| Operation / extension | An operation maps `Part` + arguments to a new `Part`; an extension is the state it keeps in the part, for itself and the operations that depend on it |

An operation crate depends in `Cargo.toml` on the crates whose state it
reads. The compiler enforces the layering; `geop-cad-*` depends on all of
them. The state is found by the extension's own type.

The test for what stays in `geop-ops`: *does the framework itself need to
interpret it?* Datums, sketches, mates, cables, threads and features do not.

## Design decisions

- **Type-keyed extension map** (`Part::ext::<E>()`, `Part::ext_mut::<E>()`).
  `E` is created lazily on first `ext_mut`, so a dependency is a Cargo edge
  and a call: nothing to register, nothing to forget. `ext` returns `None`
  for state no operation has written.
- **Extensions travel with the part.** Each entry is a trait object that
  clones itself and presents itself (`Extension::annotations`,
  `Extension::describe`), so `Part::view`, `Part::describe` and the cache work
  on a bare `Part` anywhere. No registry is threaded through callers.
- **Order is by name, never by `TypeId`.** Entries are iterated by
  `Extension::NAME`, so the same program presents identically in every build.
- **`&mut` access resets the cache** (`ext_mut` and every topology mutator):
  a part being changed has nothing stale worked out of it.
- **`RefId` becomes open** once datums and sketches leave: `(kind, id)`, the
  extension owning the kind answers `exists` and `resolve`.
- **Incremental runner**: `h_i = H(h_{i-1}, op, args, parameters read)`, plus a
  content hash of each step's output part for early cutoff. Needs a canonical
  (sorted, bit-exact) hash of a `Part`, hence deterministic builds.

## Steps

1. [x] **Extension map; threads and cables out of `Part`.**
   `Part::ext`/`ext_mut`, `Extension` trait with `NAME`, `annotations`,
   `describe`. `Cable`, `CutWire` move to `geop-ops-harness`, `CosmeticThread`
   to `geop-ops-hole`; their `Part` methods become extension traits there.
   The viewer's `threads` becomes generic `annotations`.
2. [ ] **Cells: reads and writes of a step, detected, and a runner that skips
   what does not read what changed.** See "Cells" below. Replaces the
   prefix-only reuse of `ProgramRunner::run`.
3. [ ] **Constraints solved over the runner.** A part exposes residuals `r`
   and Jacobian `J` over its variables (parameters, instance poses); the
   solver runs the program at `x`, reads `r(x)`/`J(x)` from the final part,
   steps, runs again. `Mate` and the mate-to-residual code move to
   `geop-ops-assembly`; the solver in `geop-ops/src/assembly.rs` becomes
   generic. Guard: `mates_resolved` (80 parts) must keep passing.
4. [ ] **Groups and placed replay.** A feature is a set of steps plus their
   prerequisites (never their dependents); a reference to an entity made by a
   step of the group resolves to the copy's entity (names carry the step:
   `operation_of`), anything else resolves globally. `Operation::apply_placed`
   defaults to `apply`; only operations that introduce a position (sketch
   plane, hole points, datum frame) override it. A placement carries an
   orientation sign (mirror reverses handedness). Pattern replays the group;
   `Feature`/`FeatureTool` and the `pattern` -> `hole` dependency go.
5. [ ] **Open `RefId`; datums, sketches, 3-D sketches** onto extensions.

Each step ends with `cargo test --workspace`, the ignored sweeps if blends,
shells, booleans or the solver were touched, and the web build.

## Cells (step 2)

A `Part` is a set of *cells*: `topology` (one cell), and keyed cells of
`names`, `instances`, `state`, each extension, `body_data`. Every `Part`
accessor logs the cell it touches into a log the runner installs for the
step, so a read cannot be forgotten, and a write cannot be missed.

- A step's *reads* are the cells it looked at, with their content hash; its
  *writes* are the cells it changed, with their new values.
- On a run, a step whose reads all hash as they did last time is not run:
  its recorded writes are applied. Otherwise it runs, and if its writes hash
  as before the steps after it still skip (early cutoff).
- Soundness comes from the log, not from the arguments. Reading what a
  step's arguments name is not enough: a step also reads implicitly (the
  solid it combines with by default), and skipping on a missed read gives a
  wrong part. Reading `topology()` is a read of the whole topology cell,
  which is what chained booleans do.
- Reads of a *collection* (iterate all instances) are a read of every cell
  in it, including cells added later.

Two things stand in the way of independence and have to go first:

- **A global id counter.** `Part::fresh_id` is read and written by every step
  that adds an instance, sketch or datum, which chains them all. Ids derived
  from the entity's name (unique by construction) remove it.
- **`topology()` returns `&Model`.** Fine for logging "read the topology",
  too coarse to say which solid. Acceptable: topology steps are a chain
  anyway. Assemblies, which are what needs the independence, touch
  `instances`, `state` and one extension.

Work to do in this repo first: `program/mod.rs`, `program/library.rs`,
`part/*` have uncommitted changes of the owner; step 2 restructures exactly
those files, so it starts from a commit.
