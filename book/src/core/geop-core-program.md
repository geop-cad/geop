# geop-core-program

The program machinery every part is built by, independent of what is built:
a program is a list of steps, each an `Operation`; running it builds a
`Part`, target by target, reusing every version whose arguments and reads
did not change. The values a run is given are its **inputs** (the
parameters, numbers) and its **state** — where its placed parts are, how
its joints are turned — which no step changes: a solver moves the state
until the constraints the steps define hold (see
[geop-ops-constraints](../ops/geop-ops-constraints.md)).

Nothing here knows a model, a sketch or a datum. Each of those is a target
some crate defines, together with the kinds of element it holds; this crate
only says what a target, an element and a step are, and runs programs.

## Targets and the part

The `target` and `part` modules. A `Part<I, S>` is a set of **targets**, each named by a
`TargetReference<T>` — a stable name and the type of its value: the model,
`sketch:s1`, `datums`. A step defines a target (`Part::define_target`) or
revises one an earlier step made (`Part::revise_target`) with a
*generator*, a plain function `fn(&Reader, &Args) -> GeopResult<Out>` that
captures nothing, so everything it needs is in its arguments. Every
revision is a version of its own; a read sees the latest version made
before it in the same run, and a generator revising a target reads what it
revises with `Reader::previous`.

A part is read only through a `Reader`, which records every read: a version
is reused from the last run when its step made it from equal arguments and
nothing it read has changed since, and a version rebuilt into a value equal
to the last one counts as unchanged, so what read it is reused in turn. An
editor reads through a reader too, recording nothing: the part now
(`Part::current`), or before one step of the last run (`Part::reader`).
`Reader::targets_of` lists the targets of one type, newest last — the
sketches, say.

Inputs (`Input`) and state values (`State`) are read by name through the
reader (`retrieve_input`, `retrieve_state`). A state value the run gives
none of reads as its default: the state is declared by reading it, not
given up front.

## Elements

The `element` module. A target is read by its **elements** — an edge or a face of a topology, a
line of a sketch — never as a whole. An `ElementReference<T, K>` names the
element of kind `K` called `name` in the target `T`: what an operation is
given to build on, and what a pick in the viewport becomes. The crate that
defines a target defines the kinds of element it holds (unit structs
implementing `Kind`) and how each resolves (`HasElements<K>`); an operation
that builds on one depends on that crate and names the kind in its
arguments' types, so a reference of the wrong kind does not compile, and no
crate needs to know every kind there is. `Reader::retrieve_element`
resolves one.

- A target a part has only one of — the model, the datums — is a
  *singleton* (`HasElements::SINGLETON`), and a reference into it is saved
  as the element's name alone: `"extrude(box,end)"`. Any other is saved as
  `{"target": "sketch:s1", "name": "c3"}`.
- A target meant to be read whole — a sketch, to sweep — is a kind of its
  own, its one element the empty name (`ElementReference::whole`, saved as
  `{"target": "sketch:s1"}`; `element::whole` resolves it).
- A `PickedElement` is an element as a front end names it: a kind, a
  target and a name, with no types. Only `ElementReference::from_picked`
  reads one, so an operation never matches on it.

## Operations and programs

The `operation`, `operation_registry` and `program` modules. An `Operation` is a unit struct: its `KIND` (the name its steps are stored
under), its `Args` — plain, serializable design data that names targets and
elements, never internal ids — and `run(step, part, args)`, which defines
and revises the step's targets. `StepArgs` are arguments that say which
operation they are for, so a program written in Rust pushes them without
naming the operation again.

A `Program` is an ordered list of `ProgramStep`s, each an operation's kind,
its arguments as saved JSON and an id —
`{"id": "box", "operation": "extrude", "args": {...}}`. Everything a step
creates is named after its id, and steps refer to what earlier steps built
only by name. A program is plain data: it is copied for undo, compared for
change and saved as it is, and it means the same thing every time it runs,
including after a round trip through JSON.

An `OperationRegistry` holds the operations a program may use, by kind
(`with`; a kind twice is refused). Which operations an application offers
is its own choice, so the registry is made there: the editor's is
`registry` in [geop-cad-base](../cad/geop-cad-base.md), made from the
operations it edits (see
[geop-core-editor](./geop-core-editor.md#operation-sets)).

## Running a program

The `program_runner` module. A `ProgramRunner` runs programs on one part with a registry. Every run
begins with the **given step** (`Given`), at position 0 before the
program's own: it defines what a run is given that is no number — what the
program says of its part, the files around it, the library of parts it
places, the origin (see `geop_cad_base::part::given`). So the step at
position `k` of a program is at position `k + 1` of the part
(`ProgramRunner::position`).

`run(program, inputs, state)` runs it all; `run_until(program, end, ...)`
runs the first `end` steps, as the editor does up to its marker, keeping
what the later steps made to reuse when they run again. A failed step does
not stop the run: later steps that do not depend on it still build, and the
`RunOutput` lists the failures in program order. Since every version is
reused while nothing it read changed, a run after an edit rebuilds only
what reads what the edit changed — after a parameter changed, only the
steps whose formulas read it.
