# geop-core-program

The program machinery every part is built by, independent of what is built:
a program is a list of steps, each an `Operation`; running it builds a
`Part`, a memo table of nodes. A run is a function of what it is given: its
**inputs** — what its parameters are set to — and its **state** — where its
placed parts are, how its joints are turned — which no step changes: a
solver moves the state until the constraints the steps define hold (see
[geop-lib-constraints](../lib/geop-lib-constraints.md)). The one thing it
reads besides is its **workspace**: the programs it places and the files
it imports.

Nothing here knows a model, a sketch or a datum. Each of those is a target
some crate defines, together with the kinds of element it holds; this crate
only says what a target, an element and a step are, and runs programs.

## The run model: nodes, reads, reuse

The `target`, `part` and `reader` modules. A `Part` is a memo table with
one kind of node: what one step produced of one target, keyed by the
step's id and the target's name (`TargetReference<T>`: a stable name and
the type of its value — the model, `sketch:s1`, `datums`), never by a
position. A step only *declares* what it produces:
`ctx.produce(&target, &args, make)` (`StepContext::produce`), where `make`
is a plain function `fn(&Reader, &Args) -> GeopResult<T>` that captures
nothing, so everything it needs is in its arguments. The node is computed
at once, so a later node of the same step can read it, unless it can be
reused. A step runs no geometry outside a make.

Every read through the `Reader` a make is given is recorded on its node
(`Read`): which producer's version of a target it saw, that no producer
was visible, the value of an input or of the state, a file's revision,
which targets of a type there were, and a read inside a placed part. A node
is reused iff its arguments are equal and every recorded read holds now.
So **a make runs when its node is new, its arguments differ, or something
it read changed; nothing else ever runs.** A node made again into a value
equal to the last (by `PartialEq`) keeps its version, so what read it is
reused in turn. Versions are stamps unique across all parts, so two builds
of a placed part share a version only where one carried the node over from
the other.

The producer a step sees of a target is the latest step before it, in the
current program's order, that produces it. A target several steps produce
— the model, the datums, the parameters' values — is a **chain**: its type
says so (`Target::CHAIN`), and each producer starts from what the one
before left (`Reader::previous`, the default where none did). A second
producer of a target that is no chain — a sketch, a placed part — is an
error naming both steps. Inserting, removing or reordering steps changes
which producer a reader sees, and that is caught by the recorded reads, not
by positions shifting: two independent steps swapped are both reused.

A part is read only through a `Reader`. An editor reads through one too,
recording nothing: the part now (`Part::current`), or before or after one
step of the last run (`Part::before`, `Part::after`). `Reader::targets_of`
lists the targets of one type, in the order their steps produced them; the
listing is a read of its own. A node that failed holds why, and a node
reading it fails with that message in context.

Inputs and state values are read by name through the reader
(`retrieve_input`, `retrieve_state`). A state value the run gives none of
reads as its default, the number zero: the state is declared by reading
it, not given up front.

## Values

The `values` module. What a run is given is the program's, and its file
format's: a
`Setting` — what a parameter is set to where a program places the part, a
number or a text (a table's row, a colour), saved as itself; and a
`StateValue` — a number or a `Pose` — which steps by what is added to it,
a pose by a twist in its own frame through the exponential map.
`Settings` and `StateValues` are them by name; `pose_state` names the
state where an instance is (`bolt.pose`).

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

## Placed parts

The `placed` module. A step places the part another program builds as a
node of its own (`place(ctx, name, &PlacedArgs { file, inputs })`, whose
make is `build_placed`), whose value is the built part (`PlacedPart`, an
`Rc<Part>`). The make reads the program through its reader (a file read,
in the workspace's revision of that file) and builds it with `inputs` and
the slice of the placing part's state under the instance's name — the
state `/bolt:b.pose` of the part is the state `b.pose` of the part placed as
`bolt` — the program's own where the part gives none. What a placed part is
built from is exactly that, so equal builds are shared through the
runner's build cache: a thousand screws of one size are one build, a
thousand `Rc` clones. A placed part whose state changed is built again
from a clone of the part it was, so only what reads what changed is made
again; the result is a new build. The cache is swept at the end of every
top-level run of what no part holds any more. Where a placed part goes is
no concern of its build: its pose is the placing part's state (see
`geop_ops_model::parameters::instance`), so dragging it builds nothing.

- An element of a placed part is named by the instance's path before its
  target's name: `/bolt:model`, `/asm/bolt:model` for one of a placed
  part's placed part, `/:model` for one of the part itself (`Located`:
  `INSTANCE_SEPARATOR` between instances, `TARGET_SEPARATOR` before the
  name, which may hold `:` itself). A name not of that form is refused,
  and an instance's name may hold neither. A pick names it so, and every
  `ElementReference` can hold one: a picker never asks where an element
  is from. `Reader::retrieve_element` resolves elements of the part
  itself, and refuses one of a placed part by name;
  `Reader::retrieve_placed` resolves either, as it is in its own part,
  with the instance's path (`InInstance`).
- `Reader::placed_part(path)` reads the part placed at a path, as it was
  built. What it reads is recorded as read inside the placed part
  (`Read::Inside`): a mate reading the second link of an arm is made again
  when that link changed, not when anything in the arm did.
  `Reader::instances` lists the parts placed in the part itself.
- A state behind an instance's path — `/bolt:b.pose` — is read as the
  placing part gives it where it does, else inside the placed part
  (`Reader::given_state`), so what is read is always what the placed part
  is built with.
- Files must not place themselves: the runner carries the stack of files
  being built and refuses a cycle, naming it.

## Operations and programs

The `operation`, `operation_registry` and `program` modules. An `Operation` is a unit struct: its `KIND` (the name its steps are stored
under), its `Args` — plain, serializable design data that names targets and
elements, never internal ids — and `run(ctx, args)`, which declares what
the step produces through its `StepContext`: the step's id, the runner
running it, and `produce`. `StepArgs` are arguments that say which
operation they are for, so a program written in Rust pushes them without
naming the operation again.

A `Program` is what a `.geop` file holds, `{"steps": [...], "state":
{...}}`: an ordered list of `ProgramStep`s, each an operation's kind, its
arguments as saved JSON and an id —
`{"id": "box", "operation": "extrude", "args": {...}}` — and the state it
was saved with: the state a run of it is at, which a solve moves
(`Program::run`). Everything a step creates is named after its id, so ids
are unique and spelled in an alphabet a name can be built from
(`validate_operation_id`); steps refer to what earlier steps built only by
name. A program is plain data: it is copied for undo, compared for change
and saved as it is, and it means the same thing every time it runs,
including after a round trip through JSON (`from_json`, `to_json`).
`push(id, args)` appends a step for a program written in Rust, whose
arguments name their operation; `fresh_id` gives an id no step has;
`excerpt` and `paste` copy steps between programs with the state stored
under their ids. `validate(&registry)` checks a program against the
operations it is run in — every step of an operation there is, its
arguments read by it, its id valid and unique — naming the step: the
editor checks what it loads, pastes and commits, the workspace every
program it reads.

An `OperationRegistry` holds the operations a program may use, by kind
(`with`; a kind twice is refused). Which operations an application offers
is its own choice, so the registry is made there: the editor's is
`registry` in [geop-cad-base](../cad/geop-cad-base.md), made from the
operations it edits (see
[geop-core-editor](./geop-core-editor.md#operation-sets)).

## Running a program

The `runner` module. A `Runner` runs programs, with its registry's
operations, on a part it is given; it holds no part itself, only its
workspace, its location (the file it runs and the files being built
around it) and the build cache it shares with the runners of the parts it
places. `run(part, program, upto)` runs the steps before the step `upto` —
all of them for `None` — at the program's state; `run_with` gives the
run's inputs too. The nodes of steps after `upto` are kept, to be reused
when they run again; those of steps no longer in the program are dropped
at the end of the run. A failed step does not stop the run: later steps
that do not depend on it still build, and the `RunOutput` lists the
failures in program order. `run_step(part, step, body)` runs one step by
hand, as a test of one operation does. `counts()` says how many makes ran,
how many of them placed parts, and how many placed parts were built rather
than taken from the cache: what a test of what runs counts, never a time.

## The workspace

The `workspace` module. A `Workspace` is where programs come from: a
program by its file's path (a `Program`, with the state it was saved
with), another file's text, the list of files, and how paths name
each other (`resolve`, `relative`). It says how often a file changed
(`revision(path)`): a node that read a file is made again once its
revision changes. A runner holds one, and the `Location` of the program it
runs: its file, and the files placing it. A program never knows where it
is: the runner resolves a file the program names from its location, and
refuses one placing the program it runs, however deep — the files would
place themselves — naming the cycle (`Runner::program`). A make reads a
file through its reader (`Reader::read_file`, `Reader::program`), which
records the read.

`Files` is the workspace of a set of files read from a `FileStore` (in
memory, on disk, or another store decorating one, as the standard parts
are): each file read once, read again once it changes (`Files::write`,
counted by the revision), each program checked against the operations it
is written in. A file of a library every workspace shares is named by a
scheme, `std:hex_nut.geop` (`is_shared`), the same from anywhere.

## Kept between processes

The `cache` and `kept` modules. Given a `CacheStore` (a folder on disk,
for the command line or an editor's host), a workspace's `Cache` keeps
what its programs built beside each program's file, `part.geop.geopc`,
so that reopening a program whose steps and inputs did not change builds
nothing.

Every node knows how it was made in terms that mean the same in another
process: a digest of its arguments (by their `Debug`, which writes every
number exactly), and of each read what it read and a digest of that — the
node it read, an input's or the state's value, a file's text. Its own
digest is one of that `Derivation`. A target type with a `Keeper` (the
model, a placed part's drawing) has its nodes written to the cache file
when the cache is flushed (an editor does when an edit is done, the
command line when it has built). A run takes a node from there instead of
making it when its arguments and every read are the same now, and only
then: a record is never trusted, it is checked. What is made of a part
outside its program, a drawer's drawing, is kept the same way
(`made_kept`). Values two records hold alike — the faces two revisions of
a model share — are written once (`Pool`). A cache file is of one build
of the kernel and one `FORMAT`; any other is not read.

`assert_reopens` (feature `testing`) builds a program in one process and
again in another from what the first kept, and asserts the second makes
nothing kept and builds the same part.

## Numbers as formulas

The `numeric` module. A `NumericInput` is a number an operation is given —
a length, an angle, a count — a plain value or a formula of the program's
parameters: numbers, `+ - * / ^`, parentheses, `pi` and the functions
`sqrt abs sin cos tan asin acos atan round floor ceil min max`
(`FUNCTION_NAMES`), angles in degrees (`evaluate`). A step resolves it
where it builds (`NumericInput::resolve`), reading each parameter the
formula names — the target a step of the program defines for it
(`parameter`, a `ParameterValue`: a number, or a text — the row a table
looked up; a table's columns are parameters of their own, `size.d`;
`read_parameter`) — so a change to one of them rebuilds the step
and a change to any other does not. Saved, a formula is told apart from
every other value in a step's arguments (`formulas_in`,
`rename_parameter_in`), so what reads a parameter is found, and renamed,
without asking the operation. `ParameterValues` is every parameter as it
came to, a chain every step defining parameters adds to
(`read_parameter_values`: none, for a part defining none). What
defines parameters is an operation of
[geop-ops-model](../ops/geop-ops-model.md#parameters-and-properties-parameters); how a dialog shows a
number is [geop-core-editor](./geop-core-editor.md#numbers-as-formulas)'s.

## Testing

With the feature `testing`, `testing::assert_incremental(registry,
workspace, programs)` runs a sequence of programs — one edited again and
again — on one part, and each from scratch on a fresh part by a fresh
runner, its placed parts built anew, and asserts that both hold equal
nodes of every target (`Part::differences`, by each target's `PartialEq`,
inside placed parts too) and fail alike: what makes reusing nodes
trustworthy, checked without knowing how an operation reuses anything.
