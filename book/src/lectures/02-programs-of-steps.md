# Lecture 2: A part is a program of steps

<div class="goals">

**After this lecture you can**

- read a `.geop` file: its steps, their arguments, and the names they refer to;
- explain why a parametric CAD system stores a *history* and not only a result;
- derive the name of an entity from how it was made, and explain the topological naming problem and why Geop's names are not ids;
- run a program with `geop compile`, change a dimension and rebuild it, and read the error when a step fails;
- say at a high level what a typed `ElementReference` is, and what happens in a rebuild.

</div>

Before we descend to numbers and geometry for the next many lectures, let us see the whole system working once. We will take one small part, read the whole file that defines it, and run the real kernel on it. Everything we say here will be taken apart later.

## Why store a program?

An engineer designs a bracket: a plate, four holes, a rounded corner. Next month the plate has to be 2 mm longer, and the holes should stay 10 mm from the edge. If all you have saved is the resulting solid, you have 40 faces, and you must find and shift a dozen of them by hand. And the fillet's surface has to change along with them.

So mechanical CAD stores the *history*: the steps that made the part, with their dimensions. Change a dimension and run again. This is called **parametric modelling**, and every serious system does it. Geop takes this idea further than usual and makes it literal: the document is a program, and the solid is what that program outputs. There is no other model file. Well, there is a cache next to it, but that can be deleted at will.

A program being plain data brings consequences, all good:

- A `.geop` file is JSON text. It diffs in version control and can be written by a script or a language model. There is an MCP server for the latter (`geop mcp`, `docs/mcp.md`).
- Undo is keeping old copies of the list.
- Rebuilding after an edit can reuse everything not affected ([Lecture 38](./38-runner.md)).
- "What does this part mean" has one answer, the same in the browser, on the command line and in a test.

From `core/geop-core-program/src/program.rs`, a program is little more than this:

```rust,ignore
pub struct ProgramStep {
    pub id: String,
    pub operation: String,
    pub args: serde_json::Value,
    pub active: Option<String>,
}

pub struct Program {
    pub steps: Vec<ProgramStep>,
    pub state: StateValues,
    pub display: Display,
}
```

A step has an id you choose, the kind of operation it runs, and its arguments as they are saved. The optional `active` is a formula deciding whether the step runs at all, so a program can have configurations. `state` holds where placed parts are and how joints are turned, which matters in assemblies ([Lecture 36](./36-mates.md)). `display` is how the user chose to show what the program builds. Both are left out of the file when empty.

## A whole program, read line by line

This is the example `rounded_box`. We wrote it to disk with `geop examples`, and you can see the source of it, which is a Rust function writing the same three steps, as `rounded_box` in `ops/geop-ops-fillet/src/examples.rs`. Below is its whole `.geop` text, elided only where it repeats.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"rounded_box","example":"rounded_box"}'>
<figcaption>What the program below builds, as a mesh the kernel wrote. Open it in the editor to see the three steps in the timeline, and drag the timeline back to see the part after one and two steps.</figcaption>
</figure>

### Step 1: a sketch

```json
{
  "steps": [
    {
      "id": "outline",
      "operation": "add_sketch",
      "args": {
        "plane": { "datum": "origin:xy" },
        "sketch": {
          "points": {
            "0": { "x": 9.857072060165695e-19, "y": 7.191076644733477e-19 },
            "1": { "x": 1.9999999999999998,    "y": 8.217957318638042e-19 },
            "2": { "x": 1.9999999999999998,    "y": 1.5 },
            "3": { "x": 1.1259343236666495e-19, "y": 1.5 }
          },
          "curves": {
            "4": { "type": "Line", "start": 0, "end": 1, "construction": false },
            "5": { "type": "Line", "start": 1, "end": 2, "construction": false },
            "6": { "type": "Line", "start": 2, "end": 3, "construction": false },
            "7": { "type": "Line", "start": 3, "end": 0, "construction": false }
          },
          "constraints": {
            "8":  { "type": "Fix", "point": 0, "x": 0.0, "y": 0.0 },
            "9":  { "type": "Horizontal", "line": 4 },
            "10": { "type": "Horizontal", "line": 6 },
            "11": { "type": "Vertical", "line": 5 },
            "12": { "type": "Vertical", "line": 7 },
            "13": { "type": "Length", "curve": 4, "value": 2.0 },
            "14": { "type": "Length", "curve": 5, "value": 1.5 }
          },
          "next_id": 15
        }
      }
    },
```

(We have reordered the keys and joined lines for reading; the real file has them sorted.)

The first step, with id `outline`, adds a sketch on the plane `origin:xy`. A part starts with a datum called `origin`, whose planes one can sketch on. A sketch has **points**, **curves** between them, and **constraints** among them. Each has an id, and they share one counter (`next_id`). Here are four lines around four corners. The constraints say what the sketch *means*: corner 0 is fixed at the origin, two lines are horizontal, two vertical, and two lengths are `2.0` and `1.5`. That is exactly enough to determine all four corners, with no freedom left over ([Lecture 23](./23-sketch-solver.md)).

Now look at the numbers in `points`. Corner 0 is at `$x=9.857\times10^{-19}$`. Corner 1 is at `$x=1.9999999999999998$`, not 2. These are the values where the constraint solver left them when the example was made (`solved` in `ops/geop-ops-model/src/sketch/figures.rs` calls `Sketch::solve`). Nothing was wrong: the solver stopped when the constraints held to its accuracy. This small untidiness in the very first file we opened is a preview of Part I, which explains why you must never test whether `x` is `2.0`, and of what to do instead.

### Step 2: an extrusion

```json
    {
      "id": "box",
      "operation": "extrude",
      "args": {
        "sketch": { "target": "/:sketch:outline" },
        "extent": {
          "side1": { "blind": 1.0 },
          "reversed": false,
          "symmetric": false
        },
        "output": { "solid": { "mode": "new_body" } }
      }
    },
```

The second step, `box`, extrudes the sketch. `"sketch": {"target": "/:sketch:outline"}` is a **reference to what an earlier step built**, and it is worth parsing. `sketch:outline` names a *target*, a named product of a step, here the sketch made by the step `outline`. The prefix `/:` says "of the part itself". An element of a part placed in this one would read `/bolt:…` instead, so a picker never needs to ask where an element is from ([Lecture 36](./36-mates.md)).

`extent` says how far: blind, a distance of `1.0` on one side. `output` says what to do with the result: make a new body rather than add it to or subtract it from an existing solid. If `mode` were a boolean, this is where the extrusion would call the machinery of [Lecture 22](./22-boolean.md).

### Step 3: a fillet

```json
    {
      "id": "round",
      "operation": "fillet",
      "args": {
        "edges": [
          "extrude(box,outline,c4,start)",
          "extrude(box,outline,c4,end)",
          "extrude(box,outline,c5,start)",
          "extrude(box,outline,c5,end)",
          "extrude(box,outline,c6,start)",
          "extrude(box,outline,c6,end)",
          "extrude(box,outline,c7,start)",
          "extrude(box,outline,c7,end)",
          "extrude(box,outline,p0)",
          "extrude(box,outline,p1)",
          "extrude(box,outline,p2)",
          "extrude(box,outline,p3)"
        ],
        "radius": 0.2
      }
    }
  ]
}
```

The third step rounds twelve edges by `0.2`. Read the names of those edges, because they are the main idea of this lecture.

## Names say how a thing came to be

Every vertex, edge, face and solid of a Geop model has a stable text name, of the form

```text
kind(operation, arg, ...)
```

where `kind` is the operation that created it, `operation` the id of the program step that ran it, and the arguments identify the entity within that step. Our extrusion of the sketch `outline` in the step `box` makes:

| Name | What it is |
|---|---|
| `extrude(box)` | the solid |
| `extrude(box,start)`, `extrude(box,end)` | its two flat caps |
| `extrude(box,outline,c4)` | the wall swept by sketch curve 4 |
| `extrude(box,outline,c4,start)` | the edge along curve 4 on the start cap |
| `extrude(box,outline,c4,end)` | the edge along curve 4 on the end cap |
| `extrude(box,outline,p0)` | the upright edge swept by sketch point 0 |

So the twelve edges in the fillet step are the four edges around the bottom, the four around the top, and the four uprights. No one had to label them. They are built from the program's own vocabulary: `c4` is curve 4 of the sketch, `p0` point 0. And `outline` in the name is a *scope* (`Namer::scoped` in `ops/geop-ops-model/src/names.rs`).

The rule, from the docs of `ops/geop-ops-model/src/lib.rs`:

> A name says *how an entity came to be*, never *when*: it is built only from inputs that stay the same when a part is rebuilt after an edit upstream — operation ids, sketch element ids, the names of the entities an operation consumed, and positions counted along those. Never from an internal id or the order in which an algorithm happened to create things.

### The topological naming problem

Why this much fuss? Inside the model, entities do have ids: `FaceId(42)` and so on. Consider a step whose arguments said "round edge number 17". Now the user adds a hole in an earlier step. The hole's edges get made first, ids shift, and edge 17 is now something else. The fillet moves to a nonsensical place, or silently changes the design. Or say a boolean rearranges a model so that the same face is now split in two. Which piece is the "same" face?

This is the **topological naming problem**. Every parametric system runs into it, and it is notorious among users of systems that handle it badly. Ids are positions in a history, and an upstream edit changes the history. Geometry cannot serve as an address either ("the edge nearest `$(2,1.5,0.5)$`"), because the whole point of the edit is to move the edges.

What remains is a description of **derivation**. "The edge swept by point 0 of the outline in the step box" remains true whatever the height of the box. Each operation has to define how the names of its outputs derive from the names of its inputs. For a boolean, for example ([Lecture 22](./22-boolean.md) has the full table), with `N` standing for the step's name:

- a piece of edge `E` starting at vertex `V` is named `N(E,V)`, and the first piece keeps the name `E`;
- an edge traced along faces `F1 < F2` from `P` to `Q` is `N(F1,F2,P,Q)`;
- a vertex where edges `E1 < E2` cross is `N(E1,E2,i,n)`, the `i`-th of `n` crossings along `E1`.

There are some traps in making it work. Three of them, from `AGENTS.md`:

- *Order must not reach a name.* A model's entities live in `HashMap`s, whose iteration order differs from one run to the next. A corner planner took a corner's edges in that order, and the ball's patch began at a different contact on every build. The test caught it by building a program, saving it, loading it and building it again. Wherever order matters, sort by something stable.
- *Names may grow.* A piece of a split edge is named after the edge and the vertex it starts at, and that vertex after the edge again, so a name doubled per cut. Twenty cuts made names of tens of megabytes. An argument longer than `LONGEST_ARGUMENT` is now spelled by its digest.
- *A trimmed face is still that face.* A boolean splitting a face gives one piece the name `N(F,E)` and the other keeps `F`, whichever one the splice happened to leave it on. If the boolean then keeps only one piece, that piece is the face, trimmed, and takes the name back (`name_lone_pieces`, in `lib/geop-lib-booleans/src/naming.rs`). Otherwise a later draft "about the bottom face" could no longer find it because of an irrelevant detail of the construction.

### Typed references

A reference in Rust is not a bare string. When a step refers to a target's element, its type says which target and which kind of element: `ElementReference<NamedModel, Edge>` is an edge in the model target. Here is the head of the fillet's arguments, from `ops/geop-ops-fillet/src/operation.rs`:

```rust,ignore
pub struct FilletArgs {
    /// The edges to round, all of one solid.
    pub edges: Vec<ElementReference<NamedModel, Edge>>,
    /// The radius: a number, or a formula of the part's parameters.
    pub radius: NumericInput,
    // ... end_radius, vertex_radii, setback
}
```

A reference of the wrong kind doesn't compile. In the file it is saved in the shortest unambiguous form. There is only one model in a part, so a reference into it is saved as the element's name alone; a sketch is saved as `{"target": "/:sketch:outline"}`. Note too that `radius` is a `NumericInput`, "a number, or a formula": the `0.2` in the file could have been `"thickness / 2"`, had the part defined parameters.

The types live in `core/geop-core-program/src/element.rs`. The crate defining a target (say the sketch vocabulary in `ops/geop-ops-model`) defines the kinds of element it holds, through the trait `HasElements`. The core crate knows no specifics: `core/geop-core-program` knows nothing about models and sketches at all.

## Running it

The `geop` command line is the shortest way to run a program. It is built from `cad/geop-cad-cli`. Install it with the script on the [home page](https://geop-cad.dev) or from a checkout by `cargo install --path cad/geop-cad-cli`, or just open the example in the [web app](https://app.geop-cad.dev/?example=rounded_box), in which everything we do here can be done by clicking.

Here is the session we ran for this lecture, with a release build of the kernel at version 0.12.0. You have to save the program above as `rb.geop` first; `geop examples --only rounded_box` writes it for you.

```text
$ geop compile rb.geop -o rb.stl
3 steps, 866484 bytes -> rb.stl
```

Time: 0.7 s, for 17 328 triangles. The bytes are `$84+50\cdot17\,328$`, the format of a binary STL file. Other extensions select other exporters: `step`, for instance. `AGENTS.md` asks that every run be capped in memory and time, so in the repository this looks like `timeout 120 prlimit --as=6000000000 target/release/geop compile rb.geop`. We will say why in [Lecture 41](./41-performance.md).

### Changing a dimension

Now the point of the whole exercise. Change the extrusion from `"blind": 1.0` to `"blind": 2.0`, and nothing else:

```text
$ geop compile tall.geop -o tall.stl
3 steps, 866484 bytes -> tall.stl
```

The twelve edge names have kept their meaning: the fillet found the same edges of a taller box and rounded them. We measured both meshes:

| Program | `$z$` extent | Mesh volume | Exact volume |
|---|---|---|---|
| `rb.geop` | 1.0 | 2.85566 | 2.85620 |
| `tall.geop` | 2.0 | 5.82118 | 5.82186 |

The difference between the mesh volumes, `$2.9655$`, is the mesh's estimate of the rounded rectangle's area, whose exact value is `$3-(4-\pi)(0.2)^2=2.9657$`. Both meshes happen to have 17 328 triangles, which is why the byte counts agree. The exact values come from exercise 3 of [Lecture 1](./01-what-a-kernel-is.md).

### Renaming a step

A step's id is part of every name it creates. If you rename the step `box` to `body`, you must rename the references too:

```json
{ "id": "body", "operation": "extrude", ... },
{ "id": "round", ... "edges": ["extrude(body,outline,c4,start)", ...] }
```

This builds the identical solid: same bounding box, same volume in our measurement. There is no magic behind a name. It is text, chosen to say something true.

### When a step fails

What about a design the kernel can't build? Set the radius to `2.0`, more than half the box's size:

```text
$ geop compile big.geop -o big.stl
error: GeopError
Backtrace: disabled backtrace
RootError: the blend is too large for face FaceId(42): it would meet the face's surface beyond its end
Context:
where the corner's ball touches face FaceId(42), at Vector { data: [Scalar { lo: -7.99360577730113e-14, hi: 7.99360577730113e-14 }, Scalar { lo: 1.9999999999999398, hi: 2.0000000000000613 }, Scalar { lo: -1.0169642905566435e-13, hi: 1.0191847366058938e-13 }] }
Context:
rounding the corner at vertex "extrude(box,outline,p1,start)"
Context:
fillet(round, FilletArgs { edges: [edge("/:model", "extrude(box,outline,c4,start)"), ...
Context:
making target `model` in step `round`
Context:
step "round"
```

Read this carefully, because it is the kernel's main way of talking to you. It is a chain, from the root cause up through the stack of what was being done:

- The **root** says what went wrong and why ("the blend is too large for face … it would meet the face's surface beyond its end").
- The first **context** gives the numbers. Note that the point is not printed as `(0, 2, 0)` but as an enclosure of it: `$x\in[-8\times10^{-14},8\times10^{-14}]$`. That is what Part I is about. Those are not rounding-error garbage; each coordinate is a proven bound on the true value.
- The next ones say which corner, by stable name, then which step and which of its targets.

We will explain this style of error, and why it is permanent code and not a debugging aid, in [Lecture 5](./05-honest-numerics.md).

If instead a reference names something that isn't there, say `extrude(box,outline,p9)`:

```text
RootError: no entity is named "extrude(box,outline,p9)" (names of "box": ["extrude(box)", "extrude(box,end)", "extrude(box,outline,c4)", ...
```

The error lists names a step of that id did make. So a refusal comes with enough information to fix the program, which is especially useful when the one reading it is a script or an agent. In an editor, a step that fails does not stop the others: later steps that do not depend on it still build, and the run lists all failures in program order (`RunOutput` in `core/geop-core-program/src/runner.rs`).

Also notice that next to each program the run left a file ending in `.geopc`. That is the cache of what the program built, read back on the next run when everything it depends on is the same ([Lecture 38](./38-runner.md)). It can be deleted at any time and is never committed to the repository.

<div class="kernel">

**In the kernel.** Three things in `core/geop-core-program` to know about now.

A **target** is a named product of steps, and `Part` (`core/geop-core-program/src/part.rs`) is a memo table of **nodes**, one for each step and target. A step only *declares* what it produces, by `StepContext::produce`. That takes the target's reference, the arguments, and a plain function `Make<Args, Out>` that captures nothing, so everything it needs is in its arguments. Every read this function makes through its `Reader` is recorded on the node.

Which gives the rule of the runner: **a make runs when its node is new, its arguments differ, or something it read changed; nothing else ever runs.** This is why a program can be rebuilt cheaply after an edit, and it's the topic of [Lecture 38](./38-runner.md).

Step ids are validated by `validate_operation_id`: ASCII letters, digits, `_`, `-`, `.`, so that names built from them can be parsed apart. Names are built by `Namer` in `ops/geop-ops-model/src/names.rs`.

</div>

## Other ways to run a program

All of these drive one and the same editor (`core/geop-core-editor`) over one registry of operations:

- `geop examples` writes every built-in example, as program and mesh. `--only NAME` selects some.
- The **web app** at [app.geop-cad.dev](https://app.geop-cad.dev) runs the kernel as WebAssembly in a browser. A link `https://app.geop-cad.dev/?example=rounded_box` opens that example. [Lecture 39](./39-editor.md) shows how.
- `geop open DIR` edits the files of a folder in the browser, with the kernel running natively on your machine.
- `geop mcp` runs the editor as a Model Context Protocol server, so an agent can add steps, read which ones failed, look up names and export the result. Tools and protocol are in `docs/mcp.md`. Setting it up is one command: `claude mcp add geop -- geop mcp`.
- From Rust, `Program::push` appends a step, which is how the examples are written. See `rounded_box` in `ops/geop-ops-fillet/src/examples.rs`.

You will find it helpful to keep one of these open while reading. Each lecture from here on can be tried, by changing a number and seeing what happens.

<div class="recap">

## Recap

- A Geop part is a program: an ordered list of steps, each an operation with plain, saved arguments. The solid is its output.
- Steps refer to earlier results by stable names built from how an entity was made: `extrude(box,outline,c4,end)`. Never by internal ids, position in a list or location in space.
- This is the kernel's answer to the topological naming problem. Each operation defines how its outputs' names derive from its inputs' names.
- Failures come back as a chain from the root cause through what was being done, naming entities and printing enclosures.
- Runs are memoized by what each step actually read, so editing a program rebuilds only what depends on the edit.

</div>

## Exercises

**1.** Without running anything: what are the names of the top cap, the wall along the sketch's line 6, and the edge between those two in the program above? What name would the side wall have had if the extrude's step id were `slab`?

<details class="solution"><summary>Solution</summary>

`extrude(box,end)`, `extrude(box,outline,c6)`, and `extrude(box,outline,c6,end)` (the extrude starts on the sketch plane, so the end cap is the top). With the step called `slab`, the wall is `extrude(slab,outline,c6)`.

</details>

**2.** Open the example in the [web app](https://app.geop-cad.dev/?example=rounded_box), or use `geop compile` on a modified file. Change `"value": 2.0` in constraint 13 to `3.0`. Which of the names in the fillet step change? Why does the fillet still work, though every wall moved?

<details class="solution"><summary>Solution</summary>

None. Names depend on sketch curve ids, not on where the curves are. The rebuild runs the sketch with the new dimension, whose solver moves the points; the extrude reads the sketch and is made again, and so is the fillet, which finds its twelve edges by the same names on a wider box.

</details>

**3.** Suppose a kernel names edges by the index they got on creation, and you insert a hole before the fillet in `rounded_box`. Describe what happens. Now suppose it names them by their midpoint's coordinates and you change the box's width. Describe what happens. What property does a good naming scheme need that both fail?

<details class="solution"><summary>Solution</summary>

With indices, the hole's edges take some of the indices, and the fillet rounds the wrong edges, or an index no longer exists. With coordinates, the moved edges no longer match, and the reference is dangling or matches a different edge by coincidence. Both depend on something the upstream edit changes. A good name depends only on what stays the same under edits: step ids, sketch element ids, the names of entities consumed.

</details>

**4.** Can stable names be fully stable? Suppose you delete sketch curve 4 and the point it shares with curve 5, and connect things differently. What happens to `extrude(box,outline,c4,start)`? What should the kernel do: guess, or refuse? What does Geop do?

<details class="solution"><summary>Solution</summary>

That edge no longer exists, because what it derives from no longer exists. A guess could silently change the design. Geop refuses: `no entity is named "…"`, with the names that exist. That is one failed step, reported with the argument at fault, and steps not depending on it still build. This is the honest limit of derivation-based naming: it doesn't make an edit that destroys a feature keep working, it only prevents edits that do not destroy it from changing what a reference means.

</details>

**5.** Read `validate_operation_id` in `core/geop-core-program/src/program.rs`. Why can a step id not contain a comma or a parenthesis? Give an example of two different entities whose names would collide if it could.

<details class="solution"><summary>Solution</summary>

Names are parsed: `operation_of` in `ops/geop-ops-model/src/names.rs` takes the text between `(` and the first `,` or `)` as the step's id. With an id `a,b`, the name `extrude(a,b,end)` could be the end cap of step `a,b` or an entity `b,end` of step `a`.

</details>

**6.** The sketch's stored `x` of `1.9999999999999998` is one unit in the last place below 2 (you will define that in the next lecture). Suppose that a later step, written by a careless programmer, tested `if x == 2.0`. What would happen on this file? And what if the solver had happened to stop exactly on 2.0 in the previous version of the kernel?

<details class="solution"><summary>Hint</summary>

The test is false now, and may have been true before: the program's meaning would depend on the last bit of a solver's last iteration. That is a bug in the test, not in the solver. The next three lectures are about what to write instead.

</details>
