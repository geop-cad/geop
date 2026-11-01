# Building a CAD kernel

This book is a lecture series on how to build a modern CAD kernel. It starts from a linear algebra course and a little programming, and it ends with you able to read, debug and extend a real boundary-representation kernel: its intervals, its NURBS, its booleans, its sketch and assembly solvers, its fillets and shells, and the incremental build that runs them all.

The kernel it teaches is [Geop](https://geop-cad.dev), an open-source kernel in Rust. Every statement this book makes about the kernel is meant to hold against the code in this repository, and paths and function names are cited so that you can go and read them: `core/geop-core-geometry/src/aabb.rs`, `compute_aabb`. Where something is not implemented, the book says so. The kernel changes faster than any text, so if a citation has gone stale, the code is right. Please tell us on the [Discord](https://discord.gg/btRDgrcwJ5).

## Who it is for

You should know:

- **Linear algebra**: vectors, matrices, solving systems, rank, the idea of a null space.
- **Calculus**: derivatives, Taylor expansion, the chain rule, partial derivatives. Newton's method helps.
- **Some programming**: you can read code in a C-like language. Rust is used in the excerpts, but none of its finer points are needed for the first eight parts. Reading it will come with practice.

You do not need to know anything about CAD, computational geometry or numerical analysis. Everything is built up from there: what a CAD kernel is, what floating point does, what a Bézier curve is. The book is meant to be read in order, since later lectures use what earlier ones established. A reader with a CAD background might skim Part 0 and start with [Lecture 3](./lectures/03-floating-point.md), as the interval discipline shapes everything that follows.

## How the lectures are organised

Each lecture opens with what you will be able to do after it, then develops the idea from first principles with worked examples. Boxes marked **In the kernel** point at where it lives in the code. A recap and exercises close it, with solutions or hints folded away. Try the exercise before you open the solution: much of the understanding in this subject is in the attempt.

| Part | Lectures | What it establishes |
|---|---|---|
| [0. What we are building](./lectures/01-what-a-kernel-is.md) | 1 to 2 | what a kernel is; a part as a program of steps |
| [I. Numbers you can trust](./lectures/03-floating-point.md) | 3 to 5 | floating point, intervals, and the rules of honest numerics |
| [II. Curves and surfaces](./lectures/06-bezier.md) | 6 to 11 | Bézier, B-splines, NURBS, subdivision, surfaces, evaluating relative to the patch |
| [III. Intersection](./lectures/12-subdivide-to-isolate.md) | 12 to 16 | isolate by subdivision, refine by Newton, tangency and overlap, tracing |
| [IV. Topology](./lectures/17-brep.md) | 17 to 20 | the B-rep, poles and seams, inside or outside, validation |
| [V. Booleans](./lectures/21-remesh.md) | 21 to 22 | the remesh pipeline and classification |
| [VI. Sketches](./lectures/23-sketch-solver.md) | 23 to 25 | constraint solving, rank, a proof by Krawczyk |
| [VII. Operations](./lectures/26-sweeps.md) | 26 to 35 | sweeps, lofts, fillets, one rebuild, offsets, patterns, sheet metal, subdivision surfaces, STEP, drawings |
| [VIII. Assemblies](./lectures/36-mates.md) | 36 to 37 | mates, poses, the solver, freedom |
| [IX. The program model](./lectures/38-runner.md) | 38 to 40 | the runner, the editor and its protocol, writing an operation |
| [X. The craft](./lectures/41-performance.md) | 41 to 42 | counting work, and case studies in debugging |

The appendices hold [references](./appendix/references.md), a [glossary](./appendix/glossary.md) and a [map of the crates](./appendix/crates.md), the last with a reference page for many of them. For a generated API reference, see [docs.geop-cad.dev](https://docs.geop-cad.dev).

If you are short of time, here is a thread through the book that carries the essential idea: Lectures [4](./lectures/04-intervals.md), [5](./lectures/05-honest-numerics.md), [6](./lectures/06-bezier.md), [12](./lectures/12-subdivide-to-isolate.md), [14](./lectures/14-newton.md), [17](./lectures/17-brep.md), [21](./lectures/21-remesh.md) and [42](./lectures/42-case-studies.md).

## The demos

Many pages have interactive figures. They run in your browser with no network: drag a control point, slide a knot, press *Step* to see one iteration of an algorithm. Each runs the mathematics it shows, in a few hundred lines of plain JavaScript in `book/src/demos/`, and where it is simpler than what the kernel does, its caption says how. The pure mathematics of each demo is tested against its formula by `book/scripts/test_demo_math.mjs`. On a phone, a touch on a handle drags it, and a touch elsewhere scrolls the page. On a keyboard, focus a figure and press `n` or `p` to choose a point, and the arrow keys to move it. Animations do not start by themselves if your system asks for reduced motion.

Here is one of the figures that shows a real part as the real kernel meshed it. The other figures like it link to the example in the web app, where the kernel itself runs in your browser as WebAssembly.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"rounded_box","example":"rounded_box"}'>
<figcaption>A box with every edge rounded, built by Geop from the example <code>rounded_box</code>: a sketch, an extrusion and a fillet. Drag to turn it.</figcaption>
</figure>

## Running the kernel

You can do almost everything in this book without building anything:

- **In the browser**: [app.geop-cad.dev](https://app.geop-cad.dev), the kernel as WebAssembly. Open an example with a link, such as [`?example=rounded_box`](https://app.geop-cad.dev/?example=rounded_box).
- **On the command line**: `curl -fsSL https://geop-cad.dev/install.sh | sh` installs `geop` (on Windows, `irm https://geop-cad.dev/install.ps1 | iex`). Then `geop examples --out-dir examples` writes every built-in example as a program and a mesh, `geop compile part.geop` builds a program, and `geop open my-project` edits a folder in the browser, on the kernel running on your machine.
- **For an agent**: `geop mcp` runs the editor as a Model Context Protocol server, described in `docs/mcp.md`.
- **From source**: this is a Cargo workspace. Whenever you run something that searches geometry, cap it, as this repository's rule demands:

  ```sh
  timeout 600 prlimit --as=16000000000 cargo test -p geop-core-geometry
  ```

  A search that runs away does not stop on its own; it grows until the machine runs out of memory.

## Notation

We use the following throughout.

| Symbol | Meaning |
|---|---|
| `$C(t)$` | a curve at parameter `$t$` |
| `$S(u,v)$` | a surface at parameters `$u,v$` |
| `$P_i$`, `$w_i$` | a control point, its weight |
| `$N_{i,p}$` | the B-spline basis function of degree `$p$` |
| `$B_i^n$` | the Bernstein polynomial of degree `$n$` |
| `$[a,b]$` | an interval: every real number between `$a$` and `$b$`, or the enclosure that stands for them |
| pcurve | a curve in a surface's `$(u,v)$` plane |
| `Scalar` | the kernel's interval number |
| `` `name` `` | something in the code, or the name of an element such as `extrude(box,end)` |

The kernel's three-valued comparisons appear often: `could_be_equal`, and `definitely_greater`. Lecture 4 defines them. In Rust excerpts, `?` passes an error up to the caller, and `GeopResult<T>` is either a `T` or an error with its chain of context.

## A word on how this was made

Much of this book's most useful material comes from `AGENTS.md` in the repository: a running account of what went wrong, written by those who chased it. When a lecture says that a number was "measured", it comes from there, or from a capped run made while writing the lecture, and the lecture says which. The kernel's mistakes are taught here on purpose. They are the fastest way to understand why it is built the way it is.

Questions are welcome on the project's [Discord](https://discord.gg/btRDgrcwJ5).
