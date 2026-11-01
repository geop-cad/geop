# A. References

These are the books and papers the lectures draw on, with what each is for. The kernel's own lessons are in `AGENTS.md`, which is the primary source for everything this book says about why the code is as it is.

## Numerics

- **David Goldberg**, "What Every Computer Scientist Should Know About Floating-Point Arithmetic", *ACM Computing Surveys* 23(1), 1991. Rounding, ulps, why equal is not what it seems. For [Lecture 3](../lectures/03-floating-point.md).
- **Ramon E. Moore, R. Baker Kearfott, Michael J. Cloud**, *Introduction to Interval Analysis*, SIAM, 2009. Interval arithmetic, the dependency problem, the mean value form, interval Newton and Krawczyk's operator. For [Lecture 4](../lectures/04-intervals.md), [Lecture 11](../lectures/11-evaluate-locally.md), [Lecture 24](../lectures/24-rank-and-proof.md).
- **Rudolf Krawczyk**, "Newton-Algorithmen zur Bestimmung von Nullstellen mit Fehlerschranken", *Computing* 4, 1969. Where the fixed-point proof the sketch solver uses comes from. For [Lecture 24](../lectures/24-rank-and-proof.md).

## Curves and surfaces

- **Les Piegl, Wayne Tiller**, *The NURBS Book*, 2nd edition, Springer, 1997. The standard reference for algorithms: knot insertion, global interpolation, surface constructions. The kernel's `interpolate` follows it. For [Lectures 7 to 10](../lectures/07-bsplines.md).
- **Gerald Farin**, *Curves and Surfaces for CAGD: A Practical Guide*, 5th edition, Morgan Kaufmann, 2002. The clearest treatment of the geometry behind the algorithms. For [Lecture 6](../lectures/06-bezier.md) to [Lecture 10](../lectures/10-surfaces.md).
- **Rida T. Farouki, T. N. T. Goodman**, "On the optimal stability of the Bernstein basis", *Mathematics of Computation* 65(216), 1996. Why the Bernstein form is the numerically right basis. For [Lecture 6](../lectures/06-bezier.md).

## Intersection and interrogation

- **Thomas W. Sederberg, Tomoyuki Nishita**, "Curve intersection using Bézier clipping", *Computer-Aided Design* 22(9), 1990. The clipping idea behind [Lecture 13](../lectures/13-fat-lines.md).
- **Nicholas M. Patrikalakis, Takashi Maekawa**, *Shape Interrogation for Computer Aided Design and Manufacturing*, Springer, 2002. Intersection, offsets, distance functions. For [Part III](../lectures/12-subdivide-to-isolate.md) and [Lecture 31](../lectures/31-offsets.md).

## Solid modelling

- **Martti Mäntylä**, *An Introduction to Solid Modeling*, Computer Science Press, 1988. B-rep data structures and Euler operators. For [Lecture 17](../lectures/17-brep.md).
- **Christoph M. Hoffmann**, *Geometric and Solid Modeling: An Introduction*, Morgan Kaufmann, 1989. Robustness and booleans, from the era in which these questions were first faced squarely. For [Lecture 21](../lectures/21-remesh.md) and [Lecture 22](../lectures/22-boolean.md).

## Sweeps and subdivision surfaces

- **Wenping Wang, Bert Jüttler, Dayue Zheng, Yang Liu**, "Computation of rotation minimizing frames", *ACM Transactions on Graphics* 27(1), 2008. The double reflection method. For [Lecture 27](../lectures/27-path-sweep.md).
- **Edwin Catmull, Jim Clark**, "Recursively generated B-spline surfaces on arbitrary topological meshes", *Computer-Aided Design* 10(6), 1978. For [Lecture 34](../lectures/34-subd.md).
- **Jos Stam**, "Exact evaluation of Catmull-Clark subdivision surfaces at arbitrary parameter values", *SIGGRAPH* 1998. For [Lecture 34](../lectures/34-subd.md).
- **Charles Loop, Scott Schaefer**, "Approximating Catmull-Clark subdivision surfaces with bicubic patches", *ACM Transactions on Graphics* 27(1), 2008. For [Lecture 34](../lectures/34-subd.md).

## In this repository

- `AGENTS.md`: the rulebook, with the stories behind each rule. [Lecture 42](../lectures/42-case-studies.md) retells many of them.
- `DEVELOPERS.md`: the crates, how an operation plugs in, CI, releases, the command line.
- `ARCHITECTURE_PLAN.md`: the plan for the tiers, which `scripts/check-layering.sh` checks.
- `docs/smlib-comparison/`: a comparison of this kernel with SMLib, area by area (`docs/smlib-comparison/booleans.md`, `docs/smlib-comparison/direct.md`, `docs/smlib-comparison/fillets.md`, `docs/smlib-comparison/geometry.md`, `docs/smlib-comparison/offsets.md`), the source for which *configurations* a mature kernel handles.
- `docs/mcp.md`: driving the editor as an agent. `docs/ux.md`: the rules of the editor's interaction.
- [docs.geop-cad.dev](https://docs.geop-cad.dev): the generated API reference.
