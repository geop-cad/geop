# B. Glossary

Terms as this book and the kernel use them, with the lecture that teaches each. Names in code spans are in the code.

**Affine invariance.** Transforming a curve or surface by an affine map is the same as transforming its control points. [Lecture 6](../lectures/06-bezier.md).

**Apex.** A point where a cone's generators meet, where the surface has no tangent plane. Different from a pole of a smooth surface. [Lecture 31](../lectures/31-offsets.md).

**Assembly.** A part made by placing other parts and constraining them with mates. [Lecture 36](../lectures/36-mates.md).

**AABB (axis-aligned bounding box).** The box of a curve or surface's control points, the cheapest form of the convex hull property (`compute_aabb`). [Lecture 6](../lectures/06-bezier.md).

**B-rep (boundary representation).** A solid described by its boundary: surfaces trimmed by curves, joined by shared edges and vertices. [Lecture 17](../lectures/17-brep.md).

**B-spline.** A piecewise polynomial curve with local control, defined by a degree, a knot vector and control points. [Lecture 7](../lectures/07-bsplines.md).

**Basis function.** A function `$N_{i,p}$` that scales control point `$i$`. B-spline basis functions are non-negative, sum to one, and vanish outside a few spans. [Lecture 7](../lectures/07-bsplines.md).

**Bernstein polynomial.** `$B_i^n(t)=\binom{n}{i}t^i(1-t)^{n-i}$`, the basis of a Bézier curve. [Lecture 6](../lectures/06-bezier.md).

**Bézier curve.** A polynomial curve written in the Bernstein basis, built by repeated linear interpolation. [Lecture 6](../lectures/06-bezier.md).

**Blend.** A surface rounding or bevelling between faces: a fillet or a chamfer. [Lecture 29](../lectures/29-fillets.md).

**Body.** A solid or a sheet of faces standing on their own. [Lecture 17](../lectures/17-brep.md).

**Boehm's algorithm.** Knot insertion: adding a knot without changing the curve, using the `$\alpha=(t-a)/(b-a)$` that compounds width if `$t$` is wide. [Lecture 9](../lectures/09-subdivision.md).

**Boolean.** Union, intersection or difference of two solids. [Lecture 22](../lectures/22-boolean.md).

**Boundary, outer boundary, hole.** A face has exactly one outer boundary and any number of holes, by type (`BoundaryType`, `BoundaryIndex`). [Lecture 18](../lectures/18-poles-seams-splicing.md).

**Branch and bound.** A search that bounds groups of candidates from below and an attained solution from above, discarding what cannot win (`nearest::least_distance`). [Lecture 12](../lectures/12-subdivide-to-isolate.md).

**Build cache.** The runner's table of built placed parts, so that equal builds are shared. [Lecture 38](../lectures/38-runner.md).

**Cache (`.geopc`).** What programs built, kept beside their files between processes (`Cache`). A record is never trusted, but checked. [Lecture 38](../lectures/38-runner.md).

**Catmull–Clark.** A subdivision scheme for quad meshes whose limit is a bicubic B-spline surface away from extraordinary points. [Lecture 34](../lectures/34-subd.md).

**Chain (target).** A target several steps produce, each starting from what the previous left, such as the model (`Target::CHAIN`). [Lecture 38](../lectures/38-runner.md).

**Chamfer.** A blend that is a bevel rather than a round. [Lecture 29](../lectures/29-fillets.md).

**Classification.** Deciding if a point is inside, outside or on a face or solid. [Lecture 19](../lectures/19-classification.md).

**Clipping.** Removing the parts of a parameter domain that a bound proves cannot hold a solution. [Lecture 13](../lectures/13-fat-lines.md).

**Coedge.** One side of an edge on one face: holds a sense, a pcurve and its place in the loop. [Lecture 17](../lectures/17-brep.md).

**Coincidence.** Two curves or surfaces overlapping along a stretch, rather than crossing (`Intersections::Coincident`). [Lecture 15](../lectures/15-tangency-and-overlap.md).

**`Combine`.** How a step's tools meet the part: as a new body, or united, cut or intersected with a target. [Lecture 40](../lectures/40-writing-an-operation.md).

**`Command`.** What a front end sends the editor. [Lecture 39](../lectures/39-editor.md).

**Compatible curves.** Curves brought to one degree and one knot vector, so that they can be skinned. [Lecture 28](../lectures/28-lofts-and-fills.md).

**Constraint.** A residual that must vanish: a distance, an angle, a tangency, a mate. [Lecture 23](../lectures/23-sketch-solver.md), [Lecture 36](../lectures/36-mates.md).

**Control point, control polygon.** The coefficients of a spline, and the polygon that joins them. [Lecture 6](../lectures/06-bezier.md).

**Convex hull property.** A Bézier or B-spline curve (with positive weights) lies in the hull of its control points: a finite proof about infinitely many points. [Lecture 6](../lectures/06-bezier.md).

**Coons patch, Gordon surface.** Surfaces built from their boundary curves, or a network of curves. [Lecture 28](../lectures/28-lofts-and-fills.md).

**Corrector.** The Newton iteration that brings a predicted point back to the curve. [Lecture 16](../lectures/16-tracing-and-pcurves.md).

**Counts.** How much work a runner did (`Counts`). [Lecture 41](../lectures/41-performance.md).

**CSG (constructive solid geometry).** A solid represented as a tree of booleans. [Lecture 1](../lectures/01-what-a-kernel-is.md).

**De Boor's algorithm.** Evaluation of a B-spline by repeated interpolation, generalizing de Casteljau's. [Lecture 7](../lectures/07-bsplines.md).

**De Casteljau's algorithm.** Evaluation of a Bézier curve by repeated linear interpolation. The triangle also gives the split. [Lecture 6](../lectures/06-bezier.md).

**Definitely, could be.** The three-valued comparisons on intervals: `definitely_greater`, `could_be_equal`. [Lecture 4](../lectures/04-intervals.md).

**Degenerate coedge.** A coedge backed by a vertex: a loop running along a row of a surface collapsed to a point. [Lecture 18](../lectures/18-poles-seams-splicing.md).

**Degree.** The polynomial degree of a spline. A curve with `$k$` control points and degree `$p$` has `$k+p+1$` knots. [Lecture 7](../lectures/07-bsplines.md).

**Dependency problem.** Interval arithmetic treats two occurrences of a variable as independent: `$x-x$` over `$[0,1]$` gives `$[-1,1]$`. [Lecture 4](../lectures/04-intervals.md).

**Derivation.** How a node was made, in terms that mean the same in any process (`Derivation`). [Lecture 38](../lectures/38-runner.md).

**Double reflection.** A method to carry a frame along a curve that nearly minimizes rotation. [Lecture 27](../lectures/27-path-sweep.md).

**Draft.** Tilting a face away from a direction, so that a part can leave its mould. [Lecture 30](../lectures/30-one-rebuild.md).

**`Drawer`.** What draws a crate's targets into the part's view. [Lecture 39](../lectures/39-editor.md).

**Early cut-off.** A make that produces an equal value keeps its version, so that what read it is reused. [Lecture 38](../lectures/38-runner.md).

**Edge.** A 3-D curve trimmed to its domain, between a start and an end vertex. [Lecture 17](../lectures/17-brep.md).

**`EditForm`.** A form built together with what setting each of its fields does. [Lecture 40](../lectures/40-writing-an-operation.md).

**`ElementReference`.** A typed name of an element of a target, such as a face of the model. [Lecture 40](../lectures/40-writing-an-operation.md).

**Enclosure.** An interval or box guaranteed to contain the true value. The kernel's fundamental numerical object. [Lecture 4](../lectures/04-intervals.md).

**Epsilon.** A magic tolerance in a comparison. The kernel forbids them. [Lecture 5](../lectures/05-honest-numerics.md).

**Evaluate relative to the patch.** Computing `$A/W$` around a sharp local control point, so width does not grow with distance from the origin (`spline::centered`). [Lecture 11](../lectures/11-evaluate-locally.md).

**Extrude.** A sweep along a straight line. [Lecture 26](../lectures/26-sweeps.md).

**Face.** A surface with a trim: one outer boundary and holes. [Lecture 17](../lectures/17-brep.md).

**Fat line.** A strip enclosing a curve's control polygon about a chord, used to clip another curve's parameter range. [Lecture 13](../lectures/13-fat-lines.md).

**Fillet.** A blend rounding an edge: the surface a ball sweeps as it rolls along two faces. [Lecture 29](../lectures/29-fillets.md).

**Floating point.** The machine's approximate numbers. `f64` has 53 bits of significand. [Lecture 3](../lectures/03-floating-point.md).

**Foot point.** The nearest point of a surface to a target; the `$(u,v)$` that `NurbSurface::project` finds. [Lecture 14](../lectures/14-newton.md).

**Frame.** A moving coordinate system along a path. [Lecture 27](../lectures/27-path-sweep.md).

**Free choice.** A value where any point of its interval serves equally well, such as a subdivision parameter. Only such a value may be sharpened. [Lecture 5](../lectures/05-honest-numerics.md).

**Freedom.** The directions in which an assembly can move while its constraints hold, to first order (`geop_lib_constraints::solve::freedom`). [Lecture 37](../lectures/37-assembly-solver.md).

**Front end.** What draws the kernel's output and forwards input: the web app, VS Code. [Lecture 39](../lectures/39-editor.md).

**Gauss–Legendre rule.** A quadrature with `$n$` points, exact for polynomials of degree `$2n-1$`. [Lecture 41](../lectures/41-performance.md).

**Gauss-Newton.** Newton's method with the second derivatives dropped. Over a curved surface it overshoots. [Lecture 14](../lectures/14-newton.md).

**Gizmo.** A handle a step being edited can show for a drag in 3-D (`core/geop-core-editor/src/editing/gizmo.rs`). [Lecture 39](../lectures/39-editor.md).

**Green's theorem.** Turns an area integral into a line integral around the boundary, for signed area and mass properties. [Lecture 19](../lectures/19-classification.md).

**Handles.** Interactive points of a step's visual. [Lecture 39](../lectures/39-editor.md).

**Hodograph.** The curve of a curve's derivative. [Lecture 6](../lectures/06-bezier.md).

**Homogeneous coordinates.** `$(wx,wy,wz,w)$`: a rational curve is a polynomial curve here, projected. [Lecture 8](../lectures/08-nurbs.md).

**Imprint.** Dividing faces along where another body crosses them, without removing anything. [Lecture 21](../lectures/21-remesh.md).

**Inspector.** A question answered about what a program built. It changes nothing. [Lecture 39](../lectures/39-editor.md).

**Interval arithmetic.** Arithmetic on ranges, with outward rounding, giving enclosures of all possible results. [Lecture 4](../lectures/04-intervals.md).

**Isolate.** To separate solutions into boxes each holding one, by subdivision, before refining. [Lecture 12](../lectures/12-subdivide-to-isolate.md).

**Kept.** A node written to a cache file, and read back only if everything it read reads the same (`core/geop-core-program/src/kept.rs`). [Lecture 38](../lectures/38-runner.md).

**Knot, knot vector.** The non-decreasing parameter values at which a spline's polynomial pieces join. [Lecture 7](../lectures/07-bsplines.md).

**Knot insertion.** See Boehm's algorithm.

**Krawczyk operator.** A fixed-point test on a box which, when it succeeds, proves a unique solution inside. [Lecture 24](../lectures/24-rank-and-proof.md).

**Loft.** A surface or solid skinned through several sections. [Lecture 28](../lectures/28-lofts-and-fills.md).

**Loop.** A closed cycle of coedges bounding a face. Not an entity of its own. [Lecture 17](../lectures/17-brep.md).

**Make.** The plain function that produces a node's value. [Lecture 38](../lectures/38-runner.md).

**Manifold.** Locally like a surface: an edge of a solid borders exactly two faces. [Lecture 20](../lectures/20-validation.md).

**Mate.** A constraint between two placed parts. [Lecture 36](../lectures/36-mates.md).

**MCP (Model Context Protocol).** The protocol over which an agent drives the editor, `geop mcp`. [Lecture 39](../lectures/39-editor.md).

**Mean value form.** An enclosure `$f(c)+f'(X)(X-c)$`, tighter than direct evaluation on small boxes. [Lecture 11](../lectures/11-evaluate-locally.md).

**Memo table.** See Part.

**Model.** The target holding the B-rep with stable names (`NamedModel`). [Lecture 2](../lectures/02-programs-of-steps.md).

**Multiplicity.** How many times a knot repeats. Each repeat lowers the continuity at that knot by one. [Lecture 7](../lectures/07-bsplines.md).

**Namer.** What names everything an operation makes after the step. [Lecture 22](../lectures/22-boolean.md), [Lecture 40](../lectures/40-writing-an-operation.md).

**Newton's method.** The local method: polishes an isolated solution quadratically. [Lecture 14](../lectures/14-newton.md).

**Node.** One entry of a part: what one step produced of one target. [Lecture 38](../lectures/38-runner.md).

**NURBS (non-uniform rational B-spline).** The representation of every curve and surface in this kernel: a B-spline in homogeneous coordinates. [Lecture 8](../lectures/08-nurbs.md).

**Offset.** The surface at a constant distance along a surface's normal. [Lecture 31](../lectures/31-offsets.md).

**`one_of!`.** Declares a field that takes one of several kinds of reference. [Lecture 40](../lectures/40-writing-an-operation.md).

**Operation.** A kind of step, with its arguments, build and form (`PartOperation`). [Lecture 40](../lectures/40-writing-an-operation.md).

**Panel.** Program-level state a front end shows beside the steps, by key. [Lecture 39](../lectures/39-editor.md).

**Parity.** Whether a ray crosses a boundary an odd or even number of times, which decides inside from outside. [Lecture 19](../lectures/19-classification.md).

**Part.** What running a program produces: a memo table of nodes, not a solid (`Part`). [Lecture 38](../lectures/38-runner.md).

**Partition of unity.** Basis functions summing to one, which is what affine invariance and the hull need. [Lecture 6](../lectures/06-bezier.md).

**Pattern.** Copies of a body or feature along a line, a circle or a curve. [Lecture 32](../lectures/32-patterns-holes-threads.md).

**Pcurve.** A curve in a surface's `$(u,v)$` plane, representing an edge on that face. [Lecture 16](../lectures/16-tracing-and-pcurves.md).

**Pendant.** A block of a system whose constraints only hang it off others, solved after them, alone. [Lecture 37](../lectures/37-assembly-solver.md), [Lecture 41](../lectures/41-performance.md).

**Pick.** What a click selects, found by a `Pointer`. [Lecture 39](../lectures/39-editor.md).

**Piercing point.** Where an edge passes through a face, a start point for tracing. [Lecture 21](../lectures/21-remesh.md).

**Placed part.** A part built by another program and placed in this one (`PlacedPart`). [Lecture 38](../lectures/38-runner.md).

**Pointer.** A ray and a reach, what a front end sends for a hover or a click. [Lecture 39](../lectures/39-editor.md).

**Pole.** A point where a row of a surface's control points collapses: a sphere's pole. The surface is smooth; the coordinates are not. [Lecture 18](../lectures/18-poles-seams-splicing.md).

**Pose.** A rotation and a position: where a placed part is. [Lecture 36](../lectures/36-mates.md).

**Predictor-corrector.** A step along a curve: a tangent prediction, then Newton back onto it. [Lecture 16](../lectures/16-tracing-and-pcurves.md).

**Profile.** A chain of 2-D curves to sweep, with a name for each. [Lecture 25](../lectures/25-sketch-features.md), [Lecture 26](../lectures/26-sweeps.md).

**Program.** An ordered list of steps with their arguments: what a `.geop` file holds. [Lecture 2](../lectures/02-programs-of-steps.md).

**Protocol.** The JSON of `Command` and `Update` between a front end and the editor. [Lecture 39](../lectures/39-editor.md).

**Quaternion.** Four numbers standing for a rotation. [Lecture 36](../lectures/36-mates.md).

**Rank.** The number of independent rows of a Jacobian, decided over a box, not at a point. [Lecture 24](../lectures/24-rank-and-proof.md).

**Rational.** A ratio of polynomials. What lets a spline draw a circle exactly. [Lecture 8](../lectures/08-nurbs.md).

**Rasterizer.** What turns a B-rep into triangles for a viewer (`core/geop-core-rasterize`). [Lecture 35](../lectures/35-step-and-drawings.md).

**Reach.** How far from a pointer's ray counts as under it. [Lecture 39](../lectures/39-editor.md).

**`Read`.** One thing a make read. All of them recorded on the node. [Lecture 38](../lectures/38-runner.md).

**`Reader`.** The door through which a make reads the part, recording it. [Lecture 38](../lectures/38-runner.md).

**Rebuild.** The one routine under draft, shell, face moves and knit (`geop_lib_shell::rebuilt`). [Lecture 30](../lectures/30-one-rebuild.md).

**Refine.** To tighten an isolated solution, by Newton. It may only tighten, never fail. [Lecture 14](../lectures/14-newton.md).

**`Registry`.** Everything an editor offers, added to by each crate's `register`. [Lecture 39](../lectures/39-editor.md).

**Regression test.** A real bug's program rebuilt in Rust, kept for ever. [Lecture 40](../lectures/40-writing-an-operation.md).

**Remesh.** Imprinting two bodies on each other until they meet only along shared entities, in increasing dimension. [Lecture 21](../lectures/21-remesh.md).

**Residual.** The value of a constraint, zero where it holds. [Lecture 23](../lectures/23-sketch-solver.md).

**Revolve.** A sweep around an axis. [Lecture 26](../lectures/26-sweeps.md).

**Rolling ball.** The construction of a fillet: a ball rolling along two faces, its contact curves bounding the blend. [Lecture 29](../lectures/29-fillets.md).

**Rotation-minimizing frame.** A frame along a curve that turns as little as possible about the tangent. [Lecture 27](../lectures/27-path-sweep.md).

**Round trip.** Build, save, load, build again; the same part (`assert_round_trips`). [Lecture 38](../lectures/38-runner.md).

**`Runner`.** What runs a program on a part. [Lecture 38](../lectures/38-runner.md).

**`Scalar`.** The kernel's number: an interval. [Lecture 4](../lectures/04-intervals.md).

**Screw motion.** A rotation about an axis combined with a translation along it: the exact motion along a helix. [Lecture 27](../lectures/27-path-sweep.md).

**Seam.** Where a closed surface's parameter domain joins itself. [Lecture 18](../lectures/18-poles-seams-splicing.md).

**Sense.** Whether a coedge runs along or against its edge. [Lecture 17](../lectures/17-brep.md).

**Settle.** The solve the editor runs after every edit, moving the state so constraints hold. [Lecture 39](../lectures/39-editor.md).

**Sharpen.** To replace an interval with a point in it. Legitimate only for a free choice. [Lecture 5](../lectures/05-honest-numerics.md).

**Sheet.** Faces standing on their own, bounding nothing. [Lecture 17](../lectures/17-brep.md).

**Shell (entity).** A connected set of faces. [Lecture 17](../lectures/17-brep.md).

**Shell (operation).** Hollowing a solid to walls of a thickness. [Lecture 30](../lectures/30-one-rebuild.md).

**Sketch.** A planar drawing of curves under constraints, solved. [Lecture 23](../lectures/23-sketch-solver.md).

**Skin.** A surface through a sequence of compatible sections. [Lecture 28](../lectures/28-lofts-and-fills.md).

**Solid.** A body bounded by closed shells. [Lecture 17](../lectures/17-brep.md).

**Span.** The parameter interval between two consecutive distinct knots. [Lecture 7](../lectures/07-bsplines.md).

**Splice.** Imprinting an edge into a face (`splice_edge_into_face`), with several outcomes by what its ends lie on. [Lecture 18](../lectures/18-poles-seams-splicing.md).

**State.** What a run is given besides inputs: where its placed parts are. No step changes it. [Lecture 38](../lectures/38-runner.md).

**Station.** A place along a sweep's path where a section lies. [Lecture 26](../lectures/26-sweeps.md).

**STEP.** The standard exchange format for CAD models. [Lecture 35](../lectures/35-step-and-drawings.md).

**Step.** One entry of a program: an operation with arguments and an id. [Lecture 2](../lectures/02-programs-of-steps.md).

**Stress test.** A sweep over a space of bodies, telling a refusal from a failure. [Lecture 41](../lectures/41-performance.md).

**Subdivision.** Splitting into pieces, the global method that finds every solution. [Lecture 12](../lectures/12-subdivide-to-isolate.md).

**Subdivision surface.** A surface as the limit of refining a control cage. [Lecture 34](../lectures/34-subd.md).

**Sweep.** A profile carried along a path. [Lecture 26](../lectures/26-sweeps.md).

**Tangent group.** Surfaces at a vertex that count as one condition. [Lecture 42](../lectures/42-case-studies.md).

**Target.** A named, typed slot of a part. [Lecture 38](../lectures/38-runner.md).

**Tilt.** A millionth of a radian that makes an approximate blend leave a face on the right side. [Lecture 29](../lectures/29-fillets.md).

**Tool (editor).** A mode in hand while no step is edited, such as the drag tool. [Lecture 39](../lectures/39-editor.md).

**Tool (operation).** The solid an operation builds to combine with the part. [Lecture 29](../lectures/29-fillets.md).

**Trace.** To follow a face by face intersection curve by marching. [Lecture 16](../lectures/16-tracing-and-pcurves.md).

**Trim.** The boundary of a face in its surface's `$(u,v)$` plane. [Lecture 17](../lectures/17-brep.md).

**ULP (unit in the last place).** The spacing of floating-point numbers near a value. [Lecture 3](../lectures/03-floating-point.md).

**Union.** Of two enclosures of one value, the smallest interval holding both, when you must cover both. Also a boolean. [Lecture 5](../lectures/05-honest-numerics.md).

**`Update`.** What the editor sends back: only what changed. [Lecture 39](../lectures/39-editor.md).

**Validation.** Checking that a model is structurally and geometrically consistent (`validation::validate`). [Lecture 20](../lectures/20-validation.md).

**Variation diminishing.** A line crosses a curve at most as often as its control polygon. [Lecture 6](../lectures/06-bezier.md).

**Version.** A stamp on a node's value. A read holds if it sees the same version. [Lecture 38](../lectures/38-runner.md).

**Vertex.** A point of the B-rep, itself an enclosure. [Lecture 17](../lectures/17-brep.md).

**Weight.** A rational spline's per-point factor. Positive throughout. [Lecture 8](../lectures/08-nurbs.md).

**Width.** An interval's upper minus lower bound: its honest uncertainty. [Lecture 4](../lectures/04-intervals.md).

**Wire.** Edges with no faces. [Lecture 17](../lectures/17-brep.md).

**`with_context`.** Adds a line to an error's chain as it passes up the real call stack. [Lecture 42](../lectures/42-case-studies.md).

**Workspace.** Where a runner's programs and files come from (`Workspace`). [Lecture 38](../lectures/38-runner.md).
