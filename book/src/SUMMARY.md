# Summary

[Introduction](./introduction.md)

# Part 0: What we are building

- [1. What a CAD kernel is](./lectures/01-what-a-kernel-is.md)
- [2. A part is a program of steps](./lectures/02-programs-of-steps.md)

# Part I: Numbers you can trust

- [3. Floating point](./lectures/03-floating-point.md)
- [4. Interval arithmetic and enclosures](./lectures/04-intervals.md)
- [5. The rules of honest numerics](./lectures/05-honest-numerics.md)

# Part II: Curves and surfaces

- [6. Bézier curves](./lectures/06-bezier.md)
- [7. B-splines](./lectures/07-bsplines.md)
- [8. NURBS and exact conics](./lectures/08-nurbs.md)
- [9. Knot insertion and subdivision](./lectures/09-subdivision.md)
- [10. Surfaces](./lectures/10-surfaces.md)
- [11. Evaluating relative to the patch](./lectures/11-evaluate-locally.md)

# Part III: Intersection

- [12. Subdivide to isolate](./lectures/12-subdivide-to-isolate.md)
- [13. Fat-line clipping](./lectures/13-fat-lines.md)
- [14. Newton to refine](./lectures/14-newton.md)
- [15. Tangency and overlap](./lectures/15-tangency-and-overlap.md)
- [16. Tracing and pcurves](./lectures/16-tracing-and-pcurves.md)

# Part IV: Topology

- [17. The boundary representation](./lectures/17-brep.md)
- [18. Poles, seams and splicing](./lectures/18-poles-seams-splicing.md)
- [19. Inside or outside](./lectures/19-classification.md)
- [20. Validation](./lectures/20-validation.md)

# Part V: Booleans

- [21. The remesh pipeline](./lectures/21-remesh.md)
- [22. From remesh to result](./lectures/22-boolean.md)

# Part VI: Sketches

- [23. Solving a sketch](./lectures/23-sketch-solver.md)
- [24. Rank and proof](./lectures/24-rank-and-proof.md)
- [25. Profiles, offsets and 3-D sketches](./lectures/25-sketch-features.md)

# Part VII: Operations

- [26. Sweeps: extrude and revolve](./lectures/26-sweeps.md)
- [27. Sweeping along a path](./lectures/27-path-sweep.md)
- [28. Lofts and filling surfaces](./lectures/28-lofts-and-fills.md)
- [29. Fillets and chamfers](./lectures/29-fillets.md)
- [30. One rebuild: shell, draft, moving faces](./lectures/30-one-rebuild.md)
- [31. Offsetting a surface](./lectures/31-offsets.md)
- [32. Patterns, deformations, holes and threads](./lectures/32-patterns-holes-threads.md)
- [33. Sheet metal and plastic parts](./lectures/33-sheet-metal-plastic.md)
- [34. Subdivision surfaces](./lectures/34-subd.md)
- [35. STEP and drawings](./lectures/35-step-and-drawings.md)

# Part VIII: Assemblies

- [36. Mates and poses](./lectures/36-mates.md)
- [37. The assembly solver](./lectures/37-assembly-solver.md)

# Part IX: The program model

- [38. The runner and incremental builds](./lectures/38-runner.md)
- [39. The editor, its protocol and front ends](./lectures/39-editor.md)
- [40. Writing an operation](./lectures/40-writing-an-operation.md)

# Part X: The craft

- [41. Counting work](./lectures/41-performance.md)
- [42. Case studies in debugging](./lectures/42-case-studies.md)

# Appendices

- [A. References](./appendix/references.md)
- [B. Glossary](./appendix/glossary.md)
- [C. Map of the crates](./appendix/crates.md)
    - [geop-core-math](./core/geop-core-math.md)
    - [geop-core-geometry](./core/geop-core-geometry.md)
    - [geop-core-topology](./core/geop-core-topology.md)
    - [geop-core-sketch](./core/geop-core-sketch.md)
    - [geop-core-rasterize](./core/geop-core-rasterize.md)
    - [geop-core-program](./core/geop-core-program.md)
    - [geop-core-editor](./core/geop-core-editor.md)
    - [geop-lib-sheet](./lib/geop-lib-sheet.md)
    - [geop-lib-booleans](./lib/geop-lib-booleans.md)
    - [geop-lib-sweep](./lib/geop-lib-sweep.md)
    - [geop-lib-stdlib](./lib/geop-lib-stdlib.md)
    - [geop-lib-shell](./lib/geop-lib-shell.md)
    - [geop-lib-constraints](./lib/geop-lib-constraints.md)
    - [geop-lib-fixtures](./lib/geop-lib-fixtures.md)
    - [geop-ops-model](./ops/geop-ops-model.md)
    - [geop-ops-sketch](./ops/geop-ops-sketch.md)
    - [geop-ops-reference-geometry](./ops/geop-ops-reference-geometry.md)
    - [geop-ops-boolean](./ops/geop-ops-boolean.md)
    - [geop-ops-extrude-revolve](./ops/geop-ops-extrude-revolve.md)
    - [geop-ops-assembly](./ops/geop-ops-assembly.md)
    - [geop-cad-base](./cad/geop-cad-base.md)
    - [geop-cad-web](./cad/geop-cad-web.md)
    - [geop-cad-cli](./cad/geop-cad-cli.md)
