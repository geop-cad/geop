# geop-ops-rasterize

> Brief overview only — full documentation is coming later.

Turns a `Model` into a triangle mesh, writes it out as an STL file, and
renders it for debugging.

## Rasterizing a model

`rasterize(model, n)` samples every vertex, edge and face of a model into a
`RasterizedModel`, keeping each point, polyline and triangle together with
the id of the entity it came from. This is the one place topology becomes
sampled geometry. Drawing and picking in
[geop-ops](./geop-ops.md#operations), STL export and debug rendering all
read the same triangles, so a pick can never disagree with what the viewer
drew, and an exported mesh is exactly what is on screen.

`n` is a *quality*, not a fixed sample count: it controls how finely
geometry that actually curves is approximated, while a straight edge or a
flat face stays cheap.

## Triangulating a face

`face_triangles_uv` (implemented by `grid::triangulate_face`) triangulates a
face's trimmed region in `(u, v)` space, then maps each triangle through the
surface:

1. Sample the outer loop and every hole from their pcurves into `(u, v)`
   polygons, and drop redundant collinear points.
2. Pick a uniform grid resolution, doubling it until every cell's flat
   approximation is within a curvature-derived tolerance of the true
   surface. A flat face stays coarse; a curved one gets the resolution its
   curvature needs.
3. For each cell, clip the outer loop to it (`clip::clip_to_rect`), subtract
   each hole (`clip::subtract_convex`), and ear-clip what is left
   (`polygon_triangulate`).

Both clipping operations are Sutherland–Hodgman half-plane clipping and are
computed directly from the boundary's own edges. No edge is reconstructed,
so a mesh edge can never cut across the interior instead of following the
trim. Earlier approaches (constrained Delaunay, and bridging holes to the
outer loop with a zero-width slit) each produced edges that did not follow
the boundary, and were replaced by this simpler one.

This is the one part of the kernel that uses plain `f64` rather than
interval scalars. A mesh is only a picture of the geometry, never a claim
the kernel reasons about, so there is no uncertainty worth tracking.

## STL export

`stl_triangles(raster, faces)` returns the triangles of the given faces as
`StlTriangle`s, and `write_stl` writes them in binary or ASCII
(`StlFormat`). The `(u, v)` triangulation says nothing about which side is
outside, but every face's surface normal points out of its solid, so each
triangle is wound to agree with the normal.

Faces are sampled one at a time, so two faces that share an edge each sample
it independently. The mesh is only as watertight as those samplings agree.

## Debug rendering

The `debug` module renders models for a person to look at. A
`PrimitiveScene` collects points, lines, triangles, labels and sampled NURBS
curves and surfaces, and saves them as an interactive HTML file;
`PrimitiveSceneRecorder` records a sequence of them, for example one per
iteration of a search. Attached to an error (see
[geop-core-math](../core/geop-core-math.md)), a scene is rendered when the
error is printed. `RasterizedModel::scene` draws a rasterized model, each
face in a color of the caller's choice. The `*_viz` examples render
curve-curve and curve-surface intersections.

`rasterize_topology` draws a model's raw topology rather than its shape.
Every vertex, edge, coedge and face is labelled with its id, edges and
coedges get direction arrows, faces are semi-transparent with a normal
arrow, and coedges on an outer loop and on a hole are drawn in different
colours. This helps when debugging the Euler operators, where *which*
coedge is which matters and an outer loop and a hole look the same as
geometry.
