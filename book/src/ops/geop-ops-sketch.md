# geop-ops-sketch

> Brief overview only — full documentation is coming later.

Sketches as an operation (see [geop-ops](./geop-ops.md#operations)).
`AddSketch` places a sketch, drawn with
[geop-core-sketch](../core/geop-core-sketch.md), on a base plane, a planar
face or a datum plane of the part. The plane is resolved when the step
runs, so a sketch on a face stays where the face was, whatever later steps
do to it. Every sketch point is a handle that slides in the sketch plane.
