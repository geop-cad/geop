# geop-ops-boolean

> This page is a reference to the crate; the algorithm is taught in [Lecture 22](../lectures/22-boolean.md), and this crate is read as an example in [Lecture 40](../lectures/40-writing-an-operation.md).

The operations a program combines and cuts solids with (see
[geop-core-editor](../core/geop-core-editor.md#an-operation-partoperation)),
over the algorithms of [geop-lib-booleans](../lib/geop-lib-booleans.md).

`Boolean` unites, intersects or subtracts two solids of the part by name,
consuming both; the result is named `boolean(B)`. An empty result is an
answer, not an error.

`Split` cuts a solid with a face standing on its own — and the rest of its
sheet — into pieces named `split(S,0)`, `split(S,1)`, ..., leaving the face
as it was.

`Imprint` divides two bodies where they cross, keeping both; with a face
given, only that face. `Section` makes the curves where they cross a wire.

`SplitFace` divides one face along the curves of a sketch, projected onto it
along the sketch's normal, or each point to its nearest on the face, and the
solid keeps its shape and volume. It is the kernel's imprint: the curves are
swept into sheets reaching past the face both ways, and those are imprinted on
it, so a planar face and a curved one are one path, and a curve reaching past
the face is trimmed to it. Every curve of the sketch that is no construction
curve divides (`Sketch::runs`), however they join or cross, and a text's
outlines. A piece split off the face `F` along the curve `c3` of the sketch
`K` is `split_face(S,F,K,c3)`, with `,k` for the curve's `k`-th stretch across
the face; the other piece, and what is left round a curve closing inside the
face, keeps the name `F`; the edge is `split_face(S,K,c3)`. Refused by name: a
sketch that misses the face or holds only construction curves, a curve ending
inside the face, a curve only joining two of its boundaries, as a line from
its outside to a hole does, and a curve running along one of its edges. A wall
of a cylinder is four faces, so a loop all round it is four steps.

The same combination done by another operation with the solid it builds —
an extrude cutting a pocket, say — is that operation's `Combine`, defined in
[geop-ops-model](./geop-ops-model.md) and applied by `combine_tools`.

Its examples (`examples`): `split_plate`, a plate cut in two by a face extruded
from a line, and `badge`, a disc divided off a plate's top. The tests of programs uniting extruded and revolved
solids — and extruding or revolving up to the next face of solids so
united — are here (`program_tests.rs`): this crate depends on
extrude and revolve for its example, not the other way round.
