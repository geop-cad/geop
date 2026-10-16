# geop-ops-boolean

> Brief overview only — full documentation is coming later.

The operations a program combines and cuts solids with (see
[geop-core-editor](../core/geop-core-editor.md#an-operation-partoperation)),
over the algorithms of [geop-lib-booleans](../lib/geop-lib-booleans.md).

`Boolean` unites, intersects or subtracts two solids of the part by name,
consuming both; the result is named `boolean(B)`. An empty result is an
answer, not an error.

`Split` cuts a solid with a face standing on its own — and the rest of its
sheet — into pieces named `split(S,0)`, `split(S,1)`, ..., leaving the face
as it was.

The same combination done by another operation with the solid it builds —
an extrude cutting a pocket, say — is that operation's `Combine`, defined in
[geop-ops-model](./geop-ops-model.md) and applied by `combine_tools`.
