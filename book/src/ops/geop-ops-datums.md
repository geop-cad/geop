# geop-ops-datums

> Brief overview only — full documentation is coming later.

A part's datums: reference geometry it is built *with*, not *of* — a point,
an axis, a plane or a coordinate system, by name (each a `Datum` of
[geop-core-math](../core/geop-core-math.md#primitives), carrying a
full right-handed frame, so anything built on it has axes to be built
along). The operation that adds one is
[geop-ops-add-datum](./geop-ops-add-datum.md)'s; this crate is only the
target, so every operation that builds on a datum depends on it alone.

## The target

All of a part's datums are one target, `datums()`, a `Datums` that the
steps adding datums revise, each adding its own by its step's id. The given
step of a run defines it (`define_datums`), so `ORIGIN` — the coordinate
system `origin`, the world's origin and axes, which no step adds — is there
before any step. A datum stays where it was when its step ran, whatever
later steps do to what it was built from. `read_datums(&reader)?.get(name)`
reads one; `Datums::draw` draws them all into a
[`PartView`](../core/geop-core-editor.md#the-view-view), by the sizes the viewer draws them at.

## Elements

The target is a singleton, so a reference into it is saved as the datum's
name alone — `"origin:z"`. A datum is an element of the target as a whole
(`AnyDatum`), and as what it is, the geometric kinds of
[geop-ops-model](./geop-ops-model.md#elements-and-what-they-are-used-as):
a point datum a `Point`, an axis a `Line`, a plane a `Plane`. A frame is a
point too, its origin, and its components are elements of their own, named
after it (`component_name`): `origin:z` its `z` axis, `origin:xy` the plane
normal to it. So a frame's axes and planes are each usable on their own,
and an operation that takes a plane from a face or a datum takes the
origin's planes the same way.

`datum::<K>(name)` makes a reference; `origin_plane(axis)` and
`origin_axis(axis)` are the origin's, typed as
`ElementReference<Datums, Plane>` and `ElementReference<Datums, Line>` —
what a sketch on the `xy` plane or a revolve about `z` is given.
