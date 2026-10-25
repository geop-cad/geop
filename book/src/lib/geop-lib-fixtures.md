# geop-lib-fixtures

> Brief overview only — full documentation is coming later.

Test-only steps that give a program what an operation's tests build on,
without sketching and extruding it: `Primitive`, a solid at given
coordinates — a block, a prism, a cylinder or a sphere, kept as a new body
or combined with a solid of the part as a step's `Combine` says — and
`GivenDatum`, a point, an axis, a plane or a coordinate system at given
coordinates. Both are ordinary serializable steps, so a program in an
editor's timeline can hold them and a part the front end clicks on can be
built from them; no editor offers them.

An operation's tests take the crate as a dev-dependency and register the
two steps in the registry of their programs (`registry`) or of their
editor (`register`). `drilled_box` is the box with a blind hole the
hand-written tests cut, round and inspect; `bodies` holds the boxes,
cylinders and spheres, whole, drilled, pocketed, stepped and with a boss,
that the stress sweeps of blends and shells run over.

The crate exists so that a change to the sketch or the extrude operation
retests those crates and the composition, not every crate whose tests
start from a block: a test that needs a solid starts from a `Primitive`,
one that checks how its operation meets an extrusion as such keeps
depending on `geop-ops-extrude-revolve`.
