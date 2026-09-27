# geop-core-part

> Brief overview only — full documentation is coming later.

A part: topology plus sketches, with every entity given a stable name so
later operations can refer to what earlier steps built by name rather than
by internal id.

## The part

A `Part<S>` holds a topology `Model`, the `PlacedSketch`es and `Datum`s
used to build it, and a `NameRegistry` mapping every vertex, edge, face,
solid, sketch and datum to exactly one name, and every name to exactly one
entity.

Its fields are private. Every Euler and edit operator of
[geop-core-topology](./geop-core-topology.md) has an identically named
method on `Part` that forwards to it, registers each entity it creates under
the name the caller supplies, and forgets the name of each entity it
deletes. The invariant checked by `Part::check_names` therefore holds by
construction, not by the diligence of each caller.

- **Sketches** are stored with the plane they were placed on. The plane is
  resolved once, when the sketch is added, so a sketch drawn on a face stays
  where that face *was*, even after a later operation reshapes or consumes
  the face.
- **Datums** are reference geometry — points, axes and planes the part is
  built *with*, not *of*. Every datum carries a full right-handed frame, so
  anything built on it has axes to be built along.
- **Lookups by name** (`vertex_id`, `edge_id`, `face_id`, `solid_id`,
  `sketch_id`, `datum_id`, and `coedge_id(edge, face)`) are what every
  operation that refers to existing entities is built on.
- **`PartDescription::of`** describes the topology in names only, with no
  internal id anywhere, so two parts built by the same program describe
  identically and the description diffs well as text.

## Topological naming

A name says *how* an entity came to be, never *when*. It is built only from
inputs that stay the same when a part is rebuilt after an upstream edit:
operation ids, sketch element ids, the names of the entities an operation
consumed, and positions counted along those. It never uses an internal id
or the order in which an algorithm happened to create things. A program that
refers to `extrude(box,end)` keeps meaning the same face when the box gets
taller.

Every name has the form `kind(operation,arg,...)`, built by a `Namer`:
`kind` is the operation that created the entity, `operation` the id of the
program step that ran it, and the arguments identify the entity within that
step. For example:

| Name                      | Entity                                                     |
| ------------------------- | ---------------------------------------------------------- |
| `extrude(E)`              | the solid built by extrude step `E`                        |
| `extrude(E,start)`, `extrude(E,end)` | its caps                                        |
| `extrude(E,K,c3)`         | the side face swept by line `c3` of sketch `K`             |
| `extrude(E,K,c3,end)`     | that face's edge on the end cap                            |
| `extrude(E,K,p1)`         | the edge swept by sketch point `p1`                        |
| `boolean(B,E1,E2,i,n)`    | the `i`-th of `n` points where edges `E1` and `E2` cross, counted along `E1` |

Operation ids may only contain ASCII letters, digits, `_`, `-` and `.`
(`validate_operation_id`), so the `(`, `)` and `,` that structure a name
stay unambiguous even when an argument is itself a name.
