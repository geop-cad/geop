# geop-ops-booleans

> Brief overview only — full documentation is coming later.

3-D boolean operations — union, intersection, and difference — on B-rep
solids.

```rust,ignore
let result: Option<SolidId> =
    boolean(&mut part, &namer, solid_a, solid_b, BooleanOp::Difference, RemeshParams::default())?;
```

Both input solids are consumed: their faces move into the result or are
deleted. `Ok(None)` means the result is **empty**, which is an answer and not
a failure. Intersecting two disjoint solids, or subtracting a solid that
contains the first one, legitimately leaves nothing.

The work is split in two. `remesh` does the hard part, and `boolean` is
simple *because* of it.

## Remesh

`remesh` imprints the two solids onto each other until they meet only along
entities they share. It runs in strictly increasing dimension, so each phase
can assume that everything of lower dimension has already settled:

1. **Vertices** (`remesh_vertices`): coincident vertices are merged.
2. **Vertices × edges** (`remesh_vertices_x_edges`): an edge that passes
   through a vertex of the other solid is split there.
3. **Edges × edges** (`remesh_edges_x_edges`): edges that cross are split at
   the crossing; coincident edges that share their end points are merged.
4. **Edges × faces** (`remesh_edges_x_faces`), in both directions, each step
   run to a fixed point:
   1. split every edge where it pierces a face of the other solid;
   2. imprint every edge that lies entirely within a face of the other
      solid as a new boundary of that face;
   3. scan the settled model for vertices lying on a face they have no
      boundary on — the piercing points — and record them as start points;
   4. from each start point, **trace** the face × face intersection curve
      until it reaches a known vertex, and splice it in as a new edge shared
      by both faces.

Tracing marches along the curve with a predictor-corrector step: a
tangent prediction, then a damped Newton corrector that solves
`S_a(u_a, v_a) = S_b(u_b, v_b)` together with a fourth equation holding the
point on the plane through the prediction, so the corrector moves onto the
curve without sliding along it. The predictor is sharpened (it is only a
seed) and the corrector's final iterate is not (it is the answer). A trace
ends at a vertex only if that vertex lies on *both* surfaces, not merely
because it is near. The finished curve is fitted with
`interpolate_enclosing`, so it encloses the true intersection rather than
only passing through the sampled points.

Every curve has two ends, both piercing points, so it would be traced
twice. The second trace is recognized topologically, by finding an edge that
already joins the same two vertices on both faces, and not by sampling
geometry.

`RemeshParams` bundles every tunable. They affect only how hard a search
tries (node budgets, subdivision sizes, step length), never what a
converged answer means.

## Boolean

After remeshing, no face straddles the other solid: each lies wholly inside
it, wholly outside, or on its boundary. The boolean then becomes a
classification followed by a lookup table:

1. `remesh` the two solids.
2. `classify_face` takes a point strictly inside each face's trim and asks
   `shell_contains` where it lies relative to the other solid. A point that
   lands on the other solid's boundary is set aside and the next interior
   point is tried, because the face may only touch that boundary at a point
   or along a curve. Only if every candidate lies on the boundary is the
   face coincident, and then the two normals decide between `OnSameNormal`
   and `OnOppositeNormal`.
3. Keep, reverse or drop each face:

   | Face of | Class          | Union   | Intersection | Difference `a − b` |
   | ------- | -------------- | ------- | ------------ | ------------------ |
   | `a`     | Outside        | keep    | drop         | keep               |
   | `a`     | Inside         | drop    | keep         | drop               |
   | `b`     | Outside        | keep    | drop         | drop               |
   | `b`     | Inside         | drop    | keep         | keep, **reversed** |
   | `a`     | OnSameNormal   | keep    | keep         | drop               |
   | `b`     | OnSameNormal   | drop    | drop         | drop               |
   | `a`     | OnOppositeNormal | drop  | drop         | keep               |
   | `b`     | OnOppositeNormal | drop  | drop         | drop               |

   A coincident patch is kept at most once, and always from `a`.
4. `assemble_solid` builds the survivors into one new solid and deletes
   everything no longer reachable.

The containment queries use a fixed seed, so a boolean is reproducible from
run to run.

## Naming

Everything a boolean creates is named after what it was made from, in terms
of the names the operands had *before* the boolean. With `N` the boolean's
namer:

| Entity                                           | Name              |
| ------------------------------------------------ | ----------------- |
| the result solid                                 | `N`               |
| vertex where edges `E1 < E2` cross               | `N(E1,E2,i,n)`, the `i`-th of `n` crossings along `E1` |
| vertex where edge `E` pierces face `F`           | `N(E,F,i,n)`      |
| piece of edge `E` starting at vertex `V`         | `N(E,V)`; the first piece keeps the name `E` |
| edge traced along faces `F1 < F2` from `P` to `Q` | `N(F1,F2,P,Q)`   |
| piece of face `F` split off along edge `E`       | `N(F,E)`; the other piece keeps the name `F` |

`<` compares names as strings, so a name does not depend on which operand
came first or on the order the algorithm found things in. Since `i` and `n`
are only known once every crossing has been found, `BooleanNaming` hands out
provisional names during the run and settles them all in `finish`.

## Tests

`scenes` builds a set of solid pairs from the basic shapes: a grid of every
axis-aligned relative offset of two identical solids, and hand-picked
box, figure-8 and cylinder arrangements (through holes, blind holes, nested,
disjoint, corner overlaps, coincident faces). The render tests run remesh
and every operator over all of them and write each result to `outputs/`.
One failing scene is logged and rendered as far as it got, so it never hides
the others.
