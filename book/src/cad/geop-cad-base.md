# geop-cad-base

> Brief overview only — full documentation is coming later.

Ray-based picking against kernel geometry: what an interactive CAD editor
needs from the kernel beyond building parts.

## Picking

```rust,ignore
let hit: Option<PickHit<S>> = pick(&model, &rasterized, ray, PickFilter::Any, tolerance)?;
```

`pick` casts a `Ray` (from the camera through the cursor) and returns the
nearest entity matching the `PickFilter` as a `PickHit`: its kind, its id,
the hit point and the ray parameter `t`. The caller turns the id into the
entity's stable name, which is what a program step refers to it by.

| Filter   | Hits                                                            |
| -------- | --------------------------------------------------------------- |
| `Vertex` | the nearest vertex within `tolerance` of the ray                |
| `Edge`   | the nearest edge within `tolerance`                             |
| `Face`   | the first face the ray enters                                   |
| `Solid`  | the solid owning the first face hit                             |
| `Any`    | a vertex, else an edge, else a face: the smallest entity under the cursor that is not hidden behind a face |

Face hits are exact ray/triangle intersections (Möller–Trumbore). When a
division cannot be resolved because its divisor could be zero, the hit is
reported as a miss, which is the honest answer when the enclosure cannot
rule zero out.

Picking works on the `RasterizedModel` the viewer drew (see
[geop-ops-rasterize](../ops/geop-ops-rasterize.md)) rather than
re-triangulating, so a pick can never disagree with what is on screen.
Rasterizing once per build also makes picking cheap enough to run on every
pointer move, to highlight what a click would pick.

`pick_sketch` does the same for sketches, against `SketchTargets::of(part)`.
A sketch is hit near one of its curves, or anywhere inside one of its closed
regions.

`tolerance` is the one plain `f64` here. It is a UI fuzziness derived from
screen pixels, not a geometric quantity the kernel reasons about, so it is
compared against `to_f64` rather than folded into interval arithmetic.
