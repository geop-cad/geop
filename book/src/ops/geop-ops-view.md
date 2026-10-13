# geop-ops-view

> Brief overview only — full documentation is coming later.

`PartView`: a part as the viewport draws it, and picking elements of it
with a `Pointer`.

## Drawing

A part is drawn from its rasterization — sampled edges, triangulated faces,
outlined sketches (see [geop-ops-rasterize](./geop-ops-rasterize.md)) —
and a pick tests against exactly that, never re-deriving geometry of its
own, so a pick cannot disagree with what is on screen. The same goes for
what the viewer draws besides the part: the datums are laid out here, by
the sizes the viewer draws them with (a frame's axes are `FRAME` reaches
long), and picked by the same rules. Serialized, a `PartView` is what a
viewer draws.

Each crate draws its own targets into a view — the model its vertices,
edges and faces, the sketches theirs, the datums theirs — and an editor
composes them (`geop_cad_base::part::draw`), then `PartView::finish`
measures the result.

The parts placed in a part are drawn by reference: each placed part —
however deep — is listed once, with where it is (`ViewInstance`), and drawn
from its component's own view, built once and shared by every instance of
it. Moving a placed part changes where it is, not what is drawn.

## Picking

`PartView::pick(pointer, accepts)` finds what the pointer is over among
what is drawn of the kinds a field accepts (`drawn`: `vertex`, `edge`,
`face`, `sketch`, `sketch_point`, `sketch_curve`, `datum`, `frame`). Near
means within the pointer's reach. Frame datums are drawn on top of
everything, so they are picked first; otherwise the smallest element
under the pointer and not hidden behind a face wins — a vertex or a sketch
point over an edge or a sketch curve — and else the nearest along the ray
of the faces, sketches and other datums hit. A pick names what it hits as the crate that drew it
named it, a `PickedElement`, which the crate defining the element turns
into a typed reference (see
[geop-core-editor](../core/geop-core-editor.md#picking-references)).

A pick moves the pointer into each placed part's own frame and names what
it hits as that part names it, its target behind the instance's name
(`in_instance`: `bolt/model`). `pick_instance` and `part_to_drag` find the
placed part under the pointer, for dragging it. A `frame` pick (a
`FramePick`) puts a frame on what is picked — a face, an edge, a point, a
datum — at the corner or side of a face, or the end of an edge, that the
pointer is near: what a joint is made between (see
[geop-ops-assembly](./geop-ops-assembly.md)).

## What steps add to the drawing and the structure

An editor knows no operation by name. What a step adds besides its
topology, it adds as a target of its own, which the editor collects without
knowing whose it is:

- **Annotations** (`annotations(step)`, an `Annotations`): what a step
  draws on the part besides its topology — a cosmetic thread's helix and
  designation (`geop_ops_model::draw_threads` is the example).
  `annotations_of` collects every step's.
- **Structure entries** (`listing(step)`, a `Listing` of `Listed`): what a
  step lists in the part's structure tree beside what every part has
  (datums, sketches, solids, faces standing on their own, placed parts): a
  kind (`mate`, the tree's group), a name and the elements it refers to,
  lit while it is pointed at. `listed_by_steps` collects every step's;
  `add_part` lists its mates this way.

`icons` holds the icons operations share.
