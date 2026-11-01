# Writing for this book

The book is built by `mdbook build book` (see
`.github/workflows/deploy-book.yml`) and nothing else: no preprocessor, no
kernel. The lectures are in `src/lectures/`, the older crate pages in
`src/core`, `src/lib`, `src/ops`, `src/cad`.

## A lecture

`src/lectures/06-bezier.md` is the model. Each has, in order:

- `# Lecture N: Title`, then `<div class="goals">` with what the reader can do after it;
- the lecture, in plain and precise prose, building from what earlier lectures established, with worked examples;
- `<div class="kernel">` boxes for where it is in the code;
- `<div class="recap">` under `## Recap`;
- `## Exercises`, with `<details class="solution"><summary>Solution</summary>` (or `Hint`), a blank line before and after the Markdown inside, a blank line before `</details>`.

A `<div>` needs a blank line after its opening tag and before its closing one, or the Markdown inside it is not parsed.

## Math

Math is a code span, so Markdown leaves it alone, and KaTeX renders it in the
browser: inline `` `$x_i^2$` ``, display `` `$$\sum_i x_i$$` `` (it may run over
several lines, with no blank line in it). Not in headings. In a table write
`\vert` for a bar. A real code span is one with no `$` at both ends.

## Citations

Every statement about the kernel is checked against the code. Cite real paths
and names in code spans, `core/geop-core-geometry/src/aabb.rs`, `compute_aabb`;
`scripts/check_citations.py` fails on a path or an identifier that does not
exist, so run it after the kernel changes too. A name quoted on purpose that
is not in the code goes in `scripts/citation_allow.txt`. Numbers come from
`AGENTS.md` or from a run you made (capped, as `AGENTS.md` says), and the text
says which. What is not implemented, the text says is not.

## A demo

```html
<figure class="demo" data-demo="name" data-args='{"optional":"json"}'>
<figcaption>What to do with it, and what to notice.</figcaption>
</figure>
```

`src/demos/name.js` calls `GeopBook.demo("name", init, algo)`:
`theme/geop-book.js` loads it when the figure comes near the viewport and calls
`init(ctx)`. `src/demos/de-casteljau.js` is the model. `ctx` has:

- `ctx.stage({aspect, height, maxHeight, label, draw(g, w, h, colors)})`: a canvas that follows the page's width; `stage.redraw()`;
- `ctx.view(stage, {x0,x1,y0,y1})`: world to screen, y up (`view.x`, `view.y`, `view.inv`), call `view.update()` in `draw`;
- `ctx.handles({stage, view, pts, onchange, constrain, radius})`: draggable points, touch and keyboard;
- `ctx.drag(stage, fn)`, `ctx.camera(stage, {yaw, pitch, scale})`: 3-D projection, orbited by dragging;
- `ctx.slider`, `ctx.select`, `ctx.checkbox`, `ctx.button`, `ctx.readout`, `ctx.note`;
- `ctx.player({duration | steps, loop, onframe})`: play, pause, step, reset; it never moves by itself under `prefers-reduced-motion` or off screen, and only starts by itself if `autoplay` is given;
- `ctx.colors()` (also passed to `draw`): take every colour from here, so light and dark themes work. Never hard-code one;
- `GeopBook.draw`: `polyline`, `polygon`, `dot`, `text`, `grid`.

Rules: vanilla JS, no network, no library. Keep the demo's mathematics in pure
functions passed as `algo`, and test them against their formula in
`scripts/demo_tests/name.test.mjs`. A demo is an accurate computation, not a
decoration: if it is simpler than what the kernel does, its caption says how.
Keep it to a few hundred lines. It must work at 360 px wide.

## Models from the kernel

`model-viewer` draws a mesh the kernel made: `data-args='{"model":"rounded_box","example":"rounded_box"}'`.
The meshes in `src/assets/models/` are written by `scripts/build_models.py` from
STL files `geop examples -O quality=8` wrote, and are committed, so that the
book's build needs no kernel. The link goes to the real kernel in the web app,
`https://app.geop-cad.dev/?example=<name>`.

## Checks

```sh
mdbook build book
python3 book/scripts/check_citations.py
node book/scripts/test_demo_math.mjs
NODE_PATH=<web/node_modules> node book/scripts/check_demos.mjs
```
