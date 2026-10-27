import assert from "node:assert/strict";
import { test } from "node:test";
import type { Frame, PartView, Vec3, ViewInstance } from "../src/geop.ts";
import { boxed, placedAt } from "../src/lod.ts";

/** A part placed at `origin`, unturned. */
function at(name: string, origin: Vec3): ViewInstance {
  const frame: Frame = { origin, u: [1, 0, 0], v: [0, 1, 0], w: [0, 0, 1] };
  return { name, component: "screw", frame };
}

/** A component 2 across, about `(0, 0, 1)`. */
const SCREW = { extent: { center: [0, 0, 1], size: 2 } } as unknown as PartView;

/** What a pixel measures at `p`, seen from `eye` in a perspective view: proportional to the distance. */
function seenFrom(eye: Vec3): (p: Vec3) => number {
  return (p) => Math.hypot(p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]) / 300;
}

test("a placed part's box is where the part is", () => {
  const turned: Frame = { origin: [10, 0, 0], u: [0, 1, 0], v: [-1, 0, 0], w: [0, 0, 1] };
  assert.deepEqual(placedAt(turned, [1, 2, 3]), [8, 1, 3]);
});

test("parts seen whole from afar are boxes, those near are drawn in full", () => {
  const screws = Array.from({ length: 2000 }, (_, k) => at(`screw${k}`, [(k % 50) * 4, Math.floor(k / 50) * 4, 0]));
  // From far above the whole robot every screw is a few pixels across.
  const far = boxed(screws, { screw: SCREW }, seenFrom([100, 80, 5000]));
  assert.equal(far.size, 2000);
  // Close over one screw, it and its neighbours are drawn in full.
  const near = boxed(screws, { screw: SCREW }, seenFrom([0, 0, 10]));
  assert.ok(!near.has("screw0"));
  assert.ok(near.has("screw1999"));
  // A part of a component not sent yet is not drawn at all, as a box or not.
  assert.equal(boxed([at("other", [0, 0, 0])], {}, seenFrom([0, 0, 5000])).size, 0);
});
