import assert from "node:assert/strict";
import { test } from "node:test";
import { drawOrder } from "../src/gizmo3d.ts";

test("a gizmo's squares are drawn over its rings and arrows, its cubes and ball over the squares", () => {
  const ring = drawOrder({ part: "turn", axis: 1 });
  const arrow = drawOrder({ part: "move", axis: 0 });
  const square = drawOrder({ part: "plane", axis: 2 });
  assert.ok(square > ring, "a square under a ring");
  assert.ok(square > arrow, "a square under an arrow");
  for (const part of [{ part: "scale" }, { part: "free" }, { part: "stretch", axis: 0 }] as const)
    assert.ok(drawOrder(part) > square, `a square over the ${part.part}`);
  // Every part over the model, whose highlights are drawn up to 1001.
  assert.ok(ring > 1001);
});
