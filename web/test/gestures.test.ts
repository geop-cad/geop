import assert from "node:assert/strict";
import { test } from "node:test";
import { CLICK_PX, DOUBLE_MS, isDoubleClick, movedFromPress } from "../src/gestures.ts";

test("a press that stays within the click distance is a click", () => {
  assert.equal(movedFromPress({ x: 10, y: 10 }, { x: 10, y: 10 }), false);
  assert.equal(movedFromPress({ x: 10, y: 10 }, { x: 10 + CLICK_PX, y: 10 }), false);
  assert.equal(movedFromPress({ x: 10, y: 10 }, { x: 10 + CLICK_PX + 1, y: 10 }), true);
  // The distance is straight-line: a diagonal of 3 and 3 is 4.24 pixels.
  assert.equal(movedFromPress({ x: 0, y: 0 }, { x: 3, y: 3 }), true);
});

test("a second click soon and near the first is a double click", () => {
  const first = { x: 50, y: 50, time: 1000 };
  assert.equal(isDoubleClick(first, { x: 50, y: 50 }, 1000 + DOUBLE_MS - 1), true);
  assert.equal(isDoubleClick(first, { x: 52, y: 51 }, 1100), true);
});

test("a second click too late or too far away is a click of its own", () => {
  const first = { x: 50, y: 50, time: 1000 };
  assert.equal(isDoubleClick(first, { x: 50, y: 50 }, 1000 + DOUBLE_MS), false);
  assert.equal(isDoubleClick(first, { x: 50 + CLICK_PX + 1, y: 50 }, 1100), false);
  assert.equal(isDoubleClick(null, { x: 50, y: 50 }, 1100), false);
});

test("zooming keeps what is under the pointer under it, and a notch is a notch", async () => {
  const { zoomToward, wheelSteps } = await import("../src/gestures.ts");
  // Looking down -z from 10 above the origin; the pointer over (2, 0, 0).
  const { position, target } = zoomToward([0, 0, 10], [0, 0, 0], [2, 0, 0], 1);
  const along = (p: number[]) => [p[0] - position[0], p[1] - position[1], p[2] - position[2]];
  // The ray from the eye to (2, 0, 0) keeps its slope: x / z the same.
  const before = 2 / 10;
  const toUnder = along([2, 0, 0]);
  assert.ok(Math.abs(toUnder[0] / -toUnder[2] - before) < 1e-12, `${position} ${target}`);
  assert.ok(Math.abs(position[2] - 8.5) < 1e-12, "a notch is 0.85 of the distance");
  assert.equal(wheelSteps(100, 0), -1);
  assert.equal(wheelSteps(-3, 1), 1);
});
