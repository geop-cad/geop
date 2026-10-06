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
