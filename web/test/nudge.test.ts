import assert from "node:assert/strict";
import { test } from "node:test";
import { nudge, stepsPerPress } from "../src/nudge.ts";

// The same cases as the desktop app's `a_number_is_nudged_by_its_step`.
test("a number moves by its step, rounded to the step's digits; a formula does not move", () => {
  assert.equal(nudge("0.2", 0.1, 1), "0.3");
  assert.equal(nudge("1", 0.1, -1), "0.9");
  assert.equal(nudge("1", 0.1, 10), "2");
  assert.equal(nudge("1", 0.1, 0.1), "1.01");
  assert.equal(nudge("5", 1, -1), "4");
  assert.equal(nudge("0", 0.05, 3), "0.15");
  assert.equal(nudge("width / 2", 0.1, 1), null);
  assert.equal(nudge("", 0.1, 1), null);
  assert.equal(stepsPerPress({ shiftKey: true, altKey: false }), 10);
  assert.equal(stepsPerPress({ shiftKey: false, altKey: true }), 0.1);
});
