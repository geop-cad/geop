import assert from "node:assert/strict";
import { test } from "node:test";
import { assistantFits, docked } from "../src/layout.ts";

// The windows `e2e/firstrun.mjs` visits, and what docks beside the view in each.
test("a panel docks only where the view keeps its room; the assistant starts open only beside all of them", () => {
  // A wide window has room for everything.
  assert.deepEqual(docked(1440, true), { explorer: true, assistant: true });
  assert.equal(assistantFits(1440), true);
  assert.equal(assistantFits(1366), true);
  // Below, the assistant starts closed; opened, it docks before the explorer.
  assert.equal(assistantFits(1280), false);
  assert.deepEqual(docked(1280, false), { explorer: true, assistant: false });
  assert.deepEqual(docked(1280, true), { explorer: false, assistant: true });
  // At 1024 the explorer docks, and the assistant, opened, lies over the view.
  assert.equal(assistantFits(1024), false);
  assert.deepEqual(docked(1024, false), { explorer: true, assistant: false });
  assert.deepEqual(docked(1024, true), { explorer: true, assistant: false });
  // Narrower still, the explorer moves to the program panel.
  assert.deepEqual(docked(900, false), { explorer: false, assistant: false });
});
