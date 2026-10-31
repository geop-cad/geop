import assert from "node:assert/strict";
import { test } from "node:test";
import { bugReportPayload, type BugReportInput } from "../src/bugReportPayload.ts";
import type { Program, Report } from "../src/geop.ts";

const committed: Program = {
  steps: [{ id: "box", operation: "primitive", args: { block: [0, 0, 0, 1, 1, 1] } }],
};

/** A fillet open in its dialog, too large for its edge: never committed. */
const report: Report = {
  program: committed,
  step: {
    id: null,
    index: 1,
    placed: null,
    step: { id: "fillet1", operation: "fillet", args: { edges: ["e1"], radius: 2 } },
    missing: [],
    error: "the fillet is too large for the edge",
    concerning: "radius",
  },
  version: "0.11.0",
};

const input: BugReportInput = {
  description: "  Fillet fails on the hole:\n\nGeopError: the fillet is too large  ",
  program: { steps: [] },
  report,
  lastError: null,
  commit: "d1ec42cf",
  url: "https://app.geop-cad.dev/",
  userAgent: "test",
  timestamp: "2026-10-31T12:00:00.000Z",
};

test("a report carries the step being edited, its error and the field it concerns", () => {
  const payload = bugReportPayload(input);
  assert.deepEqual(payload.step, report.step);
  assert.equal(payload.step?.step.operation, "fillet");
  assert.deepEqual(payload.step?.step.args, { edges: ["e1"], radius: 2 });
  assert.equal(payload.step?.error, "the fillet is too large for the edge");
  assert.equal(payload.step?.concerning, "radius");
});

test("a report carries the description exactly as typed", () => {
  assert.equal(bugReportPayload(input).description, input.description);
});

test("a report carries the program and the kernel's version and commit", () => {
  const payload = bugReportPayload(input);
  assert.deepEqual(payload.program, committed);
  assert.deepEqual(payload.kernel, { version: "0.11.0", commit: "d1ec42cf" });
  assert.deepEqual(payload.meta, {
    url: input.url,
    userAgent: input.userAgent,
    timestamp: input.timestamp,
    lastError: null,
  });
});

test("where the kernel cannot answer, the app's program is sent", () => {
  const payload = bugReportPayload({ ...input, report: null, lastError: "the kernel crashed" });
  assert.deepEqual(payload.program, input.program);
  assert.equal(payload.step, null);
  assert.deepEqual(payload.kernel, { version: null, commit: "d1ec42cf" });
  assert.equal(payload.meta.lastError, "the kernel crashed");
});

test("the payload survives being sent as JSON", () => {
  const payload = bugReportPayload(input);
  assert.deepEqual(JSON.parse(JSON.stringify(payload)), payload);
});
