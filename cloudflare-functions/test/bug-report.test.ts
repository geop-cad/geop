import assert from "node:assert/strict";
import { test } from "node:test";
import { onRequestPost } from "../functions/api/bug-report.ts";

/** An R2 bucket in memory: what was put, by key. */
class MemoryBucket {
  objects = new Map<string, string>();
  async put(key: string, value: string) {
    this.objects.set(key, value);
  }
}

/** A report as the web app sends it (`web/src/bugReportPayload.ts`): the step being edited, never committed, among it. */
const report = {
  description: "Fillet fails on the hole:\n\n  GeopError: the fillet is too large for the edge\n",
  program: { steps: [{ id: "box", operation: "primitive", args: { block: [0, 0, 0, 1, 1, 1] } }] },
  step: {
    id: null,
    index: 1,
    placed: null,
    step: { id: "fillet1", operation: "fillet", args: { edges: ["e1"], radius: 2 } },
    missing: [],
    error: "the fillet is too large for the edge",
    concerning: "radius",
  },
  kernel: { version: "0.11.0", commit: "d1ec42cf" },
  meta: { url: "https://app.geop-cad.dev/", userAgent: "test", timestamp: "2026-10-31T12:00:00.000Z", lastError: null },
};

test("a bug report is stored whole: the step being edited, the kernel, the description as typed", async () => {
  const bucket = new MemoryBucket();
  const body = JSON.stringify(report);
  const request = new Request("https://app.geop-cad.dev/api/bug-report", {
    method: "POST",
    headers: { "content-type": "application/json", "content-length": String(body.length) },
    body,
  });
  const context = { request, env: { BUG_REPORTS: bucket } } as unknown as Parameters<typeof onRequestPost>[0];
  const response = await onRequestPost(context);
  assert.equal(response.status, 201);
  const [stored] = [...bucket.objects.values()];
  assert.deepEqual(JSON.parse(stored), report);
});
