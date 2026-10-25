import assert from "node:assert/strict";
import { test } from "node:test";
import { TURNSTILE_ACTION, TURNSTILE_HOSTNAMES, verifyTurnstile } from "../lib/agent/turnstile.ts";

/** A siteverify that answers `outcome`, and checks what it was sent. */
function siteverify(outcome: unknown): typeof fetch {
  return (async (_url: unknown, init?: RequestInit) => {
    const form = init?.body as FormData;
    assert.equal(form.get("secret"), "secret");
    assert.equal(form.get("response"), "token");
    assert.equal(form.get("remoteip"), "1.2.3.4");
    return Response.json(outcome);
  }) as typeof fetch;
}

const check = (outcome: unknown, allowTest = false) => verifyTurnstile("secret", "token", "1.2.3.4", allowTest, siteverify(outcome));
const good = { success: true, action: TURNSTILE_ACTION, hostname: "app.geop-cad.dev", "error-codes": [] };

test("success, our action and our hostname pass", async () => {
  assert.deepEqual(await check(good), { passed: true });
});

test("success with another action is refused", async () => {
  assert.deepEqual(await check({ ...good, action: "login" }), { passed: false, reason: "action" });
  assert.deepEqual(await check({ ...good, action: undefined }), { passed: false, reason: "action" });
});

test("success on another hostname is refused", async () => {
  for (const hostname of ["evil.example", "localhost", "127.0.0.1", "example.com", undefined]) {
    assert.deepEqual(await check({ ...good, hostname }), { passed: false, reason: "hostname" });
  }
});

test("no success is refused, with only the error codes kept", async () => {
  assert.deepEqual(await check({ success: false, "error-codes": ["timeout-or-duplicate"] }), { passed: false, reason: "unsuccessful: timeout-or-duplicate" });
  assert.deepEqual(await check({ success: "true", action: TURNSTILE_ACTION, hostname: "app.geop-cad.dev" }), { passed: false, reason: "unsuccessful: " });
});

test("siteverify unreachable is refused", async () => {
  const down = (async () => { throw new TypeError("network"); }) as typeof fetch;
  assert.deepEqual(await verifyTurnstile("secret", "token", "1.2.3.4", false, down), { passed: false, reason: "siteverify unreachable" });
});

test("test hostnames only with the dev flag, and the production list has no local host", async () => {
  const testKey = { success: true, hostname: "example.com", "error-codes": [], metadata: { result_with_testing_key: true } };
  assert.equal((await check(testKey)).passed, false);
  assert.equal((await check(testKey, true)).passed, true);
  for (const local of ["localhost", "127.0.0.1"]) assert.ok(!TURNSTILE_HOSTNAMES.includes(local));
});
