import assert from "node:assert/strict";
import { test } from "node:test";
import { SESSION_LIFETIME_SECONDS } from "../lib/agent/config.ts";
import { cookieValue, sessionCookie, signSession, verifySession } from "../lib/agent/session.ts";

const sid = "0b9f4c1e-6f1a-4c55-9d2a-3c4f5e6a7b8c";
const now = 1_790_000_000_000;

test("a signed session verifies until it expires", async () => {
  const value = await signSession("secret", sid, now);
  assert.equal(await verifySession("secret", value, now), sid);
  assert.equal(await verifySession("secret", value, now + (SESSION_LIFETIME_SECONDS - 1) * 1000), sid);
  assert.equal(await verifySession("secret", value, now + SESSION_LIFETIME_SECONDS * 1000), null);
});

test("a forged, re-dated or foreign session does not verify", async () => {
  const value = await signSession("secret", sid, now);
  const [, expiry, signature] = value.split(".");
  assert.equal(await verifySession("other", value, now), null);
  assert.equal(await verifySession("secret", `${sid}.${Number(expiry) + 1000}.${signature}`, now), null);
  assert.equal(await verifySession("secret", `${sid.replace("0", "1")}.${expiry}.${signature}`, now), null);
  assert.equal(await verifySession("secret", "garbage", now), null);
  assert.equal(await verifySession("secret", `${sid}.${expiry}.!!!`, now), null);
});

test("the cookie is HttpOnly, Secure, SameSite=Strict on /api/agent, and reads back", () => {
  const cookie = sessionCookie("v.1.s");
  for (const part of ["geop_agent=v.1.s", "HttpOnly", "Secure", "SameSite=Strict", "Path=/api/agent"]) assert.ok(cookie.includes(part));
  assert.equal(cookieValue("a=b; geop_agent=v.1.s; c=d"), "v.1.s");
  assert.equal(cookieValue(null), null);
});
