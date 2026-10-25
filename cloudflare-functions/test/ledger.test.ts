import assert from "node:assert/strict";
import { test } from "node:test";
import {
  GLOBAL_TURNS_PER_MINUTE,
  IP_SESSIONS_PER_DAY,
  IP_TURNS_PER_DAY,
  RESERVATION_TTL_MS,
  SESSION_TURNS_PER_DAY,
  SESSION_TURNS_PER_MINUTE,
} from "../lib/agent/config.ts";
import { admit, available, ledger, openSession, reserve, settle, type Budgets } from "../lib/agent/ledger.ts";
import { MemoryKv } from "./memory-kv.ts";

const USD = 1e9;
const budgets: Budgets = { daily: 20 * USD, total: 100 * USD };
const t0 = Date.parse("2026-10-25T12:00:00Z");
const MIN = 60_000;
let n = 0;
const id = () => `r${n++}`;

/** One whole turn, admitted, reserved and settled; spaced so no rate limit is hit. */
function turn(kv: MemoryKv, b: Budgets, sid: string, ip: string, now: number, amount: number, cost: number) {
  const a = admit(kv, b, { sid, ip, now });
  if (!a.ok) return a;
  const rid = id();
  const r = reserve(kv, b, { sid, ip, now, id: rid, amount });
  if (r.ok) settle(kv, { id: rid, cost, now });
  return r;
}

test("reserve before, settle after: the day and the total carry the actual cost", () => {
  const kv = new MemoryKv();
  const r = reserve(kv, budgets, { sid: "s", ip: "i", now: t0, id: "a", amount: 1 * USD });
  assert.ok(r.ok);
  assert.deepEqual(r.limits, { turns_left_today: SESSION_TURNS_PER_DAY - 1, turns_per_day: SESSION_TURNS_PER_DAY });
  // While open, the reservation counts against the remainder.
  assert.equal(ledger(kv).total, 0);
  settle(kv, { id: "a", cost: 0.25 * USD, now: t0 + 1000 });
  const l = ledger(kv);
  assert.equal(l.total, 0.25 * USD);
  assert.deepEqual(l.days["2026-10-25"], { spent: 0.25 * USD, turns: 1, sessions: 0, swept: 0 });
});

test("a reservation that does not fit the day's remainder is refused, open reservations counted", () => {
  const kv = new MemoryKv();
  const small: Budgets = { daily: 1 * USD, total: 100 * USD };
  assert.ok(reserve(kv, small, { sid: "a", ip: "1", now: t0, id: "x", amount: 0.6 * USD }).ok);
  const refused = reserve(kv, small, { sid: "b", ip: "2", now: t0, id: "y", amount: 0.6 * USD });
  assert.equal(refused.ok, false);
  assert.ok(!refused.ok && refused.error === "budget" && refused.retry_after === 12 * 3600);
  // Settling below the reservation frees the rest.
  settle(kv, { id: "x", cost: 0.1 * USD, now: t0 });
  assert.ok(reserve(kv, small, { sid: "b", ip: "2", now: t0, id: "y", amount: 0.6 * USD }).ok);
});

test("the daily cap resets at 00:00 UTC; the total never does", () => {
  const kv = new MemoryKv();
  const b: Budgets = { daily: 1 * USD, total: 1.5 * USD };
  assert.ok(turn(kv, b, "s", "i", t0, 1 * USD, 1 * USD).ok);
  const sameDay = admit(kv, b, { sid: "s2", ip: "i2", now: t0 + MIN });
  assert.ok(!sameDay.ok && sameDay.error === "budget" && sameDay.retry_after !== undefined);
  const nextDay = t0 + 24 * 3600 * 1000;
  assert.ok(turn(kv, b, "s", "i", nextDay, 0.5 * USD, 0.5 * USD).ok);
  // 1.5 of 1.5 spent over two days: paused for good, with no time to come back.
  const after = admit(kv, b, { sid: "s3", ip: "i3", now: nextDay + 2 * 24 * 3600 * 1000 });
  assert.ok(!after.ok && after.error === "budget" && after.retry_after === undefined);
  assert.equal(available(kv, b, nextDay + 3 * 24 * 3600 * 1000), false);
  // Raising the total (the var) is all it takes to recharge.
  assert.equal(available(kv, { daily: 1 * USD, total: 2 * USD }, nextDay + 3 * 24 * 3600 * 1000), true);
  assert.equal(ledger(kv).total, 1.5 * USD);
});

test("caps of zero (a missing var) refuse everything", () => {
  const kv = new MemoryKv();
  const closed: Budgets = { daily: 0, total: 0 };
  assert.equal(available(kv, closed, t0), false);
  assert.ok(!admit(kv, closed, { sid: "s", ip: "i", now: t0 }).ok);
  assert.ok(!openSession(kv, closed, { sid: "s", ip: "i", now: t0 }).ok);
});

test("a reservation never settled is charged in full after its TTL, and a late settle is not charged again", () => {
  const kv = new MemoryKv();
  assert.ok(reserve(kv, budgets, { sid: "s", ip: "i", now: t0, id: "lost", amount: 0.3 * USD }).ok);
  available(kv, budgets, t0 + RESERVATION_TTL_MS + 1);
  assert.equal(ledger(kv).total, 0.3 * USD);
  assert.equal(ledger(kv).days["2026-10-25"].swept, 1);
  settle(kv, { id: "lost", cost: 0.1 * USD, now: t0 + RESERVATION_TTL_MS + 2 });
  assert.equal(ledger(kv).total, 0.3 * USD);
});

test("a settle with no reservation on record is charged", () => {
  const kv = new MemoryKv();
  settle(kv, { id: "unknown", cost: 5, now: t0 });
  assert.equal(ledger(kv).total, 5);
});

test("per-session minute rate", () => {
  const kv = new MemoryKv();
  for (let i = 0; i < SESSION_TURNS_PER_MINUTE; i++) assert.ok(admit(kv, budgets, { sid: "s", ip: "i", now: t0 + i }).ok);
  const refused = admit(kv, budgets, { sid: "s", ip: "i", now: t0 + 1000 });
  assert.ok(!refused.ok && refused.error === "rate" && refused.retry_after === 59);
  // Another session from the same address is not limited by this one's rate.
  assert.ok(admit(kv, budgets, { sid: "t", ip: "i", now: t0 + 1000 }).ok);
  // A minute after the first, one slot is free again.
  assert.ok(admit(kv, budgets, { sid: "s", ip: "i", now: t0 + MIN }).ok);
});

test("global minute rate holds over many addresses", () => {
  const kv = new MemoryKv();
  for (let i = 0; i < GLOBAL_TURNS_PER_MINUTE; i++) assert.ok(admit(kv, budgets, { sid: `s${i}`, ip: `ip${i}`, now: t0 }).ok);
  const refused = admit(kv, budgets, { sid: "x", ip: "x", now: t0 });
  assert.ok(!refused.ok && refused.error === "rate");
});

test("per-session daily quota, renewed the next UTC day", () => {
  const kv = new MemoryKv();
  for (let i = 0; i < SESSION_TURNS_PER_DAY; i++) assert.ok(turn(kv, budgets, "s", "i", t0 + i * MIN, 1000, 10).ok);
  const refused = admit(kv, budgets, { sid: "s", ip: "i", now: t0 + SESSION_TURNS_PER_DAY * MIN });
  assert.ok(!refused.ok && refused.error === "quota");
  assert.ok(turn(kv, budgets, "s", "i", t0 + 24 * 3600 * 1000, 1000, 10).ok);
});

test("per-address daily quota spans its sessions", () => {
  const kv = new MemoryKv();
  for (let i = 0; i < IP_TURNS_PER_DAY; i++) assert.ok(turn(kv, budgets, `s${Math.floor(i / 30)}`, "ip", t0 + i * MIN, 1000, 10).ok);
  const refused = admit(kv, budgets, { sid: "fresh", ip: "ip", now: t0 + IP_TURNS_PER_DAY * MIN });
  assert.ok(!refused.ok && refused.error === "quota");
});

test("two turns racing past admit cannot both take the session's last turn", () => {
  const kv = new MemoryKv();
  for (let i = 0; i < SESSION_TURNS_PER_DAY - 1; i++) assert.ok(turn(kv, budgets, "s", "i", t0 + i * MIN, 1000, 10).ok);
  const now = t0 + SESSION_TURNS_PER_DAY * MIN;
  assert.ok(admit(kv, budgets, { sid: "s", ip: "i", now }).ok);
  assert.ok(admit(kv, budgets, { sid: "s", ip: "i", now }).ok);
  assert.ok(reserve(kv, budgets, { sid: "s", ip: "i", now, id: "first", amount: 1 }).ok);
  const second = reserve(kv, budgets, { sid: "s", ip: "i", now, id: "second", amount: 1 });
  assert.ok(!second.ok && second.error === "quota");
});

test("sessions per address per day", () => {
  const kv = new MemoryKv();
  for (let i = 0; i < IP_SESSIONS_PER_DAY; i++) assert.ok(openSession(kv, budgets, { sid: `s${i}`, ip: "ip", now: t0 }).ok);
  const refused = openSession(kv, budgets, { sid: "more", ip: "ip", now: t0 });
  assert.ok(!refused.ok && refused.error === "rate");
  assert.ok(openSession(kv, budgets, { sid: "other", ip: "ip2", now: t0 }).ok);
  assert.ok(openSession(kv, budgets, { sid: "tomorrow", ip: "ip", now: t0 + 24 * 3600 * 1000 }).ok);
  assert.equal(ledger(kv).days["2026-10-25"].sessions, IP_SESSIONS_PER_DAY + 1);
});

test("concurrent reservations never let settled spend pass the cap", () => {
  // Many turns reserving at once, each settling at most its reservation:
  // whatever order they settle in, the day never goes over.
  const kv = new MemoryKv();
  const b: Budgets = { daily: 1 * USD, total: 100 * USD };
  const open: string[] = [];
  for (let i = 0; i < 50; i++) {
    const rid = `c${i}`;
    if (reserve(kv, b, { sid: `s${i}`, ip: `i${i}`, now: t0, id: rid, amount: 0.15 * USD }).ok) open.push(rid);
  }
  assert.equal(open.length, 6);
  for (const rid of open) settle(kv, { id: rid, cost: 0.15 * USD, now: t0 });
  assert.ok(ledger(kv).days["2026-10-25"].spent <= b.daily);
});

test("status says unavailable once no turn's least reservation fits", () => {
  const kv = new MemoryKv();
  const b: Budgets = { daily: 1 * USD, total: 100 * USD };
  assert.ok(turn(kv, b, "s", "i", t0, 0.95 * USD, 0.95 * USD).ok);
  assert.equal(available(kv, b, t0, 0.01 * USD), true);
  assert.equal(available(kv, b, t0, 0.1 * USD), false);
});
