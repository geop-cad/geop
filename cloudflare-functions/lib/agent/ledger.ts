/**
 * The agent chat's accounting: money, turns and rates, over a synchronous
 * key-value store. The budget worker's Durable Object runs these
 * functions on its storage (`ctx.storage.kv`), one call at a time, so
 * every check-and-update here is atomic: a cap checked is a cap held.
 *
 * Money is in nanodollars (integers). A turn first `admit`s (rates and
 * quotas, before anything is asked of the API), then `reserve`s an upper
 * bound of its cost (refused if the bound does not fit the day's or the
 * lifetime's remainder, reservations still open counted as spent), then
 * `settle`s to what the response's usage says it cost. A reservation never
 * settled is charged in full once RESERVATION_TTL_MS has passed.
 *
 * Keys:
 *   total              lifetime spend, never reset
 *   day:<YYYY-MM-DD>   the day's ledger (DayRecord), never deleted
 *   res:<id>           an open reservation (Reservation)
 *   cnt:<key>          a counter of the day in `counters_day`
 *   win:<key>          the timestamps of the last minute's turns
 *   swept:<id>         a reservation charged in full, so a late settle is not charged again
 */
import {
  GLOBAL_SESSIONS_PER_DAY,
  GLOBAL_TURNS_PER_MINUTE,
  IP_SESSIONS_PER_DAY,
  IP_TURNS_PER_DAY,
  IP_TURNS_PER_MINUTE,
  RESERVATION_TTL_MS,
  SESSION_TURNS_PER_DAY,
  SESSION_TURNS_PER_MINUTE,
} from "./config.ts";

export interface Kv {
  get<T>(key: string): T | undefined;
  put<T>(key: string, value: T): void;
  delete(key: string): boolean;
  list<T>(options: { prefix: string }): Iterable<[string, T]>;
}

/** The caps in nanodollars; 0 for a cap that is missing or unreadable, which closes the endpoint. */
export interface Budgets {
  daily: number;
  total: number;
}

export interface DayRecord {
  /** Settled spend, plus reservations charged in full. */
  spent: number;
  turns: number;
  sessions: number;
  /** Reservations charged in full because they were never settled. */
  swept: number;
}

interface Reservation {
  amount: number;
  day: string;
  at: number;
}

export interface Limits {
  turns_left_today: number;
  turns_per_day: number;
}

export type Refusal = {
  ok: false;
  error: "rate" | "quota" | "budget";
  message: string;
  retry_after?: number;
};
export type Granted = { ok: true; limits: Limits };

const MINUTE_MS = 60_000;

export function dayOf(now: number): string {
  return new Date(now).toISOString().slice(0, 10);
}

function secondsToMidnight(now: number): number {
  const midnight = new Date(dayOf(now) + "T00:00:00Z").getTime() + 24 * 60 * 60 * 1000;
  return Math.ceil((midnight - now) / 1000);
}

function dayRecord(kv: Kv, day: string): DayRecord {
  return kv.get<DayRecord>(`day:${day}`) ?? { spent: 0, turns: 0, sessions: 0, swept: 0 };
}

function updateDay(kv: Kv, day: string, change: (record: DayRecord) => void) {
  const record = dayRecord(kv, day);
  change(record);
  kv.put(`day:${day}`, record);
}

function charge(kv: Kv, day: string, amount: number, swept: boolean) {
  kv.put("total", (kv.get<number>("total") ?? 0) + amount);
  updateDay(kv, day, (record) => {
    record.spent += amount;
    if (swept) record.swept += 1;
  });
}

/** Charges reservations past their TTL in full, and drops the counters and windows of past days. */
function housekeep(kv: Kv, now: number) {
  for (const [key, reservation] of [...kv.list<Reservation>({ prefix: "res:" })]) {
    if (now - reservation.at > RESERVATION_TTL_MS) {
      kv.delete(key);
      kv.put(`swept:${key.slice(4)}`, true);
      charge(kv, reservation.day, reservation.amount, true);
    }
  }
  const today = dayOf(now);
  if (kv.get<string>("counters_day") !== today) {
    for (const prefix of ["cnt:", "win:"]) {
      for (const [key] of [...kv.list({ prefix })]) kv.delete(key);
    }
    kv.put("counters_day", today);
  }
}

/** What is left of the day's and the lifetime's budget, open reservations counted as spent. */
function remaining(kv: Kv, budgets: Budgets, now: number): { day: number; total: number } {
  const today = dayOf(now);
  let pendingToday = 0;
  let pending = 0;
  for (const [, reservation] of kv.list<Reservation>({ prefix: "res:" })) {
    pending += reservation.amount;
    if (reservation.day === today) pendingToday += reservation.amount;
  }
  return {
    day: budgets.daily - dayRecord(kv, today).spent - pendingToday,
    total: budgets.total - (kv.get<number>("total") ?? 0) - pending,
  };
}

/** Refuses when `amount` does not fit what is left; null when it does. */
function budgetRefusal(kv: Kv, budgets: Budgets, now: number, amount: number): Refusal | null {
  const left = remaining(kv, budgets, now);
  if (left.total < amount || left.total <= 0) {
    return { ok: false, error: "budget", message: "The free assistant is paused: its budget is used up." };
  }
  if (left.day < amount || left.day <= 0) {
    return {
      ok: false,
      error: "budget",
      message: "The free assistant is paused for today. It is back at 00:00 UTC.",
      retry_after: secondsToMidnight(now),
    };
  }
  return null;
}

const counter = (kv: Kv, key: string) => kv.get<number>(`cnt:${key}`) ?? 0;
const bump = (kv: Kv, key: string) => kv.put(`cnt:${key}`, counter(kv, key) + 1);

function window(kv: Kv, key: string, now: number): number[] {
  return (kv.get<number[]>(`win:${key}`) ?? []).filter((at) => now - at < MINUTE_MS);
}

export function limitsOf(kv: Kv, sid: string, ip: string): Limits {
  const left = Math.min(
    SESSION_TURNS_PER_DAY - counter(kv, `turns:s:${sid}`),
    IP_TURNS_PER_DAY - counter(kv, `turns:ip:${ip}`),
  );
  return { turns_left_today: Math.max(0, left), turns_per_day: SESSION_TURNS_PER_DAY };
}

function quotaRefusal(kv: Kv, sid: string, ip: string, now: number): Refusal | null {
  if (limitsOf(kv, sid, ip).turns_left_today > 0) return null;
  return {
    ok: false,
    error: "quota",
    message: "You have used today's free assistant turns. They renew at 00:00 UTC.",
    retry_after: secondsToMidnight(now),
  };
}

export interface Caller {
  sid: string;
  ip: string;
  now: number;
}

/**
 * Lets a turn ask the API anything (even to count its tokens): refuses
 * when the budget is gone, the caller's turns for the day are used, or a
 * per-minute rate is reached. Counts the turn in the minute windows.
 */
export function admit(kv: Kv, budgets: Budgets, { sid, ip, now }: Caller): Granted | Refusal {
  housekeep(kv, now);
  const refusal = budgetRefusal(kv, budgets, now, 0) ?? quotaRefusal(kv, sid, ip, now);
  if (refusal) return refusal;
  const windows: [string, number][] = [
    [`s:${sid}`, SESSION_TURNS_PER_MINUTE],
    [`ip:${ip}`, IP_TURNS_PER_MINUTE],
    ["global", GLOBAL_TURNS_PER_MINUTE],
  ];
  for (const [key, limit] of windows) {
    const recent = window(kv, key, now);
    if (recent.length >= limit) {
      return {
        ok: false,
        error: "rate",
        message: key === "global" ? "The assistant is busy. Try again in a minute." : "Too many requests. Wait a moment.",
        retry_after: Math.max(1, Math.ceil((recent[0] + MINUTE_MS - now) / 1000)),
      };
    }
  }
  for (const [key] of windows) kv.put(`win:${key}`, [...window(kv, key, now), now]);
  return { ok: true, limits: limitsOf(kv, sid, ip) };
}

/**
 * Holds `amount` (an upper bound of the turn's cost) under `id`, and
 * counts the turn against the caller's day. Refused when the amount does
 * not fit the day's or the lifetime's remainder, or the caller's turns ran
 * out since `admit` (two turns racing).
 */
export function reserve(kv: Kv, budgets: Budgets, caller: Caller & { id: string; amount: number }): Granted | Refusal {
  const { sid, ip, now, id, amount } = caller;
  housekeep(kv, now);
  const refusal = budgetRefusal(kv, budgets, now, amount) ?? quotaRefusal(kv, sid, ip, now);
  if (refusal) return refusal;
  const day = dayOf(now);
  kv.put<Reservation>(`res:${id}`, { amount, day, at: now });
  bump(kv, `turns:s:${sid}`);
  bump(kv, `turns:ip:${ip}`);
  updateDay(kv, day, (record) => (record.turns += 1));
  return { ok: true, limits: limitsOf(kv, sid, ip) };
}

/**
 * Closes the reservation `id`, charging `cost`, what the turn did cost.
 * One already swept was charged in full and is not charged again; with no
 * record of `id` at all, `cost` is charged to today.
 */
export function settle(kv: Kv, { id, cost, now }: { id: string; cost: number; now: number }) {
  const reservation = kv.get<Reservation>(`res:${id}`);
  if (reservation) {
    kv.delete(`res:${id}`);
    charge(kv, reservation.day, cost, false);
  } else if (!kv.get<boolean>(`swept:${id}`)) {
    charge(kv, dayOf(now), cost, false);
  }
  housekeep(kv, now);
}

/** Lets an address open a session (after its Turnstile token was verified). */
export function openSession(kv: Kv, budgets: Budgets, { sid, ip, now }: Caller): Granted | Refusal {
  housekeep(kv, now);
  const refusal = budgetRefusal(kv, budgets, now, 0);
  if (refusal) return refusal;
  const day = dayOf(now);
  if (counter(kv, `sessions:ip:${ip}`) >= IP_SESSIONS_PER_DAY || dayRecord(kv, day).sessions >= GLOBAL_SESSIONS_PER_DAY) {
    return {
      ok: false,
      error: "rate",
      message: "Too many new sessions today. Try again tomorrow.",
      retry_after: secondsToMidnight(now),
    };
  }
  bump(kv, `sessions:ip:${ip}`);
  updateDay(kv, day, (record) => (record.sessions += 1));
  return { ok: true, limits: limitsOf(kv, sid, ip) };
}

/** Whether a turn reserving `minimum` (the least any turn reserves) would fit the budget now. Nothing else is said. */
export function available(kv: Kv, budgets: Budgets, now: number, minimum = 0): boolean {
  housekeep(kv, now);
  return budgetRefusal(kv, budgets, now, minimum) === null;
}

/** For the owner: the lifetime spend and every day's record. */
export function ledger(kv: Kv): { total: number; days: Record<string, DayRecord> } {
  const days: Record<string, DayRecord> = {};
  for (const [key, record] of kv.list<DayRecord>({ prefix: "day:" })) days[key.slice(4)] = record;
  return { total: kv.get<number>("total") ?? 0, days };
}
