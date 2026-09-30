/**
 * Collects a bug report into R2. Two bindings needed on this Pages project
 * (Settings -> Functions):
 *   - R2 bucket bindings: `BUG_REPORTS`
 *   - KV namespace bindings: `RATE_LIMIT` (same namespace as the landing
 *     page's contact form — see homepage/functions/api/contact.ts — bind
 *     it to both projects)
 * Either binding missing just means that layer of protection is off, not
 * that the endpoint 500s — so this works (unprotected) before either is
 * set up, and gets safer as each is added.
 *
 * Defense in depth against a bot running up a bill, in order:
 *   1. MAX_BODY_BYTES rejects an oversized single report outright.
 *   2. checkRateLimit caps each IP to a handful of reports a day.
 *   3. checkTotalSizeBudget refuses ANY new report once the bucket's
 *      approximate running total nears the R2 free tier's 10 GB, so a
 *      flood from many distinct IPs (which (2) alone can't stop) still
 *      cannot grow the bucket past that. It's an approximate KV counter
 *      (eventually consistent, not atomic under concurrent writes), so the
 *      ceiling is set with real headroom below 10 GB — this is a
 *      best-effort backstop, not a substitute for also setting a Cloudflare
 *      billing/usage alert on the account, which is the authoritative one.
 *   4. To also get notified per report, add a Cloudflare Email Routing
 *      "Send Email" binding and extend this handler to send through it —
 *      reports still land in R2 either way, so that's a pure addition.
 */

const MAX_BODY_BYTES = 1024 * 1024; // 1 MB: a real report is a few KB; this still leaves ~200x headroom.
const MAX_REPORTS_PER_IP_PER_DAY = 5;
const TOTAL_SIZE_CEILING_BYTES = 8 * 1024 * 1024 * 1024; // 8 GB: 2 GB of headroom under R2's 10 GB free tier.

interface Env {
  BUG_REPORTS: R2Bucket;
  RATE_LIMIT?: KVNamespace;
}

/** At most `limit` requests per `ip` per UTC day. Fails closed if the KV write itself fails (e.g. its own quota is exhausted). */
async function checkRateLimit(env: Env, ip: string, bucket: string, limit: number): Promise<boolean> {
  if (!env.RATE_LIMIT) return true; // not bound yet — don't block before it's set up
  const key = `${bucket}:${ip}:${new Date().toISOString().slice(0, 10)}`;
  const count = Number((await env.RATE_LIMIT.get(key)) ?? "0");
  if (count >= limit) return false;
  try {
    await env.RATE_LIMIT.put(key, String(count + 1), { expirationTtl: 60 * 60 * 24 });
  } catch {
    return false;
  }
  return true;
}

/** Refuses once the bucket's approximate running total would cross `ceiling`. See the module doc comment for its honesty caveats. */
async function checkTotalSizeBudget(env: Env, addingBytes: number, ceiling: number): Promise<boolean> {
  if (!env.RATE_LIMIT) return true;
  const key = "bugreports-total-bytes";
  const total = Number((await env.RATE_LIMIT.get(key)) ?? "0");
  if (total + addingBytes > ceiling) return false;
  try {
    await env.RATE_LIMIT.put(key, String(total + addingBytes));
  } catch {
    return false;
  }
  return true;
}

export const onRequestPost: PagesFunction<Env> = async ({ request, env }) => {
  const contentLength = Number(request.headers.get("content-length") ?? "0");
  if (contentLength > MAX_BODY_BYTES) {
    return new Response("Report too large", { status: 413 });
  }

  const ip = request.headers.get("CF-Connecting-IP") ?? "unknown";
  if (!(await checkRateLimit(env, ip, "bugreport", MAX_REPORTS_PER_IP_PER_DAY))) {
    return new Response("Too many reports from this address today — try again tomorrow.", { status: 429 });
  }

  let payload: unknown;
  try {
    payload = await request.json();
  } catch {
    return new Response("Invalid JSON body", { status: 400 });
  }
  if (typeof payload !== "object" || payload === null) {
    return new Response("Invalid JSON body", { status: 400 });
  }

  const json = JSON.stringify(payload, null, 2);
  if (!(await checkTotalSizeBudget(env, json.length, TOTAL_SIZE_CEILING_BYTES))) {
    return new Response("Bug report storage is temporarily full — please email instead.", { status: 507 });
  }

  const id = crypto.randomUUID();
  const key = `${new Date().toISOString().replace(/[:.]/g, "-")}-${id}.json`;
  await env.BUG_REPORTS.put(key, json, {
    httpMetadata: { contentType: "application/json" },
  });

  return new Response(JSON.stringify({ ok: true, id }), {
    status: 201,
    headers: { "content-type": "application/json" },
  });
};
