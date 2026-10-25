/**
 * Server-side Turnstile check: a token passes only if siteverify says it
 * succeeded, for the action the web app renders the widget with, on a
 * hostname of ours. siteverify itself makes each token single-use.
 */

const SITEVERIFY = "https://challenges.cloudflare.com/turnstile/v0/siteverify";

/** The `action` the browser renders the widget with. */
export const TURNSTILE_ACTION = "agent_session";

/** Where the widget may have been solved in production. Never add localhost here. */
export const TURNSTILE_HOSTNAMES: readonly string[] = ["app.geop-cad.dev"];

/**
 * Accepted only with `TURNSTILE_ALLOW_TEST_HOSTNAMES=1`, which is for
 * `wrangler pages dev` with Cloudflare's test keys and must never be set in
 * production. ("example.com" is what siteverify answers for a test key.)
 */
export const TURNSTILE_TEST_HOSTNAMES: readonly string[] = ["localhost", "127.0.0.1", "example.com"];

export interface TurnstileOutcome {
  passed: boolean;
  /** Why it did not pass, for the log: siteverify's error codes or which check failed. Never the token. */
  reason?: string;
}

export async function verifyTurnstile(
  secret: string,
  token: string,
  ip: string,
  allowTestHostnames: boolean,
  fetcher: typeof fetch = fetch,
): Promise<TurnstileOutcome> {
  const form = new FormData();
  form.append("secret", secret);
  form.append("response", token);
  form.append("remoteip", ip);
  let outcome: { success?: unknown; action?: unknown; hostname?: unknown; "error-codes"?: unknown };
  try {
    const response = await fetcher(SITEVERIFY, { method: "POST", body: form });
    outcome = await response.json();
  } catch {
    return { passed: false, reason: "siteverify unreachable" };
  }
  if (outcome.success !== true) {
    const codes = Array.isArray(outcome["error-codes"]) ? outcome["error-codes"].filter((c) => typeof c === "string") : [];
    return { passed: false, reason: `unsuccessful: ${codes.join(",")}` };
  }
  const hostnames = allowTestHostnames ? [...TURNSTILE_HOSTNAMES, ...TURNSTILE_TEST_HOSTNAMES] : TURNSTILE_HOSTNAMES;
  if (typeof outcome.hostname !== "string" || !hostnames.includes(outcome.hostname)) return { passed: false, reason: "hostname" };
  // A test key answers no action of ours; outside test mode the action must be ours.
  if (outcome.action !== TURNSTILE_ACTION && !(allowTestHostnames && TURNSTILE_TEST_HOSTNAMES.includes(outcome.hostname))) {
    return { passed: false, reason: "action" };
  }
  return { passed: true };
}
