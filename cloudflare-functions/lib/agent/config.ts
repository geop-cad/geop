/**
 * Every tunable of the hosted agent chat in one place. The money caps
 * themselves are not here: they are the budget worker's vars
 * (`AGENT_DAILY_BUDGET_USD`, `AGENT_TOTAL_BUDGET_USD`, see
 * budget-worker/wrangler.toml), so raising one is a var change and a
 * redeploy of that worker, nothing else.
 */

// Per session (one cookie, one Turnstile solve, 24 h).
export const SESSION_TURNS_PER_DAY = 40;
export const SESSION_TURNS_PER_MINUTE = 6;
export const SESSION_LIFETIME_SECONDS = 24 * 60 * 60;

// Per IP address (CF-Connecting-IP).
export const IP_TURNS_PER_DAY = 80;
export const IP_TURNS_PER_MINUTE = 12;
export const IP_SESSIONS_PER_DAY = 3;

// Over everyone: a flood from many addresses still cannot burn the day's
// budget in minutes, and cannot open sessions without end.
export const GLOBAL_TURNS_PER_MINUTE = 60;
export const GLOBAL_SESSIONS_PER_DAY = 500;

// Request shape.
/** The body without the one image's base64 data. */
export const MAX_BODY_BYTES = 256 * 1024;
/** The base64 data of the one image a request may carry (a viewport screenshot). */
export const MAX_IMAGE_BASE64_BYTES = 300 * 1024;
export const MAX_IMAGES_PER_REQUEST = 1;
export const MAX_MESSAGES = 80;
/**
 * A user text block: room for the program the browser sends as context
 * with a prompt (at most 20 000 characters, `LONGEST_PROGRAM` in
 * web/src/agent.ts, and its framing). The prompt typed is capped at 4000
 * by the chat box; the cost of a request is bounded by its counted tokens
 * either way (`MAX_INPUT_TOKENS`, the reservation).
 */
export const MAX_USER_TEXT_CHARS = 24_000;
/**
 * Input tokens a request may have, counted before it is sent. Keeps every
 * request in its model's base price band (Haiku 5.5 charges more above
 * 100K) and stops a conversation growing without end.
 */
export const MAX_INPUT_TOKENS = 100_000;

// The model call.
export const DEFAULT_MODEL = "claude-sonnet-5-5";
/** Thinking counts towards it; a tool-calling turn at medium effort fits comfortably. */
export const MAX_TOKENS = 8192;
export const DEFAULT_EFFORT = "medium";
/** The SDK's own timeout; a reservation outlives it (RESERVATION_TTL_MS). */
export const UPSTREAM_TIMEOUT_MS = 5 * 60 * 1000;

// Accounting.
/**
 * A reservation not settled after this long (its request died between
 * reserving and settling) is charged in full: the API may have billed it.
 * Longer than UPSTREAM_TIMEOUT_MS, so no live request is swept.
 */
export const RESERVATION_TTL_MS = 15 * 60 * 1000;
/**
 * `count_tokens` is documented as an estimate: the reservation takes this
 * share and this many tokens on top of it, so it stays an upper bound.
 */
export const COUNT_MARGIN_RATIO = 0.1;
export const COUNT_MARGIN_TOKENS = 500;

/** Prices in nanodollars (1e-9 USD) per token, which is USD per million tokens times 1000. */
export interface Price {
  input: number;
  /** 5-minute cache writes, 1.25x input. */
  cacheWrite5m: number;
  /** 1-hour cache writes, 2x input (not used by this endpoint, priced in case usage reports them). */
  cacheWrite1h: number;
  cacheRead: number;
  output: number;
}

function price(inputPerMTok: number, cacheReadPerMTok: number, outputPerMTok: number): Price {
  const n = (usdPerMTok: number) => Math.round(usdPerMTok * 1000);
  return {
    input: n(inputPerMTok),
    cacheWrite5m: n(inputPerMTok * 1.25),
    cacheWrite1h: n(inputPerMTok * 2),
    cacheRead: n(cacheReadPerMTok),
    output: n(outputPerMTok),
  };
}

/** Models the endpoint may run; any other `AGENT_MODEL` refuses to start. */
export const PRICES: Record<string, Price> = {
  "claude-opus-5-5": price(4, 0.2, 20),
  "claude-sonnet-5-5": price(2, 0.2, 10),
  // Below 100K prompt tokens (MAX_INPUT_TOKENS keeps every request there).
  "claude-haiku-5-5": price(0.1, 0.01, 0.5),
};

export const EFFORTS = ["low", "medium", "high"] as const;
export type Effort = (typeof EFFORTS)[number];

export const NANO_PER_USD = 1e9;
