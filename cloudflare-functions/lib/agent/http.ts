/** What the agent endpoints share: their environment, its configuration, and the JSON error contract. */
import type { AgentBudget } from "../../budget-worker/src/index.ts";
import { DEFAULT_EFFORT, DEFAULT_MODEL, EFFORTS, PRICES, type Effort, type Price } from "./config.ts";
import type { Refusal } from "./ledger.ts";

/** The Pages project's bindings, secrets and vars (see docs/agent-chat.md). */
export interface AgentEnv {
  /** Durable Object binding to the class `AgentBudget` of the worker `geop-agent-budget`. */
  AGENT_BUDGET?: DurableObjectNamespace<AgentBudget>;
  ANTHROPIC_API_KEY?: string;
  SESSION_SECRET?: string;
  TURNSTILE_SECRET?: string;
  AGENT_MODEL?: string;
  AGENT_EFFORT?: string;
  /** "1" turns the assistant off. */
  AGENT_DISABLED?: string;
  /** "1" accepts Turnstile test keys' hostnames: for `wrangler pages dev` only, never in production. */
  TURNSTILE_ALLOW_TEST_HOSTNAMES?: string;
  /** For a local smoke test against a mock only; never set in production. */
  ANTHROPIC_BASE_URL?: string;
}

export type ErrorCode = "session" | "rate" | "quota" | "budget" | "invalid" | "upstream";

export function errorResponse(status: number, error: ErrorCode, message: string, retryAfter?: number): Response {
  const headers: Record<string, string> = { "cache-control": "no-store" };
  if (retryAfter !== undefined) headers["retry-after"] = String(retryAfter);
  return Response.json({ error, message, ...(retryAfter !== undefined ? { retry_after: retryAfter } : {}) }, { status, headers });
}

export function refusalResponse(refusal: Refusal): Response {
  return errorResponse(refusal.error === "budget" ? 503 : 429, refusal.error, refusal.message, refusal.retry_after);
}

export const unavailable = () => errorResponse(503, "budget", "The free assistant is paused.");

export interface Setup {
  budget: DurableObjectStub<AgentBudget>;
  apiKey: string;
  sessionSecret: string;
  turnstileSecret: string;
  model: string;
  price: Price;
  effort: Effort;
  baseURL?: string;
}

/**
 * The configuration, or null when the assistant must not run: switched
 * off, a secret or the budget binding missing, or a model without a price
 * (whose spend could not be accounted for).
 */
export function setupOf(env: AgentEnv): Setup | null {
  if (env.AGENT_DISABLED === "1") return null;
  const model = env.AGENT_MODEL || DEFAULT_MODEL;
  const price = PRICES[model];
  const effort = (env.AGENT_EFFORT || DEFAULT_EFFORT) as Effort;
  if (!price || !EFFORTS.includes(effort)) {
    console.log(JSON.stringify({ event: "agent_misconfigured", model, effort }));
    return null;
  }
  if (!env.AGENT_BUDGET || !env.ANTHROPIC_API_KEY || !env.SESSION_SECRET || !env.TURNSTILE_SECRET) {
    console.log(JSON.stringify({ event: "agent_misconfigured", missing: "binding or secret" }));
    return null;
  }
  return {
    budget: env.AGENT_BUDGET.getByName("global"),
    apiKey: env.ANTHROPIC_API_KEY,
    sessionSecret: env.SESSION_SECRET,
    turnstileSecret: env.TURNSTILE_SECRET,
    model,
    price,
    effort,
    baseURL: env.ANTHROPIC_BASE_URL || undefined,
  };
}

export function clientIp(request: Request): string {
  return request.headers.get("CF-Connecting-IP") ?? "unknown";
}

/** Refuses a cross-site POST: the cookie is SameSite=Strict already, this also holds for a browser that ignores it. */
export function sameOrigin(request: Request): boolean {
  const origin = request.headers.get("origin");
  return origin === null || origin === new URL(request.url).origin;
}
