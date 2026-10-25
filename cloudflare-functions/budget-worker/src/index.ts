/**
 * The agent chat's budget worker. Its Durable Object `AgentBudget` (one
 * instance, named "global") runs lib/agent/ledger.ts on its storage; a
 * Durable Object handles one call at a time and these calls do not await
 * anything, so every check-and-update is atomic. The Pages project binds
 * the class through `script_name = "geop-agent-budget"` and calls it over
 * RPC (functions/api/agent/*).
 */
import { DurableObject } from "cloudflare:workers";
import { NANO_PER_USD } from "../../lib/agent/config.ts";
import * as ledger from "../../lib/agent/ledger.ts";
import type { Budgets, Caller, Granted, Refusal } from "../../lib/agent/ledger.ts";

export interface BudgetEnv {
  AGENT_BUDGET: DurableObjectNamespace<AgentBudget>;
  AGENT_DAILY_BUDGET_USD?: string;
  AGENT_TOTAL_BUDGET_USD?: string;
  /** If set, GET /ledger with `Authorization: Bearer <token>` answers the ledger. */
  LEDGER_TOKEN?: string;
}

/** A cap in nanodollars; 0, which refuses every turn, unless the var is a non-negative number. */
function cap(usd: string | undefined): number {
  const value = Number(usd);
  return usd !== undefined && usd.trim() !== "" && Number.isFinite(value) && value >= 0 ? Math.floor(value * NANO_PER_USD) : 0;
}

export class AgentBudget extends DurableObject<BudgetEnv> {
  private budgets(): Budgets {
    return { daily: cap(this.env.AGENT_DAILY_BUDGET_USD), total: cap(this.env.AGENT_TOTAL_BUDGET_USD) };
  }

  admit(caller: Caller): Granted | Refusal {
    return ledger.admit(this.ctx.storage.kv, this.budgets(), caller);
  }

  reserve(request: Caller & { id: string; amount: number }): Granted | Refusal {
    return ledger.reserve(this.ctx.storage.kv, this.budgets(), request);
  }

  settle(request: { id: string; cost: number; now: number }): void {
    ledger.settle(this.ctx.storage.kv, request);
  }

  openSession(caller: Caller): Granted | Refusal {
    return ledger.openSession(this.ctx.storage.kv, this.budgets(), caller);
  }

  available(request: { now: number; minimum: number }): boolean {
    return ledger.available(this.ctx.storage.kv, this.budgets(), request.now, request.minimum);
  }

  ledger(): ReturnType<typeof ledger.ledger> & { caps: Budgets } {
    return { ...ledger.ledger(this.ctx.storage.kv), caps: this.budgets() };
  }
}

/** Compares in constant time (of the digests, so the lengths need not match). */
async function sameSecret(given: string, expected: string): Promise<boolean> {
  const digest = (text: string) => crypto.subtle.digest("SHA-256", new TextEncoder().encode(text));
  return crypto.subtle.timingSafeEqual(await digest(given), await digest(expected));
}

export default {
  async fetch(request: Request, env: BudgetEnv): Promise<Response> {
    const url = new URL(request.url);
    const token = env.LEDGER_TOKEN;
    if (request.method !== "GET" || url.pathname !== "/ledger" || !token) return new Response("Not found", { status: 404 });
    if (!(await sameSecret(request.headers.get("authorization") ?? "", `Bearer ${token}`))) {
      return new Response("Forbidden", { status: 403 });
    }
    const stub = env.AGENT_BUDGET.getByName("global");
    return Response.json(await stub.ledger());
  },
} satisfies ExportedHandler<BudgetEnv>;
