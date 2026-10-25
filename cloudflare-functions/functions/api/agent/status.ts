/** GET /api/agent/status: {"available": bool}, whether the assistant takes turns now. Nothing else is said. */
import { MAX_TOKENS } from "../../../lib/agent/config.ts";
import { reservationFor } from "../../../lib/agent/cost.ts";
import { setupOf, type AgentEnv } from "../../../lib/agent/http.ts";

/** The answer is cached this long, so polling it costs the budget object nothing. */
const CACHE_SECONDS = 30;

export const onRequestGet: PagesFunction<AgentEnv> = async ({ request, env }) => {
  const cache = caches.default;
  const key = new Request(new URL(request.url).toString(), { method: "GET" });
  const cached = await cache.match(key);
  if (cached) return cached;
  const setup = setupOf(env);
  let available = false;
  if (setup) {
    try {
      available = await setup.budget.available({ now: Date.now(), minimum: reservationFor(setup.price, 0, MAX_TOKENS) });
    } catch {
      available = false;
    }
  }
  const response = Response.json({ available }, { headers: { "cache-control": `public, max-age=${CACHE_SECONDS}` } });
  await cache.put(key, response.clone());
  return response;
};
