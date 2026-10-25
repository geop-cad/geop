/**
 * POST /api/agent/session {"turnstile": "<token>"}: verifies the Turnstile
 * token, then opens a session (at most IP_SESSIONS_PER_DAY per address)
 * and sets its signed cookie. See docs/agent-chat.md.
 */
import { clientIp, errorResponse, refusalResponse, sameOrigin, setupOf, unavailable, type AgentEnv } from "../../../lib/agent/http.ts";
import { sessionCookie, signSession } from "../../../lib/agent/session.ts";
import { verifyTurnstile } from "../../../lib/agent/turnstile.ts";

export const onRequestPost: PagesFunction<AgentEnv> = async ({ request, env }) => {
  const setup = setupOf(env);
  if (!setup) return unavailable();
  if (!sameOrigin(request)) return errorResponse(400, "invalid", "Cross-site request.");
  if (Number(request.headers.get("content-length") ?? "0") > 4096) return errorResponse(400, "invalid", "Body too large.");
  let token: unknown;
  try {
    const body = (await request.json()) as Record<string, unknown>;
    token = body?.turnstile;
  } catch {
    return errorResponse(400, "invalid", "The body must be JSON.");
  }
  if (typeof token !== "string" || token.length === 0 || token.length > 2048) {
    return errorResponse(400, "invalid", "`turnstile` must be the Turnstile token.");
  }
  const ip = clientIp(request);
  const turnstile = await verifyTurnstile(setup.turnstileSecret, token, ip, env.TURNSTILE_ALLOW_TEST_HOSTNAMES === "1");
  if (!turnstile.passed) {
    console.log(JSON.stringify({ event: "agent_turnstile_failed", reason: turnstile.reason }));
    return errorResponse(401, "session", "The verification did not pass. Please try again.");
  }
  const sid = crypto.randomUUID();
  const now = Date.now();
  const opened = await setup.budget.openSession({ sid, ip, now });
  if (!opened.ok) return refusalResponse(opened);
  console.log(JSON.stringify({ event: "agent_session" }));
  return Response.json(
    { limits: opened.limits },
    { headers: { "set-cookie": sessionCookie(await signSession(setup.sessionSecret, sid, now)), "cache-control": "no-store" } },
  );
};
