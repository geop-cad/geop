/**
 * POST /api/agent/turn {"messages": [...]}: one model turn of the browser's
 * CAD agent. The browser runs the loop and the tools; this endpoint adds
 * our system prompt, tools and model, and accounts for every cent: admit
 * (rates, quotas), count the prompt's tokens, reserve the turn's worst
 * case, call the model, settle to its usage. See docs/agent-chat.md.
 */
import Anthropic from "@anthropic-ai/sdk";
import { MAX_BODY_BYTES, MAX_IMAGE_BASE64_BYTES, MAX_INPUT_TOKENS, MAX_TOKENS, UPSTREAM_TIMEOUT_MS } from "../../../lib/agent/config.ts";
import { costOf, reservationFor } from "../../../lib/agent/cost.ts";
import { clientIp, errorResponse, refusalResponse, sameOrigin, setupOf, unavailable, type AgentEnv } from "../../../lib/agent/http.ts";
import { SYSTEM_PROMPT } from "../../../lib/agent/prompt.ts";
import { cookieValue, verifySession } from "../../../lib/agent/session.ts";
import { Invalid, validateTurn } from "../../../lib/agent/validate.ts";
import toolDefinitions from "./tools.json";

const TOOLS = toolDefinitions as Anthropic.Tool[];
const TOOL_NAMES: ReadonlySet<string> = new Set(TOOLS.map((tool) => tool.name));
/** The stable prefix, tools then system, cached at the end of the system prompt. */
const SYSTEM: Anthropic.TextBlockParam[] = [{ type: "text", text: SYSTEM_PROMPT, cache_control: { type: "ephemeral" } }];

/** Counts and costs only: never a message's content. */
function log(fields: Record<string, unknown>) {
  console.log(JSON.stringify({ event: "agent_turn", ...fields }));
}

export const onRequestPost: PagesFunction<AgentEnv> = async ({ request, env }) => {
  const setup = setupOf(env);
  if (!setup) return unavailable();
  if (!sameOrigin(request)) return errorResponse(400, "invalid", "Cross-site request.");

  const sid = await verifySession(setup.sessionSecret, cookieValue(request.headers.get("cookie")) ?? "", Date.now());
  if (!sid) return errorResponse(401, "session", "Please verify that you are human to start the assistant.");
  const ip = clientIp(request);

  const limit = MAX_BODY_BYTES + MAX_IMAGE_BASE64_BYTES;
  if (Number(request.headers.get("content-length") ?? "0") > limit) return errorResponse(400, "invalid", "The conversation is too large.");
  const raw = new Uint8Array(await request.arrayBuffer());
  if (raw.length > limit) return errorResponse(400, "invalid", "The conversation is too large.");
  let messages: Anthropic.MessageParam[];
  try {
    messages = validateTurn(JSON.parse(new TextDecoder().decode(raw)), raw.length, TOOL_NAMES);
  } catch (e) {
    if (e instanceof SyntaxError) return errorResponse(400, "invalid", "The body must be JSON.");
    if (e instanceof Invalid) return errorResponse(400, "invalid", e.message);
    throw e;
  }

  const admitted = await setup.budget.admit({ sid, ip, now: Date.now() });
  if (!admitted.ok) return refusalResponse(admitted);

  // No retries: a retried request may be billed twice against one reservation.
  const client = new Anthropic({ apiKey: setup.apiKey, baseURL: setup.baseURL, maxRetries: 0, timeout: UPSTREAM_TIMEOUT_MS });
  const thinking: Anthropic.ThinkingConfigParam = { type: "adaptive", display: "summarized" };

  let counted: number;
  try {
    counted = (await client.messages.countTokens({ model: setup.model, system: SYSTEM, tools: TOOLS, thinking, messages })).input_tokens;
  } catch (e) {
    log({ stage: "count", status: e instanceof Anthropic.APIError ? e.status : "network" });
    return errorResponse(502, "upstream", "The assistant could not be reached. Try again.");
  }
  if (counted > MAX_INPUT_TOKENS) {
    return errorResponse(400, "invalid", "The conversation is too long for the assistant. Start a new chat.");
  }

  const id = crypto.randomUUID();
  const amount = reservationFor(setup.price, counted, MAX_TOKENS);
  const reserved = await setup.budget.reserve({ sid, ip, now: Date.now(), id, amount });
  if (!reserved.ok) return refusalResponse(reserved);

  let response: Anthropic.Message;
  try {
    response = await client.messages.create({
      model: setup.model,
      max_tokens: MAX_TOKENS,
      system: SYSTEM,
      tools: TOOLS,
      thinking,
      output_config: { effort: setup.effort },
      // The growing conversation, cached at its last block.
      cache_control: { type: "ephemeral" },
      messages,
    });
  } catch (e) {
    // The API answering with a 4xx ran no model, and bills nothing. Anything
    // else (5xx, a dropped connection, a timeout) may have been billed:
    // the whole reservation stays charged.
    const status = e instanceof Anthropic.APIError ? e.status : undefined;
    const billed = !(typeof status === "number" && status >= 400 && status < 500);
    await setup.budget.settle({ id, cost: billed ? amount : 0, now: Date.now() });
    log({ stage: "create", status: status ?? "network", reserved_nano: amount, charged_nano: billed ? amount : 0 });
    return errorResponse(502, "upstream", "The assistant did not answer. Try again.");
  }

  const cost = costOf(setup.price, response.usage);
  await setup.budget.settle({ id, cost, now: Date.now() });
  log({
    model: setup.model,
    messages: messages.length,
    counted_input: counted,
    input: response.usage.input_tokens,
    cache_write: response.usage.cache_creation_input_tokens,
    cache_read: response.usage.cache_read_input_tokens,
    output: response.usage.output_tokens,
    stop_reason: response.stop_reason,
    reserved_nano: amount,
    cost_nano: cost,
    // The reservation is meant as an upper bound; say so loudly if it was not one.
    over_reservation: cost > amount,
  });
  return Response.json(
    { message: { content: response.content, stop_reason: response.stop_reason, usage: response.usage }, limits: reserved.limits },
    { headers: { "cache-control": "no-store" } },
  );
};
