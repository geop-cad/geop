// The hosted assistant's loop, without the page: what is sent to the
// server (`/api/agent/*`, see `cloudflare-functions/`), and how the tools
// the model asks for are run on the kernel and answered. The server owns
// the model, its instructions and its tools; the page sends the
// conversation, as the Anthropic Messages API has it, and runs the tools
// (`geop_core_editor::agent`, through `geop.ts`'s `runTool`).
//
// Kept free of the DOM and the kernel, so that `test/agent.test.ts` runs it
// with a fetch and a kernel of its own.

/** A block of a message, as the Messages API has it. Blocks the page does not know — thinking — are passed back as they came. */
export type ContentBlock =
  | { type: "text"; text: string }
  | { type: "image"; source: { type: "base64"; media_type: "image/png"; data: string } }
  | { type: "tool_use"; id: string; name: string; input: unknown }
  | { type: "tool_result"; tool_use_id: string; content: string; is_error?: boolean }
  | { type: "thinking"; thinking: string; signature: string }
  | { type: "redacted_thinking"; data: string };

export interface MessageParam {
  role: "user" | "assistant";
  content: ContentBlock[];
}

/** How much of the assistant is left to the user today. */
export interface Limits {
  turns_left_today: number;
  turns_per_day: number;
}

/** What the server answers a turn with. */
export interface TurnAnswer {
  message: { content: ContentBlock[]; stop_reason: string | null; usage?: unknown };
  limits?: Limits;
}

/**
 * Why the server refused: `session` (the session ended: prove again to be
 * human), `rate` (too fast), `quota` (the user's allowance for today is
 * used up), `budget` (the assistant's is, or it is off), `invalid`,
 * `upstream` (the model failed), `network` (no answer at all).
 */
export class AgentError extends Error {
  readonly code: string;
  readonly status: number;
  readonly retryAfter: number | null;
  constructor(code: string, message: string, status: number, retryAfter: number | null = null) {
    super(message);
    this.name = "AgentError";
    this.code = code;
    this.status = status;
    this.retryAfter = retryAfter;
  }
}

/** Whether `e` means the assistant cannot be used again today: the user's allowance, or everybody's, is spent. */
export function usedUp(e: unknown): boolean {
  return e instanceof AgentError && (e.code === "quota" || e.code === "budget");
}

type Fetch = (input: string, init?: RequestInit) => Promise<Response>;

/** POSTs `body` to `path` as JSON: the answer's JSON, or the server's refusal as an [[AgentError]]. */
async function post<T>(fetchFn: Fetch, path: string, body: unknown, signal?: AbortSignal): Promise<T> {
  let response: Response;
  try {
    response = await fetchFn(path, {
      method: "POST",
      headers: { "content-type": "application/json" },
      credentials: "same-origin",
      body: JSON.stringify(body),
      signal,
    });
  } catch (e) {
    if (signal?.aborted) throw e;
    throw new AgentError("network", `The assistant could not be reached: ${String(e)}`, 0);
  }
  const text = await response.text();
  let json: unknown = null;
  try {
    json = JSON.parse(text);
  } catch {
    // Not JSON: a proxy's error page, say.
  }
  if (!response.ok) {
    const refusal = (json ?? {}) as { error?: string; message?: string; retry_after?: number };
    throw new AgentError(
      refusal.error ?? (response.status === 401 ? "session" : "upstream"),
      refusal.message ?? `The assistant answered ${response.status}.`,
      response.status,
      refusal.retry_after ?? null,
    );
  }
  if (json == null)
    throw new AgentError("upstream", "The assistant answered something that is not JSON.", response.status);
  return json as T;
}

/** Start a session with the Turnstile `token`: the cookie the server sets, and the limits it tells. */
export async function startSession(token: string, fetchFn: Fetch = fetch): Promise<Limits | null> {
  const answer = await post<{ limits?: Limits }>(fetchFn, "/api/agent/session", { turnstile: token });
  return answer.limits ?? null;
}

/** One turn of the model on `messages`. */
export function postTurn(messages: MessageParam[], signal?: AbortSignal, fetchFn: Fetch = fetch): Promise<TurnAnswer> {
  return post<TurnAnswer>(fetchFn, "/api/agent/turn", { messages }, signal);
}

/** What a tool answered. */
export interface ToolResult {
  text: string;
  isError: boolean;
}

/** What the loop needs of the page. */
export interface Loop {
  /** One turn of the model; refused by throwing an [[AgentError]]. */
  turn: (messages: MessageParam[], signal: AbortSignal) => Promise<TurnAnswer>;
  /** Runs the tool `name` on the kernel. */
  runTool: (name: string, input: unknown) => Promise<ToolResult>;
  /** A message added to the conversation: the model's, or the tools' results. */
  onMessage?: (message: MessageParam) => void;
  /** A tool ran. */
  onTool?: (use: { id: string; name: string; input: unknown }, result: ToolResult) => void;
  onLimits?: (limits: Limits) => void;
}

/** Why an answer ended: the model's stop reason, `turns` (the page's cap on turns), or `stopped` (the user). */
export type Ended = string;

/** The most turns of the model one prompt may take: a loop that never ends costs the user's allowance. */
export const MAX_TURNS = 25;

/** What a stopped answer tells the model of the tools it asked for and did not get. */
const NOT_RUN = "Not run: the user stopped this answer.";

/**
 * Answer the conversation `messages`, which ends with the user's prompt:
 * turn after turn of the model, running every tool it asks for and
 * answering all of them in one message, until it stops on its own, the
 * user stops it (`signal`), or it took `maxTurns`. The conversation as it
 * then is — always one the next prompt can follow, every tool use
 * answered — and why it ended. A refusal of the server is thrown, the
 * conversation up to it told through `onMessage`.
 */
export async function answer(
  messages: MessageParam[],
  loop: Loop,
  signal: AbortSignal,
  maxTurns = MAX_TURNS,
): Promise<{ messages: MessageParam[]; ended: Ended }> {
  const history = [...messages];
  const add = (message: MessageParam) => {
    history.push(message);
    loop.onMessage?.(message);
  };
  for (let turn = 0; turn < maxTurns; turn++) {
    if (signal.aborted) return { messages: history, ended: "stopped" };
    let answered: TurnAnswer;
    try {
      answered = await loop.turn(history, signal);
    } catch (e) {
      if (signal.aborted) return { messages: history, ended: "stopped" };
      throw e;
    }
    if (answered.limits) loop.onLimits?.(answered.limits);
    const { content, stop_reason } = answered.message;
    const uses = content.filter((b): b is Extract<ContentBlock, { type: "tool_use" }> => b.type === "tool_use");
    if (stop_reason !== "tool_use" || uses.length === 0) {
      // A tool use cut off (`max_tokens`) is never answered: left out, so
      // that the conversation can go on.
      const kept = content.filter((b) => b.type !== "tool_use");
      add({ role: "assistant", content: kept.length > 0 ? kept : [{ type: "text", text: "…" }] });
      if (stop_reason === "pause_turn") continue;
      return { messages: history, ended: stop_reason ?? "end_turn" };
    }
    add({ role: "assistant", content });
    const results: ContentBlock[] = [];
    for (const use of uses) {
      let result: ToolResult;
      if (signal.aborted) result = { text: NOT_RUN, isError: true };
      else {
        try {
          result = await loop.runTool(use.name, use.input);
        } catch (e) {
          result = { text: `${use.name} failed: ${e instanceof Error ? e.message : String(e)}`, isError: true };
        }
        loop.onTool?.(use, result);
      }
      results.push({
        type: "tool_result",
        tool_use_id: use.id,
        content: result.text,
        ...(result.isError ? { is_error: true } : {}),
      });
    }
    add({ role: "user", content: results });
  }
  return { messages: history, ended: signal.aborted ? "stopped" : "turns" };
}

/**
 * The conversation `messages` with the user's prompt `text` added — and,
 * before it, `context` (what the part is now, as the model has not seen
 * it). After tool results the prompt joins their message: the
 * conversation alternates between the user and the model.
 */
export function withPrompt(messages: MessageParam[], text: string, context: ContentBlock[] = []): MessageParam[] {
  const blocks: ContentBlock[] = [...context, { type: "text", text }];
  const last = messages[messages.length - 1];
  if (last?.role === "user") return [...messages.slice(0, -1), { role: "user", content: [...last.content, ...blocks] }];
  return [...messages, { role: "user", content: blocks }];
}

/** The longest program sent along as it is; a longer one is only described, and the model asks the tools. */
const LONGEST_PROGRAM = 20000;

/** The program `program` (a `.geop` file's JSON) as context for the model. */
export function programContext(program: unknown): ContentBlock {
  const text = JSON.stringify(program);
  const steps = (program as { steps?: unknown[] } | null)?.steps?.length ?? 0;
  if (steps === 0) return { type: "text", text: "The part open in the editor is empty: it has no steps yet." };
  if (text.length > LONGEST_PROGRAM) {
    return {
      type: "text",
      text: `The part open in the editor has ${steps} steps, too long to include here: use the tools (run, list_elements) to see it.`,
    };
  }
  return {
    type: "text",
    text: `The part open in the editor now, as its .geop file holds it:\n<program>${text}</program>`,
  };
}

/** What a tool did, in a line: `Added step \`box\``. */
export function describeTool(name: string, input: unknown, result: ToolResult): string {
  const args = (input ?? {}) as Record<string, unknown>;
  const id = typeof args.id === "string" ? args.id : null;
  const quoted = (s: unknown) => `\`${String(s)}\``;
  let said: string;
  switch (name) {
    case "new_part":
      said = "Started a new part";
      break;
    case "list_operations":
      said = args.kind ? `Read how ${quoted(args.kind)} works` : "Listed the operations";
      break;
    case "add_step":
      said = id ? `Added step ${quoted(id)}` : `Added a ${quoted(args.kind)} step`;
      break;
    case "edit_step":
      said = `Changed step ${quoted(id)}`;
      break;
    case "delete_step":
      said = `Removed step ${quoted(id)}`;
      break;
    case "run":
      said = "Ran the program";
      break;
    case "list_elements":
      said = "Listed the part's elements";
      break;
    case "inspect":
      said = `Inspected: ${String(args.inspector ?? "the part")}`;
      break;
    default:
      said = `Used ${quoted(name)}`;
  }
  if (result.isError) return `${said}: ${firstLine(result.text)}`;
  const failed = failedSteps(result.text);
  return failed > 0 ? `${said} — ${failed} step${failed === 1 ? "" : "s"} failing` : said;
}

/** What went wrong, in a line: the kernel's root error if it names one, else the first line. */
function firstLine(text: string): string {
  const line = /RootError: (.*)/.exec(text)?.[1] ?? text.split("\n", 1)[0];
  return line.length > 160 ? `${line.slice(0, 160)}…` : line;
}

/** How many steps fail, by a tool's answer of the program's status. */
function failedSteps(text: string): number {
  try {
    const failed = (JSON.parse(text) as { failed?: unknown }).failed;
    return typeof failed === "number" ? failed : 0;
  } catch {
    return 0;
  }
}
