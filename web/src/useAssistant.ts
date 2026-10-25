// The hosted assistant's state: the conversation, the session with the
// server (proven human by Cloudflare Turnstile), the limits it tells, and
// what an answer can be undone to. The loop itself is `agent.ts`'s; the
// panel showing it is `Assistant.tsx`.
import { useEffect, useRef, useState } from "react";
import {
  AgentError,
  answer,
  describeTool,
  postTurn,
  programContext,
  startSession,
  usedUp,
  withPrompt,
  type Limits,
  type MessageParam,
  type ToolResult,
} from "./agent";
import type { Program } from "./geop";

const SITE_KEY = import.meta.env.VITE_TURNSTILE_SITE_KEY as string | undefined;
const TURNSTILE_SCRIPT = "https://challenges.cloudflare.com/turnstile/v0/api.js?render=explicit";

/** What the Turnstile script offers (see Cloudflare's docs). */
interface Turnstile {
  render(container: HTMLElement, options: Record<string, unknown>): string;
  reset(widget: string): void;
}

let turnstileLoaded: Promise<Turnstile> | null = null;

/** The Turnstile script, loaded once. */
function loadTurnstile(): Promise<Turnstile> {
  turnstileLoaded ??= new Promise((resolve, reject) => {
    const script = document.createElement("script");
    script.src = TURNSTILE_SCRIPT;
    script.async = true;
    script.onload = () => {
      const turnstile = (window as unknown as { turnstile?: Turnstile }).turnstile;
      if (turnstile) resolve(turnstile);
      else reject(new Error("Turnstile did not load"));
    };
    script.onerror = () => {
      turnstileLoaded = null;
      reject(new Error("Turnstile could not be loaded: is challenges.cloudflare.com blocked?"));
    };
    document.head.appendChild(script);
  });
  return turnstileLoaded;
}

/** An entry of the chat, as shown. */
export type Entry =
  | { kind: "user"; text: string }
  | { kind: "assistant"; text: string }
  | { kind: "tool"; text: string; error: boolean }
  | { kind: "notice"; text: string; error: boolean }
  | { kind: "used_up"; text: string };

/** What the assistant needs of the app. */
export interface AssistantProps {
  /** The program as the kernel has it now; `null` until it is shown. */
  program: Program | null;
  /** Whether the kernel can take an edit: it is loaded and no step is being edited. */
  idle: boolean;
  runTool: (name: string, input: unknown) => Promise<ToolResult>;
  /** Make `program` the program again, as one edit (undone by Ctrl+Z like any other). */
  restore: (program: Program) => Promise<void>;
}

/** The assistant's state, kept by the app so it survives the panel closing (and the mobile tabs switching). */
export function useAssistant({ program, idle, runTool, restore }: AssistantProps) {
  const [entries, setEntries] = useState<Entry[]>([]);
  const [limits, setLimits] = useState<Limits | null>(null);
  const [verifying, setVerifying] = useState(false);
  const [running, setRunning] = useState<AbortController | null>(null);
  /** The conversation as the model has it. */
  const messages = useRef<MessageParam[]>([]);
  /** The program as the model last saw it, as text: sent again only once it changed. */
  const seen = useRef<string | null>(null);
  /** The program before the last answer, and after it as text: what "Undo this answer" goes back to, while the part is as the answer left it. */
  const [undoable, setUndoable] = useState<{ before: Program; after: string } | null>(null);
  const programRef = useRef(program);
  programRef.current = program;

  // Turnstile: one widget, rendered once, reset for every new token.
  const widget = useRef<{ turnstile: Turnstile; id: string; host: HTMLElement } | null>(null);
  const tokenWaiting = useRef<{ resolve: (token: string) => void; reject: (e: Error) => void } | null>(null);
  const session = useRef<Promise<void> | null>(null);

  /** A fresh Turnstile token: each is good for one session. */
  async function token(): Promise<string> {
    const turnstile = await loadTurnstile();
    const next = new Promise<string>((resolve, reject) => (tokenWaiting.current = { resolve, reject }));
    if (widget.current) widget.current.turnstile.reset(widget.current.id);
    else {
      const host = document.createElement("div");
      host.className = "turnstile-host";
      document.body.appendChild(host);
      const settle = (f: (w: NonNullable<typeof tokenWaiting.current>) => void) => {
        const waiting = tokenWaiting.current;
        tokenWaiting.current = null;
        if (waiting) f(waiting);
      };
      const id = turnstile.render(host, {
        sitekey: SITE_KEY,
        action: "agent_session",
        appearance: "interaction-only",
        callback: (t: string) => settle((w) => w.resolve(t)),
        "error-callback": () => settle((w) => w.reject(new Error("The check that you are human failed. Try again."))),
        "expired-callback": () => undefined,
      });
      widget.current = { turnstile, id, host };
    }
    return next;
  }

  /** A session with the server — started once, and again (`fresh`) once it ended. */
  function ensureSession(fresh = false): Promise<void> {
    if (fresh) session.current = null;
    session.current ??= (async () => {
      setVerifying(true);
      try {
        const told = await startSession(await token());
        if (told) setLimits(told);
      } catch (e) {
        session.current = null;
        throw e;
      } finally {
        setVerifying(false);
      }
    })();
    return session.current;
  }

  useEffect(
    () => () => {
      widget.current?.host.remove();
    },
    [],
  );

  const say = (entry: Entry) => setEntries((e) => [...e, entry]);

  /** Tell what refused the answer: the server, the check, the network. */
  function refused(e: unknown) {
    if (usedUp(e)) {
      say({
        kind: "used_up",
        text:
          (e as AgentError).code === "quota"
            ? "You have used up the free assistant for today."
            : "The free assistant is used up for today.",
      });
    } else if (e instanceof AgentError && e.code === "rate") {
      const wait = e.retryAfter != null ? ` Try again in ${Math.ceil(e.retryAfter)} s.` : "";
      say({ kind: "notice", text: `Too many requests.${wait}`, error: true });
    } else {
      say({ kind: "notice", text: e instanceof Error ? e.message : String(e), error: true });
    }
  }

  /** Ask `prompt`, and run the answer to its end. */
  async function ask(prompt: string) {
    if (running || !SITE_KEY) return;
    const now = programRef.current;
    const text = now ? JSON.stringify(now) : null;
    const context = now && text !== seen.current ? [programContext(now)] : [];
    const asked = withPrompt(messages.current, prompt, context);
    say({ kind: "user", text: prompt });
    const controller = new AbortController();
    setRunning(controller);
    setUndoable(null);
    const before = now;
    const heard: MessageParam[] = [];
    try {
      await ensureSession();
      const result = await answer(
        asked,
        {
          turn: async (history, signal) => {
            try {
              return await postTurn(history, signal);
            } catch (e) {
              // The session ended: prove again to be human, once.
              if (!(e instanceof AgentError) || e.code !== "session") throw e;
              await ensureSession(true);
              return postTurn(history, signal);
            }
          },
          runTool,
          onMessage: (message) => {
            // Once the model answered, the prompt is part of the conversation,
            // and so is whatever follows, even if a later turn is refused.
            heard.push(message);
            messages.current = [...asked, ...heard];
            if (message.role !== "assistant") return;
            const said = message.content
              .flatMap((b) => (b.type === "text" ? [b.text] : []))
              .join("\n\n")
              .trim();
            if (said && said !== "…") say({ kind: "assistant", text: said });
          },
          onTool: (use, result) =>
            say({ kind: "tool", text: describeTool(use.name, use.input, result), error: result.isError }),
          onLimits: setLimits,
        },
        controller.signal,
      );
      messages.current = result.messages;
      if (result.ended === "stopped") say({ kind: "notice", text: "Stopped.", error: false });
      else if (result.ended === "turns")
        say({
          kind: "notice",
          text: "The answer took too many turns and was stopped. Say “continue” to go on.",
          error: false,
        });
      else if (result.ended === "max_tokens")
        say({ kind: "notice", text: "The answer was cut off: it got too long.", error: false });
      else if (result.ended === "refusal")
        say({ kind: "notice", text: "The assistant declined to answer that.", error: false });
    } catch (e) {
      // The conversation goes on from the prompt's question, unanswered:
      // the model has not seen it.
      refused(e);
    } finally {
      setRunning(null);
      // What the model has seen of the program is as the answer left it.
      const after = programRef.current;
      if (heard.length > 0) seen.current = after ? JSON.stringify(after) : null;
      if (before && after && JSON.stringify(before) !== JSON.stringify(after)) {
        setUndoable({ before, after: JSON.stringify(after) });
      }
    }
  }

  /** Go back to the program before the last answer. */
  async function undoAnswer() {
    if (!undoable) return;
    await restore(undoable.before);
    setUndoable(null);
    say({ kind: "notice", text: "Undid that answer. Undo (Ctrl+Z) brings it back.", error: false });
  }

  /** Start over: a new conversation. */
  function clear() {
    running?.abort();
    messages.current = [];
    seen.current = null;
    setEntries([]);
    setUndoable(null);
  }

  const canUndoAnswer = undoable != null && program != null && JSON.stringify(program) === undoable.after;
  return {
    configured: SITE_KEY != null && SITE_KEY !== "",
    entries,
    limits,
    verifying,
    running: running != null,
    idle,
    ask: (prompt: string) => void ask(prompt),
    stop: () => running?.abort(),
    connect: () => void ensureSession().catch(refused),
    clear,
    canUndoAnswer,
    undoAnswer: () => void undoAnswer(),
  };
}

export type AssistantState = ReturnType<typeof useAssistant>;
