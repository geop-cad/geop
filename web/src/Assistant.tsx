// The hosted assistant: a chat docked beside the viewport, whose model
// edits the part through the kernel's agent tools (see `agent.ts` for the
// loop, `geop_core_editor::agent` for the tools). Hidden in VS Code, where
// the user has their own agent.
import { useEffect, useRef, useState } from "react";
import { Icon } from "./icons";
import type { AssistantState } from "./useAssistant";

/** How to connect an agent of one's own, the geop CLI's MCP server. */
const MCP_DOCS = "https://github.com/geop-cad/geop/blob/main/docs/mcp.md";

/** How to use your own agent instead. */
function OwnAgent() {
  return (
    <p className="hint">
      Connect your own Claude Code instead: install the geop CLI and run <code>claude mcp add geop -- geop mcp</code>.{" "}
      <a href={MCP_DOCS} target="_blank" rel="noopener">
        How
      </a>
    </p>
  );
}

/** Text with `code` spans. */
function Said({ text }: { text: string }) {
  const parts = text.split(/(`[^`\n]+`)/);
  return (
    <>
      {parts.map((part, i) =>
        part.startsWith("`") && part.endsWith("`") && part.length > 1 ? <code key={i}>{part.slice(1, -1)}</code> : part,
      )}
    </>
  );
}

/** The chat: what was said and done, a prompt to type, and how much is left today. */
export function AssistantPanel({ assistant, onClose }: { assistant: AssistantState; onClose?: () => void }) {
  const [prompt, setPrompt] = useState("");
  const log = useRef<HTMLDivElement>(null);
  const { configured, entries, limits, verifying, running, idle } = assistant;
  const spent = entries.some((e) => e.kind === "used_up") || limits?.turns_left_today === 0;

  useEffect(() => {
    if (configured) assistant.connect();
    // Once, when the panel is first shown: the check runs before the first message.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [configured]);
  useEffect(() => {
    log.current?.scrollTo({ top: log.current.scrollHeight });
  }, [entries.length, running]);

  const send = () => {
    const text = prompt.trim();
    if (!text || running || !idle || spent) return;
    setPrompt("");
    assistant.ask(text);
  };

  return (
    <section className="assistant" aria-label="Assistant">
      <header className="assistant-head">
        <h2>Assistant</h2>
        {limits && (
          <span className="assistant-limits" title="Each reply of the model is a turn; a prompt may take several.">
            {limits.turns_left_today} of {limits.turns_per_day} turns left today
          </span>
        )}
        {entries.length > 0 && !running && (
          <button className="small" onClick={assistant.clear} title="Start a new conversation">
            New
          </button>
        )}
        {onClose && (
          <button className="tool-icon" onClick={onClose} title="Close the assistant" aria-label="Close the assistant">
            ×
          </button>
        )}
      </header>
      <div className="assistant-log" ref={log}>
        {!configured ? (
          <>
            <p className="hint">The hosted assistant is not configured in this build.</p>
            <OwnAgent />
          </>
        ) : (
          entries.length === 0 && (
            <p className="hint">
              Describe a part or a change — “a 40 × 20 × 5 plate with four 3 mm holes”, “round the top edges” — and the
              assistant edits the program for you. Your messages and the program are sent to geop's server and to
              Anthropic to answer them.
            </p>
          )
        )}
        {entries.map((entry, i) => {
          switch (entry.kind) {
            case "user":
              return (
                <div key={i} className="chat-user">
                  {entry.text}
                </div>
              );
            case "assistant":
              return (
                <div key={i} className="chat-assistant">
                  <Said text={entry.text} />
                </div>
              );
            case "tool":
              return (
                <div key={i} className={`chat-tool${entry.error ? " error-card" : ""}`}>
                  <Said text={entry.text} />
                </div>
              );
            case "notice":
              return (
                <p key={i} className={entry.error ? "chat-notice op-error-text" : "chat-notice"}>
                  {entry.text}
                </p>
              );
            case "used_up":
              return (
                <div key={i} className="chat-used-up">
                  <p>{entry.text}</p>
                  <OwnAgent />
                </div>
              );
          }
        })}
        {limits?.turns_left_today === 0 && !entries.some((e) => e.kind === "used_up") && (
          <div className="chat-used-up">
            <p>The free assistant is used up for today.</p>
            <OwnAgent />
          </div>
        )}
        {running && (
          <div className="chat-working" role="status">
            <span className="spinner" /> Working…
          </div>
        )}
        {verifying && !running && <p className="chat-notice">Checking that you are human…</p>}
        {assistant.canUndoAnswer && !running && (
          <button
            className="small chat-undo"
            onClick={assistant.undoAnswer}
            title="Go back to the part as it was before this answer"
          >
            <Icon name="undo" /> Undo this answer
          </button>
        )}
      </div>
      {configured && (
        <form
          className="assistant-input"
          onSubmit={(e) => {
            e.preventDefault();
            send();
          }}
        >
          <textarea
            value={prompt}
            rows={3}
            placeholder={
              spent
                ? "The free assistant is used up for today."
                : idle
                  ? "Ask for a part or a change…"
                  : "Finish the step being edited first."
            }
            disabled={spent}
            onChange={(e) => setPrompt(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter" && !e.shiftKey) {
                e.preventDefault();
                send();
              }
            }}
          />
          {running ? (
            <button type="button" onClick={assistant.stop}>
              Stop
            </button>
          ) : (
            <button type="submit" className="primary" disabled={!prompt.trim() || !idle || spent}>
              Send
            </button>
          )}
        </form>
      )}
    </section>
  );
}
