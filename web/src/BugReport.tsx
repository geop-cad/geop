import { useState } from "react";
import { createPortal } from "react-dom";
import type { Program } from "./geop";

interface Props {
  program: Program;
  committedError: string | null;
  /** Where to put the form instead of floating it over the viewport — the mobile layout's "Bug" tab pane. */
  panelHost?: HTMLElement | null;
  onOpen?: () => void;
  onClose?: () => void;
}

type Status = "idle" | "sending" | "sent" | { error: string };

/** A button that opens a small form and posts it, with the current program attached, to /api/bug-report. */
export function BugReport({ program, committedError, panelHost, onOpen, onClose }: Props) {
  const [open, setOpen] = useState(false);
  const [description, setDescription] = useState("");
  const [status, setStatus] = useState<Status>("idle");

  function openDialog() {
    setOpen(true);
    onOpen?.();
  }

  function close() {
    setOpen(false);
    setDescription("");
    setStatus("idle");
    onClose?.();
  }

  async function send() {
    setStatus("sending");
    try {
      const res = await fetch("/api/bug-report", {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({
          description,
          program,
          meta: {
            url: window.location.href,
            userAgent: navigator.userAgent,
            timestamp: new Date().toISOString(),
            committedError,
          },
        }),
      });
      if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
      setStatus("sent");
    } catch (e) {
      setStatus({ error: String(e) });
    }
  }

  if (!open) {
    return <button onClick={openDialog}>Report bug</button>;
  }

  const content = (
    <div className="popup-backdrop">
      <div className="popup">
        <div className="button-row">
          <button className="small" onClick={close}>
            {status === "sent" ? "Close" : "Cancel"}
          </button>
          {status !== "sent" && (
            <button className="primary" disabled={status === "sending"} onClick={send}>
              {status === "sending" ? "Sending…" : "Send"}
            </button>
          )}
        </div>

        <h2>Report a bug</h2>
        {status === "sent" ? (
          <p className="hint">
            Sent — thank you. The current file and some basic diagnostics (browser, URL, the last error if any) went with it.
          </p>
        ) : (
          <>
            <p className="hint">What went wrong? The current file and some basic diagnostics (browser, URL, the last error if any) are attached automatically.</p>
            <textarea
              className="bug-report-text"
              rows={5}
              autoFocus
              value={description}
              onChange={(e) => setDescription(e.target.value)}
              placeholder="What were you doing, and what did you expect instead?"
            />
            {typeof status === "object" && <p className="op-error-text">Failed to send: {status.error}</p>}
          </>
        )}
      </div>
    </div>
  );

  return panelHost ? createPortal(content, panelHost) : content;
}
