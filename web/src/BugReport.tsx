import { useState } from "react";
import { createPortal } from "react-dom";
import type { Program, Report } from "./geop";
import { bugReportPayload } from "./bugReportPayload";
import { PRIVACY_URL } from "./Privacy";

/** The commit the app was built from (see `vite.config.ts`), if the build knew it. */
const COMMIT = (import.meta.env.VITE_GEOP_COMMIT as string | undefined) || null;

interface Props {
  /** Whether the form is shown. */
  open: boolean;
  program: Program;
  /** The last error the app showed. */
  lastError: string | null;
  /** What the kernel says of the session — the step being edited among it — or `null` where it cannot say. */
  collect: () => Promise<Report | null>;
  /** Where to put the form instead of floating it over the viewport — the mobile layout's "Bug" tab pane. */
  panelHost?: HTMLElement | null;
  onClose: () => void;
}

type Status = "idle" | "sending" | "sent" | { error: string };

/**
 * A small form, while `open`, that posts a description to /api/bug-report,
 * with the program, the step being edited with its error, and the kernel's
 * version attached (see [[bugReportPayload]]). What is typed is kept until
 * it is sent: closing the form to look something up loses nothing.
 */
export function BugReport({ open, program, lastError, collect, panelHost, onClose }: Props) {
  const [description, setDescription] = useState("");
  const [status, setStatus] = useState<Status>("idle");

  function close() {
    if (status === "sent") setDescription("");
    setStatus("idle");
    onClose();
  }

  async function send() {
    setStatus("sending");
    try {
      let report: Report | null = null;
      try {
        report = await collect();
      } catch {
        // A kernel that cannot answer still gets the report sent, with
        // the program the app has.
      }
      const payload = bugReportPayload({
        description,
        program,
        report,
        lastError,
        commit: COMMIT,
        url: window.location.href,
        userAgent: navigator.userAgent,
        timestamp: new Date().toISOString(),
      });
      const res = await fetch("/api/bug-report", {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify(payload),
      });
      if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
      setStatus("sent");
    } catch (e) {
      setStatus({ error: String(e) });
    }
  }

  if (!open) return null;

  const attached =
    "the current file, the step you are editing with its error, the version of geop and some basic diagnostics (browser, URL, the last error if any)";
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
          <p className="hint">Sent — thank you. With it went {attached}.</p>
        ) : (
          <>
            <p className="hint">
              What went wrong? Attached automatically: {attached}. See the{" "}
              <a href={PRIVACY_URL} target="_blank" rel="noopener">
                privacy notice
              </a>
              .
            </p>
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
