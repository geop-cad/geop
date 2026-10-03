import { useState } from "react";
import { analyticsStatus, setAnalyticsOptOut } from "./analytics";

export const PRIVACY_URL = "https://geop-cad.dev/privacy.html";

/** What the app collects, while `open`, and a switch to turn usage statistics off. */
export function Privacy({ open, onClose }: { open: boolean; onClose: () => void }) {
  const [status, setStatus] = useState(analyticsStatus);

  if (!open) return null;

  const toggle = (on: boolean) => {
    setAnalyticsOptOut(!on);
    setStatus(analyticsStatus());
  };

  return (
    <div className="popup-backdrop">
      <div className="popup">
        <div className="button-row">
          <button className="small" onClick={onClose}>
            Close
          </button>
        </div>
        <h2>Privacy</h2>
        <p className="hint">
          Your parts stay in your browser. They are only sent anywhere if you send a bug report, which attaches the
          current file.
        </p>
        <p className="hint">
          To learn which features are used and which fail, the app counts anonymous usage statistics: which kind of
          operation you add or edit, and the kernel's error message when a step fails — never your parts, sketches or
          names. No cookies, nothing stored on your device, no recordings.
        </p>
        {status === "unconfigured" ? null : status === "browser" ? (
          <p className="hint">Statistics are off: your browser asks sites not to track it.</p>
        ) : (
          <label className="field">
            <input type="checkbox" checked={status === null} onChange={(e) => toggle(e.target.checked)} />
            Share anonymous usage statistics
          </label>
        )}
        <p className="hint">
          <a href={PRIVACY_URL} target="_blank" rel="noopener">
            Full privacy notice
          </a>
        </p>
      </div>
    </div>
  );
}
