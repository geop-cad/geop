import type { ReactNode } from "react";
import type { Program } from "./geop";
import { BugReport } from "./BugReport";
import { Privacy } from "./Privacy";

interface Props {
  busy: boolean;
  /** Whether the program is a file somebody else owns (VS Code): saving, loading and reporting are theirs. */
  hosted: boolean;
  hasSteps: boolean;
  onSave: () => void;
  onLoadFile: (file: File) => void;
  exampleNames: string[];
  onLoadExample: (name: string) => void;
  canUndo: boolean;
  onUndo: () => void;
  canRedo: boolean;
  onRedo: () => void;
  /** The operation-buttons row, rendered once in App and placed here (desktop) and in MobileBottom. */
  operationButtons: ReactNode;
  /** What the editor is in the middle of, when it is working in a plane. */
  badge: string | null;
  /** Why the last command was refused, if it was. */
  error: string | null;
  program: Program;
  stepCount: number;
  triangleCount: number;
  /** Where the bug-report form goes instead of floating over the viewport — the mobile "Bug" tab, when on mobile. */
  bugReportHost: HTMLElement | null;
  onBugReportOpen: () => void;
  onBugReportClose: () => void;
}

/** The app's top bar: file actions, undo/redo, the operation buttons (desktop only), and status. */
export function Toolbar({
  busy,
  hosted,
  hasSteps,
  onSave,
  onLoadFile,
  exampleNames,
  onLoadExample,
  canUndo,
  onUndo,
  canRedo,
  onRedo,
  operationButtons,
  badge,
  error,
  program,
  stepCount,
  triangleCount,
  bugReportHost,
  onBugReportOpen,
  onBugReportClose,
}: Props) {
  return (
    <header className="toolbar">
      <h1>Geop</h1>
      <div className="tools">
        {!hosted && (
          <>
            <button disabled={busy || !hasSteps} onClick={onSave}>
              Save
            </button>
            <label className={`file-button${busy ? " disabled" : ""}`}>
              Load
              <input
                type="file"
                accept=".geop,.json,application/json"
                disabled={busy}
                onChange={(e) => {
                  const file = e.target.files?.[0];
                  e.target.value = "";
                  if (file) onLoadFile(file);
                }}
              />
            </label>
          </>
        )}
        <select
          value=""
          disabled={busy}
          onChange={(e) => {
            if (e.target.value) onLoadExample(e.target.value);
          }}
        >
          <option value="">Examples…</option>
          {exampleNames.map((name) => (
            <option key={name} value={name}>
              {name}
            </option>
          ))}
        </select>
        <button disabled={busy || !canUndo} onClick={onUndo}>
          Undo
        </button>
        <button disabled={busy || !canRedo} onClick={onRedo}>
          Redo
        </button>
      </div>
      <div className="tools desktop-only">{operationButtons}</div>
      {badge && <span className="mode-badge">{badge}</span>}
      {!hosted && (
        <>
          <BugReport
            program={program}
            committedError={error}
            panelHost={bugReportHost}
            onOpen={onBugReportOpen}
            onClose={onBugReportClose}
          />
          <Privacy />
        </>
      )}
      {!busy && (
        <span className="stats">
          {stepCount} step{stepCount === 1 ? "" : "s"} · {triangleCount} tris
        </span>
      )}
    </header>
  );
}
