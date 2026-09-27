import type { ReactNode } from "react";
import { entityLabel, type EntityRef, type Program, type RunResult } from "./geop";
import { BugReport } from "./BugReport";

interface Props {
  busy: boolean;
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
  /** The plane being sketched on, if sketching. */
  sketchingOn: EntityRef | null;
  committed: RunResult | null;
  committedError: string | null;
  program: Program;
  stepCount: number;
  triangleCount: number;
}

/** The app's top bar: file actions, undo/redo, the operation buttons (desktop only), and status. */
export function Toolbar({
  busy,
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
  sketchingOn,
  committed,
  committedError,
  program,
  stepCount,
  triangleCount,
}: Props) {
  return (
    <header className="toolbar">
      <h1>Geop</h1>
      <div className="tools">
        <button disabled={busy || !hasSteps} onClick={onSave}>
          Save
        </button>
        <label className={`file-button${busy ? " disabled" : ""}`}>
          Load
          <input
            type="file"
            accept=".json,application/json"
            disabled={busy}
            onChange={(e) => {
              const file = e.target.files?.[0];
              e.target.value = "";
              if (file) onLoadFile(file);
            }}
          />
        </label>
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
      {sketchingOn && <span className="mode-badge">Sketching on {entityLabel(sketchingOn)}</span>}
      <BugReport program={program} committedError={committedError} />
      {committed && (
        <span className="stats">
          {stepCount} step{stepCount === 1 ? "" : "s"} · {triangleCount} tris
        </span>
      )}
    </header>
  );
}
