import { useRef, useState, type ReactNode } from "react";
import type { Program } from "./geop";
import { BugReport } from "./BugReport";
import { Icon } from "./icons";
import { Menu, type MenuEntry } from "./Menu";
import { Privacy } from "./Privacy";

interface Props {
  busy: boolean;
  /** Whether the program files are somebody else's (VS Code): saving, loading and reporting are theirs. */
  hosted: boolean;
  hasSteps: boolean;
  onSave: () => void;
  /** Export a drawing of the part (see the drawing step) as SVG or DXF. */
  onExportDrawing: (format: "svg" | "dxf") => void;
  onLoadFile: (file: File) => void;
  exampleNames: string[];
  onLoadExample: (name: string) => void;
  /** Examples of several files: none where the files are not the app's own (VS Code). */
  workspaceExampleNames: string[];
  onLoadWorkspaceExample: (name: string) => void;
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
  /** Whether the bug-report form is open. */
  bugReportOpen: boolean;
  /** Where the bug-report form goes instead of floating over the viewport — the mobile "Bug" tab, when on mobile. */
  bugReportHost: HTMLElement | null;
  onBugReportOpen: () => void;
  onBugReportClose: () => void;
}

/** `name`, as an example is offered: `box_with_drill_hole` reads "Box with drill hole". */
function title(name: string): string {
  const words = name.replace(/_/g, " ");
  return words.charAt(0).toUpperCase() + words.slice(1);
}

/**
 * The app's top bar: the File menu — opening, saving, the examples — undo
 * and redo, the operations (desktop only; on mobile they have a tab), what
 * the editor is in the middle of, and Help.
 */
export function Toolbar({
  busy,
  hosted,
  hasSteps,
  onSave,
  onExportDrawing,
  onLoadFile,
  exampleNames,
  onLoadExample,
  workspaceExampleNames,
  onLoadWorkspaceExample,
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
  bugReportOpen,
  bugReportHost,
  onBugReportOpen,
  onBugReportClose,
}: Props) {
  const fileInput = useRef<HTMLInputElement>(null);
  const [privacyOpen, setPrivacyOpen] = useState(false);

  const file: MenuEntry[] = [
    ...(hosted
      ? []
      : ([
          { kind: "item", label: "Open file…", icon: "open", onSelect: () => fileInput.current?.click() },
          { kind: "item", label: "Save", icon: "save", hint: "download", disabled: !hasSteps, onSelect: onSave },
          { kind: "separator" },
        ] as MenuEntry[])),
    { kind: "heading", label: "Drawing" },
    {
      kind: "item",
      label: "Export drawing as SVG",
      icon: "drawing",
      hint: "views, hidden lines",
      disabled: !hasSteps,
      onSelect: () => onExportDrawing("svg"),
    },
    {
      kind: "item",
      label: "Export drawing as DXF",
      icon: "drawing",
      disabled: !hasSteps,
      onSelect: () => onExportDrawing("dxf"),
    },
    { kind: "separator" },
    { kind: "heading", label: "Example parts" },
    ...exampleNames.map(
      (name): MenuEntry => ({ kind: "item", label: title(name), icon: "example", onSelect: () => onLoadExample(name) }),
    ),
    ...(workspaceExampleNames.length > 0
      ? [
          { kind: "heading", label: "Example assemblies" } as MenuEntry,
          ...workspaceExampleNames.map(
            (name): MenuEntry => ({
              kind: "item",
              label: title(name),
              icon: "assembly",
              hint: "several files",
              onSelect: () => onLoadWorkspaceExample(name),
            }),
          ),
        ]
      : []),
  ];
  const help: MenuEntry[] = hosted
    ? []
    : [
        { kind: "item", label: "Report a bug…", icon: "bug", onSelect: onBugReportOpen },
        { kind: "item", label: "Privacy", icon: "privacy", onSelect: () => setPrivacyOpen(true) },
      ];

  return (
    <header className="toolbar">
      <span className="brand">geop</span>
      <Menu label="File" entries={file} disabled={busy} className="file-menu" />
      <input
        ref={fileInput}
        type="file"
        hidden
        accept=".geop,.json,application/json"
        onChange={(e) => {
          const chosen = e.target.files?.[0];
          e.target.value = "";
          if (chosen) onLoadFile(chosen);
        }}
      />
      <div className="toolbar-group" role="group" aria-label="History">
        <button className="tool-icon" disabled={busy || !canUndo} onClick={onUndo} title="Undo (Ctrl+Z)" aria-label="Undo">
          <Icon name="undo" />
        </button>
        <button
          className="tool-icon"
          disabled={busy || !canRedo}
          onClick={onRedo}
          title="Redo (Ctrl+Shift+Z)"
          aria-label="Redo"
        >
          <Icon name="redo" />
        </button>
      </div>
      <div className="toolbar-divider desktop-only" />
      <div className="desktop-only">{operationButtons}</div>
      <div className="toolbar-spacer" />
      {badge && <span className="mode-badge">{badge}</span>}
      {!busy && (
        <span className="stats" title="Steps in the program, and triangles drawn">
          {stepCount} step{stepCount === 1 ? "" : "s"} · {triangleCount} tris
        </span>
      )}
      {help.length > 0 && (
        <Menu label={<Icon name="help" />} title="Help" entries={help} align="right" className="help-menu" />
      )}
      {!hosted && (
        <>
          <BugReport
            open={bugReportOpen}
            program={program}
            committedError={error}
            panelHost={bugReportHost}
            onClose={onBugReportClose}
          />
          <Privacy open={privacyOpen} onClose={() => setPrivacyOpen(false)} />
        </>
      )}
    </header>
  );
}
