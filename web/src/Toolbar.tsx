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
  /** Export the assembly as a URDF robot, for simulators. */
  onExportUrdf: () => void;
  /** Write the part shown as a STEP file and save it. */
  onExportStep: () => void;
  /** Write the flat pattern of the newest sheet-metal body as a DXF file for laser cutting. */
  onExportFlatPattern: () => void;
  /** Write every solid of the part shown as an STL mesh and save it. */
  onExportStl: () => void;
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
  /** The operations, as a ribbon: shown on desktop only — MobileBottom has its own tab of them. */
  operationRibbon: ReactNode;
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
  onExportUrdf,
  onExportStep,
  onExportFlatPattern,
  onExportStl,
  onLoadFile,
  exampleNames,
  onLoadExample,
  workspaceExampleNames,
  onLoadWorkspaceExample,
  canUndo,
  onUndo,
  canRedo,
  onRedo,
  operationRibbon,
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
        ] as MenuEntry[])),
    { kind: "item", label: "Download STEP", icon: "save", hint: "for other CAD", disabled: !hasSteps, onSelect: onExportStep },
    { kind: "item", label: "Download STL", icon: "save", hint: "mesh, for 3-D printing", disabled: !hasSteps, onSelect: onExportStl },
    { kind: "separator" },
    { kind: "heading", label: "Sheet metal" },
    {
      kind: "item",
      label: "Export flat pattern as DXF",
      icon: "flat_pattern",
      hint: "for laser cutting",
      disabled: !hasSteps,
      onSelect: onExportFlatPattern,
    },
    { kind: "separator" },
    { kind: "heading", label: "Robot" },
    {
      kind: "item",
      label: "Export URDF",
      icon: "robot",
      hint: "joints, inertia, meshes",
      disabled: !hasSteps,
      onSelect: onExportUrdf,
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
      {operationRibbon}
      <div className="toolbar-spacer mobile-only" />
      {badge && <span className="mode-badge">{badge}</span>}
      {!busy && (
        <span className="stats" title="Steps in the program, and triangles drawn">
          <span>
            {stepCount} step{stepCount === 1 ? "" : "s"}
          </span>
          {/* Two lines in the bar; one in what is read of it. */}
          <span className="stats-separator"> · </span>
          <span>{triangleCount} tris</span>
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
