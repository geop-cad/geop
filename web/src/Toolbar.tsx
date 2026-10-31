import { useRef, useState, type ReactNode } from "react";
import type { ExporterInfo, Program, Report } from "./geop";
import { BugReport } from "./BugReport";
import { acceptedFiles } from "./files";
import { Icon } from "./icons";
import { Menu, type MenuEntry } from "./Menu";
import { Privacy } from "./Privacy";
import { title } from "./examples";
import { withShortcut } from "./shortcuts";
import logo from "./assets/logo.svg";

/** The community's Discord server. */
const DISCORD = "https://discord.gg/btRDgrcwJ5";

interface Props {
  busy: boolean;
  /** Whether the program files are somebody else's (VS Code): saving, loading and reporting are theirs. */
  hosted: boolean;
  hasSteps: boolean;
  /** Start a new, empty part in a file of its own. */
  onNewPart: () => void;
  onSave: () => void;
  /** What the kernel can write of the part: a "Download" each. */
  exporters: ExporterInfo[];
  /** The extensions of the data files programs read beside them: loadable too. */
  reads: string[];
  /** Write the part shown with the exporter `kind`, and save it. */
  onExport: (kind: string) => void;
  onLoadFile: (file: File) => void;
  /** Open another folder to edit, where one can be (`geop app`). */
  onOpenFolder: (() => void) | null;
  exampleNames: string[];
  onLoadExample: (name: string) => void;
  /** Examples of several files: none where the files are not the app's own (VS Code). */
  workspaceExampleNames: string[];
  onLoadWorkspaceExample: (name: string) => void;
  /** The keys of undo and redo, as the kernel names them. */
  undoKey: string | null;
  redoKey: string | null;
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
  /** Whether the bug-report form is open. */
  bugReportOpen: boolean;
  /** Where the bug-report form goes instead of floating over the viewport — the mobile "Bug" tab, when on mobile. */
  bugReportHost: HTMLElement | null;
  onBugReportOpen: () => void;
  /** What a bug report attaches of the session, as the kernel says it (see [[Report]]). */
  onBugReportCollect: () => Promise<Report | null>;
  /** Whether only the operations that are robust today are offered. */
  reducedMode: boolean;
  onReducedMode: (on: boolean) => void;
  onBugReportClose: () => void;
  /** Whether the assistant's panel is open; `null` where there is no assistant (VS Code, whose user has their own agent). */
  assistantOpen: boolean | null;
  onAssistant: () => void;
}

/**
 * The app's top bar: the File menu — opening, saving, the examples — undo
 * and redo, the operations (desktop only; on mobile they have a tab), what
 * the editor is in the middle of, and Help.
 */
export function Toolbar({
  undoKey,
  redoKey,
  busy,
  hosted,
  hasSteps,
  onNewPart,
  onSave,
  exporters,
  reads,
  onExport,
  onLoadFile,
  onOpenFolder,
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
  bugReportOpen,
  bugReportHost,
  onBugReportOpen,
  onBugReportCollect,
  reducedMode,
  onReducedMode,
  onBugReportClose,
  assistantOpen,
  onAssistant,
}: Props) {
  const fileInput = useRef<HTMLInputElement>(null);
  const [privacyOpen, setPrivacyOpen] = useState(false);

  const file: MenuEntry[] = [
    ...(hosted
      ? []
      : ([
          { kind: "item", label: "New part", icon: "new_part", hint: "empty file", onSelect: onNewPart },
          { kind: "item", label: "Open file…", icon: "open", onSelect: () => fileInput.current?.click() },
          ...(onOpenFolder
            ? [{ kind: "item", label: "Open folder…", icon: "open", hint: "in its own window", onSelect: onOpenFolder }]
            : []),
          { kind: "item", label: "Save", icon: "save", hint: "download", disabled: !hasSteps, onSelect: onSave },
        ] as MenuEntry[])),
    ...exporters.map(
      (exporter): MenuEntry => ({
        kind: "item",
        label: `Download ${exporter.label}`,
        icon: "save",
        hint: exporter.doc,
        disabled: !hasSteps,
        onSelect: () => onExport(exporter.kind),
      }),
    ),
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
  const help: MenuEntry[] = [
    {
      kind: "item",
      label: reducedMode ? "Show all operations" : "Show only robust operations",
      hint: reducedMode ? "reduced mode is on" : "reduced mode is off",
      title: "Reduced mode offers only the operations that are robust today; the others are experimental",
      icon: "more",
      onSelect: () => onReducedMode(!reducedMode),
    },
    ...(hosted
      ? []
      : ([
          { kind: "separator" },
          { kind: "item", label: "Report a bug…", icon: "bug", onSelect: onBugReportOpen },
          {
            kind: "item",
            label: "Discord community",
            icon: "chat",
            title: "Ask questions and show what you built: discord.gg/btRDgrcwJ5",
            onSelect: () => window.open(DISCORD, "_blank", "noopener"),
          },
          { kind: "item", label: "Privacy", icon: "privacy", onSelect: () => setPrivacyOpen(true) },
        ] as MenuEntry[])),
  ];

  return (
    <header className="toolbar">
      <span className="brand" title="geop">
        <img src={logo} alt="geop" width={20} height={20} />
        <span className="brand-name">geop</span>
      </span>
      <Menu label="File" entries={file} disabled={busy} className="file-menu" />
      <input
        ref={fileInput}
        type="file"
        hidden
        accept={acceptedFiles(reads)}
        onChange={(e) => {
          const chosen = e.target.files?.[0];
          e.target.value = "";
          if (chosen) onLoadFile(chosen);
        }}
      />
      <div className="toolbar-group" role="group" aria-label="History">
        <button className="tool-icon" disabled={busy || !canUndo} onClick={onUndo} title={withShortcut("Undo", undoKey)} aria-label="Undo">
          <Icon name="undo" />
        </button>
        <button
          className="tool-icon"
          disabled={busy || !canRedo}
          onClick={onRedo}
          title={withShortcut("Redo", redoKey)}
          aria-label="Redo"
        >
          <Icon name="redo" />
        </button>
      </div>
      {operationRibbon}
      <div className="toolbar-spacer mobile-only" />
      {badge && <span className="mode-badge">{badge}</span>}
      {!hosted && (
        <a
          className="star-link desktop-only"
          href="https://github.com/geop-cad/geop"
          target="_blank"
          rel="noopener"
          title="geop is open source: star it on GitHub"
          aria-label="Star on GitHub"
        >
          <span aria-hidden="true">★</span>
          <span className="link-label"> Star on GitHub</span>
        </a>
      )}
      {!hosted && (
        <a
          className="star-link discord-link desktop-only"
          href={DISCORD}
          target="_blank"
          rel="noopener"
          title="Ask questions and show what you built on the geop Discord"
          aria-label="Discord"
        >
          <Icon name="chat" />
          <span className="link-label"> Discord</span>
        </a>
      )}
      {!hosted && (
        <button
          className="star-link bug-link desktop-only"
          onClick={onBugReportOpen}
          title="Something went wrong? Send us the part and what happened"
          aria-label="Report a bug"
        >
          <Icon name="bug" />
          <span className="link-label"> Report a bug</span>
        </button>
      )}
      {assistantOpen != null && (
        <button
          className={`assistant-toggle desktop-only${assistantOpen ? " active" : ""}`}
          onClick={onAssistant}
          title="Ask the assistant to model or change the part"
          aria-label="Assistant"
          aria-pressed={assistantOpen}
        >
          <Icon name="chat" />
          <span className="link-label"> Assistant</span>
        </button>
      )}
      {help.length > 0 && (
        <Menu label={<Icon name="help" />} title="Help" entries={help} align="right" className="help-menu" arrow={false} />
      )}
      {!hosted && (
        <>
          <BugReport
            open={bugReportOpen}
            program={program}
            lastError={error}
            collect={onBugReportCollect}
            panelHost={bugReportHost}
            onClose={onBugReportClose}
          />
          <Privacy open={privacyOpen} onClose={() => setPrivacyOpen(false)} />
        </>
      )}
    </header>
  );
}
