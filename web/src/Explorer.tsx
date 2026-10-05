// The browser's program files as VS Code's explorer shows a workspace: a
// tree of folders and `.geop` files, the one edited lit, with what can be
// done to them — new, rename, delete, upload, download — beside them. What
// happens to the files is the caller's (see `files.ts`); this only shows
// them and says what the user asked for.

import { useRef, useState, type ReactNode } from "react";
import { asPath, tree, validPath, type Entry, type Workspace } from "./files";

interface Props {
  workspace: Workspace;
  /** Whether another file can be opened, made or changed: not while a step is edited. */
  enabled: boolean;
  onOpen: (path: string) => void;
  onCreate: (path: string) => void;
  onRename: (from: string, to: string) => void;
  /** Delete these files: one, or every file of a folder. */
  onDelete: (paths: string[]) => void;
  onUpload: (files: File[]) => void;
  onDownload: (path: string) => void;
}

/** An icon, 16 px, drawn in the text's color. */
function Icon({ children, title }: { children: ReactNode; title?: string }) {
  return (
    <svg className="icon" viewBox="0 0 16 16" width="16" height="16" aria-hidden={title ? undefined : true}>
      {title && <title>{title}</title>}
      {children}
    </svg>
  );
}

const stroke = { fill: "none", stroke: "currentColor", strokeWidth: 1.2, strokeLinejoin: "round" as const };

const NewFileIcon = () => (
  <Icon>
    <path {...stroke} d="M4 1.5h5l3 3v5M4 1.5v13h5M9 1.5v3h3" />
    <path {...stroke} d="M12 11v4M10 13h4" />
  </Icon>
);
const UploadIcon = () => (
  <Icon>
    <path {...stroke} d="M8 11V2.5M5 5.5l3-3 3 3M2.5 10v3.5h11V10" />
  </Icon>
);
const DownloadIcon = () => (
  <Icon>
    <path {...stroke} d="M8 2.5V11M5 8l3 3 3-3M2.5 10v3.5h11V10" />
  </Icon>
);
const RenameIcon = () => (
  <Icon>
    <path {...stroke} d="M10.5 2.5l3 3-8 8h-3v-3z" />
  </Icon>
);
const DeleteIcon = () => (
  <Icon>
    <path {...stroke} d="M3 4.5h10M6.5 4.5V3h3v1.5M4.5 4.5l.6 9h5.8l.6-9" />
  </Icon>
);
const FileIcon = () => (
  <Icon>
    <path d="M4 2h5l3 3v9H4z" fill="#4472c4" opacity="0.25" />
    <path {...stroke} stroke="#6c9cff" d="M4 2h5l3 3v9H4zM9 2v3h3" />
  </Icon>
);
const Chevron = ({ open }: { open: boolean }) => (
  <Icon>
    <path {...stroke} d={open ? "M4.5 6.5l3.5 3.5 3.5-3.5" : "M6.5 4.5l3.5 3.5-3.5 3.5"} />
  </Icon>
);

/** An inline name field, as VS Code's explorer edits names: Enter takes it, Escape or leaving it does not. */
function NameInput({
  initial,
  depth,
  existing,
  onDone,
}: {
  initial: string;
  depth: number;
  /** Paths already taken, which the result must not be — except `initial` itself. */
  existing: (path: string) => boolean;
  /** The path chosen, or `null` for none. */
  onDone: (path: string | null) => void;
}) {
  const [value, setValue] = useState(initial);
  const done = useRef(false);
  const path = asPath(value);
  const problem =
    value.trim() === ""
      ? null
      : !validPath(path)
        ? "Not a valid file name"
        : path !== initial && existing(path)
          ? `${path} already exists`
          : null;
  const finish = (path: string | null) => {
    if (done.current) return;
    done.current = true;
    onDone(path);
  };
  return (
    <li className="explorer-row editing" style={{ paddingLeft: indent(depth) }}>
      <FileIcon />
      <div className="explorer-input">
        <input
          autoFocus
          value={value}
          placeholder="name.geop"
          className={problem ? "invalid" : ""}
          onFocus={(e) => e.target.setSelectionRange(0, e.target.value.replace(/\.geop$/, "").length)}
          onChange={(e) => setValue(e.target.value)}
          onKeyDown={(e) => {
            e.stopPropagation();
            if (e.key === "Enter" && !problem && value.trim() !== "") finish(path);
            if (e.key === "Escape") finish(null);
          }}
          onBlur={() => finish(null)}
        />
        {problem && <div className="explorer-problem">{problem}</div>}
      </div>
    </li>
  );
}

/** How far a row at `depth` is indented. */
function indent(depth: number): string {
  return `${8 + depth * 12}px`;
}

/** The parent folder of `path`, with a trailing `/` — or nothing, at the top. */
function folderOf(path: string): string {
  const slash = path.lastIndexOf("/");
  return slash < 0 ? "" : path.slice(0, slash + 1);
}

export function Explorer({ workspace, enabled, onOpen, onCreate, onRename, onDelete, onUpload, onDownload }: Props) {
  const [collapsed, setCollapsed] = useState<Set<string>>(new Set());
  const [sectionOpen, setSectionOpen] = useState(true);
  /** What is being named: a new file, in a folder, or a file being renamed. */
  const [naming, setNaming] = useState<{ kind: "new"; folder: string } | { kind: "rename"; path: string } | null>(null);
  const uploadRef = useRef<HTMLInputElement>(null);
  const exists = (path: string) => path in workspace.files;
  const activeFolder = folderOf(workspace.active);

  const toggle = (folder: string) =>
    setCollapsed((c) => {
      const next = new Set(c);
      if (next.has(folder)) next.delete(folder);
      else next.add(folder);
      return next;
    });

  const rows = (entries: Entry[], depth: number): ReactNode[] =>
    entries.flatMap((entry) => {
      if (entry.kind === "folder") {
        const open = !collapsed.has(entry.path);
        const inside = open ? rows(entry.children, depth + 1) : [];
        if (open && naming?.kind === "new" && naming.folder === `${entry.path}/`) {
          inside.unshift(newFileRow(depth + 1));
        }
        return [
          <li
            key={`folder:${entry.path}`}
            className="explorer-row folder"
            style={{ paddingLeft: indent(depth) }}
            onClick={() => toggle(entry.path)}
            title={entry.path}
          >
            <Chevron open={open} />
            <span className="explorer-name">{entry.name}</span>
            <span className="explorer-row-actions">
              <button
                title="Delete folder"
                disabled={!enabled}
                onClick={(e) => {
                  e.stopPropagation();
                  const files = Object.keys(workspace.files).filter((p) => p.startsWith(`${entry.path}/`));
                  const what = files.length === 1 ? "its file" : `its ${files.length} files`;
                  if (window.confirm(`Delete ${entry.path} and ${what}? This cannot be undone.`)) onDelete(files);
                }}
              >
                <DeleteIcon />
              </button>
            </span>
          </li>,
          ...inside,
        ];
      }
      if (naming?.kind === "rename" && naming.path === entry.path) {
        return [
          <NameInput
            key={`rename:${entry.path}`}
            initial={entry.path}
            depth={depth}
            existing={exists}
            onDone={(to) => {
              setNaming(null);
              if (to && to !== entry.path) onRename(entry.path, to);
            }}
          />,
        ];
      }
      const active = entry.path === workspace.active;
      return [
        <li
          key={`file:${entry.path}`}
          className={`explorer-row file${active ? " active" : ""}${enabled ? "" : " disabled"}`}
          style={{ paddingLeft: indent(depth) }}
          title={entry.path}
          onClick={() => enabled && !active && onOpen(entry.path)}
        >
          <span className="explorer-chevron-space" />
          <FileIcon />
          <span className="explorer-name">{entry.name}</span>
          <span className="explorer-row-actions">
            <button
              title="Rename"
              disabled={!enabled}
              onClick={(e) => {
                e.stopPropagation();
                setNaming({ kind: "rename", path: entry.path });
              }}
            >
              <RenameIcon />
            </button>
            <button
              title="Delete"
              disabled={!enabled}
              onClick={(e) => {
                e.stopPropagation();
                if (window.confirm(`Delete ${entry.path}? This cannot be undone.`)) onDelete([entry.path]);
              }}
            >
              <DeleteIcon />
            </button>
          </span>
        </li>,
      ];
    });

  const newFileRow = (depth: number) => (
    <NameInput
      key="new-file"
      initial={naming?.kind === "new" ? naming.folder : ""}
      depth={depth}
      existing={exists}
      onDone={(path) => {
        setNaming(null);
        if (path) onCreate(path);
      }}
    />
  );

  const top = rows(tree(workspace.files), 0);
  if (naming?.kind === "new" && naming.folder === "") top.unshift(newFileRow(0));

  return (
    <div className="explorer">
      <div className="explorer-title">Explorer</div>
      <div className="explorer-section-header" onClick={() => setSectionOpen((o) => !o)}>
        <Chevron open={sectionOpen} />
        <span className="explorer-section-name">Geop files</span>
        <span className="explorer-actions" onClick={(e) => e.stopPropagation()}>
          <button
            title="New file"
            disabled={!enabled}
            onClick={() => {
              setSectionOpen(true);
              setNaming({ kind: "new", folder: activeFolder });
            }}
          >
            <NewFileIcon />
          </button>
          <button title="Upload files" disabled={!enabled} onClick={() => uploadRef.current?.click()}>
            <UploadIcon />
          </button>
          <button title={`Download ${workspace.active}`} onClick={() => onDownload(workspace.active)}>
            <DownloadIcon />
          </button>
          <input
            ref={uploadRef}
            type="file"
            multiple
            accept=".geop,.json,application/json,.step,.stp"
            hidden
            onChange={(e) => {
              const files = [...(e.target.files ?? [])];
              e.target.value = "";
              if (files.length) onUpload(files);
            }}
          />
        </span>
      </div>
      {sectionOpen && <ul className="explorer-tree">{top}</ul>}
    </div>
  );
}
