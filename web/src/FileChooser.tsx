// A file field's explorer: the workspace's files as a tree of folders (see
// `fileTree.ts`), the one chosen revealed and selected, those the field
// does not take greyed, found by typing, moved through by the arrow keys,
// and chosen by a double click or Enter. Where the field takes files from
// elsewhere, one can be chosen from disk too, which the caller stores next
// to the program. Nothing here knows what the file is for.

import { useEffect, useMemo, useRef, useState, type KeyboardEvent } from "react";
import { createPortal } from "react-dom";
import { Chevron, FileIcon, UploadIcon } from "./Explorer";
import { fileTree, foldersOf, rowKey, visibleRows, type FileNode } from "./fileTree";
import type { FileEntry } from "./geop";

interface Props {
  /** What the field is called, as the explorer's title. */
  label: string;
  /** The path chosen now, if one is. */
  value: string;
  files: FileEntry[];
  /** The extensions, without the dot, of a file chosen from disk: none, for no upload. */
  upload: string[];
  onChoose: (path: string) => void;
  /** Store `file`, chosen from disk, next to the program: its path, or `null` if it could not be. */
  onUpload: (file: File) => Promise<string | null>;
  onClose: () => void;
}

/** How far a row at `depth` is indented. */
function indent(depth: number): string {
  return `${6 + depth * 14}px`;
}

/** The folder a row's node is in, by key: none at the top. */
function parentOf(node: FileNode): string | undefined {
  const path = node.kind === "folder" ? node.key.replace(/\/$/, "") : node.path;
  return foldersOf(path).pop();
}

export function FileChooser({ label, value, files, upload, onChoose, onUpload, onClose }: Props) {
  const tree = useMemo(() => fileTree(files), [files]);
  const [query, setQuery] = useState("");
  const [acceptedOnly, setAcceptedOnly] = useState(false);
  const [expanded, setExpanded] = useState<Set<string>>(() => new Set(value ? foldersOf(value) : []));
  const [selected, setSelected] = useState<string | null>(value || null);
  const rows = visibleRows(tree, expanded, query, acceptedOnly);
  const index = rows.findIndex((r) => rowKey(r.node) === selected);
  const current = index >= 0 ? rows[index] : undefined;
  const listRef = useRef<HTMLUListElement>(null);
  const uploadRef = useRef<HTMLInputElement>(null);

  // The selected row kept in sight as the keys move it.
  useEffect(() => {
    listRef.current?.querySelector(".selected")?.scrollIntoView({ block: "nearest" });
  }, [selected, rows.length]);

  const setOpen = (key: string, open: boolean) =>
    setExpanded((e) => {
      const next = new Set(e);
      if (open) next.add(key);
      else next.delete(key);
      return next;
    });
  const choose = (node: FileNode | undefined) => {
    if (node?.kind !== "file" || !node.accepted) return;
    onChoose(node.path);
    onClose();
  };
  /** A folder clicked or entered: opened or closed — not while searching, where every folder holding a match is open. */
  const toggle = (node: FileNode) => {
    if (node.kind === "folder" && query.trim() === "") setOpen(node.key, !expanded.has(node.key));
  };
  const move = (to: number) => {
    const row = rows[Math.max(0, Math.min(rows.length - 1, to))];
    if (row) setSelected(rowKey(row.node));
  };

  const onKeyDown = (e: KeyboardEvent) => {
    const searching = query.trim() !== "";
    switch (e.key) {
      case "ArrowDown":
        move(index + 1);
        break;
      case "ArrowUp":
        move(index < 0 ? rows.length - 1 : index - 1);
        break;
      case "Home":
        if (e.target instanceof HTMLInputElement && e.target.type === "search") return;
        move(0);
        break;
      case "End":
        if (e.target instanceof HTMLInputElement && e.target.type === "search") return;
        move(rows.length - 1);
        break;
      case "ArrowRight":
        if (current?.node.kind !== "folder") return;
        if (!current.open && !searching) setOpen(current.node.key, true);
        else move(index + 1);
        break;
      case "ArrowLeft": {
        if (!current) return;
        if (current.node.kind === "folder" && current.open && !searching) setOpen(current.node.key, false);
        else {
          const parent = parentOf(current.node);
          if (parent) setSelected(parent);
        }
        break;
      }
      case "Enter":
        if (!current) return;
        if (current.node.kind === "folder") toggle(current.node);
        else choose(current.node);
        break;
      case "Escape":
        onClose();
        break;
      default:
        return;
    }
    e.preventDefault();
    e.stopPropagation();
  };

  const chosen = current?.node.kind === "file" && current.node.accepted ? current.node : undefined;
  const accepted = files.filter((f) => f.accepted).length;

  const content = (
    <div className="file-chooser-backdrop" onMouseDown={(e) => e.target === e.currentTarget && onClose()}>
      <div className="file-chooser" role="dialog" aria-label={`Choose ${label}`} onKeyDown={onKeyDown}>
        <div className="file-chooser-head">
          <h2>Choose {label}</h2>
          <button className="file-chooser-close" title="Close (Escape)" onClick={onClose}>
            ×
          </button>
        </div>
        <input
          type="search"
          className="file-chooser-search"
          autoFocus
          placeholder="Search files…"
          value={query}
          onChange={(e) => {
            // The first file found that the field takes is selected, so Enter chooses it.
            const found = visibleRows(tree, expanded, e.target.value, acceptedOnly).find(
              ({ node }) => node.kind === "file" && node.accepted,
            );
            setQuery(e.target.value);
            setSelected(found ? rowKey(found.node) : null);
          }}
        />
        <ul ref={listRef} className="file-chooser-tree" role="tree" aria-label="Files">
          {rows.map(({ node, depth, open }) => {
            const key = rowKey(node);
            const classes = ["file-chooser-row", node.kind, key === selected ? "selected" : ""];
            if (node.kind === "file" && !node.accepted) classes.push("unaccepted");
            if (node.kind === "file" && node.path === value) classes.push("current");
            return (
              <li
                key={key}
                role="treeitem"
                aria-selected={key === selected}
                aria-expanded={node.kind === "folder" ? open : undefined}
                aria-disabled={node.kind === "file" && !node.accepted ? true : undefined}
                className={classes.join(" ")}
                style={{ paddingLeft: indent(depth) }}
                title={node.kind === "file" ? (node.accepted ? node.path : `${node.path} — not one this field takes`) : key}
                // The search box keeps the focus, and with it the keys.
                onMouseDown={(e) => e.preventDefault()}
                onClick={() => {
                  setSelected(key);
                  toggle(node);
                }}
                onDoubleClick={() => choose(node)}
              >
                {node.kind === "folder" ? <Chevron open={open} /> : <span className="file-chooser-chevron-space" />}
                {node.kind === "file" && <FileIcon />}
                <span className="file-chooser-name">{node.name}</span>
              </li>
            );
          })}
          {rows.length === 0 && (
            <li className="file-chooser-empty">
              {files.length === 0
                ? "There are no other files in this workspace."
                : query.trim() !== ""
                  ? `No file matches “${query.trim()}”.`
                  : "None of the files is one this field takes."}
            </li>
          )}
        </ul>
        <div className="file-chooser-footer">
          <label className="file-chooser-filter" title="Hide the files this field does not take">
            <input type="checkbox" checked={acceptedOnly} onChange={(e) => setAcceptedOnly(e.target.checked)} />
            Only usable files ({accepted})
          </label>
          {upload.length > 0 && (
            <>
              <button
                className="file-chooser-upload"
                title={`Add a file (${upload.map((e) => `.${e}`).join(", ")}) from disk, stored next to this program`}
                onClick={() => uploadRef.current?.click()}
              >
                <UploadIcon /> From disk…
              </button>
              <input
                ref={uploadRef}
                type="file"
                hidden
                accept={upload.map((e) => `.${e}`).join(",")}
                onChange={(e) => {
                  const file = e.target.files?.[0];
                  e.target.value = "";
                  if (!file) return;
                  void onUpload(file).then((path) => {
                    if (path == null) return;
                    onChoose(path);
                    onClose();
                  });
                }}
              />
            </>
          )}
          <span className="button-row">
            <button onClick={onClose}>Cancel</button>
            <button className="primary" disabled={!chosen} onClick={() => choose(chosen)}>
              Choose
            </button>
          </span>
        </div>
      </div>
    </div>
  );
  // Over everything, not inside the dialog: a popup's backdrop filter
  // would be the containing block of a fixed overlay inside it.
  return createPortal(content, document.body);
}
