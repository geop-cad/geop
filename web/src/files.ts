// The browser's program files: what VS Code's workspace folder is for the
// extension. Each `.geop` file is a program, by path — `/`-separated, so a
// file in a folder is `parts/bolt.geop` — and a program places the parts of
// others by their paths relative to its own (see `geop_ops::program::library`).
// A STEP file (`.step`, `.stp`) is data a program imports, kept as its text.
//
// The files live here, in the page, and are kept in the browser's storage
// so they survive a reload; the kernel is sent them (`files` command) and
// told which one is edited (`load` with its path). Nothing here knows what
// a program says: the text is stored as the kernel wrote it.

import type { Program } from "./geop";

/** Every file, and the one being edited. */
export interface Workspace {
  /** Each file's program, as JSON text, by path. */
  files: Record<string, string>;
  active: string;
}

const STORAGE_KEY = "geop-workspace";
const EMPTY: Program = { steps: [] };

/** A program as a file's text: one step per object, as the kernel's own files read. */
export function programText(program: Program): string {
  return JSON.stringify(program, null, 2) + "\n";
}

/** The program a file's text holds — an empty file being an empty program. */
export function parseProgram(text: string): Program {
  return text.trim() === "" ? EMPTY : (JSON.parse(text) as Program);
}

/** What a new browser starts with: one empty file. */
function fresh(): Workspace {
  return { files: { "part.geop": programText(EMPTY) }, active: "part.geop" };
}

/** The workspace kept in this browser, or a fresh one. */
export function loadWorkspace(): Workspace {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (stored) {
      const workspace = JSON.parse(stored) as Workspace;
      if (Object.keys(workspace.files).length > 0) {
        if (!(workspace.active in workspace.files) || isStepFile(workspace.active))
          workspace.active = Object.keys(workspace.files).filter((p) => !isStepFile(p)).sort()[0] ?? "part.geop";
        if (!(workspace.active in workspace.files)) workspace.files[workspace.active] = programText(EMPTY);
        return workspace;
      }
    }
  } catch {
    // Storage blocked or unreadable: start afresh, in memory only.
  }
  return fresh();
}

/** Keep `workspace` in this browser, where storage allows. */
export function saveWorkspace(workspace: Workspace) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(workspace));
  } catch {
    // Storage blocked or full: the files still live for this page.
  }
}

/** Whether `path` names a STEP file, which a program imports rather than being one. */
export function isStepFile(path: string): boolean {
  const lower = path.toLowerCase();
  return lower.endsWith(".step") || lower.endsWith(".stp");
}

/** Whether `path` can name a file: not empty, no empty or dot-only segments, ending in `.geop` (or naming a STEP file). */
export function validPath(path: string): boolean {
  const segments = path.split("/");
  return (path.endsWith(".geop") || isStepFile(path)) && segments.every((s) => s !== "" && s !== "." && s !== "..");
}

/** `name`, as a file path: trimmed, `.geop` added if missing. */
export function asPath(name: string): string {
  const trimmed = name.trim().replace(/\\/g, "/").replace(/^\/+/, "");
  return trimmed.endsWith(".geop") ? trimmed : `${trimmed}.geop`;
}

/** A path not yet taken, like `path`: `part.geop`, then `part 2.geop`, ... */
export function freePath(files: Record<string, string>, path: string): string {
  if (!(path in files)) return path;
  const dot = path.lastIndexOf(".");
  const [stem, extension] = dot > 0 ? [path.slice(0, dot), path.slice(dot)] : [path, ""];
  for (let n = 2; ; n++) {
    const candidate = `${stem} ${n}${extension}`;
    if (!(candidate in files)) return candidate;
  }
}

/** A folder or a file of the tree the explorer shows. */
export type Entry =
  | { kind: "folder"; name: string; path: string; children: Entry[] }
  | { kind: "file"; name: string; path: string };

/** The files as a tree of folders — folders first, then files, each sorted by name. */
export function tree(files: Record<string, string>): Entry[] {
  const root: Entry[] = [];
  for (const path of Object.keys(files)) {
    const segments = path.split("/");
    let level = root;
    segments.slice(0, -1).forEach((name, i) => {
      const folderPath = segments.slice(0, i + 1).join("/");
      let folder = level.find((e) => e.kind === "folder" && e.name === name);
      if (!folder) {
        folder = { kind: "folder", name, path: folderPath, children: [] };
        level.push(folder);
      }
      level = (folder as Extract<Entry, { kind: "folder" }>).children;
    });
    level.push({ kind: "file", name: segments[segments.length - 1], path });
  }
  const sort = (entries: Entry[]) => {
    entries.sort((a, b) => (a.kind === b.kind ? a.name.localeCompare(b.name) : a.kind === "folder" ? -1 : 1));
    for (const e of entries) if (e.kind === "folder") sort(e.children);
  };
  sort(root);
  return root;
}
