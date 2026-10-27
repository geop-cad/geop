// The files a file field offers, as the explorer choosing one shows them:
// a tree of folders, from the `/`-separated paths the kernel sends — each
// relative to the program, so a file beside its folder is under `..` — and
// a library every workspace shares as a root of its own, by its scheme
// (`std:hex_nut.geop`; see `geop_core_program::workspace::is_shared`).
// Nothing here knows what the files are for: the kernel says which of them
// the field takes.

import type { FileEntry } from "./geop";

/** A folder or a file of the tree. A folder's `key` is the start of its files' paths: `parts/`, `std:`. */
export type FileNode =
  | { kind: "folder"; name: string; key: string; children: FileNode[] }
  | { kind: "file"; name: string; path: string; accepted: boolean };

/** A row of the tree as shown: a node, how deep it is, and — for a folder — whether it is open. */
export interface FileRow {
  node: FileNode;
  depth: number;
  open: boolean;
}

/** The scheme a path of a shared library starts with, `std:`, if it is one. */
export function schemeOf(path: string): string | null {
  const match = /^([A-Za-z]{2,}):/.exec(path);
  return match ? `${match[1]}:` : null;
}

/** The folders a node with this key or path is in, outermost first, by key. */
export function foldersOf(path: string): string[] {
  const scheme = schemeOf(path);
  const rest = scheme ? path.slice(scheme.length) : path;
  const segments = rest.split("/").slice(0, -1);
  const folders = scheme ? [scheme] : [];
  let key = scheme ?? "";
  for (const segment of segments) {
    key += `${segment}/`;
    folders.push(key);
  }
  return folders;
}

/**
 * `files` as a tree: folders first, then files, each sorted by name — `..`
 * before every other folder, and a shared library's root after every other
 * entry.
 */
export function fileTree(files: readonly FileEntry[]): FileNode[] {
  const root: FileNode[] = [];
  const folders = new Map<string, FileNode[]>([["", root]]);
  const folder = (key: string, name: string, into: FileNode[]): FileNode[] => {
    let children = folders.get(key);
    if (!children) {
      children = [];
      folders.set(key, children);
      into.push({ kind: "folder", name, key, children });
    }
    return children;
  };
  for (const { path, accepted } of files) {
    let level = root;
    for (const key of foldersOf(path)) {
      const name = schemeOf(key) === key ? key : key.slice(0, -1).split("/").pop()!;
      level = folder(key, name, level);
    }
    const scheme = schemeOf(path);
    const name = (scheme ? path.slice(scheme.length) : path).split("/").pop()!;
    level.push({ kind: "file", name, path, accepted });
  }
  const rank = (n: FileNode) =>
    n.kind === "file" ? 2 : schemeOf(n.key) === n.key ? 3 : n.name === ".." ? 0 : 1;
  const sort = (nodes: FileNode[]) => {
    nodes.sort((a, b) => rank(a) - rank(b) || a.name.localeCompare(b.name));
    for (const n of nodes) if (n.kind === "folder") sort(n.children);
  };
  sort(root);
  return root;
}

/** Whether `query` matches the file `path`: every word of it is in the path, ignoring case. */
export function matches(path: string, query: string): boolean {
  const lower = path.toLowerCase();
  return query
    .toLowerCase()
    .split(/\s+/)
    .every((word) => lower.includes(word));
}

/**
 * The rows of `tree` shown: the files `query` matches, in the folders open —
 * those `expanded`, and every folder holding a match while there is a
 * query, so what is found is seen — and, with `acceptedOnly`, only
 * the files the field takes. A folder left with nothing to show is not shown.
 */
export function visibleRows(
  tree: readonly FileNode[],
  expanded: ReadonlySet<string>,
  query: string,
  acceptedOnly = false,
): FileRow[] {
  const searching = query.trim() !== "";
  const shows = (node: FileNode): boolean =>
    node.kind === "file"
      ? (!acceptedOnly || node.accepted) && (!searching || matches(node.path, query.trim()))
      : node.children.some(shows);
  const rows: FileRow[] = [];
  const walk = (nodes: readonly FileNode[], depth: number) => {
    for (const node of nodes) {
      if (!shows(node)) continue;
      const open = node.kind === "folder" && (searching || expanded.has(node.key));
      rows.push({ node, depth, open });
      if (node.kind === "folder" && open) walk(node.children, depth + 1);
    }
  };
  walk(tree, 0);
  return rows;
}

/** The key a row is known by: a folder's key, a file's path. */
export function rowKey(node: FileNode): string {
  return node.kind === "folder" ? node.key : node.path;
}
