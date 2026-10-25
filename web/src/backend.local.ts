// Where the kernel runs in the page `geop open` serves: in the `geop`
// process serving it, reached over a WebSocket — each message a command of
// the protocol `geop serve` speaks on its stdin, answered in order by one
// message, the update. The files are a folder on disk, which the same
// process reads and writes for the page (`/api/files`, `/api/file`). See
// `backend.ts`, which this replaces, and `cad/geop-cad-cli/src/open.rs` for
// the other end.
//
// The page is given a token in its URL's fragment (`#token=...`), which
// the process asks of every call: no other page in the browser knows it.
import type { Host } from "./backend";
import { KernelCrashed } from "./crash";
import { isProgram, type Workspace, type WorkspaceStore } from "./files";

const token = new URLSearchParams(window.location.hash.slice(1)).get("token") ?? "";

/** A call of the file API. */
async function api(path: string, init: RequestInit = {}): Promise<Response> {
  const response = await fetch(path, { ...init, headers: { Authorization: `Bearer ${token}` } });
  if (!response.ok) throw new Error(`${response.status}: ${await response.text()}`);
  return response;
}

/** The folder's files, as `GET /api/files` lists them. */
interface Listing {
  root: string;
  files: Record<string, string>;
  /** The program named on the command line, to edit first. */
  active: string | null;
}

/** Why the page cannot work, if it cannot: then every call fails saying so. */
let failure: Error | null = null;

// Read before the app starts: its first render takes the files (`store.load`).
let listing: Listing | null = null;
try {
  listing = (await (await api("/api/files")).json()) as Listing;
  const name = listing.root.split(/[\\/]/).filter(Boolean).pop();
  if (name) document.title = `${name} · geop`;
} catch (e) {
  failure = new Error(
    `this page could not read its files from geop open (${e instanceof Error ? e.message : e}): ` +
      `open the URL geop open printed, with its #token`,
  );
}

export const host: Host | null = null;

/** What each file of the folder holds, as last read or written. */
const onDisk = new Map(Object.entries(listing?.files ?? {}));
/** The files as the page last had them, not yet written. */
let unwritten: Record<string, string> | null = null;
let writing = false;
let timer: ReturnType<typeof setTimeout> | null = null;
/** The failures said already: each is said once. */
const said = new Set<string>();

/** Whether a file holding `before` needs writing to hold `after`: a program, only if it says something else. */
function changed(path: string, before: string | undefined, after: string): boolean {
  if (before === after) return false;
  if (before == null || !isProgram(path)) return true;
  try {
    return JSON.stringify(JSON.parse(before)) !== JSON.stringify(JSON.parse(after));
  } catch {
    return true;
  }
}

/**
 * Write the files changed since they were last written, and remove those
 * gone. One write at a time, of the newest files: a drag changes them on
 * every frame, and only where it ends matters.
 */
async function write(keepalive = false) {
  if (writing || !unwritten) return;
  writing = true;
  const files = unwritten;
  unwritten = null;
  const query = (path: string) => `/api/file?path=${encodeURIComponent(path)}`;
  try {
    for (const [path, text] of Object.entries(files)) {
      if (!changed(path, onDisk.get(path), text)) continue;
      await api(query(path), { method: "PUT", body: text, keepalive: keepalive && text.length < 60_000 });
      onDisk.set(path, text);
    }
    for (const path of [...onDisk.keys()]) {
      if (path in files) continue;
      await api(query(path), { method: "DELETE", keepalive });
      onDisk.delete(path);
    }
  } catch (e) {
    // Written again with the next change.
    unwritten ??= files;
    const message = `geop could not save your files to disk: ${e instanceof Error ? e.message : e}`;
    console.error(message);
    if (!said.has(message)) {
      said.add(message);
      window.alert(message);
    }
  } finally {
    writing = false;
    if (unwritten && !keepalive) schedule();
  }
}

function schedule() {
  if (timer != null) clearTimeout(timer);
  timer = setTimeout(() => {
    timer = null;
    void write();
  }, 150);
}

// Leaving the page: what is not written yet is, without waiting.
window.addEventListener("pagehide", () => {
  if (timer != null) clearTimeout(timer);
  void write(true);
});

/** The files are the folder's. */
export const store: WorkspaceStore = {
  load: () => listing && { files: { ...listing.files }, active: listing.active ?? "" },
  save(workspace: Workspace) {
    if (!listing) return;
    unwritten = workspace.files;
    schedule();
  },
};

// ── the kernel ─────────────────────────────────────────────────────────

/** Each command sent and not answered yet, in the order sent: answered in that order. */
const waiting: { resolve: (update: string) => void; reject: (e: Error) => void }[] = [];
/** How many answers to drop: those to commands sent after one that crashed the kernel. */
let dropping = 0;
/** Whether the commands that give a fresh kernel the program are being answered. */
let restoring = 0;
/** Why the kernel is stopped for good, once it is. */
let stopped: Error | null = failure;
/** The commands that bring a fresh kernel to where the app is (see [[onRestart]]). */
let restore: () => string[] = () => [];
let socket: Promise<WebSocket> | null = null;
/** The socket, once open. */
let opened: WebSocket | null = null;

/**
 * Restart a kernel that crashed with `commands()`: the files and the
 * program the app holds. The process serving the page starts a fresh
 * editor itself.
 */
export function onRestart(commands: () => string[]) {
  restore = commands;
}

function stop(reason: Error) {
  stopped ??= reason;
  for (const pending of waiting.splice(0)) pending.reject(stopped);
}

function answered(line: string) {
  if (dropping > 0) {
    dropping--;
    return;
  }
  const pending = waiting.shift();
  const wasRestoring = restoring > 0;
  if (wasRestoring) restoring--;
  if (line.startsWith('{"crashed":')) {
    const reason = `the kernel crashed: ${(JSON.parse(line) as { crashed: string }).crashed}`;
    if (wasRestoring) {
      stop(new KernelCrashed(`${reason} — again, while reloading the program: reload the page`));
      return;
    }
    const error = new KernelCrashed(reason);
    pending?.reject(error);
    // What was sent after it went to the fresh kernel before it had the
    // program: failed, and its answers dropped.
    dropping = waiting.length;
    for (const later of waiting.splice(0)) later.reject(error);
    // Sent now, before any command the app sends on hearing of the crash.
    const commands = restore();
    restoring = commands.length;
    for (const command of commands) if (opened) send(opened, command).catch(() => {});
    return;
  }
  if (line.startsWith('{"fatal":')) pending?.reject(new Error((JSON.parse(line) as { fatal: string }).fatal));
  else pending?.resolve(line);
}

function connect(): Promise<WebSocket> {
  return new Promise((resolve, reject) => {
    const scheme = window.location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${scheme}://${window.location.host}/ws?token=${encodeURIComponent(token)}`);
    ws.onopen = () => {
      opened = ws;
      resolve(ws);
    };
    ws.onmessage = (e: MessageEvent<string>) => answered(e.data);
    ws.onerror = () => reject(new Error("could not reach geop open: is it still running?"));
    ws.onclose = () =>
      stop(new Error("geop open stopped: run it again, then reload this page (your files are saved on disk)"));
  });
}

function send(ws: WebSocket, command: string): Promise<string> {
  if (stopped) return Promise.reject(stopped);
  return new Promise((resolve, reject) => {
    waiting.push({ resolve, reject });
    ws.send(command);
  });
}

/** Connect to the kernel. Safe to call repeatedly; only connects once. */
export function loadBackend(): Promise<void> {
  if (stopped) return Promise.reject(stopped);
  socket ??= connect();
  return socket.then(() => {});
}

/** Run one command (`geop_cad_base::Command` as JSON); the answer is an `Update` as JSON. */
export async function call(command: string): Promise<string> {
  if (stopped) throw stopped;
  socket ??= connect();
  return send(await socket, command);
}

/**
 * Not offered here yet: the hosted assistant runs on the browser's own
 * kernel. Connect your own agent to the files instead (`geop mcp`).
 */
export function callTool(_name: string, _input: string): Promise<string> {
  return Promise.reject(new Error("the assistant is not offered in geop open yet: connect your own agent with `geop mcp`"));
}
