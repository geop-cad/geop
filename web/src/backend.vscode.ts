// Where the kernel runs in the VS Code extension: in a native process the
// extension host spawns (`geop serve`), reached through the webview's
// message channel. See `backend.ts`, which this replaces, and
// `vscode-extension/src/` for the other end.
import type { Host } from "./backend";
import { KernelCrashed } from "./crash";

interface VsCodeApi {
  postMessage(message: unknown): void;
}
declare function acquireVsCodeApi(): VsCodeApi;

/** What the extension host sends to the page. */
type FromHost =
  | { type: "answer"; id: number; update: string }
  | { type: "failure"; id: number; message: string; crashed?: boolean }
  | { type: "document"; text: string; path: string }
  | { type: "files"; files: Record<string, string | null> }
  | { type: "added"; id: number; path: string }
  | { type: "not_added"; id: number; message: string };

const vscode = acquireVsCodeApi();

let nextId = 0;
const waiting = new Map<number, { resolve: (update: string) => void; reject: (e: Error) => void }>();
/** Files sent to be stored next to the document, by id, until the host has stored them. */
const adding = new Map<number, { resolve: (path: string) => void; reject: (e: Error) => void }>();
let onText: ((text: string, path: string) => void) | null = null;
let onFiles: ((files: Record<string, string | null>) => void) | null = null;
/** Files that came before anyone listened for them: the host sends them right after the document. */
let unheard: Record<string, string | null> = {};

window.addEventListener("message", (e: MessageEvent<FromHost>) => {
  const message = e.data;
  if (message.type === "document") {
    onText?.(message.text, message.path);
    return;
  }
  if (message.type === "files") {
    if (onFiles) onFiles(message.files);
    else unheard = { ...unheard, ...message.files };
    return;
  }
  if (message.type === "added" || message.type === "not_added") {
    const pending = adding.get(message.id);
    adding.delete(message.id);
    if (message.type === "added") pending?.resolve(message.path);
    else pending?.reject(new Error(message.message));
    return;
  }
  const pending = waiting.get(message.id);
  if (!pending) return;
  waiting.delete(message.id);
  if (message.type === "answer") pending.resolve(message.update);
  else pending.reject(message.crashed ? new KernelCrashed(message.message) : new Error(message.message));
});

export const host: Host = {
  onDocument(callback) {
    onText = callback;
    // The host sends the document once the page can take it.
    vscode.postMessage({ type: "ready" });
  },
  onFiles(callback) {
    onFiles = callback;
    if (Object.keys(unheard).length > 0) callback(unheard);
    unheard = {};
  },
  programChanged(program) {
    vscode.postMessage({ type: "program", program });
  },
  saveFile(file) {
    vscode.postMessage({ type: "save", file });
  },
  addFile(name, text) {
    const id = nextId++;
    return new Promise((resolve, reject) => {
      adding.set(id, { resolve, reject });
      vscode.postMessage({ type: "add", id, name, text });
    });
  },
};

/**
 * Nothing to do: the extension host restarts a kernel that crashed with the
 * workspace's files and the document itself (see
 * `vscode-extension/src/server.ts`).
 */
export function onRestart(_commands: () => string[]) {}

/** The process is started by the extension host, before the page is shown. */
export async function loadBackend(): Promise<void> {}

export function call(command: string): Promise<string> {
  const id = nextId++;
  return new Promise((resolve, reject) => {
    waiting.set(id, { resolve, reject });
    vscode.postMessage({ type: "command", id, command });
  });
}
