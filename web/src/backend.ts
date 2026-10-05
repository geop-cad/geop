// Where the kernel runs: here, in the browser, as a wasm module in a
// worker (`kernel.worker.ts`) — off the page's thread, and replaced by a
// fresh one if it crashes.
//
// The app talks to the kernel through this module only (see `geop.ts`), so
// where the kernel lives is one build-time choice: `backend.vscode.ts`
// replaces this file in the VS Code extension's build (see `vite.config.ts`),
// where the kernel is a native process on the host. Both export the same
// things.
import { KernelCrashed } from "./crash";
import type { ExportedFile } from "./geop";
import type { FromKernel, ToKernel } from "./kernel.worker";

/**
 * The program files as somebody else owns them — a VS Code workspace, the
 * program edited being one of its documents. `null` where the app owns them
 * itself (the browser: see `files.ts`).
 */
export interface Host {
  /**
   * Call `onText` with the document's text and its path — which the files
   * it places are named relative to — now, and again whenever it is changed
   * from outside the app (a text edit, VS Code's own undo).
   */
  onDocument(onText: (text: string, path: string) => void): void;
  /**
   * Call `onFiles` with the other program files of the workspace, by path,
   * once the document has been sent, and with those that changed — their
   * text, or `null` for one that is gone — whenever any do.
   */
  onFiles(onFiles: (files: Record<string, string | null>) => void): void;
  /** The program is now this (a JSON value): write it to the document. */
  programChanged(program: unknown): void;
  /** Offer to save `file`, an exported file, next to the document. */
  saveFile(file: ExportedFile): void;
  /**
   * Store a file the user chose — `name`, holding `text` — next to the
   * document, under another name if a file of that name there holds
   * something else; resolves with its path relative to the document once
   * the kernel has been sent it (`onFiles`).
   */
  addFile(name: string, text: string): Promise<string>;
}

export const host: Host | null = null;

/** One worker running the kernel, and the commands it has not answered yet. */
interface Kernel {
  worker: Worker;
  loaded: Promise<void>;
  waiting: Map<number, { resolve: (update: string) => void; reject: (e: Error) => void }>;
}

let kernel: Kernel | null = null;
/** The kernel being given the program after a crash, until it has it. */
let restoring: Kernel | null = null;
/** Why the kernel is stopped for good, once it is: it crashed again while being given the program. */
let stopped: Error | null = null;
let nextId = 0;
/** The commands that bring a fresh kernel to where the app is (see [[onRestart]]). */
let restore: () => string[] = () => [];

/**
 * Restart a kernel that crashed with `commands()`: the files and the
 * program the app holds, as it was before the command that crashed it.
 */
export function onRestart(commands: () => string[]) {
  restore = commands;
}

function start(): Kernel {
  const worker = new Worker(new URL("./kernel.worker.ts", import.meta.url), { type: "module" });
  let loaded!: { resolve: () => void; reject: (e: Error) => void };
  const started: Kernel = {
    worker,
    loaded: new Promise((resolve, reject) => (loaded = { resolve, reject })),
    waiting: new Map(),
  };
  worker.onmessage = (e: MessageEvent<FromKernel>) => {
    const message = e.data;
    if ("loaded" in message) {
      if (message.loaded) loaded.resolve();
      else loaded.reject(new Error(message.error));
      return;
    }
    if ("crashed" in message) {
      crashed(started, `the kernel crashed: ${message.crashed}`);
      return;
    }
    const pending = started.waiting.get(message.id);
    started.waiting.delete(message.id);
    if ("update" in message) pending?.resolve(message.update);
    else pending?.reject(new Error(message.failure));
  };
  // A worker that dies without saying why — out of memory, say — crashed too.
  worker.onerror = (e) => crashed(started, `the kernel crashed: ${e.message}`);
  return started;
}

/** `dead` crashed: every command it has not answered fails, and a fresh kernel is given the program — unless it was being given it already. */
function crashed(dead: Kernel, reason: string) {
  if (dead !== kernel) return;
  dead.worker.terminate();
  const error = new KernelCrashed(reason);
  for (const pending of dead.waiting.values()) pending.reject(error);
  dead.waiting.clear();
  if (dead === restoring) {
    stopped = new KernelCrashed(`${reason} — again, while reloading the program: reload the page`);
    kernel = null;
    return;
  }
  const fresh = start();
  kernel = fresh;
  restoring = fresh;
  // Before anything else is sent to it: each worker runs its commands in order.
  void Promise.allSettled(restore().map((command) => request(fresh, command))).then(() => {
    if (restoring === fresh) restoring = null;
  });
}

function request(to: Kernel, command: string): Promise<string> {
  const id = nextId++;
  return new Promise((resolve, reject) => {
    to.waiting.set(id, { resolve, reject });
    to.worker.postMessage({ id, command } satisfies ToKernel);
  });
}

/** Start the kernel. Safe to call repeatedly; only runs once. */
export function loadBackend(): Promise<void> {
  kernel ??= start();
  return kernel.loaded;
}

/** Run one command (`geop_cad_base::Command` as JSON); the answer is an `Update` as JSON. */
export async function call(command: string): Promise<string> {
  if (stopped) throw stopped;
  kernel ??= start();
  return request(kernel, command);
}
