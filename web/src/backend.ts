// Where the kernel runs: here, in the browser, as a wasm module.
//
// The app talks to the kernel through this module only (see `geop.ts`), so
// where the kernel lives is one build-time choice: `backend.vscode.ts`
// replaces this file in the VS Code extension's build (see `vite.config.ts`),
// where the kernel is a native process on the host. Both export the same
// three things.
import init, { handle, init_panic_hook } from "./wasm/pkg/geop.js";

/**
 * The program as a file somebody else owns — a VS Code document. `null`
 * where the app owns it itself (the browser, which saves and loads by
 * download and upload).
 */
export interface Host {
  /**
   * Call `onText` with the document's text now, and again whenever it is
   * changed from outside the app (a text edit, VS Code's own undo).
   */
  onDocument(onText: (text: string) => void): void;
  /** The program is now this (a JSON value): write it to the document. */
  programChanged(program: unknown): void;
}

export const host: Host | null = null;

/** Start the kernel. Safe to call repeatedly; only runs once. */
export const loadBackend: () => Promise<void> = (() => {
  let ready: Promise<void> | null = null;
  return () => (ready ??= init().then(() => init_panic_hook()));
})();

/** Run one command (`geop_cad_base::Command` as JSON); the answer is an `Update` as JSON. */
export async function call(command: string): Promise<string> {
  return handle(command);
}
