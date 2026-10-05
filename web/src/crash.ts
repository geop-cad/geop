// What the kernel crashing looks like to the app, wherever the kernel runs.

/**
 * The kernel crashed running a command: it panicked — a bug, `message`
 * says where — or ran out of memory. Whoever runs it has started another
 * already, with the program as it was before that command (see
 * `backend.ts`, and `vscode-extension/src/server.ts`), so the app only has
 * to show again what the kernel holds.
 */
export class KernelCrashed extends Error {
  constructor(message: string) {
    super(message);
    this.name = "KernelCrashed";
  }
}
