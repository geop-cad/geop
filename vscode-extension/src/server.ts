import { ChildProcess, spawn } from "node:child_process";

/**
 * The kernel crashed running a command: it panicked — a bug, the message
 * says where — or its process ended. A fresh one has been started with the
 * document, as [[GeopServer]] says.
 */
export class KernelCrashed extends Error {}

/**
 * The commands that bring a fresh kernel to where one was: the other
 * program files of the workspace, by path, and the document — its text,
 * and its path, which the files it places are named relative to. A
 * document that is no program is left out: the page says why when it is
 * sent.
 */
export function restoreCommands(files: Record<string, string>, text: string, path: string): string[] {
  const commands = [JSON.stringify({ command: "files", files })];
  try {
    const program = text.trim() === "" ? { steps: [] } : JSON.parse(text);
    commands.push(JSON.stringify({ command: "load", program, path }));
  } catch {
    // Not a program: nothing to load.
  }
  return commands;
}

/**
 * One `geop serve` process: the kernel's editor (see `geop_cad_base::editor`)
 * behind a line protocol. Every command is a line of JSON on its stdin, and
 * is answered, in order, by one line on its stdout — so a request is matched
 * to its answer by position alone.
 */
class Kernel {
  private readonly waiting: { resolve: (update: string) => void; reject: (e: Error) => void }[] = [];
  private buffered = "";
  /** Why it is no longer running, once it is not. */
  private gone: Error | null = null;

  constructor(
    private readonly child: ChildProcess,
    onStderr: (text: string) => void,
    /** Called once, if it crashes: with what it said, or how it ended. */
    private readonly onCrash: (reason: KernelCrashed) => void,
  ) {
    child.stdout!.setEncoding("utf8");
    child.stdout!.on("data", (chunk: string) => this.read(chunk));
    child.stderr!.setEncoding("utf8");
    child.stderr!.on("data", onStderr);
    // Not started at all is no crash: another would fail alike.
    child.on("error", (e) => this.end(new Error(`could not run the geop kernel: ${e.message}`)));
    child.on("exit", (code, signal) =>
      this.crash(new KernelCrashed(`the geop kernel exited (${signal ?? `code ${code}`})`)),
    );
  }

  /** Send one command (JSON); resolves to the update (JSON). */
  request(command: string): Promise<string> {
    if (this.gone) return Promise.reject(this.gone);
    return new Promise((resolve, reject) => {
      this.waiting.push({ resolve, reject });
      // A command is one line: JSON.stringify never writes a raw newline.
      this.child.stdin!.write(command + "\n");
    });
  }

  /** Stop it: it crashed, or is not needed any more. */
  end(reason: Error) {
    this.gone ??= reason;
    for (const pending of this.waiting.splice(0)) pending.reject(this.gone);
    this.child.kill();
  }

  private crash(reason: KernelCrashed) {
    if (this.gone) return;
    this.end(reason);
    this.onCrash(reason);
  }

  private read(chunk: string) {
    this.buffered += chunk;
    for (let end = this.buffered.indexOf("\n"); end >= 0; end = this.buffered.indexOf("\n")) {
      const line = this.buffered.slice(0, end);
      this.buffered = this.buffered.slice(end + 1);
      // An update starts with its `error`; only a command the kernel could
      // not read at all is answered `{"fatal": ...}`, and a panic
      // `{"crashed": ...}`, after which the process ends (see `serve`).
      if (line.startsWith('{"crashed":')) {
        const message = (JSON.parse(line) as { crashed: string }).crashed;
        this.crash(new KernelCrashed(`the geop kernel crashed: ${message}`));
        return;
      }
      const pending = this.waiting.shift();
      if (!pending) continue;
      if (line.startsWith('{"fatal":')) pending.reject(new Error((JSON.parse(line) as { fatal: string }).fatal));
      else pending.resolve(line);
    }
  }
}

/**
 * The kernel of one open editor, which holds the state of the program
 * being edited — so one is started per editor, and lives exactly as long
 * as it.
 *
 * A kernel that crashes fails the command it was running, and every one
 * after it, with a [[KernelCrashed]]. Another is started at once and given
 * `restore()`'s commands — the workspace files and the document, see
 * [[restoreCommands]] — before any other: the program is back as the
 * document has it, and the page only has to show it again. One that
 * crashes again while being restored is not restarted: the document itself
 * crashes it, and every command fails saying so.
 */
export class GeopServer {
  private kernel: Kernel;
  /** The kernel being given the program after a crash, until it has it. */
  private restoring: Kernel | null = null;
  /** Settled once the kernel has the program: every command waits for it. */
  private restored: Promise<void> = Promise.resolve();
  /** Why it is stopped for good, once it is. */
  private gone: Error | null = null;

  constructor(
    private readonly executable: string,
    private readonly onStderr: (text: string) => void,
    private readonly restore: () => Promise<string[]>,
  ) {
    this.kernel = this.spawn();
  }

  /** Send one command (JSON); resolves to the update (JSON). */
  async request(command: string): Promise<string> {
    // After the restore going on when it was sent — and any started since.
    for (let restored = this.restored; ; restored = this.restored) {
      await restored;
      if (restored === this.restored) break;
    }
    if (this.gone) throw this.gone;
    return this.kernel.request(command);
  }

  dispose() {
    this.gone = new Error("the editor was closed");
    this.kernel.end(this.gone);
  }

  private spawn(): Kernel {
    const child = spawn(this.executable, ["serve"], { stdio: ["pipe", "pipe", "pipe"] });
    const kernel: Kernel = new Kernel(child, this.onStderr, (reason) => this.crashed(kernel, reason));
    return kernel;
  }

  private crashed(kernel: Kernel, reason: KernelCrashed) {
    if (this.gone) return;
    if (kernel === this.restoring) {
      this.gone = new KernelCrashed(
        `${reason.message} — again, while reloading the document: fix it, then close and reopen this editor`,
      );
      return;
    }
    this.onStderr(`${reason.message}; restarting it with the document\n`);
    const fresh = this.spawn();
    this.kernel = fresh;
    this.restoring = fresh;
    this.restored = (async () => {
      // A command refused is no crash: the page says why when it shows
      // the program. A crash sets `gone`.
      const commands = await this.restore().catch(() => []);
      await Promise.allSettled(commands.map((c) => fresh.request(c)));
      if (this.restoring === fresh) this.restoring = null;
    })();
  }
}
