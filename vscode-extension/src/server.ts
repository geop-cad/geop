import { ChildProcess, spawn } from "node:child_process";

/**
 * One `geop serve` process: the kernel's editor (see `geop_cad_base::editor`)
 * behind a line protocol. Every command is a line of JSON on its stdin, and
 * is answered, in order, by one line on its stdout — so a request is matched
 * to its answer by position alone.
 *
 * The process holds the state of the program being edited, so one is
 * started per open editor, and lives exactly as long as it.
 */
export class GeopServer {
  private readonly waiting: { resolve: (update: string) => void; reject: (e: Error) => void }[] = [];
  private buffered = "";
  /** Why it is no longer running, once it is not. */
  private gone: Error | null = null;

  private constructor(
    private readonly child: ChildProcess,
    onStderr: (text: string) => void,
  ) {
    child.stdout!.setEncoding("utf8");
    child.stdout!.on("data", (chunk: string) => this.read(chunk));
    child.stderr!.setEncoding("utf8");
    child.stderr!.on("data", onStderr);
    child.on("error", (e) => this.end(new Error(`could not run the geop kernel: ${e.message}`)));
    child.on("exit", (code, signal) =>
      this.end(new Error(`the geop kernel exited (${signal ?? `code ${code}`}); close and reopen this editor`)),
    );
  }

  static start(executable: string, onStderr: (text: string) => void): GeopServer {
    return new GeopServer(spawn(executable, ["serve"], { stdio: ["pipe", "pipe", "pipe"] }), onStderr);
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

  dispose() {
    this.end(new Error("the editor was closed"));
    this.child.kill();
  }

  private read(chunk: string) {
    this.buffered += chunk;
    for (let end = this.buffered.indexOf("\n"); end >= 0; end = this.buffered.indexOf("\n")) {
      const line = this.buffered.slice(0, end);
      this.buffered = this.buffered.slice(end + 1);
      const pending = this.waiting.shift();
      if (!pending) continue;
      // An update starts with its `error`; only a command the kernel could
      // not read at all is answered `{"fatal": ...}` (see `serve`).
      if (line.startsWith('{"fatal":')) pending.reject(new Error((JSON.parse(line) as { fatal: string }).fatal));
      else pending.resolve(line);
    }
  }

  private end(reason: Error) {
    this.gone ??= reason;
    for (const pending of this.waiting.splice(0)) pending.reject(this.gone);
  }
}
