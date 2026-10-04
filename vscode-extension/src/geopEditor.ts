import * as fs from "node:fs";
import * as path from "node:path";
import * as vscode from "vscode";
import { GeopServer } from "./server";

/** What the page sends to the extension; see `web/src/backend.vscode.ts`. */
type FromPage =
  | { type: "ready" }
  | { type: "command"; id: number; command: string }
  | { type: "program"; program: unknown }
  | { type: "save"; file: { name: string; text?: string; bytes?: string } };

/**
 * A `.geop` file as the web editor: the page is the one `web/` builds, the
 * kernel a native `geop serve` process, and the file the program, as JSON.
 *
 * The text document stays the source of truth — saving, undo, hot exit,
 * diffs and source control are all VS Code's — so the page writes every
 * change to the program into it as a text edit, and is sent the text again
 * whenever it is changed some other way.
 *
 * A program places the parts other `.geop` files build, by their paths
 * relative to its own. So the page is told the document's path within its
 * workspace folder, and sent every other `.geop` file of the folder — the
 * text of an open document as edited, saved or not — and again whenever
 * one changes, appears or goes.
 */
export class GeopEditorProvider implements vscode.CustomTextEditorProvider {
  static readonly viewType = "geop.editor";

  constructor(
    private readonly context: vscode.ExtensionContext,
    private readonly log: vscode.OutputChannel,
  ) {}

  async resolveCustomTextEditor(document: vscode.TextDocument, panel: vscode.WebviewPanel): Promise<void> {
    const media = vscode.Uri.joinPath(this.context.extensionUri, "media");
    panel.webview.options = { enableScripts: true, localResourceRoots: [media] };
    panel.webview.html = this.html(panel.webview, media);

    let server: GeopServer;
    try {
      server = GeopServer.start(this.executable(), (text) => this.log.append(text));
    } catch (e) {
      panel.webview.html = failure(String(e));
      return;
    }
    const subscriptions: vscode.Disposable[] = [{ dispose: () => server.dispose() }];
    panel.onDidDispose(() => subscriptions.forEach((s) => s.dispose()));

    /** The program last exchanged with the page, canonically written: what the document already says, so it need not be sent or written again. */
    let known: string | null = null;
    /**
     * The programs written into the document whose change has not come back
     * yet, canonically: the page's own writes, which are no news to it. A
     * drag writes one per frame, and their change events can arrive after
     * the next is already written — so not just the last one.
     */
    const written = new Set<string>();
    /** The newest program the page sent and that is not written yet: only it is, once the write before it is done. */
    let latest: unknown = null;
    let writing = false;
    const write = async () => {
      writing = true;
      try {
        while (latest !== null) {
          const program = latest;
          latest = null;
          const text = canonicalOf(program);
          if (text === canonical(document.getText())) continue;
          known = text;
          written.add(text);
          const edit = new vscode.WorkspaceEdit();
          edit.replace(document.uri, new vscode.Range(0, 0, document.lineCount, 0), JSON.stringify(program, null, 2) + "\n");
          await vscode.workspace.applyEdit(edit);
        }
      } finally {
        writing = false;
      }
    };
    const canonical = (text: string): string | null => {
      if (text.trim() === "") return canonicalOf({ steps: [] });
      try {
        return canonicalOf(JSON.parse(text));
      } catch {
        return null;
      }
    };
    const folder = vscode.workspace.getWorkspaceFolder(document.uri)?.uri ?? vscode.Uri.joinPath(document.uri, "..");
    /** `uri`'s path as the kernel names files: relative to the folder, `/`-separated. */
    const pathOf = (uri: vscode.Uri) => path.posix.relative(folder.path, uri.path);
    const isOther = (uri: vscode.Uri) =>
      uri.path.endsWith(".geop") && uri.toString() !== document.uri.toString() && !pathOf(uri).startsWith("..");
    /** The text of the program file `uri`: as edited, if it is open. */
    const textOf = async (uri: vscode.Uri): Promise<string> => {
      const open = vscode.workspace.textDocuments.find((d) => d.uri.toString() === uri.toString());
      if (open) return open.getText();
      return new TextDecoder().decode(await vscode.workspace.fs.readFile(uri));
    };
    const sendFiles = (files: Record<string, string | null>) => {
      if (Object.keys(files).length > 0) void panel.webview.postMessage({ type: "files", files });
    };
    const sendAllFiles = async () => {
      const uris = await vscode.workspace.findFiles(new vscode.RelativePattern(folder, "**/*.geop"), "**/node_modules/**");
      const files: Record<string, string | null> = {};
      for (const uri of uris.filter(isOther)) {
        try {
          files[pathOf(uri)] = await textOf(uri);
        } catch (e) {
          this.log.appendLine(`could not read ${uri.fsPath}: ${e}`);
        }
      }
      sendFiles(files);
    };
    const sendDocument = () => {
      known = canonical(document.getText());
      void panel.webview.postMessage({ type: "document", text: document.getText(), path: pathOf(document.uri) });
    };
    const watcher = vscode.workspace.createFileSystemWatcher(new vscode.RelativePattern(folder, "**/*.geop"));
    /** A file changed on disk: sent, unless it is open — then its edits are what counts, and were sent as made. */
    const onDisk = async (uri: vscode.Uri) => {
      if (!isOther(uri) || vscode.workspace.textDocuments.some((d) => d.uri.toString() === uri.toString() && d.isDirty)) return;
      try {
        sendFiles({ [pathOf(uri)]: await textOf(uri) });
      } catch (e) {
        this.log.appendLine(`could not read ${uri.fsPath}: ${e}`);
      }
    };

    subscriptions.push(
      watcher,
      watcher.onDidCreate(onDisk),
      watcher.onDidChange(onDisk),
      watcher.onDidDelete((uri) => {
        if (isOther(uri)) sendFiles({ [pathOf(uri)]: null });
      }),
      vscode.workspace.onDidChangeTextDocument((e) => {
        if (isOther(e.document.uri) && e.contentChanges.length > 0) {
          sendFiles({ [pathOf(e.document.uri)]: e.document.getText() });
        }
      }),
      // Closed unsaved, a document's file is what is on disk again.
      vscode.workspace.onDidCloseTextDocument((closed) => void onDisk(closed.uri)),
      vscode.workspace.onDidChangeTextDocument((e) => {
        if (e.document.uri.toString() !== document.uri.toString() || e.contentChanges.length === 0) return;
        // The page's own writes come back here: they are not news to it.
        const text = canonical(document.getText());
        if (text !== null && written.has(text)) {
          // This write has landed, and every one before it: none of those
          // can still come back.
          for (const pending of written) {
            written.delete(pending);
            if (pending === text) break;
          }
          return;
        }
        if (known !== null && text === known) return;
        sendDocument();
      }),
      panel.webview.onDidReceiveMessage(async (message: FromPage) => {
        switch (message.type) {
          case "ready":
            // The files first: the document is built with what it places.
            await sendAllFiles();
            sendDocument();
            break;
          case "command":
            try {
              const update = await server.request(message.command);
              void panel.webview.postMessage({ type: "answer", id: message.id, update });
            } catch (e) {
              void panel.webview.postMessage({ type: "failure", id: message.id, message: String(e) });
            }
            break;
          case "program":
            latest = message.program;
            if (!writing) await write();
            break;
          case "save": {
            // An exported file — a drawing, a robot — saved where the user
            // says, next to the document unless told otherwise: text, or
            // bytes sent as base64.
            const { name, text, bytes } = message.file;
            const target = await vscode.window.showSaveDialog({
              defaultUri: vscode.Uri.joinPath(document.uri, "..", name),
            });
            const content = bytes != null ? Buffer.from(bytes, "base64") : new TextEncoder().encode(text ?? "");
            if (target) await vscode.workspace.fs.writeFile(target, content);
            break;
          }
        }
      }),
    );
  }

  /** The kernel to run: the one configured, or the one bundled. */
  private executable(): string {
    const configured = vscode.workspace.getConfiguration("geop").get<string>("serverPath", "");
    if (configured) return configured;
    const bundled = path.join(
      this.context.extensionPath,
      "bin",
      process.platform === "win32" ? "geop.exe" : "geop",
    );
    if (!fs.existsSync(bundled)) {
      throw new Error(
        `no geop kernel at ${bundled}: build it with \`npm run build:server\`, or set geop.serverPath`,
      );
    }
    return bundled;
  }

  /** The built page, its asset URLs pointed at the webview's own origin. */
  private html(webview: vscode.Webview, media: vscode.Uri): string {
    const index = path.join(media.fsPath, "index.html");
    if (!fs.existsSync(index)) return failure(`the editor page is not built (${index}): run \`npm run build:web\``);
    const base = webview.asWebviewUri(media).toString();
    const csp = [
      "default-src 'none'",
      `img-src ${webview.cspSource} data: blob:`,
      `style-src ${webview.cspSource} 'unsafe-inline'`,
      `script-src ${webview.cspSource}`,
      `font-src ${webview.cspSource}`,
      `connect-src ${webview.cspSource}`,
    ].join("; ");
    return fs
      .readFileSync(index, "utf8")
      .replace(/(src|href)="\.\//g, `$1="${base}/`)
      .replace("<head>", `<head>\n    <meta http-equiv="Content-Security-Policy" content="${csp}" />`);
  }
}

/** A program as one string, however it was formatted: for telling whether two are the same. */
function canonicalOf(program: unknown): string {
  return JSON.stringify(program);
}

function failure(message: string): string {
  const escaped = message.replace(/&/g, "&amp;").replace(/</g, "&lt;");
  return `<!doctype html><html><body><p>Geop could not start: ${escaped}</p></body></html>`;
}
