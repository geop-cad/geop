import * as fs from "node:fs";
import * as path from "node:path";
import * as vscode from "vscode";
import { GeopServer } from "./server";

/** What the page sends to the extension; see `web/src/backend.vscode.ts`. */
type FromPage =
  | { type: "ready" }
  | { type: "command"; id: number; command: string }
  | { type: "program"; program: unknown };

/**
 * A `.geop` file as the web editor: the page is the one `web/` builds, the
 * kernel a native `geop serve` process, and the file the program, as JSON.
 *
 * The text document stays the source of truth — saving, undo, hot exit,
 * diffs and source control are all VS Code's — so the page writes every
 * change to the program into it as a text edit, and is sent the text again
 * whenever it is changed some other way.
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
    const canonical = (text: string): string | null => {
      if (text.trim() === "") return canonicalOf({ steps: [] });
      try {
        return canonicalOf(JSON.parse(text));
      } catch {
        return null;
      }
    };
    const sendDocument = () => {
      known = canonical(document.getText());
      void panel.webview.postMessage({ type: "document", text: document.getText() });
    };

    subscriptions.push(
      vscode.workspace.onDidChangeTextDocument((e) => {
        if (e.document.uri.toString() !== document.uri.toString() || e.contentChanges.length === 0) return;
        // The page's own writes come back here: they are not news to it.
        if (known !== null && canonical(document.getText()) === known) return;
        sendDocument();
      }),
      panel.webview.onDidReceiveMessage(async (message: FromPage) => {
        switch (message.type) {
          case "ready":
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
          case "program": {
            const written = canonicalOf(message.program);
            if (written === canonical(document.getText())) break;
            known = written;
            const edit = new vscode.WorkspaceEdit();
            edit.replace(
              document.uri,
              new vscode.Range(0, 0, document.lineCount, 0),
              JSON.stringify(message.program, null, 2) + "\n",
            );
            await vscode.workspace.applyEdit(edit);
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
