// Stands in for VS Code in the end-to-end checks of the extension: serves
// the page the extension shows (`vscode-extension/media/`, from `npm run
// build:vscode`) with an `acquireVsCodeApi` shim that carries its messages
// over a WebSocket, and answers them as `vscode-extension/src/geopEditor.ts`
// does — each page its own `geop serve` process, run and restarted after a
// crash by the extension's own `GeopServer`, the workspace folder's other
// program files sent first, then the document.
//
// The page opens the document `doc` (relative to `folder`) at `/?doc=<doc>`.
// Every program it writes back is written to the document's file and
// recorded (`written[doc]`), every exported file it asks to save is
// recorded (`saved[doc]`), and a file it adds is written next to the
// document.

import fs from "node:fs";
import { createRequire } from "node:module";
import http from "node:http";
import path from "node:path";
import { WebSocketServer } from "ws";
import { freePort, ROOT } from "./lib.mjs";

// The extension's kernel process, compiled (`npm run compile` in
// `vscode-extension/`): how it talks to `geop serve`, and restarts it.
const { GeopServer, KernelCrashed, restoreCommands } = createRequire(import.meta.url)(
  path.join(ROOT, "vscode-extension", "out", "server.js"),
);

const SHIM = `<script>
const ws = new WebSocket("ws://" + location.host + "/ws" + location.search);
const queue = [];
ws.onopen = () => { for (const m of queue) ws.send(m); queue.length = 0; };
ws.onmessage = (e) => window.postMessage(JSON.parse(e.data), "*");
window.acquireVsCodeApi = () => ({ postMessage(m) { const s = JSON.stringify(m); ws.readyState === 1 ? ws.send(s) : queue.push(s); } });
</script>`;

const TYPES = {
  ".js": "text/javascript",
  ".css": "text/css",
  ".svg": "image/svg+xml",
  ".html": "text/html",
  ".wasm": "application/wasm",
};

/** Every `.geop` and STEP file under `folder` but `doc`, by path relative to it. */
function otherFiles(folder, doc) {
  const files = {};
  const walk = (dir) => {
    for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
      const p = path.join(dir, entry.name);
      const relative = path.relative(folder, p).split(path.sep).join("/");
      if (entry.isDirectory()) walk(p);
      else if (/\.(geop|step|stp)$/i.test(p) && relative !== doc) files[relative] = fs.readFileSync(p, "utf8");
    }
  };
  walk(folder);
  return files;
}

/** Serve `media` for documents of `folder`, each page driving its own `exe serve`. */
export async function startBridge({ media, exe, folder }) {
  const written = {};
  const saved = {};
  const server = http.createServer((req, res) => {
    const url = new URL(req.url, "http://x");
    const file = path.join(media, url.pathname === "/" ? "index.html" : decodeURIComponent(url.pathname));
    if (!file.startsWith(media) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) {
      res.writeHead(404);
      res.end();
      return;
    }
    let body = fs.readFileSync(file);
    if (file.endsWith("index.html")) body = body.toString().replace("<head>", "<head>" + SHIM);
    res.writeHead(200, { "content-type": TYPES[path.extname(file)] ?? "application/octet-stream" });
    res.end(body);
  });
  const servers = new Set();
  const wss = new WebSocketServer({ server, path: "/ws" });
  wss.on("connection", (ws, req) => {
    const doc = new URL(req.url, "http://x").searchParams.get("doc");
    // The extension's own kernel process, restarted as it restarts it.
    const geop = new GeopServer(
      exe,
      (text) => process.stderr.write(`[geop ${doc}] ${text}`),
      async () => restoreCommands(otherFiles(folder, doc), fs.readFileSync(path.join(folder, doc), "utf8"), doc),
    );
    servers.add(geop);
    ws.on("message", (raw) => {
      const message = JSON.parse(raw);
      switch (message.type) {
        case "ready": {
          const files = otherFiles(folder, doc);
          if (Object.keys(files).length > 0) ws.send(JSON.stringify({ type: "files", files }));
          const text = fs.readFileSync(path.join(folder, doc), "utf8");
          ws.send(JSON.stringify({ type: "document", text, path: doc }));
          break;
        }
        case "command":
          geop.request(message.command).then(
            (update) => ws.send(JSON.stringify({ type: "answer", id: message.id, update })),
            (e) =>
              ws.send(
                JSON.stringify({ type: "failure", id: message.id, message: e.message, crashed: e instanceof KernelCrashed }),
              ),
          );
          break;
        case "program":
          (written[doc] ??= []).push(message.program);
          // As VS Code would: the document is what the page wrote.
          fs.writeFileSync(path.join(folder, doc), JSON.stringify(message.program, null, 2) + "\n");
          break;
        case "save":
          (saved[doc] ??= []).push(message.file);
          break;
        case "add": {
          // As the extension does: stored next to the document, under
          // another name if one of its name there holds something else,
          // and sent to the kernel before the page is told its path.
          const dir = path.dirname(path.join(folder, doc));
          const name = path.basename(message.name);
          const dot = name.lastIndexOf(".");
          const [stem, extension] = dot > 0 ? [name.slice(0, dot), name.slice(dot)] : [name, ""];
          let target = path.join(dir, name);
          for (let n = 2; fs.existsSync(target) && fs.readFileSync(target, "utf8") !== message.text; n++) {
            target = path.join(dir, `${stem} ${n}${extension}`);
          }
          fs.writeFileSync(target, message.text);
          const relative = path.relative(folder, target).split(path.sep).join("/");
          ws.send(JSON.stringify({ type: "files", files: { [relative]: message.text } }));
          ws.send(JSON.stringify({ type: "added", id: message.id, path: path.basename(target) }));
          break;
        }
      }
    });
    ws.on("close", () => {
      geop.dispose();
      servers.delete(geop);
    });
  });
  const port = await freePort();
  await new Promise((resolve) => server.listen(port, "127.0.0.1", resolve));
  return {
    url: (doc) => `http://127.0.0.1:${port}/?doc=${encodeURIComponent(doc)}`,
    written,
    saved,
    stop() {
      for (const geop of servers) geop.dispose();
      wss.close();
      server.close();
    },
  };
}
