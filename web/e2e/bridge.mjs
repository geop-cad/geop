// Stands in for VS Code in the end-to-end checks of the extension: serves
// the page the extension shows (`vscode-extension/media/`, from `npm run
// build:vscode`) with an `acquireVsCodeApi` shim that carries its messages
// over a WebSocket, and answers them as `vscode-extension/src/geopEditor.ts`
// does — each page its own `geop serve` process, the workspace folder's
// other program files sent first, then the document.
//
// The page opens the document `doc` (relative to `folder`) at `/?doc=<doc>`.
// What it writes back is recorded instead of edited into a document: every
// program (`written[doc]`) and every exported file it asks to save
// (`saved[doc]`).

import { spawn } from "node:child_process";
import fs from "node:fs";
import http from "node:http";
import path from "node:path";
import { WebSocketServer } from "ws";
import { freePort } from "./lib.mjs";

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
  const children = new Set();
  const wss = new WebSocketServer({ server, path: "/ws" });
  wss.on("connection", (ws, req) => {
    const doc = new URL(req.url, "http://x").searchParams.get("doc");
    const child = spawn(exe, ["serve"], { stdio: ["pipe", "pipe", "pipe"] });
    children.add(child);
    child.stderr.on("data", (d) => process.stderr.write(`[geop ${doc}] ${d}`));
    // Answers come in the order the commands went: a line each.
    const waiting = [];
    let buffered = "";
    child.stdout.setEncoding("utf8");
    child.stdout.on("data", (chunk) => {
      buffered += chunk;
      for (let end = buffered.indexOf("\n"); end >= 0; end = buffered.indexOf("\n")) {
        const line = buffered.slice(0, end);
        buffered = buffered.slice(end + 1);
        const id = waiting.shift();
        if (line.startsWith('{"fatal":')) ws.send(JSON.stringify({ type: "failure", id, message: JSON.parse(line).fatal }));
        else ws.send(JSON.stringify({ type: "answer", id, update: line }));
      }
    });
    child.on("exit", (code, signal) => {
      children.delete(child);
      // Whatever still waits is never answered: fail it, as the extension would.
      for (const id of waiting.splice(0)) {
        ws.send(JSON.stringify({ type: "failure", id, message: `geop serve exited (${code ?? signal})` }));
      }
    });
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
          waiting.push(message.id);
          child.stdin.write(message.command + "\n");
          break;
        case "program":
          (written[doc] ??= []).push(message.program);
          break;
        case "save":
          (saved[doc] ??= []).push(message.file);
          break;
      }
    });
    ws.on("close", () => child.kill());
  });
  const port = await freePort();
  await new Promise((resolve) => server.listen(port, "127.0.0.1", resolve));
  return {
    url: (doc) => `http://127.0.0.1:${port}/?doc=${encodeURIComponent(doc)}`,
    written,
    saved,
    stop() {
      for (const child of children) child.kill();
      wss.close();
      server.close();
    },
  };
}
