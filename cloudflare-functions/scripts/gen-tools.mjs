// Writes functions/api/agent/tools.json from the tools `geop mcp` lists,
// as Anthropic tool definitions ({name, description, input_schema}).
// Tools that only make sense with a file system (save, load_program) or
// that a user starts themselves (export: a download) are left out.
//
//   cargo build --release -p geop-cad-cli
//   node scripts/gen-tools.mjs [path/to/geop]
import { spawn } from "node:child_process";
import { writeFileSync } from "node:fs";
import { createInterface } from "node:readline";
import { fileURLToPath } from "node:url";

const DROPPED = new Set(["save", "load_program", "export"]);
const geop = process.argv[2] ?? fileURLToPath(new URL("../../target/release/geop", import.meta.url));
const out = fileURLToPath(new URL("../functions/api/agent/tools.json", import.meta.url));

const child = spawn(geop, ["mcp"], { stdio: ["pipe", "pipe", "inherit"] });
const timer = setTimeout(() => { console.error("geop mcp did not answer in time"); child.kill(); process.exit(1); }, 60_000);
const pending = new Map();
createInterface({ input: child.stdout }).on("line", (line) => {
  const message = JSON.parse(line);
  pending.get(message.id)?.(message);
});
function call(id, method, params) {
  return new Promise((resolve) => {
    pending.set(id, resolve);
    child.stdin.write(JSON.stringify({ jsonrpc: "2.0", id, method, params }) + "\n");
  });
}

await call(1, "initialize", { protocolVersion: "2025-06-18", capabilities: {}, clientInfo: { name: "gen-tools", version: "0" } });
child.stdin.write(JSON.stringify({ jsonrpc: "2.0", method: "notifications/initialized" }) + "\n");
const listed = await call(2, "tools/list", {});
if (listed.error) throw new Error(`tools/list failed: ${JSON.stringify(listed.error)}`);
const tools = listed.result.tools
  .filter((tool) => !DROPPED.has(tool.name))
  .map((tool) => ({ name: tool.name, description: tool.description, input_schema: tool.inputSchema }));
writeFileSync(out, JSON.stringify(tools, null, 2) + "\n");
console.log(`wrote ${tools.length} tools to ${out}: ${tools.map((t) => t.name).join(", ")}`);
clearTimeout(timer);
child.stdin.end();
child.kill();
