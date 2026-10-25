// A stand-in for the Messages API, for the local smoke test in docs/agent-chat.md: it answers
// count_tokens and messages (a tool_use of `run`; FAIL=1 makes messages answer 500) and logs
// the shape of what it was sent. No real API is called.
import { createServer } from "node:http";
let calls = 0;
createServer(async (req, res) => {
  let body = "";
  for await (const chunk of req) body += chunk;
  const json = JSON.parse(body || "{}");
  const shape = { path: req.url, model: json.model, max_tokens: json.max_tokens, tools: json.tools?.map((t) => t.name).join(","), system_cached: !!json.system?.[0]?.cache_control, top_cache: !!json.cache_control, thinking: json.thinking, output_config: json.output_config, messages: json.messages?.length };
  console.log(JSON.stringify(shape));
  res.setHeader("content-type", "application/json");
  if (req.url.startsWith("/v1/messages/count_tokens")) return res.end(JSON.stringify({ input_tokens: 3000 }));
  if (process.env.FAIL === "1") { res.statusCode = 500; return res.end(JSON.stringify({ type: "error", error: { type: "api_error", message: "boom" } })); }
  calls++;
  res.end(JSON.stringify({
    id: `msg_${calls}`, type: "message", role: "assistant", model: json.model, stop_reason: "tool_use", stop_sequence: null,
    content: [{ type: "thinking", thinking: "Plan.", signature: "sig" }, { type: "tool_use", id: `toolu_${calls}`, name: "run", input: {}, caller: { type: "direct" } }],
    usage: { input_tokens: 100, cache_creation_input_tokens: 2500, cache_read_input_tokens: 0, output_tokens: 200 },
  }));
}).listen(8788, "127.0.0.1", () => console.log("mock on 8788"));
