import assert from "node:assert/strict";
import { test } from "node:test";
import { MAX_BODY_BYTES, MAX_MESSAGES, MAX_USER_TEXT_CHARS } from "../lib/agent/config.ts";
import { Invalid, validateTurn } from "../lib/agent/validate.ts";

const tools = new Set(["add_step", "run"]);
const check = (body: unknown, bytes = 1000) => validateTurn(body, bytes, tools);
const invalid = (body: unknown, pattern: RegExp, bytes = 1000) =>
  assert.throws(() => check(body, bytes), (e: unknown) => e instanceof Invalid && pattern.test(e.message));

const toolTurn = [
  { role: "user", content: "Make a 20 mm cube." },
  {
    role: "assistant",
    content: [
      { type: "thinking", thinking: "", signature: "sig" },
      { type: "text", text: "Adding it.", citations: null },
      { type: "tool_use", id: "tu1", name: "add_step", input: { kind: "extrude" }, caller: { type: "direct" } },
    ],
  },
  { role: "user", content: [{ type: "tool_result", tool_use_id: "tu1", content: [{ type: "text", text: "{\"failed\":0}" }] }] },
];

test("a whole tool round trip passes, rebuilt without extra fields", () => {
  const messages = check({ messages: toolTurn });
  assert.equal(messages.length, 3);
  assert.deepEqual(messages[1].content, [
    { type: "thinking", thinking: "", signature: "sig" },
    { type: "text", text: "Adding it." },
    { type: "tool_use", id: "tu1", name: "add_step", input: { kind: "extrude" }, caller: { type: "direct" } },
  ]);
});

test("the client sets nothing but messages", () => {
  for (const key of ["model", "system", "tools", "max_tokens", "thinking", "tool_choice"]) {
    invalid({ messages: [{ role: "user", content: "hi" }], [key]: "x" }, new RegExp(key));
  }
});

test("roles alternate, start and end with the user", () => {
  invalid({ messages: [] }, /non-empty/);
  invalid({ messages: [{ role: "assistant", content: [{ type: "text", text: "x" }] }] }, /must be user/);
  invalid({ messages: toolTurn.slice(0, 2) }, /end with a user/);
  invalid({ messages: [{ role: "user", content: "a" }, { role: "user", content: "b" }, { role: "user", content: "c" }] }, /must be assistant/);
  invalid({ messages: [{ role: "system", content: "x" }] }, /must be user/);
});

test("at most MAX_MESSAGES messages", () => {
  const messages = Array.from({ length: MAX_MESSAGES + 1 }, (_, i) =>
    i % 2 === 0 ? { role: "user", content: "x" } : { role: "assistant", content: [{ type: "text", text: "y" }] },
  );
  invalid({ messages }, /more than/);
});

test("user text is limited per block", () => {
  check({ messages: [{ role: "user", content: "x".repeat(MAX_USER_TEXT_CHARS) }] });
  invalid({ messages: [{ role: "user", content: "x".repeat(MAX_USER_TEXT_CHARS + 1) }] }, /longer than/);
  invalid({ messages: [{ role: "user", content: [{ type: "text", text: "x".repeat(MAX_USER_TEXT_CHARS + 1) }] }] }, /longer than/);
});

test("only our tools, and only blocks a CAD session sends", () => {
  const withTool = (name: string) => [toolTurn[0], { role: "assistant", content: [{ type: "tool_use", id: "t", name, input: {} }] }, { role: "user", content: [{ type: "tool_result", tool_use_id: "t", content: "ok" }] }];
  invalid({ messages: withTool("web_search") }, /not a tool/);
  invalid({ messages: [{ role: "user", content: [{ type: "document", source: {} }] }] }, /text or tool_result/);
  invalid({ messages: [{ role: "user", content: [{ type: "image", source: {} }] }] }, /text or tool_result/);
  invalid({ messages: [toolTurn[0], { role: "assistant", content: [{ type: "server_tool_use", id: "x" }] }, toolTurn[0]] }, /must be text, thinking/);
  invalid({ messages: [{ role: "user", content: [{ type: "text", text: "x", cache_control: { type: "ephemeral" } }] }] }, /cache_control/);
});

test("every tool_use is answered by a tool_result, and nothing else is", () => {
  invalid({ messages: [...toolTurn.slice(0, 2), { role: "user", content: "never mind" }] }, /answer the tool_use/);
  invalid({ messages: [...toolTurn.slice(0, 2), { role: "user", content: [{ type: "text", text: "x" }] }] }, /no tool_result for tu1/);
  invalid({ messages: [{ role: "user", content: [{ type: "tool_result", tool_use_id: "nope", content: "x" }] }] }, /answers no tool_use/);
});

const png = (data: string) => ({ type: "image", source: { type: "base64", media_type: "image/png", data } });
const withImages = (...images: unknown[]) => ({
  messages: [...toolTurn.slice(0, 2), { role: "user", content: [{ type: "tool_result", tool_use_id: "tu1", content: [{ type: "text", text: "shot" }, ...images] }] }],
});

test("one png or jpeg screenshot in a tool result", () => {
  check(withImages(png("aGVsbG8=")));
  invalid(withImages(png("aGVsbG8="), png("aGVsbG8=")), /at most 1 image/);
  invalid(withImages({ type: "image", source: { type: "base64", media_type: "image/gif", data: "aGVsbG8=" } }), /png or image\/jpeg/);
  invalid(withImages({ type: "image", source: { type: "url", url: "https://example.com/x.png" } }), /url/);
  invalid(withImages(png("x".repeat(300 * 1024 + 1))), /larger than/);
});

test("the body limit leaves the image out", () => {
  const data = "A".repeat(200 * 1024);
  check(withImages(png(data)), MAX_BODY_BYTES + data.length);
  invalid(withImages(png(data)), /larger than/, MAX_BODY_BYTES + data.length + 1);
  invalid({ messages: [{ role: "user", content: "hi" }] }, /larger than/, MAX_BODY_BYTES + 1);
});
