/**
 * The one shape of conversation the endpoint forwards: a CAD session the
 * browser runs, with our tools. Everything else is refused, and what is
 * accepted is rebuilt field by field, so nothing the client adds (a
 * `cache_control`, a field of a newer API) reaches the request.
 */
import type Anthropic from "@anthropic-ai/sdk";
import {
  MAX_BODY_BYTES,
  MAX_IMAGE_BASE64_BYTES,
  MAX_IMAGES_PER_REQUEST,
  MAX_MESSAGES,
  MAX_USER_TEXT_CHARS,
} from "./config.ts";

type MessageParam = Anthropic.MessageParam;
type ContentBlockParam = Anthropic.ContentBlockParam;

export class Invalid extends Error {}

type Json = Record<string, unknown>;

function isObject(value: unknown): value is Json {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function fail(path: string, what: string): never {
  throw new Invalid(`${path}: ${what}`);
}

function string(value: unknown, path: string): string {
  if (typeof value !== "string") fail(path, "must be a string");
  return value;
}

function onlyKeys(value: Json, allowed: string[], path: string) {
  for (const key of Object.keys(value)) {
    if (!allowed.includes(key)) fail(path, `unexpected field \`${key}\``);
  }
}

interface Context {
  toolNames: ReadonlySet<string>;
  images: number;
  imageBytes: number;
}

function userText(text: unknown, path: string): string {
  const s = string(text, path);
  if (s.length > MAX_USER_TEXT_CHARS) fail(path, `longer than ${MAX_USER_TEXT_CHARS} characters`);
  return s;
}

function image(block: Json, path: string, ctx: Context): Anthropic.ImageBlockParam {
  onlyKeys(block, ["type", "source"], path);
  const source = block.source;
  if (!isObject(source)) fail(`${path}.source`, "must be an object");
  onlyKeys(source, ["type", "media_type", "data"], `${path}.source`);
  if (source.type !== "base64") fail(`${path}.source.type`, "must be base64");
  const mediaType = source.media_type;
  if (mediaType !== "image/png" && mediaType !== "image/jpeg") fail(`${path}.source.media_type`, "must be image/png or image/jpeg");
  const data = string(source.data, `${path}.source.data`);
  if (data.length > MAX_IMAGE_BASE64_BYTES) fail(`${path}.source.data`, `larger than ${MAX_IMAGE_BASE64_BYTES} bytes`);
  if (!/^[A-Za-z0-9+/]*={0,2}$/.test(data)) fail(`${path}.source.data`, "is not base64");
  ctx.images += 1;
  ctx.imageBytes += data.length;
  if (ctx.images > MAX_IMAGES_PER_REQUEST) fail(path, `at most ${MAX_IMAGES_PER_REQUEST} image per request`);
  return { type: "image", source: { type: "base64", media_type: mediaType, data } };
}

function toolResult(block: Json, path: string, ctx: Context, expected: Set<string>): Anthropic.ToolResultBlockParam {
  onlyKeys(block, ["type", "tool_use_id", "content", "is_error"], path);
  const id = string(block.tool_use_id, `${path}.tool_use_id`);
  if (!expected.delete(id)) fail(`${path}.tool_use_id`, "answers no tool_use of the message before");
  const result: Anthropic.ToolResultBlockParam = { type: "tool_result", tool_use_id: id };
  if (block.is_error !== undefined) {
    if (typeof block.is_error !== "boolean") fail(`${path}.is_error`, "must be a boolean");
    result.is_error = block.is_error;
  }
  const content = block.content;
  if (typeof content === "string") {
    result.content = content;
  } else if (Array.isArray(content)) {
    result.content = content.map((part, i) => {
      const p = `${path}.content[${i}]`;
      if (!isObject(part)) fail(p, "must be an object");
      if (part.type === "text") {
        onlyKeys(part, ["type", "text"], p);
        return { type: "text" as const, text: string(part.text, `${p}.text`) };
      }
      if (part.type === "image") return image(part, p, ctx);
      fail(`${p}.type`, "must be text or image");
    });
  } else if (content !== undefined) {
    fail(`${path}.content`, "must be a string or an array");
  }
  return result;
}

function userContent(content: unknown, path: string, ctx: Context, expected: Set<string>): string | ContentBlockParam[] {
  if (typeof content === "string") {
    if (expected.size > 0) fail(path, "must answer the tool_use blocks of the message before");
    return userText(content, path);
  }
  if (!Array.isArray(content) || content.length === 0) fail(path, "must be a string or a non-empty array");
  const blocks = content.map((block, i): ContentBlockParam => {
    const p = `${path}[${i}]`;
    if (!isObject(block)) fail(p, "must be an object");
    if (block.type === "text") {
      onlyKeys(block, ["type", "text"], p);
      return { type: "text", text: userText(block.text, `${p}.text`) };
    }
    if (block.type === "tool_result") return toolResult(block, p, ctx, expected);
    fail(`${p}.type`, "must be text or tool_result");
  });
  if (expected.size > 0) fail(path, `has no tool_result for ${[...expected].join(", ")}`);
  return blocks;
}

/** An assistant turn as the model returned it. Returns it and the ids of its tool calls. */
function assistantContent(content: unknown, path: string, ctx: Context): [ContentBlockParam[], Set<string>] {
  if (!Array.isArray(content) || content.length === 0) fail(path, "must be a non-empty array");
  const toolUses = new Set<string>();
  const blocks = content.map((block, i): ContentBlockParam => {
    const p = `${path}[${i}]`;
    if (!isObject(block)) fail(p, "must be an object");
    switch (block.type) {
      case "text":
        // `citations: null` comes back on text blocks; there are no citations here.
        onlyKeys(block, ["type", "text", "citations"], p);
        if (block.citations !== undefined && block.citations !== null) fail(`${p}.citations`, "must be null");
        return { type: "text", text: string(block.text, `${p}.text`) };
      case "thinking":
        onlyKeys(block, ["type", "thinking", "signature"], p);
        return { type: "thinking", thinking: string(block.thinking, `${p}.thinking`), signature: string(block.signature, `${p}.signature`) };
      case "redacted_thinking":
        onlyKeys(block, ["type", "data"], p);
        return { type: "redacted_thinking", data: string(block.data, `${p}.data`) };
      case "tool_use": {
        onlyKeys(block, ["type", "id", "name", "input", "caller", "toolset_name"], p);
        const id = string(block.id, `${p}.id`);
        const name = string(block.name, `${p}.name`);
        if (!ctx.toolNames.has(name)) fail(`${p}.name`, `\`${name}\` is not a tool of this assistant`);
        if (!isObject(block.input)) fail(`${p}.input`, "must be an object");
        // Our tools are only ever called by the model directly, never by a toolset.
        const caller = block.caller;
        if (caller !== undefined && !(isObject(caller) && caller.type === "direct" && Object.keys(caller).length === 1)) {
          fail(`${p}.caller`, "must be {\"type\": \"direct\"}");
        }
        if (block.toolset_name !== undefined && block.toolset_name !== null) fail(`${p}.toolset_name`, "must be null");
        if (toolUses.has(id)) fail(`${p}.id`, "is used twice");
        toolUses.add(id);
        return { type: "tool_use", id, name, input: block.input, ...(caller ? { caller: { type: "direct" as const } } : {}) };
      }
      default:
        fail(`${p}.type`, "must be text, thinking, redacted_thinking or tool_use");
    }
  });
  return [blocks, toolUses];
}

/**
 * The messages of a turn request's `body`, which is `rawBytes` long.
 * Throws `Invalid`, naming what is wrong, for anything but a conversation
 * of user and assistant turns, alternating and ending with the user, in
 * the blocks a browser CAD session sends.
 */
export function validateTurn(body: unknown, rawBytes: number, toolNames: ReadonlySet<string>): MessageParam[] {
  if (!isObject(body)) fail("body", "must be an object");
  onlyKeys(body, ["messages"], "body");
  const messages = body.messages;
  if (!Array.isArray(messages) || messages.length === 0) fail("messages", "must be a non-empty array");
  if (messages.length > MAX_MESSAGES) fail("messages", `more than ${MAX_MESSAGES}`);
  if (messages.length % 2 === 0) fail("messages", "must end with a user turn");
  const ctx: Context = { toolNames, images: 0, imageBytes: 0 };
  let expected = new Set<string>();
  const out = messages.map((message, i): MessageParam => {
    const path = `messages[${i}]`;
    if (!isObject(message)) fail(path, "must be an object");
    onlyKeys(message, ["role", "content"], path);
    const role = i % 2 === 0 ? "user" : "assistant";
    if (message.role !== role) fail(`${path}.role`, `must be ${role}: turns alternate, starting with the user`);
    if (role === "user") return { role, content: userContent(message.content, `${path}.content`, ctx, expected) };
    const [content, toolUses] = assistantContent(message.content, `${path}.content`, ctx);
    expected = toolUses;
    return { role, content };
  });
  if (rawBytes - ctx.imageBytes > MAX_BODY_BYTES) fail("body", `larger than ${MAX_BODY_BYTES} bytes besides the image`);
  return out;
}
