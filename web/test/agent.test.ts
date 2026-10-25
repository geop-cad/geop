import assert from "node:assert/strict";
import { test } from "node:test";
import {
  AgentError,
  answer,
  describeTool,
  postTurn,
  usedUp,
  withPrompt,
  type ContentBlock,
  type MessageParam,
  type TurnAnswer,
} from "../src/agent.ts";

/** A server answering turns with `answers`, one after another, and what it was sent. */
function server(answers: TurnAnswer[]) {
  const sent: MessageParam[][] = [];
  return {
    sent,
    turn: async (messages: MessageParam[]) => {
      sent.push(structuredClone(messages));
      const next = answers.shift();
      if (!next) throw new Error("asked one turn too many");
      return next;
    },
  };
}

const said = (text: string, stop = "end_turn"): TurnAnswer => ({
  message: { content: [{ type: "text", text }], stop_reason: stop },
  limits: { turns_left_today: 9, turns_per_day: 10 },
});
const uses = (...calls: [string, string, unknown][]): TurnAnswer => ({
  message: {
    content: [
      { type: "thinking", thinking: "plan", signature: "sig" },
      ...calls.map(([id, name, input]): ContentBlock => ({ type: "tool_use", id, name, input })),
    ],
    stop_reason: "tool_use",
  },
});

test("every tool asked for runs, and all are answered in one message, until the model stops", async () => {
  const model = server([
    uses(["a", "add_step", { kind: "extrude", id: "box" }], ["b", "run", {}]),
    said("Done: a box."),
  ]);
  const ran: string[] = [];
  const result = await answer(
    withPrompt([], "a box"),
    {
      turn: model.turn,
      runTool: async (name) => {
        ran.push(name);
        return name === "run" ? { text: '{"failed": 1}', isError: false } : { text: "{}", isError: false };
      },
    },
    new AbortController().signal,
  );
  assert.deepEqual(ran, ["add_step", "run"]);
  assert.equal(result.ended, "end_turn");
  assert.equal(model.sent.length, 2);
  const second = model.sent[1];
  // The thinking block went back unchanged; the results came in one user message.
  assert.deepEqual(second[1].content[0], { type: "thinking", thinking: "plan", signature: "sig" });
  assert.equal(second[2].role, "user");
  assert.deepEqual(
    second[2].content.map((b) => (b.type === "tool_result" ? b.tool_use_id : b.type)),
    ["a", "b"],
  );
  assert.equal(result.messages.length, 4);
});

test("a tool that fails is answered as an error, and the loop goes on", async () => {
  const model = server([uses(["a", "delete_step", { id: "x" }]), said("It did not exist.")]);
  const result = await answer(
    withPrompt([], "remove x"),
    {
      turn: model.turn,
      runTool: async () => {
        throw new Error("the kernel crashed");
      },
    },
    new AbortController().signal,
  );
  const results = result.messages[2].content[0];
  assert.equal(results.type, "tool_result");
  assert.equal(results.type === "tool_result" && results.is_error, true);
  assert.equal(result.ended, "end_turn");
});

test("an answer takes no more turns than the cap, and ends answerable", async () => {
  const forever = Array.from({ length: 5 }, (_, i) => uses([`t${i}`, "run", {}]));
  const result = await answer(
    withPrompt([], "loop"),
    { turn: server(forever).turn, runTool: async () => ({ text: "{}", isError: false }) },
    new AbortController().signal,
    3,
  );
  assert.equal(result.ended, "turns");
  // The last message answers the last tool use: the next prompt joins it.
  const last = result.messages[result.messages.length - 1];
  assert.equal(last.role, "user");
  const next = withPrompt(result.messages, "continue");
  assert.equal(next.length, result.messages.length);
  assert.deepEqual(next[next.length - 1].content.at(-1), { type: "text", text: "continue" });
});

test("stopping answers the tools not run, and asks no further turn", async () => {
  const controller = new AbortController();
  const model = server([uses(["a", "run", {}], ["b", "run", {}]), said("never")]);
  let ran = 0;
  const result = await answer(
    withPrompt([], "go"),
    {
      turn: model.turn,
      runTool: async () => {
        ran++;
        controller.abort();
        return { text: "{}", isError: false };
      },
    },
    controller.signal,
  );
  assert.equal(ran, 1);
  assert.equal(result.ended, "stopped");
  assert.equal(model.sent.length, 1);
  const answered = result.messages[2].content;
  assert.equal(answered.length, 2, "both tool uses answered");
});

test("a tool use cut off by max_tokens is left out of the conversation", async () => {
  const cut: TurnAnswer = {
    message: {
      content: [
        { type: "text", text: "Adding" },
        { type: "tool_use", id: "a", name: "add_step", input: {} },
      ],
      stop_reason: "max_tokens",
    },
  };
  const result = await answer(
    withPrompt([], "x"),
    { turn: server([cut]).turn, runTool: async () => assert.fail("not run") },
    new AbortController().signal,
  );
  assert.equal(result.ended, "max_tokens");
  assert.deepEqual(result.messages[1].content, [{ type: "text", text: "Adding" }]);
});

test("the server's refusals are read from its JSON", async () => {
  const refusing = (status: number, body: unknown) => async () =>
    new Response(JSON.stringify(body), { status, headers: { "content-type": "application/json" } });
  await assert.rejects(
    postTurn([], undefined, refusing(429, { error: "quota", message: "used up" })),
    (e: unknown) => e instanceof AgentError && e.code === "quota" && usedUp(e),
  );
  await assert.rejects(
    postTurn([], undefined, refusing(429, { error: "rate", message: "slow down", retry_after: 3 })),
    (e: unknown) => e instanceof AgentError && e.code === "rate" && e.retryAfter === 3 && !usedUp(e),
  );
  await assert.rejects(
    postTurn([], undefined, async () => new Response("<html>", { status: 502 })),
    (e: unknown) => e instanceof AgentError && e.code === "upstream",
  );
  let body: unknown = null;
  const ok = await postTurn([{ role: "user", content: [{ type: "text", text: "hi" }] }], undefined, async (_, init) => {
    body = JSON.parse(String(init?.body));
    return new Response(JSON.stringify(said("hello")));
  });
  assert.equal(ok.message.stop_reason, "end_turn");
  assert.deepEqual(body, { messages: [{ role: "user", content: [{ type: "text", text: "hi" }] }] });
});

test("a tool's card says what it did, and what failed", () => {
  assert.equal(describeTool("add_step", { id: "box" }, { text: '{"failed": 0}', isError: false }), "Added step `box`");
  assert.equal(
    describeTool("edit_step", { id: "box" }, { text: '{"failed": 2}', isError: false }),
    "Changed step `box` — 2 steps failing",
  );
  assert.equal(
    describeTool("delete_step", { id: "x" }, { text: "delete_step failed: no step x\nmore", isError: true }),
    "Removed step `x`: delete_step failed: no step x",
  );
  assert.equal(
    describeTool(
      "add_step",
      { id: "b" },
      { text: "add_step failed: GeopError\nBacktrace: …\nRootError: no sketch\n", isError: true },
    ),
    "Added step `b`: no sketch",
  );
});
