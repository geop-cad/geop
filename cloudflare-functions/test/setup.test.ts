import assert from "node:assert/strict";
import { test } from "node:test";
import { DEFAULT_MODEL } from "../lib/agent/config.ts";
import { setupOf, type AgentEnv } from "../lib/agent/http.ts";

const env = (extra: Partial<AgentEnv> = {}): AgentEnv => ({
  AGENT_BUDGET: { getByName: () => ({}) } as unknown as AgentEnv["AGENT_BUDGET"],
  ANTHROPIC_API_KEY: "k",
  SESSION_SECRET: "s",
  TURNSTILE_SECRET: "t",
  ...extra,
});

test("defaults to Sonnet 5.5 at medium effort", () => {
  const setup = setupOf(env());
  assert.equal(DEFAULT_MODEL, "claude-sonnet-5-5");
  assert.equal(setup?.model, "claude-sonnet-5-5");
  assert.equal(setup?.effort, "medium");
});

test("refuses to run: switched off, a model without a price, an unknown effort, a secret or the binding missing", () => {
  assert.equal(setupOf(env({ AGENT_DISABLED: "1" })), null);
  assert.equal(setupOf(env({ AGENT_MODEL: "claude-unknown" })), null);
  assert.equal(setupOf(env({ AGENT_EFFORT: "max" })), null);
  for (const key of ["AGENT_BUDGET", "ANTHROPIC_API_KEY", "SESSION_SECRET", "TURNSTILE_SECRET"] as const) {
    assert.equal(setupOf(env({ [key]: undefined })), null);
  }
  assert.equal(setupOf(env({ AGENT_MODEL: "claude-opus-5-5" }))?.price.output, 20000);
});
