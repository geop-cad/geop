import assert from "node:assert/strict";
import { test } from "node:test";
import { BYTES_PREFIX, fileContent } from "../src/files.ts";

test("a text file is held as its text", () => {
  assert.equal(fileContent(new TextEncoder().encode("solid cube\n")), "solid cube\n");
  assert.equal(fileContent(new TextEncoder().encode("café")), "café");
});

test("bytes that are not text are held as a data URL of them, as the kernel decodes it", () => {
  // A binary STL's header and triangle count: 80 bytes, then 12 as a u32.
  const bytes = new Uint8Array(84);
  bytes[80] = 12;
  bytes[0] = 0xff;
  const held = fileContent(bytes);
  assert.ok(held.startsWith(BYTES_PREFIX), held);
  assert.deepEqual(new Uint8Array(Buffer.from(held.slice(BYTES_PREFIX.length), "base64")), bytes);
  // `geop_core_program::workspace::tests::files_hold_bytes_as_text`
  // spells the same three bytes so.
  assert.equal(fileContent(new Uint8Array([0xff, 0xf8, 0xf1])), `${BYTES_PREFIX}//jx`);
});
