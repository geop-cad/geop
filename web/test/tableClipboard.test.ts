import assert from "node:assert/strict";
import { test } from "node:test";
import { parseBlock } from "../src/tableClipboard.ts";

test("a spreadsheet's block is rows of tab-separated cells, its trailing line end closing the last row", () => {
  assert.deepEqual(parseBlock("M4\t4\t2.8\nM5\t5\t3.5\n"), [
    ["M4", "4", "2.8"],
    ["M5", "5", "3.5"],
  ]);
  assert.deepEqual(parseBlock("a\tb\r\nc\td"), [
    ["a", "b"],
    ["c", "d"],
  ]);
});

test("a single cell is typed, not pasted as a block", () => {
  assert.equal(parseBlock("12.5"), null);
  assert.equal(parseBlock("width / 2\n"), null);
  assert.deepEqual(parseBlock("1\n2"), [["1"], ["2"]]);
  assert.deepEqual(parseBlock("1\t2"), [["1", "2"]]);
});

test("an empty cell is kept, so a pasted gap lands as one", () => {
  assert.deepEqual(parseBlock("a\t\tc\n\tb\t"), [
    ["a", "", "c"],
    ["", "b", ""],
  ]);
});
