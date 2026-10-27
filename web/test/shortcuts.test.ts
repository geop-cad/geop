import assert from "node:assert/strict";
import { test } from "node:test";
import { formatShortcut, matches, parseChord, pressed, search, type KeyLike, type PaletteEntry } from "../src/shortcuts.ts";

const key = (k: string, mods: Partial<KeyLike> = {}): KeyLike => ({
  key: k,
  shiftKey: false,
  ctrlKey: false,
  metaKey: false,
  altKey: false,
  ...mods,
});

test("a letter matches whatever its case, Shift only where it says", () => {
  const e = parseChord("E");
  assert.equal(matches(e, key("e")), true);
  assert.equal(matches(e, key("E", { shiftKey: true })), false);
  assert.equal(matches(e, key("e", { ctrlKey: true })), false);
  const f = parseChord("Shift+F");
  assert.equal(matches(f, key("F", { shiftKey: true })), true);
  assert.equal(matches(f, key("f")), false);
});

test("Mod is Ctrl or ⌘, and a symbol typed with Shift is the symbol", () => {
  const k = parseChord("Mod+K");
  assert.equal(matches(k, key("k", { ctrlKey: true })), true);
  assert.equal(matches(k, key("k", { metaKey: true })), true);
  assert.equal(matches(k, key("k")), false);
  assert.equal(matches(parseChord("?"), key("?", { shiftKey: true })), true);
  assert.equal(matches(parseChord("Mod+Enter"), key("Enter", { ctrlKey: true })), true);
});

test("shortcuts are shown on the platform's terms", () => {
  assert.equal(formatShortcut("Shift+F", false), "Shift+F");
  assert.equal(formatShortcut("Shift+F", true), "⇧F");
  assert.equal(formatShortcut("Mod+K", false), "Ctrl+K");
  assert.equal(formatShortcut("Mod+K", true), "⌘K");
  assert.equal(formatShortcut("Escape", false), "Esc");
});

const entry = (label: string, group = "Operation", extra: Partial<PaletteEntry> = {}): PaletteEntry => ({
  id: label,
  label,
  group,
  run: () => {},
  ...extra,
});

test("the palette ranks a prefix first, then a word, then scattered letters, and leaves out what does not match", () => {
  const entries = [entry("Linear pattern"), entry("Extrude"), entry("Circular pattern"), entry("Extract face")];
  assert.deepEqual(
    search(entries, "pat").map((e) => e.label),
    ["Linear pattern", "Circular pattern"],
  );
  assert.deepEqual(
    search(entries, "ext").map((e) => e.label),
    ["Extrude", "Extract face"],
  );
  assert.deepEqual(
    search(entries, "exf").map((e) => e.label),
    ["Extract face"],
  );
  assert.deepEqual(search(entries, "zzz"), []);
  assert.equal(search(entries, "").length, entries.length);
});

test("a group name finds its entries, and what is disabled comes last", () => {
  const entries = [entry("Undo", "Edit", { disabled: true }), entry("Redo", "Edit"), entry("Extrude")];
  assert.deepEqual(
    search(entries, "edit").map((e) => e.label),
    ["Redo", "Undo"],
  );
});

test("a key press runs the entry whose shortcut it is, never a disabled one", () => {
  const entries = [
    entry("Extrude", "Operation", { shortcut: "E", disabled: true }),
    entry("Fillet", "Operation", { shortcut: "Shift+F" }),
  ];
  assert.equal(pressed(entries, key("e")), undefined);
  assert.equal(pressed(entries, key("F", { shiftKey: true }))?.label, "Fillet");
});
