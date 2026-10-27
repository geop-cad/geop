// Keyboard shortcuts as the kernel and the app name them — `E`,
// `Shift+F`, `Mod+K` (Ctrl, or ⌘ on a Mac) — matched against key
// events, shown on the platform's own terms, and searched by the command
// palette. Pure logic, tested by `npm test` (`test/shortcuts.test.ts`).

/** A key and the modifiers held with it, as a shortcut names them. */
export interface Chord {
  key: string;
  shift: boolean;
  /** Ctrl, or ⌘ on a Mac. */
  mod: boolean;
  alt: boolean;
}

/** `text` read as a chord: modifiers before the key, joined by `+` — `Shift+F`, `Mod+Enter`, `?`. */
export function parseChord(text: string): Chord {
  const parts = text.split("+");
  // `+` itself is the key of `Shift++`.
  const key = parts.pop() || "+";
  const has = (m: string) => parts.some((p) => p.toLowerCase() === m);
  return { key: key.length === 1 ? key.toLowerCase() : key, shift: has("shift"), mod: has("mod"), alt: has("alt") };
}

/** The part of a key event a chord is matched against. */
export interface KeyLike {
  key: string;
  shiftKey: boolean;
  ctrlKey: boolean;
  metaKey: boolean;
  altKey: boolean;
}

/**
 * Whether `e` presses `chord`. A letter is matched whatever its case; a
 * symbol typed with Shift (`?`) is the symbol, Shift or not.
 */
export function matches(chord: Chord, e: KeyLike): boolean {
  const key = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (key !== chord.key) return false;
  const letter = /^[a-z0-9]$/.test(chord.key) || chord.key.length > 1;
  if (letter && e.shiftKey !== chord.shift) return false;
  return (e.ctrlKey || e.metaKey) === chord.mod && e.altKey === chord.alt;
}

/** Whether keys are shown as a Mac shows them. */
export function isMac(platform: string = typeof navigator === "undefined" ? "" : navigator.platform): boolean {
  return /mac|iphone|ipad/i.test(platform);
}

/** `text`, a shortcut, as the platform shows it: `⇧F` and `⌘K` on a Mac, `Shift+F` and `Ctrl+K` elsewhere. */
export function formatShortcut(text: string, mac: boolean = isMac()): string {
  const { key, shift, mod, alt } = parseChord(text);
  const name = key.length === 1 ? key.toUpperCase() : key === "Escape" ? "Esc" : key;
  if (mac) return `${mod ? "⌘" : ""}${alt ? "⌥" : ""}${shift ? "⇧" : ""}${name === "Enter" ? "↩" : name}`;
  return [mod && "Ctrl", alt && "Alt", shift && "Shift", name].filter(Boolean).join("+");
}

/** `title` with its shortcut after it, as a tooltip says it: `Extrude (E)`. */
export function withShortcut(title: string, shortcut: string | null | undefined): string {
  return shortcut ? `${title} (${formatShortcut(shortcut)})` : title;
}

/**
 * How well `query` matches `text`, for the palette to rank by: `null` if
 * its letters do not all appear in `text` in order. Higher is better: the
 * text starting with the query, then a word starting with it, then the
 * letters closest together, earliest.
 */
export function score(query: string, text: string): number | null {
  const q = query.trim().toLowerCase();
  const t = text.toLowerCase();
  if (q === "") return 0;
  if (t.startsWith(q)) return 1000 - t.length;
  const word = t.split(/[\s:_\-/·]+/).findIndex((w) => w.startsWith(q));
  if (word >= 0) return 800 - word * 10 - t.length;
  const at = t.indexOf(q);
  if (at >= 0) return 600 - at - t.length;
  // The letters in order, as close together as they come.
  let from = 0;
  let spread = 0;
  let first = -1;
  for (const c of q) {
    if (c === " ") continue;
    const i = t.indexOf(c, from);
    if (i < 0) return null;
    if (first < 0) first = i;
    else spread += i - from;
    from = i + 1;
  }
  return 300 - spread * 5 - first - t.length;
}

/** Something the palette offers: a command, what it is called, where it is grouped, and its keys. */
export interface PaletteEntry {
  id: string;
  label: string;
  /** What it is: `Operation`, `Step`, `Example`, `View`, `Edit`. */
  group: string;
  /** Said beside it, quieter: what an operation does, a step's summary. */
  detail?: string;
  shortcut?: string | null;
  disabled?: boolean;
  run: () => void;
}

/** `entries` matching `query`, best first; with no query, as given. Disabled ones last. */
export function search(entries: PaletteEntry[], query: string): PaletteEntry[] {
  const scored = entries
    .map((entry, i) => {
      // A group's name finds all of it, after what the query names itself.
      const group = query.trim() !== "" && entry.group.toLowerCase().startsWith(query.trim().toLowerCase());
      return { entry, i, s: score(query, entry.label) ?? (group ? -1000 : null) };
    })
    .filter((x): x is { entry: PaletteEntry; i: number; s: number } => x.s != null);
  scored.sort((a, b) => Number(!!a.entry.disabled) - Number(!!b.entry.disabled) || b.s - a.s || a.i - b.i);
  return scored.map((x) => x.entry);
}

/** The first of `entries` whose shortcut `e` presses, and is not disabled. */
export function pressed(entries: PaletteEntry[], e: KeyLike): PaletteEntry | undefined {
  return entries.find((entry) => !entry.disabled && entry.shortcut && matches(parseChord(entry.shortcut), e));
}
