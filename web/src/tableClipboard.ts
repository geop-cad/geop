// What a table field makes of the clipboard: a block of cells, as a
// spreadsheet puts it there — tab-separated columns, one row per line.

/**
 * `text` as rows of cells: columns split at tabs, rows at line ends, as a
 * spreadsheet copies them; a trailing line end closes the last row rather
 * than opening an empty one. `null` for text that is a single cell, which
 * is typed into the cell as any text is, not pasted as a block.
 */
export function parseBlock(text: string): string[][] | null {
  const lines = text.replace(/\r\n?/g, "\n").replace(/\n$/, "").split("\n");
  const cells = lines.map((line) => line.split("\t"));
  if (cells.length === 1 && cells[0].length === 1) return null;
  return cells;
}
