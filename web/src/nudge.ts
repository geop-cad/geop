// A number field moved without typing: by the arrow keys, a step at a time
// (Shift ten, Alt a tenth), or scrubbed by dragging its label, a step per
// few pixels. The desktop app moves its fields the same way
// (`cad/geop-cad-desktop/src/ui/dialog.rs`, `nudge`). Pure logic, tested by
// `npm test` (`test/nudge.test.ts`).

/** How many pixels of a drag along a field's label move it one step. */
export const SCRUB_PX = 4;

/** How many steps one press moves a number: ten with Shift, a tenth with Alt. */
export function stepsPerPress(mods: { shiftKey: boolean; altKey: boolean }): number {
  return mods.shiftKey ? 10 : mods.altKey ? 0.1 : 1;
}

/**
 * `text`, a field's number, moved by `steps` of `step`, rounded to the
 * digits that step is given in — `0.30000000000000004` reads `0.3` — or
 * `null` for a formula, which is followed, not moved.
 */
export function nudge(text: string, step: number, steps: number): string | null {
  const value = Number(text.trim());
  if (text.trim() === "" || !Number.isFinite(value) || !(step > 0)) return null;
  const by = step * steps;
  const digits = Math.min(12, Math.max(0, Math.ceil(-Math.log10(Math.abs(by)) - 1e-9)) + 1);
  return String(Number((value + by).toFixed(digits)));
}
