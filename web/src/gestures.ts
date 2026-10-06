/** How far, in pixels, a press may move and still be a click. */
export const CLICK_PX = 4;
/** How soon, in milliseconds, a second click makes a double click. */
export const DOUBLE_MS = 350;

interface Point {
  x: number;
  y: number;
}

/** Whether the pointer, now at `to`, has moved too far from where it went down at `from` for the press to be a click. */
export function movedFromPress(from: Point, to: Point): boolean {
  return Math.hypot(to.x - from.x, to.y - from.y) > CLICK_PX;
}

/** Whether a click at `at`, `now` (in milliseconds), is the second of a double click, the first being `last`, if any. */
export function isDoubleClick(last: (Point & { time: number }) | null, at: Point, now: number): boolean {
  return last != null && now - last.time < DOUBLE_MS && !movedFromPress(last, at);
}
