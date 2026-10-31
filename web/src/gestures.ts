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

type V3 = [number, number, number];

/** How much nearer one notch of the wheel takes the view: as the desktop app zooms. */
export const ZOOM_PER_STEP = 0.85;

/**
 * How many notches a wheel event turns, positive in: a line (`deltaMode`
 * 1) or a hundred pixels (0, a notch in most browsers) or a quarter page
 * (2) each — a trackpad's pinch comes as small pixel deltas.
 */
export function wheelSteps(deltaY: number, deltaMode: number): number {
  const per = deltaMode === 1 ? 3 : deltaMode === 2 ? 0.25 : 100;
  return -deltaY / per;
}

/**
 * The camera at `position` looking at `target`, zoomed `steps` notches
 * (positive in) towards `under` — the point under the pointer, in the
 * plane through the target facing the eye — so that what is under the
 * pointer stays under it: the target moves towards it as the distance
 * shrinks, the direction of view kept. As the desktop app zooms
 * (`Camera::zoom`).
 */
export function zoomToward(position: V3, target: V3, under: V3, steps: number): { position: V3; target: V3 } {
  const factor = Math.pow(ZOOM_PER_STEP, steps);
  const back = position.map((p, k) => p - target[k]) as V3;
  const to = under.map((u, k) => u + (target[k] - u) * factor) as V3;
  return { position: to.map((t, k) => t + back[k] * factor) as V3, target: to };
}
