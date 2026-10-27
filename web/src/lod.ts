// How much of each placed part to draw: a part that covers only a few
// pixels on screen — a screw of a robot seen whole — is drawn as its box,
// not its triangles. Its box is its component's extent, where it is: a
// placed part drawn as a box is drawn where it is and as big, and drawn in
// full again once it is near enough to show more than a few pixels.
//
// A part lit or with something of it hidden is always drawn in full (see
// `placed3d.ts`): what is being looked at is never a box.

import type { Frame, PartView, Vec3, ViewInstance } from "./geop";

/** How many pixels across a placed part's box must cover on screen for it to be drawn in full. */
export const BOX_BELOW_PX = 6;

/** The point `p` of a part placed in `frame`, where the part drawn has it. */
export function placedAt(frame: Frame, p: Vec3): Vec3 {
  const { origin, u, v, w } = frame;
  return [0, 1, 2].map((k) => origin[k] + u[k] * p[0] + v[k] * p[1] + w[k] * p[2]) as Vec3;
}

/**
 * The names of the placed parts of `instances` drawn as boxes: those whose
 * component's box, of `components`, spans fewer than [[BOX_BELOW_PX]]
 * pixels where it is — `pixel(at)` saying what one pixel measures in the
 * world at the point `at`.
 */
export function boxed(
  instances: ViewInstance[],
  components: Record<string, PartView>,
  pixel: (at: Vec3) => number,
): Set<string> {
  const boxes = new Set<string>();
  for (const instance of instances) {
    const view = components[instance.component];
    if (!view) continue;
    const at = placedAt(instance.frame, view.extent.center);
    if (view.extent.size < BOX_BELOW_PX * pixel(at)) boxes.add(instance.name);
  }
  return boxes;
}

/** Whether two sets of names hold the same names. */
export function sameNames(a: Set<string>, b: Set<string>): boolean {
  return a.size === b.size && [...a].every((name) => b.has(name));
}
