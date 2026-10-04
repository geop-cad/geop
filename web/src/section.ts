// A section view: a viewing aid that cuts the view along a plane, never the
// model (see `InspectPanel` and `SceneViewer`).

import type { Frame, Vec3 } from "./geop";

/** Which plane a section view cuts along: a world axis, or the plane of what the measure tool picked. */
export type SectionAxis = "x" | "y" | "z" | "picked";

/** A section view: a viewing aid, not a change of the model. */
export interface Section {
  axis: SectionAxis;
  /** Along the plane's normal, from the part's centre — or from the picked plane's origin. */
  offset: number;
  /** Which side is cut away. */
  flip: boolean;
  /** The picked plane, for `picked`. */
  plane: Frame | null;
}

/** The plane a section cuts along: through `origin`, cutting away what `normal` points to. */
export interface SectionPlane {
  origin: Vec3;
  normal: Vec3;
}

/** Where `section` cuts, around the part's centre `center`. */
export function sectionPlane(section: Section, center: Vec3): SectionPlane | null {
  const axes: Record<string, Vec3> = { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };
  const base = section.axis === "picked" ? section.plane : null;
  if (section.axis === "picked" && !base) return null;
  const normal0: Vec3 = base ? base.w : axes[section.axis];
  const normal = (section.flip ? normal0.map((x) => -x) : normal0) as Vec3;
  const from = base ? base.origin : center;
  const origin = [0, 1, 2].map((k) => from[k] + normal0[k] * section.offset) as Vec3;
  return { origin, normal };
}
