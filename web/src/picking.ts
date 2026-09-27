import { pickRay, type ArgKind, type Highlight, type PickFilter, type Vec3 } from "./geop";
import type { ViewRay } from "./SceneViewer";

/**
 * What a click along `ray` picks for an argument of `kind`: the nearest
 * of what the kernel hits and the datum the ray hits, if any.
 */
export function pickAt(kind: ArgKind, ray: ViewRay): { hit: Highlight; point: Vec3 } | null {
  const filter: PickFilter =
    kind.type === "solid" || kind.type === "combine"
      ? "solid"
      : kind.type === "sketch"
        ? "sketch"
        : kind.type === "selection"
          ? "any"
          : "face";
  // A sketch is hit inside its regions anyway; the tolerance is for
  // clicking on its curves, e.g. an open profile's — and on a vertex or
  // an edge, a few pixels.
  const tolerance = filter === "sketch" ? 0.03 : filter === "any" ? ray.pixel * 6 : 0.001;
  const found = pickRay(ray.origin, ray.dir, filter, tolerance);
  const datum = ray.datum;
  if (datum && (!found || datum.distance <= found.t)) return { hit: datum.entity, point: datum.point };
  if (!found) return null;
  const type = ({ vertex: "Vertex", edge: "Edge", face: "Face", solid: "Solid", sketch: "Sketch" } as const)[found.kind];
  return { hit: { type, name: found.name }, point: found.point };
}
