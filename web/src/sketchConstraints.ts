// Which constraints apply to a selection, how each constraint is labeled
// on the canvas, and structural edits: adding entities under fresh ids and
// deleting them. Ids are never reused, so an edit never renumbers what is
// left.

import { entries, type Constraint, type Id, type Sketch, type SketchCurve, type SketchPoint } from "./geop";
import { arcCircle, cross, curvePolyline, dist, dot, len, polylineMid, pt, sub, type P2 } from "./sketchGeometry";

export interface Selection {
  points: Id[];
  curves: Id[];
}

export const EMPTY_SELECTION: Selection = { points: [], curves: [] };

export interface ConstraintOption {
  label: string;
  title: string;
  make: () => Constraint;
}

const isRound = (c: SketchCurve) => c.type === "Arc" || c.type === "Circle";

function endpoints(c: SketchCurve): [number, number] | null {
  switch (c.type) {
    case "Line":
    case "Arc":
      return [c.start, c.end];
    case "Spline":
      return [c.control_points[0], c.control_points[c.control_points.length - 1]];
    case "Circle":
      return null;
  }
}

/** Union-find classes of points under `Coincident` constraints, like `Sketch::point_classes`. */
export function pointClasses(sketch: Sketch): Map<Id, Id> {
  const parent = new Map<Id, Id>(entries(sketch.points).map(([id]) => [id, id]));
  const find = (i: Id): Id => {
    const p = parent.get(i) ?? i;
    if (p === i) return i;
    const root = find(p);
    parent.set(i, root);
    return root;
  };
  for (const [, c] of entries(sketch.constraints)) {
    if (c.type === "Coincident") {
      const [a, b] = [find(c.a), find(c.b)];
      parent.set(Math.max(a, b), Math.min(a, b));
    }
  }
  return new Map(entries(sketch.points).map(([id]) => [id, find(id)]));
}

function shareEndpoint(sketch: Sketch, a: SketchCurve, b: SketchCurve): boolean {
  const ea = endpoints(a);
  const eb = endpoints(b);
  if (!ea || !eb) return false;
  const cls = pointClasses(sketch);
  return ea.some((p) => eb.some((q) => cls.get(p) === cls.get(q)));
}

export function lineLength(sketch: Sketch, c: SketchCurve): number {
  if (c.type !== "Line") return 0;
  return dist(pt(sketch, c.start), pt(sketch, c.end));
}

export function radiusOf(sketch: Sketch, c: SketchCurve): number {
  if (c.type === "Circle") return c.radius;
  if (c.type === "Arc") return arcCircle(pt(sketch, c.start), pt(sketch, c.end), c.sweep).radius;
  return 0;
}

function arcLength(sketch: Sketch, c: SketchCurve): number {
  if (c.type !== "Arc") return 0;
  const l = dist(pt(sketch, c.start), pt(sketch, c.end));
  const h = c.sweep / 2;
  return h === 0 ? l : (l * h) / Math.sin(h);
}

function direction(sketch: Sketch, c: SketchCurve): P2 {
  if (c.type !== "Line") return [1, 0];
  return sub(pt(sketch, c.end), pt(sketch, c.start));
}

/** Signed distance of `p` from the line through curve `c`. */
function lineDistance(sketch: Sketch, c: SketchCurve, p: P2): number {
  if (c.type !== "Line") return 0;
  const a = pt(sketch, c.start);
  const d = direction(sketch, c);
  return cross(d, sub(p, a)) / len(d);
}

/** Every constraint that fits the current selection, measuring dimensions from the current geometry. */
export function constraintOptions(sketch: Sketch, sel: Selection): ConstraintOption[] {
  const P = sel.points;
  const C = sel.curves;
  const curve = (i: Id) => sketch.curves[i];
  const opts: ConstraintOption[] = [];
  const add = (label: string, title: string, make: () => Constraint) => opts.push({ label, title, make });

  if (C.length === 0 && P.length === 1) {
    const p = sketch.points[P[0]];
    add("Fix", "Fix the point where it is", () => ({ type: "Fix", point: P[0], x: p.x, y: p.y }));
  }
  if (C.length === 0 && P.length === 2) {
    const [a, b] = P;
    const pa = pt(sketch, a);
    const pb = pt(sketch, b);
    add("Coincident", "Make the two points one", () => ({ type: "Coincident", a, b }));
    add("Horizontal", "Same y", () => ({ type: "DistanceY", a, b, value: 0 }));
    add("Vertical", "Same x", () => ({ type: "DistanceX", a, b, value: 0 }));
    add("Distance", "Distance between the points", () => ({ type: "Distance", a, b, value: dist(pa, pb) }));
    add("Δx", "Horizontal distance", () => ({ type: "DistanceX", a, b, value: pb[0] - pa[0] }));
    add("Δy", "Vertical distance", () => ({ type: "DistanceY", a, b, value: pb[1] - pa[1] }));
  }
  if (P.length === 0 && C.length === 1) {
    const i = C[0];
    const c = curve(i);
    if (c.type === "Line") {
      add("Horizontal", "Horizontal line", () => ({ type: "Horizontal", line: i }));
      add("Vertical", "Vertical line", () => ({ type: "Vertical", line: i }));
      add("Length", "Line length", () => ({ type: "Length", curve: i, value: lineLength(sketch, c) }));
    }
    if (isRound(c)) {
      add("Radius", "Radius", () => ({ type: "Radius", curve: i, value: radiusOf(sketch, c) }));
    }
    if (c.type === "Arc") {
      add("Arc length", "Length along the arc", () => ({ type: "Length", curve: i, value: arcLength(sketch, c) }));
    }
  }
  if (P.length === 0 && C.length === 2) {
    const [a, b] = C;
    const ca = curve(a);
    const cb = curve(b);
    if (ca.type === "Line" && cb.type === "Line") {
      add("Parallel", "Parallel lines", () => ({ type: "Parallel", a, b }));
      add("Perpendicular", "Perpendicular lines", () => ({ type: "Perpendicular", a, b }));
      add("Collinear", "On one line", () => ({ type: "Collinear", a, b }));
      add("Equal", "Equal length", () => ({ type: "Equal", a, b }));
      add("Angle", "Angle from the first line to the second", () => {
        const da = direction(sketch, ca);
        const db = direction(sketch, cb);
        return { type: "Angle", a, b, value: Math.atan2(cross(da, db), dot(da, db)) };
      });
    }
    const lineRound = (ca.type === "Line" && isRound(cb)) || (isRound(ca) && cb.type === "Line");
    const shared = shareEndpoint(sketch, ca, cb) && !(ca.type === "Line" && cb.type === "Line");
    if (shared || lineRound || (isRound(ca) && isRound(cb))) {
      add("Tangent", "Tangent curves", () => ({ type: "Tangent", a, b }));
    }
    if (isRound(ca) && isRound(cb)) {
      add("Concentric", "Same center", () => ({ type: "Concentric", a, b }));
      add("Equal", "Equal radius", () => ({ type: "Equal", a, b }));
    }
  }
  if (P.length === 1 && C.length === 1) {
    const [p] = P;
    const [i] = C;
    const c = curve(i);
    if (c.type !== "Spline") add("On curve", "Point lies on the curve", () => ({ type: "PointOnCurve", point: p, curve: i }));
    if (c.type === "Line" || c.type === "Arc") add("Midpoint", "Point is the curve's midpoint", () => ({ type: "Midpoint", point: p, curve: i }));
    if (c.type === "Line") {
      add("Distance", "Distance from the line", () => ({
        type: "PointLineDistance",
        point: p,
        line: i,
        value: Math.abs(lineDistance(sketch, c, pt(sketch, p))),
      }));
    }
  }
  if (P.length === 2 && C.length === 1 && curve(C[0]).type === "Line") {
    const [a, b] = P;
    add("Symmetric", "Mirror images across the line", () => ({ type: "Symmetric", a, b, line: C[0] }));
  }
  return opts;
}

/** The value of a dimensional constraint, if it has one. */
export function constraintValue(c: Constraint): number | null {
  return "value" in c ? c.value : null;
}

/** How a constraint is presented: its label and where it sits on the canvas. */
export function constraintGlyph(sketch: Sketch, c: Constraint): { text: string; at: P2 } | null {
  const mid = (i: Id) => polylineMid(curvePolyline(sketch, sketch.curves[i]));
  const avg = (a: P2, b: P2): P2 => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  const fmt = (v: number) => (Math.abs(v) >= 100 ? v.toFixed(1) : v.toFixed(2));
  switch (c.type) {
    case "Coincident":
      return null;
    case "PointOnCurve":
      return { text: "◦", at: pt(sketch, c.point) };
    case "Horizontal":
      return { text: "H", at: mid(c.line) };
    case "Vertical":
      return { text: "V", at: mid(c.line) };
    case "Parallel":
      return { text: "∥", at: mid(c.a) };
    case "Perpendicular":
      return { text: "⊥", at: mid(c.a) };
    case "Collinear":
      return { text: "≡", at: mid(c.a) };
    case "Tangent":
      return { text: "T", at: mid(c.a) };
    case "Equal":
      return { text: "=", at: mid(c.a) };
    case "Concentric":
      return { text: "◎", at: mid(c.a) };
    case "Midpoint":
      return { text: "M", at: pt(sketch, c.point) };
    case "Symmetric":
      return { text: "⇆", at: avg(pt(sketch, c.a), pt(sketch, c.b)) };
    case "Fix":
      return { text: "⚓", at: pt(sketch, c.point) };
    case "Distance":
      return { text: fmt(c.value), at: avg(pt(sketch, c.a), pt(sketch, c.b)) };
    case "DistanceX":
      return { text: c.value === 0 ? "|" : `Δx ${fmt(c.value)}`, at: avg(pt(sketch, c.a), pt(sketch, c.b)) };
    case "DistanceY":
      return { text: c.value === 0 ? "—" : `Δy ${fmt(c.value)}`, at: avg(pt(sketch, c.a), pt(sketch, c.b)) };
    case "PointLineDistance":
      return { text: fmt(c.value), at: pt(sketch, c.point) };
    case "Length":
      return { text: fmt(c.value), at: mid(c.curve) };
    case "Radius":
      return { text: `R${fmt(c.value)}`, at: mid(c.curve) };
    case "Angle":
      return { text: `${((c.value * 180) / Math.PI).toFixed(1)}°`, at: mid(c.b) };
  }
}

/** Human-readable name of a constraint, for the constraint list. */
export function constraintName(c: Constraint): string {
  switch (c.type) {
    case "Coincident":
      return `Coincident p${c.a}, p${c.b}`;
    case "PointOnCurve":
      return `p${c.point} on c${c.curve}`;
    case "Horizontal":
    case "Vertical":
      return `${c.type} c${c.line}`;
    case "Parallel":
    case "Perpendicular":
    case "Collinear":
    case "Tangent":
    case "Equal":
    case "Concentric":
      return `${c.type} c${c.a}, c${c.b}`;
    case "Midpoint":
      return `p${c.point} midpoint of c${c.curve}`;
    case "Symmetric":
      return `Symmetric p${c.a}, p${c.b} about c${c.line}`;
    case "Fix":
      return `Fix p${c.point}`;
    case "Distance":
      return `Distance p${c.a}–p${c.b}`;
    case "DistanceX":
      return `Δx p${c.a}–p${c.b}`;
    case "DistanceY":
      return `Δy p${c.a}–p${c.b}`;
    case "PointLineDistance":
      return `Distance p${c.point}–c${c.line}`;
    case "Length":
      return `Length c${c.curve}`;
    case "Radius":
      return `Radius c${c.curve}`;
    case "Angle":
      return `Angle c${c.a}→c${c.b} (°)`;
  }
}

export function constraintPoints(c: Constraint): Id[] {
  switch (c.type) {
    case "Coincident":
    case "Distance":
    case "DistanceX":
    case "DistanceY":
    case "Symmetric":
      return [c.a, c.b];
    case "PointOnCurve":
    case "Midpoint":
    case "Fix":
    case "PointLineDistance":
      return [c.point];
    default:
      return [];
  }
}

export function constraintCurves(c: Constraint): Id[] {
  switch (c.type) {
    case "PointOnCurve":
    case "Midpoint":
    case "Length":
    case "Radius":
      return [c.curve];
    case "Horizontal":
    case "Vertical":
    case "PointLineDistance":
    case "Symmetric":
      return [c.line];
    case "Parallel":
    case "Perpendicular":
    case "Collinear":
    case "Tangent":
    case "Equal":
    case "Concentric":
    case "Angle":
      return [c.a, c.b];
    default:
      return [];
  }
}

export function curvePoints(c: SketchCurve): Id[] {
  switch (c.type) {
    case "Line":
    case "Arc":
      return [c.start, c.end];
    case "Circle":
      return [c.center];
    case "Spline":
      return c.control_points;
  }
}

/** `record` without the entries whose id `keep` rejects. */
function filterRecord<T>(record: Record<Id, T>, keep: (id: Id) => boolean): Record<Id, T> {
  return Object.fromEntries(entries(record).filter(([id]) => keep(id)));
}

/** `sketch` with `entity` added to `kind` under a fresh id, and that id. */
function insert<K extends "points" | "curves" | "constraints">(
  sketch: Sketch,
  kind: K,
  entity: Sketch[K][Id],
): [Sketch, Id] {
  const id = sketch.next_id;
  return [{ ...sketch, [kind]: { ...sketch[kind], [id]: entity }, next_id: id + 1 }, id];
}

export const insertPoint = (sketch: Sketch, point: SketchPoint) => insert(sketch, "points", point);
export const insertCurve = (sketch: Sketch, curve: SketchCurve) => insert(sketch, "curves", curve);
export const insertConstraint = (sketch: Sketch, constraint: Constraint) => insert(sketch, "constraints", constraint);

/**
 * Delete the selected curves and points (and the given constraints), plus
 * everything that depends on them: curves on a deleted point, constraints on
 * any deleted entity, and points no longer used by anything. Everything
 * else keeps its id.
 */
export function deleteEntities(sketch: Sketch, sel: Selection, constraints: Id[] = []): Sketch {
  const selectedPoint = (p: Id) => sel.points.includes(p);
  const deadCurve = new Set(
    entries(sketch.curves)
      .filter(([i, c]) => sel.curves.includes(i) || curvePoints(c).some(selectedPoint))
      .map(([i]) => i),
  );
  const deadConstraint = new Set(
    entries(sketch.constraints)
      .filter(
        ([i, c]) =>
          constraints.includes(i) || constraintPoints(c).some(selectedPoint) || constraintCurves(c).some((k) => deadCurve.has(k)),
      )
      .map(([i]) => i),
  );
  // A point goes if selected, or if the deletion orphaned it: it was used
  // before and nothing surviving uses it now. Lone points drawn on purpose
  // stay.
  const usedBefore = new Set<Id>();
  const usedAfter = new Set<Id>();
  for (const [i, c] of entries(sketch.curves)) {
    for (const p of curvePoints(c)) {
      usedBefore.add(p);
      if (!deadCurve.has(i)) usedAfter.add(p);
    }
  }
  for (const [i, c] of entries(sketch.constraints)) {
    for (const p of constraintPoints(c)) {
      usedBefore.add(p);
      if (!deadConstraint.has(i)) usedAfter.add(p);
    }
  }
  const deadPoint = (p: Id) => selectedPoint(p) || (usedBefore.has(p) && !usedAfter.has(p));

  return {
    ...sketch,
    points: filterRecord(sketch.points, (i) => !deadPoint(i)),
    curves: filterRecord(sketch.curves, (i) => !deadCurve.has(i)),
    constraints: filterRecord(sketch.constraints, (i) => !deadConstraint.has(i)),
  };
}
