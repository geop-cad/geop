// Plain-number geometry of sketch entities, for drawing and hit-testing in
// the sketch editor. Mirrors the conventions of `geop_core_sketch`
// (`geometry.rs`, `profile.rs`): an arc turns counter-clockwise by `sweep`
// from `start` to `end`; a spline is a clamped uniform B-spline of degree
// `min(3, n - 1)`.

import type { Frame, Sketch, SketchCurve, Vec3 } from "./geop";

export type P2 = [number, number];

export const sub = (a: P2, b: P2): P2 => [a[0] - b[0], a[1] - b[1]];
export const add = (a: P2, b: P2): P2 => [a[0] + b[0], a[1] + b[1]];
export const scale = (a: P2, s: number): P2 => [a[0] * s, a[1] * s];
export const dot = (a: P2, b: P2) => a[0] * b[0] + a[1] * b[1];
export const cross = (a: P2, b: P2) => a[0] * b[1] - a[1] * b[0];
export const len = (a: P2) => Math.hypot(a[0], a[1]);
export const dist = (a: P2, b: P2) => len(sub(a, b));

export function pt(sketch: Sketch, i: number): P2 {
  const p = sketch.points[i];
  return [p.x, p.y];
}

/** Center and (positive) radius of the arc from `s` to `e` turning by `sweep`. Infinite for `sweep = 0`. */
export function arcCircle(s: P2, e: P2, sweep: number): { center: P2; radius: number } {
  const chord = sub(e, s);
  const l = len(chord);
  const half = sweep / 2;
  const left: P2 = [-chord[1] / l, chord[0] / l];
  const d = (l / 2) * (Math.cos(half) / Math.sin(half));
  return { center: add(scale(add(s, e), 0.5), scale(left, d)), radius: l / (2 * Math.abs(Math.sin(half))) };
}

/** The signed sweep of the arc from `s` to `e` passing through `p` (inscribed angle theorem). */
export function sweepThrough(s: P2, e: P2, p: P2): number {
  const a = sub(s, p);
  const b = sub(e, p);
  const phi = Math.abs(Math.atan2(cross(a, b), dot(a, b)));
  // `p` right of the chord: a counter-clockwise arc bulges that way.
  const sign = cross(sub(e, s), sub(p, s)) < 0 ? 1 : -1;
  return sign * (2 * Math.PI - 2 * phi);
}

function arcPolyline(s: P2, e: P2, sweep: number): P2[] {
  if (sweep === 0 || !Number.isFinite(sweep)) return [s, e];
  const { center, radius } = arcCircle(s, e, sweep);
  const a0 = Math.atan2(s[1] - center[1], s[0] - center[0]);
  const n = Math.max(8, Math.ceil((Math.abs(sweep) / Math.PI) * 48));
  const out: P2[] = [];
  for (let i = 0; i <= n; i++) {
    const a = a0 + (sweep * i) / n;
    out.push([center[0] + radius * Math.cos(a), center[1] + radius * Math.sin(a)]);
  }
  out[0] = s;
  out[n] = e;
  return out;
}

function circlePolyline(c: P2, r: number): P2[] {
  const out: P2[] = [];
  for (let i = 0; i <= 96; i++) {
    const a = (2 * Math.PI * i) / 96;
    out.push([c[0] + r * Math.cos(a), c[1] + r * Math.sin(a)]);
  }
  return out;
}

export function splinePoint(cps: P2[], t: number): P2 {
  const n = cps.length;
  const p = Math.min(3, n - 1);
  const spans = n - p;
  const knots: number[] = [];
  for (let i = 0; i <= p; i++) knots.push(0);
  for (let i = 1; i < spans; i++) knots.push(i / spans);
  for (let i = 0; i <= p; i++) knots.push(1);
  let k = p;
  for (let j = n - 1; j >= p; j--) {
    if (knots[j] <= t) {
      k = j;
      break;
    }
  }
  const d: P2[] = [];
  for (let j = 0; j <= p; j++) d.push(cps[j + k - p]);
  for (let r = 1; r <= p; r++) {
    for (let j = p; j >= r; j--) {
      const i = j + k - p;
      const denom = knots[i + p + 1 - r] - knots[i];
      const alpha = denom === 0 ? 0 : (t - knots[i]) / denom;
      d[j] = [(1 - alpha) * d[j - 1][0] + alpha * d[j][0], (1 - alpha) * d[j - 1][1] + alpha * d[j][1]];
    }
  }
  return d[p];
}

function splinePolyline(cps: P2[]): P2[] {
  if (cps.length < 2) return cps;
  const n = 24 * cps.length;
  const out: P2[] = [];
  for (let i = 0; i <= n; i++) out.push(splinePoint(cps, i / n));
  return out;
}

export function curvePolyline(sketch: Sketch, curve: SketchCurve): P2[] {
  switch (curve.type) {
    case "Line":
      return [pt(sketch, curve.start), pt(sketch, curve.end)];
    case "Arc":
      return arcPolyline(pt(sketch, curve.start), pt(sketch, curve.end), curve.sweep);
    case "Circle":
      return circlePolyline(pt(sketch, curve.center), curve.radius);
    case "Spline":
      return splinePolyline(curve.control_points.map((i) => pt(sketch, i)));
  }
}

/** Distance from `p` to the segment `a..b`. */
export function segmentDistance(p: P2, a: P2, b: P2): number {
  const ab = sub(b, a);
  const l2 = dot(ab, ab);
  const t = l2 === 0 ? 0 : Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2));
  return dist(p, add(a, scale(ab, t)));
}

export function polylineDistance(p: P2, poly: P2[]): number {
  let best = Infinity;
  for (let i = 0; i + 1 < poly.length; i++) best = Math.min(best, segmentDistance(p, poly[i], poly[i + 1]));
  return best;
}

/** The point halfway along a polyline, by arc length — where a curve's label goes. */
export function polylineMid(poly: P2[]): P2 {
  let total = 0;
  for (let i = 0; i + 1 < poly.length; i++) total += dist(poly[i], poly[i + 1]);
  let acc = 0;
  for (let i = 0; i + 1 < poly.length; i++) {
    const d = dist(poly[i], poly[i + 1]);
    if (acc + d >= total / 2 && d > 0) return add(poly[i], scale(sub(poly[i + 1], poly[i]), (total / 2 - acc) / d));
    acc += d;
  }
  return poly[0];
}

/** Project a world point into sketch coordinates `(x, y)` and its height above the plane. */
export function toSketch(frame: Frame, p: Vec3): { xy: P2; h: number } {
  const d: Vec3 = [p[0] - frame.origin[0], p[1] - frame.origin[1], p[2] - frame.origin[2]];
  const d3 = (a: Vec3) => d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
  return { xy: [d3(frame.u), d3(frame.v)], h: d3(frame.normal) };
}
