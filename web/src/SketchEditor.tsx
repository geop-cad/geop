import { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type ReactElement } from "react";
import { createPortal } from "react-dom";
import type { SketchView } from "./camera";
import {
  entries,
  solveSketch,
  type Constraint,
  type Id,
  type Sketch,
  type SketchCurve,
  type SolveResult,
} from "./geop";
import {
  EMPTY_SELECTION,
  constraintGlyph,
  constraintName,
  constraintOptions,
  constraintCurves,
  constraintPoints,
  constraintValue,
  deleteEntities,
  insertConstraint,
  insertCurve,
  insertPoint,
  type Selection,
} from "./sketchConstraints";
import {
  add,
  curvePolyline,
  dist,
  polylineDistance,
  pt,
  sub,
  sweepThrough,
  type P2,
} from "./sketchGeometry";

type Tool = "select" | "point" | "line" | "rectangle" | "arc" | "circle" | "spline";

const TOOLS: { tool: Tool; label: string; key: string }[] = [
  { tool: "select", label: "Select", key: "Escape" },
  { tool: "line", label: "Line", key: "l" },
  { tool: "rectangle", label: "Rectangle", key: "r" },
  { tool: "arc", label: "Arc", key: "a" },
  { tool: "circle", label: "Circle", key: "c" },
  { tool: "spline", label: "Spline", key: "s" },
  { tool: "point", label: "Point", key: "p" },
];

/** The drawing in progress: points already placed for the curve being drawn. */
type Draft =
  | { tool: "line"; start: Id }
  | { tool: "rectangle"; corner: Id }
  | { tool: "arc"; start: Id; end?: Id }
  | { tool: "circle"; center: Id }
  | { tool: "spline"; points: Id[] }
  | null;

/** A drag in progress, with the sketch as it was when it started. */
type Drag =
  | { kind: "points"; points: Id[]; origins: P2[]; grab: P2 }
  | { kind: "circle"; curve: Id }
  | { kind: "arc"; curve: Id }
  | { kind: "pan"; view: View; grab: P2 };

/** Viewport: `scale` pixels per sketch unit, `(cx, cy)` at the center. */
interface View {
  cx: number;
  cy: number;
  scale: number;
}

const SNAP_PX = 9;
/** Lines within this slope of horizontal/vertical get the constraint automatically. */
const AUTO_HV_SLOPE = Math.tan((2 * Math.PI) / 180);

const COLORS = {
  free: "#6cb4ff",
  constrained: "#e6e6e6",
  selected: "#ffd84a",
  failed: "#ff5d5d",
  construction: "#8a8a8a",
  draft: "#ffa040",
};

interface Props {
  initial: Sketch;
  /** Where to start looking: what the 3-D view shows, head-on to the plane, when the editor opens. */
  initialView: SketchView;
  /**
   * The editor is drawn over the 3-D view and drives its camera: every
   * pan and zoom is reported here, to be shown as a camera pose.
   */
  onView: (view: SketchView) => void;
  onFinish: (sketch: Sketch) => void;
  /** Back to the step's form, keeping the drawing — e.g. to put it on another plane. */
  onSetup: (sketch: Sketch) => void;
  onCancel: () => void;
  /**
   * Where to put the Draw/Constrain/Status/Constraints panel instead of
   * floating it over the canvas — the mobile layout's "Draw" tab pane.
   * `null`/absent: floats over the canvas, as on desktop.
   */
  panelHost?: HTMLElement | null;
}

function emptyResult(sketch: Sketch): SolveResult {
  return {
    sketch,
    report: {
      converged: true,
      max_residual: 0,
      iterations: 0,
      dof: 0,
      free_points: Object.fromEntries(entries(sketch.points).map(([id]) => [id, true])),
      free_curves: Object.fromEntries(entries(sketch.curves).map(([id]) => [id, true])),
      failed_constraints: [],
    },
    regions: [],
    regions_error: null,
  };
}

/** The 2-D constraint sketch editor: draw curves, constrain them, and hand back the solved sketch. */
export function SketchEditor({ initial, initialView, onView, onFinish, onSetup, onCancel, panelHost }: Props) {
  const [result, setResult] = useState<SolveResult>(() => {
    try {
      return solveSketch(initial);
    } catch {
      return emptyResult(initial);
    }
  });
  const [solveError, setSolveError] = useState<string | null>(null);
  const sketch = result.sketch;
  const [tool, setTool] = useState<Tool>("select");
  const [draft, setDraft] = useState<Draft>(null);
  const [selection, setSelection] = useState<Selection>(EMPTY_SELECTION);
  const [selectedConstraint, setSelectedConstraint] = useState<Id | null>(null);
  const [cursor, setCursor] = useState<P2>([0, 0]);
  /** The canvas's size, once measured: until then there is no view to report. */
  const [measured, setMeasured] = useState<{ w: number; h: number } | null>(null);
  const size = useMemo(() => measured ?? { w: 1, h: 1 }, [measured]);
  const [view, setView] = useState<View>({
    cx: initialView.center[0],
    cy: initialView.center[1],
    scale: initialView.scale,
  });
  const containerRef = useRef<HTMLDivElement>(null);
  const svgRef = useRef<SVGSVGElement>(null);
  const dragRef = useRef<Drag | null>(null);
  const downRef = useRef<{ x: number; y: number } | null>(null);
  const frameRequest = useRef<number | null>(null);

  useLayoutEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const measure = () => setMeasured({ w: el.clientWidth, h: el.clientHeight });
    measure();
    const ro = new ResizeObserver(measure);
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const onViewRef = useRef(onView);
  onViewRef.current = onView;
  useEffect(() => {
    if (measured) onViewRef.current({ center: [view.cx, view.cy], scale: view.scale, height: measured.h });
  }, [view, measured]);

  const toScreen = useCallback(
    (p: P2): P2 => [size.w / 2 + (p[0] - view.cx) * view.scale, size.h / 2 - (p[1] - view.cy) * view.scale],
    [size, view],
  );
  const toWorld = useCallback(
    (x: number, y: number): P2 => [view.cx + (x - size.w / 2) / view.scale, view.cy - (y - size.h / 2) / view.scale],
    [size, view],
  );

  /** Solve `next` and make it current; a sketch the solver rejects is kept as drawn, with the error shown. */
  const commit = useCallback((next: Sketch, drags: [number, number, number][] = []) => {
    try {
      setResult(solveSketch(next, drags));
      setSolveError(null);
    } catch (e) {
      setResult(emptyResult(next));
      setSolveError(String(e));
    }
  }, []);

  const tolerance = SNAP_PX / view.scale;

  const pointAt = (s: Sketch, p: P2): Id | null => {
    let best: Id | null = null;
    let bestD = tolerance;
    for (const [i, q] of entries(s.points)) {
      const d = dist([q.x, q.y], p);
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    }
    return best;
  };

  const curveAt = (s: Sketch, p: P2): Id | null => {
    let best: Id | null = null;
    let bestD = tolerance;
    for (const [i, c] of entries(s.curves)) {
      const d = polylineDistance(p, curvePolyline(s, c));
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    }
    return best;
  };

  /**
   * The point to use at `p`: an existing point there, or a new one — fixed
   * at the origin if placed on it, or constrained onto a curve it lands on.
   */
  const placePoint = (s: Sketch, p: P2): [Sketch, Id] => {
    const existing = pointAt(s, p);
    if (existing != null) return [s, existing];
    if (dist(p, [0, 0]) < tolerance) {
      const [next, id] = insertPoint(s, { x: 0, y: 0 });
      return [insertConstraint(next, { type: "Fix", point: id, x: 0, y: 0 })[0], id];
    }
    const [next, id] = insertPoint(s, { x: p[0], y: p[1] });
    const on = curveAt(s, p);
    if (on != null && s.curves[on].type !== "Spline") {
      return [insertConstraint(next, { type: "PointOnCurve", point: id, curve: on })[0], id];
    }
    return [next, id];
  };

  /** `s` with `curve` added, plus the constraints `extra` puts on it. */
  const addCurve = (s: Sketch, curve: SketchCurve, extra: (curve: Id) => Constraint[] = () => []): Sketch => {
    let [next, id] = insertCurve(s, curve);
    for (const c of extra(id)) [next] = insertConstraint(next, c);
    return next;
  };

  function handleClick(p: P2, shift: boolean) {
    switch (tool) {
      case "select": {
        const glyph = glyphAt(p);
        if (glyph != null) {
          setSelectedConstraint(glyph);
          setSelection(EMPTY_SELECTION);
          return;
        }
        setSelectedConstraint(null);
        const point = pointAt(sketch, p);
        const curve = point == null ? curveAt(sketch, p) : null;
        if (point != null || curve != null) {
          setSelection((sel) => {
            const toggle = (xs: Id[], x: Id) => (xs.includes(x) ? xs.filter((y) => y !== x) : [...xs, x]);
            return point != null ? { ...sel, points: toggle(sel.points, point) } : { ...sel, curves: toggle(sel.curves, curve!) };
          });
          return;
        }
        // Nothing there yet — but the origin is always selectable, to
        // make other entities coincident with it: place (or find) its
        // point, fixed there, the same way a drawing tool would.
        if (dist(p, [0, 0]) < tolerance) {
          const [next, id] = placePoint(sketch, p);
          commit(next);
          setSelection((sel) => ({ ...sel, points: shift ? [...sel.points, id] : [id] }));
          return;
        }
        if (!shift) setSelection(EMPTY_SELECTION);
        return;
      }
      case "point": {
        const [next] = placePoint(sketch, p);
        commit(next);
        return;
      }
      case "line": {
        const [next, index] = placePoint(sketch, p);
        if (!draft || draft.tool !== "line") {
          commit(next);
          setDraft({ tool: "line", start: index });
          return;
        }
        if (index === draft.start) return;
        const a = pt(next, draft.start);
        const b = pt(next, index);
        const extra = (line: Id): Constraint[] => {
          if (Math.abs(b[1] - a[1]) <= AUTO_HV_SLOPE * Math.abs(b[0] - a[0])) return [{ type: "Horizontal", line }];
          if (Math.abs(b[0] - a[0]) <= AUTO_HV_SLOPE * Math.abs(b[1] - a[1])) return [{ type: "Vertical", line }];
          return [];
        };
        commit(addCurve(next, { type: "Line", start: draft.start, end: index, construction: false }, extra));
        setDraft({ tool: "line", start: index });
        return;
      }
      case "rectangle": {
        if (!draft || draft.tool !== "rectangle") {
          const [next, index] = placePoint(sketch, p);
          commit(next);
          setDraft({ tool: "rectangle", corner: index });
          return;
        }
        const first = pt(sketch, draft.corner);
        if (Math.abs(p[0] - first[0]) < tolerance || Math.abs(p[1] - first[1]) < tolerance) return;
        // Two opposite corners, plus the two the rectangle implies. The
        // sides carry horizontal/vertical constraints, so it stays a
        // rectangle when anything is dragged later.
        let [next, opposite] = placePoint(sketch, p);
        let second: Id, fourth: Id;
        [next, second] = insertPoint(next, { x: p[0], y: first[1] });
        [next, fourth] = insertPoint(next, { x: first[0], y: p[1] });
        const corners = [draft.corner, second, opposite, fourth];
        for (let i = 0; i < 4; i++) {
          next = addCurve(
            next,
            { type: "Line", start: corners[i], end: corners[(i + 1) % 4], construction: false },
            (line) => [{ type: i % 2 === 0 ? "Horizontal" : "Vertical", line }],
          );
        }
        commit(next);
        setDraft(null);
        return;
      }
      case "arc": {
        if (!draft || draft.tool !== "arc") {
          const [next, index] = placePoint(sketch, p);
          commit(next);
          setDraft({ tool: "arc", start: index });
        } else if (draft.end == null) {
          const [next, index] = placePoint(sketch, p);
          if (index === draft.start) return;
          commit(next);
          setDraft({ ...draft, end: index });
        } else {
          const sweep = sweepThrough(pt(sketch, draft.start), pt(sketch, draft.end), p);
          commit(addCurve(sketch, { type: "Arc", start: draft.start, end: draft.end, sweep, construction: false }));
          setDraft(null);
        }
        return;
      }
      case "circle": {
        if (!draft || draft.tool !== "circle") {
          const [next, index] = placePoint(sketch, p);
          commit(next);
          setDraft({ tool: "circle", center: index });
        } else {
          const radius = dist(pt(sketch, draft.center), p);
          if (radius > 0) commit(addCurve(sketch, { type: "Circle", center: draft.center, radius, construction: false }));
          setDraft(null);
        }
        return;
      }
      case "spline": {
        const [next, index] = placePoint(sketch, p);
        commit(next);
        const points = draft && draft.tool === "spline" ? draft.points : [];
        if (points.length > 0 && points[points.length - 1] === index) return;
        setDraft({ tool: "spline", points: [...points, index] });
        return;
      }
    }
  }

  /** Finish a multi-click curve (spline) or end a line chain. */
  function finishDraft() {
    if (draft?.tool === "spline" && draft.points.length >= 2) {
      commit(addCurve(sketch, { type: "Spline", control_points: draft.points, construction: false }));
    }
    setDraft(null);
  }

  /** Every constraint's glyph (or `null`), with the constraint's id, in id order. */
  const glyphs = useMemo(
    () => entries(sketch.constraints).map(([id, c]) => ({ id, glyph: constraintGlyph(sketch, c) })),
    [sketch],
  );

  /** The id of the constraint whose glyph is at `p`, if any. */
  function glyphAt(p: P2): Id | null {
    const s = toScreen(p);
    let found: Id | null = null;
    glyphs.forEach(({ id }, k) => {
      const q = glyphScreen(k);
      if (q && Math.abs(q[0] - s[0]) < 12 && Math.abs(q[1] - s[1]) < 9) found = id;
    });
    return found;
  }

  /** Where the `k`-th glyph is drawn: offset from its anchor so labels on one entity do not stack. */
  function glyphScreen(k: number): P2 | null {
    const g = glyphs[k].glyph;
    if (!g) return null;
    const s = toScreen(g.at);
    const stack = glyphs.slice(0, k).filter(({ glyph: h }) => h && dist(h.at, g.at) < 1e-9).length;
    return [s[0] + 14 + stack * 22, s[1] - 12];
  }

  // ── pointer handling ─────────────────────────────────────────────────────

  function localXY(e: React.PointerEvent | React.WheelEvent): [number, number] {
    const rect = svgRef.current!.getBoundingClientRect();
    return [e.clientX - rect.left, e.clientY - rect.top];
  }

  function onPointerDown(e: React.PointerEvent) {
    const [x, y] = localXY(e);
    const p = toWorld(x, y);
    downRef.current = { x, y };
    (e.target as Element).setPointerCapture?.(e.pointerId);
    if (e.button === 1 || e.button === 2) {
      dragRef.current = { kind: "pan", view, grab: [x, y] };
      return;
    }
    if (tool !== "select") return;
    const point = pointAt(sketch, p);
    if (point != null) {
      dragRef.current = { kind: "points", points: [point], origins: [pt(sketch, point)], grab: p };
      return;
    }
    const curve = curveAt(sketch, p);
    if (curve != null) {
      const c = sketch.curves[curve];
      if (c.type === "Circle") dragRef.current = { kind: "circle", curve };
      else if (c.type === "Arc") dragRef.current = { kind: "arc", curve };
      else {
        const points = c.type === "Line" ? [c.start, c.end] : c.control_points;
        dragRef.current = { kind: "points", points, origins: points.map((i) => pt(sketch, i)), grab: p };
      }
      return;
    }
    dragRef.current = { kind: "pan", view, grab: [x, y] };
  }

  const latest = useRef<{ p: P2; screen: [number, number] }>({ p: [0, 0], screen: [0, 0] });
  const sketchRef = useRef(sketch);
  sketchRef.current = sketch;

  function applyDrag() {
    frameRequest.current = null;
    const drag = dragRef.current;
    if (!drag) return;
    const { p, screen } = latest.current;
    const s = sketchRef.current;
    switch (drag.kind) {
      case "pan":
        setView({
          ...drag.view,
          cx: drag.view.cx - (screen[0] - drag.grab[0]) / drag.view.scale,
          cy: drag.view.cy + (screen[1] - drag.grab[1]) / drag.view.scale,
        });
        return;
      case "points": {
        const delta = sub(p, drag.grab);
        commit(
          s,
          drag.points.map((i, k) => {
            const target = add(drag.origins[k], delta);
            return [i, target[0], target[1]];
          }),
        );
        return;
      }
      case "circle": {
        const c = s.curves[drag.curve];
        if (c.type !== "Circle") return;
        const radius = dist(pt(s, c.center), p);
        commit({ ...s, curves: { ...s.curves, [drag.curve]: { ...c, radius } } });
        return;
      }
      case "arc": {
        const c = s.curves[drag.curve];
        if (c.type !== "Arc") return;
        const sweep = sweepThrough(pt(s, c.start), pt(s, c.end), p);
        if (Number.isFinite(sweep)) commit({ ...s, curves: { ...s.curves, [drag.curve]: { ...c, sweep } } });
        return;
      }
    }
  }

  function onPointerMove(e: React.PointerEvent) {
    const [x, y] = localXY(e);
    const p = toWorld(x, y);
    setCursor(p);
    latest.current = { p, screen: [x, y] };
    const down = downRef.current;
    if (!dragRef.current || !down || Math.hypot(x - down.x, y - down.y) < 4) return;
    if (frameRequest.current == null) frameRequest.current = requestAnimationFrame(applyDrag);
  }

  function onPointerUp(e: React.PointerEvent) {
    const [x, y] = localXY(e);
    const down = downRef.current;
    const wasClick = down && Math.hypot(x - down.x, y - down.y) < 4;
    downRef.current = null;
    dragRef.current = null;
    if (frameRequest.current != null) {
      cancelAnimationFrame(frameRequest.current);
      frameRequest.current = null;
    }
    if (!wasClick) return;
    if (e.button === 2) {
      finishDraft();
      return;
    }
    if (e.button === 0) handleClick(toWorld(x, y), e.shiftKey);
  }

  function onWheel(e: React.WheelEvent) {
    const [x, y] = localXY(e);
    const before = toWorld(x, y);
    const scale = Math.min(1e5, Math.max(1e-3, view.scale * Math.exp(-e.deltaY * 0.0015)));
    // Keep the point under the cursor fixed.
    setView({
      scale,
      cx: before[0] - (x - size.w / 2) / scale,
      cy: before[1] + (y - size.h / 2) / scale,
    });
  }

  // ── editing ──────────────────────────────────────────────────────────────

  function deleteSelection() {
    if (selectedConstraint != null) {
      commit(deleteEntities(sketch, EMPTY_SELECTION, [selectedConstraint]));
      setSelectedConstraint(null);
      return;
    }
    if (selection.points.length === 0 && selection.curves.length === 0) return;
    commit(deleteEntities(sketch, selection));
    setSelection(EMPTY_SELECTION);
  }

  function toggleConstruction() {
    const construction = !selection.curves.every((i) => sketch.curves[i].construction);
    const curves = { ...sketch.curves };
    for (const i of selection.curves) curves[i] = { ...curves[i], construction };
    commit({ ...sketch, curves });
  }

  function addConstraint(c: Constraint) {
    commit(insertConstraint(sketch, c)[0]);
    setSelection(EMPTY_SELECTION);
  }

  function setConstraintValue(id: Id, value: number) {
    const c = sketch.constraints[id];
    if (!("value" in c)) return;
    commit({ ...sketch, constraints: { ...sketch.constraints, [id]: { ...c, value } } });
  }

  function deleteConstraint(id: Id) {
    commit(deleteEntities(sketch, EMPTY_SELECTION, [id]));
    if (selectedConstraint === id) setSelectedConstraint(null);
  }

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.target instanceof HTMLInputElement) return;
      if (e.key === "Escape") {
        if (draft) finishDraft();
        else {
          setSelection(EMPTY_SELECTION);
          setSelectedConstraint(null);
          setTool("select");
        }
      } else if (e.key === "Enter") finishDraft();
      else if (e.key === "Delete" || e.key === "Backspace") deleteSelection();
      else {
        const t = TOOLS.find((t) => t.key === e.key.toLowerCase());
        if (t) {
          finishDraft();
          setTool(t.tool);
        }
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  });

  // ── rendering ────────────────────────────────────────────────────────────

  const report = result.report;
  const failed = report.failed_constraints.map((i) => sketch.constraints[i]).filter((c) => c != null);
  const curveList = entries(sketch.curves);
  const constraintList = entries(sketch.constraints);
  const failedCurves = new Set(failed.flatMap(constraintCurves));
  const failedPoints = new Set(failed.flatMap(constraintPoints));

  const polyPath = (poly: P2[]) =>
    poly.map((q, i) => {
      const s = toScreen(q);
      return `${i === 0 ? "M" : "L"}${s[0].toFixed(1)},${s[1].toFixed(1)}`;
    }).join("");

  const curveColor = (i: Id) => {
    if (selection.curves.includes(i)) return COLORS.selected;
    if (failedCurves.has(i)) return COLORS.failed;
    if (sketch.curves[i].construction) return COLORS.construction;
    return report.free_curves[i] ?? true ? COLORS.free : COLORS.constrained;
  };
  const pointColor = (i: Id) => {
    if (selection.points.includes(i)) return COLORS.selected;
    if (failedPoints.has(i)) return COLORS.failed;
    return report.free_points[i] ?? true ? COLORS.free : COLORS.constrained;
  };

  // Grid spacing: a power of ten giving 20–200 px cells.
  const gridStep = Math.pow(10, Math.ceil(Math.log10(20 / view.scale)));
  const grid: ReactElement[] = [];
  {
    const [wx0, wy1] = toWorld(0, 0);
    const [wx1, wy0] = toWorld(size.w, size.h);
    for (let gx = Math.ceil(wx0 / gridStep) * gridStep; gx <= wx1; gx += gridStep) {
      const [sx] = toScreen([gx, 0]);
      const major = Math.abs(Math.round(gx / gridStep) % 10) === 0;
      grid.push(<line key={`gx${gx}`} x1={sx} y1={0} x2={sx} y2={size.h} className={major ? "grid-major" : "grid-minor"} />);
    }
    for (let gy = Math.ceil(wy0 / gridStep) * gridStep; gy <= wy1; gy += gridStep) {
      const [, sy] = toScreen([0, gy]);
      const major = Math.abs(Math.round(gy / gridStep) % 10) === 0;
      grid.push(<line key={`gy${gy}`} x1={0} y1={sy} x2={size.w} y2={sy} className={major ? "grid-major" : "grid-minor"} />);
    }
  }
  const origin = toScreen([0, 0]);

  // Preview of the curve being drawn, to the cursor.
  let draftPath: string | null = null;
  if (draft?.tool === "line") draftPath = polyPath([pt(sketch, draft.start), cursor]);
  if (draft?.tool === "arc") {
    const s = pt(sketch, draft.start);
    if (draft.end == null) draftPath = polyPath([s, cursor]);
    else {
      const e = pt(sketch, draft.end);
      const sweep = sweepThrough(s, e, cursor);
      if (Number.isFinite(sweep)) {
        draftPath = polyPath(curvePolyline(sketch, { type: "Arc", start: draft.start, end: draft.end, sweep, construction: false }));
      }
    }
  }
  if (draft?.tool === "rectangle") {
    const [x0, y0] = pt(sketch, draft.corner);
    draftPath = polyPath([[x0, y0], [cursor[0], y0], cursor, [x0, cursor[1]], [x0, y0]]);
  }
  if (draft?.tool === "circle") {
    const r = dist(pt(sketch, draft.center), cursor);
    const c = toScreen(pt(sketch, draft.center));
    draftPath = `M${c[0] + r * view.scale},${c[1]}a${r * view.scale},${r * view.scale} 0 1,0 ${-2 * r * view.scale},0a${r * view.scale},${r * view.scale} 0 1,0 ${2 * r * view.scale},0`;
  }
  if (draft?.tool === "spline" && draft.points.length > 0) {
    const cps = [...draft.points.map((i) => pt(sketch, i)), cursor];
    const temp: Sketch = { ...sketch, points: Object.fromEntries(cps.map(([x, y], i) => [i, { x, y }])) };
    draftPath = polyPath(curvePolyline(temp, { type: "Spline", control_points: cps.map((_, i) => i), construction: false }));
  }

  const options = constraintOptions(sketch, selection);
  const nCurves = curveList.filter(([, c]) => !c.construction).length;

  const status = solveError
    ? solveError
    : !report.converged
      ? `Over-constrained or conflicting: ${report.failed_constraints.length} constraint(s) cannot be met`
      : report.dof === 0
        ? "Fully constrained"
        : `${report.dof} degree${report.dof === 1 ? "" : "s"} of freedom`;

  const panel = (
    <aside className="sketch-panel">
      <div className="button-row">
        <button className="small" onClick={onCancel}>
          Cancel
        </button>
        <button className="primary" onClick={() => onFinish(sketch)} disabled={curveList.length === 0}>
          Finish sketch
        </button>
      </div>

      <section className="panel">
        <h2>Draw</h2>
        <div className="button-grid">
          {TOOLS.map((t) => (
            <button
              key={t.tool}
              className={tool === t.tool ? "active" : ""}
              onClick={() => {
                finishDraft();
                setTool(t.tool);
              }}
              title={`Shortcut: ${t.key}`}
            >
              {t.label}
            </button>
          ))}
        </div>
      </section>

      <section className="panel">
        <h2>Constrain</h2>
        {options.length === 0 && selection.curves.length === 0 && (
          <p className="hint">Select points and curves (Select tool) to see the constraints that apply.</p>
        )}
        <div className="button-grid">
          {options.map((o) => (
            <button key={o.label} title={o.title} onClick={() => addConstraint(o.make())}>
              {o.label}
            </button>
          ))}
        </div>
        {(selection.curves.length > 0 || selection.points.length > 0 || selectedConstraint != null) && (
          <div className="button-grid">
            {selection.curves.length > 0 && <button onClick={toggleConstruction}>Construction</button>}
            <button onClick={deleteSelection}>Delete</button>
          </div>
        )}
      </section>

      <section className="panel">
        <h2>Status</h2>
        <p className={report.converged && !solveError ? (report.dof === 0 ? "status-ok" : "hint") : "status-bad"}>{status}</p>
        <p className="hint">
          {nCurves} curve{nCurves === 1 ? "" : "s"} ·{" "}
          {result.regions_error ? result.regions_error : `${result.regions.length} closed region${result.regions.length === 1 ? "" : "s"}`}
        </p>
      </section>

      <section className="panel constraint-list">
        <h2>Constraints</h2>
        {constraintList.length === 0 && <p className="hint">None yet.</p>}
        <ol>
          {constraintList.map(([i, c]) => {
            const value = constraintValue(c);
            const isAngle = c.type === "Angle";
            const shown = value == null ? "" : String(Number((isAngle ? (value * 180) / Math.PI : value).toFixed(6)));
            return (
              <li
                key={i}
                className={[
                  report.failed_constraints.includes(i) ? "failed" : "",
                  selectedConstraint === i ? "selected" : "",
                ].join(" ")}
                onClick={() => {
                  setSelectedConstraint(i);
                  setSelection(EMPTY_SELECTION);
                }}
              >
                <span className="constraint-name">{constraintName(c)}</span>
                {value != null && (
                  // Typing only edits the field; Enter applies it, and
                  // leaving the field without Enter puts the value back.
                  // Keyed by the value, so a solve that changes it shows.
                  <input
                    key={shown}
                    type="number"
                    step={isAngle ? 1 : 0.1}
                    defaultValue={shown}
                    title="Enter applies"
                    onKeyDown={(e) => {
                      if (e.key === "Escape") e.currentTarget.blur();
                      if (e.key !== "Enter") return;
                      const v = Number(e.currentTarget.value);
                      if (e.currentTarget.value !== "" && !Number.isNaN(v)) setConstraintValue(i, isAngle ? (v * Math.PI) / 180 : v);
                    }}
                    onBlur={(e) => (e.currentTarget.value = shown)}
                  />
                )}
                <button
                  className="constraint-remove"
                  title="Delete this constraint"
                  onClick={(e) => {
                    e.stopPropagation();
                    deleteConstraint(i);
                  }}
                >
                  ✕
                </button>
              </li>
            );
          })}
        </ol>
      </section>

      <button onClick={() => onSetup(sketch)} title="Back to the step's form, keeping the drawing — e.g. to choose another plane">
        Sketch setup…
      </button>
    </aside>
  );

  return (
    <div className="sketch-editor">
      <div className="sketch-canvas" ref={containerRef}>
        <svg
          ref={svgRef}
          width={size.w}
          height={size.h}
          onPointerDown={onPointerDown}
          onPointerMove={onPointerMove}
          onPointerUp={onPointerUp}
          onWheel={onWheel}
          onDoubleClick={finishDraft}
          onContextMenu={(e) => e.preventDefault()}
        >
          {/* See-through: the model is the 3-D view underneath, seen head-on. */}
          <rect width={size.w} height={size.h} className="sketch-bg" />
          {grid}
          <line x1={0} y1={origin[1]} x2={size.w} y2={origin[1]} className="axis-x" />
          <line x1={origin[0]} y1={0} x2={origin[0]} y2={size.h} className="axis-y" />
          {result.regions.map((region, i) => (
            <path key={`g${i}`} d={region.map((l) => polyPath(l) + "Z").join("")} className="sketch-region" fillRule="evenodd" />
          ))}
          {curveList.map(([i, c]) => (
            <path
              key={`c${i}`}
              d={polyPath(curvePolyline(sketch, c))}
              stroke={curveColor(i)}
              strokeWidth={selection.curves.includes(i) ? 3 : 2}
              strokeDasharray={c.construction ? "6 4" : undefined}
              fill="none"
            />
          ))}
          {curveList.map(([i, c]) =>
            c.type === "Spline" ? (
              <path key={`h${i}`} d={polyPath(c.control_points.map((k) => pt(sketch, k)))} className="spline-hull" />
            ) : null,
          )}
          {draftPath && <path d={draftPath} stroke={COLORS.draft} strokeWidth={1.5} strokeDasharray="4 3" fill="none" />}
          {entries(sketch.points).map(([i, p]) => {
            const s = toScreen([p.x, p.y]);
            return <circle key={`p${i}`} cx={s[0]} cy={s[1]} r={selection.points.includes(i) ? 5 : 3.5} fill={pointColor(i)} />;
          })}
          {glyphs.map(({ id, glyph: g }, k) => {
            if (!g) return null;
            const s = glyphScreen(k)!;
            const failed = report.failed_constraints.includes(id);
            const selected = selectedConstraint === id;
            return (
              <g key={`k${id}`} transform={`translate(${s[0]},${s[1]})`} className="glyph">
                <rect
                  x={-4 - g.text.length * 3.3}
                  y={-9}
                  width={8 + g.text.length * 6.6}
                  height={16}
                  rx={3}
                  className={selected ? "glyph-bg selected" : failed ? "glyph-bg failed" : "glyph-bg"}
                />
                <text textAnchor="middle" y={3}>
                  {g.text}
                </text>
              </g>
            );
          })}
        </svg>
        <div className="sketch-coords">
          x {cursor[0].toFixed(3)} · y {cursor[1].toFixed(3)}
        </div>
        <div className="sketch-hint">
          {tool === "line" && (draft ? "Click the next point · right-click / Esc ends the chain" : "Click the start point")}
          {tool === "arc" && (!draft ? "Click the start point" : draft.tool === "arc" && draft.end == null ? "Click the end point" : "Click a point the arc passes through")}
          {tool === "rectangle" && (!draft ? "Click one corner" : "Click the opposite corner")}
          {tool === "circle" && (!draft ? "Click the center" : "Click a point on the circle")}
          {tool === "spline" && "Click control points · double-click / Enter / right-click finishes"}
          {tool === "point" && "Click to place a point"}
          {tool === "select" && "Click to select (adds to the selection) · drag points or curves · drag empty space to pan"}
        </div>
      </div>

      {panelHost ? createPortal(panel, panelHost) : panel}
    </div>
  );
}
