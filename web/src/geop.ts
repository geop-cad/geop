// Thin, typed wrapper around the generated `wasm-bindgen` bindings — the
// rest of the app imports from here instead of `./wasm/pkg/geop.js`
// directly, so the wasm-loading/init dance and JSON (de)serialization stay
// in one place.
//
// The app edits a part program (`geop_ops_parts::Program`) that lives in
// the wasm module: it never changes the program itself, but sends each
// change as a `ProgramEdit` through [[updateProgram]] — the same edits,
// applied by the same code, as every other editor of these programs.
import init, {
  example_programs,
  init_panic_hook,
  inspect_selection_fit,
  operation_schemas,
  pick_ray,
  preview_program,
  program,
  run_program,
  sketch_plane,
  solve_sketch,
  update_program,
} from "./wasm/pkg/geop.js";

export type Vec3 = [number, number, number];

export interface Scene {
  /** `[x, y, z, colorHex]` per point. */
  points: [number, number, number, number][];
  /** Per point: the name of the vertex it draws. */
  point_names: string[];
  /** `[x0, y0, z0, x1, y1, z1, colorHex]` per line segment. */
  lines: [number, number, number, number, number, number, number][];
  /** `[ax, ay, az, bx, by, bz, cx, cy, cz, colorHex]` per triangle. */
  triangles: [
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
  ][];
  /**
   * The kernel's surface normal at each triangle corner,
   * `[nax, nay, naz, nbx, ..., ncz]`, one per entry of `triangles` — what
   * makes a curved face shade smoothly instead of as facets.
   */
  normals: number[][];
  /** Per triangle: the index in `faces` of the face it belongs to. */
  triangle_faces: number[];
  /** Every face drawn: its name, and the name of the solid it bounds. */
  faces: { name: string; solid: string | null }[];
  /** Per line: the index in `sketch_names` of the sketch it belongs to, or `-1` for an edge of the model. */
  line_sketches: number[];
  sketch_names: string[];
  /** Per line: the index in `edge_names` of the edge of the model it draws, or `-1` for a sketch's. */
  line_edges: number[];
  edge_names: string[];
}

/**
 * An entity to draw highlighted: one a step can refer to (see
 * [[EntityRef]]), a whole solid — every face of it — or a sketch's curves.
 */
export type Highlight = EntityRef | { type: "Solid"; name: string } | { type: "Sketch"; name: string };

/** Whether two entities (or highlights) are the same one. */
export function sameEntity(a: Highlight, b: Highlight): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}

let ready: Promise<void> | null = null;

/** Instantiate the wasm module. Safe to call repeatedly; only runs once. */
export function loadGeop(): Promise<void> {
  if (!ready) {
    ready = init().then(() => {
      init_panic_hook();
    });
  }
  return ready;
}

// ── operations ───────────────────────────────────────────────────────────────
//
// Mirrors `geop_ops_parts::OperationSchema`: what every operation takes, so
// the app builds its forms from these instead of knowing operations by hand.

/** What kind of value an argument holds, and so how it is entered. */
export type ArgKind =
  | { type: "number"; default: number; min: number; max: number }
  | { type: "bool"; default: boolean }
  /** The name of a solid, picked in the viewport. */
  | { type: "solid" }
  /** The name of a face, picked in the viewport. */
  | { type: "face" }
  /** The name of a sketch of the part, picked in the viewport. */
  | { type: "sketch" }
  /** The id of a line of the sketch named by the argument `sketch`. */
  | { type: "sketch_line"; sketch: string }
  | { type: "choice"; options: string[]; default: string }
  /** A plane to sketch on, as an [[EntityRef]]: a base plane, a planar face or a datum plane. */
  | { type: "plane" }
  /** [[EntityRef]]s to build on — points, edges, faces, datums — picked in the viewport, in order. */
  | { type: "selection" }
  /**
   * A [[Construction]]: how to build a datum from the entities of the
   * argument `selection`, one of `options` — each of which fits only some
   * selections (see [[inspectSelection]]).
   */
  | { type: "construction"; selection: string; options: ConstructionSchema[] }
  /** A [[Sketch]], drawn on the plane given by the argument `plane`. */
  | { type: "drawing"; plane: string }
  /**
   * A [[Combine]]: a new body, or a boolean with a target solid picked in
   * the viewport. Until the user picks a mode, the sign of the number
   * argument `sign` (if any) picks it: join when positive, cut when negative.
   */
  | { type: "combine"; sign: string | null };

/**
 * What an extrude or revolve does with the solid it builds: keep it as a
 * new body, or combine it with `target` (which that consumes). The result
 * is named after the step either way.
 */
export type Combine =
  | { mode: "new_body" }
  | { mode: "union" | "intersection" | "difference"; target: string | null };

/** What a datum stands for. */
export type DatumKind = "point" | "axis" | "plane";

/** What a construction needs an input to be — see `geop_ops_parts::operation::Role`. */
export type Role = "point" | "line" | "plane" | "edge" | "circle" | "round";

/** One way to build a datum: what it builds, what it needs selected, and the values it takes besides. */
export interface ConstructionSchema {
  method: string;
  label: string;
  doc: string;
  result: DatumKind;
  inputs: Role[];
  params: ArgSchema[];
}

/** A chosen construction: its method, and a value for each of its params. */
export type Construction = { method: string } & Record<string, unknown>;

/** What a selection can be used as and built into — see [[inspectSelection]]. */
export interface SelectionFit {
  /** Per selected entity: the roles it can fill. None for one the part does not have. */
  roles: Role[][];
  /** The methods of the constructions that fit. */
  fits: string[];
}

export interface ArgSchema {
  name: string;
  doc: string;
  kind: ArgKind;
}

export interface OperationSchema {
  /** How a step spells the operation: `extrude`. */
  kind: string;
  label: string;
  doc: string;
  args: ArgSchema[];
}

export type Args = Record<string, unknown>;

/** An operation with its arguments: `{operation: "extrude", args: {...}}`. */
export interface Operation {
  operation: string;
  args: Args;
}

export interface Step extends Operation {
  id: string;
}

export interface Program {
  steps: Step[];
}

/** A change to the program — see `geop_ops_parts::ProgramEdit`. */
export type ProgramEdit =
  | ({ edit: "insert"; index: number; id?: string } & Operation)
  | ({ edit: "update"; id: string } & Operation)
  | { edit: "remove"; id: string }
  | { edit: "move"; id: string; index: number }
  | { edit: "replace"; program: Program };

export interface StepResult {
  id: string;
  error: string | null;
}

/** Where a handle writes in its step: field names into the step's arguments. */
export type ArgPath = string[];

/**
 * A draggable value of a step (see `geop_ops_parts::operation::Handle`):
 * where it is, how it moves, and which argument(s) it writes. Every run
 * returns every step's handles; `group` says whether a handle belongs to a
 * feature or to a sketch, for choosing which to offer.
 */
export type StepHandle = {
  step: string;
  label: string;
  group: "feature" | "sketch";
  position: Vec3;
} & (
  | { motion: "linear"; direction: Vec3; arg: ArgPath; value: number; scale: number }
  | { motion: "planar"; u: Vec3; v: Vec3; x: ArgPath; y: ArgPath; value: [number, number] }
);

/** A sketch of the built part. */
export interface SketchInfo {
  name: string;
  lines: { id: Id; construction: boolean }[];
  frame: Frame;
}

/** A datum of the built part: reference geometry, drawn and pickable. */
export interface DatumInfo {
  name: string;
  kind: DatumKind;
  frame: Frame;
}

/** What a run built. */
export interface RunResult {
  /** One per step that ran; the last may be the failure that stopped it. */
  results: StepResult[];
  scene: Scene;
  /** Solid names, oldest first. */
  solids: string[];
  sketches: SketchInfo[];
  /** Datums, oldest first. */
  datums: DatumInfo[];
  handles: StepHandle[];
}

export function operationSchemas(): OperationSchema[] {
  return JSON.parse(operation_schemas()) as OperationSchema[];
}

/** The program being edited. */
export function currentProgram(): Program {
  return JSON.parse(program()) as Program;
}

/** Apply `edit` to the program — the only way it changes. Throws if it is rejected; returns the id of the step it touched. */
export function updateProgram(edit: ProgramEdit): string | null {
  return JSON.parse(update_program(JSON.stringify(edit))) as string | null;
}

/**
 * Build the program's first `stop` steps (all, if `null`). The built part
 * is what [[pickRay]] and [[sketchPlane]] see.
 */
export function runProgram(stop: number | null): RunResult {
  return JSON.parse(run_program(JSON.stringify(stop))) as RunResult;
}

/** Like [[runProgram]], for the program with `edit` applied — without applying it. */
export function previewProgram(edit: ProgramEdit, stop: number | null): RunResult {
  return JSON.parse(preview_program(JSON.stringify(edit), JSON.stringify(stop))) as RunResult;
}

export function examplePrograms(): { name: string; program: Program }[] {
  return JSON.parse(example_programs()) as { name: string; program: Program }[];
}

// ── picking ──────────────────────────────────────────────────────────────────

/** What a pick looks for: one kind of entity, or `"any"` — the smallest visible vertex, edge or face under the ray. */
export type PickFilter = "vertex" | "edge" | "face" | "solid" | "sketch" | "any";

export type PickKind = "vertex" | "edge" | "face" | "solid" | "sketch";

export interface PickHit {
  kind: PickKind;
  /** The name of what was hit — what a step refers to it by. */
  name: string;
  point: Vec3;
  t: number;
  /** For a face or solid hit, the name of the solid it belongs to. */
  solid: string | null;
}

/**
 * Cast a ray against the part of the most recent [[runProgram]] (never a
 * preview's). `tolerance` is a world-space distance, only meaningful for
 * `"vertex"`/`"edge"` and a sketch's curves — a `"sketch"` is also hit
 * anywhere inside its closed regions.
 */
export function pickRay(origin: Vec3, dir: Vec3, filter: PickFilter, tolerance: number): PickHit | null {
  const json = pick_ray(origin[0], origin[1], origin[2], dir[0], dir[1], dir[2], filter, tolerance);
  return JSON.parse(json) as PickHit | null;
}

// ── sketching ────────────────────────────────────────────────────────────────
//
// Mirrors `geop_core_sketch::Sketch` (serialized as-is by the wasm crate):
// every point, curve and constraint keyed by a stable id — a JSON object key —
// that is handed out once and never reused, so curves, constraints and the
// kernel's names of what is built from a sketch (`extrude(op4,op3,c7)`) keep
// referring to the same entity however the sketch is edited around it.

/** A sketch entity's id. Unique per kind; `Sketch.next_id` is above all of them. */
export type Id = number;

export type WorldAxis = "X" | "Y" | "Z";

/**
 * Something a step builds on (see `geop_ops_parts::EntityRef`): the
 * origin, a world axis, a base plane through the origin named by its normal
 * (`Z` is normal to the z axis), or a vertex, edge, face or datum of the
 * part by name.
 */
export type EntityRef =
  | { type: "Origin" }
  | { type: "Axis"; axis: WorldAxis }
  | { type: "Plane"; normal: WorldAxis }
  | { type: "Vertex" | "Edge" | "Face" | "Datum"; name: string };

/** How an entity is shown: `Z plane`, or its name. */
export function entityLabel(entity: EntityRef): string {
  switch (entity.type) {
    case "Origin":
      return "Origin";
    case "Axis":
      return `${entity.axis} axis`;
    case "Plane":
      return `${entity.normal} plane`;
    default:
      return entity.name;
  }
}

/** An argument's value as the entities it holds — none if it has none yet. */
export function entities(value: unknown): EntityRef[] {
  return Array.isArray(value) ? (value as EntityRef[]) : [];
}

/** Which kind of datum an entity of the origin gizmo is — see `originGizmo.ts`. */
export function baseDatumKind(entity: EntityRef): DatumKind | null {
  return entity.type === "Origin" ? "point" : entity.type === "Axis" ? "axis" : entity.type === "Plane" ? "plane" : null;
}

export interface SketchPoint {
  x: number;
  y: number;
}

export type CurveKind =
  | { type: "Line"; start: number; end: number }
  /** Turns counter-clockwise by `sweep` radians (clockwise if negative). */
  | { type: "Arc"; start: number; end: number; sweep: number }
  | { type: "Circle"; center: number; radius: number }
  | { type: "Spline"; control_points: number[] };

export type SketchCurve = CurveKind & { construction: boolean };

export type Constraint =
  | { type: "Coincident"; a: number; b: number }
  | { type: "PointOnCurve"; point: number; curve: number }
  | { type: "Horizontal"; line: number }
  | { type: "Vertical"; line: number }
  | { type: "Parallel"; a: number; b: number }
  | { type: "Perpendicular"; a: number; b: number }
  | { type: "Collinear"; a: number; b: number }
  | { type: "Tangent"; a: number; b: number }
  | { type: "Equal"; a: number; b: number }
  | { type: "Concentric"; a: number; b: number }
  | { type: "Midpoint"; point: number; curve: number }
  | { type: "Symmetric"; a: number; b: number; line: number }
  | { type: "Fix"; point: number; x: number; y: number }
  | { type: "Distance"; a: number; b: number; value: number }
  | { type: "DistanceX"; a: number; b: number; value: number }
  | { type: "DistanceY"; a: number; b: number; value: number }
  | { type: "PointLineDistance"; point: number; line: number; value: number }
  | { type: "Length"; curve: number; value: number }
  | { type: "Radius"; curve: number; value: number }
  | { type: "Angle"; a: number; b: number; value: number };

export interface Sketch {
  points: Record<Id, SketchPoint>;
  curves: Record<Id, SketchCurve>;
  constraints: Record<Id, Constraint>;
  /** The id the next added entity gets. */
  next_id: number;
}

export const EMPTY_SKETCH: Sketch = { points: {}, curves: {}, constraints: {}, next_id: 0 };

/** Every `[id, entity]` of an id-keyed record, in id order. */
export function entries<T>(record: Record<Id, T>): [Id, T][] {
  return Object.entries(record)
    .map(([id, value]) => [Number(id), value] as [Id, T])
    .sort((a, b) => a[0] - b[0]);
}

export interface SolveReport {
  converged: boolean;
  max_residual: number;
  iterations: number;
  dof: number;
  free_points: Record<Id, boolean>;
  free_curves: Record<Id, boolean>;
  failed_constraints: Id[];
}

export interface SolveResult {
  sketch: Sketch;
  report: SolveReport;
  /** Per region: `[outer, ...holes]`, each a polyline in sketch coordinates. */
  regions: [number, number][][][];
  regions_error: string | null;
}

/** A sketch plane: sketch `(x, y)` lies at `origin + x u + y v`. */
export interface Frame {
  origin: Vec3;
  u: Vec3;
  v: Vec3;
  normal: Vec3;
}

/** Solve `sketch`, pulling each `[pointId, x, y]` of `drags` towards its target. */
export function solveSketch(sketch: Sketch, drags: [number, number, number][] = []): SolveResult {
  return JSON.parse(solve_sketch(JSON.stringify(sketch), JSON.stringify(drags))) as SolveResult;
}

/** Resolve `plane` against the part of the most recent [[runProgram]]. Throws if it is not planar. */
export function sketchPlane(plane: EntityRef): Frame {
  return JSON.parse(sketch_plane(JSON.stringify(plane))) as Frame;
}

// ── datums ───────────────────────────────────────────────────────────────────

/**
 * What each entity of `selection` can be used as in the part of the most
 * recent [[runProgram]], and which datum constructions fit it — by the very
 * matching a step applies.
 */
export function inspectSelection(selection: EntityRef[]): SelectionFit {
  return JSON.parse(inspect_selection_fit(JSON.stringify(selection))) as SelectionFit;
}
