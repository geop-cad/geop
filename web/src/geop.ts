// Thin, typed wrapper around the generated `wasm-bindgen` bindings — the
// rest of the app imports from here instead of `./wasm/pkg/geop.js`
// directly, so the wasm-loading/init dance and JSON (de)serialization stay
// in one place.
//
// The app edits a part program (`geop_cad_base::Program`) that lives in
// the wasm module: it never changes the program itself, but sends each
// change as a `ProgramEdit` through [[updateProgram]] — the same edits,
// applied by the same code, as every other editor of these programs.
//
// A step is edited through [[editStep]]: the app sends what the user did —
// a dialog control used, a click or a drag in the viewport as a ray, a
// key — and draws what comes back. Which operations exist, what their
// dialogs hold, what a click picks or snaps to: all of it is decided in the
// kernel (see `geop_ops::ui`), none of it here.
import init, {
  describe_program,
  edit_step,
  example_programs,
  init_panic_hook,
  operation_infos,
  preview_program,
  program,
  run_program,
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

/** One of a frame's own axes: `x` is its `u`, `y` its `v`, `z` its `w`. */
export type FrameAxis = "x" | "y" | "z";

/**
 * A part of a frame datum used on its own (see
 * `geop_core_math::primitives::DatumComponent`): one of its axes, or the
 * plane normal to one — `{plane: "z"}` is its xy plane.
 */
export type DatumComponent = { axis: FrameAxis } | { plane: FrameAxis };

/**
 * Something picked in the viewport (see `geop_ops::EntityRef`): an entity
 * of the part by name — for a frame datum, perhaps one of its axes or
 * planes. Every part has the frame datum `origin`.
 */
export type EntityRef =
  | { type: "Vertex" | "Edge" | "Face" | "Solid" | "Sketch"; name: string }
  | { type: "Datum"; name: string; component?: DatumComponent };

/** Whether two entities are the same one. */
export function sameEntity(a: EntityRef, b: EntityRef): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}

/** What a datum stands for. */
export type DatumKind = "point" | "axis" | "plane" | "frame";

/** A frame: `(x, y, z)` in it lies at `origin + x u + y v + z w`; as a plane, its `u`/`v` plane. */
export interface Frame {
  origin: Vec3;
  u: Vec3;
  v: Vec3;
  w: Vec3;
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

// ── programs ─────────────────────────────────────────────────────────────────

/** An operation the editor offers — see `geop_ops::OperationInfo`. */
export interface OperationInfo {
  /** How a step spells it: `extrude`. */
  kind: string;
  label: string;
  doc: string;
}

/** An operation with its arguments: `{operation: "extrude", args: {...}}`. */
export interface Operation {
  operation: string;
  args: unknown;
}

export interface Step extends Operation {
  id: string;
}

export interface Program {
  steps: Step[];
}

/** A step as a list of steps shows it. */
export interface StepInfo {
  id: string;
  kind: string;
  label: string;
  /** Its arguments in one line. */
  summary: string;
}

/** A change to the program — see `geop_ops::ProgramEdit`. */
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

/** A datum of the built part: reference geometry, drawn and pickable. */
export interface DatumInfo {
  name: string;
  kind: DatumKind;
  frame: Frame;
}

/** Where the drawing is and how big: datum planes and axes are drawn this big around the point of them nearest its center. */
export interface Extent {
  center: Vec3;
  size: number;
}

/** What a run built. */
export interface RunResult {
  /** One per step that ran; the last may be the failure that stopped it. */
  results: StepResult[];
  scene: Scene;
  /** Datums, oldest first. */
  datums: DatumInfo[];
  extent: Extent;
  /** The sketches and datums the steps that ran build on — hidden, since what was made from them shows them now. */
  references: EntityRef[];
}

export function operationInfos(): OperationInfo[] {
  return JSON.parse(operation_infos()) as OperationInfo[];
}

/** The program being edited. */
export function currentProgram(): Program {
  return JSON.parse(program()) as Program;
}

/** Every step of the program, as a list of steps shows it. */
export function describeProgram(): StepInfo[] {
  return JSON.parse(describe_program()) as StepInfo[];
}

/** Apply `edit` to the program — the only way it changes. Throws if it is rejected; returns the id of the step it touched. */
export function updateProgram(edit: ProgramEdit): string | null {
  return JSON.parse(update_program(JSON.stringify(edit))) as string | null;
}

/**
 * Build the program's first `stop` steps (all, if `null`). The built part
 * is what [[editStep]] edits against.
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

// ── editing a step ───────────────────────────────────────────────────────────
//
// Mirrors `geop_ops::ui`.

/** A ray from the eye, through the pointer. */
export interface Ray {
  origin: Vec3;
  dir: Vec3;
}

/**
 * How far from its ray a pointer reaches: a cone from the eye in
 * perspective, a tube in an orthographic view. Everything drawn at a
 * constant size on screen is laid out in reaches (see [[REACH_PX]]).
 */
export type Reach = { type: "cone"; slope: number } | { type: "tube"; radius: number };

/** Where the pointer is, and how far it reaches. */
export interface Pointer {
  ray: Ray;
  reach: Reach;
}

export type DialogValue =
  | { type: "press" }
  | { type: "remove" }
  | { type: "bool"; value: boolean }
  | { type: "number"; value: number }
  | { type: "choice"; value: string };

/** One thing the user did in the viewport. */
export type PointerEvent_ =
  | { type: "hover"; pointer: Pointer }
  | { type: "leave" }
  | { type: "click"; pointer: Pointer; button: "primary" | "secondary"; double: boolean; shift: boolean }
  | { type: "drag"; from: Pointer; to: Pointer; done: boolean };

/** One thing the user did while editing a step. */
export type EditEvent =
  | { type: "dialog"; key: string; value: DialogValue }
  | { type: "key"; key: string }
  | PointerEvent_;

export type Tone = "normal" | "hint" | "error" | "success";

export interface ButtonItem {
  key: string;
  label: string;
  title: string | null;
  active: boolean;
  enabled: boolean;
}

export interface Choice {
  value: string;
  label: string;
  enabled: boolean;
  title: string | null;
  group: string | null;
}

export interface ListItem {
  key: string;
  label: string;
  detail: string | null;
  tone: Tone;
  selected: boolean;
  removable: boolean;
  value: number | null;
}

/** A dialog primitive. */
export type Control =
  | { type: "heading"; text: string }
  | { type: "text"; text: string; tone: Tone }
  | { type: "button"; label: string; title: string | null; active: boolean; enabled: boolean; primary: boolean }
  | { type: "buttons"; buttons: ButtonItem[] }
  | { type: "checkbox"; label: string; value: boolean }
  | { type: "number"; label: string; value: number; slider: [number, number] | null; step: number }
  | { type: "select"; label: string; value: string; options: Choice[]; style: "dropdown" | "radio" }
  | { type: "list"; items: ListItem[]; empty: string };

/** A control, under the key the events it sends carry. */
export type Field = { key: string } & Control;

export type Shape =
  | { shape: "point"; at: Vec3 }
  | { shape: "polyline"; points: Vec3[] }
  | { shape: "triangles"; triangles: [Vec3, Vec3, Vec3][] }
  /** Moved by `offset`, in reaches (see [[REACH_PX]]). */
  | { shape: "label"; at: Vec3; text: string; offset: Vec3 }
  | { shape: "handle"; at: Vec3; direction: Vec3 | null };

export type Style =
  | "free"
  | "fixed"
  | "selected"
  | "hover"
  | "failed"
  | "construction"
  | "draft"
  | "region"
  | "guide"
  | "handle";

export type Visual = { key: string; style: Style } & Shape;

/** What a click picks: a kind of entity. */
export type Target =
  | "vertex"
  | "edge"
  | "face"
  | "solid"
  | "sketch"
  /** A datum of the kind — or a frame's axis or plane, or a frame as a whole for a point. */
  | { datum: DatumKind };

/** What an operation shows for a step. */
export interface Presentation {
  dialog: Field[];
  visuals: Visual[];
  /** Entities of the part to draw lit. */
  highlights: EntityRef[];
  /** What a click picks right now. */
  pickable: Target[];
  /** A plane to work in, head on. */
  focus: Frame | null;
  /** Whether a press where the pointer last hovered starts a drag. */
  grab: boolean;
}

/** A step being edited: the step as it now is, the session to send back, and what to show. */
export interface Edited {
  operation: Operation;
  session: unknown;
  presentation: Presentation;
}

/**
 * Edit a step against the part of the most recent [[runProgram]]: `event`
 * applied to `operation` with `session` (from the last call, or `null` to
 * start afresh) — or, without an event, only what to show. `{kind}` in
 * place of an operation starts a new step of that kind.
 */
export function editStep(
  request: { operation: Operation; session: unknown; event?: EditEvent } | { kind: string },
): Edited {
  return JSON.parse(edit_step(JSON.stringify(request))) as Edited;
}
