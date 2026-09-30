// Thin, typed wrapper around the generated `wasm-bindgen` bindings — the
// rest of the app imports from here instead of `./wasm/pkg/geop.js`
// directly, so the wasm-loading/init dance and JSON (de)serialization stay
// in one place.
//
// The whole editor lives in the kernel (see `geop_cad_base::editor`): the
// app sends every command — a step started, a click in the viewport as a
// ray, a slider moved, undo — through [[send]], and draws the [[Update]]
// that comes back. Which operations exist, what their dialogs hold, what a
// click picks or snaps to, what runs and what is drawn: all of it is
// decided there, none of it here.
import init, { handle, init_panic_hook } from "./wasm/pkg/geop.js";

export type Vec3 = [number, number, number];

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

/** How an entity is shown: its name, and which component of a frame. */
export function entityLabel(e: EntityRef): string {
  if (e.type !== "Datum" || !e.component) return e.name;
  if ("axis" in e.component) return `${e.name} ${e.component.axis} axis`;
  const plane = { x: "yz", y: "zx", z: "xy" }[e.component.plane];
  return `${e.name} ${plane} plane`;
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

// ── the part, as drawn ───────────────────────────────────────────────────────
//
// Mirrors `geop_ops::ui::PartView`.

/** A datum of the part: reference geometry, drawn and pickable. */
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

/** A part as the viewport draws it, every entity by name. */
export interface PartView {
  vertices: { name: string; at: Vec3 }[];
  edges: { name: string; polyline: Vec3[] }[];
  /** Triangulated, with the kernel's surface normal at each corner. */
  faces: { name: string; solid: string | null; triangles: [Vec3, Vec3, Vec3][]; normals: [Vec3, Vec3, Vec3][] }[];
  /** Curves in their plane's `u`/`v` coordinates. */
  sketches: { name: string; plane: Frame; curves: { construction: boolean; polyline: [number, number][] }[] }[];
  datums: DatumInfo[];
  /** The part's solids, oldest first. */
  solids: string[];
  extent: Extent;
}

// ── programs ─────────────────────────────────────────────────────────────────

/** An operation the editor offers — see `geop_ops::OperationInfo`. */
export interface OperationInfo {
  /** How a step spells it: `extrude`. */
  kind: string;
  label: string;
  doc: string;
}

/** A step of a program: an operation with its arguments, and its id. */
export interface Step {
  id: string;
  operation: string;
  args: unknown;
}

export interface Program {
  steps: Step[];
}

// ── editing ──────────────────────────────────────────────────────────────────
//
// Mirrors `geop_ops::ui` and `geop_cad_base::editor`.

/**
 * How far from its ray a pointer reaches: a cone from the eye in
 * perspective, a tube in an orthographic view. Everything drawn at a
 * constant size on screen is laid out in reaches (see [[REACH_PX]]).
 */
export type Reach = { type: "cone"; slope: number } | { type: "tube"; radius: number };

/** Where the pointer is: the ray from the eye through it, and how far it reaches. */
export interface Pointer {
  ray: { origin: Vec3; dir: Vec3 };
  reach: Reach;
}

/** What a field was set to — or that it was pressed. */
export type Value =
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
export type EditEvent = { type: "dialog"; key: string; value: Value } | { type: "key"; key: string } | PointerEvent_;

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

/** What a click picks: a kind of entity. */
export type Target = "vertex" | "edge" | "face" | "solid" | "sketch" | { datum: DatumKind };

/** A dialog primitive. */
export type Control =
  | { type: "heading"; text: string }
  | { type: "text"; text: string; tone: Tone }
  | { type: "buttons"; buttons: ButtonItem[] }
  | { type: "checkbox"; label: string; value: boolean }
  | { type: "number"; label: string; value: number; slider: [number, number] | null; step: number }
  /** Grouped options are shown all at once, the rest as a dropdown. */
  | { type: "select"; label: string; value: string; options: Choice[] }
  /** Entities picked in the viewport; pressing it arms it. */
  | { type: "pick"; label: string; value: EntityRef[]; targets: Target[]; multiple: boolean; armed: boolean }
  | { type: "list"; items: ListItem[]; empty: string };

/** A control, under the key the events it sends carry. */
export type Field = { key: string } & Control;

export type Shape =
  | { shape: "point"; at: Vec3 }
  | { shape: "polyline"; points: Vec3[] }
  | { shape: "triangles"; triangles: [Vec3, Vec3, Vec3][] }
  /** Moved by `offset`, in reaches (see [[REACH_PX]]). */
  | { shape: "label"; at: Vec3; text: string; offset: Vec3 }
  | { shape: "handle"; at: Vec3; direction: Vec3 };

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

/** What the step being edited shows. */
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

/** Something the user did — see `geop_cad_base::Command`. */
export type Command =
  | { command: "show" }
  | { command: "new"; kind: string }
  | { command: "open"; id: string }
  | { command: "event"; event: EditEvent }
  | { command: "commit" }
  | { command: "cancel" }
  | { command: "preview"; preview: boolean }
  | { command: "remove"; id: string }
  | { command: "move"; id: string; index: number }
  | { command: "seek"; marker: number | null }
  | { command: "load"; program: Program }
  | { command: "load_example"; name: string }
  | { command: "undo" }
  | { command: "redo" };

/** A step of the program, as a list of steps shows it. */
export interface StepInfo {
  id: string;
  kind: string;
  label: string;
  /** Its fields in one line. */
  summary: string;
  error: string | null;
  /** Whether it runs: it is before where the program runs to. */
  runs: boolean;
  editing: boolean;
}

export interface ProgramState {
  program: Program;
  steps: StepInfo[];
  /** How many steps run: new steps go there. */
  marker: number;
  can_undo: boolean;
  can_redo: boolean;
  operations: OperationInfo[];
  examples: string[];
}

export interface SceneState {
  part: PartView;
  /** Sketches and datums, by name, not to draw. */
  hidden: string[];
}

export interface StepState {
  kind: string;
  label: string;
  doc: string;
  /** None yet, for a new step. */
  id: string | null;
  presentation: Presentation;
  /** Why it does not build; only a step that builds can be committed. */
  error: string | null;
  preview: boolean;
}

/** What to show after a command; what did not change is left out. */
export interface Update {
  /** Why the command was refused, if it was. */
  error: string | null;
  program: ProgramState | null;
  scene: SceneState | null;
  /** The step being edited, if one is. */
  step: StepState | null;
}

/** Apply `command` in the kernel, and get back what to show now. */
export function send(command: Command): Update {
  return JSON.parse(handle(JSON.stringify(command))) as Update;
}
