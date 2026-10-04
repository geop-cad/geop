// Thin, typed wrapper around the kernel — the rest of the app imports from
// here instead of `./backend` directly, so JSON (de)serialization stays in
// one place, and where the kernel runs (wasm in the browser, a native
// process in VS Code) is invisible to it.
//
// The whole editor lives in the kernel (see `geop_cad_base::editor`): the
// app sends every command — a step started, a click in the viewport as a
// ray, a slider moved, undo — through [[send]], and draws the [[Update]]
// that comes back. Which operations exist, what their dialogs hold, what a
// click picks or snaps to, what runs and what is drawn: all of it is
// decided there, none of it here.
import { call, loadBackend } from "./backend";

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
 * planes; for a sketch, perhaps one of its curves, by id. Every part has the
 * frame datum `origin`. An entity of a part placed in this one is named
 * behind the placing step's id: `bolt/extrude(head,end)`.
 */
export type EntityRef =
  | { type: "Vertex" | "Edge" | "Face" | "Solid" | "Sketch" | "Sketch3d"; name: string }
  | { type: "Datum"; name: string; component?: DatumComponent }
  | { type: "SketchCurve"; sketch: string; curve: number }
  | { type: "SketchPoint"; sketch: string; point: number };

/** Whether two entities are the same one. */
export function sameEntity(a: EntityRef, b: EntityRef): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}

/** How an entity is shown: its name, and which component of a frame. */
export function entityLabel(e: EntityRef): string {
  if (e.type === "SketchCurve") return `${e.sketch} c${e.curve}`;
  if (e.type === "SketchPoint") return `${e.sketch} p${e.point}`;
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

/** Start the kernel (see `backend.ts`). Safe to call repeatedly; only runs once. */
export const loadGeop = loadBackend;

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

/**
 * A part placed in the part drawn — however deep — drawn as its component's
 * view (see [[SceneState.components]]) in its own `frame`.
 */
export interface ViewInstance {
  /** Its name in the part drawn: `bolt`, or `asm/bolt`. Its entities are named behind it. */
  name: string;
  /** The key of its component's view. */
  component: string;
  frame: Frame;
}

/** A part as the viewport draws it, every entity by name. */
export interface PartView {
  /** Each with the solid it belongs to, if any — else the faces standing on their own it bounds, hidden with all of them. */
  vertices: { name: string; solid: string | null; faces: string[]; at: Vec3 }[];
  edges: { name: string; solid: string | null; faces: string[]; polyline: Vec3[] }[];
  /** Triangulated, with the kernel's surface normal at each corner. */
  faces: { name: string; solid: string | null; triangles: [Vec3, Vec3, Vec3][]; normals: [Vec3, Vec3, Vec3][] }[];
  /** Curves in their plane's `u`/`v` coordinates. */
  sketches: {
    name: string;
    plane: Frame;
    curves: { id: number; construction: boolean; polyline: [number, number][] }[];
    points: { id: number; at: [number, number] }[];
  }[];
  /** 3-D sketches: curves and points in space. */
  sketches3d: {
    name: string;
    curves: { id: number; construction: boolean; polyline: Vec3[] }[];
    points: { id: number; at: Vec3 }[];
  }[];
  datums: DatumInfo[];
  /** Cosmetic threads, each drawn as the helix it runs along on its face, and what it is called (`M6x1`). */
  threads: { name: string; designation: string; polyline: Vec3[] }[];
  /** The part's solids, oldest first. */
  solids: string[];
  extent: Extent;
  /** The part's colour, `#rrggbb`; none for the viewer's own. */
  color: string | null;
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

/** A pose, as a file has it: a position and a rotation quaternion `[w, x, y, z]`. */
export interface Pose {
  position: Vec3;
  rotation: [number, number, number, number];
}

/** A row of a table parameter: a variant, by name, with a value per column. */
export interface ParameterRow {
  name: string;
  values: number[];
}

/** What a parameter is — see `geop_ops::parameters::ParameterKind`. */
export type ParameterKind =
  /** A number, given as a formula of the parameters before it; `min`/`max` what a slider offers. */
  | { type: "number"; expression: string; min?: number | null; max?: number | null }
  /** A family of variants: `name` is the selected row, `name.column` its value there. */
  | { type: "table"; columns: string[]; rows: ParameterRow[]; selected: string };

export type Parameter = { name: string } & ParameterKind;

/** A program's parameters: the part's colour, and its named values, in order. */
export interface Parameters {
  color?: string | null;
  /** What the part is made of — its density in kg/m³ — none for one not given (weighed as water). */
  material?: Material | null;
  values?: Parameter[];
}

/** A material: a name, and its density in kg/m³ — see `geop_ops::parameters::Material`. */
export interface Material {
  name: string;
  density: number;
}

/** What a parameter resolves to: a number, a pose, or text (a table's row, a colour). */
export type ParamValue = number | Pose | string;

/**
 * A program: its parameters, its steps — its structure — and its state — the values it is built with,
 * by name: where every placed part is (`bolt.pose`), solved by the kernel.
 */
export interface Program {
  parameters?: Parameters;
  steps: Step[];
  state?: Record<string, ParamValue>;
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
  /** Text typed: a value as a formula, a colour. */
  | { type: "text"; value: string }
  | { type: "choice"; value: string }
  /** The entity at this index taken out of a reference field. */
  | { type: "remove_at"; value: number }
  /** Everything taken out of a reference field. */
  | { type: "clear" };

/** One thing the user did in the viewport. */
export type PointerEvent_ =
  /** `shift` held turns snapping off. */
  | { type: "hover"; pointer: Pointer; shift: boolean }
  | { type: "leave" }
  | { type: "click"; pointer: Pointer; button: "primary" | "secondary"; double: boolean; shift: boolean }
  | { type: "drag"; from: Pointer; to: Pointer; done: boolean; shift: boolean };

/** One thing the user did while editing a step. */
export type EditEvent = { type: "dialog"; key: string; value: Value } | { type: "key"; key: string } | PointerEvent_;

export type Tone = "normal" | "hint" | "error" | "success";

/** Something to do or choose — see `geop_ops::ui::Action`. */
export interface Action {
  /** What pressing it sends, as a choice. */
  value: string;
  label: string;
  /** What it does — or, disabled, why it cannot be done now. */
  title: string | null;
  group: string | null;
  enabled: boolean;
  /** Shown pressed: the tool in hand, the way chosen. */
  active: boolean;
  /** The icon it is shown as (see `icons.tsx`), its label its tooltip; none to show the label. */
  icon: string | null;
}

export interface Choice {
  value: string;
  label: string;
}

export interface ListItem {
  key: string;
  label: string;
  detail: string | null;
  tone: Tone;
  selected: boolean;
  removable: boolean;
  value: number | null;
  /** Text to edit in place — a value as a formula — sent back as `text`. */
  text: string | null;
}

/** What an entity can be used as, and what a pick looks for — see `geop_ops::operation::Role`. */
export type Role =
  | "point"
  | "line"
  | "plane"
  | "edge"
  | "circle"
  | "round"
  | "face"
  | "solid"
  | "sheet"
  | "sketch"
  | "path";

/** What a number measures. */
export type Unit = "length" | "angle" | "fraction" | "count";

/** An entity a reference field holds, and what the kernel found it to be. */
export interface Picked {
  entity: EntityRef;
  detail: string | null;
  tone: Tone;
}

/** A dialog primitive. */
export type Control =
  | { type: "heading"; text: string }
  | { type: "text"; text: string; tone: Tone }
  /** Pressing one sends its value as a choice. */
  | { type: "actions"; actions: Action[] }
  | { type: "checkbox"; label: string; value: boolean }
  /** A slider over `range`, if given. */
  | { type: "number"; label: string; value: number; unit: Unit; range: [number, number] | null; step: number }
  /** Found by typing, if `searchable`. */
  | { type: "select"; label: string; value: string; options: Choice[]; searchable: boolean }
  /** A colour, `#rrggbb`, sent back as `text`. */
  | { type: "color"; label: string; value: string }
  /**
   * Entities picked in the viewport that can fill one of `roles`: pressing
   * it arms it; an entity is taken out by `remove_at`, all by `clear`.
   */
  | {
      type: "reference";
      label: string;
      roles: Role[];
      scope: EntityRef | null;
      value: Picked[];
      multiple: boolean;
      /** Whether the step needs it before it builds; one that is not is picked for only when pressed. */
      required: boolean;
      armed: boolean;
    }
  | { type: "list"; items: ListItem[]; empty: string };

/** A control, under the key the events it sends carry. */
export type Field = { key: string } & Control;

export type Shape =
  | { shape: "point"; at: Vec3 }
  | { shape: "polyline"; points: Vec3[] }
  | { shape: "triangles"; triangles: [Vec3, Vec3, Vec3][] }
  /** Moved by `offset`, in reaches (see [[REACH_PX]]). */
  | { shape: "label"; at: Vec3; text: string; offset: Vec3 }
  | { shape: "handle"; at: Vec3; direction: Vec3 }
  /** A placed part, by its instance's name, drawn lit when hovered or selected. */
  | { shape: "instance"; name: string };

export type Style =
  | "free"
  | "fixed"
  | "selected"
  | "hover"
  | "failed"
  | "construction"
  /** Given rather than drawn: a sketch's axes, what it projects. */
  | "reference"
  | "draft"
  | "region"
  | "guide"
  | "handle"
  /** Where what is drawn or dragged would snap to. */
  | "snap"
  /** What the tool in hand would remove. */
  | "removed";

export type Visual = { key: string; style: Style } & Shape;

/** What the step being edited shows. */
export interface Presentation {
  dialog: Field[];
  visuals: Visual[];
  /** Entities of the part to draw lit. */
  highlights: EntityRef[];
  /** What a click picks right now: entities that can fill one of these. */
  pickable: Role[];
  /** A plane to work in, head on. */
  focus: Frame | null;
  /** Whether a press where the pointer last hovered starts a drag. */
  grab: boolean;
  /** A value asked for in place, at `at`: what is typed goes to the field `key` as `text`. */
  prompt: Prompt | null;
}

/** A value asked for in place in the viewport — see `geop_ops::ui::Prompt`. */
export interface Prompt {
  key: string;
  label: string;
  value: string;
  at: Vec3;
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
  /** `path`: the file the program is in, which the files it places are named relative to. */
  | { command: "load"; program: Program; path?: string }
  /** The other program files are now these, by path: their text, or `null` for one that is gone. */
  | { command: "files"; files: Record<string, string | null> }
  | { command: "load_example"; name: string }
  /** Add the files of an example of several files, put in `folder`, and edit the first. */
  | { command: "load_workspace_example"; name: string; folder?: string }
  | { command: "undo" }
  | { command: "redo" }
  /** Show or hide the datum, sketch, solid, face standing on its own or placed part `name`. */
  | { command: "visibility"; name: string; visible: boolean }
  /** The program's parameters are now these. */
  | { command: "parameters"; parameters: Parameters }
  /** Take the drag tool in hand, or put it down: with no step edited, drag any placed part. */
  | { command: "drag_tool"; on: boolean }
  /** Set a joint's coordinate — an angle in degrees, or a distance: the parts move to it. */
  | { command: "joint"; parameter: string; value: number }
  /** Take the measure tool in hand, or put it down: with no step edited, clicks pick up to two entities to measure. */
  | { command: "measure_tool"; on: boolean }
  /** Ask a question of the part as drawn, answered in [[Update]] `inspection`; changes nothing. */
  | { command: "inspect"; query: "mass_properties" | "interference" }
  /** Write a drawing of the part — the drawing step `id`, else the one edited or the last — as SVG or DXF. */
  | { command: "export_drawing"; id?: string; format: "svg" | "dxf"; date: string };

/** A joint's coordinate — see `geop_ops::assembly::JointValue`. */
export interface JointValue {
  /** The parameter of the program's state it is: `add_part(fore,m1).angle`. */
  parameter: string;
  motion: "angle" | "distance";
  value: number;
  min: number | null;
  max: number | null;
}

/** A joint of the part shown — see `geop_ops::assembly::JointInfo`. */
export interface JointInfo {
  name: string;
  kind: string;
  values: JointValue[];
}

/** How free each placed part is, and which mates conflict. */
export interface MateFreedom {
  parts: Record<string, number>;
  total: number;
  conflicting: string[];
}

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
  /** The file it is in. */
  path: string;
  steps: StepInfo[];
  /** How many steps run: new steps go there. */
  marker: number;
  can_undo: boolean;
  can_redo: boolean;
  /** Whether the drag tool is in hand. */
  drag_tool: boolean;
  /** Whether the measure tool is in hand. */
  measure_tool: boolean;
  /** The materials a part's can be picked from. */
  materials: Material[];
  /** What the parameters resolve to, by name — numbers, a table's row and columns, the colour — and why those that do not resolve fail. */
  parameters: { values: Record<string, ParamValue>; errors: Record<string, string> };
  operations: OperationInfo[];
  examples: string[];
  /** Examples of several files, for [[Command]] `load_workspace_example`. */
  workspace_examples: string[];
  /** The joints of the part shown, for [[Command]] `joint`. */
  joints: JointInfo[];
  /** How free its placed parts are, if it has any. */
  freedom: MateFreedom | null;
}

/** What kind of thing a [[StructureItem]] is. */
export type StructureKind = "datum" | "sketch" | "solid" | "face" | "part" | "mate";

/** A datum, sketch, solid, face standing on its own, placed part or mate of the part drawn: whether it is shown, if it is drawn at all. */
export interface StructureItem {
  kind: StructureKind;
  name: string;
  visible: boolean | null;
}

/**
 * What is drawn. The parts placed in the part come as what changed since
 * the last scene ([[applyScene]] keeps them), and the views of their
 * components once each.
 */
export interface SceneState {
  /** The part, without the parts placed in it. */
  part: PartView;
  /** What the part has beyond its faces, to list and show or hide. */
  structure: StructureItem[];
  /** Sketches and datums, by name, not to draw. */
  hidden: string[];
  /** The placed parts new, moved or drawn from another component since the last scene — every one, if `all`. */
  instances: ViewInstance[];
  /** Whether `instances` are all there are: forget any others. */
  all: boolean;
  /** The names of the placed parts gone since the last scene. */
  removed: string[];
  /** The views of the components the last scene's placed parts did not use, by key. */
  components: Record<string, PartView>;
}

/** The placed parts drawn, by name, and the views of the components they are drawn from, by key. */
export interface Placed {
  instances: Map<string, ViewInstance>;
  components: Record<string, PartView>;
}

/**
 * `placed` with `scene`'s changes: its placed parts added, moved and
 * removed, and the components they use — those no placed part uses any
 * more forgotten, as the kernel forgets having sent them.
 */
export function applyScene(placed: Placed, scene: SceneState): Placed {
  const instances = scene.all ? new Map<string, ViewInstance>() : new Map(placed.instances);
  for (const name of scene.removed) instances.delete(name);
  for (const instance of scene.instances) instances.set(instance.name, instance);
  const known = { ...placed.components, ...scene.components };
  const components: Record<string, PartView> = {};
  for (const instance of instances.values()) {
    const view = known[instance.component];
    if (view) components[instance.component] = view;
  }
  return { instances, components };
}

export interface StepState {
  kind: string;
  label: string;
  doc: string;
  /** None yet, for a new step. */
  id: string | null;
  presentation: Presentation;
  /** What it still needs picked, by its fields' labels: until then it is not built, and has no error. */
  missing: string[];
  /** Why it does not build, once it has what it needs; only a step that builds can be committed. */
  error: string | null;
  preview: boolean;
  /** Whether one of its own edits can be undone, or redone. */
  can_undo: boolean;
  can_redo: boolean;
}

/** What to show after a command; what did not change is left out. */
export interface Update {
  /** Why the command was refused, if it was. */
  error: string | null;
  program: ProgramState | null;
  scene: SceneState | null;
  /** The step being edited, if one is. */
  step: StepState | null;
  /** The program files the command added, the one now edited first. */
  files: { path: string; program: Program }[] | null;
  /** What the drag or measure tool shows, while it is in hand and no step is edited. */
  tool: Presentation | null;
  /** The file `export_drawing` wrote, to save. */
  export: { name: string; text: string } | null;
  /** What the measure tool's picks measure, while it is in hand — or the answer to an `inspect` command. */
  inspection: Inspection | null;
}

// ── inspecting — see `geop_ops_inspect` ──────────────────────────────────────

/** A number and how far the truth may be from it. */
export interface Bounded {
  value: number;
  error: number;
}

/** One number a measurement shows: `mm`, `mm²`, `mm³`, `kg` or `°`. */
export type Measured = { label: string; unit: string } & Bounded;

export interface Measurement {
  entities: EntityRef[];
  values: Measured[];
  /** Where the least distance between two entities is attained. */
  witness: [Vec3, Vec3] | null;
  /** The plane of one planar entity picked alone, to cut a section with. */
  plane: Frame | null;
  error: string | null;
}

/** Mass properties in mm and kg: volume mm³, area mm², mass kg, centre mm, inertia about the centre kg·mm². */
export interface MassSummary {
  volume: Bounded;
  area: Bounded;
  mass: Bounded;
  center: [Bounded, Bounded, Bounded];
  inertia: Bounded[][];
  principal_moments: [Bounded, Bounded, Bounded];
  principal_axes: [Vec3, Vec3, Vec3];
  converged: boolean;
}

export interface BodyMass {
  name: string;
  material: string;
  density: number;
  /** Whether no material was given, and water assumed. */
  assumed: boolean;
  properties: MassSummary | null;
  error: string | null;
}

export interface MassReport {
  bodies: BodyMass[];
  total: MassSummary | null;
}

export interface InterferenceReport {
  solids: number;
  found: { a: string; b: string; contact: "overlap" | "touch"; volume: Bounded | null }[];
  unchecked: { a: string; b: string; error: string }[];
}

export type Inspection =
  | ({ kind: "measure" } & Measurement)
  | ({ kind: "mass_properties" } & MassReport)
  | ({ kind: "interference" } & InterferenceReport);

/** Apply `command` in the kernel, and get back what to show now. */
export async function send(command: Command): Promise<Update> {
  return JSON.parse(await call(JSON.stringify(command))) as Update;
}
