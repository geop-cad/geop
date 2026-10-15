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
  | { type: "Vertex" | "Edge" | "Face" | "Solid" | "Sketch" | "Sketch3d" | "Feature"; name: string }
  | { type: "Datum"; name: string; component?: DatumComponent }
  | { type: "SketchCurve"; sketch: string; curve: number }
  | { type: "SketchPoint"; sketch: string; point: number }
  /** A reference frame on `on`, put at `at` — a vertex of it, or an edge of it — if it says where. */
  | { type: "Frame"; on: EntityRef; at?: EntityRef };

/** Whether two entities are the same one. */
export function sameEntity(a: EntityRef, b: EntityRef): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}

/** How an entity is shown: its name, and which component of a frame. */
export function entityLabel(e: EntityRef): string {
  if (e.type === "SketchCurve") return `${e.sketch} c${e.curve}`;
  if (e.type === "SketchPoint") return `${e.sketch} p${e.point}`;
  if (e.type === "Frame") return e.at ? `frame on ${entityLabel(e.on)} at ${entityLabel(e.at)}` : `frame on ${entityLabel(e.on)}`;
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
  /**
   * Each with the solid it belongs to, if any — else the faces standing on their own it bounds, hidden with all of
   * them.
   */
  vertices: { name: string; solid: string | null; faces: string[]; at: Vec3 }[];
  edges: { name: string; solid: string | null; faces: string[]; polyline: Vec3[] }[];
  /** Triangulated, with the kernel's surface normal at each corner. */
  faces: {
    name: string;
    solid: string | null;
    /** The step whose feature made it — a cut, a boss, a hole — if one did: what picking it as a feature picks. */
    feature?: string;
    triangles: [Vec3, Vec3, Vec3][];
    normals: [Vec3, Vec3, Vec3][];
  }[];
  /** Curves in their plane's `u`/`v` coordinates. */
  sketches: {
    name: string;
    plane: Frame;
    curves: { id: number; construction: boolean; polyline: [number, number][] }[];
    points: { id: number; at: [number, number] }[];
  }[];
  datums: DatumInfo[];
  /** What the part's extensions draw besides its topology — a cosmetic thread as the helix it runs along on its face — and what each is called (`M6x1`). */
  annotations: { name: string; label: string; polyline: Vec3[] }[];
  /** The part's solids, oldest first. */
  solids: string[];
  extent: Extent;
  /** The part's colour, `#rrggbb`; none for the viewer's own. */
  color: string | null;
}

// ── programs ─────────────────────────────────────────────────────────────────

/** An operation the editor offers — see `geop_core_program::operation::OperationInfo`. */
export interface OperationInfo {
  /** How a step spells it: `extrude`. */
  kind: string;
  label: string;
  doc: string;
  /** The section of the toolbar it is in: `Features` (see `geop_core_program::operation::OperationGroup`). */
  group: string;
  /** How prominently its section offers it (see `geop_core_program::operation::OperationTier`). */
  tier: OperationTier;
  /** Not yet robust enough to offer by default: reduced mode leaves it out. */
  experimental: boolean;
  /** The extensions of the files its steps read that are no programs: `step` — data kept beside the programs. */
  reads: string[];
  /** What it is shown as: SVG markup on a 20 x 20 grid (see [[Icon]]); none to show its label. */
  icon: string | null;
}

/** A kind of file the editor writes of the part — see `geop_ops::output::ExporterInfo`. */
export interface ExporterInfo {
  /** What [[Command]] `export` asks for it by: `stl`. */
  kind: string;
  /** What it is offered as: `STL`. */
  label: string;
  /** What the file is for, in a few words. */
  doc: string;
  /** The formats it writes, by their files' extension, the default first. */
  formats: string[];
}

/** A question the editor answers about the part — see `geop_ops::output::InspectorInfo`. */
export interface InspectorInfo {
  /** What [[Command]] `inspect` and `pick_tool` ask it by, and what its answers are: `mass_properties`. */
  kind: string;
  label: string;
  doc: string;
  /** For one asked about entities picked: what it picks, and how many at most. */
  picks: { roles: string[]; most: number } | null;
  /** What it is shown as: SVG markup (see [[Icon]]). */
  icon: string | null;
}

/**
 * How prominently the toolbar offers an operation: a big button, a small
 * one, or only in its section's menu.
 */
export type OperationTier = "Big" | "Small" | "Menu";

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
  /** What the part is, in words, where it is listed: `ISO 4762 socket head cap screw`. */
  title?: string | null;
  /** What it is ordered as, before the values it is built with: `ISO 4762`, for `ISO 4762 M4x12`. */
  designation?: string | null;
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
 * A program: its steps — its structure, its parameters and properties among them — and its state —
 * the values it is built with, by name: where every placed part is (`bolt.pose`), solved by the kernel.
 */
export interface Program {
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
  /** The icon it is shown as — SVG markup (see [[Icon]]) — its label its tooltip; none to show the label. */
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
  | "curve"
  | "circle"
  | "round"
  | "face"
  | "solid"
  | "sheet"
  | "sketch"
  | "path"
  | "feature"
  | "frame";

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
  /** Files the step can be saved as: pressing one has the exporter `exporter` write it in that format (see [[Command]] `export`). */
  | { type: "download"; exporter: string; formats: Choice[] }
  /**
   * A slider over `range`, if given. A field that takes formulas has
   * `text`: the value as given — a number, or a formula of the parameters —
   * edited in place of `value` and sent back as `text`; `value` is what it
   * comes to, `error` why it does not evaluate.
   */
  | {
      type: "number";
      label: string;
      value: number;
      unit: Unit;
      range: [number, number] | null;
      step: number;
      text: string | null;
      error: string | null;
    }
  /** Found by typing, if `searchable`. */
  | { type: "select"; label: string; value: string; options: Choice[]; searchable: boolean }
  /**
   * A file the program reads, by its path relative to the program: one of
   * `options`, the files of the kinds `accept` (extensions without the dot)
   * next to it, or one chosen from disk — stored next to the program, its
   * path sent as `choice`.
   */
  | { type: "file"; label: string; value: string; options: Choice[]; accept: string[] }
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
  | { shape: "instance"; name: string }
  /** Three arrows from `at` along `axes` — the world's x, y and z, or a frame's — a constant size on screen (see `TRIAD_REACHES` in visuals3d.ts). */
  | { shape: "triad"; at: Vec3; axes: [Vec3, Vec3, Vec3] };

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
  | "removed"
  /** Text printed on a sheet of its own: a drawing's title block, parts list, captions. */
  | "paper";

export type Visual = { key: string; style: Style } & Shape;

/** A part of a gizmo, by the index of its axis — see `geop_ops::ui::GizmoPart`. */
export type GizmoPart = { part: "free" | "scale" } | { part: "move" | "plane" | "turn" | "stretch"; axis: number };

/**
 * A gizmo to move, turn or scale by, laid out in reaches (see
 * `geop_ops::ui::gizmo::size`) — see `geop_ops::ui::GizmoView`.
 */
export interface GizmoView {
  at: Vec3;
  /** Unit and orthogonal: the world's, or what it stands on's own. */
  axes: [Vec3, Vec3, Vec3];
  modes: { translate: boolean; rotate: boolean; scale: boolean };
  hover: GizmoPart | null;
  active: GizmoPart | null;
  /** What a drag under way has done so far. */
  readout: string | null;
}

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
  /** A sheet of its own the step is edited on, in `focus`, and how far it reaches: no part is drawn, and the view frames it. */
  sheet: Extent | null;
  /** Whether a press where the pointer last hovered starts a drag. */
  grab: boolean;
  /** A value asked for in place, at `at`: what is typed goes to the field `key` as `text`. */
  prompt: Prompt | null;
  /** A gizmo to move, turn or scale by. */
  gizmo: GizmoView | null;
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
  /** The step `id` as text to paste, in the update's `export`. */
  | { command: "copy"; id: string }
  /** Insert the steps of `text` — what `copy` gave — where new steps go. */
  | { command: "paste"; text: string }
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
  /** Rename the parameter `from` to `to`, and every formula reading it. */
  | { command: "rename_parameter"; from: string; to: string }
  /** Take the drag tool in hand, or put it down: with no step edited, drag any placed part. */
  | { command: "drag_tool"; on: boolean }
  /** Set a joint's coordinate — an angle in degrees, or a distance: the parts move to it. */
  | { command: "joint"; parameter: string; value: number }
  /** Take the tool of the inspector `inspector` (one that picks) in hand, or — `null` — put it down: with no step edited, clicks pick what it asks about. */
  | { command: "pick_tool"; inspector: string | null }
  /** Ask the inspector `inspector` about the part as drawn, told its own `options`: answered in [[Update]] `inspection`; changes nothing. */
  | { command: "inspect"; inspector: string; options?: unknown }
  /**
   * Write the part shown with the exporter `exporter` — in `format`, else
   * its first — told its own `options` and dated `date` (`YYYY-MM-DD`): the
   * update's `export`.
   */
  | { command: "export"; exporter: string; format?: string; options?: unknown; date?: string }
  /** Panic, on purpose: how recovering from a kernel that crashed is checked (`e2e/`). */
  | { command: "crash" };

/** A file the kernel wrote, to save: text, or bytes in base64. */
export interface ExportedFile {
  name: string;
  /** What to save it as: `model/stl`. */
  media_type: string;
  text?: string;
  bytes?: string;
}

/** A value of the state a user sets, a joint's coordinate — see `geop_ops::solving::SettingState`. */
export interface JointValue {
  /** The parameter of the program's state it is: `add_part(fore,m1).angle`. */
  parameter: string;
  motion: "angle" | "distance";
  value: number;
  min: number | null;
  max: number | null;
}

/** Values a step offers to set, a joint's coordinates — see `geop_ops::solving::SettingsState`. */
export interface JointInfo {
  name: string;
  /** What it is called: `fore m1`, the mate `m1` of the step `fore`. */
  label: string;
  kind: string;
  /** What it joins, as the entities its frames are on read: the part it holds, then the part it moves. */
  between: string[];
  values: JointValue[];
}

/** How free each placed part is — see `geop_ops::solving::Freedom`. */
export interface MateFreedom {
  parts: Record<string, number>;
  total: number;
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
  /** The inspector whose pick tool is in hand, if one is. */
  pick_tool: string | null;
  /** The materials a part's can be picked from. */
  materials: Material[];
  /** What the parameters resolve to, by name — numbers, a table's row and columns, the colour — and why those that do not resolve fail. */
  parameters: {
    /** What the program's parameters and properties steps define. */
    defined: Parameters;
    values: Record<string, ParamValue>;
    errors: Record<string, string>;
  };
  /** What reads each parameter, by name: the steps and other parameters whose formulas do — what fails if it goes. */
  parameter_uses: Record<string, string[]>;
  operations: OperationInfo[];
  /** What can be written of the part, for [[Command]] `export`. */
  exporters: ExporterInfo[];
  /** What can be asked about it, for [[Command]] `inspect` and `pick_tool`. */
  inspectors: InspectorInfo[];
  examples: string[];
  /** Examples of several files, for [[Command]] `load_workspace_example`. */
  workspace_examples: string[];
  /** The joints of the part shown, for [[Command]] `joint`. */
  joints: JointInfo[];
  /** How free its placed parts are, if it has any. */
  freedom: MateFreedom | null;
  /** A smallest set of its constraints that cannot all hold together: none where they all hold. */
  conflicting: string[];
}

/**
 * What kind of thing a [[StructureItem]] is: `datum`, `sketch`, `solid`,
 * `face`, `part` — or what a step lists it as, `mate` (see
 * `geop_ops::ui::Listed`).
 */
export type StructureKind = string;

/** An entry of the part drawn's structure: whether it is shown, if it is drawn at all, and what it refers to. */
export interface StructureItem {
  kind: StructureKind;
  name: string;
  visible: boolean | null;
  /** The entities it is between: lit while it is pointed at. */
  entities: EntityRef[];
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
  /** What the drag or pick tool shows, while it is in hand and no step is edited. */
  tool: Presentation | null;
  /** The file `export` wrote, or `copy` copied. */
  export: ExportedFile | null;
  /** The pick tool's inspector's answer about its picks, while it is in hand — or the answer to an `inspect` command. */
  inspection: Inspection | null;
}

// ── inspecting — see `geop_ops::output::Answer` ───────────────────────────────

/** A row of an answer's table, or an item of its list. */
export interface AnswerRow {
  label: string | null;
  /** Each text of one or more lines. */
  cells: string[];
  /** Said in full while it is pointed at. */
  detail: string | null;
  /** What to light while it is pointed at. */
  light: EntityRef[];
  tone: Tone;
}

/** What pressing an answer's button does: ask the inspector again, or save what an exporter writes. */
export type Does =
  | { type: "inspect"; options: unknown }
  | { type: "export"; exporter: string; options: unknown };

export interface AnswerAction {
  label: string;
  doc: string;
  /** Whether the answer shown was asked so: lit. */
  chosen: boolean;
  does: Does;
}

/** A part of an answer. */
export type Block =
  | { type: "text"; text: string; tone: Tone }
  | { type: "list"; items: AnswerRow[] }
  | { type: "table"; head: string[]; rows: AnswerRow[]; foot: AnswerRow[] }
  | { type: "group"; title: string; detail: string | null; open: boolean; light: EntityRef[]; blocks: Block[] }
  | { type: "actions"; actions: AnswerAction[] };

/** An inspector's answer as it is shown, whatever the inspector. */
export interface Answer {
  blocks: Block[];
  /** A plane the answer is of — a planar face or datum plane picked — to cut a section with. */
  plane: Frame | null;
}

/**
 * An inspector's answer — see `geop_ops::output::Inspection`: `report` is
 * the answer as data, `shown` as it is shown; one that shows nothing is
 * shown as its report.
 */
export interface Inspection {
  kind: string;
  report: unknown;
  shown: Answer;
}

/** Apply `command` in the kernel, and get back what to show now. */
export async function send(command: Command): Promise<Update> {
  return JSON.parse(await call(JSON.stringify(command))) as Update;
}
