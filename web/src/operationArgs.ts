// What the app knows about arguments generically — from their kind alone,
// never from which operation they belong to: sensible defaults, whether a
// form is complete, and a one-line summary for the timeline.

import {
  EMPTY_SKETCH,
  entities,
  entityLabel,
  entries,
  inspectSelection,
  sameEntity,
  type ArgKind,
  type ArgPath,
  type ArgSchema,
  type Args,
  type Combine,
  type Construction,
  type ConstructionSchema,
  type DatumKind,
  type EntityRef,
  type Highlight,
  type OperationSchema,
  type Role,
  type RunResult,
  type Sketch,
  type SketchInfo,
} from "./geop";

/** The line of `sketch` to use by default: its first construction line, else its first line. */
export function defaultLine(sketch: SketchInfo | undefined): number | null {
  if (!sketch) return null;
  return (sketch.lines.find((l) => l.construction) ?? sketch.lines[0])?.id ?? null;
}

/** `schema`'s construction with every param at its default. */
export function defaultConstruction(schema: ConstructionSchema): Construction {
  const construction: Construction = { method: schema.method };
  for (const param of schema.params) {
    if (param.kind.type === "number" || param.kind.type === "bool") construction[param.name] = param.kind.default;
  }
  return construction;
}

/** The default value of `arg` in a new step of `schema`, given what the part built so far holds. */
function defaultValue(schema: OperationSchema, arg: ArgSchema, args: Args, run: RunResult | null): unknown {
  const sketches = run?.sketches ?? [];
  const solids = run?.solids ?? [];
  switch (arg.kind.type) {
    case "number":
    case "bool":
    case "choice":
      return arg.kind.default;
    case "sketch":
      return sketches[sketches.length - 1]?.name ?? null;
    case "sketch_line": {
      const sketch = args[arg.kind.sketch];
      return defaultLine(sketches.find((s) => s.name === sketch));
    }
    case "solid": {
      // The most recent solids, in order: the last solid argument gets the
      // newest — what a boolean right after building a tool body means.
      const solidArgs = schema.args.filter((a) => a.kind.type === "solid");
      const k = solidArgs.indexOf(arg);
      return solids[solids.length - solidArgs.length + k] ?? null;
    }
    case "face":
      return null;
    case "plane":
      return { type: "Plane", normal: "Z" } satisfies EntityRef;
    case "selection":
      return [] satisfies EntityRef[];
    case "construction":
      return defaultConstruction(arg.kind.options[0]);
    case "drawing":
      return EMPTY_SKETCH;
    case "combine": {
      // Into the newest solid, if there is one: building onto what is
      // there is the common case.
      const target = solids[solids.length - 1];
      return (target ? { mode: "union", target } : { mode: "new_body" }) satisfies Combine;
    }
  }
}

/** Arguments for a new step of `schema`, each at its default. */
export function defaultArgs(schema: OperationSchema, run: RunResult | null): Args {
  const args: Args = {};
  for (const arg of schema.args) args[arg.name] = defaultValue(schema, arg, args, run);
  return args;
}

/**
 * `args` with `name` set to `value`, and every argument that depends on it
 * brought along: a line of the sketch it names is reset to its default for
 * the new sketch, a construction over the selection it names switches to
 * the first that fits if it no longer does, and a combine argument that
 * follows its sign (see the `combine` [[ArgKind]]) joins for a positive
 * value and cuts for a negative one — unless it is in `touched`, the
 * arguments the user set themselves.
 */
export function withArg(
  schema: OperationSchema,
  args: Args,
  name: string,
  value: unknown,
  run: RunResult | null,
  touched: string[],
): Args {
  const next = { ...args, [name]: value };
  for (const arg of schema.args) {
    if (arg.kind.type === "sketch_line" && arg.kind.sketch === name) {
      next[arg.name] = defaultValue(schema, arg, next, run);
    }
    if (arg.kind.type === "construction" && arg.kind.selection === name) {
      const { fits } = inspectSelection(entities(value));
      const current = next[arg.name] as Construction;
      const first = arg.kind.options.find((o) => fits.includes(o.method));
      if (!fits.includes(current.method) && first) next[arg.name] = defaultConstruction(first);
    }
    if (arg.kind.type === "combine" && arg.kind.sign === name && !touched.includes(arg.name)) {
      const combine = next[arg.name] as Combine;
      if (combine.mode !== "new_body" && typeof value === "number" && value !== 0) {
        next[arg.name] = { ...combine, mode: value > 0 ? "union" : "difference" } satisfies Combine;
      }
    }
  }
  return next;
}

/** `args` with the value at `path` (see [[ArgPath]]) replaced by `value`; nothing else is changed or shared. */
export function withPath(args: Args, path: ArgPath, value: unknown): Args {
  const [head, ...rest] = path;
  if (rest.length === 0) return { ...args, [head]: value };
  return { ...args, [head]: withPath((args[head] ?? {}) as Args, rest, value) };
}

/**
 * Whether every argument has a value — a drawing, at least one curve; a
 * selection, at least one entity; a construction, one that fits its
 * selection.
 */
export function isComplete(schema: OperationSchema, args: Args): boolean {
  return schema.args.every((arg) => {
    const value = args[arg.name];
    if (value == null) return false;
    if (arg.kind.type === "drawing") return entries((value as Sketch).curves).length > 0;
    if (arg.kind.type === "selection") return (value as EntityRef[]).length > 0;
    if (arg.kind.type === "construction") {
      return inspectSelection(entities(args[arg.kind.selection])).fits.includes((value as Construction).method);
    }
    if (arg.kind.type === "combine") {
      const combine = value as Combine;
      return combine.mode === "new_body" || combine.target != null;
    }
    return true;
  });
}

/**
 * Which kinds of datum — on the origin gizmo, or among the part's datums
 * — an argument of `kind` can take: a plane argument takes a plane, a
 * selection any of them.
 */
export function acceptedDatums(kind: ArgKind): DatumKind[] {
  return kind.type === "plane" ? ["plane"] : kind.type === "selection" ? ["point", "axis", "plane"] : [];
}

/**
 * The value an argument of `kind`, now `current`, takes from picking `hit`
 * — or `null` if it cannot take it. A selection gains the entity, or loses
 * it if it had it.
 */
export function pickedValue(kind: ArgKind, current: unknown, hit: Highlight): unknown {
  switch (kind.type) {
    case "plane":
      return hit.type === "Face" || hit.type === "Plane" || hit.type === "Datum" ? hit : null;
    case "selection": {
      if (hit.type === "Solid" || hit.type === "Sketch") return null;
      const selection = entities(current);
      return selection.some((e) => sameEntity(e, hit))
        ? selection.filter((e) => !sameEntity(e, hit))
        : [...selection, hit];
    }
    case "combine": {
      const combine = current as Combine;
      return "name" in hit
        ? ({ mode: combine.mode === "new_body" ? "union" : combine.mode, target: hit.name } satisfies Combine)
        : null;
    }
    default:
      return "name" in hit ? hit.name : null;
  }
}

/** What a construction needs an entity to be, in words. */
export const ROLE_LABELS: Record<Role, string> = {
  point: "a point",
  line: "a line",
  plane: "a plane",
  edge: "an edge",
  circle: "a circular edge",
  round: "a circular edge or a round face",
};

/** How each combine mode is offered. */
export const COMBINE_MODES: { mode: Combine["mode"]; label: string }[] = [
  { mode: "new_body", label: "New body" },
  { mode: "union", label: "Join" },
  { mode: "difference", label: "Cut" },
  { mode: "intersection", label: "Intersect" },
];

/** One argument's value in a few characters. */
function valueSummary(arg: ArgSchema, value: unknown): string {
  if (value == null) return "?";
  switch (arg.kind.type) {
    case "number":
      return (value as number).toFixed(2);
    case "bool":
      return value ? "yes" : "no";
    case "sketch_line":
      return `c${value as number}`;
    case "plane":
      return entityLabel(value as EntityRef);
    case "selection":
      return `[${entities(value).map(entityLabel).join(", ")}]`;
    case "construction": {
      const method = (value as Construction).method;
      return arg.kind.options.find((o) => o.method === method)?.label ?? method;
    }
    case "drawing": {
      const n = entries((value as Sketch).curves).length;
      return `${n} curve${n === 1 ? "" : "s"}`;
    }
    case "combine": {
      const combine = value as Combine;
      const label = COMBINE_MODES.find((m) => m.mode === combine.mode)?.label ?? combine.mode;
      return combine.mode === "new_body" ? label : `${label.toLowerCase()} ${combine.target ?? "?"}`;
    }
    default:
      return String(value);
  }
}

/** A step's arguments in one line: `sketch=outline, distance=1.00`. */
export function argsSummary(schema: OperationSchema | undefined, args: Args): string {
  if (!schema) return JSON.stringify(args);
  return schema.args.map((arg) => `${arg.name}=${valueSummary(arg, args[arg.name])}`).join(", ");
}
