import { Dropdown, SliderNumber } from "./controls";
import type {
  ArgSchema,
  Args,
  Combine,
  Construction,
  ConstructionSchema,
  DatumKind,
  EntityRef,
  OperationSchema,
  RunResult,
  Sketch,
} from "./geop";
import { COMBINE_MODES, ROLE_LABELS, defaultConstruction } from "./operationArgs";
import { entities, entityLabel, entries, inspectSelection } from "./geop";

/** How the constructions are grouped: by what they build. */
const RESULTS: { result: DatumKind; title: string }[] = [
  { result: "point", title: "Point" },
  { result: "axis", title: "Axis" },
  { result: "plane", title: "Plane" },
];

/** What `construction` needs selected, in words: `a point and a plane`. */
function needs(construction: ConstructionSchema): string {
  return construction.inputs.map((role) => ROLE_LABELS[role]).join(" and ");
}

interface Props {
  schema: OperationSchema;
  args: Args;
  setArg: (name: string, value: unknown) => void;
  /** Editing an existing step (its id), or adding a new one (`null`). */
  stepId: string | null;
  /** What the steps before this one built: the sketches and solids to choose from. */
  before: RunResult | null;
  /** The argument waiting for a pick in the viewport, if any. */
  pickArg: string | null;
  setPickArg: (name: string | null) => void;
  /** Open the sketch editor for a drawing argument. */
  onDraw: (name: string) => void;
  preview: boolean;
  setPreview: (v: boolean) => void;
  previewError: string | null;
  error: string | null;
  complete: boolean;
  onCommit: () => void;
  onCancel: () => void;
}

/**
 * The form for one step, built from its operation's schema: one control per
 * argument, chosen by the argument's kind alone — so any operation the
 * kernel registers gets a form without this knowing about it.
 */
export function OperationForm({
  schema,
  args,
  setArg,
  stepId,
  before,
  pickArg,
  setPickArg,
  onDraw,
  preview,
  setPreview,
  previewError,
  error,
  complete,
  onCommit,
  onCancel,
}: Props) {
  const togglePick = (name: string) => setPickArg(pickArg === name ? null : name);

  /** The control for a plain value — a number, a flag, a choice — of `arg`, now `value`. */
  function valueField(arg: ArgSchema, value: unknown, onChange: (value: unknown) => void) {
    const kind = arg.kind;
    switch (kind.type) {
      case "number":
        return (
          <SliderNumber
            label={arg.name}
            value={value as number}
            onChange={onChange}
            min={kind.min}
            max={kind.max}
            step={(kind.max - kind.min) / 200}
          />
        );
      case "bool":
        return (
          <label className="row">
            <input type="checkbox" checked={value as boolean} onChange={(e) => onChange(e.target.checked)} />
            {arg.name}
          </label>
        );
      case "choice":
        return (
          <Dropdown
            label={arg.name}
            value={value as string}
            options={kind.options.map((o) => ({ value: o, label: o }))}
            onChange={onChange}
          />
        );
      default:
        return null;
    }
  }

  function field(arg: ArgSchema) {
    const value = args[arg.name];
    const kind = arg.kind;
    switch (kind.type) {
      case "number":
      case "bool":
      case "choice":
        return valueField(arg, value, (v) => setArg(arg.name, v));
      case "sketch_line": {
        const sketch = before?.sketches.find((s) => s.name === args[kind.sketch]);
        const lines = sketch?.lines ?? [];
        return lines.length === 0 ? (
          <p className="hint">The sketch has no line.</p>
        ) : (
          <Dropdown
            label={arg.name}
            value={String(value ?? "")}
            options={lines.map((l) => ({
              value: String(l.id),
              label: `Line c${l.id}${l.construction ? " (construction)" : ""}`,
            }))}
            onChange={(v) => setArg(arg.name, Number(v))}
          />
        );
      }
      case "solid":
      case "face":
      case "sketch":
        return kind.type === "sketch" && (before?.sketches.length ?? 0) === 0 ? (
          <p className="hint">No sketch yet — add one first.</p>
        ) : (
          <button className={pickArg === arg.name ? "active" : ""} onClick={() => togglePick(arg.name)}>
            {arg.name}: {(value as string | null) ?? `pick a ${kind.type}…`}
          </button>
        );
      case "plane":
        return (
          <button className={pickArg === arg.name ? "active" : ""} onClick={() => togglePick(arg.name)}>
            {arg.name}: {entityLabel(value as EntityRef)}
          </button>
        );
      case "selection": {
        const selection = entities(value);
        const { roles } = inspectSelection(selection);
        const picking = pickArg === arg.name;
        return (
          <div className="selection">
            <div className="row">
              <button className={picking ? "active" : ""} onClick={() => togglePick(arg.name)}>
                {picking ? "Done picking" : `Pick ${arg.name}…`}
              </button>
              {selection.length > 0 && (
                <button className="small" onClick={() => setArg(arg.name, [])}>
                  Clear
                </button>
              )}
            </div>
            <ul className="selection-list">
              {selection.map((entity, i) => (
                <li key={JSON.stringify(entity)}>
                  <span className="selection-name" title={entityLabel(entity)}>
                    {entityLabel(entity)}
                  </span>
                  <span className="selection-roles">{roles[i]?.length ? roles[i].join(" · ") : "not found"}</span>
                  <button
                    className="selection-remove"
                    title="Remove from the selection"
                    onClick={() => setArg(arg.name, selection.filter((_, k) => k !== i))}
                  >
                    ✕
                  </button>
                </li>
              ))}
              {selection.length === 0 && <li className="hint">Nothing selected yet.</li>}
            </ul>
          </div>
        );
      }
      case "construction": {
        const construction = value as Construction;
        const { fits } = inspectSelection(entities(args[kind.selection]));
        const chosen = kind.options.find((o) => o.method === construction.method);
        return (
          <div className="constructions">
            {RESULTS.map(({ result, title }) => (
              <fieldset key={result}>
                <legend>{title}</legend>
                {kind.options
                  .filter((o) => o.result === result)
                  .map((o) => {
                    const fitting = fits.includes(o.method);
                    const checked = construction.method === o.method;
                    return (
                      <label
                        key={o.method}
                        className={`construction${fitting ? "" : " disabled"}${checked ? " checked" : ""}`}
                        title={fitting ? o.doc : `${o.doc}\n\nNeeds ${needs(o)} selected.`}
                      >
                        <input
                          type="radio"
                          name={arg.name}
                          disabled={!fitting}
                          checked={checked}
                          onChange={() => setArg(arg.name, defaultConstruction(o))}
                        />
                        {o.label}
                      </label>
                    );
                  })}
              </fieldset>
            ))}
            {chosen && (
              <div className="construction-params">
                <p className="hint">{chosen.doc}</p>
                {!fits.includes(chosen.method) && <p className="op-error-text">Needs {needs(chosen)} selected.</p>}
                {chosen.params.map((param) => (
                  <div key={param.name} title={param.doc}>
                    {valueField(param, construction[param.name], (v) => setArg(arg.name, { ...construction, [param.name]: v }))}
                  </div>
                ))}
              </div>
            )}
          </div>
        );
      }
      case "combine": {
        const combine = value as Combine;
        const target = combine.mode === "new_body" ? null : combine.target;
        const newest = before?.solids[before.solids.length - 1] ?? null;
        return (
          <>
            <Dropdown
              label={arg.name}
              value={combine.mode}
              options={COMBINE_MODES.map((m) => ({ value: m.mode, label: m.label }))}
              onChange={(mode) =>
                setArg(arg.name, mode === "new_body" ? { mode } : { mode, target: target ?? newest })
              }
            />
            {combine.mode !== "new_body" && (
              <button className={pickArg === arg.name ? "active" : ""} onClick={() => togglePick(arg.name)}>
                target: {target ?? "pick a solid…"}
              </button>
            )}
          </>
        );
      }
      case "drawing": {
        const n = entries((value as Sketch).curves).length;
        return (
          <button className="primary" onClick={() => onDraw(arg.name)}>
            {n === 0 ? "Draw sketch…" : `Edit sketch (${n} curve${n === 1 ? "" : "s"})…`}
          </button>
        );
      }
    }
  }

  return (
    <div className="popup-backdrop">
      <div className="popup">
        <div className="button-row">
          <button className="small" onClick={onCancel}>
            Cancel
          </button>
          <button className="primary" disabled={!complete} onClick={onCommit}>
            OK
          </button>
        </div>

        <h2>
          {schema.label}
          {stepId && <span className="op-id"> · {stepId}</span>}
        </h2>
        <p className="hint">{schema.doc}</p>
        {schema.args.map((arg) => (
          <div key={arg.name} className="field" title={arg.doc}>
            {field(arg)}
            <p className="hint">{arg.doc}</p>
          </div>
        ))}

        {pickArg && (
          <p className="hint pick-hint-inline">
            {schema.args.find((a) => a.name === pickArg)?.kind.type === "selection"
              ? "Click points, edges, faces and planes — and the origin's — to add them, or again to remove them…"
              : `Click in the viewport to pick ${pickArg}…`}
          </p>
        )}
        {error && <p className="op-error-text">{error}</p>}

        <label className="row preview-row">
          <input type="checkbox" checked={preview} onChange={(e) => setPreview(e.target.checked)} />
          Preview result in viewport
        </label>
        {preview && previewError && <p className="op-error-text">{previewError}</p>}
      </div>
    </div>
  );
}
