import { useState } from "react";
import { ColorInput } from "./controls";
import type { Material, Parameter, Parameters, ParamValue } from "./geop";
import { Icon } from "./icons";
import { FACE_COLOR } from "./partScene";

interface Props {
  parameters: Parameters;
  /** What they resolve to, and why those that do not fail. */
  resolved: { values: Record<string, ParamValue>; errors: Record<string, string> };
  enabled: boolean;
  /** The parameters are now these; settled once the part is built with them. */
  onChange: (parameters: Parameters) => Promise<unknown> | void;
  /** What reads each parameter, by name: steps, and other parameters. */
  uses: Record<string, string[]>;
  /** Rename the parameter `from` to `to`, and every formula reading it. */
  onRename: (from: string, to: string) => void;
  /** The materials the part's can be picked from; any other is given by its density. */
  materials: Material[];
  /** Whether no step is edited: a table's step can only be opened then. */
  idle: boolean;
  /** The step each table is looked up in, by the table's name. */
  tableSteps: Record<string, string>;
  /** Open the step `id`: a table is edited in its own step's dialog, a popup. */
  onOpenStep: (id: string) => void;
  /** The parameters are now these, with the table `name` added: set, then opened to be filled in. */
  onAddTable: (parameters: Parameters, name: string) => void;
}

/**
 * The part's material: one of `materials`, or any other by its density in
 * kg/m³ — what its mass and inertia are computed with. None given, the part
 * is weighed as water.
 */
function MaterialRow({
  material,
  materials,
  enabled,
  onChange,
}: {
  material: Material | null | undefined;
  materials: Material[];
  enabled: boolean;
  onChange: (material: Material | null) => void;
}) {
  const known = material && materials.find((m) => m.name === material.name && m.density === material.density);
  const choice = !material ? "" : known ? material.name : "custom";
  return (
    <div className="parameter-row" title="What the part is made of: its mass and inertia are computed with its density">
      <span className="parameter-kind">
        <Icon name="mass" />
      </span>
      <span className="parameter-name">material</span>
      <select
        className="parameter-formula"
        value={choice}
        disabled={!enabled}
        aria-label="The part's material"
        onChange={(e) => {
          const name = e.target.value;
          if (name === "") onChange(null);
          else if (name === "custom") onChange({ name: "Custom", density: material?.density ?? 1000 });
          else onChange(materials.find((m) => m.name === name) ?? null);
        }}
      >
        <option value="">none (water)</option>
        {materials.map((m) => (
          <option key={m.name} value={m.name}>
            {m.name} — {m.density} kg/m³
          </option>
        ))}
        <option value="custom">custom density…</option>
      </select>
      {choice === "custom" && material && (
        <input
          key={material.density}
          className="parameter-value"
          type="number"
          min={0}
          defaultValue={material.density}
          title="Density, kg/m³"
          aria-label="Density in kg/m³"
          disabled={!enabled}
          onBlur={(e) => {
            const density = Number(e.target.value);
            if (density > 0 && density !== material.density) onChange({ ...material, density });
          }}
          onKeyDown={(e) => {
            if (e.key === "Enter") e.currentTarget.blur();
          }}
        />
      )}
    </div>
  );
}

/**
 * Text edited in place, given when Enter is pressed or the field is left —
 * so a formula is not rebuilt for every key. Keyed by its value, so a
 * change from elsewhere shows.
 */
function Field({
  value,
  onChange,
  placeholder,
  className,
  title,
}: {
  value: string;
  onChange: (v: string) => void;
  placeholder?: string;
  className?: string;
  title?: string;
}) {
  const give = (e: { currentTarget: HTMLInputElement }) => {
    if (e.currentTarget.value !== value) onChange(e.currentTarget.value);
  };
  return (
    <input
      key={value}
      type="text"
      className={className}
      defaultValue={value}
      placeholder={placeholder}
      title={title}
      onBlur={give}
      onKeyDown={(e) => {
        if (e.key === "Enter") give(e);
        if (e.key === "Escape") {
          e.currentTarget.value = value;
          e.currentTarget.blur();
        }
      }}
    />
  );
}

/** A number typed in place; empty for none. */
function OptionalNumber({
  value,
  onChange,
  placeholder,
}: {
  value?: number | null;
  onChange: (v: number | null) => void;
  placeholder: string;
}) {
  return (
    <Field
      value={value == null ? "" : String(value)}
      placeholder={placeholder}
      onChange={(text) => {
        const v = Number(text);
        onChange(text.trim() === "" || Number.isNaN(v) ? null : v);
      }}
    />
  );
}

/** A name none of `names` is yet: `base`, or `base2`, `base3`, ... */
function freshName(names: string[], base: string): string {
  const taken = new Set(names);
  if (!taken.has(base)) return base;
  for (let n = 2; ; n++) if (!taken.has(`${base}${n}`)) return `${base}${n}`;
}

/** How a resolved value reads. */
function shown(value: ParamValue | undefined): string {
  if (typeof value === "number") return String(Number(value.toFixed(6)));
  if (typeof value === "string") return value;
  return "";
}

type NumberParameter = Extract<Parameter, { type: "number" }>;

/**
 * One number parameter's details, in a popup: its name, its formula and
 * slider range, and what reads it. A new name is given everywhere it is
 * read; removing one that is read says what will fail first. A table is
 * edited in its own step's dialog instead.
 */
function ParameterDialog({
  parameter,
  value,
  error,
  readers,
  onChange,
  onRename,
  onRemove,
  onClose,
}: {
  parameter: NumberParameter;
  value: ParamValue | undefined;
  error: string | undefined;
  /** The steps and other parameters that read it. */
  readers: string[];
  onChange: (p: Parameter) => void;
  onRename: (name: string) => void;
  onRemove: () => void;
  onClose: () => void;
}) {
  const [removing, setRemoving] = useState(false);
  return (
    <div className="modal-backdrop" onPointerDown={(e) => e.target === e.currentTarget && onClose()}>
      <div className="modal" role="dialog" aria-label={`Parameter ${parameter.name}`}>
        <div className="modal-head">
          <Icon name="number" />
          <h2>Number</h2>
          <button className="icon-only" onClick={onClose} title="Close" aria-label="Close">
            ✕
          </button>
        </div>
        <label className="modal-field">
          <span>Name</span>
          <Field
            value={parameter.name}
            title="Renamed in every formula that reads it, too"
            onChange={(name) => onRename(name.trim())}
          />
        </label>
        <p className="hint parameter-readers">
          {readers.length > 0 ? `Read by ${readers.join(", ")}.` : "Nothing reads it yet."}
        </p>
        <label className="modal-field">
          <span>Value</span>
          <Field
            className="formula"
            value={parameter.expression}
            placeholder="a number or formula"
            title="A number, or a formula of the other parameters: width / 2, sqrt(a^2 + b^2), screw.diameter"
            onChange={(expression) => onChange({ ...parameter, expression })}
          />
          <span className="parameter-value">{shown(value)}</span>
        </label>
        <div className="modal-field" title="What a slider offers when the part is placed">
          <span>Slider</span>
          <OptionalNumber value={parameter.min} placeholder="min" onChange={(min) => onChange({ ...parameter, min })} />
          <span className="hint">to</span>
          <OptionalNumber value={parameter.max} placeholder="max" onChange={(max) => onChange({ ...parameter, max })} />
        </div>
        {error && <p className="op-error-text">{error}</p>}
        {removing && (
          <p className="warning-text">
            {readers.join(", ")} {readers.length === 1 ? "reads" : "read"} {parameter.name}, and will fail without it.
          </p>
        )}
        <div className="modal-actions">
          <button
            className="danger"
            onClick={() => (readers.length > 0 && !removing ? setRemoving(true) : onRemove())}
          >
            {removing ? "Remove anyway" : "Remove parameter"}
          </button>
          <button className="primary" onClick={onClose}>
            Done
          </button>
        </div>
      </div>
    </div>
  );
}

/**
 * The program's parameters: the part's colour, numbers given as formulas
 * of the other parameters, and tables of variants — a family of parts, one
 * row each — whose built row's columns read as `name.column`. Sketch
 * dimensions read them by name; a program placing the part gives them other
 * values. One compact row each; a number's details open in a popup, a table
 * in its own step's dialog, a popup of the same make.
 */
export function ParametersPanel({
  parameters,
  resolved,
  enabled,
  idle,
  onChange,
  materials,
  uses,
  tableSteps,
  onRename,
  onOpenStep,
  onAddTable,
}: Props) {
  const values = parameters.values ?? [];
  const [editing, setEditing] = useState<number | null>(null);
  const set = (index: number, parameter: Parameter) =>
    onChange({ ...parameters, values: values.map((p, i) => (i === index ? parameter : p)) });
  const add = (parameter: Parameter) => {
    onChange({ ...parameters, values: [...values, parameter] });
    setEditing(values.length);
  };
  const open = editing != null ? values[editing] : undefined;
  return (
    <div className="parameters">
      <div className="parameter-row">
        <span className="parameter-kind">
          <ColorInput
            value={parameters.color ?? `#${FACE_COLOR.toString(16).padStart(6, "0")}`}
            title="The part's colour"
            onChange={(color) => onChange({ ...parameters, color })}
          />
        </span>
        <span className="parameter-name">color</span>
        <span className="parameter-value">{parameters.color ?? "default"}</span>
        {parameters.color && (
          <button
            className="icon-only"
            title="Back to the default colour"
            aria-label="Back to the default colour"
            disabled={!enabled}
            onClick={() => onChange({ ...parameters, color: null })}
          >
            ✕
          </button>
        )}
      </div>
      <MaterialRow
        material={parameters.material}
        materials={materials}
        enabled={enabled}
        onChange={(material) => onChange({ ...parameters, material })}
      />
      {values.map((p, index) => {
        const error = resolved.errors[p.name];
        return (
          <div key={index} className={["parameter-row", error ? "failed" : ""].join(" ")} title={error}>
            <span className="parameter-kind">
              <Icon name={p.type === "table" ? "table" : "number"} />
            </span>
            <span className="parameter-name">{p.name}</span>
            {p.type === "number" ? (
              <Field
                className="parameter-formula"
                value={p.expression}
                placeholder="formula"
                title="A number, or a formula of the other parameters"
                onChange={(expression) => set(index, { ...p, expression })}
              />
            ) : (
              <select
                className="parameter-formula"
                value={p.selected}
                title="The row built"
                disabled={!enabled}
                onChange={(e) => set(index, { ...p, selected: e.target.value })}
              >
                {p.rows.map((r) => (
                  <option key={r.name} value={r.name}>
                    {r.name}
                  </option>
                ))}
              </select>
            )}
            {p.type === "number" && <span className="parameter-value">{error ? "!" : shown(resolved.values[p.name])}</span>}
            {p.type === "number" ? (
              <button
                className="icon-only"
                title="Edit"
                aria-label={`Edit ${p.name}`}
                disabled={!enabled}
                onClick={() => setEditing(index)}
              >
                <Icon name="edit" />
              </button>
            ) : (
              <button
                className="icon-only"
                title={idle ? "Edit the table in its step" : "Finish the step being edited first"}
                aria-label={`Edit ${p.name}`}
                disabled={!enabled || !idle || tableSteps[p.name] == null}
                onClick={() => onOpenStep(tableSteps[p.name])}
              >
                <Icon name="edit" />
              </button>
            )}
          </div>
        );
      })}
      <div className="parameter-add">
        <button
          className="small"
          disabled={!enabled}
          onClick={() => add({ name: freshName(values.map((v) => v.name), "length"), type: "number", expression: "10" })}
        >
          + Number
        </button>
        <button
          className="small"
          title="A family of variants — screw sizes, say — of which one row is built"
          disabled={!enabled || !idle}
          onClick={() => {
            const name = freshName(values.map((v) => v.name), "size");
            const table: Parameter = {
              name,
              type: "table",
              columns: ["diameter"],
              rows: [
                { name: "small", values: [3] },
                { name: "large", values: [5] },
              ],
              selected: "small",
            };
            onAddTable({ ...parameters, values: [...values, table] }, name);
          }}
        >
          + Table
        </button>
      </div>
      {open && open.type === "number" && editing != null && (
        <ParameterDialog
          parameter={open}
          value={resolved.values[open.name]}
          error={resolved.errors[open.name]}
          readers={uses[open.name] ?? []}
          onChange={(p) => set(editing, p)}
          onRename={(name) => onRename(open.name, name)}
          onRemove={() => {
            onChange({ ...parameters, values: values.filter((_, i) => i !== editing) });
            setEditing(null);
          }}
          onClose={() => setEditing(null)}
        />
      )}
    </div>
  );
}
