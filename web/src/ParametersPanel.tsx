import { useState } from "react";
import { ColorInput } from "./controls";
import type { Parameter, ParameterRow, Parameters, ParamValue } from "./geop";
import { Icon } from "./icons";

interface Props {
  parameters: Parameters;
  /** What they resolve to, and why those that do not fail. */
  resolved: { values: Record<string, ParamValue>; errors: Record<string, string> };
  enabled: boolean;
  /** The parameters are now these; settled once the part is built with them. */
  onChange: (parameters: Parameters) => Promise<unknown> | void;
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

type Table = Extract<Parameter, { type: "table" }>;

/** A table's columns and rows, edited as a grid: which row is built, each row's name and values, columns added and taken out. */
function TableEditor({ table, onChange }: { table: Table; onChange: (t: Table) => void }) {
  const { columns, rows, selected } = table;
  const setRows = (next: ParameterRow[]) => onChange({ ...table, rows: next });
  return (
    <div className="table-editor">
      <table>
        <thead>
          <tr>
            <th title="The row built">Built</th>
            <th>Row</th>
            {columns.map((c, k) => (
              <th key={k}>
                <span className="table-column">
                  <Field
                    value={c}
                    onChange={(name) => onChange({ ...table, columns: columns.map((x, j) => (j === k ? name : x)) })}
                  />
                  <button
                    className="icon-only"
                    title={`Remove the column ${c}`}
                    aria-label={`Remove the column ${c}`}
                    onClick={() =>
                      onChange({
                        ...table,
                        columns: columns.filter((_, j) => j !== k),
                        rows: rows.map((r) => ({ ...r, values: r.values.filter((_, j) => j !== k) })),
                      })
                    }
                  >
                    ✕
                  </button>
                </span>
              </th>
            ))}
            <th>
              <button
                className="small"
                title="Add a column"
                onClick={() =>
                  onChange({
                    ...table,
                    columns: [...columns, freshName(columns, "value")],
                    rows: rows.map((r) => ({ ...r, values: [...r.values, 0] })),
                  })
                }
              >
                + column
              </button>
            </th>
          </tr>
        </thead>
        <tbody>
          {rows.map((row, r) => (
            <tr key={r} className={row.name === selected ? "selected" : ""}>
              <td className="table-built">
                <input
                  type="radio"
                  title="Build this row"
                  checked={row.name === selected}
                  onChange={() => onChange({ ...table, selected: row.name })}
                />
              </td>
              <td>
                <Field
                  value={row.name}
                  onChange={(name) =>
                    onChange({
                      ...table,
                      selected: selected === row.name ? name : selected,
                      rows: rows.map((x, j) => (j === r ? { ...x, name } : x)),
                    })
                  }
                />
              </td>
              {row.values.map((v, k) => (
                <td key={k}>
                  <Field
                    value={String(v)}
                    onChange={(text) => {
                      const n = Number(text);
                      if (!Number.isNaN(n))
                        setRows(
                          rows.map((x, j) => (j === r ? { ...x, values: x.values.map((y, m) => (m === k ? n : y)) } : x)),
                        );
                    }}
                  />
                </td>
              ))}
              <td>
                <button
                  className="icon-only"
                  title="Remove the row"
                  aria-label={`Remove the row ${row.name}`}
                  disabled={rows.length <= 1}
                  onClick={() => {
                    const left = rows.filter((_, j) => j !== r);
                    onChange({ ...table, rows: left, selected: row.name === selected ? left[0].name : selected });
                  }}
                >
                  ✕
                </button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      <button
        className="small"
        onClick={() => setRows([...rows, { name: freshName(rows.map((r) => r.name), "row"), values: columns.map(() => 0) }])}
      >
        + row
      </button>
    </div>
  );
}

/** One parameter's details, in a popup: its name, and its formula and slider range or its table. */
function ParameterDialog({
  parameter,
  value,
  error,
  onChange,
  onRemove,
  onClose,
}: {
  parameter: Parameter;
  value: ParamValue | undefined;
  error: string | undefined;
  onChange: (p: Parameter) => void;
  onRemove: () => void;
  onClose: () => void;
}) {
  return (
    <div className="modal-backdrop" onPointerDown={(e) => e.target === e.currentTarget && onClose()}>
      <div className="modal" role="dialog" aria-label={`Parameter ${parameter.name}`}>
        <div className="modal-head">
          <Icon name={parameter.type === "table" ? "table" : "number"} />
          <h2>{parameter.type === "table" ? "Table" : "Number"}</h2>
          <button className="icon-only" onClick={onClose} title="Close" aria-label="Close">
            ✕
          </button>
        </div>
        <label className="modal-field">
          <span>Name</span>
          <Field value={parameter.name} onChange={(name) => onChange({ ...parameter, name })} />
        </label>
        {parameter.type === "number" ? (
          <>
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
          </>
        ) : (
          <>
            <p className="hint">
              A family of variants, one per row: the row built is the parameter's value, and{" "}
              <code>
                {parameter.name}.{parameter.columns[0] ?? "column"}
              </code>{" "}
              its value in a column.
            </p>
            <TableEditor table={parameter} onChange={onChange} />
          </>
        )}
        {error && <p className="op-error-text">{error}</p>}
        <div className="modal-actions">
          <button className="danger" onClick={onRemove}>
            Remove parameter
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
 * values. One compact row each; the details open in a popup.
 */
export function ParametersPanel({ parameters, resolved, enabled, onChange }: Props) {
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
            value={parameters.color ?? "#4472c4"}
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
            <button
              className="icon-only"
              title="Edit"
              aria-label={`Edit ${p.name}`}
              disabled={!enabled}
              onClick={() => setEditing(index)}
            >
              <Icon name="edit" />
            </button>
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
          disabled={!enabled}
          title="A family of variants — screw sizes, say — of which one row is built"
          onClick={() =>
            add({
              name: freshName(values.map((v) => v.name), "size"),
              type: "table",
              columns: ["diameter"],
              rows: [
                { name: "small", values: [3] },
                { name: "large", values: [5] },
              ],
              selected: "small",
            })
          }
        >
          + Table
        </button>
      </div>
      {open && editing != null && (
        <ParameterDialog
          parameter={open}
          value={resolved.values[open.name]}
          error={resolved.errors[open.name]}
          onChange={(p) => set(editing, p)}
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
