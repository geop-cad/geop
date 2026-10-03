import type { Parameter, ParameterRow, Parameters, ParamValue } from "./geop";

interface Props {
  parameters: Parameters;
  /** What they resolve to, and why those that do not fail. */
  resolved: { values: Record<string, ParamValue>; errors: Record<string, string> };
  enabled: boolean;
  /** The parameters are now these. */
  onChange: (parameters: Parameters) => void;
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
function OptionalNumber({ value, onChange, placeholder }: { value?: number | null; onChange: (v: number | null) => void; placeholder: string }) {
  return (
    <Field
      className="parameter-bound"
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

/**
 * The program's parameters: the part's colour, numbers given as formulas
 * of the other parameters, and tables of variants — a family of
 * parts, one row each — whose selected row's columns read as
 * `name.column`. Sketch dimensions read them by name; a program placing
 * the part gives them other values.
 */
export function ParametersPanel({ parameters, resolved, enabled, onChange }: Props) {
  const values = parameters.values ?? [];
  const set = (index: number, parameter: Parameter) =>
    onChange({ ...parameters, values: values.map((p, i) => (i === index ? parameter : p)) });
  const remove = (index: number) => onChange({ ...parameters, values: values.filter((_, i) => i !== index) });
  const add = (parameter: Parameter) => onChange({ ...parameters, values: [...values, parameter] });

  const table = (index: number, p: Extract<Parameter, { type: "table" }>) => {
    const setRows = (rows: ParameterRow[]) => set(index, { ...p, rows });
    return (
      <div className="parameter-table-wrap">
        <table className="parameter-table">
          <thead>
            <tr>
              <th />
              <th>row</th>
              {p.columns.map((c, k) => (
                <th key={k}>
                  <span className="parameter-column">
                    <Field
                      value={c}
                      onChange={(name) => set(index, { ...p, columns: p.columns.map((x, j) => (j === k ? name : x)) })}
                    />
                    <button
                      className="item-remove"
                      title={`Remove the column ${c}`}
                      aria-label={`Remove the column ${c}`}
                      onClick={() =>
                        set(index, {
                          ...p,
                          columns: p.columns.filter((_, j) => j !== k),
                          rows: p.rows.map((r) => ({ ...r, values: r.values.filter((_, j) => j !== k) })),
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
                    set(index, {
                      ...p,
                      columns: [...p.columns, freshName(p.columns, "value")],
                      rows: p.rows.map((r) => ({ ...r, values: [...r.values, 0] })),
                    })
                  }
                >
                  +
                </button>
              </th>
            </tr>
          </thead>
          <tbody>
            {p.rows.map((row, r) => (
              <tr key={r} className={row.name === p.selected ? "selected" : ""}>
                <td>
                  <input
                    type="radio"
                    title="Build this row"
                    checked={row.name === p.selected}
                    onChange={() => set(index, { ...p, selected: row.name })}
                  />
                </td>
                <td>
                  <Field
                    value={row.name}
                    onChange={(name) =>
                      set(index, {
                        ...p,
                        selected: p.selected === row.name ? name : p.selected,
                        rows: p.rows.map((x, j) => (j === r ? { ...x, name } : x)),
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
                          setRows(p.rows.map((x, j) => (j === r ? { ...x, values: x.values.map((y, m) => (m === k ? n : y)) } : x)));
                      }}
                    />
                  </td>
                ))}
                <td>
                  <button
                    className="item-remove"
                    title="Remove the row"
                    disabled={p.rows.length <= 1}
                    onClick={() => {
                      const rows = p.rows.filter((_, j) => j !== r);
                      set(index, { ...p, rows, selected: row.name === p.selected ? rows[0].name : p.selected });
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
          onClick={() =>
            setRows([
              ...p.rows,
              { name: freshName(p.rows.map((r) => r.name), "row"), values: p.columns.map(() => 0) },
            ])
          }
        >
          + row
        </button>
      </div>
    );
  };

  return (
    <fieldset className="parameters" disabled={!enabled}>
      <div className="parameter parameter-color">
        <span className="parameter-name">color</span>
        <input
          type="color"
          value={parameters.color ?? "#4472c4"}
          title="The part's colour"
          onChange={(e) => onChange({ ...parameters, color: e.target.value })}
        />
        {parameters.color && (
          <button className="small" title="Back to the default colour" onClick={() => onChange({ ...parameters, color: null })}>
            default
          </button>
        )}
      </div>
      {values.map((p, index) => {
        const error = resolved.errors[p.name];
        return (
          <div key={index} className={["parameter", error ? "failed" : ""].join(" ")}>
            <div className="parameter-head">
              <Field className="parameter-name" value={p.name} onChange={(name) => set(index, { ...p, name })} title="Its name, as formulas read it" />
              {p.type === "number" ? (
                <>
                  <span className="parameter-eq">=</span>
                  <Field
                    className="parameter-expression"
                    value={p.expression}
                    placeholder="a number or formula"
                    title="A number, or a formula of the other parameters: width / 2, sqrt(a^2 + b^2), screw.diameter"
                    onChange={(expression) => set(index, { ...p, expression })}
                  />
                  <span className="parameter-value" title="What it is now">
                    {shown(resolved.values[p.name])}
                  </span>
                </>
              ) : (
                <span className="parameter-value" title="The row built">
                  {shown(resolved.values[p.name])}
                </span>
              )}
              <button className="item-remove" title="Remove the parameter" onClick={() => remove(index)}>
                ✕
              </button>
            </div>
            {p.type === "number" && (
              <div className="parameter-bounds" title="What a slider offers when the part is placed">
                <OptionalNumber value={p.min} placeholder="min" onChange={(min) => set(index, { ...p, min })} />
                <span>…</span>
                <OptionalNumber value={p.max} placeholder="max" onChange={(max) => set(index, { ...p, max })} />
              </div>
            )}
            {p.type === "table" && table(index, p)}
            {error && <p className="op-error-text">{error}</p>}
          </div>
        );
      })}
      <div className="button-row">
        <button className="small" onClick={() => add({ name: freshName(values.map((v) => v.name), "length"), type: "number", expression: "10" })}>
          + number
        </button>
        <button
          className="small"
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
          + table
        </button>
      </div>
    </fieldset>
  );
}
