import { ColorInput, Dropdown, SearchSelect, SliderNumber } from "./controls";
import { Icon } from "./icons";
import { entityLabel, type Action, type Control, type StepState, type Tone, type Unit, type Value } from "./geop";

interface Props {
  /** The step being edited, as the kernel shows it. */
  step: StepState;
  /** A field was used; settled once the step is updated. */
  onDialog: (key: string, value: Value) => Promise<unknown> | void;
  setPreview: (v: boolean) => void;
  /** Why the last command was refused, if it was. */
  error: string | null;
  onCommit: () => void;
  onCancel: () => void;
}

/** The class a text of `tone` reads in. */
const TONES: Record<Tone, string> = {
  normal: "",
  hint: "hint",
  error: "op-error-text",
  success: "status-ok",
};

/**
 * A number typed in place: Enter applies it, leaving the field without
 * Enter puts the value back. Keyed by the value, so a change from elsewhere
 * shows.
 */
function NumberInput({ value, step, onChange }: { value: number; step: number; onChange: (v: number) => void }) {
  const shown = String(Number(value.toFixed(6)));
  return (
    <input
      key={shown}
      type="number"
      step={step}
      defaultValue={shown}
      title="Enter applies"
      onClick={(e) => e.stopPropagation()}
      onKeyDown={(e) => {
        if (e.key === "Escape") e.currentTarget.blur();
        if (e.key !== "Enter") return;
        const v = Number(e.currentTarget.value);
        if (e.currentTarget.value !== "" && !Number.isNaN(v)) onChange(v);
      }}
      onBlur={(e) => (e.currentTarget.value = shown)}
    />
  );
}

/**
 * Text typed in place — a value, or a formula of the part's parameters:
 * Enter applies it, leaving the field without Enter puts it back. Keyed by
 * the text, so a change from elsewhere shows.
 */
function TextInput({ value, onChange }: { value: string; onChange: (v: string) => void }) {
  return (
    <input
      key={value}
      type="text"
      className="formula-input"
      defaultValue={value}
      title="A number, or a formula of the parameters · Enter applies"
      onClick={(e) => e.stopPropagation()}
      onKeyDown={(e) => {
        if (e.key === "Escape") e.currentTarget.blur();
        if (e.key === "Enter" && e.currentTarget.value.trim() !== "") onChange(e.currentTarget.value);
      }}
      onBlur={(e) => (e.currentTarget.value = value)}
    />
  );
}

/**
 * A number that may be given as a formula of the parameters, typed as in
 * the parameters panel: its text edited in place (Enter applies), with a
 * slider beside it while the kernel offers one — a plain value — and what
 * a formula comes to, or why it does not evaluate, while it is one.
 */
function FormulaNumber({
  label,
  c,
  send,
}: {
  label: string;
  c: Extract<Control, { type: "number" }>;
  send: (value: Value) => void;
}) {
  const text = c.text ?? "";
  const formula = text.trim() !== "" && Number.isNaN(Number(text));
  return (
    <>
      <div className="slider-number formula-number">
        <span className="slider-number-label">{label}</span>
        {c.range ? (
          <input
            type="range"
            min={c.range[0]}
            max={c.range[1]}
            step={c.step}
            value={c.value}
            onChange={(e) => send({ type: "number", value: Number(e.target.value) })}
          />
        ) : (
          <span className="hint formula-value">
            {formula && c.error == null ? `= ${Number(c.value.toFixed(6))}` : ""}
          </span>
        )}
        <TextInput value={text} onChange={(value) => send({ type: "text", value })} />
      </div>
      {c.error && <p className="op-error-text">{c.error}</p>}
    </>
  );
}

/** Actions grouped as their `group`s say, in the order the groups first appear. */
function groups(actions: Action[]): [string | null, Action[]][] {
  const out: [string | null, Action[]][] = [];
  for (const action of actions) {
    const last = out[out.length - 1];
    if (last && last[0] === action.group) last[1].push(action);
    else out.push([action.group, [action]]);
  }
  return out;
}

/** How a number of `unit` reads after its value. */
const UNITS: Record<Unit, string> = { length: "", angle: "°", fraction: "", count: "" };

/** What a reference field's button says: what it holds, or that it waits for a pick. */
function referenced(c: Extract<Control, { type: "reference" }>): string {
  if (c.value.length === 0) return c.armed ? "click in the viewport…" : "pick…";
  if (c.multiple) return c.armed ? "click to add or take out…" : "pick more…";
  return entityLabel(c.value[0].entity);
}

/**
 * The dialog of the step being edited: every field the operation shows,
 * rendered from its primitives alone — so any operation the kernel offers
 * gets its dialog without this knowing about it — and the controls every
 * step has: preview, OK, Cancel.
 */
export function DialogView({ step, onDialog, setPreview, error, onCommit, onCancel }: Props) {
  function control(key: string, c: Control) {
    const send = (value: Value) => onDialog(key, value);
    switch (c.type) {
      case "heading":
        return <h3 className="dialog-heading">{c.text}</h3>;
      case "text":
        return <p className={TONES[c.tone]}>{c.text}</p>;
      case "reference":
        return (
          <div className="reference">
            <button
              className={c.armed ? "active" : ""}
              title={c.multiple ? "Pick in the viewport — again to take out" : "Pick in the viewport"}
              onClick={() => send({ type: "press" })}
            >
              {c.label}: {referenced(c)}
            </button>
            {c.value.some((p) => p.tone === "error") && !c.multiple && (
              <p className="op-error-text">{c.value.find((p) => p.tone === "error")?.detail}</p>
            )}
            {c.multiple && c.value.length > 0 && (
              <ol className="dialog-list">
                {c.value.map((p, i) => (
                  <li key={i} className={p.tone === "error" ? "failed" : ""}>
                    <span className="item-label" title={entityLabel(p.entity)}>
                      {entityLabel(p.entity)}
                    </span>
                    {p.detail && <span className="item-detail">{p.detail}</span>}
                    <button className="item-remove" title="Take out" onClick={() => send({ type: "remove_at", value: i })}>
                      ✕
                    </button>
                  </li>
                ))}
              </ol>
            )}
            {c.multiple && c.value.length > 0 && (
              <button className="small" onClick={() => send({ type: "clear" })}>
                Clear
              </button>
            )}
          </div>
        );
      case "actions": {
        const button = (a: Action) =>
          a.icon != null ? (
            <button
              key={a.value}
              className={["icon-button", a.active ? "active" : ""].join(" ")}
              disabled={!a.enabled}
              title={a.title ?? a.label}
              aria-label={a.label}
              onClick={() => send({ type: "choice", value: a.value })}
            >
              <Icon name={a.icon} />
            </button>
          ) : (
            <button
              key={a.value}
              className={a.active ? "active" : ""}
              disabled={!a.enabled}
              title={a.title ?? undefined}
              onClick={() => send({ type: "choice", value: a.value })}
            >
              {a.label}
            </button>
          );
        if (c.actions.every((a) => a.icon != null))
          return (
            <div className="icon-toolbar">
              {groups(c.actions).map(([group, actions]) => (
                <div key={group ?? ""} className="icon-group" role="group" aria-label={group ?? undefined}>
                  {group && <span className="icon-group-label">{group}</span>}
                  <div className="icon-group-buttons">{actions.map(button)}</div>
                </div>
              ))}
            </div>
          );
        if (c.actions.every((a) => a.group == null)) return <div className="button-grid">{c.actions.map(button)}</div>;
        return (
          <div className="constructions">
            {groups(c.actions).map(([group, actions]) => (
              <fieldset key={group ?? ""}>
                {group && <legend>{group}</legend>}
                {actions.map(button)}
              </fieldset>
            ))}
          </div>
        );
      }
      case "checkbox":
        return (
          <label className="row">
            <input type="checkbox" checked={c.value} onChange={(e) => send({ type: "bool", value: e.target.checked })} />
            {c.label}
          </label>
        );
      case "number": {
        const label = UNITS[c.unit] ? `${c.label} (${UNITS[c.unit]})` : c.label;
        if (c.text != null) return <FormulaNumber label={label} c={c} send={send} />;
        return c.range ? (
          <SliderNumber
            label={label}
            value={c.value}
            onChange={(value) => send({ type: "number", value })}
            min={c.range[0]}
            max={c.range[1]}
            step={c.step}
          />
        ) : (
          <label className="row">
            {label}
            <NumberInput value={c.value} step={c.step} onChange={(value) => send({ type: "number", value })} />
          </label>
        );
      }
      case "select":
        return c.searchable ? (
          <SearchSelect
            label={c.label}
            value={c.value}
            options={c.options}
            onChange={(value) => send({ type: "choice", value })}
          />
        ) : (
          <Dropdown
            label={c.label}
            value={c.value}
            options={c.options}
            onChange={(value) => send({ type: "choice", value })}
          />
        );
      case "color":
        return (
          <label className="row">
            {c.label}
            <ColorInput value={c.value} onChange={(color) => send({ type: "text", value: color })} />
          </label>
        );
      case "list":
        return (
          <ol className="dialog-list">
            {c.items.length === 0 && <li className="hint">{c.empty}</li>}
            {c.items.map((item) => (
              <li
                key={item.key}
                className={[item.selected ? "selected" : "", item.tone === "error" ? "failed" : ""].join(" ")}
                onClick={() => onDialog(item.key, { type: "press" })}
              >
                <span className="item-label" title={item.label}>
                  {item.label}
                </span>
                {item.detail && (
                  <span className="item-detail" title={item.detail}>
                    {item.detail}
                  </span>
                )}
                {item.text != null && (
                  <TextInput value={item.text} onChange={(value) => onDialog(item.key, { type: "text", value })} />
                )}
                {item.text == null && item.value != null && (
                  <NumberInput
                    value={item.value}
                    step={0.1}
                    onChange={(value) => onDialog(item.key, { type: "number", value })}
                  />
                )}
                {item.removable && (
                  <button
                    className="item-remove"
                    title="Remove"
                    onClick={(e) => {
                      e.stopPropagation();
                      onDialog(item.key, { type: "remove" });
                    }}
                  >
                    ✕
                  </button>
                )}
              </li>
            ))}
          </ol>
        );
    }
  }

  return (
    <div className="popup-backdrop">
      <div className="popup">
        <div className="button-row">
          <button className="small" onClick={onCancel}>
            Cancel
          </button>
          <button
            className="primary"
            disabled={step.error != null || step.missing.length > 0}
            title={step.missing.length > 0 ? `Pick the ${step.missing.join(" and ")} first` : undefined}
            onClick={onCommit}
          >
            OK
          </button>
        </div>

        <h2 title={step.doc}>
          {step.label}
          {step.id && <span className="op-id"> · {step.id}</span>}
        </h2>
        {step.presentation.dialog.map(({ key, ...c }) => (
          <div key={key} className="field">
            {control(key, c as Control)}
          </div>
        ))}
        {error && <p className="op-error-text">{error}</p>}

        <label className="row preview-row">
          <input type="checkbox" checked={step.preview} onChange={(e) => setPreview(e.target.checked)} />
          Preview result in viewport
        </label>
        {step.missing.length > 0 ? (
          <p className="hint">Pick the {step.missing.join(" and ")} in the viewport to continue.</p>
        ) : (
          step.error && <p className="op-error-text">{step.error}</p>
        )}
      </div>
    </div>
  );
}
