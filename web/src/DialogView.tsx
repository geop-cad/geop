import { Dropdown, SliderNumber } from "./controls";
import type { Choice, Control, DialogValue, Field, Tone } from "./geop";

interface Props {
  /** The operation's short name, and the step's id if it has one yet. */
  label: string;
  stepId: string | null;
  doc: string;
  /** What the operation shows, in order. */
  dialog: Field[];
  /** A control was used. */
  onDialog: (key: string, value: DialogValue) => void;
  preview: boolean;
  setPreview: (v: boolean) => void;
  previewError: string | null;
  error: string | null;
  /** Whether the step builds, and so can go into the program. */
  canCommit: boolean;
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

/** Options grouped as their `group`s say, in the order the groups first appear. */
function groups(options: Choice[]): [string | null, Choice[]][] {
  const out: [string | null, Choice[]][] = [];
  for (const option of options) {
    const last = out[out.length - 1];
    if (last && last[0] === option.group) last[1].push(option);
    else out.push([option.group, [option]]);
  }
  return out;
}

/**
 * The dialog of the step being edited: every control the operation shows,
 * rendered from its primitives alone — so any operation the kernel offers
 * gets its dialog without this knowing about it — and the controls every
 * step has: preview, OK, Cancel.
 */
export function DialogView({
  label,
  stepId,
  doc,
  dialog,
  onDialog,
  preview,
  setPreview,
  previewError,
  error,
  canCommit,
  onCommit,
  onCancel,
}: Props) {
  function control(key: string, c: Control) {
    const send = (value: DialogValue) => onDialog(key, value);
    switch (c.type) {
      case "heading":
        return <h3 className="dialog-heading">{c.text}</h3>;
      case "text":
        return <p className={TONES[c.tone]}>{c.text}</p>;
      case "button":
        return (
          <button
            className={[c.active ? "active" : "", c.primary ? "primary" : ""].join(" ")}
            disabled={!c.enabled}
            title={c.title ?? undefined}
            onClick={() => send({ type: "press" })}
          >
            {c.label}
          </button>
        );
      case "buttons":
        return (
          <div className="button-grid">
            {c.buttons.map((b) => (
              <button
                key={b.key}
                className={b.active ? "active" : ""}
                disabled={!b.enabled}
                title={b.title ?? undefined}
                onClick={() => onDialog(b.key, { type: "press" })}
              >
                {b.label}
              </button>
            ))}
          </div>
        );
      case "checkbox":
        return (
          <label className="row">
            <input type="checkbox" checked={c.value} onChange={(e) => send({ type: "bool", value: e.target.checked })} />
            {c.label}
          </label>
        );
      case "number":
        return c.slider ? (
          <SliderNumber
            label={c.label}
            value={c.value}
            onChange={(value) => send({ type: "number", value })}
            min={c.slider[0]}
            max={c.slider[1]}
            step={c.step}
          />
        ) : (
          <label className="row">
            {c.label}
            <NumberInput value={c.value} step={c.step} onChange={(value) => send({ type: "number", value })} />
          </label>
        );
      case "select":
        if (c.style === "dropdown") {
          return (
            <Dropdown
              label={c.label}
              value={c.value}
              options={c.options.map((o) => ({ value: o.value, label: o.label }))}
              onChange={(value) => send({ type: "choice", value })}
            />
          );
        }
        return (
          <div className="constructions">
            {groups(c.options).map(([group, options]) => (
              <fieldset key={group ?? ""}>
                {group && <legend>{group}</legend>}
                {options.map((o) => {
                  const checked = c.value === o.value;
                  return (
                    <label
                      key={o.value}
                      className={`construction${o.enabled ? "" : " disabled"}${checked ? " checked" : ""}`}
                      title={o.title ?? undefined}
                    >
                      <input
                        type="radio"
                        name={key}
                        disabled={!o.enabled}
                        checked={checked}
                        onChange={() => send({ type: "choice", value: o.value })}
                      />
                      {o.label}
                    </label>
                  );
                })}
              </fieldset>
            ))}
          </div>
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
                {item.detail && <span className="item-detail">{item.detail}</span>}
                {item.value != null && (
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
          <button className="primary" disabled={!canCommit} onClick={onCommit}>
            OK
          </button>
        </div>

        <h2>
          {label}
          {stepId && <span className="op-id"> · {stepId}</span>}
        </h2>
        <p className="hint">{doc}</p>
        {dialog.map(({ key, ...c }) => (
          <div key={key} className="field">
            {control(key, c as Control)}
          </div>
        ))}
        {error && <p className="op-error-text">{error}</p>}

        <label className="row preview-row">
          <input type="checkbox" checked={preview} onChange={(e) => setPreview(e.target.checked)} />
          Preview result in viewport
        </label>
        {previewError && <p className="op-error-text">{previewError}</p>}
      </div>
    </div>
  );
}
