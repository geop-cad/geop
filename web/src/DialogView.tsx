import { useState, type ReactNode } from "react";
import { ColorInput, Dropdown, SearchSelect } from "./controls";
import { FileChooser } from "./FileChooser";
import { Icon } from "./icons";
import { formatShortcut } from "./shortcuts";
import { TableField } from "./TableField";
import { elementLabel, type Action, type Control, type StepState, type Tone, type Unit, type Value } from "./geop";

interface Props {
  /** The step being edited, as the kernel shows it. */
  step: StepState;
  /** A field was used; settled once the step is updated. */
  onDialog: (key: string, value: Value) => Promise<unknown> | void;
  /**
   * Store `file`, chosen from disk for a file field, next to the program:
   * its path relative to the program, or `null` if it could not be.
   */
  onChooseFile: (file: File) => Promise<string | null>;
  setPreview: (v: boolean) => void;
  /** Why the last command was refused, if it was. */
  error: string | null;
  onCommit: () => void;
  onCancel: () => void;
  /** A file the step can be saved as was asked for: what the exporter `exporter` writes in `format`. */
  onDownload: (exporter: string, format: string) => void;
}

/** The class a text of `tone` reads in. */
const TONES: Record<Tone, string> = {
  normal: "",
  hint: "hint",
  error: "op-error-text",
  success: "status-ok",
};

/** How a number of `unit` reads after its value; nothing for a bare number. */
const UNITS: Record<Unit, string> = { length: "mm", angle: "°", fraction: "", count: "" };

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
function TextInput({
  value,
  placeholder,
  title,
  formula,
  onChange,
}: {
  value: string;
  placeholder?: string;
  title?: string;
  /** Whether it takes a formula: shown in a monospaced face. */
  formula?: boolean;
  onChange: (v: string) => void;
}) {
  return (
    <input
      key={value}
      type="text"
      className={formula ? "formula-input" : "text-input"}
      defaultValue={value}
      placeholder={placeholder}
      title={title ?? "Enter applies"}
      spellCheck={false}
      onClick={(e) => e.stopPropagation()}
      onKeyDown={(e) => {
        if (e.key === "Escape") e.currentTarget.blur();
        if (e.key === "Enter" && (formula ? e.currentTarget.value.trim() !== "" : true)) onChange(e.currentTarget.value);
      }}
      onBlur={(e) => (e.currentTarget.value = value)}
    />
  );
}

/** A field's label beside its control, the one grid every labelled field is laid out in. */
function Labelled({ label, unit, children }: { label: string; unit?: string; children: ReactNode }) {
  return (
    <label className="row field-row">
      <span className="field-label">
        {label}
        {unit && <span className="field-unit">{unit}</span>}
      </span>
      {children}
    </label>
  );
}

/**
 * A number that may be given as a formula of the parameters: its text
 * edited in place (Enter applies), marked as taking formulas, and what a
 * formula comes to — or why it does not evaluate — beside it.
 */
function FormulaNumber({ c, send }: { c: Extract<Control, { type: "number" }>; send: (value: Value) => void }) {
  const text = c.text ?? "";
  const formula = text.trim() !== "" && Number.isNaN(Number(text));
  return (
    <>
      <Labelled label={c.label} unit={UNITS[c.unit]}>
        <span className="formula-box" title="A number, or a formula of the parameters: width / 2, screw.d · Enter applies">
          <span className="formula-mark" aria-hidden="true">
            ƒ
          </span>
          <TextInput value={text} formula onChange={(value) => send({ type: "text", value })} />
        </span>
      </Labelled>
      {formula && c.error == null && <p className="hint field-note">= {Number(c.value.toFixed(6))}</p>}
      {c.error && <p className="op-error-text field-note">{c.error}</p>}
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

/** What a reference field's button says: what it holds, or that it waits for a pick. */
function referenced(c: Extract<Control, { type: "reference" }>): string {
  if (c.value.length === 0) return c.armed ? "click in the viewport…" : "pick…";
  if (c.multiple) return c.armed ? "click to add or take out…" : `${c.value.length} picked — pick more…`;
  return elementLabel(c.value[0].element);
}

/**
 * A file field: the path chosen, and a button browsing the workspace's
 * files for another in an explorer — where one from disk can be chosen
 * too, if the field takes such, stored next to the program first.
 */
function FileField({
  c,
  send,
  onChooseFile,
}: {
  c: Extract<Control, { type: "file" }>;
  send: (value: Value) => void;
  onChooseFile: (file: File) => Promise<string | null>;
}) {
  const [browsing, setBrowsing] = useState(false);
  return (
    <div className="row field-row file-field" data-field={c.label}>
      <span className="field-label">{c.label}</span>
      <button
        className={`file-field-browse${c.value ? " holds" : ""}`}
        title={c.value ? `${c.value} — browse for another` : "Browse the files"}
        onClick={() => setBrowsing(true)}
      >
        <span className="file-field-path">{c.value || "choose a file…"}</span>
        <span className="file-field-dots" aria-hidden>
          …
        </span>
      </button>
      {browsing && (
        <FileChooser
          label={c.label}
          value={c.value}
          files={c.files}
          upload={c.upload}
          onChoose={(value) => send({ type: "choice", value })}
          onUpload={onChooseFile}
          onClose={() => setBrowsing(false)}
        />
      )}
    </div>
  );
}

/**
 * One field of a dialog, rendered from its primitive alone — so any
 * operation the kernel offers gets its dialog without this knowing about
 * it. The same renderer serves both layouts: a field looks the same beside
 * the viewport and in a popup.
 */
function FieldView({
  k,
  c,
  onDialog,
  onChooseFile,
  onDownload,
}: {
  k: string;
  c: Control;
  onDialog: Props["onDialog"];
  onChooseFile: Props["onChooseFile"];
  onDownload: Props["onDownload"];
}) {
  const send = (value: Value) => onDialog(k, value);
  switch (c.type) {
    case "heading":
      return <h3 className="dialog-heading">{c.text}</h3>;
    case "text":
      return <p className={TONES[c.tone]}>{c.text}</p>;
    case "input":
      return (
        <Labelled label={c.label}>
          <TextInput
            value={c.value}
            placeholder={c.placeholder ?? undefined}
            onChange={(value) => send({ type: "text", value })}
          />
        </Labelled>
      );
    case "reference": {
      const failed = c.value.find((p) => p.tone === "error");
      return (
        <div className="reference" data-field={c.label}>
          <div className="row field-row">
            <span className="field-label">{c.label}</span>
            <span className="reference-box">
              <button
                className={["reference-pick", c.armed ? "active" : "", c.value.length > 0 ? "holds" : ""].join(" ")}
                title={c.multiple ? "Pick in the viewport — again to take out" : "Pick in the viewport"}
                onClick={() => send({ type: "press" })}
              >
                <Icon name="pick" />
                <span className="reference-text">{referenced(c)}</span>
              </button>
              {c.value.length > 0 && (
                <button
                  className="icon-only reference-clear"
                  title={c.multiple ? "Take everything out" : "Take it out"}
                  aria-label={`Clear ${c.label}`}
                  onClick={() => send({ type: "clear" })}
                >
                  ✕
                </button>
              )}
            </span>
          </div>
          {failed && !c.multiple && <p className="op-error-text field-note">{failed.detail}</p>}
          {c.multiple && c.value.length > 0 && (
            <ol className="dialog-list">
              {c.value.map((p, i) => (
                <li key={i} className={p.tone === "error" ? "failed" : ""}>
                  <span className="item-label" title={elementLabel(p.element)}>
                    {elementLabel(p.element)}
                  </span>
                  {p.detail && <span className="item-detail">{p.detail}</span>}
                  <button className="item-remove" title="Take out" onClick={() => send({ type: "remove_at", value: i })}>
                    ✕
                  </button>
                </li>
              ))}
            </ol>
          )}
        </div>
      );
    }
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
    case "download":
      return (
        <div className="button-grid download">
          {c.formats.map((f) => (
            <button key={f.value} title={`Download as ${f.label}`} onClick={() => onDownload(c.exporter, f.value)}>
              <Icon name="save" /> Download {f.label}
            </button>
          ))}
        </div>
      );
    case "checkbox":
      return (
        <label className="row field-row checkbox-row">
          <span className="field-label">{c.label}</span>
          <input type="checkbox" checked={c.value} onChange={(e) => send({ type: "bool", value: e.target.checked })} />
        </label>
      );
    case "number": {
      if (c.text != null) return <FormulaNumber c={c} send={send} />;
      return (
        <Labelled label={c.label} unit={UNITS[c.unit]}>
          <NumberInput value={c.value} step={c.step} onChange={(value) => send({ type: "number", value })} />
        </Labelled>
      );
    }
    case "select":
      return c.searchable ? (
        <SearchSelect label={c.label} value={c.value} options={c.options} onChange={(value) => send({ type: "choice", value })} />
      ) : (
        <Dropdown label={c.label} value={c.value} options={c.options} onChange={(value) => send({ type: "choice", value })} />
      );
    case "file":
      return <FileField c={c} send={send} onChooseFile={onChooseFile} />;
    case "color":
      return (
        <Labelled label={c.label}>
          <ColorInput value={c.value} onChange={(color) => send({ type: "text", value: color })} />
        </Labelled>
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
                <TextInput value={item.text} formula onChange={(value) => onDialog(item.key, { type: "text", value })} />
              )}
              {item.text == null && item.value != null && (
                <NumberInput value={item.value} step={0.1} onChange={(value) => onDialog(item.key, { type: "number", value })} />
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
    case "table":
      return <TableField table={c} send={(edit) => send({ type: "table", value: edit })} />;
  }
}

/**
 * The dialog of the step being edited: every field the operation shows,
 * rendered from its primitives alone, and the controls every step has —
 * preview, OK, Cancel — in the layout the operation asks for: beside the
 * viewport, one field under the other, with the part in reach for picks;
 * or, for a form of tables, a wide popup over the middle of it. The fields
 * are the same components in both; only the frame differs.
 */
export function DialogView({ step, onDialog, onChooseFile, setPreview, error, onCommit, onCancel, onDownload }: Props) {
  const popup = step.presentation.layout === "popup";
  const blocked = step.error != null || step.missing.length > 0;
  // Why the step does not build is said beside the field it fails with,
  // and summed up under the fields.
  const fields = step.presentation.dialog.map(({ key, failure, ...c }) => (
    <div key={key} className={`field field-${c.type}${failure ? " field-failed" : ""}`} data-field-key={key}>
      <FieldView k={key} c={c as Control} onDialog={onDialog} onChooseFile={onChooseFile} onDownload={onDownload} />
      {failure && (
        <p className="op-error-text field-note field-failure" role="alert">
          {failure}
        </p>
      )}
    </div>
  ));
  const status = (
    <>
      {error && <p className="op-error-text">{error}</p>}
      {step.missing.length > 0 ? (
        <p className="hint pick-hint-inline">Pick the {step.missing.join(" and ")} in the viewport to continue.</p>
      ) : (
        step.error && <p className="op-error-text">{step.error}</p>
      )}
    </>
  );
  const footer = (
    <div className="dialog-footer">
      <label className="row preview-row">
        <input type="checkbox" checked={step.preview} onChange={(e) => setPreview(e.target.checked)} />
        Preview
      </label>
      <div className="button-row">
        <button className="small" onClick={onCancel} title="Leave the program as it was (Esc, before anything is changed)">
          Cancel
        </button>
        <button
          className="primary"
          disabled={blocked}
          title={
            step.missing.length > 0
              ? `Pick the ${step.missing.join(" and ")} first`
              : `Put the step into the program (${formatShortcut("Mod+Enter")}, or Enter with nothing in progress)`
          }
          onClick={onCommit}
        >
          OK
        </button>
      </div>
    </div>
  );
  const heading = (
    <h2 title={step.doc}>
      {step.label}
      {step.id && <span className="op-id"> · {step.id}</span>}
    </h2>
  );
  if (popup) {
    return (
      <div className="popup-backdrop centered">
        <div className="popup wide" role="dialog" aria-label={step.label}>
          <div className="dialog-head">
            {heading}
            <button className="icon-only" onClick={onCancel} title="Cancel" aria-label="Cancel">
              ✕
            </button>
          </div>
          <div className="dialog-body">{fields}</div>
          {status}
          {footer}
        </div>
      </div>
    );
  }
  return (
    <div className="popup-backdrop">
      <div className="popup" role="dialog" aria-label={step.label}>
        {heading}
        <div className="dialog-body">{fields}</div>
        {status}
        {footer}
      </div>
    </div>
  );
}
