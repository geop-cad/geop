import { useRef, type ClipboardEvent, type KeyboardEvent } from "react";
import type { Cell, Column, Table, TableEdit, Tone } from "./geop";
import { Icon } from "./icons";
import { parseBlock } from "./tableClipboard";

interface Props {
  table: Table;
  /** Something was done to the table; settled once the step is updated. */
  send: (edit: TableEdit) => void;
}

/** The class a cell's detail reads in, by tone. */
const DETAIL_TONES: Record<Tone, string> = {
  normal: "",
  hint: "hint",
  error: "op-error-text",
  success: "status-ok",
};

/**
 * Text typed into a cell or a header: applied when Enter is pressed or the
 * field is left, if it changed; Escape puts the text back. Keyed by the
 * text, so a change from elsewhere shows.
 */
function CellInput({
  value,
  kind,
  placeholder,
  title,
  onChange,
  onKeyDown,
  onPaste,
  dataRow,
  dataColumn,
}: {
  value: string;
  kind: Column["kind"]["type"];
  placeholder?: string;
  title?: string;
  onChange: (text: string) => void;
  onKeyDown?: (e: KeyboardEvent<HTMLInputElement>) => void;
  onPaste?: (e: ClipboardEvent<HTMLInputElement>) => void;
  dataRow?: number;
  dataColumn?: number;
}) {
  // What was given already: Enter gives, and the blur that follows the
  // focus moving on must not give the same text again. Keyed by the value,
  // the input is made anew once the kernel shows the change.
  const given = useRef<string | null>(null);
  const give = (el: HTMLInputElement) => {
    if (el.value === value || el.value === given.current) return;
    given.current = el.value;
    onChange(el.value);
  };
  return (
    <input
      key={value}
      type="text"
      className={`cell-input cell-${kind}`}
      defaultValue={value}
      placeholder={placeholder}
      title={title}
      inputMode={kind === "number" ? "decimal" : undefined}
      spellCheck={false}
      data-row={dataRow}
      data-column={dataColumn}
      onClick={(e) => e.stopPropagation()}
      onFocus={(e) => e.currentTarget.select()}
      onBlur={(e) => give(e.currentTarget)}
      onPaste={onPaste}
      onKeyDown={(e) => {
        if (e.key === "Escape") {
          e.currentTarget.value = value;
          e.currentTarget.blur();
          return;
        }
        if (e.key === "Enter") give(e.currentTarget);
        onKeyDown?.(e);
      }}
    />
  );
}

/**
 * A table edited in place, as the kernel describes it ([[Table]]): a
 * header per column — the user's columns renamed there, and taken out —
 * a row per entry with an input per cell, and the row chosen marked. Rows
 * are added, moved and taken out by their buttons. Enter in a cell moves
 * down a row, Tab across, the arrow keys where they point when the caret
 * is at the text's edge, and a block pasted from a spreadsheet is sent as
 * one paste, the rows it runs over added. Everything done is sent to the
 * kernel as one [[TableEdit]]: nothing is kept here.
 */
export function TableField({ table, send }: Props) {
  const root = useRef<HTMLTableElement>(null);
  const columns = table.columns.length;
  const rows = table.rows.length;

  /** Move the focus to the cell at `row`, `column`, if there is one. */
  const focus = (row: number, column: number) => {
    const next = root.current?.querySelector<HTMLInputElement>(`input[data-row="${row}"][data-column="${column}"]`);
    next?.focus();
    return next != null;
  };

  const navigate = (row: number, column: number) => (e: KeyboardEvent<HTMLInputElement>) => {
    const el = e.currentTarget;
    const atStart = el.selectionStart === 0 && el.selectionEnd === 0;
    const atEnd = el.selectionStart === el.value.length && el.selectionEnd === el.value.length;
    let to: [number, number] | null = null;
    if (e.key === "Enter") to = e.shiftKey ? [row - 1, column] : [row + 1, column];
    else if (e.key === "ArrowDown") to = [row + 1, column];
    else if (e.key === "ArrowUp") to = [row - 1, column];
    else if (e.key === "ArrowRight" && atEnd) to = [row, column + 1];
    else if (e.key === "ArrowLeft" && atStart) to = [row, column - 1];
    if (!to) return;
    e.preventDefault();
    if (to[0] < 0 || to[1] < 0 || to[1] >= columns) return;
    if (to[0] >= rows) {
      // Enter on the last row adds one, which the update then shows.
      if (e.key === "Enter" && table.rows_editable) {
        el.blur();
        send({ edit: "add_row" });
      }
      return;
    }
    if (!focus(to[0], to[1])) el.blur();
  };

  const paste = (row: number, column: number) => (e: ClipboardEvent<HTMLInputElement>) => {
    const cells = parseBlock(e.clipboardData.getData("text/plain"));
    if (!cells) return;
    e.preventDefault();
    e.currentTarget.blur();
    send({ edit: "paste", row, column, cells });
  };

  const header = (c: Column, k: number) => {
    const unit = c.kind.type === "number" && c.kind.unit === "angle" ? " (°)" : "";
    if (!c.editable) return <span className="table-head-label">{c.label + unit}</span>;
    return (
      <span className="table-head-editable">
        <CellInput
          value={c.label}
          kind="text"
          title="The column's name — renamed by typing"
          onChange={(text) => send({ edit: "column", column: k, text })}
        />
        <button
          className="icon-only"
          title={`Remove the column ${c.label}`}
          aria-label={`Remove the column ${c.label}`}
          onClick={() => send({ edit: "remove_column", column: k })}
        >
          ✕
        </button>
      </span>
    );
  };

  const cell = (c: Cell, row: number, column: number) => {
    const kind = table.columns[column]?.kind.type ?? "text";
    return (
      <td key={column} className={c.tone === "error" ? "failed" : ""}>
        <CellInput
          value={c.text}
          kind={kind}
          title={c.detail ?? undefined}
          onChange={(text) => send({ edit: "cell", row, column, text })}
          onKeyDown={navigate(row, column)}
          onPaste={paste(row, column)}
          dataRow={row}
          dataColumn={column}
        />
        {c.detail && <span className={["cell-detail", DETAIL_TONES[c.tone]].join(" ")}>{c.detail}</span>}
      </td>
    );
  };

  const actions = table.rows_editable || table.rows.some((r) => r.removable);
  return (
    <div className="table-field">
      <table ref={root}>
        <thead>
          <tr>
            {table.selectable && (
              <th className="table-select" title="The row chosen">
                <Icon name="check" />
              </th>
            )}
            {table.columns.map((c, k) => (
              <th key={k} className={`table-head-${c.kind.type}`}>
                {header(c, k)}
              </th>
            ))}
            {table.columns_editable && (
              <th className="table-add-column">
                <button className="small" title="Add a column" onClick={() => send({ edit: "add_column" })}>
                  + column
                </button>
              </th>
            )}
            {actions && <th className="table-actions" />}
          </tr>
        </thead>
        <tbody>
          {table.rows.map((r, row) => (
            <tr key={row} className={r.selected ? "selected" : ""}>
              {table.selectable && (
                <td className="table-select">
                  <input
                    type="radio"
                    name="table-selected"
                    title="Choose this row"
                    aria-label={`Choose row ${row + 1}`}
                    checked={r.selected}
                    onChange={() => send({ edit: "select_row", row })}
                  />
                </td>
              )}
              {r.cells.map((c, column) => cell(c, row, column))}
              {table.columns_editable && <td />}
              {actions && (
                <td className="table-actions">
                  {table.rows_editable && (
                    <>
                      <button
                        className="icon-only"
                        title="Move up"
                        aria-label={`Move row ${row + 1} up`}
                        disabled={row === 0}
                        onClick={() => send({ edit: "move_row", row, to: row - 1 })}
                      >
                        ▲
                      </button>
                      <button
                        className="icon-only"
                        title="Move down"
                        aria-label={`Move row ${row + 1} down`}
                        disabled={row === rows - 1}
                        onClick={() => send({ edit: "move_row", row, to: row + 1 })}
                      >
                        ▼
                      </button>
                    </>
                  )}
                  <button
                    className="icon-only"
                    title={r.removable ? "Remove the row" : "The last row stays"}
                    aria-label={`Remove row ${row + 1}`}
                    disabled={!r.removable}
                    onClick={() => send({ edit: "remove_row", row })}
                  >
                    ✕
                  </button>
                </td>
              )}
            </tr>
          ))}
        </tbody>
      </table>
      {rows === 0 && <p className="hint table-empty">{table.empty}</p>}
      {table.rows_editable && (
        <div className="table-foot">
          <button className="small" title="Add a row after the last (Enter in the last row adds one too)" onClick={() => send({ edit: "add_row" })}>
            + row
          </button>
          <span className="hint">Paste from a spreadsheet into a cell to fill the table from there.</span>
        </div>
      )}
    </div>
  );
}
