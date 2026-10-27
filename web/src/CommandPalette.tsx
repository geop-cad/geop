import { useEffect, useMemo, useRef, useState } from "react";
import { formatShortcut, search, type PaletteEntry } from "./shortcuts";

interface Props {
  entries: PaletteEntry[];
  onClose: () => void;
}

/**
 * The command palette (Ctrl+K, ⌘K): every operation, step, example and
 * action of the editor, found by typing a few letters of it — arrows to
 * choose, Enter to run, Escape to close — each with its key, if it has one.
 */
export function CommandPalette({ entries, onClose }: Props) {
  const [query, setQuery] = useState("");
  const [chosen, setChosen] = useState(0);
  const found = useMemo(() => search(entries, query), [entries, query]);
  const list = useRef<HTMLUListElement>(null);
  const at = Math.min(chosen, Math.max(found.length - 1, 0));

  // The one chosen stays in sight as the arrows move it.
  useEffect(() => {
    list.current?.children[at]?.scrollIntoView({ block: "nearest" });
  }, [at]);

  const run = (entry: PaletteEntry | undefined) => {
    if (!entry || entry.disabled) return;
    onClose();
    entry.run();
  };

  return (
    <div className="palette-backdrop" onMouseDown={onClose}>
      <div className="palette" role="dialog" aria-label="Command palette" onMouseDown={(e) => e.stopPropagation()}>
        <input
          className="palette-input"
          autoFocus
          placeholder="Search operations, steps, examples and actions…"
          value={query}
          aria-label="Search"
          onChange={(e) => {
            setQuery(e.target.value);
            setChosen(0);
          }}
          onKeyDown={(e) => {
            e.stopPropagation();
            if (e.key === "ArrowDown") {
              e.preventDefault();
              setChosen((i) => Math.min(i + 1, found.length - 1));
            } else if (e.key === "ArrowUp") {
              e.preventDefault();
              setChosen((i) => Math.max(i - 1, 0));
            } else if (e.key === "Enter") {
              e.preventDefault();
              run(found[at]);
            } else if (e.key === "Escape") {
              e.preventDefault();
              onClose();
            }
          }}
        />
        {found.length === 0 ? (
          <p className="palette-empty">Nothing matches “{query}”.</p>
        ) : (
          <ul ref={list} className="palette-list" role="listbox">
            {found.map((entry, i) => (
              <li
                key={entry.id}
                role="option"
                aria-selected={i === at}
                aria-disabled={entry.disabled}
                className={["palette-item", i === at ? "chosen" : "", entry.disabled ? "disabled" : ""].join(" ")}
                onMouseMove={() => setChosen(i)}
                onClick={() => run(entry)}
              >
                <span className="palette-group">{entry.group}</span>
                <span className="palette-label">{entry.label}</span>
                {entry.detail && <span className="palette-detail">{entry.detail}</span>}
                {entry.shortcut && <kbd className="palette-key">{formatShortcut(entry.shortcut)}</kbd>}
              </li>
            ))}
          </ul>
        )}
      </div>
    </div>
  );
}

/** The keys of the editor, grouped as the palette groups them: what `?` shows. */
export function ShortcutsOverlay({ entries, onClose }: Props) {
  const groups = new Map<string, PaletteEntry[]>();
  for (const entry of entries) {
    if (!entry.shortcut) continue;
    groups.set(entry.group, [...(groups.get(entry.group) ?? []), entry]);
  }
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape" || e.key === "?") {
        e.preventDefault();
        e.stopImmediatePropagation();
        onClose();
      }
    };
    window.addEventListener("keydown", onKey, { capture: true });
    return () => window.removeEventListener("keydown", onKey, { capture: true });
  }, [onClose]);
  return (
    <div className="palette-backdrop" onMouseDown={onClose}>
      <div className="shortcuts" role="dialog" aria-label="Keyboard shortcuts" onMouseDown={(e) => e.stopPropagation()}>
        <div className="dialog-head">
          <h2>Keyboard shortcuts</h2>
          <button className="icon-only" onClick={onClose} title="Close (Esc)" aria-label="Close">
            ✕
          </button>
        </div>
        <div className="shortcuts-groups">
          {[...groups].map(([group, list]) => (
            <section key={group}>
              <h3>{group}</h3>
              <dl>
                {list.map((entry) => (
                  <div key={entry.id} className="shortcut-row">
                    <dt>{entry.label}</dt>
                    <dd>
                      <kbd>{formatShortcut(entry.shortcut ?? "")}</kbd>
                    </dd>
                  </div>
                ))}
              </dl>
            </section>
          ))}
        </div>
        <p className="hint">Operations start by their key while no step is edited; a step's own tools take letters while it is.</p>
      </div>
    </div>
  );
}
