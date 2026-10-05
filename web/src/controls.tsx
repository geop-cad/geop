import { useEffect, useMemo, useRef, useState } from "react";

interface DropdownProps<T extends string> {
  label: string;
  value: T;
  options: readonly { value: T; label: string }[];
  onChange: (v: T) => void;
}

/** A labeled `<select>` for a small fixed set of string options (e.g. an axis). */
export function Dropdown<T extends string>({ label, value, options, onChange }: DropdownProps<T>) {
  return (
    <label className="row">
      {label}
      <select value={value} onChange={(e) => onChange(e.target.value as T)}>
        {options.map((o) => (
          <option key={o.value} value={o.value}>
            {o.label}
          </option>
        ))}
      </select>
    </label>
  );
}

/**
 * A labeled choice among many options, found by typing: the options whose
 * label contains what is typed are listed, and picking one chooses it.
 */
export function SearchSelect({ label, value, options, onChange }: DropdownProps<string>) {
  const [query, setQuery] = useState<string | null>(null);
  const current = options.find((o) => o.value === value)?.label ?? value;
  const shown = useMemo(() => {
    if (query == null) return [];
    const q = query.toLowerCase();
    return options.filter((o) => o.label.toLowerCase().includes(q)).slice(0, 50);
  }, [query, options]);
  const choose = (v: string) => {
    setQuery(null);
    onChange(v);
  };
  return (
    <label className="row search-select">
      {label}
      <span className="search-select-box">
        <input
          type="search"
          value={query ?? current}
          placeholder="Search…"
          onFocus={(e) => {
            setQuery("");
            e.currentTarget.select();
          }}
          onChange={(e) => setQuery(e.target.value)}
          onBlur={() => setTimeout(() => setQuery(null), 150)}
          onKeyDown={(e) => {
            if (e.key === "Enter" && shown.length > 0) {
              e.preventDefault();
              choose(shown[0].value);
              e.currentTarget.blur();
            }
            if (e.key === "Escape") e.currentTarget.blur();
          }}
        />
        {query != null && shown.length > 0 && (
          <ul className="search-select-options">
            {shown.map((o) => (
              <li
                key={o.value}
                className={o.value === value ? "selected" : ""}
                onMouseDown={(e) => {
                  e.preventDefault();
                  choose(o.value);
                }}
              >
                {o.label || "—"}
              </li>
            ))}
          </ul>
        )}
      </span>
    </label>
  );
}

/**
 * A colour picked, `#rrggbb`. Shown at once as it is dragged, and given on
 * as it changes — but one at a time: while the last one is still being
 * worked in, only the newest waits, so a dragged picker never queues up
 * every colour it passed.
 */
export function ColorInput({
  value,
  onChange,
  title,
}: {
  value: string;
  onChange: (color: string) => Promise<unknown> | void;
  title?: string;
}) {
  const [shown, setShown] = useState(value);
  useEffect(() => setShown(value), [value]);
  const busy = useRef(false);
  const waiting = useRef<string | null>(null);
  const give = (color: string) => {
    if (busy.current) {
      waiting.current = color;
      return;
    }
    busy.current = true;
    void Promise.resolve(onChange(color)).finally(() => {
      busy.current = false;
      const next = waiting.current;
      waiting.current = null;
      if (next != null) give(next);
    });
  };
  return (
    <input
      type="color"
      value={shown}
      title={title}
      onChange={(e) => {
        setShown(e.target.value);
        give(e.target.value);
      }}
    />
  );
}
