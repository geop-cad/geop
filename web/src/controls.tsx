import { useMemo, useState } from "react";

interface SliderNumberProps {
  label: string;
  value: number;
  onChange: (v: number) => void;
  min: number;
  max: number;
  step: number;
}

/** A range slider paired with a number input for the same value — either can be dragged or typed into. */
export function SliderNumber({ label, value, onChange, min, max, step }: SliderNumberProps) {
  return (
    <label className="slider-number">
      <span className="slider-number-label">{label}</span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
      />
      <input
        type="number"
        className="slider-number-input"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => {
          const v = Number(e.target.value);
          if (!Number.isNaN(v)) onChange(v);
        }}
      />
    </label>
  );
}

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
