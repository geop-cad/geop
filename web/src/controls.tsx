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
