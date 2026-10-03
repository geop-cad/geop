import { useEffect, useRef, useState, type ReactNode } from "react";
import { Icon } from "./icons";

/** An entry of a [[Menu]]: something to do, a heading over the entries after it, or a line between groups. */
export type MenuEntry =
  | { kind: "item"; label: string; icon?: string; hint?: string; disabled?: boolean; onSelect: () => void }
  | { kind: "heading"; label: string }
  | { kind: "separator" };

interface Props {
  /** What the button shows. */
  label: ReactNode;
  title?: string;
  entries: MenuEntry[];
  disabled?: boolean;
  /** Which side of the button the menu opens to. */
  align?: "left" | "right";
  className?: string;
}

/**
 * A button that drops down a menu: picking an entry does it and closes the
 * menu, as do a click elsewhere and Escape.
 */
export function Menu({ label, title, entries, disabled, align = "left", className }: Props) {
  const [open, setOpen] = useState(false);
  const root = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const away = (e: PointerEvent) => {
      if (!root.current?.contains(e.target as Node)) setOpen(false);
    };
    const escape = (e: KeyboardEvent) => {
      if (e.key === "Escape") setOpen(false);
    };
    window.addEventListener("pointerdown", away);
    window.addEventListener("keydown", escape);
    return () => {
      window.removeEventListener("pointerdown", away);
      window.removeEventListener("keydown", escape);
    };
  }, [open]);
  return (
    <div className={["dropdown", className ?? ""].join(" ")} ref={root}>
      <button
        className={["dropdown-trigger", open ? "open" : ""].join(" ")}
        title={title}
        disabled={disabled}
        aria-haspopup="menu"
        aria-expanded={open}
        onClick={() => setOpen(!open)}
      >
        {label}
        <Icon name="chevron" />
      </button>
      {open && (
        <div className={`dropdown-menu ${align}`} role="menu">
          {entries.map((entry, i) => {
            switch (entry.kind) {
              case "heading":
                return (
                  <div key={i} className="dropdown-heading">
                    {entry.label}
                  </div>
                );
              case "separator":
                return <div key={i} className="dropdown-separator" />;
              case "item":
                return (
                  <button
                    key={i}
                    role="menuitem"
                    className="dropdown-item"
                    disabled={entry.disabled}
                    onClick={() => {
                      setOpen(false);
                      entry.onSelect();
                    }}
                  >
                    <span className="dropdown-item-icon">{entry.icon && <Icon name={entry.icon} />}</span>
                    <span className="dropdown-item-label">{entry.label}</span>
                    {entry.hint && <span className="dropdown-item-hint">{entry.hint}</span>}
                  </button>
                );
            }
          })}
        </div>
      )}
    </div>
  );
}
