import { useEffect, useLayoutEffect, useRef, useState, type ReactNode } from "react";
import { Icon } from "./icons";

/** An entry of a [[Menu]]: something to do, a heading over the entries after it, or a line between groups. */
export type MenuEntry =
  | {
      kind: "item";
      label: string;
      icon?: string;
      hint?: string;
      /** What it does, at length: the entry's tooltip. */
      title?: string;
      disabled?: boolean;
      /** Whether it is what is being done now: drawn as an active button is. */
      active?: boolean;
      onSelect: () => void;
    }
  | { kind: "heading"; label: string }
  | { kind: "separator" };

interface Props {
  /** What the button shows. */
  label: ReactNode;
  title?: string;
  entries: MenuEntry[];
  disabled?: boolean;
  /** Which side of the button the menu opens to, if it fits the window there; else the other. */
  align?: "left" | "right";
  className?: string;
  /** Classes of the button itself. */
  triggerClassName?: string;
}

/**
 * A button that drops down a menu: picking an entry does it and closes the
 * menu, as do a click elsewhere and Escape.
 */
export function Menu({ label, title, entries, disabled, align = "left", className, triggerClassName }: Props) {
  const [open, setOpen] = useState(false);
  const [side, setSide] = useState(align);
  const root = useRef<HTMLDivElement>(null);
  const menu = useRef<HTMLDivElement>(null);
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
  // Opened towards `align`, unless that runs off the window and the other side does not.
  useLayoutEffect(() => {
    if (!open || !menu.current || !root.current) return;
    const width = menu.current.getBoundingClientRect().width;
    const button = root.current.getBoundingClientRect();
    const fitsLeft = button.left + width <= window.innerWidth;
    const fitsRight = button.right - width >= 0;
    setSide(align === "left" ? (fitsLeft || !fitsRight ? "left" : "right") : fitsRight || !fitsLeft ? "right" : "left");
  }, [open, align]);
  return (
    <div className={["dropdown", className ?? ""].join(" ")} ref={root}>
      <button
        className={["dropdown-trigger", triggerClassName ?? "", open ? "open" : ""].join(" ")}
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
        <div ref={menu} className={`dropdown-menu ${side}`} role="menu">
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
                    className={["dropdown-item", entry.active ? "active" : ""].join(" ")}
                    title={entry.title}
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
