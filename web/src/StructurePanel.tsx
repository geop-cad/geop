import type { StructureItem, StructureKind } from "./geop";

interface Props {
  items: StructureItem[];
  enabled: boolean;
  /** Show or hide `name`. */
  onVisibility: (name: string, visible: boolean) => void;
}

/** The kinds listed, in order, with what their group is called. */
const GROUPS: [StructureKind, string][] = [
  ["solid", "Solids"],
  ["sketch", "Sketches"],
  ["datum", "Datums"],
  ["part", "Placed parts"],
  ["mate", "Mates"],
];

/** An eye, open or shut: whether something is shown. */
function Eye({ open }: { open: boolean }) {
  return (
    <svg viewBox="0 0 20 20" width="16" height="16" fill="none" stroke="currentColor" strokeWidth="1.5" aria-hidden="true">
      <path d="M2 10 C5 4.5, 15 4.5, 18 10 C15 15.5, 5 15.5, 2 10 Z" />
      {open ? <circle cx="10" cy="10" r="2.5" fill="currentColor" /> : <path d="M3 17 L17 3" />}
    </svg>
  );
}

/**
 * What the part has beyond its faces — its solids, sketches, datums, the
 * parts placed in it and their mates — grouped by kind, each with a
 * switch to show or hide it. Whether it is shown is the kernel's: what the
 * editor hides by itself — a sketch once extruded — until the user says
 * otherwise.
 */
export function StructurePanel({ items, enabled, onVisibility }: Props) {
  const groups = GROUPS.map(([kind, title]) => [title, items.filter((i) => i.kind === kind)] as const).filter(
    ([, list]) => list.length > 0,
  );
  if (groups.length === 0) return <p className="hint">Nothing yet.</p>;
  return (
    <div className="structure">
      {groups.map(([title, list]) => (
        <details key={title} className="structure-group" open>
          <summary>
            {title} <span className="structure-count">{list.length}</span>
          </summary>
          <ul>
            {list.map((item) => (
              <li key={item.name} className={item.visible === false ? "hidden-item" : ""} title={item.name}>
                <span className="structure-name">{item.name}</span>
                {item.visible != null && (
                  <button
                    className="structure-eye"
                    disabled={!enabled}
                    title={item.visible ? "Hide" : "Show"}
                    aria-label={`${item.visible ? "Hide" : "Show"} ${item.name}`}
                    onClick={() => onVisibility(item.name, !item.visible)}
                  >
                    <Eye open={item.visible} />
                  </button>
                )}
              </li>
            ))}
          </ul>
        </details>
      ))}
    </div>
  );
}
