import type { PickedElement, StructureItem } from "./geop";

interface Props {
  items: StructureItem[];
  /** What to say beside an item, by its name: how free a placed part is. */
  details: Record<string, string>;
  enabled: boolean;
  /** Show or hide `name`. */
  onVisibility: (name: string, visible: boolean) => void;
  /** Light these entities in the view, or nothing. */
  onLight: (entities: PickedElement[]) => void;
  /** The file of every part placed, however deep, by its path: `neck/head`. */
  placed: Map<string, string>;
  /** The placed part new steps go into, built in context, if one is chosen. */
  context: string | null;
  /** New steps go into the part placed at `path`, built in context — or, none, into the program. */
  onContext: (path: string | null) => void;
  /** Open the file `file` a part is placed from, to edit on its own — where the front end opens files. */
  onOpenFile?: (file: string) => void;
}

/** The items of each group, by what the kernel calls their kind, the groups in the order their first items come in. */
function grouped(items: StructureItem[]): [string, StructureItem[]][] {
  const groups = new Map<string, StructureItem[]>();
  for (const item of items) groups.set(item.kind_label, [...(groups.get(item.kind_label) ?? []), item]);
  return [...groups.entries()];
}

/** How deep the item `item` is placed, if it is a placed part: none for one of the part's own, one for a part placed in one. */
function depth(item: StructureItem, placed: Map<string, string>): number {
  return placed.has(item.name) ? item.name.split("/").length - 1 : 0;
}

/** The last name of the path `path`: `head` of `neck/head`. */
function baseName(path: string): string {
  return path.slice(path.lastIndexOf("/") + 1);
}

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
 * What the part has beyond the faces of its solids — its solids, faces
 * standing on their own, sketches, datums, the parts placed in it, and
 * whatever its steps list, such as mates — grouped by kind, as the kernel
 * names and orders them, each drawn one
 * with a switch to show or hide it, each pointed at lighting what it
 * refers to. Whether it is shown is the kernel's: what the editor hides by
 * itself — a sketch once extruded — until the user says otherwise.
 */
export function StructurePanel({ items, details, enabled, onVisibility, onLight, placed, context, onContext, onOpenFile }: Props) {
  const groups = grouped(items);
  if (groups.length === 0) return <p className="hint">Nothing yet: what the steps build is listed here, to show or hide.</p>;
  return (
    <div className="structure">
      {groups.map(([title, list]) => (
        <details key={title} className="structure-group" open>
          <summary>
            {title} <span className="structure-count">{list.length}</span>
          </summary>
          <ul>
            {list.map((item) => (
              <li
                key={item.name}
                className={item.visible === false ? "hidden-item" : ""}
                title={item.name}
                onMouseEnter={() => item.elements.length > 0 && onLight(item.elements)}
                onMouseLeave={() => item.elements.length > 0 && onLight([])}
              >
                {/* A part placed in a placed part, under it: by its own name, indented as deep as it is. */}
                <span className="structure-name" style={{ paddingLeft: `${depth(item, placed) * 14}px` }}>
                  {placed.has(item.name) ? baseName(item.name) : item.name}
                </span>
                {item.detail != null && <span className="structure-file">{item.detail}</span>}
                {details[item.name] != null && <span className="structure-detail">{details[item.name]}</span>}
                {placed.has(item.name) && (
                  <button
                    className={`structure-action${context === item.name ? " on" : ""}`}
                    disabled={!enabled}
                    title={
                      context === item.name
                        ? "New steps go into this part: click to put them into the program again"
                        : "Edit in place: new steps go into this part, built against the parts placed before it here"
                    }
                    aria-label={`Edit ${item.name} in place`}
                    aria-pressed={context === item.name}
                    onClick={() => onContext(context === item.name ? null : item.name)}
                  >
                    ✎
                  </button>
                )}
                {placed.has(item.name) && onOpenFile && (
                  <button
                    className="structure-action"
                    disabled={!enabled}
                    title={`Open ${placed.get(item.name)}, to edit the part on its own`}
                    aria-label={`Open ${placed.get(item.name)}`}
                    onClick={() => onOpenFile(placed.get(item.name) ?? "")}
                  >
                    ↗
                  </button>
                )}
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
