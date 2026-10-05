import { useLayoutEffect, useRef, useState, type ReactNode } from "react";
import { Icon } from "./icons";
import { Menu, type MenuEntry } from "./Menu";

/** A button of the operations: an operation to start a step of, or a tool of the editor's. */
export interface Tool {
  /** Its icon's name, and what tells it from the others. */
  kind: string;
  label: string;
  /** What it does, at length: its tooltip. */
  doc: string;
  /** The section of the toolbar it is in (see `geop_ops::OperationGroup`). */
  group: string;
  /** Whether it is one of the few used most, shown big. */
  primary: boolean;
  active: boolean;
  disabled: boolean;
  onSelect: () => void;
}

/** The tools of each group, the groups in the order their first tools come in. */
function grouped(tools: Tool[]): [string, Tool[]][] {
  const groups = new Map<string, Tool[]>();
  for (const tool of tools) groups.set(tool.group, [...(groups.get(tool.group) ?? []), tool]);
  return [...groups];
}

/** `tools` as entries of a menu. */
function entries(tools: Tool[]): MenuEntry[] {
  return tools.map((t) => ({
    kind: "item",
    label: t.label,
    icon: t.kind,
    title: t.doc,
    disabled: t.disabled,
    active: t.active,
    onSelect: t.onSelect,
  }));
}

/** `items` in columns of `n`, top to bottom. */
function columns<T>(items: T[], n: number): T[][] {
  const out: T[][] = [];
  for (let i = 0; i < items.length; i += n) out.push(items.slice(i, i + n));
  return out;
}

/** A tool as a button: `big` with its icon over its label, else small, icon beside label. */
function ToolButton({ tool, big }: { tool: Tool; big: boolean }) {
  return (
    <button
      title={tool.doc}
      className={["op-button", big ? "big" : "small", tool.active ? "active" : ""].join(" ")}
      disabled={tool.disabled}
      onClick={tool.onSelect}
    >
      <Icon name={tool.kind} />
      <span>{tool.label}</span>
    </button>
  );
}

/**
 * How much of a group is shown: everything (its primary tools big, the rest
 * small and stacked), only its primary tools, only a menu of it, or nothing
 * — it is in the overflow menu then.
 */
type Level = 0 | 1 | 2 | 3;

/** A group at `level` (not 3): its tools, and its name as the menu of all of them. */
function Group({ name, tools, level }: { name: string; tools: Tool[]; level: Level }) {
  // A group of one tool shows it big: there is nothing to stack it with.
  const big = (t: Tool) => t.primary || tools.length === 1;
  const primary = tools.filter(big);
  const rest = tools.filter((t) => !big(t));
  const active = tools.some((t) => t.active);
  if ((level >= 2 && tools.length > 1) || (level === 1 && primary.length === 0)) {
    return (
      <div className="ribbon-group collapsed" role="group" aria-label={name}>
        <Menu
          label={
            <>
              <Icon name={tools[0].kind} />
              <span>{name}</span>
            </>
          }
          title={`${name}: ${tools.map((t) => t.label).join(", ")}`}
          entries={entries(tools)}
          className="ribbon-menu"
          triggerClassName={["big", active ? "active" : ""].join(" ")}
        />
      </div>
    );
  }
  return (
    <div className="ribbon-group" role="group" aria-label={name}>
      <div className="ribbon-buttons">
        {primary.map((t) => (
          <ToolButton key={t.kind} tool={t} big />
        ))}
        {level === 0 &&
          columns(rest, 3).map((column) => (
            <div key={column[0].kind} className="ribbon-column">
              {column.map((t) => (
                <ToolButton key={t.kind} tool={t} big={false} />
              ))}
            </div>
          ))}
      </div>
      <Menu
        label={name}
        title={`Every operation of ${name}`}
        entries={entries(tools)}
        className="ribbon-menu"
        triggerClassName={["ribbon-label", level === 1 && rest.some((t) => t.active) ? "active" : ""].join(" ")}
      />
    </div>
  );
}

/** The groups at `levels`, and the overflow menu of those at 3. */
function Groups({ groups, levels }: { groups: [string, Tool[]][]; levels: Level[] }): ReactNode {
  const over = groups.filter((_, i) => levels[i] === 3);
  return (
    <>
      {groups.map(([name, tools], i) =>
        levels[i] === 3 ? null : <Group key={name} name={name} tools={tools} level={levels[i]} />,
      )}
      {over.length > 0 && (
        <div className="ribbon-group collapsed" role="group" aria-label="More">
          <Menu
            label={
              <>
                <Icon name="more" />
                <span>More</span>
              </>
            }
            title={`More operations: ${over.map(([name]) => name).join(", ")}`}
            entries={over.flatMap(([name, tools]): MenuEntry[] => [
              { kind: "heading", label: name },
              ...entries(tools),
            ])}
            className="ribbon-menu ribbon-overflow"
            triggerClassName={["big", over.some(([, tools]) => tools.some((t) => t.active)) ? "active" : ""].join(" ")}
            align="right"
          />
        </div>
      )}
    </>
  );
}

/**
 * The levels the groups are shown at so they fit `available` pixels: the
 * earlier a group, the more it matters. Groups give way level by level,
 * each level from the last group: first their small tools, then their
 * primary ones, then their place in the bar — no primary tool goes while
 * a small one is still shown. What room that leaves goes back to the
 * first groups, as far as it reaches. `widths[i][l]` is group `i`'s at
 * level `l`.
 */
function fit(widths: number[][], overflow: number, available: number): Level[] {
  const levels: Level[] = widths.map(() => 0);
  const fits = () =>
    levels.reduce<number>((sum, l, i) => sum + (l === 3 ? 0 : widths[i][l]), 0) + (levels.includes(3) ? overflow : 0) <=
    available;
  give: for (const level of [1, 2, 3] as const) {
    for (let i = widths.length - 1; i >= 0; i--) {
      if (fits()) break give;
      levels[i] = level;
    }
  }
  for (let i = 0; i < levels.length; i++) {
    while (levels[i] > 0) {
      levels[i]--;
      if (!fits()) {
        levels[i]++;
        break;
      }
    }
  }
  return levels;
}

/**
 * The operations of the desktop toolbar, as a CAD ribbon: a section per
 * group, its most used operations big, the others small and stacked in
 * columns, and its name a menu of all of them. Where the window is too
 * narrow, groups give way from the end — to their primary operations, to
 * a menu, to the overflow menu — so every operation stays a click or two
 * away and the bar never scrolls.
 *
 * Each group is measured at each level once, in a hidden copy, whenever
 * the tools' labels change; the levels are then chosen for the width the
 * ribbon has, whenever that changes.
 */
export function OperationRibbon({ tools }: { tools: Tool[] }) {
  const groups = grouped(tools);
  const root = useRef<HTMLDivElement>(null);
  const measure = useRef<HTMLDivElement>(null);
  const [available, setAvailable] = useState<number | null>(null);
  const [widths, setWidths] = useState<{ groups: number[][]; overflow: number } | null>(null);
  const shape = JSON.stringify(groups.map(([name, ts]) => [name, ts.map((t) => [t.label, t.primary])]));

  useLayoutEffect(() => {
    const el = root.current;
    if (!el) return;
    const observer = new ResizeObserver(() => setAvailable(el.clientWidth));
    observer.observe(el);
    setAvailable(el.clientWidth);
    return () => observer.disconnect();
  }, []);
  useLayoutEffect(() => {
    const el = measure.current;
    if (!el) return;
    const read = () => {
      const width = (selector: string) =>
        [...el.querySelectorAll(selector)].map((e) => e.getBoundingClientRect().width);
      const levels = [0, 1, 2].map((l) => width(`.ribbon-measure-level-${l} > .ribbon-group`));
      setWidths({
        groups: groups.map((_, i) => levels.map((l) => Math.ceil(l[i]))),
        overflow: Math.ceil(width(".ribbon-measure-overflow > .ribbon-group")[0] ?? 0),
      });
    };
    read();
    // The labels' font may arrive later, and widen them.
    void document.fonts?.ready.then(read);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [shape]);

  const levels: Level[] =
    widths && available != null && widths.groups.length === groups.length
      ? fit(widths.groups, widths.overflow, available)
      : groups.map(() => 2);
  return (
    <div ref={root} className="desktop-only operation-strip">
      <div className="operation-ribbon" role="toolbar" aria-label="Operations">
        <Groups groups={groups} levels={levels} />
      </div>
      <div ref={measure} className="ribbon-measure" aria-hidden="true" inert>
        {([0, 1, 2] as const).map((level) => (
          <div key={level} className={`ribbon-measure-level-${level}`}>
            {groups.map(([name, ts]) => (
              <Group key={name} name={name} tools={ts} level={level} />
            ))}
          </div>
        ))}
        <div className="ribbon-measure-overflow">
          <Groups groups={groups.slice(0, 1)} levels={[3]} />
        </div>
      </div>
    </div>
  );
}

/** The operations on a phone: a section per group, every button in it. */
export function OperationGrid({ tools }: { tools: Tool[] }) {
  return (
    <div className="operation-grid" role="toolbar" aria-label="Operations">
      {grouped(tools).map(([name, ts]) => (
        <section key={name} className="operation-grid-group" aria-label={name}>
          <h3>{name}</h3>
          <div className="operation-grid-buttons">
            {ts.map((t) => (
              <ToolButton key={t.kind} tool={t} big={false} />
            ))}
          </div>
        </section>
      ))}
    </div>
  );
}
