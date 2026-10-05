import { useLayoutEffect, useRef, useState, type ReactNode } from "react";
import type { OperationTier } from "./geop";
import { Icon } from "./icons";
import { DropdownArrow, Menu, type MenuEntry } from "./Menu";

/** A button of the operations: an operation to start a step of, or a tool of the editor's. */
export interface Tool {
  /** Its icon's name, and what tells it from the others. */
  kind: string;
  label: string;
  /** What it does, at length: its tooltip. */
  doc: string;
  /** The section of the toolbar it is in (see `geop_ops::OperationGroup`). */
  group: string;
  /** How prominently its section shows it (see `geop_ops::OperationTier`). */
  tier: OperationTier;
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

/** `tools` as entries of a menu: those its group shows as buttons, a line, and those only in it. */
function entries(tools: Tool[]): MenuEntry[] {
  return tools.flatMap((t, i): MenuEntry[] => [
    ...(t.tier === "Menu" && i > 0 && tools[i - 1].tier !== "Menu" ? [{ kind: "separator" } as MenuEntry] : []),
    {
      kind: "item",
      label: t.label,
      icon: t.kind,
      title: t.doc,
      disabled: t.disabled,
      active: t.active,
      onSelect: t.onSelect,
    },
  ]);
}

/** `items` in columns of `n`, top to bottom. */
function columns<T>(items: T[], n: number): T[][] {
  const out: T[][] = [];
  for (let i = 0; i < items.length; i += n) out.push(items.slice(i, i + n));
  return out;
}

/** A tool as a button: `big`, its icon over its label and as high as the ribbon, else one row, icon beside label. */
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
 * How much of a group is shown, from the most to the least room it takes:
 * 0, its big tools big and its small ones in rows beside them; 1, only its
 * big tools, big; 2, only its big tools, in rows; 3, a single button of
 * its menu; 4, nothing — it is in the overflow menu. Whatever a group does
 * not show as a button is in its menu, which its caption opens and which
 * lists all of it.
 */
type Level = 0 | 1 | 2 | 3 | 4;

/** The tools a group shows as buttons at `level`: those big, and those in rows. */
function shown(tools: Tool[], level: Level): { big: Tool[]; rows: Tool[] } {
  const big = tools.filter((t) => t.tier === "Big");
  const small = tools.filter((t) => t.tier === "Small");
  switch (level) {
    case 0:
      return { big, rows: small };
    case 1:
      return { big, rows: [] };
    case 2:
      return { big: [], rows: big };
    default:
      return { big: [], rows: [] };
  }
}

/** The label of a group collapsed to its menu: an icon over its name, with the menu's arrow after it. */
function Collapsed({ icon, name }: { icon: string; name: string }) {
  return (
    <>
      <Icon name={icon} />
      <span className="ribbon-caption-text">
        {name}
        <DropdownArrow />
      </span>
    </>
  );
}

/** A group at `level` (not 4): its buttons, and under them its caption, the menu of all of it. */
function Group({ name, tools, level }: { name: string; tools: Tool[]; level: Level }) {
  const { big, rows } = shown(tools, level);
  const hiddenActive = tools.some((t) => t.active && !big.includes(t) && !rows.includes(t));
  const title = `${name}: ${tools.map((t) => t.label).join(", ")}`;
  if (level >= 3) {
    return (
      <div className="ribbon-group collapsed" role="group" aria-label={name}>
        <Menu
          label={<Collapsed icon={tools[0].kind} name={name} />}
          title={title}
          entries={entries(tools)}
          className="ribbon-menu"
          triggerClassName={["ribbon-collapsed", hiddenActive ? "active" : ""].join(" ")}
          arrow={false}
        />
      </div>
    );
  }
  return (
    <div className="ribbon-group" role="group" aria-label={name}>
      <div className="ribbon-buttons">
        {big.map((t) => (
          <ToolButton key={t.kind} tool={t} big />
        ))}
        {columns(rows, 3).map((column) => (
          <div key={column[0].kind} className="ribbon-column">
            {column.map((t) => (
              <ToolButton key={t.kind} tool={t} big={false} />
            ))}
          </div>
        ))}
      </div>
      <Menu
        label={<span className="ribbon-caption-text">{name}</span>}
        title={title}
        entries={entries(tools)}
        className="ribbon-menu"
        triggerClassName={["ribbon-caption", hiddenActive ? "active" : ""].join(" ")}
      />
    </div>
  );
}

/** The groups at `levels`, and the overflow menu of those at 4. */
function Groups({ groups, levels }: { groups: [string, Tool[]][]; levels: Level[] }): ReactNode {
  const over = groups.filter((_, i) => levels[i] === 4);
  return (
    <>
      {groups.map(([name, tools], i) =>
        levels[i] === 4 ? null : <Group key={name} name={name} tools={tools} level={levels[i]} />,
      )}
      {over.length > 0 && (
        <div className="ribbon-group collapsed" role="group" aria-label="More">
          <Menu
            label={<Collapsed icon="more" name="More" />}
            arrow={false}
            title={`More operations: ${over.map(([name]) => name).join(", ")}`}
            entries={over.flatMap(([name, tools]): MenuEntry[] => [
              { kind: "heading", label: name },
              ...entries(tools),
            ])}
            className="ribbon-menu ribbon-overflow"
            triggerClassName={["ribbon-collapsed", over.some(([, tools]) => tools.some((t) => t.active)) ? "active" : ""].join(
              " ",
            )}
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
 * each level from the last group — first their small tools, then their big
 * ones shrink to rows, then they collapse to a menu, then into the overflow
 * — so no big button shrinks while a small one is still shown. What room
 * that leaves goes back to the groups still open, the earliest first, as
 * far as it reaches.
 * `widths[i][l]` is group `i`'s width at level `l` (to 3). A level wider
 * than the one before it is skipped, and so is one only as wide, except a
 * group's collapse: once groups collapse, those it costs no room to
 * collapse do too, and the bar stays one of menus rather than a mix.
 */
function fit(widths: number[][], overflow: number, available: number): Level[] {
  const steps: Level[][] = widths.map((w) => {
    const s: Level[] = [0];
    for (const l of [1, 2, 3] as const) {
      const before = w[s[s.length - 1]];
      if (w[l] < before || (l === 3 && w[l] === before)) s.push(l);
    }
    return [...s, 4];
  });
  const at = widths.map(() => 0);
  const levels = () => at.map((p, i) => steps[i][p]);
  const fits = () => {
    const ls = levels();
    const sum = ls.reduce<number>((sum, l, i) => sum + (l === 4 ? 0 : widths[i][l]), 0);
    return sum + (ls.includes(4) ? overflow : 0) <= available;
  };
  give: for (const level of [1, 2, 3, 4] as const) {
    for (let i = widths.length - 1; i >= 0; i--) {
      if (fits()) break give;
      while (at[i] + 1 < steps[i].length && steps[i][at[i] + 1] <= level) at[i]++;
    }
  }
  for (let i = 0; i < at.length; i++) {
    while (at[i] > 0 && steps[i][at[i]] < 3) {
      at[i]--;
      if (!fits()) {
        at[i]++;
        break;
      }
    }
  }
  return levels();
}

/**
 * The operations of the desktop toolbar, as a CAD ribbon: a section per
 * group, divided from the next, its few most used operations big, the
 * common ones small and stacked in rows beside them, and its caption a menu
 * of all of it — the rarely used operations are only there. Where the
 * window is too narrow, groups give way from the end (see `fit`), so every
 * operation stays a click or two away and the bar never scrolls.
 *
 * Each group is measured at each level once, in a hidden copy, whenever
 * the tools change; the levels are then chosen for the width the ribbon
 * has, whenever that changes.
 */
export function OperationRibbon({ tools }: { tools: Tool[] }) {
  const groups = grouped(tools);
  const root = useRef<HTMLDivElement>(null);
  const measure = useRef<HTMLDivElement>(null);
  const [available, setAvailable] = useState<number | null>(null);
  const [widths, setWidths] = useState<{ groups: number[][]; overflow: number } | null>(null);
  const shape = JSON.stringify(groups.map(([name, ts]) => [name, ts.map((t) => [t.label, t.tier])]));

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
      const levels = [0, 1, 2, 3].map((l) => width(`.ribbon-measure-level-${l} > .ribbon-group`));
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
      : groups.map(() => 3);
  return (
    <div ref={root} className="desktop-only operation-strip">
      <div className="operation-ribbon" role="toolbar" aria-label="Operations">
        <Groups groups={groups} levels={levels} />
      </div>
      <div ref={measure} className="ribbon-measure" aria-hidden="true" inert>
        {([0, 1, 2, 3] as const).map((level) => (
          <div key={level} className={`ribbon-measure-level-${level}`}>
            {groups.map(([name, ts]) => (
              <Group key={name} name={name} tools={ts} level={level} />
            ))}
          </div>
        ))}
        <div className="ribbon-measure-overflow">
          <Groups groups={groups.slice(0, 1)} levels={[4]} />
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
