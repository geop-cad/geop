// The app's own icons, by name: of its menus, panels and tools. What the
// kernel offers — operations, inspectors, a step's tools — it draws itself,
// sent as SVG markup on the same grid (see [[Icon]]). Drawn on a 20 x 20
// grid in the current colour, so a button's state — active, disabled,
// hovered — colours its icon too.

import type { ReactElement } from "react";

/** A point drawn as a dot. */
const dot = (x: number, y: number, r = 1.6) => <circle cx={x} cy={y} r={r} fill="currentColor" stroke="none" />;

/** Each icon's drawing, by name. */
const ICONS: Record<string, ReactElement> = {
  chevron: <path d="M6 8 L10 12 L14 8" />,
  open: (
    <>
      <path d="M3 6 L3 15.5 L16 15.5 L17.5 8.5 L6 8.5 L4.5 15.5" />
      <path d="M3 6 L3 4.5 L7.5 4.5 L9 6 L14.5 6 L14.5 8.5" />
    </>
  ),
  save: (
    <>
      <path d="M10 3.5 L10 12.5 M6.5 9 L10 12.5 L13.5 9" />
      <path d="M4 13.5 L4 16.5 L16 16.5 L16 13.5" />
    </>
  ),
  example: (
    <>
      <path d="M10 3 L16 6.5 L16 13.5 L10 17 L4 13.5 L4 6.5 Z" />
      <path d="M4 6.5 L10 10 L16 6.5 M10 10 L10 17" />
    </>
  ),
  assembly: (
    <>
      <rect x="3" y="9" width="7" height="7" />
      <path d="M10 12 L13 12 M13 4 L17 4 L17 11 L13 11 Z" />
    </>
  ),
  undo: <path d="M7 5 L3.5 8.5 L7 12 M3.5 8.5 L12 8.5 A4.5 4.5 0 0 1 12 17.5 L9 17.5" />,
  redo: <path d="M13 5 L16.5 8.5 L13 12 M16.5 8.5 L8 8.5 A4.5 4.5 0 0 0 8 17.5 L11 17.5" />,
  help: (
    <>
      <circle cx="10" cy="10" r="7" />
      <path d="M7.8 7.8 A2.3 2.3 0 1 1 10.5 10.1 C10 10.3 10 10.8 10 11.6" />
      {dot(10, 14, 1)}
    </>
  ),
  bug: (
    <>
      <rect x="6.5" y="6" width="7" height="10" rx="3.5" />
      <path d="M8 6 L7 3.5 M12 6 L13 3.5 M6.5 9.5 L3.5 8.5 M6.5 12.5 L3.5 13.5 M13.5 9.5 L16.5 8.5 M13.5 12.5 L16.5 13.5" />
    </>
  ),
  privacy: <path d="M10 3 L16 5.5 L16 10 C16 13.5 13.5 16 10 17 C6.5 16 4 13.5 4 10 L4 5.5 Z" />,
  new_part: (
    <>
      <path d="M10 3 L16 6.5 L16 13.5 L10 17 L4 13.5 L4 6.5 Z" />
      <path d="M10 10 L16 6.5 M10 10 L4 6.5 M10 10 L10 17" />
      <path d="M13 1.5 L13 4.8" strokeWidth="1" />
    </>
  ),
  drag: (
    <>
      <path d="M10 2.5 L10 17.5 M2.5 10 L17.5 10" />
      <path d="M8 4.5 L10 2.5 L12 4.5 M8 15.5 L10 17.5 L12 15.5 M4.5 8 L2.5 10 L4.5 12 M15.5 8 L17.5 10 L15.5 12" />
    </>
  ),
  edit: (
    <>
      <path d="M12.5 4.5 L15.5 7.5 L8 15 L4.5 15.5 L5 12 Z" />
    </>
  ),
  table: (
    <>
      <rect x="3" y="4" width="14" height="12" rx="1" />
      <path d="M3 8 L17 8 M8 8 L8 16" />
    </>
  ),
  number: <path d="M7 4 L5.5 16 M13 4 L11.5 16 M4 8 L16 8 M3.5 12 L15.5 12" />,
  mass: (
    <>
      <path d="M5 8 L15 8 L17 17 L3 17 Z" />
      <circle cx="10" cy="5" r="2.2" />
    </>
  ),
  section: (
    <>
      <path d="M4 7 L10 4 L16 7 L16 13 L10 16 L4 13 Z" />
      <path d="M2 11 L18 9" strokeDasharray="2 1.5" />
    </>
  ),
  more: (
    <>
      {dot(4.5, 10)}
      {dot(10, 10)}
      {dot(15.5, 10)}
    </>
  ),
};

/**
 * The icon `name`: one of the app's own, by its name (`undo`) — or none, for
 * a name there is no icon for — or one the kernel sends, as SVG markup on
 * the same grid (an operation's, an inspector's, a tool's; see
 * `geop_core_program::operation::OperationInfo::icon`), which starts with
 * `<` as no name does.
 */
export function Icon({ name }: { name: string }) {
  if (name.startsWith("<")) return <svg {...FRAME} dangerouslySetInnerHTML={{ __html: name }} />;
  const drawing = ICONS[name];
  if (!drawing) return null;
  return <svg {...FRAME}>{drawing}</svg>;
}

/** The `<svg>` every icon is drawn in. */
const FRAME = {
  className: "icon",
  viewBox: "0 0 20 20",
  width: "20",
  height: "20",
  fill: "none",
  stroke: "currentColor",
  strokeWidth: "1.5",
  strokeLinecap: "round",
  strokeLinejoin: "round",
  "aria-hidden": true,
} as const;
