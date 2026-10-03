// The icons actions are shown as (see `geop_ops::ui::Action::icon`), by
// name: one per drawing tool and per kind of constraint the sketch editor
// offers. Drawn on a 20 x 20 grid in the current colour, so a button's
// state — active, disabled, hovered — colours its icon too.

import type { ReactElement } from "react";

/** A point drawn as a dot. */
const dot = (x: number, y: number, r = 1.6) => <circle cx={x} cy={y} r={r} fill="currentColor" stroke="none" />;

/** Each icon's drawing, by name. */
const ICONS: Record<string, ReactElement> = {
  // ── drawing ─────────────────────────────────────────────────────────
  line: (
    <>
      <path d="M4 16 L16 4" />
      {dot(4, 16)}
      {dot(16, 4)}
    </>
  ),
  rectangle: (
    <>
      <rect x="3.5" y="5.5" width="13" height="9" />
      {dot(3.5, 14.5)}
      {dot(16.5, 5.5)}
    </>
  ),
  center_rectangle: (
    <>
      <rect x="3.5" y="5.5" width="13" height="9" />
      <path d="M3.5 14.5 L16.5 5.5" strokeDasharray="1.5 1.5" />
      {dot(10, 10)}
      {dot(16.5, 5.5)}
    </>
  ),
  three_point_rectangle: (
    <>
      <path d="M3 12 L11 4 L17 10 L9 18 Z" />
      {dot(3, 12)}
      {dot(11, 4)}
      {dot(17, 10)}
    </>
  ),
  circle: (
    <>
      <circle cx="10" cy="10" r="6.5" />
      {dot(10, 10)}
    </>
  ),
  three_point_circle: (
    <>
      <circle cx="10" cy="10" r="6.5" />
      {dot(3.5, 10)}
      {dot(13.25, 4.4)}
      {dot(13.25, 15.6)}
    </>
  ),
  arc: (
    <>
      <path d="M3.5 15 A7 7 0 0 1 16.5 15" />
      {dot(3.5, 15)}
      {dot(10, 8.6)}
      {dot(16.5, 15)}
    </>
  ),
  center_arc: (
    <>
      <path d="M16.5 14 A6.5 6.5 0 0 0 10 7.5" />
      <path d="M10 14 L16.5 14 M10 14 L10 7.5" strokeDasharray="1.5 1.5" />
      {dot(10, 14)}
    </>
  ),
  tangent_arc: (
    <>
      <path d="M2.5 15.5 L9 15.5 A5 5 0 0 0 9 5.5" />
      {dot(9, 15.5)}
    </>
  ),
  polygon: (
    <>
      <path d="M10 3.5 L16.2 8 L13.8 15.3 L6.2 15.3 L3.8 8 Z" />
      {dot(10, 10, 1.2)}
    </>
  ),
  slot: (
    <>
      <path d="M6 6.5 L14 6.5 A3.5 3.5 0 0 1 14 13.5 L6 13.5 A3.5 3.5 0 0 1 6 6.5 Z" />
      {dot(6, 10, 1.2)}
      {dot(14, 10, 1.2)}
    </>
  ),
  spline: (
    <>
      <path d="M3 15 C6 3, 10 17, 17 5" />
      {dot(3, 15)}
      {dot(17, 5)}
    </>
  ),
  point: <>{dot(10, 10, 2.4)}</>,
  fillet: (
    <>
      <path d="M4 16 L4 10 A6 6 0 0 1 10 4 L16 4" />
      <path d="M4 10 L4 4 L10 4" strokeDasharray="1.5 1.5" />
    </>
  ),
  construction: (
    <>
      <path d="M3 17 L17 3" strokeDasharray="2.5 2" />
    </>
  ),
  project: (
    <>
      <path d="M5 3.5 L15 3.5 L15 8.5 L5 8.5 Z" />
      <path d="M10 9.5 L10 13" />
      <path d="M8 11.5 L10 13.5 L12 11.5" />
      <path d="M3 16.5 L17 16.5" />
    </>
  ),
  // ── constraints ─────────────────────────────────────────────────────
  coincident: (
    <>
      <circle cx="10" cy="10" r="4" />
      {dot(10, 10)}
    </>
  ),
  horizontal: (
    <>
      <path d="M3 10 L17 10" />
      <path d="M7 6.5 L7 13.5 M13 6.5 L13 13.5" strokeWidth="1" />
    </>
  ),
  vertical: (
    <>
      <path d="M10 3 L10 17" />
      <path d="M6.5 7 L13.5 7 M6.5 13 L13.5 13" strokeWidth="1" />
    </>
  ),
  parallel: <path d="M5 16 L11 4 M9 16 L15 4" />,
  perpendicular: <path d="M4 16 L16 16 M10 16 L10 4" />,
  tangent: (
    <>
      <circle cx="10" cy="11.5" r="4.5" />
      <path d="M3 7 L17 7" />
    </>
  ),
  collinear: (
    <>
      <path d="M3 15 L8 10 M11 7 L17 3" />
      <path d="M8 10 L11 7" strokeDasharray="1.2 1.2" strokeWidth="1" />
    </>
  ),
  equal: <path d="M5 8 L15 8 M5 12 L15 12" />,
  concentric: (
    <>
      <circle cx="10" cy="10" r="6.5" />
      <circle cx="10" cy="10" r="3.2" />
    </>
  ),
  midpoint: (
    <>
      <path d="M3 13 L17 13" />
      <path d="M10 6 L7 10 L13 10 Z" fill="currentColor" />
    </>
  ),
  symmetric: (
    <>
      <path d="M10 3 L10 17" strokeDasharray="1.5 1.5" />
      {dot(5, 10)}
      {dot(15, 10)}
      <path d="M6.5 10 L8.5 10 M11.5 10 L13.5 10" strokeWidth="1" />
    </>
  ),
  fix: (
    <>
      <rect x="5.5" y="9" width="9" height="7.5" rx="1" />
      <path d="M7.5 9 L7.5 6.5 A2.5 2.5 0 0 1 12.5 6.5 L12.5 9" />
    </>
  ),
  distance: (
    <>
      <path d="M3 5 L3 15 M17 5 L17 15" strokeWidth="1" />
      <path d="M3 10 L17 10 M3 10 L6 8 M3 10 L6 12 M17 10 L14 8 M17 10 L14 12" />
    </>
  ),
  distance_x: (
    <>
      <path d="M3 4 L3 16 M17 4 L17 16" strokeWidth="1" />
      <path d="M3 13 L17 13 M3 13 L6 11 M3 13 L6 15 M17 13 L14 11 M17 13 L14 15" />
      <text x="10" y="9" fontSize="6" textAnchor="middle" fill="currentColor" stroke="none">
        x
      </text>
    </>
  ),
  distance_y: (
    <>
      <path d="M4 3 L16 3 M4 17 L16 17" strokeWidth="1" />
      <path d="M13 3 L13 17 M13 3 L11 6 M13 3 L15 6 M13 17 L11 14 M13 17 L15 14" />
      <text x="7" y="12" fontSize="6" textAnchor="middle" fill="currentColor" stroke="none">
        y
      </text>
    </>
  ),
  radius: (
    <>
      <path d="M17 10 A7 7 0 0 1 3 10" />
      <path d="M10 10 L15 5.5" />
      {dot(10, 10, 1.2)}
      <text x="7" y="8" fontSize="6" textAnchor="middle" fill="currentColor" stroke="none">
        R
      </text>
    </>
  ),
  diameter: (
    <>
      <circle cx="10" cy="10" r="6.5" />
      <path d="M5.4 14.6 L14.6 5.4" />
    </>
  ),
  angle: (
    <>
      <path d="M3 16 L17 16 M3 16 L13 5" />
      <path d="M10 16 A7 7 0 0 0 7.8 10.8" />
    </>
  ),
};

/** The icon `name` — or none, for a name there is no icon for. */
export function Icon({ name }: { name: string }) {
  const drawing = ICONS[name];
  if (!drawing) return null;
  return (
    <svg
      className="icon"
      viewBox="0 0 20 20"
      width="20"
      height="20"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.5"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      {drawing}
    </svg>
  );
}
