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
  trim: (
    <>
      <path d="M3 10 L17 10" strokeDasharray="1.5 1.5" />
      <path d="M7 3 L7 17 M13 3 L13 17" />
      <path d="M8.5 6 L11.5 14 M11.5 6 L8.5 14" strokeWidth="1.2" />
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
  // ── the app ─────────────────────────────────────────────────────────
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
  // ── operations, by kind ─────────────────────────────────────────────
  add_sketch: (
    <>
      <path d="M3.5 16.5 L4.5 12.5 L13 4 L16 7 L7.5 15.5 Z" />
      <path d="M11.5 5.5 L14.5 8.5" />
    </>
  ),
  extrude: (
    <>
      <path d="M4 12 L10 15 L16 12 L10 9 Z" />
      <path d="M10 8 L10 2.5 M7.8 4.7 L10 2.5 L12.2 4.7" />
    </>
  ),
  revolve: (
    <>
      <path d="M10 2.5 L10 17.5" strokeDasharray="1.5 1.5" />
      <path d="M15.5 10 A5.5 2.5 0 1 1 13 7.9" />
      <path d="M12 6 L13.4 8 L11.2 8.8" />
    </>
  ),
  sweep: (
    <>
      <path d="M3 15 C3 9 7 6 11 6 L16.5 6" strokeDasharray="1.5 1.5" />
      <ellipse cx="3" cy="15" rx="1.8" ry="2.5" />
      <path d="M14.5 3.8 L16.8 6 L14.5 8.2" />
    </>
  ),
  loft: (
    <>
      <rect x="4" y="12" width="12" height="5" />
      <ellipse cx="10" cy="4.5" rx="3.5" ry="1.8" />
      <path d="M4 12 L6.5 4.5 M16 12 L13.5 4.5" />
    </>
  ),
  boolean: (
    <>
      <rect x="3" y="3" width="9" height="9" />
      <rect x="8" y="8" width="9" height="9" />
    </>
  ),
  split: (
    <>
      <path d="M3 4 L10 4 L8 16 L3 16 Z" />
      <path d="M12.5 4 L17 4 L17 16 L10.5 16 Z" />
    </>
  ),
  delete_body: (
    <>
      <path d="M4 6 L16 6 M8 6 L8 4 L12 4 L12 6" />
      <path d="M5.5 6 L6.5 16.5 L13.5 16.5 L14.5 6" />
    </>
  ),
  extract_face: (
    <>
      <path d="M3 10 L8 12.5 L13 10 L8 7.5 Z" strokeDasharray="1.5 1.5" />
      <path d="M7 5.5 L12 8 L17 5.5 L12 3 Z" />
    </>
  ),
  project_curve: (
    <>
      <path d="M4 4 C7 2 10 6 14 3.5" />
      <path d="M3 13 L10 16.5 L17 13 L10 9.5 Z" />
      <path d="M9 5 L9 11.5 M7.5 10 L9 11.5 L10.5 10" strokeWidth="1" />
    </>
  ),
  // `fillet` is the sketch tool's icon above, and the fillet operation's.
  chamfer: (
    <>
      <path d="M4 16 L4 10 L10 4 L16 4" />
      <path d="M4 10 L4 4 L10 4" strokeDasharray="1.5 1.5" />
    </>
  ),
  shell: (
    <>
      <path d="M3 4 L3 16 L17 16 L17 4" />
      <path d="M6 4 L6 13 L14 13 L14 4" />
    </>
  ),
  add_datum: (
    <>
      <path d="M2.5 13 L7 8 L17.5 8 L13 13 Z" />
      <path d="M10 2.5 L10 17.5" />
    </>
  ),
  add_part: (
    <>
      <path d="M10 3 L16 6.5 L16 13.5 L10 17 L4 13.5 L4 6.5 Z" />
      <path d="M10 10 L16 6.5 M10 10 L4 6.5 M10 10 L10 17" />
      <path d="M13 1.5 L13 4.8" strokeWidth="1" />
    </>
  ),
  linear_pattern: (
    <>
      <rect x="2.5" y="7.5" width="4" height="5" />
      <rect x="8" y="7.5" width="4" height="5" strokeDasharray="1.5 1.5" />
      <rect x="13.5" y="7.5" width="4" height="5" strokeDasharray="1.5 1.5" />
      <path d="M3 16 L17 16 M15.5 14.5 L17 16 L15.5 17.5" strokeWidth="1" />
    </>
  ),
  circular_pattern: (
    <>
      <circle cx="10" cy="10" r="6" strokeWidth="1" />
      <rect x="8.5" y="2" width="3" height="3" />
      <rect x="14.5" y="8.5" width="3" height="3" strokeDasharray="1.2 1.2" />
      <rect x="8.5" y="15" width="3" height="3" strokeDasharray="1.2 1.2" />
      <rect x="2.5" y="8.5" width="3" height="3" strokeDasharray="1.2 1.2" />
    </>
  ),
  mirror: (
    <>
      <path d="M10 2.5 L10 17.5" strokeDasharray="1.5 1.5" />
      <path d="M8 5 L3 7 L3 14 L8 15 Z" />
      <path d="M12 5 L17 7 L17 14 L12 15 Z" strokeDasharray="1.5 1.5" />
    </>
  ),
  move_body: (
    <>
      <rect x="3" y="9" width="6" height="6" strokeDasharray="1.5 1.5" />
      <rect x="11" y="4" width="6" height="6" />
      <path d="M7 8 L11.5 4.5 M9 4.5 L11.5 4.5 L11.5 7" strokeWidth="1" />
    </>
  ),
  route: (
    <>
      <rect x="1.5" y="12.5" width="3.5" height="4" rx="0.5" />
      <rect x="15" y="3.5" width="3.5" height="4" rx="0.5" />
      <path d="M5 14.5 L7 14.5 C 11 14.5, 9 5.5, 13 5.5 L15 5.5" />
      <circle cx="10" cy="10" r="1.6" strokeWidth="1" />
    </>
  ),
  hole: (
    <>
      <path d="M2.5 6 L17.5 6 M2.5 6 L2.5 17 M17.5 6 L17.5 17" />
      <path d="M5 6 L5 9 L7.5 9 L7.5 17 M15 6 L15 9 L12.5 9 L12.5 17" />
      <path d="M10 2 L10 17" strokeWidth="1" strokeDasharray="1.5 1.5" />
    </>
  ),
  thread: (
    <>
      <path d="M6 3 L6 17 M14 3 L14 17" />
      <path d="M6 5 L14 7 M6 9 L14 11 M6 13 L14 15" strokeWidth="1" />
    </>
  ),
  boundary_surface: (
    <>
      <path d="M3 15 C6 13 10 16 13 14 L17 6 C14 7 10 4 7 6 Z" />
      <path d="M5 10.5 C8 9 11 11.5 15 10" strokeDasharray="1.5 1.5" />
    </>
  ),
  offset_surface: (
    <>
      <path d="M3 14 C7 10 12 16 17 12" />
      <path d="M3 8 C7 4 12 10 17 6" strokeDasharray="1.5 1.5" />
      <path d="M10 12.5 L10 7.5 M8.5 9 L10 7.5 L11.5 9" strokeWidth="1" />
    </>
  ),
  thicken: (
    <>
      <path d="M3 13 C7 9 12 15 17 11 L17 7.5 C12 11.5 7 5.5 3 9.5 Z" />
    </>
  ),
  knit: (
    <>
      <path d="M3 5 L9 5 L9 15 L3 15 Z" />
      <path d="M11 5 L17 5 L17 15 L11 15 Z" />
      <path d="M8 8 L12 8 M8 12 L12 12" strokeWidth="1" />
    </>
  ),
  trim_surface: (
    <>
      <path d="M3 6 L10 6 L10 16 L3 16 Z" />
      <path d="M10 6 L17 6 L17 16 L10 16" strokeDasharray="1.5 1.5" />
      <path d="M10 3 L10 18.5" strokeWidth="1" />
    </>
  ),
  extend_surface: (
    <>
      <path d="M3 5 L11 5 L11 15 L3 15 Z" />
      <path d="M11 5 L16 5 L16 15 L11 15" strokeDasharray="1.5 1.5" />
      <path d="M12.5 10 L17.5 10 M15.5 8 L17.5 10 L15.5 12" strokeWidth="1" />
    </>
  ),
  base_flange: (
    <>
      <path d="M3 13 L10 16.5 L17 13 L10 9.5 Z" />
      <path d="M3 13 L3 14.5 L10 18 L17 14.5 L17 13" strokeWidth="1" />
    </>
  ),
  edge_flange: (
    <>
      <path d="M3 15 L12 15 Q15 15 15 12 L15 4" />
      <path d="M3 17 L12 17 Q17 17 17 12 L17 4" />
    </>
  ),
  flat_pattern: (
    <>
      <rect x="3" y="7" width="14" height="6" />
      <path d="M8 7 L8 13 M12 7 L12 13" strokeDasharray="1.5 1.5" />
    </>
  ),
  subd: (
    <>
      <path d="M4 5 L16 5 L16 15 L4 15 Z" strokeDasharray="1.5 1.5" />
      <path d="M10 6.5 C14.5 6.5 14.5 13.5 10 13.5 C5.5 13.5 5.5 6.5 10 6.5 Z" />
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
  add_sketch3d: (
    <>
      <path d="M3 16 L8 13 L8 6 L16 3" />
      <path d="M3 16 L3 11 M3 16 L7 17" strokeDasharray="1.2 1.2" />
      {dot(8, 13, 1.4)}
      {dot(8, 6, 1.4)}
    </>
  ),
  part_pattern: (
    <>
      <circle cx="10" cy="10" r="6.5" strokeDasharray="1.5 1.5" strokeWidth="1" />
      {dot(10, 3.5, 1.6)}
      {dot(15.6, 13.2, 1.6)}
      {dot(4.4, 13.2, 1.6)}
    </>
  ),
  draft: (
    <>
      <path d="M3 16 L17 16 L14 4 L6 4 Z" />
      <path d="M6 4 L3 4 L3 16" strokeDasharray="1.5 1.5" />
    </>
  ),
  lip: (
    <>
      <path d="M3 16 L3 8 L9 8 L9 4 L12 4 L12 8 L17 8 L17 16" />
    </>
  ),
  groove: (
    <>
      <path d="M3 16 L3 6 L8 6 L8 10 L12 10 L12 6 L17 6 L17 16" />
    </>
  ),
  rib: (
    <>
      <path d="M3 4 L3 16 L17 16 L17 4" />
      <path d="M9 16 L9 8 L11 8 L11 16" />
    </>
  ),
  // ── inspecting ──────────────────────────────────────────────────────
  measure: (
    <>
      <path d="M2.5 13 L13 2.5 L17.5 7 L7 17.5 Z" />
      <path d="M6 9.5 L7.5 11 M8.5 7 L10.5 9 M11 4.5 L12.5 6" strokeWidth="1" />
    </>
  ),
  mass: (
    <>
      <path d="M5 8 L15 8 L17 17 L3 17 Z" />
      <circle cx="10" cy="5" r="2.2" />
    </>
  ),
  interference: (
    <>
      <rect x="3" y="3" width="9" height="9" />
      <rect x="8" y="8" width="9" height="9" />
      <path d="M8 12 L12 8" strokeWidth="1" />
    </>
  ),
  section: (
    <>
      <path d="M4 7 L10 4 L16 7 L16 13 L10 16 L4 13 Z" />
      <path d="M2 11 L18 9" strokeDasharray="2 1.5" />
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
