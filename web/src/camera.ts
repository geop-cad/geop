// The 3-D camera's pose, and what a screen pixel measures in the world.
//
// Working in a plane (see `Presentation.focus`), the camera turns to face
// it head on, from as far away as it was: the view rotates into the plane
// rather than cutting to somewhere else.

import * as THREE from "three";
import type { Extent, Frame, Vec3 } from "./geop";

/** Where the camera is, what it looks at, and which way is up for it. */
export interface CameraPose {
  position: Vec3;
  target: Vec3;
  up: Vec3;
}

/** The camera's vertical field of view, in degrees. */
export const CAMERA_FOV = 50;

/**
 * How the 3-D view projects. Both show the same thing at the camera's
 * target: an orthographic view is sized from the camera's distance to it,
 * exactly as a perspective one is, so "this far back" means "this scale" in
 * either — and switching keeps the view.
 */
export type Projection = "perspective" | "orthographic";

/**
 * How far the pointer reaches, in pixels: what counts as under it (see
 * `geop_ops::ui::Reach`). Whatever is drawn at a constant size on screen is
 * laid out in reaches in the kernel, and drawn here as this many pixels
 * each.
 */
export const REACH_PX = 9;

/** What one screen pixel measures, in world units, at `at` — for a viewport `height` pixels tall. */
export function worldPerPixel(camera: THREE.Camera, at: THREE.Vector3, height: number): number {
  if (camera instanceof THREE.OrthographicCamera) return (camera.top - camera.bottom) / camera.zoom / Math.max(height, 1);
  return 1 / scaleForDistance(camera.position.distanceTo(at), Math.max(height, 1));
}

/** The view a fresh session starts from: an empty part's drawing (100 mm, see `geop_ops::ui::EMPTY_EXTENT`) framed. */
export const DEFAULT_POSE: CameraPose = { position: [66, 44, 88], target: [0, 0, 0], up: [0, 1, 0] };

/** What a drawing is, for whether to frame it: its extent, and how many solids and placed parts it has. */
export interface Drawn {
  extent: Extent;
  solids: number;
}

/**
 * Whether the drawing, `before` and now `after`, is to be framed from a
 * camera at `pose`: when its first solid or placed part appears, or when it
 * grew by half again and is now twice as big as the view shows. Any other
 * edit leaves the camera where the user put it — zoomed in on a detail, say.
 */
export function wantsFraming(before: Drawn, after: Drawn, pose: CameraPose): boolean {
  if (before.solids === 0 && after.solids > 0) return true;
  const distance = Math.hypot(...[0, 1, 2].map((k) => pose.position[k] - pose.target[k]));
  const shown = 2 * distance * Math.tan((CAMERA_FOV * Math.PI) / 360);
  return after.extent.size > 1.5 * before.extent.size && after.extent.size > 2 * shown;
}

/** A point of the plane `frame`, `height` above it, in world coordinates. */
function planeToWorld(frame: Frame, p: [number, number], height = 0): Vec3 {
  return [0, 1, 2].map(
    (k) => frame.origin[k] + p[0] * frame.u[k] + p[1] * frame.v[k] + height * frame.w[k],
  ) as Vec3;
}

/** What one unit covers on screen from `distance` away, for a viewport `height` pixels tall. */
export function scaleForDistance(distance: number, height: number): number {
  return height / (2 * distance * Math.tan((CAMERA_FOV * Math.PI) / 360));
}

/**
 * The same view the camera has now, turned as little as it can to face
 * `frame` head-on: same look-at point (projected onto the plane) and the
 * same distance, from the side of the plane the camera is on already, and
 * with whichever of the plane's directions — `±u`, `±v` — is nearest the
 * camera's up as up. So working in a plane rotates the view into it rather
 * than cutting to somewhere else, or turning it on its side.
 */
export function headOnPose(frame: Frame, pose: CameraPose): CameraPose {
  const dot = (a: Vec3 | number[], b: Vec3 | number[]) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const d = [0, 1, 2].map((k) => pose.target[k] - frame.origin[k]);
  const center: [number, number] = [dot(d, frame.u), dot(d, frame.v)];
  const eye = [0, 1, 2].map((k) => pose.position[k] - pose.target[k]);
  const distance = Math.hypot(eye[0], eye[1], eye[2]);
  const side = dot(eye, frame.w) < 0 ? -1 : 1;
  const negate = (a: Vec3): Vec3 => [-a[0], -a[1], -a[2]];
  const ups: Vec3[] = [frame.v, frame.u, negate(frame.u), negate(frame.v)];
  const up = ups.reduce((best, a) => (dot(a, pose.up) > dot(best, pose.up) ? a : best));
  return {
    position: planeToWorld(frame, center, side * distance),
    target: planeToWorld(frame, center),
    up,
  };
}

/** How far in from a side of the view cube its edges and corners reach, as a share of its size. */
export const CUBE_BAND = 0.2;

/**
 * The region of the view cube — 1 across, about the origin, its faces
 * square to the world's axes — that its surface's point `p` lies in: a
 * face, an edge or a corner, as the direction from the cube's centre to
 * it, each component -1, 0 or 1. A point within [[CUBE_BAND]] of a side
 * of the face it is on is on that side's edge; of two, on their corner.
 */
export function cubeRegion(p: Vec3): Vec3 {
  const largest = Math.max(...p.map(Math.abs));
  return p.map((x) => (Math.abs(x) === largest || Math.abs(x) > 0.5 - CUBE_BAND ? Math.sign(x) : 0)) as Vec3;
}

/** What the view cube's region `region` (see [[cubeRegion]]) is called: `Top`, `Front Right`, `Top Front Right`. */
export function regionName(region: Vec3): string {
  const [x, y, z] = region;
  const named: [number, string, string][] = [
    [y, "Top", "Bottom"],
    [z, "Front", "Back"],
    [x, "Right", "Left"],
  ];
  return named
    .filter(([k]) => k !== 0)
    .map(([k, plus, minus]) => (k > 0 ? plus : minus))
    .join(" ");
}

/**
 * The standard view along `direction` — from the side it points to,
 * towards the target — of the camera at `pose`: the same target and
 * distance, and the world's `y` up; looking straight down, the back
 * (`-z`) up, so the front lies at the foot of the view, as on a drawing,
 * and looking straight up, the front up.
 */
export function standardPose(direction: Vec3, pose: CameraPose): CameraPose {
  const length = Math.hypot(...direction);
  const d = direction.map((x) => x / length) as Vec3;
  const distance = Math.hypot(...[0, 1, 2].map((k) => pose.position[k] - pose.target[k]));
  const across = [0, 1, 0].map((y, k) => y - d[k] * d[1]);
  const n = Math.hypot(...across);
  const up: Vec3 = d[0] === 0 && d[2] === 0 ? [0, 0, -Math.sign(d[1])] : (across.map((x) => x / n) as Vec3);
  return {
    position: [0, 1, 2].map((k) => pose.target[k] + d[k] * distance) as Vec3,
    target: pose.target,
    up,
  };
}

/**
 * The view that frames `extent` — the ball around its box — in a viewport
 * of `aspect` (width over height): looking at its centre from the
 * direction, and with the up, `pose` has, from just far enough that the
 * ball fits both ways.
 */
export function fitPose(extent: Extent, pose: CameraPose, aspect: number): CameraPose {
  const eye = [0, 1, 2].map((k) => pose.position[k] - pose.target[k]);
  const length = Math.hypot(eye[0], eye[1], eye[2]);
  const direction = length > 0 ? eye.map((x) => x / length) : [0, 0, 1];
  const half = (CAMERA_FOV * Math.PI) / 360;
  const narrowest = Math.min(half, Math.atan(Math.tan(half) * aspect));
  const distance = extent.size / 2 / Math.sin(narrowest);
  return {
    position: [0, 1, 2].map((k) => extent.center[k] + direction[k] * distance) as Vec3,
    target: extent.center,
    up: pose.up,
  };
}
