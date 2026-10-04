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

/** The view a fresh session starts from. */
export const DEFAULT_POSE: CameraPose = { position: [3, 2, 4], target: [0, 0, 0], up: [0, 1, 0] };

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
