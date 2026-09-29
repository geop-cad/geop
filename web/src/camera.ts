// The 3-D camera's pose, and what a screen pixel measures in the world.
//
// Working in a plane (see `Presentation.focus`), the camera turns to face
// it head on, from as far away as it was: the view rotates into the plane
// rather than cutting to somewhere else.

import * as THREE from "three";
import type { Frame, Vec3 } from "./geop";

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
    (k) => frame.origin[k] + p[0] * frame.u[k] + p[1] * frame.v[k] + height * frame.normal[k],
  ) as Vec3;
}

/** What one unit covers on screen from `distance` away, for a viewport `height` pixels tall. */
export function scaleForDistance(distance: number, height: number): number {
  return height / (2 * distance * Math.tan((CAMERA_FOV * Math.PI) / 360));
}

/**
 * The same view the camera has now, turned to face `frame` head-on: same
 * look-at point (projected onto the plane) and the same distance, so
 * working in a plane rotates the view into it rather than cutting to
 * somewhere else.
 */
export function headOnPose(frame: Frame, pose: CameraPose): CameraPose {
  const d = [0, 1, 2].map((k) => pose.target[k] - frame.origin[k]);
  const along = (a: Vec3) => d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
  const center: [number, number] = [along(frame.u), along(frame.v)];
  const distance = Math.hypot(
    pose.position[0] - pose.target[0],
    pose.position[1] - pose.target[1],
    pose.position[2] - pose.target[2],
  );
  return {
    position: planeToWorld(frame, center, distance),
    target: planeToWorld(frame, center),
    up: frame.v,
  };
}
