// The 3-D camera's pose, and how it corresponds to a 2-D sketch view.
//
// Entering a sketch, the camera turns to face the sketch plane, and from
// then on the sketch editor — drawn over the 3-D view — drives it: every
// pan and zoom of the 2-D view is a camera pose head-on to the plane. Both
// need the same conversion between "camera at this distance" and "this many
// pixels per sketch unit", which lives here.

import * as THREE from "three";
import type { Frame, Vec3 } from "./geop";
import { toSketch, type P2 } from "./sketchGeometry";

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

/** Where a sketch canvas is looking: `scale` pixels per sketch unit. */
export interface SketchView {
  center: P2;
  scale: number;
  /** The canvas's height in pixels, which is what ties `scale` to a camera distance. */
  height: number;
}

/** A point of the sketch plane, `height` above it, in world coordinates. */
export function planeToWorld(frame: Frame, p: P2, height = 0): Vec3 {
  return [0, 1, 2].map(
    (k) => frame.origin[k] + p[0] * frame.u[k] + p[1] * frame.v[k] + height * frame.normal[k],
  ) as Vec3;
}

/** How far back the camera has to sit for one unit to cover `scale` pixels. */
export function distanceForScale(scale: number, height: number): number {
  return height / (2 * scale * Math.tan((CAMERA_FOV * Math.PI) / 360));
}

/** The inverse: what one unit covers on screen from `distance` away. */
export function scaleForDistance(distance: number, height: number): number {
  return height / (2 * distance * Math.tan((CAMERA_FOV * Math.PI) / 360));
}

/**
 * The camera pose that shows exactly what a 2-D sketch view shows: head-on
 * to the plane, centered on the same point, far enough back for the same
 * scale.
 */
export function poseForSketchView(frame: Frame, view: SketchView): CameraPose {
  const distance = distanceForScale(view.scale, view.height);
  return {
    position: planeToWorld(frame, view.center, distance),
    target: planeToWorld(frame, view.center),
    up: frame.v,
  };
}

/**
 * The same view the camera has now, turned to face `frame` head-on: same
 * look-at point (projected onto the plane) and the same distance, so
 * entering a sketch rotates the view into the plane rather than cutting to
 * somewhere else.
 */
export function headOnPose(frame: Frame, pose: CameraPose): CameraPose {
  const center = toSketch(frame, pose.target).xy;
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
