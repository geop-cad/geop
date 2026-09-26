// Handles in the 3-D view: drawing the handles steps offer (see
// `geop_ops_parts::operation::Handle`), hit-testing them, and turning a drag
// into new values for the argument paths they name. Nothing here knows any
// operation: a handle says where it is, how it moves, and what it writes.

import * as THREE from "three";
import { worldPerPixel } from "./camera";
import type { ArgPath, StepHandle } from "./geop";

/** A new value for the argument at `path` of a handle's step. */
export interface HandleEdit {
  path: ArgPath;
  value: number;
}

/** How big a handle's ball is on screen, in pixels. */
const RADIUS_PX = 7;
/** Dragged values snap to this: a drag is for rough shaping, a form for exact values. */
const SNAP = 0.01;

const COLOR = new THREE.Color(0xffa040);
const LIT = new THREE.Color(0xffe0a0);

const snap = (v: number) => Math.round(v / SNAP) * SNAP;
const vec = (v: [number, number, number]) => new THREE.Vector3(...v);

/** A stable key for a handle across runs: its step and what it adjusts. */
export const handleKey = (h: StepHandle) => `${h.step}:${h.label}`;

function overlay(): THREE.MeshBasicMaterial {
  return new THREE.MeshBasicMaterial({ color: COLOR, depthTest: false, depthWrite: false, transparent: true });
}

/** One handle's shape at unit size: a ball, with arrows along a linear handle's direction. */
function buildHandle(handle: StepHandle): THREE.Group {
  const group = new THREE.Group();
  group.add(new THREE.Mesh(new THREE.SphereGeometry(1, 16, 12), overlay()));
  if (handle.motion === "linear") {
    const dir = vec(handle.direction).normalize();
    for (const sign of [1, -1]) {
      const cone = new THREE.Mesh(new THREE.ConeGeometry(0.8, 1.6, 12), overlay());
      const d = dir.clone().multiplyScalar(sign);
      cone.position.copy(d).multiplyScalar(2.2);
      cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d);
      group.add(cone);
    }
  }
  group.position.set(...handle.position);
  group.userData.handle = handle;
  group.traverse((o) => (o.renderOrder = 1001));
  return group;
}

/** The handles on screen: rebuilt when the set changes, rescaled and lit every frame. */
export class HandleLayer {
  readonly group = new THREE.Group();
  private signature = "";

  sync(handles: StepHandle[]) {
    const signature = JSON.stringify(handles.map((h) => [handleKey(h), h.position]));
    if (signature === this.signature) return;
    this.signature = signature;
    this.group.traverse((o) => {
      const mesh = o as THREE.Mesh;
      mesh.geometry?.dispose();
      (mesh.material as THREE.Material | undefined)?.dispose();
    });
    this.group.clear();
    for (const handle of handles) this.group.add(buildHandle(handle));
  }

  /** Keep every handle the same size on screen, and light `lit` — hovered or dragged. */
  update(camera: THREE.Camera, height: number, lit: StepHandle | null) {
    for (const child of this.group.children) {
      child.scale.setScalar(worldPerPixel(camera, child.position, height) * RADIUS_PX);
      const on = lit != null && handleKey(child.userData.handle as StepHandle) === handleKey(lit);
      child.traverse((o) => ((o as THREE.Mesh).material as THREE.MeshBasicMaterial | undefined)?.color?.copy(on ? LIT : COLOR));
    }
  }

  /** The handle `raycaster` hits, nearest first. */
  pick(raycaster: THREE.Raycaster): StepHandle | null {
    for (const hit of raycaster.intersectObject(this.group, true)) {
      let o: THREE.Object3D | null = hit.object;
      while (o && !o.userData.handle) o = o.parent;
      if (o) return o.userData.handle as StepHandle;
    }
    return null;
  }
}

/** The parameter `t` of the point on the line `p + t d` nearest to `ray`. */
function lineParameter(p: THREE.Vector3, d: THREE.Vector3, ray: THREE.Ray): number | null {
  const w0 = p.clone().sub(ray.origin);
  const b = d.dot(ray.direction);
  const c = ray.direction.dot(ray.direction);
  const denom = d.dot(d) * c - b * b;
  // Looking straight along the line: no point of it is nearer than another.
  if (Math.abs(denom) < 1e-9) return null;
  return (b * ray.direction.dot(w0) - c * d.dot(w0)) / denom;
}

/** Where `ray` meets the plane through `p` spanned by `u` and `v`. */
function planeHit(p: THREE.Vector3, u: THREE.Vector3, v: THREE.Vector3, ray: THREE.Ray): THREE.Vector3 | null {
  const n = u.clone().cross(v);
  const denom = ray.direction.dot(n);
  if (Math.abs(denom) < 1e-9) return null;
  const s = p.clone().sub(ray.origin).dot(n) / denom;
  return s < 0 ? null : ray.origin.clone().addScaledVector(ray.direction, s);
}

/** A drag in progress: the handle as it was grabbed, and where on its track it was grabbed. */
export interface HandleDrag {
  handle: StepHandle;
  grab: number | THREE.Vector3;
}

/** Start dragging `handle`, grabbed along `ray`. */
export function startDrag(handle: StepHandle, ray: THREE.Ray): HandleDrag | null {
  const p = vec(handle.position);
  const grab =
    handle.motion === "linear"
      ? lineParameter(p, vec(handle.direction), ray)
      : planeHit(p, vec(handle.u), vec(handle.v), ray);
  return grab == null ? null : { handle, grab };
}

/** The values `drag`'s handle writes with the pointer now along `ray` — none if its track is out of reach. */
export function dragEdits(drag: HandleDrag, ray: THREE.Ray): HandleEdit[] {
  const h = drag.handle;
  const p = vec(h.position);
  if (h.motion === "linear") {
    const t = lineParameter(p, vec(h.direction), ray);
    if (t == null) return [];
    return [{ path: h.arg, value: snap(h.value + (t - (drag.grab as number)) / h.scale) }];
  }
  const hit = planeHit(p, vec(h.u), vec(h.v), ray);
  if (!hit) return [];
  const moved = hit.sub(drag.grab as THREE.Vector3);
  return [
    { path: h.x, value: snap(h.value[0] + moved.dot(vec(h.u))) },
    { path: h.y, value: snap(h.value[1] + moved.dot(vec(h.v))) },
  ];
}
