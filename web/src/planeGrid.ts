// A grid on the plane an operation works in (see `Presentation.focus`):
// lines a power of ten apart, 20 to 200 pixels at the current zoom, every
// tenth one stronger, and the plane's own axes through its origin.

import * as THREE from "three";
import { worldPerPixel } from "./camera";
import type { Frame } from "./geop";

const MINOR = 0x2c2c2c;
const MAJOR = 0x3c3c3c;
const AXIS_U = 0x994444;
const AXIS_V = 0x449944;

const vec = (v: [number, number, number]) => new THREE.Vector3(...v);

/** The grid on screen: rebuilt when its plane, spacing or place in view changes. */
export class PlaneGrid {
  readonly group = new THREE.Group();
  private frame: Frame | null = null;
  private key = "";

  /** Draw on `frame` from now on — or nothing, for `null`. */
  sync(frame: Frame | null) {
    if (JSON.stringify(frame) === JSON.stringify(this.frame)) return;
    this.frame = frame;
    this.key = "";
    this.clear();
  }

  private clear() {
    this.group.traverse((o) => {
      const line = o as THREE.LineSegments;
      line.geometry?.dispose();
      (line.material as THREE.Material | undefined)?.dispose();
    });
    this.group.clear();
  }

  /** Cover what `camera`, looking at `target` in a viewport `height` pixels tall, sees of the plane. */
  update(camera: THREE.Camera, height: number, target: THREE.Vector3) {
    const frame = this.frame;
    if (!frame) return;
    const px = worldPerPixel(camera, target, height);
    const step = Math.pow(10, Math.ceil(Math.log10(20 * px)));
    const origin = vec(frame.origin);
    const [u, v] = [vec(frame.u), vec(frame.v)];
    const local = target.clone().sub(origin);
    const [cu, cv] = [Math.round(local.dot(u) / step), Math.round(local.dot(v) / step)];
    const n = Math.ceil((px * height * 1.2) / step) + 1;
    const key = `${step}:${cu}:${cv}:${n}`;
    if (key === this.key) return;
    this.key = key;
    this.clear();

    const at = (x: number, y: number) => origin.clone().addScaledVector(u, x).addScaledVector(v, y);
    const minor: THREE.Vector3[] = [];
    const major: THREE.Vector3[] = [];
    for (let i = -n; i <= n; i++) {
      const [x, y] = [(cu + i) * step, (cv + i) * step];
      const [lo, hi] = [-n, n];
      const into = (k: number) => (Math.abs(k % 10) === 0 ? major : minor);
      into(cu + i).push(at(x, (cv + lo) * step), at(x, (cv + hi) * step));
      into(cv + i).push(at((cu + lo) * step, y), at((cu + hi) * step, y));
    }
    const span = (n + Math.max(Math.abs(cu), Math.abs(cv))) * step;
    const segments = (points: THREE.Vector3[], color: number) => {
      const line = new THREE.LineSegments(
        new THREE.BufferGeometry().setFromPoints(points),
        new THREE.LineBasicMaterial({ color, depthWrite: false }),
      );
      line.renderOrder = -1;
      this.group.add(line);
    };
    segments(minor, MINOR);
    segments(major, MAJOR);
    segments([at(-span, 0), at(span, 0)], AXIS_U);
    segments([at(0, -span), at(0, span)], AXIS_V);
  }
}
