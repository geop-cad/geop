// The gizmo a step offers to move, turn or scale by (see
// `geop_ops::ui::gizmo`): a ball, arrows, squares, rings and cubes, drawn
// over everything at a constant size on screen. Nothing here decides what
// the pointer is over or what a drag does — the kernel hit-tests the same
// parts, laid out by the same sizes in reaches, and says which one is
// hovered or dragged.

import * as THREE from "three";
import { CSS2DObject } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import { REACH_PX, worldPerPixel } from "./camera";
import type { GizmoPart, GizmoView } from "./geop";

/** `geop_ops::ui::gizmo::size`, in reaches. */
const SIZE = {
  free: 1.4,
  arrow: [2.5, 10] as const,
  plane: [2.5, 4] as const,
  ring: 7,
  cubeAt: 12.5,
  cube: 0.9,
  /** How far from looking along an arrow it still shows, as a sine. */
  facing: 0.2,
};

const AXIS_COLORS = [0xe0584f, 0x5cc25c, 0x4f86f0];
const PLAIN = 0xdddddd;
const LIT = 0xffd84a;

const same = (a: GizmoPart | null, b: GizmoPart) =>
  a != null && a.part === b.part && ("axis" in a ? a.axis : -1) === ("axis" in b ? b.axis : -1);

const vec = (v: [number, number, number]) => new THREE.Vector3(...v);

/** Drawn over the model, as the step's other visuals are. */
function material(color: number, opacity = 0.9) {
  return new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity,
    depthTest: false,
    depthWrite: false,
    side: THREE.DoubleSide,
  });
}

/** Turns `object`, built along `y`, to point along local axis `i`. */
function along(object: THREE.Object3D, i: number) {
  const to = new THREE.Vector3().setComponent(i, 1);
  object.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), to);
  return object;
}

/** The meshes of one part, in the gizmo's own axes, a reach long a unit. */
function build(part: GizmoPart): THREE.Object3D {
  const group = new THREE.Group();
  const color = "axis" in part ? AXIS_COLORS[part.axis] : PLAIN;
  switch (part.part) {
    case "free":
      group.add(new THREE.Mesh(new THREE.SphereGeometry(SIZE.free, 20, 14), material(color, 0.55)));
      break;
    case "move": {
      const [from, tip] = SIZE.arrow;
      const head = 2;
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, tip - head - from, 8), material(color));
      shaft.position.y = (from + tip - head) / 2;
      const cone = new THREE.Mesh(new THREE.ConeGeometry(0.65, head, 16), material(color));
      cone.position.y = tip - head / 2;
      const arrow = new THREE.Group().add(shaft, cone);
      group.add(along(arrow, part.axis));
      break;
    }
    case "plane": {
      const [lo, hi] = SIZE.plane;
      const [j, k] = [(part.axis + 1) % 3, (part.axis + 2) % 3];
      const square = new THREE.Mesh(new THREE.PlaneGeometry(hi - lo, hi - lo), material(color, 0.45));
      // A plane's square lies along the two other axes.
      const centre = new THREE.Vector3().setComponent(j, (lo + hi) / 2).setComponent(k, (lo + hi) / 2);
      square.position.copy(centre);
      square.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3().setComponent(part.axis, 1));
      group.add(square);
      break;
    }
    case "turn": {
      const ring = new THREE.Mesh(new THREE.TorusGeometry(SIZE.ring, 0.16, 8, 96), material(color));
      ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3().setComponent(part.axis, 1));
      group.add(ring);
      break;
    }
    case "stretch":
    case "scale": {
      const box = new THREE.Mesh(new THREE.BoxGeometry(2 * SIZE.cube, 2 * SIZE.cube, 2 * SIZE.cube), material(color));
      const at =
        part.part === "stretch"
          ? new THREE.Vector3().setComponent(part.axis, SIZE.cubeAt)
          : new THREE.Vector3(1, 1, 1).normalize().multiplyScalar(SIZE.cubeAt);
      box.position.copy(at);
      group.add(box);
      break;
    }
  }
  group.userData.part = part;
  return group;
}

/** The parts a gizmo of `modes` shows, as the kernel lists them. */
function parts(modes: GizmoView["modes"]): GizmoPart[] {
  const out: GizmoPart[] = [];
  const axes = [0, 1, 2];
  if (modes.translate) {
    out.push({ part: "free" });
    for (const axis of axes) out.push({ part: "move", axis });
    for (const axis of axes) out.push({ part: "plane", axis });
  }
  if (modes.rotate) for (const axis of axes) out.push({ part: "turn", axis });
  if (modes.scale) {
    for (const axis of axes) out.push({ part: "stretch", axis });
    out.push({ part: "scale" });
  }
  return out;
}

/** The gizmo on screen: rebuilt when it changes, sized to the screen every frame. */
export class GizmoLayer {
  readonly group = new THREE.Group();
  private view: GizmoView | null = null;
  private signature = "";
  private readout: CSS2DObject;
  private readoutEl: HTMLDivElement;

  constructor() {
    this.readoutEl = document.createElement("div");
    this.readoutEl.className = "gizmo-readout";
    this.readout = new CSS2DObject(this.readoutEl);
  }

  sync(view: GizmoView | null) {
    const signature = JSON.stringify(view && { at: view.at, axes: view.axes, modes: view.modes });
    this.view = view;
    if (signature !== this.signature) {
      this.signature = signature;
      this.group.traverse((o) => {
        const mesh = o as THREE.Mesh;
        mesh.geometry?.dispose();
        (mesh.material as THREE.Material | undefined)?.dispose();
      });
      this.group.clear();
      if (view) {
        const frame = new THREE.Group();
        const [u, v, w] = view.axes.map(vec);
        frame.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(u, v, w));
        for (const part of parts(view.modes)) frame.add(build(part));
        frame.traverse((o) => (o.renderOrder = 1002));
        this.group.add(frame);
        this.group.add(this.readout);
        this.group.position.set(...view.at);
      }
    }
    // Lit where the pointer is over it, or what is dragged; the rest faded
    // while a drag is under way.
    const frame = this.group.children[0];
    if (!view || !frame) return;
    for (const child of frame.children) {
      const part = child.userData.part as GizmoPart;
      const lit = same(view.active, part) || (view.active == null && same(view.hover, part));
      const faded = view.active != null && !lit;
      const color = lit ? LIT : "axis" in part ? AXIS_COLORS[part.axis] : PLAIN;
      child.traverse((o) => {
        const m = (o as THREE.Mesh).material as THREE.MeshBasicMaterial | undefined;
        if (!m) return;
        m.color.setHex(color);
        m.userData.base ??= m.opacity;
        m.opacity = faded ? 0.15 : (m.userData.base as number);
      });
    }
    this.readoutEl.textContent = view.readout ?? "";
    this.readoutEl.style.display = view.readout ? "" : "none";
  }

  /** Keep it the same size on screen for `camera` in a viewport `height` pixels tall, hiding what is seen end on. */
  update(camera: THREE.Camera, height: number) {
    const view = this.view;
    const frame = this.group.children[0];
    if (!view || !frame) return;
    const at = vec(view.at);
    const reach = worldPerPixel(camera, at, height) * REACH_PX;
    frame.scale.setScalar(reach);
    this.readout.position.set(0, -reach * 3, 0);
    // Where the eye looks along, at the gizmo: what the kernel tests with
    // the pointer's ray (see `GizmoView::shown`).
    const dir =
      camera instanceof THREE.OrthographicCamera
        ? camera.getWorldDirection(new THREE.Vector3())
        : at.clone().sub(camera.position).normalize();
    const axes = view.axes.map(vec);
    const across = (a: THREE.Vector3) => a.clone().cross(dir).length() >= SIZE.facing;
    for (const child of frame.children) {
      const part = child.userData.part as GizmoPart;
      switch (part.part) {
        case "move":
        case "stretch":
          child.visible = across(axes[part.axis]);
          break;
        case "plane":
          child.visible = Math.abs(axes[part.axis].dot(dir)) >= SIZE.facing;
          break;
        case "scale":
          child.visible = across(axes[0].clone().add(axes[1]).add(axes[2]).normalize());
          break;
        default:
          child.visible = true;
      }
    }
  }
}
