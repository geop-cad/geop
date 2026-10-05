// What an operation shows in the 3-D view while one of its steps is edited
// (see `geop_ops::ui::Visual`): points, curves, filled areas, labels and
// handles, drawn over the model. Nothing here knows any operation, and
// nothing here decides what a click hits — the kernel hit-tests the same
// visuals, by the same on-screen sizes this draws them at.

import * as THREE from "three";
import { CSS2DObject } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import { REACH_PX, worldPerPixel } from "./camera";
import type { Style, Visual } from "./geop";

/** How each style is drawn. */
const COLORS: Record<Style, number> = {
  free: 0x6cb4ff,
  fixed: 0xe6e6e6,
  selected: 0xffd84a,
  hover: 0xffe0a0,
  failed: 0xff5d5d,
  construction: 0x8a8a8a,
  reference: 0xb48cf0,
  draft: 0xffa040,
  region: 0x6cb4ff,
  guide: 0x777777,
  handle: 0xffa040,
  snap: 0x58e07a,
  removed: 0xff4f9a,
};

/** Drawn dashed: what is not part of the result. */
const DASHED: Style[] = ["construction", "draft", "guide", "reference"];

/** How big a point is on screen, in pixels. */
const POINT_PX = 7;
/** How big a handle's ball is on screen, in pixels — `geop_ops::ui::hit::HANDLE` reaches. */
const HANDLE_PX = 0.8 * REACH_PX;
/** How long a triad's arrows are, in reaches — `geop_ops::ui::TRIAD`. */
export const TRIAD_REACHES = 6;
/** A triad's arrows, coloured as the frame datums' axes are. */
const TRIAD_AXES: { dir: THREE.Vector3; color: number }[] = [
  { dir: new THREE.Vector3(1, 0, 0), color: 0xff5555 },
  { dir: new THREE.Vector3(0, 1, 0), color: 0x55dd55 },
  { dir: new THREE.Vector3(0, 0, 1), color: 0x5599ff },
];

/** Drawn over the model: what is being edited must be seen wherever it is. */
function overlay<M extends THREE.Material>(material: M): M {
  material.depthTest = false;
  material.depthWrite = false;
  material.transparent = true;
  return material;
}

const vec = (v: [number, number, number]) => new THREE.Vector3(...v);

/** One visual as three.js objects. */
function build(visual: Visual): THREE.Object3D {
  const color = COLORS[visual.style];
  switch (visual.shape) {
    case "point": {
      if (visual.style === "snap") {
        // A ring around where it snaps to: crisp, and over everything.
        const element = document.createElement("div");
        element.className = "snap-marker";
        const marker = new CSS2DObject(element);
        marker.position.set(...visual.at);
        return marker;
      }
      const geometry = new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(visual.at, 3));
      const size = visual.style === "selected" || visual.style === "hover" ? POINT_PX + 3 : POINT_PX;
      return new THREE.Points(geometry, overlay(new THREE.PointsMaterial({ color, size, sizeAttenuation: false })));
    }
    case "polyline": {
      const geometry = new THREE.BufferGeometry().setFromPoints(visual.points.map(vec));
      if (!DASHED.includes(visual.style)) return new THREE.Line(geometry, overlay(new THREE.LineBasicMaterial({ color })));
      const line = new THREE.Line(geometry, overlay(new THREE.LineDashedMaterial({ color })));
      line.computeLineDistances();
      // Reference geometry dotted, the rest dashed.
      line.userData.dashed = visual.style === "reference" ? "dotted" : "dashed";
      return line;
    }
    case "triangles": {
      const geometry = new THREE.BufferGeometry().setAttribute(
        "position",
        new THREE.Float32BufferAttribute(visual.triangles.flat(2), 3),
      );
      const material = overlay(new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide }));
      material.opacity = 0.15;
      return new THREE.Mesh(geometry, material);
    }
    case "label": {
      // Centred where the kernel hit-tests it: its point, moved by its
      // offset in reaches — placed every frame, as a reach's size changes.
      const element = document.createElement("div");
      element.className = `visual-label ${visual.style}`;
      element.textContent = visual.text;
      const label = new CSS2DObject(element);
      label.userData.label = { at: vec(visual.at), offset: vec(visual.offset) };
      label.position.set(...visual.at);
      return label;
    }
    case "instance":
      // Drawn by the scene already; the scene lights it (see SceneViewer).
      return new THREE.Group();
    case "handle": {
      const group = new THREE.Group();
      const material = () => overlay(new THREE.MeshBasicMaterial({ color }));
      group.add(new THREE.Mesh(new THREE.SphereGeometry(1, 16, 12), material()));
      if (visual.direction) {
        const dir = vec(visual.direction).normalize();
        for (const sign of [1, -1]) {
          const cone = new THREE.Mesh(new THREE.ConeGeometry(0.8, 1.6, 12), material());
          const d = dir.clone().multiplyScalar(sign);
          cone.position.copy(d).multiplyScalar(2.2);
          cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d);
          group.add(cone);
        }
      }
      group.position.set(...visual.at);
      group.userData.handle = true;
      return group;
    }
    case "triad": {
      // Unit arrows, scaled to the screen every frame: a shaft and a tip
      // along each axis, coloured as the origin's — no planes.
      const group = new THREE.Group();
      for (const { dir, color } of TRIAD_AXES) {
        const material = () => overlay(new THREE.MeshBasicMaterial({ color }));
        const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.8, 8), material());
        shaft.position.copy(dir).multiplyScalar(0.4);
        shaft.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
        const tip = new THREE.Mesh(new THREE.ConeGeometry(0.06, 0.2, 12), material());
        tip.position.copy(dir).multiplyScalar(0.9);
        tip.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
        group.add(shaft, tip);
      }
      group.position.set(...visual.at);
      group.userData.triad = true;
      return group;
    }
  }
}

/** The visuals on screen: rebuilt when they change, sized to the screen every frame. */
export class VisualLayer {
  readonly group = new THREE.Group();
  private signature = "";

  sync(visuals: Visual[]) {
    const signature = JSON.stringify(visuals);
    if (signature === this.signature) return;
    this.signature = signature;
    this.group.traverse((o) => {
      const mesh = o as THREE.Mesh;
      mesh.geometry?.dispose();
      (mesh.material as THREE.Material | undefined)?.dispose();
    });
    this.group.clear();
    for (const visual of visuals) {
      const object = build(visual);
      object.traverse((o) => (o.renderOrder = 1001));
      this.group.add(object);
    }
  }

  /**
   * Keep handles, label offsets and dashes the same size on screen for
   * `camera` in a viewport `height` pixels tall; dashes are measured at
   * `target`, where the camera looks.
   */
  update(camera: THREE.Camera, height: number, target: THREE.Vector3) {
    const px = worldPerPixel(camera, target, height);
    for (const child of this.group.children) {
      if (child.userData.handle) child.scale.setScalar(worldPerPixel(camera, child.position, height) * HANDLE_PX);
      if (child.userData.triad) {
        child.scale.setScalar(worldPerPixel(camera, child.position, height) * REACH_PX * TRIAD_REACHES);
      }
      const label = child.userData.label as { at: THREE.Vector3; offset: THREE.Vector3 } | undefined;
      if (label) {
        const reach = worldPerPixel(camera, label.at, height) * REACH_PX;
        child.position.copy(label.at).addScaledVector(label.offset, reach);
      }
      if (child.userData.dashed) {
        const material = (child as THREE.Line).material as THREE.LineDashedMaterial;
        const dotted = child.userData.dashed === "dotted";
        material.dashSize = (dotted ? 2 : 6) * px;
        material.gapSize = (dotted ? 3 : 4) * px;
      }
    }
  }
}
