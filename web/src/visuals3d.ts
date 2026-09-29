// What an operation shows in the 3-D view while one of its steps is edited
// (see `geop_ops::ui::Visual`): points, curves, filled areas, labels and
// handles, drawn over the model. Nothing here knows any operation, and
// nothing here decides what a click hits — the kernel hit-tests the same
// visuals, by the same on-screen sizes this draws them at.

import * as THREE from "three";
import { CSS2DObject } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import { worldPerPixel } from "./camera";
import type { Style, Visual } from "./geop";

/** How each style is drawn. */
const COLORS: Record<Style, number> = {
  free: 0x6cb4ff,
  fixed: 0xe6e6e6,
  selected: 0xffd84a,
  hover: 0xffe0a0,
  failed: 0xff5d5d,
  construction: 0x8a8a8a,
  draft: 0xffa040,
  region: 0x6cb4ff,
  guide: 0x777777,
  handle: 0xffa040,
};

/** Drawn dashed: what is not part of the result. */
const DASHED: Style[] = ["construction", "draft", "guide"];

/** How big a point is on screen, in pixels. */
const POINT_PX = 7;
/** How big a handle's ball is on screen, in pixels — `geop_ops::ui::hit::HANDLE_PX`. */
const HANDLE_PX = 7;

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
      const geometry = new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(visual.at, 3));
      const size = visual.style === "selected" || visual.style === "hover" ? POINT_PX + 3 : POINT_PX;
      return new THREE.Points(geometry, overlay(new THREE.PointsMaterial({ color, size, sizeAttenuation: false })));
    }
    case "polyline": {
      const geometry = new THREE.BufferGeometry().setFromPoints(visual.points.map(vec));
      if (!DASHED.includes(visual.style)) return new THREE.Line(geometry, overlay(new THREE.LineBasicMaterial({ color })));
      const line = new THREE.Line(geometry, overlay(new THREE.LineDashedMaterial({ color })));
      line.computeLineDistances();
      line.userData.dashed = true;
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
      // The element is placed by the renderer; its child is moved on
      // screen by the offset, which the renderer's transform leaves alone.
      const outer = document.createElement("div");
      const inner = document.createElement("div");
      inner.className = `visual-label ${visual.style}`;
      inner.textContent = visual.text;
      inner.style.transform = `translate(${visual.offset[0]}px, ${-visual.offset[1]}px)`;
      outer.appendChild(inner);
      const label = new CSS2DObject(outer);
      label.position.set(...visual.at);
      return label;
    }
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
   * Keep handles and dashes the same size on screen for `camera` in a
   * viewport `height` pixels tall; dashes are measured at `target`, where
   * the camera looks.
   */
  update(camera: THREE.Camera, height: number, target: THREE.Vector3) {
    const px = worldPerPixel(camera, target, height);
    for (const child of this.group.children) {
      if (child.userData.handle) child.scale.setScalar(worldPerPixel(camera, child.position, height) * HANDLE_PX);
      if (child.userData.dashed) {
        const material = (child as THREE.Line).material as THREE.LineDashedMaterial;
        material.dashSize = 6 * px;
        material.gapSize = 4 * px;
      }
    }
  }
}
