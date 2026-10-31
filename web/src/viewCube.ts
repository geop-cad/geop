// The view cube: a cube in the viewport's corner, turned as the view is
// turned, its faces named for the standard views — Front (+z), Right (+x),
// Top (+y) and their opposites. A face, an edge or a corner of it, clicked,
// turns the view to look from there (see `camera.ts`: `cubeRegion`,
// `standardPose`); dragged, it turns the view about its target. It is drawn
// by the viewer's own renderer, into its corner (see `SceneViewer`).

import * as THREE from "three";
import { CUBE_BAND, cubeRegion } from "./camera";
import type { Vec3 } from "./geop";

/** The faces' names, in the order a box's faces are: +x, -x, +y, -y, +z, -z. */
const FACE_NAMES = ["Right", "Left", "Top", "Bottom", "Front", "Back"];

const FACE_COLOR = "#3a4250";
const TEXT_COLOR = "#dfe5ee";
const LIT = 0xffc94a;

/** A face's texture: its name, centred. */
function label(text: string): THREE.Texture {
  const size = 128;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size;
  const g = canvas.getContext("2d");
  if (g) {
    g.fillStyle = FACE_COLOR;
    g.fillRect(0, 0, size, size);
    g.fillStyle = TEXT_COLOR;
    g.font = `600 ${text.length > 5 ? 26 : 30}px system-ui, sans-serif`;
    g.textAlign = "center";
    g.textBaseline = "middle";
    g.fillText(text, size / 2, size / 2);
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

/** The view cube's scene, its camera, and the region of it the pointer is over. */
export class ViewCube {
  readonly scene = new THREE.Scene();
  readonly camera = new THREE.OrthographicCamera(-0.9, 0.9, 0.9, -0.9, 0.1, 10);
  private readonly cube: THREE.Mesh;
  private readonly lit: THREE.Mesh;
  private readonly raycaster = new THREE.Raycaster();

  constructor() {
    const materials = FACE_NAMES.map((name) => new THREE.MeshBasicMaterial({ map: label(name) }));
    this.cube = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), materials);
    this.scene.add(this.cube);
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(this.cube.geometry),
      new THREE.LineBasicMaterial({ color: 0x8a94a6 }),
    );
    this.scene.add(edges);
    // What the pointer is over: drawn over the cube where it is.
    this.lit = new THREE.Mesh(
      new THREE.BoxGeometry(1, 1, 1),
      new THREE.MeshBasicMaterial({ color: LIT, transparent: true, opacity: 0.55, depthTest: false }),
    );
    this.lit.renderOrder = 1;
    this.lit.visible = false;
    this.scene.add(this.lit);
    // The world's axes from the cube's back corner, coloured x, y, z.
    const axes = new THREE.AxesHelper(0.75);
    axes.position.set(-0.5, -0.5, -0.5);
    this.scene.add(axes);
  }

  /** Turned as a camera turned `quaternion` sees the world. */
  orient(quaternion: THREE.Quaternion) {
    this.camera.quaternion.copy(quaternion);
    this.camera.position.set(0, 0, 3).applyQuaternion(quaternion);
    this.camera.updateMatrixWorld();
  }

  /** The region of the cube (see [[cubeRegion]]) at `ndc`, in the cube's own view, if the cube is there. */
  regionAt(ndc: THREE.Vector2): Vec3 | null {
    this.raycaster.setFromCamera(ndc, this.camera);
    const hit = this.raycaster.intersectObject(this.cube, false)[0];
    return hit ? cubeRegion([hit.point.x, hit.point.y, hit.point.z]) : null;
  }

  /** Lights `region`, a face, an edge or a corner — or nothing. */
  light(region: Vec3 | null) {
    this.lit.visible = region != null;
    if (!region) return;
    // A slab over the region, standing out of the cube along each side
    // the region is towards, as wide as the region across the others.
    const band = CUBE_BAND;
    const size = region.map((k) => (k === 0 ? 1 - 2 * band : band + 0.02));
    this.lit.scale.set(size[0], size[1], size[2]);
    this.lit.position.set(...(region.map((k) => k * (0.5 - band / 2 + 0.01)) as Vec3));
  }

  dispose() {
    this.scene.traverse((o) => {
      const mesh = o as THREE.Mesh;
      mesh.geometry?.dispose();
      const materials = Array.isArray(mesh.material) ? mesh.material : mesh.material ? [mesh.material] : [];
      for (const m of materials) {
        (m as THREE.MeshBasicMaterial).map?.dispose();
        m.dispose();
      }
    });
  }
}
