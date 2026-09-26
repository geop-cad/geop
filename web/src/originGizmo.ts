// The origin gizmo: the world origin as something to click on. A ball at
// the origin (the origin point), a pointer along each axis (a direction),
// and a small square in each base plane (the plane, named by its normal).
//
// It is drawn on top of the model and at a constant size on screen, so it
// can be clicked whatever the view. What a click on it selects is the
// [[EntityRef]] a step refers to that part by — the origin, a world axis, a
// base plane — and which kinds a form argument accepts is up to the
// argument (see `acceptedDatums` in `operationArgs.ts`).

import * as THREE from "three";
import { worldPerPixel } from "./camera";
import { baseDatumKind, sameEntity, type DatumKind, type EntityRef, type Highlight, type WorldAxis } from "./geop";

const WHITE = new THREE.Color(0xffffff);

/** How tall the gizmo's axes are on screen, in pixels. */
const SIZE_PX = 90;

const AXES: { name: WorldAxis; dir: THREE.Vector3; color: number }[] = [
  { name: "X", dir: new THREE.Vector3(1, 0, 0), color: 0xff5555 },
  { name: "Y", dir: new THREE.Vector3(0, 1, 0), color: 0x55dd55 },
  { name: "Z", dir: new THREE.Vector3(0, 0, 1), color: 0x5599ff },
];

/** A material drawn over the model, so the gizmo is never hidden. */
function overlay(color: number, opacity: number): THREE.MeshBasicMaterial {
  const material = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity,
    depthTest: false,
    depthWrite: false,
    side: THREE.DoubleSide,
  });
  material.userData.opacity = opacity;
  material.userData.color = new THREE.Color(color);
  return material;
}

/** An invisible, generously sized stand-in for a thin part, so it is easy to hit. */
function hitArea(geometry: THREE.BufferGeometry, entity: EntityRef): THREE.Mesh {
  // Double-sided: a plane's square is hit from either side.
  const mesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide }));
  mesh.userData.entity = entity;
  return mesh;
}

/** A part of the gizmo: what is drawn (and its hit areas), and what a click on it selects. */
function part(children: THREE.Object3D[], entity: EntityRef): THREE.Group {
  const group = new THREE.Group();
  group.add(...children);
  group.userData.entity = entity;
  return group;
}

/** Rotate `object`, built along `+y`, to point along `dir`. */
function alongAxis(object: THREE.Object3D, dir: THREE.Vector3) {
  object.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
}

/** The gizmo at unit size (axes of length 1); [[updateGizmo]] scales it to the view. */
export function buildGizmo(): THREE.Group {
  const gizmo = new THREE.Group();
  gizmo.renderOrder = 1000;

  const point: EntityRef = { type: "Origin" };
  const ball = new THREE.Mesh(new THREE.SphereGeometry(0.07, 16, 12), overlay(0xffffff, 1));
  gizmo.add(part([ball, hitArea(new THREE.SphereGeometry(0.12, 8, 6), point)], point));

  for (const { name, dir, color } of AXES) {
    const direction: EntityRef = { type: "Axis", axis: name };
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.85, 8), overlay(color, 1));
    shaft.position.copy(dir).multiplyScalar(0.5);
    alongAxis(shaft, dir);
    const tip = new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.15, 12), overlay(color, 1));
    tip.position.copy(dir).multiplyScalar(0.925);
    alongAxis(tip, dir);
    const hit = hitArea(new THREE.CylinderGeometry(0.06, 0.06, 0.9, 6), direction);
    hit.position.copy(dir).multiplyScalar(0.55);
    alongAxis(hit, dir);
    gizmo.add(part([shaft, tip, hit], direction));

    // The plane normal to this axis: a square in the positive quadrant of
    // the other two, off the origin so it does not cover the ball.
    const plane: EntityRef = { type: "Plane", normal: name };
    const square = new THREE.PlaneGeometry(0.3, 0.3);
    const fill = new THREE.Mesh(square, overlay(color, 0.3));
    const edgeMaterial = new THREE.LineBasicMaterial({ color, depthTest: false, transparent: true });
    edgeMaterial.userData.opacity = 1;
    edgeMaterial.userData.color = new THREE.Color(color);
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(square), edgeMaterial);
    const hitSquare = hitArea(square, plane);
    const squareGroup = new THREE.Group();
    squareGroup.add(fill, edges, hitSquare);
    // `PlaneGeometry` lies in xy, facing +z: turn +z onto the normal, and
    // move the square's center to (0.45, 0.45) in the plane's own axes.
    squareGroup.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir);
    const [a, b] = AXES.filter((other) => other.name !== name).map((other) => other.dir);
    squareGroup.position.copy(a).add(b).multiplyScalar(0.45);
    gizmo.add(part([squareGroup], plane));
  }

  gizmo.traverse((o) => (o.renderOrder = 1000));
  return gizmo;
}

/**
 * Keep the gizmo at a constant size on screen for `camera` in a viewport
 * `height` pixels tall, and show which parts can be picked: those of
 * `pickable` kinds stand out, the rest fade, and those in `lit` — what a
 * click would pick, what is picked already — light up.
 */
export function updateGizmo(
  gizmo: THREE.Group,
  camera: THREE.Camera,
  height: number,
  pickable: DatumKind[],
  lit: Highlight[],
) {
  gizmo.scale.setScalar(worldPerPixel(camera, new THREE.Vector3(), height) * SIZE_PX);
  for (const child of gizmo.children) {
    const entity = child.userData.entity as EntityRef;
    const kind = baseDatumKind(entity)!;
    const isLit = lit.some((l) => sameEntity(l, entity));
    // Nothing being picked: everything as drawn. Otherwise what can be
    // picked stands out and the rest fades.
    const factor = isLit ? 3 : pickable.length === 0 ? 1 : pickable.includes(kind) ? 1.8 : 0.25;
    child.traverse((o) => {
      const material = (o as THREE.Mesh).material as THREE.MeshBasicMaterial | undefined;
      if (material?.userData.opacity == null) return;
      material.opacity = Math.min(1, material.userData.opacity * factor);
      material.color.copy(material.userData.color).lerp(WHITE, isLit ? 0.55 : 0);
    });
  }
}

/** The part of a `pickable` kind that `raycaster` hits on the gizmo, nearest first. */
export function pickGizmo(
  gizmo: THREE.Group,
  raycaster: THREE.Raycaster,
  pickable: DatumKind[],
): { entity: EntityRef; point: [number, number, number] } | null {
  const hits = raycaster.intersectObject(gizmo, true);
  for (const hit of hits) {
    const entity = hit.object.userData.entity as EntityRef | undefined;
    const kind = entity && baseDatumKind(entity);
    if (entity && kind && pickable.includes(kind)) {
      // A click selects the entity itself, not a spot on its handle: mark
      // it where the entity is.
      const at =
        entity.type === "Origin"
          ? new THREE.Vector3()
          : entity.type === "Axis"
            ? AXES.find((a) => a.name === entity.axis)!.dir.clone().multiplyScalar(gizmo.scale.x)
            : hit.point;
      return { entity, point: [at.x, at.y, at.z] };
    }
  }
  return null;
}
