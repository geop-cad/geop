// Datums in the 3-D view: the reference points, axes, planes and
// coordinate systems a part is built with (see
// `geop_core_math::primitives::Datum`), drawn so they can be seen and
// picked — a plane as a translucent square, an axis as a long dashed line,
// a point as its own small frame of three axes, and a coordinate system —
// like the `origin` every part has — as a ball, an arrow along each axis
// and a square in each of its planes, each of which can be picked on its
// own.
//
// Picking one is the kernel's (see `geop_ops::ui::PartView`), which lays
// datums out the same way: a plane as a square the drawing's size (see
// `Extent`) around the point of it nearest the drawing's center, an axis as
// a line that long, a coordinate system at `FRAME` reaches. Planes and axes
// are part of the scene — a plane in front of the model hides it — while
// points and coordinate systems are drawn over it, since they often lie
// inside it.

import * as THREE from "three";
import { REACH_PX, worldPerPixel } from "./camera";
import {
  sameEntity,
  type DatumInfo,
  type DatumKind,
  type EntityRef,
  type FrameAxis,
  type Vec3,
} from "./geop";

const COLOR = new THREE.Color(0xb58cff);
const LIT = new THREE.Color(0xffc94a);
/** A point or frame datum's axes, tinted towards x red, y green, z blue so its frame reads at a glance. */
const AXIS_TINTS = [0xff8080, 0x80e080, 0x80b0ff].map((c) => new THREE.Color(c));

/** How long a point datum's axes are on screen, in pixels. */
const TRIAD_PX = 32;
/** How long a coordinate system's axes are on screen, in pixels — `geop_ops::ui::view::FRAME` reaches. */
const FRAME_PX = 10 * REACH_PX;

/** A coordinate system's axes, as its own `u`, `v`, `w` — at unit size, turned by the datum's basis. */
const FRAME_AXES: { name: FrameAxis; dir: THREE.Vector3; color: number }[] = [
  { name: "x", dir: new THREE.Vector3(1, 0, 0), color: 0xff5555 },
  { name: "y", dir: new THREE.Vector3(0, 1, 0), color: 0x55dd55 },
  { name: "z", dir: new THREE.Vector3(0, 0, 1), color: 0x5599ff },
];
const vec = (v: Vec3) => new THREE.Vector3(...v);

/** A material that remembers how it is drawn when nothing is lit or being picked. */
function remembered<M extends THREE.Material & { color: THREE.Color }>(material: M, color: THREE.Color, opacity: number): M {
  material.transparent = true;
  material.opacity = opacity;
  material.color.copy(color);
  material.userData = { color: color.clone(), opacity };
  return material;
}

/** The rotation that turns x, y, z onto the datum's u, v, w. */
function basis(datum: DatumInfo): THREE.Quaternion {
  const { u, v, w } = datum.frame;
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(vec(u), vec(v), vec(w)));
}

/** A part of a datum that is picked on its own: what is drawn, what a click on it selects, and as which kind. */
function pickablePart(children: THREE.Object3D[], entity: EntityRef, kinds: DatumKind[]): THREE.Group {
  const group = new THREE.Group();
  group.add(...children);
  group.userData.entity = entity;
  group.userData.kinds = kinds;
  return group;
}

/**
 * A coordinate system at unit size (axes of length 1), in its own basis:
 * its ball — the whole frame — an arrow along each axis, and a square in
 * each plane, in the positive quadrant of the other two axes and off the
 * origin so it does not cover the ball, as the kernel picks them.
 */
function buildFrame(name: string): THREE.Group {
  const frame = new THREE.Group();
  const overlay = (color: number, opacity: number) =>
    remembered(new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, depthWrite: false }), new THREE.Color(color), opacity);
  const alongAxis = (object: THREE.Object3D, dir: THREE.Vector3) =>
    object.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);

  const ball = new THREE.Mesh(new THREE.SphereGeometry(0.07, 16, 12), overlay(0xffffff, 1));
  frame.add(pickablePart([ball], { type: "Datum", name }, ["point", "frame"]));
  for (const { name: axis, dir, color } of FRAME_AXES) {
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.85, 8), overlay(color, 1));
    shaft.position.copy(dir).multiplyScalar(0.5);
    alongAxis(shaft, dir);
    const tip = new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.15, 12), overlay(color, 1));
    tip.position.copy(dir).multiplyScalar(0.925);
    alongAxis(tip, dir);
    frame.add(pickablePart([shaft, tip], { type: "Datum", name, component: { axis } }, ["axis"]));

    const square = new THREE.PlaneGeometry(0.3, 0.3);
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(square),
      remembered(new THREE.LineBasicMaterial(), new THREE.Color(color), 1),
    );
    const plane = new THREE.Group();
    plane.add(new THREE.Mesh(square, overlay(color, 0.3)), edges);
    // `PlaneGeometry` lies in xy, facing +z: turn +z onto the normal, and
    // move the square's center to (0.45, 0.45) in the plane's own axes.
    plane.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir);
    const [a, b] = FRAME_AXES.filter((other) => other.name !== axis).map((other) => other.dir);
    plane.position.copy(a).add(b).multiplyScalar(0.45);
    frame.add(pickablePart([plane], { type: "Datum", name, component: { plane: axis } }, ["plane"]));
  }
  frame.userData.screenPx = FRAME_PX;
  return frame;
}

/**
 * One datum, `size` across if it is a plane or an axis, at unit size if it
 * is a point or a frame. A plane or an axis is endless, and drawn around the
 * point of it nearest `center` — the model's — rather than around its
 * frame's origin, which can be anywhere on it.
 */
function buildDatum(datum: DatumInfo, size: number, center: THREE.Vector3): THREE.Group {
  const group = new THREE.Group();
  const origin = vec(datum.frame.origin);
  const w = vec(datum.frame.w);
  const offset = center.clone().sub(origin);
  if (datum.kind === "plane") origin.add(offset.sub(w.clone().multiplyScalar(offset.dot(w))));
  if (datum.kind === "axis") origin.add(w.clone().multiplyScalar(offset.dot(w)));
  group.position.copy(origin);
  group.quaternion.copy(basis(datum));
  switch (datum.kind) {
    case "plane": {
      const square = new THREE.PlaneGeometry(size, size);
      const fill = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, depthWrite: false });
      group.add(new THREE.Mesh(square, remembered(fill, COLOR, 0.12)));
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(square), remembered(new THREE.LineBasicMaterial(), COLOR, 0.8));
      group.add(edges);
      break;
    }
    case "axis": {
      const geometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, -size),
        new THREE.Vector3(0, 0, size),
      ]);
      const line = new THREE.Line(geometry, remembered(new THREE.LineDashedMaterial({ dashSize: size / 30, gapSize: size / 60 }), COLOR, 0.9));
      line.computeLineDistances();
      group.add(line);
      break;
    }
    case "frame":
      group.add(buildFrame(datum.name));
      break;
    case "point": {
      const triad = new THREE.Group();
      triad.userData.screenPx = TRIAD_PX;
      triad.add(new THREE.Mesh(new THREE.SphereGeometry(0.12, 12, 8), remembered(new THREE.MeshBasicMaterial(), COLOR, 1)));
      const ends = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)];
      ends.forEach((end, k) => {
        const geometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), end]);
        triad.add(new THREE.Line(geometry, remembered(new THREE.LineBasicMaterial(), AXIS_TINTS[k], 1)));
      });
      group.add(triad);
      break;
    }
  }
  if (datum.kind === "point" || datum.kind === "frame") {
    // A point often lies inside the solid — a hole's center, say — so it
    // is drawn over the model, and a frame with it.
    group.traverse((o) => {
      const material = (o as THREE.Mesh).material as THREE.Material | undefined;
      if (material) material.depthTest = false;
      o.renderOrder = 998;
    });
  }
  if (datum.kind !== "frame") {
    group.userData.entity = { type: "Datum", name: datum.name } satisfies EntityRef;
    group.userData.kinds = [datum.kind];
  }
  return group;
}

/** The datums on screen: rebuilt when they change, rescaled and lit every frame. */
export class DatumLayer {
  readonly group = new THREE.Group();
  private signature = "";

  /** Show `datums` but those in `hidden`, planes and axes `size` across around `center`. */
  sync(datums: DatumInfo[], hidden: string[], size: number, center: THREE.Vector3) {
    const shown = datums.filter((d) => !hidden.includes(d.name));
    const signature = JSON.stringify([shown, size, center.toArray()]);
    if (signature === this.signature) return;
    this.signature = signature;
    this.group.traverse((o) => {
      const mesh = o as THREE.Mesh;
      mesh.geometry?.dispose();
      (mesh.material as THREE.Material | undefined)?.dispose();
    });
    this.group.clear();
    for (const datum of shown) this.group.add(buildDatum(datum, size, center));
  }

  /**
   * Keep points and coordinate systems at a constant size on screen for
   * `camera` in a viewport `height` pixels tall, and show what can be
   * picked: parts of `pickable` kinds stand out, the rest fade, and those
   * in `lit` light up.
   */
  update(camera: THREE.Camera, height: number, pickable: DatumKind[], lit: EntityRef[]) {
    for (const datum of this.group.children) {
      for (const sized of datum.children.filter((c) => c.userData.screenPx)) {
        sized.scale.setScalar(worldPerPixel(camera, datum.position, height) * (sized.userData.screenPx as number));
      }
    }
    const parts: THREE.Object3D[] = [];
    this.group.traverse((o) => {
      if (o.userData.entity) parts.push(o);
    });
    for (const part of parts) {
      const isLit = lit.some((l) => sameEntity(l, part.userData.entity as EntityRef));
      const kinds = part.userData.kinds as DatumKind[];
      const factor = isLit ? 2.5 : pickable.length === 0 || kinds.some((k) => pickable.includes(k)) ? 1 : 0.3;
      part.traverse((o) => {
        const material = (o as THREE.Mesh).material as (THREE.Material & { color: THREE.Color }) | undefined;
        if (material?.userData.color == null) return;
        material.opacity = Math.min(1, material.userData.opacity * factor);
        material.color.copy(isLit ? LIT : material.userData.color);
      });
    }
  }
}
