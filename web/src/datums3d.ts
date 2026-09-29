// Datums in the 3-D view: the reference points, axes, planes and
// coordinate systems a part is built with (see
// `geop_core_math::primitives::Datum`), drawn so they can be seen and
// picked — a plane as a translucent square, an axis as a long dashed line,
// a point as its own small frame of three axes, a coordinate system as those
// axes with its xy plane marked between them.
//
// Picking one is the kernel's (see `geop_ops::ui::PartView`), which lays
// datums out the same way: a plane as a square the drawing's size (see
// `Extent`) around the point of it nearest the drawing's center, an axis as
// a line that long. Unlike the origin gizmo it is part of the scene, not
// drawn over it: a plane in front of the model hides it.

import * as THREE from "three";
import { worldPerPixel } from "./camera";
import { sameEntity, type DatumInfo, type DatumKind, type EntityRef, type Vec3 } from "./geop";

const COLOR = new THREE.Color(0xb58cff);
const LIT = new THREE.Color(0xffc94a);
/** A point or frame datum's axes, tinted towards x red, y green, z blue so its frame reads at a glance. */
const AXIS_TINTS = [0xff8080, 0x80e080, 0x80b0ff].map((c) => new THREE.Color(c));

/** How long a point or frame datum's axes are on screen, in pixels. */
const TRIAD_PX = 32;
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
  const { u, v, normal } = datum.frame;
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(vec(u), vec(v), vec(normal)));
}

/**
 * One datum, `size` across if it is a plane or an axis, at unit size if it
 * is a point or a frame. A plane or an axis is endless, and drawn around the point of
 * it nearest `center` — the model's — rather than around its frame's origin,
 * which can be anywhere on it.
 */
function buildDatum(datum: DatumInfo, size: number, center: THREE.Vector3): THREE.Group {
  const group = new THREE.Group();
  const origin = vec(datum.frame.origin);
  const w = vec(datum.frame.normal);
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
    case "point":
    case "frame": {
      const triad = new THREE.Group();
      triad.userData.triad = true;
      triad.add(new THREE.Mesh(new THREE.SphereGeometry(0.12, 12, 8), remembered(new THREE.MeshBasicMaterial(), COLOR, 1)));
      const ends = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)];
      ends.forEach((end, k) => {
        const geometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), end]);
        triad.add(new THREE.Line(geometry, remembered(new THREE.LineBasicMaterial(), AXIS_TINTS[k], 1)));
      });
      if (datum.kind === "frame") {
        // The corner of its xy plane between the x and y axes: a whole
        // coordinate system, not just a point that happens to have one.
        const square = new THREE.PlaneGeometry(0.5, 0.5).translate(0.25, 0.25, 0);
        const fill = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, depthWrite: false });
        triad.add(new THREE.Mesh(square, remembered(fill, COLOR, 0.35)));
      }
      // A point often lies inside the solid — a hole's center, say — so
      // it is drawn over the model, like the origin gizmo.
      triad.traverse((o) => {
        const material = (o as THREE.Mesh).material as THREE.Material | undefined;
        if (material) material.depthTest = false;
        o.renderOrder = 998;
      });
      group.add(triad);
      break;
    }
  }
  group.userData.entity = { type: "Datum", name: datum.name } satisfies EntityRef;
  group.userData.kind = datum.kind;
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
   * Keep points at a constant size on screen for `camera` in a viewport
   * `height` pixels tall, and show which datums can be picked: those of
   * `pickable` kinds stand out, the rest fade, and those in `lit` light up.
   */
  update(camera: THREE.Camera, height: number, pickable: DatumKind[], lit: EntityRef[]) {
    for (const datum of this.group.children) {
      const triad = datum.children.find((c) => c.userData.triad);
      triad?.scale.setScalar(worldPerPixel(camera, datum.position, height) * TRIAD_PX);
      const isLit = lit.some((l) => sameEntity(l, datum.userData.entity as EntityRef));
      const factor = isLit ? 2.5 : pickable.length === 0 || pickable.includes(datum.userData.kind as DatumKind) ? 1 : 0.3;
      datum.traverse((o) => {
        const material = (o as THREE.Mesh).material as (THREE.Material & { color: THREE.Color }) | undefined;
        if (material?.userData.color == null) return;
        material.opacity = Math.min(1, material.userData.opacity * factor);
        material.color.copy(isLit ? LIT : material.userData.color);
      });
    }
  }
}
