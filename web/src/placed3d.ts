// The parts placed in the part drawn — however many, however deep — drawn
// from their components' views, each built once however often it is placed.
//
// A component's triangles are one `InstancedMesh` for every placed part
// drawn from it, and its edges and corners one merged buffer: a robot of
// two thousand screws is a few draw calls, not thousands. A placed part
// that is lit, or has something of it hidden, is drawn on its own instead,
// from buffers of its own, so that its faces can be tinted by name; the
// others stay in the batch.

import * as THREE from "three";
import type { EntityRef, PartView, ViewInstance } from "./geop";
import {
  applyHighlight,
  buildSceneGroup,
  disposeGroup,
  flatten,
  frameMatrix,
  stencilCounters,
  type Scene,
} from "./partScene";

/** How a placed part is to be drawn: what of it is lit, and hidden. */
export interface PlacedLook {
  /** Its entities to light, as it names them. */
  highlights: EntityRef[];
  /** Its sketches, solids and faces not to draw, as it names them. */
  hidden: string[];
  /** Whether it is drawn at all. */
  visible: boolean;
}

/** A component's view, built into buffers once, and its batch. */
interface Shared {
  scene: Scene;
  /** Its triangles: positions, normals and colors, shared by every instance in the batch. */
  faces: THREE.BufferGeometry | null;
  /** Its edges and corners, in its own frame: `[x0, y0, z0, x1, y1, z1]` per segment, and the colour of each. */
  segments: Float32Array;
  segmentColors: Float32Array;
  points: Float32Array;
  pointColors: Float32Array;
  /** The batch drawn now, and what it was built from: the names and frames of the parts in it. */
  batch: THREE.Group | null;
  built: string;
}

/** A placed part drawn on its own. */
interface Single {
  component: string;
  group: THREE.Group;
}

/** The faces' material of every batch: coloured by vertex, as a part drawn on its own is. */
const FACES = new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide, roughness: 0.6 });
const LINES = new THREE.LineBasicMaterial({ vertexColors: true });
const POINTS = new THREE.PointsMaterial({ size: 0.02, vertexColors: true });

/** The colour `hex` as three numbers, `count` times over. */
function repeat(hex: number, count: number): number[] {
  const c = new THREE.Color(hex);
  return Array.from({ length: count }, () => [c.r, c.g, c.b]).flat();
}

/** `view`'s buffers. */
function share(view: PartView): Shared {
  const scene = flatten(view);
  let faces: THREE.BufferGeometry | null = null;
  if (scene.triangles.length > 0) {
    const n = scene.triangles.length;
    const positions = new Float32Array(n * 9);
    const normals = new Float32Array(n * 9);
    const colors = new Float32Array(n * 9);
    scene.triangles.forEach(([a, b, c, hex], i) => {
      positions.set([...a, ...b, ...c], i * 9);
      normals.set(scene.normals[i].flat(), i * 9);
      colors.set(repeat(hex, 3), i * 9);
    });
    faces = new THREE.BufferGeometry();
    faces.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    faces.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    faces.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  }
  return {
    scene,
    faces,
    segments: new Float32Array(scene.lines.flatMap(([x0, y0, z0, x1, y1, z1]) => [x0, y0, z0, x1, y1, z1])),
    segmentColors: new Float32Array(scene.lines.flatMap((l) => repeat(l[6], 2))),
    points: new Float32Array(scene.points.flatMap(([x, y, z]) => [x, y, z])),
    pointColors: new Float32Array(scene.points.flatMap((p) => repeat(p[3], 1))),
    batch: null,
    built: "",
  };
}

/** `coords` — `[x, y, z]` after one another — moved by each of `matrices`, one copy per matrix. */
function moved(coords: Float32Array, matrices: THREE.Matrix4[]): Float32Array {
  const out = new Float32Array(coords.length * matrices.length);
  const p = new THREE.Vector3();
  matrices.forEach((m, k) => {
    const offset = k * coords.length;
    for (let i = 0; i < coords.length; i += 3) {
      p.set(coords[i], coords[i + 1], coords[i + 2]).applyMatrix4(m);
      out[offset + i] = p.x;
      out[offset + i + 1] = p.y;
      out[offset + i + 2] = p.z;
    }
  });
  return out;
}

/** `colors` once per copy of `copies`. */
function copied(colors: Float32Array, copies: number): Float32Array {
  const out = new Float32Array(colors.length * copies);
  for (let k = 0; k < copies; k++) out.set(colors, k * colors.length);
  return out;
}

/** The batch of `shared` for the placed parts at `matrices`. */
function batchOf(shared: Shared, matrices: THREE.Matrix4[]): THREE.Group {
  const group = new THREE.Group();
  if (shared.faces) {
    // The geometry is the component's, shared: disposing the batch leaves it.
    const mesh = new THREE.InstancedMesh(shared.faces, FACES, matrices.length);
    matrices.forEach((m, i) => mesh.setMatrixAt(i, m));
    mesh.instanceMatrix.needsUpdate = true;
    mesh.computeBoundingSphere();
    group.add(mesh);
    // A section caps the placed parts as it does the part's own solids.
    group.add(...stencilCounters(shared.faces, matrices));
  }
  const merged = (coords: Float32Array, colors: Float32Array) => {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(moved(coords, matrices), 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(copied(colors, matrices.length), 3));
    return geometry;
  };
  if (shared.segments.length > 0) group.add(new THREE.LineSegments(merged(shared.segments, shared.segmentColors), LINES));
  if (shared.points.length > 0) group.add(new THREE.Points(merged(shared.points, shared.pointColors), POINTS));
  return group;
}

/** Free what a batch owns: its merged buffers, its instanced meshes and its section counters' materials — not the component's geometry, nor the shared materials. */
function disposeBatch(batch: THREE.Group) {
  for (const child of batch.children) {
    if (child instanceof THREE.InstancedMesh) {
      child.dispose();
      if (child.userData.stencil) (child.material as THREE.Material).dispose();
    } else (child as THREE.LineSegments | THREE.Points).geometry.dispose();
  }
  batch.clear();
}

/** The placed parts of the part drawn, as three.js draws them: see the module docs. */
export class PlacedLayer {
  readonly group = new THREE.Group();
  private shared = new Map<string, Shared>();
  private singles = new Map<string, Single>();
  private instances: ViewInstance[] = [];
  private components: Record<string, PartView> = {};

  /** Draw `instances`, from the views `components`, each looking as `look` says. */
  update(instances: ViewInstance[], components: Record<string, PartView>, look: (name: string) => PlacedLook) {
    this.instances = instances;
    this.components = components;
    this.light(look);
  }

  /** Draw the placed parts as `look` says, rebuilding only the batches whose parts changed. */
  light(look: (name: string) => PlacedLook) {
    // Which component's batch each placed part goes into, or drawn alone.
    const batched = new Map<string, ViewInstance[]>();
    const alone = new Map<string, { instance: ViewInstance; look: PlacedLook }>();
    for (const instance of this.instances) {
      const view = this.components[instance.component];
      if (!view) continue;
      const seen = look(instance.name);
      if (!seen.visible) continue;
      if (seen.highlights.length > 0 || seen.hidden.length > 0) {
        alone.set(instance.name, { instance, look: seen });
      } else {
        const list = batched.get(instance.component) ?? [];
        list.push(instance);
        batched.set(instance.component, list);
      }
    }

    // The components used: built once, and freed once no placed part uses them.
    const used = new Set([...batched.keys(), ...[...alone.values()].map((a) => a.instance.component)]);
    for (const [key, shared] of this.shared) {
      if (used.has(key) && this.components[key]) continue;
      if (shared.batch) {
        this.group.remove(shared.batch);
        disposeBatch(shared.batch);
      }
      shared.faces?.dispose();
      this.shared.delete(key);
    }
    for (const key of used) {
      if (!this.shared.has(key)) this.shared.set(key, share(this.components[key]));
    }

    for (const [key, shared] of this.shared) {
      const parts = batched.get(key) ?? [];
      const built = JSON.stringify(parts.map((p) => [p.name, p.frame]));
      if (built === shared.built) continue;
      if (shared.batch) {
        this.group.remove(shared.batch);
        disposeBatch(shared.batch);
        shared.batch = null;
      }
      shared.built = built;
      if (parts.length === 0) continue;
      shared.batch = batchOf(
        shared,
        parts.map((p) => frameMatrix(p.frame)),
      );
      this.group.add(shared.batch);
    }

    for (const [name, single] of this.singles) {
      const now = alone.get(name);
      if (now && now.instance.component === single.component) continue;
      this.group.remove(single.group);
      disposeGroup(single.group);
      this.singles.delete(name);
    }
    for (const [name, { instance, look: seen }] of alone) {
      const shared = this.shared.get(instance.component);
      if (!shared) continue;
      let single = this.singles.get(name);
      if (!single) {
        const group = buildSceneGroup(shared.scene);
        group.matrixAutoUpdate = false;
        this.group.add(group);
        single = { component: instance.component, group };
        this.singles.set(name, single);
      }
      single.group.matrix.copy(frameMatrix(instance.frame));
      single.group.matrixWorldNeedsUpdate = true;
      applyHighlight(single.group, shared.scene, seen.highlights, seen.hidden);
    }
  }

  /** Free everything drawn. */
  dispose() {
    for (const single of this.singles.values()) disposeGroup(single.group);
    for (const shared of this.shared.values()) {
      if (shared.batch) disposeBatch(shared.batch);
      shared.faces?.dispose();
    }
    this.singles.clear();
    this.shared.clear();
    this.group.clear();
  }
}
