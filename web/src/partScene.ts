// A part as three.js draws it: flattened into buffers tagged with the names
// of what they draw ([[flatten]]), built into meshes, lines and points
// ([[buildSceneGroup]]), and highlighted by name ([[applyHighlight]]) —
// shared by the part drawn and the parts placed in it (see `placed3d.ts`).

import * as THREE from "three";
import type { EntityRef, Frame, PartView, Vec3 } from "./geop";

const vec = (v: [number, number, number]) => new THREE.Vector3(v[0], v[1], v[2]);

/** How the part's entities are colored. */
export const VERTEX_COLOR = 0x3c3c3c;
export const EDGE_COLOR = 0x808080;
export const FACE_COLOR = 0x4472c4;
/** Sketch curves: profile geometry and construction geometry. */
export const SKETCH_COLOR = 0xffa040;
export const CONSTRUCTION_COLOR = 0x808080;
/** Cosmetic threads: the helix each is drawn as on its face. */
const THREAD_COLOR = 0x2a2a2a;

/**
 * A part as flat buffers, each element colored and tagged with the name of
 * what it draws — what three.js takes, and what highlighting by name needs.
 */
export interface Scene {
  /** `[x, y, z, colorHex]` per point, and the name of the vertex each draws. */
  points: [number, number, number, number][];
  point_names: string[];
  /** `[x0, y0, z0, x1, y1, z1, colorHex]` per line segment. */
  lines: [number, number, number, number, number, number, number][];
  /** Per line: the sketch it belongs to and the id of its curve there, or the edge it draws. */
  line_sketches: (string | null)[];
  line_curves: (number | null)[];
  line_edges: (string | null)[];
  /** Every sketch's points, by the sketch and their id in it. */
  sketch_points: { sketch: string; id: number; at: Vec3 }[];
  /** Per triangle: its corners, its color, its corners' normals, and the index in `faces` of its face. */
  triangles: [Vec3, Vec3, Vec3, number][];
  normals: [Vec3, Vec3, Vec3][];
  triangle_faces: number[];
  faces: { name: string; solid: string | null }[];
}

/** The point `(x, y)` of `plane`. */
export function inPlane(plane: Frame, [x, y]: [number, number]): Vec3 {
  return [0, 1, 2].map((k) => plane.origin[k] + x * plane.u[k] + y * plane.v[k]) as Vec3;
}

/** A colour `#rrggbb` as a number three.js takes; `fallback` for none, or one it cannot read. */
export function colorHex(color: string | null | undefined, fallback: number): number {
  const hex = color?.match(/^#([0-9a-fA-F]{6})$/)?.[1];
  return hex ? parseInt(hex, 16) : fallback;
}

/**
 * `part` as a [[Scene]], its faces in its own colour if it has one, without
 * the solids and the faces standing on their own in `hidden` — nor the edges
 * and vertices of only those.
 */
export function flatten(part: PartView, hidden: string[] = []): Scene {
  const faceColor = colorHex(part.color, FACE_COLOR);
  // An edge or a vertex bounds its faces: of no solid, it goes when they all do.
  const shown = (solid: string | null, faces: string[] = []) =>
    (solid == null || !hidden.includes(solid)) &&
    (solid != null || faces.length === 0 || faces.some((f) => !hidden.includes(f)));
  const scene: Scene = {
    points: [],
    point_names: [],
    lines: [],
    line_sketches: [],
    line_curves: [],
    line_edges: [],
    sketch_points: [],
    triangles: [],
    normals: [],
    triangle_faces: [],
    faces: [],
  };
  for (const v of part.vertices) {
    if (!shown(v.solid, v.faces)) continue;
    scene.points.push([...v.at, VERTEX_COLOR]);
    scene.point_names.push(v.name);
  }
  const line = (a: Vec3, b: Vec3, color: number, sketch: string | null, curve: number | null, edge: string | null) => {
    scene.lines.push([...a, ...b, color]);
    scene.line_sketches.push(sketch);
    scene.line_curves.push(curve);
    scene.line_edges.push(edge);
  };
  for (const e of part.edges) {
    if (!shown(e.solid, e.faces)) continue;
    for (let i = 1; i < e.polyline.length; i++) line(e.polyline[i - 1], e.polyline[i], EDGE_COLOR, null, null, e.name);
  }
  for (const t of part.threads) {
    for (let i = 1; i < t.polyline.length; i++) line(t.polyline[i - 1], t.polyline[i], THREAD_COLOR, null, null, null);
  }
  part.faces.forEach((f) => {
    if (!shown(f.solid) || (f.solid == null && hidden.includes(f.name))) return;
    const index = scene.faces.length;
    scene.faces.push({ name: f.name, solid: f.solid });
    f.triangles.forEach(([a, b, c], i) => {
      scene.triangles.push([a, b, c, faceColor]);
      scene.normals.push(f.normals[i]);
      scene.triangle_faces.push(index);
    });
  });
  for (const sketch of part.sketches) {
    for (const curve of sketch.curves) {
      const color = curve.construction ? CONSTRUCTION_COLOR : SKETCH_COLOR;
      const points = curve.polyline.map((p) => inPlane(sketch.plane, p));
      for (let i = 1; i < points.length; i++) line(points[i - 1], points[i], color, sketch.name, curve.id, null);
    }
    for (const point of sketch.points) {
      scene.sketch_points.push({ sketch: sketch.name, id: point.id, at: inPlane(sketch.plane, point.at) });
    }
  }
  for (const sketch of part.sketches3d ?? []) {
    for (const curve of sketch.curves) {
      const color = curve.construction ? CONSTRUCTION_COLOR : SKETCH_COLOR;
      const points = curve.polyline;
      for (let i = 1; i < points.length; i++) line(points[i - 1], points[i], color, sketch.name, curve.id, null);
    }
    for (const point of sketch.points) {
      scene.sketch_points.push({ sketch: sketch.name, id: point.id, at: point.at });
    }
  }
  return scene;
}

/** The colour a section view caps cut solids with. */
export const CAP_COLOR = 0xc86464;

/**
 * Two copies of a solid's triangles that draw nothing but count, in the
 * stencil buffer, how often the eye's ray through each pixel enters and
 * leaves the solid — back faces up, front faces down — so that where a
 * section has cut the solid open, the count is not zero and the cap is drawn
 * (see the cap in [[SceneViewer]]). Hidden until a section is on. With
 * `matrices`, one copy of the solid at each, as an `InstancedMesh` (see
 * `placed3d.ts`).
 */
export function stencilCounters(geometry: THREE.BufferGeometry, matrices?: THREE.Matrix4[]): THREE.Mesh[] {
  return (
    [
      [THREE.BackSide, THREE.IncrementWrapStencilOp],
      [THREE.FrontSide, THREE.DecrementWrapStencilOp],
    ] as const
  ).map(([side, op]) => {
    const material = new THREE.MeshBasicMaterial({
      side,
      colorWrite: false,
      depthWrite: false,
      depthTest: false,
      stencilWrite: true,
      stencilFunc: THREE.AlwaysStencilFunc,
      stencilFail: op,
      stencilZFail: op,
      stencilZPass: op,
    });
    let mesh: THREE.Mesh;
    if (matrices) {
      const instanced = new THREE.InstancedMesh(geometry, material, matrices.length);
      matrices.forEach((m, i) => instanced.setMatrixAt(i, m));
      instanced.instanceMatrix.needsUpdate = true;
      instanced.computeBoundingSphere();
      mesh = instanced;
    } else {
      mesh = new THREE.Mesh(geometry, material);
    }
    mesh.renderOrder = 1;
    mesh.visible = false;
    mesh.userData.stencil = true;
    return mesh;
  });
}

/** The color a highlighted entity is drawn in. */
export const HIGHLIGHT = new THREE.Color(0xffc94a);

/** What a triangle mesh needs to highlight faces by name: its base colors and each triangle's face. */
export interface TriangleTags {
  mesh: THREE.Mesh;
  base: Float32Array;
  triangleFaces: number[];
  faces: Scene["faces"];
}

/** Build the meshes/lines/points of one [[Scene]] into a group. */
export function buildSceneGroup(scene: Scene): THREE.Group {
  const group = new THREE.Group();

  // Triangles: one mesh, vertex colors so per-triangle color still works.
  if (scene.triangles.length > 0) {
    const positions = new Float32Array(scene.triangles.length * 9);
    const colors = new Float32Array(scene.triangles.length * 9);
    const normals = new Float32Array(scene.triangles.length * 9);
    scene.triangles.forEach(([a, b, c, hex], i) => {
      const o = i * 9;
      positions.set([...a, ...b, ...c], o);
      normals.set(scene.normals[i].flat(), o);
      const color = new THREE.Color(hex);
      for (let v = 0; v < 3; v++) {
        colors.set([color.r, color.g, color.b], o + v * 3);
      }
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    // The kernel's own normals: averaging the mesh's facet normals
    // (computeVertexNormals) cannot, since every triangle here has its own
    // three vertices — that is what made curved faces look faceted however
    // finely they were tessellated.
    geometry.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    const material = new THREE.MeshStandardMaterial({
      vertexColors: true,
      side: THREE.DoubleSide,
      roughness: 0.6,
    });
    const mesh = new THREE.Mesh(geometry, material);
    group.add(mesh);
    group.add(...stencilCounters(geometry));
    group.userData.triangles = {
      mesh,
      base: colors.slice(),
      triangleFaces: scene.triangle_faces,
      faces: scene.faces,
    } satisfies TriangleTags;
  }

  // Lines: one LineSegments per sketch and color (and one per color for
  // the model's edges), so a sketch can be highlighted on its own.
  const byOwner = new Map<string, { hex: number; sketch: string | null; coords: number[] }>();
  scene.lines.forEach(([x0, y0, z0, x1, y1, z1, hex], i) => {
    const sketch = scene.line_sketches[i];
    const key = `${sketch ?? ""}:${hex}`;
    const entry = byOwner.get(key) ?? { hex, sketch, coords: [] };
    entry.coords.push(x0, y0, z0, x1, y1, z1);
    byOwner.set(key, entry);
  });
  for (const { hex, sketch, coords } of byOwner.values()) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(coords), 3));
    const lines = new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: hex }));
    lines.userData = { sketch, color: hex };
    group.add(lines);
  }

  // Sketch points: one Points per sketch, shown and hidden with its lines.
  const bySketch = new Map<string, number[]>();
  for (const { sketch, at } of scene.sketch_points) {
    const coords = bySketch.get(sketch) ?? [];
    coords.push(...at);
    bySketch.set(sketch, coords);
  }
  for (const [sketch, coords] of bySketch) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(coords), 3));
    const material = new THREE.PointsMaterial({ color: SKETCH_COLOR, size: 5, sizeAttenuation: false });
    const points = new THREE.Points(geometry, material);
    points.userData = { sketch, color: SKETCH_COLOR };
    group.add(points);
  }

  // Points.
  if (scene.points.length > 0) {
    const positions = new Float32Array(scene.points.length * 3);
    const colors = new Float32Array(scene.points.length * 3);
    scene.points.forEach(([x, y, z, hex], i) => {
      positions.set([x, y, z], i * 3);
      const color = new THREE.Color(hex);
      colors.set([color.r, color.g, color.b], i * 3);
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    group.add(new THREE.Points(geometry, new THREE.PointsMaterial({ size: 0.02, vertexColors: true })));
  }

  return group;
}

/** Blend `colors[k..k+3]` most of the way to the highlight color. */
export function tint(colors: Float32Array, k: number) {
  colors[k] += (HIGHLIGHT.r - colors[k]) * 0.7;
  colors[k + 1] += (HIGHLIGHT.g - colors[k + 1]) * 0.7;
  colors[k + 2] += (HIGHLIGHT.b - colors[k + 2]) * 0.7;
}

/**
 * Draw `highlights` highlighted in `group` (built by [[buildSceneGroup]]
 * from `scene`), the sketches in `hidden` not at all — unless lit — and
 * everything else as built. Lit edges and vertices are drawn again on top,
 * so they can be seen wherever they are.
 */
export function applyHighlight(group: THREE.Group, scene: Scene, highlights: EntityRef[], hidden: string[]) {
  const named = (type: EntityRef["type"]) =>
    new Set(highlights.flatMap((h) => (h.type === type && "name" in h ? [h.name] : [])));
  const [faces, solids, planar, edges, vertices, spatial] = (
    ["Face", "Solid", "Sketch", "Edge", "Vertex", "Sketch3d"] as const
  ).map(named);
  const sketches = new Set([...planar, ...spatial]);
  /** A curve's or a point's key: its sketch and its id there. */
  const curveKey = (sketch: string, id: number) => `${sketch}\u0000${id}`;
  const curves = new Set(highlights.flatMap((h) => (h.type === "SketchCurve" ? [curveKey(h.sketch, h.curve)] : [])));
  const sketchPoints = new Set(
    highlights.flatMap((h) => (h.type === "SketchPoint" ? [curveKey(h.sketch, h.point)] : [])),
  );
  const tags = group.userData.triangles as TriangleTags | undefined;
  if (tags) {
    const attribute = tags.mesh.geometry.getAttribute("color") as THREE.BufferAttribute;
    const colors = attribute.array as Float32Array;
    colors.set(tags.base);
    tags.triangleFaces.forEach((f, i) => {
      const face = tags.faces[f];
      if (!face || !(faces.has(face.name) || (face.solid != null && solids.has(face.solid)))) return;
      for (let k = i * 9; k < i * 9 + 9; k += 3) tint(colors, k);
    });
    attribute.needsUpdate = true;
  }
  for (const child of group.children) {
    const drawn = child instanceof THREE.LineSegments || child instanceof THREE.Points;
    if (!drawn || child.userData.sketch == null) continue;
    const lit = sketches.has(child.userData.sketch);
    child.visible = lit || !hidden.includes(child.userData.sketch);
    const material = child.material as THREE.LineBasicMaterial | THREE.PointsMaterial;
    material.color.set(lit ? HIGHLIGHT : child.userData.color);
    // A sketch often lies on or behind the model (on a face, or under the
    // solid made from it): lit, it is drawn on top, so it can be seen.
    material.depthTest = !lit;
    child.renderOrder = lit ? 999 : 0;
  }

  const old = group.children.find((c) => c.userData.overlay);
  if (old) {
    group.remove(old);
    disposeGroup(old as THREE.Group);
  }
  if (edges.size === 0 && vertices.size === 0 && curves.size === 0 && sketchPoints.size === 0) return;
  const overlay = new THREE.Group();
  overlay.userData.overlay = true;
  const coords: number[] = [];
  scene.lines.forEach(([x0, y0, z0, x1, y1, z1], i) => {
    const edge = scene.line_edges[i];
    const sketch = scene.line_sketches[i];
    const curve = scene.line_curves[i];
    const lit =
      (edge != null && edges.has(edge)) || (sketch != null && curve != null && curves.has(curveKey(sketch, curve)));
    if (lit) coords.push(x0, y0, z0, x1, y1, z1);
  });
  if (coords.length) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(coords), 3));
    overlay.add(new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: HIGHLIGHT, depthTest: false })));
  }
  const points = scene.points
    .filter((_, i) => vertices.has(scene.point_names[i]))
    .map(([x, y, z]): Vec3 => [x, y, z])
    .concat(scene.sketch_points.filter((p) => sketchPoints.has(curveKey(p.sketch, p.id))).map((p) => p.at));
  if (points.length) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(points.flatMap(([x, y, z]) => [x, y, z])), 3));
    overlay.add(new THREE.Points(geometry, new THREE.PointsMaterial({ color: HIGHLIGHT, size: 9, sizeAttenuation: false, depthTest: false })));
  }
  overlay.traverse((o) => (o.renderOrder = 999));
  group.add(overlay);
}

/** Where `frame` puts what is drawn in its own coordinates. */
export function frameMatrix(frame: Frame): THREE.Matrix4 {
  return new THREE.Matrix4().makeBasis(vec(frame.u), vec(frame.v), vec(frame.w)).setPosition(vec(frame.origin));
}

/** The entities of `refs` that lie in the placed part `instance`, as it names them. */
export function localTo(instance: string, refs: EntityRef[]): EntityRef[] {
  const prefix = `${instance}/`;
  return refs.flatMap((r): EntityRef[] => {
    if (r.type === "SketchCurve" || r.type === "SketchPoint") {
      return r.sketch.startsWith(prefix) ? [{ ...r, sketch: r.sketch.slice(prefix.length) }] : [];
    }
    return r.name.startsWith(prefix) ? [{ ...r, name: r.name.slice(prefix.length) }] : [];
  });
}

/** Free every geometry and material a group owns. */
export function disposeGroup(group: THREE.Object3D) {
  group.traverse((o) => {
    const any = o as THREE.Mesh;
    any.geometry?.dispose();
    const material = any.material;
    if (Array.isArray(material)) material.forEach((m) => m.dispose());
    else material?.dispose();
  });
  group.clear();
}
