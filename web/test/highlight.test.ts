import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";
import type { Lit } from "../src/geop.ts";
import { applyHighlight, LIT_FACE_OPACITY, type Scene, type TriangleTags } from "../src/partScene.ts";

/** Two faces of one triangle each, `front` nearer the eye than `back`. */
function twoFaces(): THREE.Group {
  const positions = new Float32Array([0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0]);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(18).fill(0.5), 3));
  const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ vertexColors: true }));
  const group = new THREE.Group();
  group.add(mesh);
  const tags: TriangleTags = {
    mesh,
    base: new Float32Array(18).fill(0.5),
    triangleFaces: [0, 1],
    faces: [{ name: "front" }, { name: "back" }] as unknown as TriangleTags["faces"],
  };
  group.userData.triangles = tags;
  return group;
}

const NO_LINES = {
  lines: [],
  line_edges: [],
  line_sketches: [],
  line_curves: [],
  points: [],
  point_names: [],
  sketch_points: [],
} as unknown as Scene;

function lit(faces: string[]): Lit {
  return {
    whole: false,
    faces,
    edges: [],
    vertices: [],
    solids: [],
    features: [],
    sketches: [],
    curves: [],
    sketch_points: [],
    datums: [],
  };
}

/** The overlay `applyHighlight` added to `group`, if any. */
const overlayOf = (group: THREE.Group) => group.children.find((c) => c.userData.overlay) as THREE.Group | undefined;

test("a lit face is drawn again over what lies in front of it, ignoring depth", () => {
  const group = twoFaces();
  applyHighlight(group, NO_LINES, lit(["back"]), []);
  const overlay = overlayOf(group);
  assert.ok(overlay, "lit, the back face has an overlay");
  const mesh = overlay.children.find((c) => c instanceof THREE.Mesh) as THREE.Mesh;
  assert.ok(mesh, "the overlay draws the face");
  const material = mesh.material as THREE.MeshBasicMaterial;
  assert.equal(material.depthTest, false, "it is not hidden by the face in front");
  assert.equal(material.depthWrite, false);
  assert.ok(material.transparent && material.opacity === LIT_FACE_OPACITY, "it is see-through");
  assert.equal(mesh.renderOrder, 999, "it is drawn after the part");
  const drawn = Array.from(mesh.geometry.getAttribute("position").array);
  assert.deepEqual(drawn, [0, 0, 0, 1, 0, 0, 0, 1, 0], "exactly the back face's triangle");
});

test("lighting nothing leaves no overlay, and lighting again replaces the last", () => {
  const group = twoFaces();
  applyHighlight(group, NO_LINES, lit(["front"]), []);
  applyHighlight(group, NO_LINES, lit(["back"]), []);
  assert.equal(group.children.filter((c) => c.userData.overlay).length, 1);
  applyHighlight(group, NO_LINES, null, []);
  assert.equal(overlayOf(group), undefined);
});
