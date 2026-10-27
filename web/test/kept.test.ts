import assert from "node:assert/strict";
import { test } from "node:test";
import type { FaceMesh, SceneState, SentFaceMesh, SentPartView } from "../src/geop.ts";
import { applyScene, NOTHING_PLACED, readMesh } from "../src/kept.ts";

/** A mesh of one triangle at height `z`. */
function mesh(z: number): FaceMesh {
  return {
    triangles: [
      [
        [0, 0, z],
        [1, 0, z],
        [0, 1, z],
      ],
    ],
    normals: [
      [
        [0, 0, 1],
        [0, 0, 1],
        [0, 0, 1],
      ],
    ],
  };
}

/** `values` as the kernel sends numbers: little endian, in base64. */
function base64(values: Float32Array | Uint32Array): string {
  return btoa(String.fromCharCode(...new Uint8Array(values.buffer)));
}

/** `mesh` as the kernel sends it: each corner once, each triangle by its corners. */
function sent(mesh: FaceMesh): SentFaceMesh {
  const corners: number[][] = [];
  const indices: number[] = [];
  mesh.triangles.forEach((triangle, t) =>
    triangle.forEach((p, k) => {
      const corner = [...p, ...mesh.normals[t][k]];
      let at = corners.findIndex((c) => c.every((x, i) => x === corner[i]));
      if (at < 0) at = corners.push(corner) - 1;
      indices.push(at);
    }),
  );
  return {
    positions: base64(new Float32Array(corners.flatMap((c) => c.slice(0, 3)))),
    normals: base64(new Float32Array(corners.flatMap((c) => c.slice(3)))),
    indices: base64(new Uint32Array(indices)),
  };
}

test("a mesh is read as the kernel sent it", () => {
  const two: FaceMesh = {
    triangles: [...mesh(0.5).triangles, [[1, 0, 0.5], [1, 1, 0.5], [0, 1, 0.5]]],
    normals: [...mesh(0.5).normals, mesh(0.5).normals[0]],
  };
  assert.deepEqual(readMesh(sent(two)), two);
  // As `geop_core_editor::view::FaceMesh` writes one triangle at z = 1:
  // its three corners, then indices 0 1 2.
  const kernel: SentFaceMesh = {
    positions: "AAAAAAAAAAAAAIA/AACAPwAAAAAAAIA/AAAAAAAAgD8AAIA/",
    normals: "AAAAAAAAAAAAAIA/AAAAAAAAAAAAAIA/AAAAAAAAAAAAAIA/",
    indices: "AAAAAAEAAAACAAAA",
  };
  assert.deepEqual(readMesh(kernel), mesh(1));
});

/** A part of the faces `faces`, each named with the key of its mesh. */
function part(faces: [string, string][]): SentPartView {
  return {
    model: "model",
    datum_target: "",
    color: null,
    extent: { min: [0, 0, 0], max: [1, 1, 1] },
    vertices: [],
    edges: [],
    faces: faces.map(([name, key]) => ({ name, solid: "s", mesh: key })),
    sketches: [],
    datums: [],
    annotations: [],
    solids: ["s"],
  } as unknown as SentPartView;
}

/** A scene of `faces`, sending `meshes` and forgetting `forgotten`. */
function scene(
  faces: [string, string][],
  meshes: Record<string, FaceMesh>,
  { all = false, forgotten = [] as string[] } = {},
): SceneState {
  return {
    part: part(faces),
    structure: [],
    hidden: [],
    instances: [],
    all,
    removed: [],
    components: {},
    forgotten: [],
    meshes: Object.fromEntries(Object.entries(meshes).map(([key, m]) => [key, sent(m)])),
    forgotten_meshes: forgotten,
  };
}

test("a face is drawn with the mesh sent before by its key, the same mesh", () => {
  const first = applyScene(NOTHING_PLACED, scene([["top", "a"], ["side", "b"]], { a: mesh(1), b: mesh(0) }));
  const again = applyScene(first.placed, scene([["top", "a"]], {}));
  assert.equal(again.scene.part.faces[0].triangles, first.scene.part.faces[0].triangles);
  assert.deepEqual(again.scene.part.faces[0].triangles, mesh(1).triangles);
});

test("a mesh forgotten is not kept, and one never sent is an error", () => {
  const first = applyScene(NOTHING_PLACED, scene([["top", "a"]], { a: mesh(1) }));
  const second = applyScene(first.placed, scene([["bottom", "b"]], { b: mesh(0) }, { forgotten: ["a"] }));
  assert.deepEqual(Object.keys(second.placed.meshes), ["b"]);
  assert.throws(() => applyScene(second.placed, scene([["top", "a"]], {})), /never sent/);
});

test("a scene that is all there is forgets every mesh kept before", () => {
  const first = applyScene(NOTHING_PLACED, scene([["top", "a"]], { a: mesh(1) }));
  const all = applyScene(first.placed, scene([["bottom", "b"]], { b: mesh(0) }, { all: true }));
  assert.deepEqual(Object.keys(all.placed.meshes), ["b"]);
});
