import assert from "node:assert/strict";
import { test } from "node:test";
import type { FaceMesh, SceneState, SentPartView } from "../src/geop.ts";
import { applyScene, NOTHING_PLACED } from "../src/kept.ts";

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
    meshes,
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
