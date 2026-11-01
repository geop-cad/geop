import assert from "node:assert/strict";
import { test } from "node:test";
import type { PartView, Vec3 } from "../src/geop.ts";
import { colorHex, flatten, sameFaces } from "../src/partScene.ts";

const triangle: [Vec3, Vec3, Vec3] = [
  [0, 0, 0],
  [1, 0, 0],
  [0, 1, 0],
];
const up: [Vec3, Vec3, Vec3] = [
  [0, 0, 1],
  [0, 0, 1],
  [0, 0, 1],
];

/** A part of two faces, `top` and `side`, of one triangle each, drawn from one mesh of each. */
function part(color: string | null, face_colors?: Record<string, string>): PartView {
  const face = (name: string) => ({ name, solid: "s", triangles: [triangle], normals: [up] });
  return { faces: [face("top"), face("side")], color, face_colors, vertices: [], edges: [], sketches: [], annotations: [] } as unknown as PartView;
}

/** The colours of the triangles of `view`, as drawn. */
function drawn(view: PartView, faceColors = true): number[] {
  return flatten(view, [], faceColors).triangles.map((t) => t[3]);
}

test("a face's colour is over the part's, and a name no face has draws nothing", () => {
  const view = part("#0000ff", { top: "#ff0000", gone: "#00ff00" });
  assert.deepEqual(drawn(view), [0xff0000, 0x0000ff]);
});

test("in the part's colour alone, under a colour the placed part is drawn in", () => {
  const view = part("#0000ff", { top: "#ff0000" });
  assert.deepEqual(drawn(view, false), [0x0000ff, 0x0000ff]);
});

test("a part with no colour of its own keeps the viewer's, beside a face's", () => {
  const [top, side] = drawn(part(null, { top: "#ff0000" }));
  assert.equal(top, 0xff0000);
  assert.equal(side, colorHex(null, side));
});

test("the faces are built again when one of them changes colour, and only then", () => {
  const colored = (color: string) => {
    const scene = flatten(part("#0000ff", { top: color }));
    // One mesh for both scenes, as the kernel sends a mesh once.
    scene.face_meshes = ["top-mesh", "side-mesh"];
    return scene;
  };
  assert.ok(sameFaces(colored("#ff0000"), colored("#ff0000")));
  assert.ok(!sameFaces(colored("#ff0000"), colored("#00ff00")));
});
