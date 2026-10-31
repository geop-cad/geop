import assert from "node:assert/strict";
import { test } from "node:test";
import { DEFAULT_POSE, cubeRegion, regionName, standardPose } from "../src/camera.ts";
import type { Vec3 } from "../src/geop.ts";

test("a point of the view cube is on a face, else on the edge or the corner it is near", () => {
  assert.deepEqual(cubeRegion([0.1, 0.5, -0.2]), [0, 1, 0]);
  assert.deepEqual(cubeRegion([0.45, 0.5, 0]), [1, 1, 0]);
  assert.deepEqual(cubeRegion([0.45, 0.5, -0.4]), [1, 1, -1]);
  assert.deepEqual(cubeRegion([-0.5, 0, 0.29]), [-1, 0, 0]);
  assert.equal(regionName([0, 1, 0]), "Top");
  assert.equal(regionName([1, 1, -1]), "Top Back Right");
  assert.equal(regionName([0, 0, 1]), "Front");
});

const close = (a: Vec3, b: Vec3) => a.every((x, k) => Math.abs(x - b[k]) < 1e-12);

test("a standard view keeps the target and the distance, the world's y up", () => {
  const pose = { position: [10, 0, 0] as Vec3, target: [1, 2, 3] as Vec3, up: [0, 0, 1] as Vec3 };
  const front = standardPose([0, 0, 1], pose);
  const distance = Math.hypot(9, 2, 3);
  assert.ok(close(front.position, [1, 2, 3 + distance]), `${front.position}`);
  assert.deepEqual(front.target, [1, 2, 3]);
  assert.ok(close(front.up, [0, 1, 0]));
  // From the top the back is up, so the front is at the foot of the view.
  const top = standardPose([0, 1, 0], DEFAULT_POSE);
  assert.ok(close(top.up, [0, 0, -1]), `${top.up}`);
  assert.ok(close(standardPose([0, -1, 0], DEFAULT_POSE).up, [0, 0, 1]));
  // A corner: up is the world's y, square to the line of sight.
  const iso = standardPose([1, 1, 1], DEFAULT_POSE);
  const sight = iso.position.map((p, k) => p - iso.target[k]);
  assert.ok(Math.abs(sight.reduce((s, x, k) => s + x * iso.up[k], 0)) < 1e-9);
  assert.ok(iso.up[1] > 0.8);
});
