// Parity against the known answer for a 4 x 3 rectangle less a circle of radius
// 0.5 at (1.5, 1.5); the ray's direction must not change the answer; a strip that
// grazes the hole flips it.
const hole = [1.5, 1.5];
function truth(p) {
  const inRect = p[0] > 0 && p[0] < 4 && p[1] > 0 && p[1] < 3;
  return inRect && Math.hypot(p[0] - hole[0], p[1] - hole[1]) > 0.5 ? "Inside" : "Outside";
}
export function parity_is_right_whatever_the_direction(algo, assert) {
  const trim = algo.trim(hole);
  const queries = [[2.5, 1.5], [1.5, 1.5], [0.5, 2.5], [5, 1], [-1, -1], [3.9, 2.9], [1.5, 1.8], [1.5, 2.2]];
  for (const q of queries) {
    for (let k = 0; k < 16; k++) {
      const a = 0.37 + k * 0.391, d = [Math.cos(a), Math.sin(a)];
      const r = algo.classify(trim, q, d, 0, 1e-3);
      if (r.cls === "Rejected") continue;
      assert.equal(r.cls, truth(q), `${q} along ${a}`);
    }
  }
}
export function on_the_boundary_is_said(algo, assert) {
  const trim = algo.trim(hole);
  assert.equal(algo.classify(trim, [0, 0], [1, 0], 0, 1e-3).cls, "OnVertex");
  assert.equal(algo.classify(trim, [2, 0], [0.6, 0.8], 0, 1e-3).cls, "OnCoedge");
  assert.equal(algo.classify(trim, [2, 1.5], [0.6, 0.8], 0, 1e-3).cls, "OnVertex");
}
export function a_ray_through_a_vertex_is_rejected(algo, assert) {
  const trim = algo.trim(hole);
  const q = [2, 2], d = [(4 - 2) / Math.hypot(2, 1), (3 - 2) / Math.hypot(2, 1)];
  assert.equal(algo.classify(trim, q, d, 0, 1e-3).cls, "Rejected");
}
export function a_strip_grazing_the_rim_flips_the_parity(algo, assert) {
  const trim = algo.trim(hole);
  // From 0.8 off the hole's axis, along a ray passing 0.53 from it: 0.03 clear
  // of the rim. A sharp ray misses it, and a strip half-width 0.06 overlaps it.
  const q = [hole[0] + 0.8, hole[1]];
  const a = Math.PI - Math.asin(0.53 / 0.8), d = [Math.cos(a), Math.sin(a)];
  const sharp = algo.classify(trim, q, d, 0, 1e-3);
  const strip = algo.classify(trim, q, d, 0.06, 1e-3);
  assert.equal(sharp.cls, "Inside");
  assert.equal(strip.cls, "Outside");
}
