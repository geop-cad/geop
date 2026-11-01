// Green's area against known areas and a finer rule; the sliver's sign trap.
const near = (a, b, e) => Math.abs(a - b) <= e;

export function gauss_legendre_integrates_degree_five_exactly(algo, assert) {
  const f = (t) => 7 * t ** 5 - 3 * t ** 4 + t ** 2 - 2;
  const exact = 7 / 6 - 3 / 5 + 1 / 3 - 2;
  const q = algo.GL.reduce((s, [t, w]) => s + w * f(t), 0);
  assert.ok(near(q, exact, 1e-14), `${q} vs ${exact}`);
}
export function a_square_has_its_area_and_its_sign(algo, assert) {
  const c = [[0, 0], [2, 0], [2, 3], [0, 3]];
  const loop = c.map((p, i) => [algo.lineSpan(p, c[(i + 1) % 4])]);
  assert.ok(near(algo.greenArea(loop), 6, 1e-14));
  const cw = c.slice().reverse().map((p, i, r) => [algo.lineSpan(p, r[(i + 1) % 4])]);
  assert.ok(near(algo.greenArea(cw), -6, 1e-14));
}
export function exact_on_a_cubic_span_against_a_fine_rule(algo, assert) {
  const loop = algo.blob();
  const N = 400;
  const fine = Array.from({ length: N }, (_, i) => [(i + 0.5) / N, 1 / N]);
  assert.ok(near(algo.greenArea(loop), algo.greenArea(loop, fine), 1e-5));
}
export function the_polygon_converges_to_the_area(algo, assert) {
  const loop = algo.blob();
  const a = algo.greenArea(loop);
  const e1 = Math.abs(algo.polygonArea(algo.samplePolygon(loop, 8)) - a);
  const e2 = Math.abs(algo.polygonArea(algo.samplePolygon(loop, 64)) - a);
  assert.ok(e2 < e1 / 16, `${e1} ${e2}`);
}
export function the_area_does_not_depend_on_the_origin(algo, assert) {
  const loop = algo.blob();
  const moved = loop.map((pc) => pc.map((s) => s.map((p) => [p[0] + 1000, p[1] - 500])));
  assert.ok(near(algo.greenArea(loop), algo.greenArea(moved), 1e-7));
}
export function the_sliver_samples_to_the_wrong_sign(algo, assert) {
  const loop = algo.sliver(0.02, -0.05, 8);
  const area = algo.greenArea(loop);
  const poly = algo.polygonArea(algo.samplePolygon(loop, 8));
  assert.ok(area > 0, `true area ${area}`);
  assert.ok(poly < 0, `polygon ${poly}`);
  // The mean height of the wiggle is 0.02, over a unit base.
  assert.ok(near(area, 0.02, 0.003), `${area}`);
}
