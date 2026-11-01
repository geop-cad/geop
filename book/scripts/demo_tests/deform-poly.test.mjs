// The composition on control points against the map applied point by point.
const P = [[0.3, 0], [1.6, 1.2], [-0.8, 2.8], [0.6, 4]];

export function product_of_bernstein_polynomials_is_the_product(algo, assert) {
  const a = [1, -2, 0.5, 3], b = [2, 1, -1];
  const ab = algo.product(a, b);
  assert.equal(ab.length, a.length + b.length - 1);
  for (let i = 0; i <= 10; i++) {
    const u = i / 10;
    assert.ok(Math.abs(algo.evalBernstein(ab, u) - algo.evalBernstein(a, u) * algo.evalBernstein(b, u)) < 1e-12);
  }
}
export function a_taper_of_a_cubic_is_exact_and_of_degree_six(algo, assert) {
  const map = algo.makeMap("taper", 0, 4, 0.3);
  const r = algo.compose(P, map, 1);
  assert.equal(r.degree, 6);
  assert.equal(r.bound, 0);
  assert.ok(r.measured < 1e-12, `measured ${r.measured}`);
}
export function a_twist_and_a_bend_are_of_degree_twelve_within_the_bound(algo, assert) {
  for (const [kind, amount] of [["twist", 3], ["twist", -1.5], ["bend", 1.5], ["bend", -2.7]]) {
    const map = algo.makeMap(kind, 0, 4, amount);
    for (const pieces of [1, 2, 4, 8]) {
      const r = algo.compose(P, map, pieces);
      assert.equal(r.degree, 12);
      assert.ok(r.measured <= r.bound + 1e-12, `${kind} ${pieces}: measured ${r.measured} above bound ${r.bound}`);
    }
  }
}
export function the_bound_falls_as_the_fourth_power_of_the_piece(algo, assert) {
  // A straight stretch up the axis: halving the piece halves the reach exactly.
  const line = [[0.5, 0], [0.5, 4 / 3], [0.5, 8 / 3], [0.5, 4]];
  const map = algo.makeMap("twist", 0, 4, 2);
  const a = algo.compose(line, map, 1).bound, b = algo.compose(line, map, 2).bound;
  assert.ok(Math.abs(a / b - 16) < 1e-9, `ratio ${a / b}`);
}
export function a_bend_winds_the_axis_onto_a_circle_of_the_zones_length(algo, assert) {
  const angle = 1.2, L = 4, R = L / angle, map = algo.makeMap("bend", 0, L, angle);
  for (let i = 0; i <= 10; i++) {
    const s = (L * i) / 10, q = algo.mapPoint(map, [0, s]);
    assert.ok(Math.abs(Math.hypot(q[0] - R, q[1]) - R) < 1e-12);
    // The arc length from the start is the distance along the axis.
    assert.ok(Math.abs(Math.atan2(q[1], R - q[0]) * R - s) < 1e-12);
  }
  // A point a distance x towards the centre stays that far off the arc.
  const q = algo.mapPoint(map, [0.5, L / 2]);
  assert.ok(Math.abs(Math.hypot(q[0] - R, q[1]) - (R - 0.5)) < 1e-12);
}
export function the_interpolant_passes_through_its_chebyshev_points(algo, assert) {
  const f = { k: 0, m: 0, c: 1, s: 0, r: 0.8 };
  const ip = algo.interpolant(f, 0, 2, 2, 3);
  for (let j = 0; j < 4; j++) {
    const n = Math.cos((Math.PI * (2 * j + 1)) / 8);
    const poly = ip.c.reduce((sum, c, k) => sum + c * n ** k, 0);
    assert.ok(Math.abs(poly - algo.evalCoefficient(f, 2 + 2 * n)) < 1e-12);
  }
}
