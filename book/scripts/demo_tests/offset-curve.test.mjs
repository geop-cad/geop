// O' = S' (1 - d kappa) against a finite difference, the fit's order, and cusps at
// the centre of curvature.
const P = [[0.5, 0.9], [1.7, 3.6], [3.9, 3.5], [5.5, 1.2]];
export function offset_derivative_formula(algo, assert) {
  const d = 0.6, h = 1e-5;
  for (const t of [0.1, 0.4, 0.8]) {
    const a = algo.offsetPoint(P, t + h, d), b = algo.offsetPoint(P, t - h, d), m = algo.offsetDerivative(P, t, d);
    assert.ok(Math.hypot((a[0] - b[0]) / (2 * h) - m[0], (a[1] - b[1]) / (2 * h) - m[1]) < 1e-5);
  }
}
export function points_are_d_from_the_curve_along_the_normal(algo, assert) {
  const d = 0.37;
  for (const t of [0, 0.3, 1]) {
    const o = algo.offsetPoint(P, t, d), s = algo.b(P, t);
    assert.ok(Math.abs(Math.hypot(o[0] - s[0], o[1] - s[1]) - d) < 1e-12);
  }
}
export function straight_curve_offsets_exactly(algo, assert) {
  const L = [[0, 0], [1, 0], [2, 0], [3, 0]];
  assert.ok(algo.fitError(L, 0.5, 1, 40) < 1e-12);
}
export function hermite_fit_falls_at_fourth_order(algo, assert) {
  const ratio = algo.fitError(P, 0.4, 8, 40) / algo.fitError(P, 0.4, 16, 40);
  assert.ok(ratio > 10 && ratio < 22, `ratio ${ratio}`);
}
export function curvature_of_a_parabola(algo, assert) {
  // y = x^2 as a Bezier: kappa at the vertex is 2.
  const Q = [[-1, 1], [-1 / 3, -1 / 3], [1 / 3, -1 / 3], [1, 1]];
  assert.ok(Math.abs(algo.kappa(Q, 0.5) - 2) < 1e-12);
}
export function cusps_appear_past_the_radius_of_curvature(algo, assert) {
  const Q = [[-1, 1], [-1 / 3, -1 / 3], [1 / 3, -1 / 3], [1, 1]];
  // Radius of curvature 0.5 at the vertex, on the curve's left: a smaller offset is regular, a larger folds.
  assert.equal(algo.cusps(Q, 0.45, 2000).length, 0);
  assert.equal(algo.cusps(Q, 0.55, 2000).length, 2);
}
