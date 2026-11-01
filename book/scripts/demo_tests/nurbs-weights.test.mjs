// The homogeneous de Casteljau against the rational sum; the circle and the
// conic equation against the weights; the quotient rule against a difference.
const P = [[0.5, 0.5], [2.6, 3.2], [4.7, 0.5]];
const close = (a, b, e, assert) => assert.ok(Math.abs(a[0] - b[0]) < e && Math.abs(a[1] - b[1]) < e, `${a} vs ${b}`);

export function homogeneous_de_casteljau_agrees_with_the_sum(a, assert) {
  for (const w of [[1, 1, 1], [1, 0.3, 1], [2, 3, 0.5]]) {
    for (let i = 0; i <= 20; i++) close(a.evaluate(P, w, i / 20), a.rationalSum(P, w, i / 20), 1e-12, assert);
  }
}
export function equal_weights_are_the_polynomial_curve(a, assert) {
  const t = 0.3;
  const poly = [0, 1].map((c) => 0.49 * P[0][c] + 0.42 * P[1][c] + 0.09 * P[2][c]);
  close(a.evaluate(P, [1, 1, 1], t), poly, 1e-12, assert);
  close(a.evaluate(P, [3, 3, 3], t), poly, 1e-12, assert);
}
export function every_circle_span_is_on_the_circle(a, assert) {
  for (const spans of [3, 4, 5, 8]) assert.ok(a.circleError(spans, 1.6, 500) < 1e-14, `${spans} spans`);
}
export function a_quarter_is_weighted_cos_45(a, assert) {
  const s = a.arcSpan(0, 90, 1);
  assert.ok(Math.abs(s.weights[1] - Math.SQRT1_2) < 1e-15);
  close(s.points[1], [1, 1], 1e-15, assert);
}
export function conic_equation_holds_for_any_weights(a, assert) {
  for (const w of [[1, 1, 1], [1, 0.4, 1], [1, 2, 1], [2, 1.5, 0.7]]) {
    for (let i = 0; i <= 10; i++) {
      assert.ok(Math.abs(a.conicResidual(P, w, a.evaluate(P, w, i / 10))) < 1e-10, `${w} ${i}`);
    }
  }
}
export function reparametrizing_keeps_the_point_set_and_the_invariant(a, assert) {
  // w = (2, sqrt 2, 1) is (1, 1, 1) reparametrized by a linear rational map.
  assert.equal(a.conicType([2, Math.SQRT2, 1]), "parabola");
  for (let i = 0; i <= 10; i++) {
    const q = a.evaluate(P, [2, Math.SQRT2, 1], i / 10);
    assert.ok(Math.abs(a.conicResidual(P, [1, 1, 1], q)) < 1e-10);
  }
}
export function conic_type_by_invariant(a, assert) {
  assert.equal(a.conicType([1, 0.5, 1]), "ellipse");
  assert.equal(a.conicType([1, 1, 1]), "parabola");
  assert.equal(a.conicType([1, 2, 1]), "hyperbola");
}
export function quotient_rule_matches_a_central_difference(a, assert) {
  const w = [1, 0.7, 1.3], h = 1e-6;
  for (const t of [0.1, 0.5, 0.9]) {
    const num = [0, 1].map((c) => (a.evaluate(P, w, t + h)[c] - a.evaluate(P, w, t - h)[c]) / (2 * h));
    close(a.derivative(P, w, t), num, 1e-6, assert);
  }
}
