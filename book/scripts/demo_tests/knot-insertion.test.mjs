// Knot insertion must not move the curve; the intervals must enclose; a cut
// arc must stay on its circle; a wide cut must be the wider one.
const P = [[0.4, 0.5], [1.2, 3.2], [2.8, 0.6], [3.6, 3.4], [5.1, 3.3], [5.8, 0.8]];
const K = [0, 0, 0, 0, 1 / 3, 2 / 3, 1, 1, 1, 1];
const close = (x, y, assert) => assert.ok(Math.abs(x[0] - y[0]) < 1e-12 && Math.abs(x[1] - y[1]) < 1e-12, `${x} vs ${y}`);

export function inserting_a_knot_keeps_the_curve(a, assert) {
  const r = a.insertKnot(3, K, P, 0.45);
  assert.equal(r.points.length, P.length + 1);
  assert.equal(r.knots.length, K.length + 1);
  for (let i = 0; i <= 50; i++) close(a.evalSpline(3, K, P, i / 50), a.evalSpline(3, r.knots, r.points, i / 50), assert);
}
export function inserting_an_existing_knot_raises_its_multiplicity(a, assert) {
  const r = a.insertKnot(3, K, P, 1 / 3);
  assert.equal(r.knots.filter((k) => k === 1 / 3).length, 2);
  for (let i = 0; i <= 50; i++) close(a.evalSpline(3, K, P, i / 50), a.evalSpline(3, r.knots, r.points, i / 50), assert);
}
export function new_points_lie_on_the_old_polygon(a, assert) {
  const r = a.insertKnot(3, K, P, 0.45);
  assert.ok(r.alphas.every((x) => x > 0 && x < 1));
}
export function refining_keeps_the_curve_and_halves_every_span(a, assert) {
  const r = a.refineAll(3, K, P);
  assert.equal(a.spanBoxes(3, r.knots, r.points).length, 2 * a.spanBoxes(3, K, P).length);
  for (let i = 0; i <= 50; i++) close(a.evalSpline(3, K, P, i / 50), a.evalSpline(3, r.knots, r.points, i / 50), assert);
}
export function boxes_enclose_the_curve_on_their_span(a, assert) {
  for (const b of a.spanBoxes(3, K, P)) {
    for (let i = 0; i < 20; i++) {
      const t = K[b.span] + (K[b.span + 1] - K[b.span]) * i / 20;
      const q = a.evalSpline(3, K, P, t);
      assert.ok(q[0] >= b.x0 && q[0] <= b.x1 && q[1] >= b.y0 && q[1] <= b.y1);
    }
  }
}
export function intervals_round_outward_and_enclose(a, assert) {
  assert.ok(a.nextUp(1) > 1 && a.nextDown(1) < 1 && a.nextUp(0) > 0 && a.nextDown(0) < 0);
  assert.equal(a.nextDown(a.nextUp(1)), 1);
  const s = a.add(a.I(0.1), a.I(0.2));
  assert.ok(s[0] < 0.1 + 0.2 && s[1] > 0.1 + 0.2);
  const m = a.mul(a.I(-1, 2), a.I(3, 4));
  assert.ok(m[0] <= -4 && m[1] >= 8);
  const d = a.div(a.I(1), a.I(3));
  assert.ok(d[0] < 1 / 3 && d[1] > 1 / 3);
  assert.throws(() => a.div(a.I(1), a.I(-1, 1)));
}
export function interpolate_keeps_equal_ends_exactly(a, assert) {
  const one = a.I(1);
  assert.deepEqual(a.interpolate(one, one, a.I(0.2, 0.7)), one);
}
export function arc_pieces_stay_on_the_circle(a, assert) {
  const span = a.quarterArc(4);
  const t = a.parameterAt(span, Math.PI / 6, 4);
  const c = a.cutRight(span, a.I((t[0] + t[1]) / 2), undefined);
  const A = a.evalSpan(c, a.I((c.a[0] + c.b[0]) / 2));
  const x = (A[0][0] + A[0][1]) / (A[2][0] + A[2][1]), y = (A[1][0] + A[1][1]) / (A[2][0] + A[2][1]);
  assert.ok(Math.abs(Math.hypot(x, y) - 4) < 1e-12);
}
export function parameter_found_reaches_the_point(a, assert) {
  const span = a.quarterArc(4);
  const t = a.parameterAt(span, Math.PI / 3, 4);
  const A = a.evalSpan(span, t);
  const c = [(A[0][0] + A[0][1]) / (A[2][0] + A[2][1]), (A[1][0] + A[1][1]) / (A[2][0] + A[2][1])];
  assert.ok(Math.abs(c[0] - 2) < 1e-12 && Math.abs(c[1] - 4 * Math.sin(Math.PI / 3)) < 1e-12);
}
export function wide_cuts_compound_and_sharp_cuts_do_not(a, assert) {
  const wide = a.cutManyTimes(40, true), sharp = a.cutManyTimes(40, false);
  assert.ok(sharp[40] < 1e-12, `sharp ${sharp[40]}`);
  assert.ok(!isFinite(wide[40]) || wide[40] > 1000 * sharp[40], `wide ${wide[40]} sharp ${sharp[40]}`);
}
