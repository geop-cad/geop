// Cox-de Boor against de Boor, partition of unity, local support, and the
// derivative's formula against a central difference.
const P = [[0, 0], [1, 2], [2, -1], [3, 3], [4, 0], [5, 1], [6, 2]];
const K = [0, 0, 0, 0, 0.2, 0.5, 0.5, 1, 1, 1, 1];
const samples = Array.from({ length: 101 }, (_, i) => i / 100);

export function de_boor_agrees_with_the_basis_sum(a, assert) {
  for (const t of samples) {
    const x = a.deBoor(3, K, P, t), y = a.byBasis(3, K, P, t);
    assert.ok(Math.abs(x[0] - y[0]) < 1e-12 && Math.abs(x[1] - y[1]) < 1e-12, `t=${t}: ${x} vs ${y}`);
  }
}
export function partition_of_unity(a, assert) {
  for (const t of samples) {
    let s = 0;
    for (let i = 0; i < P.length; i++) s += a.basis(i, 3, K, t, 1);
    assert.ok(Math.abs(s - 1) < 1e-12, `t=${t}: ${s}`);
  }
}
export function local_support_and_non_negative(a, assert) {
  for (const t of samples) {
    for (let i = 0; i < P.length; i++) {
      const b = a.basis(i, 3, K, t, 1);
      assert.ok(b >= 0);
      if (t < K[i] || t > K[i + 4]) assert.equal(b, 0);
    }
  }
}
export function clamped_curve_interpolates_its_ends(a, assert) {
  assert.deepEqual(a.deBoor(3, K, P, 0), P[0]);
  assert.deepEqual(a.deBoor(3, K, P, 1), P[6]);
}
export function derivative_matches_a_central_difference(a, assert) {
  const d = a.derivativeSpline(3, K, P);
  for (const t of [0.05, 0.3, 0.42, 0.7, 0.95]) {
    const h = 1e-6;
    const num = [0, 1].map((c) => (a.deBoor(3, K, P, t + h)[c] - a.deBoor(3, K, P, t - h)[c]) / (2 * h));
    const got = a.deBoor(d.degree, d.knots, d.points, t);
    assert.ok(Math.abs(got[0] - num[0]) < 1e-5 && Math.abs(got[1] - num[1]) < 1e-5, `t=${t}: ${got} vs ${num}`);
  }
}
export function multiplicity_drops_continuity(a, assert) {
  const j = a.joints(3, K, P.length);
  assert.equal(JSON.stringify(Array.from(j).map((x) => [x.knot, x.multiplicity, x.continuity])), "[[0.2,1,2],[0.5,2,1]]");
  // At a knot of multiplicity 2 the first derivative is continuous and the second jumps.
  const d = a.derivativeSpline(3, K, P), d2 = a.derivativeSpline(2, d.knots, d.points);
  const e = 1e-7;
  const l1 = a.deBoor(2, d.knots, d.points, 0.5 - e), r1 = a.deBoor(2, d.knots, d.points, 0.5 + e);
  assert.ok(Math.abs(l1[0] - r1[0]) < 1e-4 && Math.abs(l1[1] - r1[1]) < 1e-4);
  const l2 = a.deBoor(1, d2.knots, d2.points, 0.5 - e), r2 = a.deBoor(1, d2.knots, d2.points, 0.5 + e);
  assert.ok(Math.abs(l2[1] - r2[1]) > 1, `second derivative ${l2} ${r2}`);
}
