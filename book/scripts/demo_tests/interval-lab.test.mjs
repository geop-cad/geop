// Enclosure, the worked numbers of Lecture 4, and the orders of convergence.
const within = (v, w, tol = 1e-12) => Math.abs(v - w) < tol;

export function every_form_encloses_the_true_range(a, assert) {
  for (const k of Object.keys(a.fns)) {
    const fn = a.fns[k];
    for (let i = 0; i < 400; i++) {
      const t = (i * 0.6180339887) % 1, s = (i * 0.7548776662) % 1;
      const c = fn.dom[0] + t * (fn.dom[1] - fn.dom[0]);
      const h = s * (fn.dom[1] - fn.dom[0]) / 4;
      const X = [Math.max(fn.dom[0], c - h), Math.min(fn.dom[1], c + h)];
      const T = a.trueRange(fn, X);
      for (const Y of [a.naive(fn, X), a.meanValue(fn, X), a.hull(a.pieces(fn, X, 5))]) {
        assert.ok(Y[0] <= T[0] + 1e-15 && Y[1] >= T[1] - 1e-15, `${k} ${X}: ${Y} misses ${T}`);
      }
    }
  }
}
export function worked_example_of_the_lecture(a, assert) {
  const fn = a.fns.parabola, X = [0.4, 0.6];
  const n = a.naive(fn, X), m = a.meanValue(fn, X), t = a.trueRange(fn, X);
  assert.ok(within(n[0], 0.16) && within(n[1], 0.36));
  assert.ok(within(m[0], 0.23) && within(m[1], 0.27));
  assert.ok(within(t[0], 0.24) && within(t[1], 0.25));
  const two = a.hull(a.pieces(fn, X, 2));
  assert.ok(within(two[0], 0.2) && within(two[1], 0.3));
  const whole = a.naive(fn, [0, 1]);
  assert.ok(within(whole[0], 0) && within(whole[1], 1));
}
export function x_minus_x_is_as_wide_as_twice_x(a, assert) {
  const Y = a.naive(a.fns.self, [1, 2]);
  assert.ok(within(Y[0], -1) && within(Y[1], 1));
}
export function naive_excess_is_linear_and_mean_value_quadratic(a, assert) {
  const fn = a.fns.parabola;
  const excess = (form, h) => {
    const X = [0.3 - h, 0.3 + h], Y = form(fn, X), T = a.trueRange(fn, X);
    return (Y[1] - Y[0]) - (T[1] - T[0]);
  };
  const rn = excess(a.naive, 0.02) / excess(a.naive, 0.01);
  const rm = excess(a.meanValue, 0.02) / excess(a.meanValue, 0.01);
  assert.ok(rn > 1.8 && rn < 2.2, `naive ratio ${rn}`);
  assert.ok(rm > 3.6 && rm < 4.4, `mean value ratio ${rm}`);
}
export function rounding_is_one_ulp_outward_like_the_scalar(a, assert) {
  const s = a.add([0.1, 0.1], [0.2, 0.2]);
  assert.equal(s[0], 0.3);
  assert.equal(s[1], 0.3000000000000001);
  const z = a.sub([0.7, 0.7], [0.7, 0.7]);
  assert.equal(z[0], -5e-324);
  assert.equal(z[1], 5e-324);
}
