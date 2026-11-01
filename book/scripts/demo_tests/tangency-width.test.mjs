// The enclosure holds the exact zero set and is close to it, with the width
// the lecture states: about eps / sqrt(c) transversally, 2 sqrt(eps) at tangency.
const covers = (pieces, a, b) => pieces.some((p) => p[0] <= a + 1e-15 && b - 1e-15 <= p[1]);

export function encloses_the_exact_zero_set(algo, assert) {
  for (const c of [0.2, 0.01, 0, -0.001]) {
    for (const eps of [1e-3, 1e-6, 1e-10]) {
      const r = algo.enclose(c, eps), ex = algo.exact(c, eps);
      for (const [a, b] of ex) assert.ok(covers(r.pieces, a, b), `c=${c} eps=${eps}: ${JSON.stringify(r.pieces)} vs ${JSON.stringify(ex)}`);
      const w = r.pieces.reduce((s, p) => s + p[1] - p[0], 0), wx = ex.reduce((s, p) => s + p[1] - p[0], 0);
      assert.ok(w <= 2 * wx + 1e-8, `c=${c} eps=${eps}: width ${w} exact ${wx}`);
    }
  }
}
export function transversal_width_is_eps_over_sqrt_c(algo, assert) {
  const c = 0.25, eps = 1e-8, r = algo.enclose(c, eps);
  assert.equal(r.pieces.length, 2);
  const w = r.pieces[0][1] - r.pieces[0][0];
  // Roots at +-sqrt(c): width eps / sqrt(c) = 2e-8.
  assert.ok(Math.abs(w / 2e-8 - 1) < 0.1, `${w}`);
}
export function tangent_width_is_twice_sqrt_eps(algo, assert) {
  const eps = 1e-8, r = algo.enclose(0, eps);
  assert.equal(r.pieces.length, 1);
  const w = r.pieces[0][1] - r.pieces[0][0];
  assert.ok(Math.abs(w / 2e-4 - 1) < 0.1, `${w}`);
}
export function a_range_really_encloses(algo, assert) {
  for (const [a, b] of [[-0.5, 0.25], [0.1, 0.9], [-1, 1]]) {
    const [lo, hi] = algo.range(a, b, 0.1, 0.01);
    for (let i = 0; i <= 50; i++) {
      const t = a + (b - a) * i / 50, v = t * t - 0.1;
      assert.ok(v - 0.01 >= lo - 1e-12 && v + 0.01 <= hi + 1e-12);
    }
  }
}
export function no_crossing_is_proved_when_the_band_clears_zero(algo, assert) {
  assert.equal(algo.enclose(-0.01, 1e-6).pieces.length, 0);
}
export function newton_slows_to_halving_at_a_double_root(algo, assert) {
  const simple = algo.newtonIterations(0.25, 0.7), double = algo.newtonIterations(0, 0.7);
  assert.ok(simple < 8, `${simple}`);
  assert.ok(double > 40, `${double}`);
}
