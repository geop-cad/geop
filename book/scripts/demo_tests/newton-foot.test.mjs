// On a circle the steps have closed forms (Lecture 14):
// Gauss-Newton: e' = e - (rho/R) sin e. Newton: e' = e - tan e.
const R = 1, rho = 3; // target at distance rho from the centre: d = rho - R = 2R outside.

export function gauss_newton_step_has_its_closed_form(algo, assert) {
  const c = algo.circle(R), T = [rho, 0];
  for (const t of [0.1, -0.4, 1.0]) {
    const e = -t; // theta = 0, e = theta - t
    assert.ok(Math.abs(algo.gaussNewtonStep(c, T, t) - (rho / R) * Math.sin(e)) < 1e-12);
  }
}
export function newton_step_has_its_closed_form(algo, assert) {
  const c = algo.circle(R), T = [rho, 0];
  for (const t of [0.1, -0.4, 1.0]) {
    const s = algo.newtonStep(c, T, t);
    assert.equal(s.newton, true);
    assert.ok(Math.abs(s.step - Math.tan(-t)) < 1e-12);
  }
}
export function gauss_newton_swings_for_ever_twice_the_radius_off(algo, assert) {
  const ts = algo.iterateGN(algo.circle(R), [rho, 0], 0.2, 400);
  // Settled into a 2-cycle e -> -e with 2 e = 3 sin e, e about 1.496.
  const a = ts[398], b = ts[399];
  assert.ok(Math.abs(algo.wrap(a) + algo.wrap(b)) < 1e-9, `${a} ${b}`);
  const e = Math.abs(algo.wrap(a));
  assert.ok(Math.abs(2 * e - 3 * Math.sin(e)) < 1e-9, `${e}`);
  assert.ok(e > 1.4);
}
export function error_multiplier_is_minus_d_over_r(algo, assert) {
  // Near the foot point, for a target d = 0.5 R outside: error times -0.5 each step.
  const ts = algo.iterateGN(algo.circle(R), [1.5, 0], 1e-3, 3);
  assert.ok(Math.abs(ts[1] / ts[0] + 0.5) < 1e-5);
  assert.ok(Math.abs(ts[2] / ts[1] + 0.5) < 1e-5);
}
export function newton_converges_cubically_on_the_circle(algo, assert) {
  const n = algo.iterateNewton(algo.circle(R), [rho, 0], 0.5, 4).ts;
  // e' = e - tan e = -e^3 / 3 + ...
  assert.ok(Math.abs(n[1] / Math.pow(n[0], 3)) < 0.5);
  assert.ok(Math.abs(Math.abs(n[2] / Math.pow(n[1], 3)) - 1 / 3) < 0.05, `${n}`);
}
export function newton_converges_quadratically_on_an_ellipse(algo, assert) {
  const c = algo.ellipse(2, 1), T = [2.5, 1.2], star = algo.truth(c, T);
  const n = algo.iterateNewton(c, T, star + 0.3, 6).ts.map((t) => Math.abs(algo.wrap(t - star)));
  // The ratio e_{k+1} / e_k^2 stays bounded while digits double.
  assert.ok(n[2] < n[1] * n[1] * 5 && n[3] < n[2] * n[2] * 5, `${n}`);
  assert.ok(n[4] < 1e-12, `${n}`);
}
export function truth_is_a_foot_point(algo, assert) {
  const c = algo.ellipse(2, 1), T = [2.5, 1.2], t = algo.truth(c, T), p = c.c(t), d = c.d(t);
  assert.ok(Math.abs((T[0] - p[0]) * d[0] + (T[1] - p[1]) * d[1]) < 1e-10);
}
