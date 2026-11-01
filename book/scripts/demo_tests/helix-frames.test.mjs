// Double reflection against the closed forms: on a helix the rotation-minimizing
// frame turns against the screw frame by theta sin(alpha), the torsion's integral;
// on a planar curve it stays the plane's normal; each step keeps square and unit.
export function twist_is_the_torsion_integral(algo, assert) {
  for (const [r, pitch] of [[1, 1], [1, 1.2], [0.5, 4], [2, 0.3]]) {
    const theta = 4 * Math.PI;
    const tw = algo.twistAgainstScrew(r, pitch, theta, 200);
    const a = algo.analyticTwist(r, pitch, theta);
    assert.ok(Math.abs(Math.abs(tw[tw.length - 1]) - a) < 1e-5, `${tw[tw.length - 1]} vs ${a}`);
  }
}
export function converges_at_fourth_order(algo, assert) {
  const theta = 2 * Math.PI, a = algo.analyticTwist(1, 3, theta);
  const err = (n) => { const t = algo.twistAgainstScrew(1, 3, theta, n); return Math.abs(Math.abs(t[n]) - a); };
  const ratio = err(10) / err(20);
  assert.ok(ratio > 10 && ratio < 22, `ratio ${ratio}`);
}
export function stays_square_to_the_tangent_and_unit(algo, assert) {
  const h = algo.helix(1, 1.5), steps = 60, theta = 6 * Math.PI;
  const vs = algo.carried(h, 0, theta, steps, [1, 0, 0]);
  vs.forEach((v, i) => {
    const T = algo.unit(h.d1(theta * i / steps));
    assert.ok(Math.abs(algo.dot(v, T)) < 1e-9, `step ${i}: ${algo.dot(v, T)}`);
    assert.ok(Math.abs(algo.dot(v, v) - 1) < 1e-9);
  });
}
export function planar_curve_keeps_the_plane_normal(algo, assert) {
  const s = algo.sCurve(0.6);
  const vs = algo.carried(s, -1.6, 1.6, 40, [0, 0, 1]);
  vs.forEach((v) => assert.ok(Math.abs(v[2] - 1) < 1e-12 && Math.abs(v[0]) < 1e-12 && Math.abs(v[1]) < 1e-12));
}
export function frenet_flips_at_the_inflection(algo, assert) {
  const s = algo.sCurve(0.6);
  const before = algo.frenet(s, -0.01), after = algo.frenet(s, 0.01);
  assert.ok(algo.dot(before.n, after.n) < -0.99, "the normal turned round");
  assert.equal(algo.frenet(s, 0), null);
}
export function screw_frame_is_invariant(algo, assert) {
  // The screw motion: turn by phi about z and rise c phi takes the helix to itself and
  // the radial direction at t to that at t + phi.
  const r = 1.3, pitch = 2, c = pitch / (2 * Math.PI), phi = 0.9, h = algo.helix(r, pitch);
  const turn = (p) => [Math.cos(phi) * p[0] - Math.sin(phi) * p[1], Math.sin(phi) * p[0] + Math.cos(phi) * p[1], p[2]];
  for (const t of [0, 0.3, 2, 5]) {
    const moved = turn(h.x(t));
    moved[2] += c * phi;
    const direct = h.x(t + phi);
    assert.ok(Math.hypot(moved[0] - direct[0], moved[1] - direct[1], moved[2] - direct[2]) < 1e-12);
    const v = turn(algo.screwVector(r, pitch, t)), w = algo.screwVector(r, pitch, t + phi);
    assert.ok(Math.hypot(v[0] - w[0], v[1] - w[1], v[2] - w[2]) < 1e-12);
  }
}
