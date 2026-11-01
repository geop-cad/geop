// The ball against a configuration with a closed form, and the tilt's claims.
export function ball_in_a_right_angle_is_r_from_both_lines(algo, assert) {
  // Straight "curves" along +x and +y from the origin: the centre is (r, r).
  const A = [[0, 0], [1, 0], [2, 0]], B = [[0, 0], [0, 1], [0, 2]], r = 0.4;
  const S = algo.setup(A, B, r), b = algo.ball(A, B, r, S.sideA, S.sideB, S.seed);
  assert.ok(Math.hypot(b.C[0] - r, b.C[1] - r) < 1e-10, `${b.C}`);
  assert.ok(Math.hypot(b.Ta[0] - r, b.Ta[1]) < 1e-10);
  assert.ok(Math.hypot(b.Tb[0], b.Tb[1] - r) < 1e-10);
}
export function ball_at_a_slanted_corner(algo, assert) {
  // Lines at 60 degrees: the centre lies r / sin(30 deg) along the bisector.
  const c = Math.cos(Math.PI / 3), s = Math.sin(Math.PI / 3), r = 0.3;
  const A = [[0, 0], [1, 0], [2, 0]], B = [[0, 0], [c, s], [2 * c, 2 * s]];
  const S = algo.setup(A, B, r), b = algo.ball(A, B, r, S.sideA, S.sideB, S.seed);
  const d = r / Math.sin(Math.PI / 6), bx = Math.cos(Math.PI / 6) * d, by = Math.sin(Math.PI / 6) * d;
  assert.ok(Math.hypot(b.C[0] - bx, b.C[1] - by) < 1e-10, `${b.C} vs ${bx},${by}`);
}
export function ball_on_curved_faces_touches_both(algo, assert) {
  const A = [[0.6, 0.5], [3.6, 0.2], [6.2, 1.2]], B = [[0.6, 0.5], [0.3, 2.2], [1.6, 3.9]], r = 0.8;
  const S = algo.setup(A, B, r), b = algo.ball(A, B, r, S.sideA, S.sideB, S.seed);
  assert.ok(b.residual < 1e-10, `residual ${b.residual}`);
  // Each contact is the foot of the centre: the radius is square to the tangent, by Newton.
  assert.ok(b.ta > 0 && b.ta < 1 && b.tb > 0 && b.tb < 1);
}
export function an_untilted_section_dips(algo, assert) {
  const f = algo.tiltProfile(0.5, 5, 6, 0, 600);
  assert.ok(algo.dips(f, 1e-13) >= 1, "an interpolant through exact stations crosses the face between them");
}
export function a_tilt_larger_than_the_error_keeps_it_on_one_side(algo, assert) {
  const base = algo.tiltProfile(0.5, 5, 6, 0, 600);
  const err = Math.max(...base.map(Math.abs));
  const f = algo.tiltProfile(0.5, 5, 6, 2 * err, 600);
  assert.equal(algo.dips(f, 0), 0);
  assert.ok(Math.min(...f) > 0);
}
export function error_falls_at_fourth_order(algo, assert) {
  const err = (n) => Math.max(...algo.tiltProfile(0.5, 5, n, 0, 2000).map(Math.abs));
  const ratio = err(24) / err(48);
  assert.ok(ratio > 10 && ratio < 30, `ratio ${ratio}`);
}
