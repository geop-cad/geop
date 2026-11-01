// Exact sign against the determinant worked out by hand, and the counts Lecture 3
// quotes, checked independently with Python fractions.
const P = [0.5, 0.5], Q = [12, 12], R = [24, 24];

export function exact_sign_of_clear_cases(a, assert) {
  assert.equal(a.orientExact([0, 0], [1, 0], [0, 1]), 1);
  assert.equal(a.orientExact([0, 0], [0, 1], [1, 0]), -1);
  assert.equal(a.orientExact([0, 0], [1, 1], [2, 2]), 0);
  assert.equal(a.orientExact(P, Q, R), 0);
}
export function float_is_wrong_as_often_as_measured(a, assert) {
  const f = a.grid(P, Q, R, 16, a.orientFloat), e = a.grid(P, Q, R, 16, a.orientExact);
  let wrong = 0;
  for (let j = 0; j < 16; j++) for (let i = 0; i < 16; i++) if (f[j][i] !== e[j][i]) wrong++;
  assert.equal(wrong, 114);
}
export function interval_verdict_never_contradicts_the_exact_sign(a, assert) {
  const v = a.grid(P, Q, R, 64, a.orientInterval), e = a.grid(P, Q, R, 64, a.orientExact);
  for (let j = 0; j < 64; j++) for (let i = 0; i < 64; i++) assert.ok(v[j][i] === 0 || v[j][i] === e[j][i]);
}
export function interval_verdict_decides_far_from_the_line(a, assert) {
  assert.equal(a.orientInterval([0.5, 0.6], Q, R), a.orientExact([0.5, 0.6], Q, R));
  assert.notEqual(a.orientInterval([0.5, 0.6], Q, R), 0);
}
