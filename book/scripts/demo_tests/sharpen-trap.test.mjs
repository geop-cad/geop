// What Lecture 5 says of the three strategies, as measured.
export function newton_converges_to_the_root(a, assert) {
  const s = a.solve(0.7, "two", 20);
  assert.ok(Math.abs((s.answer[0] + s.answer[1]) / 2 - 0.7) < 1e-14);
}
export function two_copies_never_misclassifies(a, assert) {
  const v = a.survey(1000, "two", 20);
  assert.equal(v.wrong, 0);
  assert.equal(v.contains, 1000);
  assert.equal(v.failed, 0);
}
export function sharpening_the_answer_misclassifies_often(a, assert) {
  const v = a.survey(1000, "sharp", 20);
  assert.equal(v.wrong, 87);
}
export function two_copies_stay_as_wide_as_a_single_step(a, assert) {
  const s = a.solve(0.7, "two", 60).widths;
  const last = s[s.length - 1];
  assert.ok(last < 1e-14, `width ${last}`);
}
export function seeding_as_it_is_doubles_the_width_until_it_fails(a, assert) {
  const s = a.solve(0.7, "wide", 200);
  assert.ok(s.failedAt > 0, "never failed");
  const w = s.widths;
  const r = w[25] / w[24];
  assert.ok(r > 1.5 && r < 2.5, `ratio ${r}`);
}
