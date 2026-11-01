// The numbers Lecture 3 quotes, from an independent calculation (Python's math.fsum,
// struct-packed singles).
export function harmonic_sums_match_an_independent_run(a, assert) {
  assert.equal(a.forward(1000000, "double"), 14.392726722864989);
  assert.equal(a.backward(1000000, "double"), 14.392726722865772);
  assert.equal(a.forward(1000000, "single"), 14.3573579788208);
  assert.equal(a.backward(1000000, "single"), 14.392651557922363);
  assert.ok(Math.abs(a.compensated(1000000) - 14.392726722865724) < 1e-13);
}
export function naive_cancels_to_nothing_and_the_identity_does_not(a, assert) {
  assert.equal(a.naive(1e-8), 0);
  assert.equal(a.naive(2e-8), 0.5551115123125782);
  assert.ok(Math.abs(a.stable(1e-8) - 0.5) < 1e-15);
  assert.ok(Math.abs(a.stable(1) - (1 - Math.cos(1))) < 1e-15);
}
