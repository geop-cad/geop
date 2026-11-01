// The bend's formulas against the geometry of its section.
export function a_quarter_bend_as_in_the_lecture(algo, assert) {
  const ba = algo.bendAllowance(Math.PI / 2, 2, 1, 0.44);
  assert.ok(Math.abs(ba - 3.8327) < 1e-3, `BA ${ba}`);
  assert.ok(Math.abs(algo.outsideSetback(Math.PI / 2, 2, 1) - 3) < 1e-12);
  assert.ok(Math.abs(algo.bendDeduction(Math.PI / 2, 2, 1, 0.44) - (6 - ba)) < 1e-12);
}
export function the_neutral_section_is_as_long_as_the_flat(algo, assert) {
  for (const [angle, r, T, K] of [[1.57, 1, 1, 0.44], [2.5, 0.3, 0.7, 0.3], [0.4, 2, 0.2, 0.5]]) {
    const flatA = 3, flatB = 2;
    const neutral = algo.section(angle, r, K * T, flatA, flatB, 4000);
    const want = flatA + flatB + algo.bendAllowance(angle, r, T, K);
    assert.ok(Math.abs(algo.polyLength(neutral) - want) < 1e-5);
  }
}
export function the_inside_is_shorter_and_the_outside_longer(algo, assert) {
  const inner = algo.polyLength(algo.section(1.2, 1, 0, 2, 2, 2000));
  const outer = algo.polyLength(algo.section(1.2, 1, 1, 2, 2, 2000));
  assert.ok(Math.abs(inner - (4 + 1.2 * 1)) < 1e-5);
  assert.ok(Math.abs(outer - (4 + 1.2 * 2)) < 1e-5);
}
export function outer_sharp_lengths_meet_at_the_setback(algo, assert) {
  // The lines of the outsides of the two flanges cross at the outer sharp corner.
  const angle = 1.1, r = 0.8, T = 0.5;
  const o = algo.section(angle, r, T, 1, 1, 200);
  const a0 = o[0], a1 = o[1], b0 = o[o.length - 2], b1 = o[o.length - 1];
  const db = [b1[0] - b0[0], b1[1] - b0[1]];
  // First line is y = -T: find b's parameter there.
  const s = (a0[1] - b0[1]) / db[1];
  const x = b0[0] + s * db[0];
  assert.ok(Math.abs(x - a1[0] - algo.outsideSetback(angle, r, T)) < 1e-9);
}
export function flat_length_of_a_flange_ninety_degrees(algo, assert) {
  // 100 + 50 outer sharp, 2 thick, radius 2, K 0.44.
  const L = algo.flatLength(100, 50, Math.PI / 2, 2, 2, 0.44);
  const want = 150 - (2 * 4 - (Math.PI / 2) * 2.88);
  assert.ok(Math.abs(L - want) < 1e-12);
}
