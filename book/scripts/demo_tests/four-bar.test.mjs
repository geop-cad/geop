// The four-bar: rank and freedom where they are known, the dead point singular,
// the pull obeying the link lengths, and Jacobians against differences.
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

export function a_regular_four_bar_has_rank_8_and_one_freedom(algo, assert) {
  const s = algo.PRESETS.crank, m = algo.mechanism(...s.L, s.crank, false);
  const r = algo.solve(m.pins, m.x, null);
  assert.equal(r.vars, 9); assert.equal(r.rows, 8); assert.equal(r.rank, 8); assert.equal(r.freedom, 1);
  assert.ok(r.residual < 1e-12);
}

export function at_the_rockers_limit_the_rank_stays_and_the_rocker_stands_still(algo, assert) {
  const [L1, L2, L3, L4] = algo.PRESETS.crank.L;
  const m = algo.mechanism(L1, L2, L3, L4, algo.deadCrank(L1, L2, L3, L4), false);
  const e = algo.evaluate(m.pins, m.x);
  assert.ok(e.c.every((v) => Math.abs(v) < 1e-7), `pins hold ${e.c}`);
  const r = algo.solve(m.pins, m.x, null, { iterations: 3 });
  assert.equal(r.rank, 8);
  assert.ok(Math.abs(r.motion[8] / r.motion[2]) < 1e-6, `rate ${r.motion[8] / r.motion[2]}`);
  // A and C with B on the segment between them: crank and coupler in line.
  const A = algo.placed(m.x, 0, [0, 0]).v, C = algo.placed(m.x, 1, [L2, 0]).v;
  assert.ok(Math.abs(dist(A, C) - (L1 + L2)) < 1e-7);
}

export function a_flat_parallelogram_loses_rank_and_a_turned_one_does_not(algo, assert) {
  const [L1, L2, L3, L4] = algo.PRESETS.para.L;
  const flat = algo.mechanism(L1, L2, L3, L4, 0, false);
  assert.equal(algo.rankInfo(algo.evaluate(flat.pins, flat.x).J, 9).rank, 7);
  const turned = algo.mechanism(L1, L2, L3, L4, 0.4, false);
  assert.equal(algo.rankInfo(algo.evaluate(turned.pins, turned.x).J, 9).rank, 8);
}

export function a_third_parallel_crank_is_redundant(algo, assert) {
  const s = algo.PRESETS.extra, m = algo.mechanism(...s.L, s.crank, true);
  const r = algo.solve(m.pins, m.x, null);
  assert.equal(r.vars, 12); assert.equal(r.rows, 12);
  assert.equal(r.rank, 11);
  assert.equal(r.freedom, 1);
  assert.ok(r.residual < 1e-10, `${r.residual}`);
}

export function dragging_keeps_every_link_its_length(algo, assert) {
  const s = algo.PRESETS.crank, m = algo.mechanism(...s.L, s.crank, false);
  for (const to of [[0.8, 0.8], [-1, 0.2], [0, -1], [1, -0.5]]) {
    const r = algo.solve(m.pins, m.x, { b: 0, p: [s.L[0], 0], to }, { iterations: 60 });
    assert.ok(r.residual < 1e-9, `pins off by ${r.residual}`);
    const A = algo.placed(m.x, 0, [0, 0]).v, B = algo.placed(m.x, 0, [s.L[0], 0]).v;
    const C = algo.placed(m.x, 1, [s.L[1], 0]).v, D = algo.placed(m.x, 2, [0, 0]).v;
    assert.ok(Math.abs(dist(A, B) - s.L[0]) < 1e-12);
    assert.ok(Math.abs(dist(B, C) - s.L[1]) < 1e-8);
    assert.ok(Math.abs(dist(C, D) - s.L[2]) < 1e-8);
    assert.ok(dist(A, [0, 0]) < 1e-8 && dist(D, [s.L[3], 0]) < 1e-8);
  }
}

export function jacobian_agrees_with_differences(algo, assert) {
  const s = algo.PRESETS.crank, m = algo.mechanism(...s.L, s.crank, false);
  const J = algo.evaluate(m.pins, m.x).J, h = 1e-6;
  for (let j = 0; j < m.x.length; j++) {
    const p = m.x.slice(), q = m.x.slice();
    p[j] += h; q[j] -= h;
    const a = algo.evaluate(m.pins, p).c, b = algo.evaluate(m.pins, q).c;
    for (let i = 0; i < a.length; i++) assert.ok(Math.abs((a[i] - b[i]) / (2 * h) - J[i][j]) < 1e-7);
  }
}
