// The sketch solver against analytic solutions, ranks and gradients.
const pick = (algo, keys) => algo.CATALOGUE.filter((e) => keys.includes(e.key)).map((e) => e.c);

export function solves_a_rectangle_exactly(algo, assert) {
  const x = algo.start();
  const r = algo.solve(pick(algo, ["fix", "h0", "h2", "v1", "v3", "w", "h"]), x, { size: 5 });
  assert.ok(r.residual < 1e-11, `residual ${r.residual}`);
  const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-10, `${a} vs ${b}`);
  near(x[0], 1); near(x[1], 1);
  near(x[3], 1); near(Math.abs(x[2] - 1), 3.5);
  near(x[4], x[2]); near(Math.abs(x[5] - 1), 2);
  near(x[6], 1); near(x[7], x[5]);
  // 2 fix + 4 straight + 2 lengths = 8 rows on the 8 corner coordinates; the circle's 3 are left.
  assert.equal(r.rank, 8);
  assert.equal(r.dof, 3);
}

export function an_implied_perpendicular_adds_a_row_and_no_rank(algo, assert) {
  const base = pick(algo, ["fix", "h0", "v1", "w", "h"]);
  const x = algo.start();
  const a = algo.solve(base, x, { size: 5 });
  const y = algo.start();
  const b = algo.solve(base.concat(pick(algo, ["perp"])), y, { size: 5 });
  assert.equal(b.rows, a.rows + 1);
  assert.equal(b.rank, a.rank);
  assert.equal(b.dof, a.dof);
  assert.ok(b.residual < 1e-9);
}

export function a_conflict_leaves_a_residual_and_says_so(algo, assert) {
  const x = algo.start();
  const r = algo.solve(pick(algo, ["fix", "h0", "v1", "w", "h", "diag"]), x, { size: 5 });
  assert.ok(r.conflict, `residual ${r.residual}`);
}

export function a_tangent_circle_touches_the_line(algo, assert) {
  const x = algo.start();
  const r = algo.solve(pick(algo, ["fix", "h0", "w", "r", "t"]), x, { size: 5 });
  assert.ok(r.residual < 1e-10);
  // The bottom is horizontal, so the centre's height differs from it by the radius.
  assert.ok(Math.abs(Math.abs(x[9] - x[1]) - x[10]) < 1e-9);
  assert.ok(Math.abs(x[10] - 0.7) < 1e-10);
}

export function a_pull_moves_a_free_point_to_the_target(algo, assert) {
  const x = algo.start();
  const r = algo.solve(pick(algo, ["fix", "h0"]), x, { size: 5, pulls: [{ i: 4, to: 2.5 }, { i: 5, to: 3.5 }] });
  assert.ok(Math.abs(x[4] - 2.5) < 1e-6 && Math.abs(x[5] - 3.5) < 1e-6, `${x[4]}, ${x[5]}`);
  assert.ok(r.residual < 1e-10);
}

export function gradients_agree_with_differences(algo, assert) {
  const x = algo.start();
  const cs = algo.CATALOGUE.map((e) => e.c);
  const sign = { t: 1 };
  const J = algo.evaluate(cs, x, 5, sign).J;
  const h = 1e-6;
  for (let j = 0; j < x.length; j++) {
    const p = x.slice(), q = x.slice();
    p[j] += h; q[j] -= h;
    const a = algo.evaluate(cs, p, 5, sign).c, b = algo.evaluate(cs, q, 5, sign).c;
    for (let i = 0; i < a.length; i++) {
      const fd = (a[i] - b[i]) / (2 * h);
      assert.ok(Math.abs(fd - J[i][j]) < 1e-6, `row ${i} col ${j}: ${fd} vs ${J[i][j]}`);
    }
  }
}

export function householder_solves_least_squares(algo, assert) {
  // Fit y = a + b t through (0,1), (1,3), (2,4): a = 7/6, b = 3/2.
  const y = algo.lstsq([[1, 0], [1, 1], [1, 2]], [1, 3, 4], 2);
  assert.ok(Math.abs(y[0] - 7 / 6) < 1e-12 && Math.abs(y[1] - 1.5) < 1e-12, `${y}`);
}
