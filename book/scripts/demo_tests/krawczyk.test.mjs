// Interval arithmetic encloses, and K(X) keeps every root X holds; proofs come
// out where they should and the tangency is never proven.
const R = (() => { let s = 12345; return () => { s = (s * 1664525 + 1013904223) % 4294967296; return s / 4294967296; }; })();

export function interval_operations_enclose_their_points(algo, assert) {
  for (let k = 0; k < 2000; k++) {
    const a = [R() * 4 - 2, 0], b = [R() * 4 - 2, 0];
    a[1] = a[0] + R(); b[1] = b[0] + R();
    const s = a[0] + R() * (a[1] - a[0]), t = b[0] + R() * (b[1] - b[0]);
    for (const [op, v] of [[algo.iadd, s + t], [algo.isub, s - t], [algo.imul, s * t]]) {
      const r = op(a, b);
      assert.ok(r[0] <= v && v <= r[1], `${v} not in ${r}`);
    }
  }
}

export function any_root_in_the_box_stays_in_K(algo, assert) {
  let tested = 0;
  for (let k = 0; k < 3000; k++) {
    const m = R() * 1.6 - 0.8, h = R() * 1.6 - 0.8;
    const c = [R() * 3 - 1.5, R() * 3 - 1.5], r = 0.02 + R();
    const t = algo.krawczyk(c, [r, r], m, h);
    if (!t.invertible) continue;
    for (const p of algo.roots(m, h)) {
      if (Math.abs(p[0] - c[0]) <= r && Math.abs(p[1] - c[1]) <= r) {
        tested++;
        assert.ok(t.K[0][0] <= p[0] + 1e-12 && p[0] - 1e-12 <= t.K[0][1], `x root ${p} not in K ${t.K}`);
        assert.ok(t.K[1][0] <= p[1] + 1e-12 && p[1] - 1e-12 <= t.K[1][1], `y root ${p} not in K ${t.K}`);
      }
    }
  }
  assert.ok(tested > 100, `only ${tested} roots tested`);
}

export function a_transversal_crossing_is_proven_and_the_box_holds_it(algo, assert) {
  const c = algo.polish([0.9, 0.5], 0, 0.6, 8);
  assert.ok(Math.abs(c[0] - 0.8) < 1e-14 && Math.abs(c[1] - 0.6) < 1e-14);
  const w = algo.widen(c, 0, 0.6, 40);
  assert.ok(w.radius !== null && w.radius < 1e-12, `${w.radius}`);
  assert.equal(w.test.verdict, "proven");
  assert.ok(w.test.K[0][0] <= 0.8 && 0.8 <= w.test.K[0][1]);
}

export function a_box_with_no_root_can_be_proven_empty(algo, assert) {
  const t = algo.krawczyk([1.5, 1.5], [0.05, 0.05], 0, 0.6);
  assert.equal(t.verdict, "none");
}

export function the_tangent_case_is_never_proven(algo, assert) {
  for (const x of [0.3, 1e-4, 1e-6, 1e-8]) {
    const w = algo.widen([x, 1], 0, 1, 45);
    if (w.radius !== null) assert.notEqual(w.test.verdict, "proven", `proved at ${x}`);
    for (let r = 1e-12; r < 1; r *= 4) {
      const t = algo.krawczyk([x, 1], [r, r], 0, 1);
      assert.notEqual(t.verdict, "proven", `proved at ${x}, radius ${r}`);
    }
  }
}
