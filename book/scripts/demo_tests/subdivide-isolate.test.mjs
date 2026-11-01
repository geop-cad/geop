// The search keeps every true crossing, separates transversal ones, rejects
// curves that are apart; the leaf counts grow as Lecture 12 says they do.
function run(algo, name, h) {
  const [a, b] = algo.preset(name);
  const r = algo.search(a, b, h, 2000000);
  return { a, b, r, cs: algo.clusters(r.leaves) };
}

export function three_crossings_are_isolated(algo, assert) {
  const { a, b, cs, r } = run(algo, "three crossings", 0.002);
  assert.equal(r.exhausted, false);
  assert.equal(cs.length, 3);
  for (const c of cs) {
    const pa = algo.evaluate(a, (c.s0 + c.s1) / 2), pb = algo.evaluate(b, (c.t0 + c.t1) / 2);
    assert.ok(Math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < 0.02);
  }
}
export function every_sign_change_lies_in_a_candidate(algo, assert) {
  // The second curve of this scene is nearly level: its height and the first's
  // cross at sign changes of y_a(s) - y_b(t) with x_a(s) = x_b(t). Walk along
  // the first curve, find crossings by brute force, check each is covered.
  const { a, b, cs } = run(algo, "three crossings", 0.01);
  const dist = (s) => {
    const p = algo.evaluate(a, s);
    let best = null;
    for (let t = 0; t <= 1; t += 1 / 2000) {
      const q = algo.evaluate(b, t);
      const d = Math.abs(p[0] - q[0]);
      if (!best || d < best.d) best = { d, y: p[1] - q[1] };
    }
    return best.y;
  };
  let prev = dist(0), n = 0;
  for (let k = 1; k <= 400; k++) {
    const s = k / 400, cur = dist(s);
    if (prev * cur < 0) {
      n++;
      assert.ok(cs.some((c) => c.s0 <= s + 1 / 400 && s - 1 / 400 <= c.s1), `a crossing at s=${s} has no candidate`);
    }
    prev = cur;
  }
  assert.equal(n, 3);
}
export function curves_apart_are_rejected_with_no_leaves(algo, assert) {
  const { r } = run(algo, "apart", 0.01);
  assert.equal(r.leaves.length, 0);
  assert.ok(r.rejected > 0);
}
export function a_halve_is_the_same_curve(algo, assert) {
  const { a } = run(algo, "apart", 0.1);
  const [l, r] = algo.halve(a);
  for (let i = 0; i <= 10; i++) {
    const t = i / 10;
    const p = algo.evaluate(l, t), q = algo.evaluate(a, t / 2);
    assert.ok(Math.hypot(p[0] - q[0], p[1] - q[1]) < 1e-12);
    const u = algo.evaluate(r, t), v = algo.evaluate(a, 0.5 + t / 2);
    assert.ok(Math.hypot(u[0] - v[0], u[1] - v[1]) < 1e-12);
  }
}
export function leaf_counts_scale_as_the_lecture_says(algo, assert) {
  // Halving the handoff repeatedly: a transversal crossing keeps a bounded
  // leaf count, a tangency grows by about sqrt(2) per halving (four times the
  // leaves in sixteen times finer), an overlap about twice per halving.
  const growth = (name) => {
    const n = (h) => run(algo, name, h).r.leaves.length;
    return n(0.04 / 16) / n(0.04);
  };
  assert.ok(growth("three crossings") < 2.5, `transversal grew ${growth("three crossings")}`);
  const tangent = growth("tangent");
  assert.ok(tangent > 2 && tangent < 6, `tangent grew ${tangent}`);
  const overlap = growth("overlapping");
  assert.ok(overlap > 10 && overlap < 24, `overlap grew ${overlap}`);
}
