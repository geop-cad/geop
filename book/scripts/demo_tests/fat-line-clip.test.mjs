// The distance function is what it says, a clip keeps every zero, and a
// transversal crossing converges quadratically, faster than bisection.
const A = [[0.3, 0.4], [1.6, 3.9], [3.6, 3.4], [5.6, 1.2]];
const B = [[0.4, 3.2], [2.1, 0.2], [3.9, 0.9], [5.5, 3.6]];

export function distance_function_matches_the_point_on_the_curve(algo, assert) {
  const line = algo.fatLine(B), d = algo.distances(A, line);
  for (let i = 0; i <= 20; i++) {
    const t = i / 20, p = algo.evaluate(A, t);
    const direct = line.n[0] * p[0] + line.n[1] * p[1] + line.c;
    assert.ok(Math.abs(direct - algo.bernsteinValue(d, t)) < 1e-12);
  }
}
export function the_fat_line_holds_its_whole_curve(algo, assert) {
  const line = algo.fatLine(B);
  for (let i = 0; i <= 100; i++) {
    const p = algo.evaluate(B, i / 100), v = line.n[0] * p[0] + line.n[1] * p[1] + line.c;
    assert.ok(v >= line.dmin - 1e-12 && v <= line.dmax + 1e-12);
  }
}
export function both_clips_keep_every_t_in_the_band(algo, assert) {
  const line = algo.fatLine(B), d = algo.distances(A, line);
  const hull = algo.hullClip(d, line.dmin, line.dmax), chord = algo.chordClip(d, line.dmin, line.dmax);
  for (let i = 0; i <= 1000; i++) {
    const t = i / 1000, v = algo.bernsteinValue(d, t);
    if (v >= line.dmin && v <= line.dmax) {
      assert.ok(t >= hull[0] - 1e-12 && t <= hull[1] + 1e-12, `hull lost ${t}`);
      assert.ok(t >= chord[0] - 1e-12 && t <= chord[1] + 1e-12, `chord lost ${t}`);
    }
  }
  // The band around the chord contains the hull, so it is never tighter.
  assert.ok(chord[0] <= hull[0] + 1e-12 && chord[1] >= hull[1] - 1e-12);
}
export function a_clip_proves_curves_apart(algo, assert) {
  const far = B.map((p) => [p[0], p[1] + 10]);
  const line = algo.fatLine(far), d = algo.distances(A, line);
  assert.equal(algo.hullClip(d, line.dmin, line.dmax), null);
  assert.equal(algo.chordClip(d, line.dmin, line.dmax), null);
}
export function finds_both_crossings_and_they_are_on_both_curves(algo, assert) {
  for (const mode of ["hull", "chord"]) {
    const r = algo.run(A, B, mode, 1e-9, 2000);
    assert.equal(r.finished, true, mode);
    assert.equal(r.found.length, 2, `${mode}: ${r.found.length}`);
    for (const [a, b] of r.found) {
      const p = algo.evaluate(A, (a[0] + a[1]) / 2), q = algo.evaluate(B, (b[0] + b[1]) / 2);
      assert.ok(Math.hypot(p[0] - q[0], p[1] - q[1]) < 1e-7);
    }
  }
}
export function clipping_uses_far_fewer_pairs_than_bisection(algo, assert) {
  const r = algo.run(A, B, "hull", 1e-9, 2000);
  const nodes = algo.bisectionNodes(A, B, 1e-9);
  assert.ok(r.clips * 3 < nodes, `${r.clips} clips, ${nodes} pairs`);
}
export function widths_fall_faster_than_any_fixed_factor(algo, assert) {
  // One crossing: curves crossing once at a good angle.
  const a = [[0, 0], [1, 1.2], [2, 1.8], [3, 3]], b = [[0, 3], [1, 2], [2, 0.8], [3, 0]];
  const r = algo.run(a, b, "hull", 1e-12, 200);
  const widths = r.log.filter((e) => e.action === "restricted" && e.which === 0).map((e) => e.state.a[1] - e.state.a[0]);
  assert.equal(r.found.length, 1);
  // Superlinear: once small, each round's ratio is itself much smaller than the last's.
  const last = widths.length - 1;
  assert.ok(widths[last] / widths[last - 1] < 0.1, `${widths}`);
  assert.ok(last <= 8, `${widths}`);
}
