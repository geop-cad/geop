// De Casteljau against the Bernstein sum, and the split's two pieces against
// the whole curve.
const P = [[0.5, 0.6], [1.5, 3.2], [4.2, 3.6], [5.5, 0.8], [2, -1]];
const close = (a, b, assert) => { assert.ok(Math.abs(a[0] - b[0]) < 1e-12 && Math.abs(a[1] - b[1]) < 1e-12, `${a} vs ${b}`); };

export function agrees_with_the_bernstein_sum(algo, assert) {
  for (let i = 0; i <= 20; i++) close(algo.evaluate(P, i / 20), algo.bernstein(P, i / 20), assert);
}
export function interpolates_its_end_points(algo, assert) {
  close(algo.evaluate(P, 0), P[0], assert);
  close(algo.evaluate(P, 1), P[4], assert);
}
export function split_pieces_trace_the_same_curve(algo, assert) {
  const c = 0.3, s = algo.split(P, c);
  for (let i = 0; i <= 10; i++) {
    const t = i / 10;
    close(algo.evaluate(s.left, t), algo.evaluate(P, c * t), assert);
    close(algo.evaluate(s.right, t), algo.evaluate(P, c + (1 - c) * t), assert);
  }
}
export function curve_stays_in_the_hull(algo, assert) {
  const h = algo.hull(P);
  const inside = (q) => h.every((a, i) => {
    const b = h[(i + 1) % h.length];
    return (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0]) >= -1e-12;
  });
  for (let i = 0; i <= 50; i++) assert.ok(inside(algo.evaluate(P, i / 50)));
}
