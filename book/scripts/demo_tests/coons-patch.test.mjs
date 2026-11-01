// The Coons formula reproduces its four sides exactly, and ruled surfaces alone do not.
const close = (a, b, assert) => assert.ok(Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 1e-12, `${a} vs ${b}`);

export function interpolates_all_four_sides(algo, assert) {
  const s = algo.initial();
  for (let i = 0; i <= 10; i++) {
    const t = i / 10;
    close(algo.coons(s, t, 0), algo.bez(s.c0, t), assert);
    close(algo.coons(s, t, 1), algo.bez(s.c1, t), assert);
    close(algo.coons(s, 0, t), algo.bez(s.d0, t), assert);
    close(algo.coons(s, 1, t), algo.bez(s.d1, t), assert);
  }
}
export function a_ruled_surface_misses_two_sides(algo, assert) {
  const s = algo.initial();
  const p = algo.ruledU(s, 0, 0.25), q = algo.bez(s.d0, 0.25);
  assert.ok(Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]) > 0.1);
}
export function bilinear_data_gives_the_bilinear_patch(algo, assert) {
  // Straight sides: the Coons patch is the bilinear one.
  const l = (a, b) => [a, [a[0] + (b[0] - a[0]) / 3, a[1] + (b[1] - a[1]) / 3, a[2] + (b[2] - a[2]) / 3], [a[0] + 2 * (b[0] - a[0]) / 3, a[1] + 2 * (b[1] - a[1]) / 3, a[2] + 2 * (b[2] - a[2]) / 3], b];
  const a = [0, 0, 0], b = [1, 0, 1], c = [0, 1, 2], d = [1, 1, -1];
  const s = { c0: l(a, b), c1: l(c, d), d0: l(a, c), d1: l(b, d) };
  for (const [u, v] of [[0.3, 0.7], [0.5, 0.5], [0.9, 0.1]]) close(algo.coons(s, u, v), algo.bilinear(s, u, v), assert);
}
