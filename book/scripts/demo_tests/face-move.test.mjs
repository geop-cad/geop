// The 2-D rebuild: corners at the meeting of the lines, refusal exactly where a side
// would turn around.
const square = [[0, 0], [4, 0], [4, 2], [0, 2]];
export function unmoved_is_unchanged(algo, assert) {
  const r = algo.rebuild(square, algo.linesOf(square));
  assert.equal(r.refused, undefined);
  r.pts.forEach((p, i) => assert.ok(Math.hypot(p[0] - square[i][0], p[1] - square[i][1]) < 1e-12));
}
export function moving_a_side_moves_only_its_corners(algo, assert) {
  const lines = algo.linesOf(square);
  lines[1] = algo.moved(lines[1], 1);
  const r = algo.rebuild(square, lines);
  [[0, 0], [5, 0], [5, 2], [0, 2]].forEach((q, i) => assert.ok(Math.hypot(r.pts[i][0] - q[0], r.pts[i][1] - q[1]) < 1e-12));
}
export function a_side_moved_past_its_opposite_is_refused(algo, assert) {
  const lines = algo.linesOf(square);
  lines[1] = algo.moved(lines[1], -3.999);
  assert.equal(algo.rebuild(square, lines).refused, undefined);
  const again = algo.linesOf(square);
  again[1] = algo.moved(again[1], -4.001);
  assert.match(algo.rebuild(square, again).refused, /e0|e2/);
}
export function parallel_neighbours_do_not_meet(algo, assert) {
  const lines = algo.linesOf(square);
  lines[1] = algo.turned(lines[1], [4, 1], Math.PI / 2);
  assert.match(algo.rebuild(square, lines).refused, /no longer meet/);
}
export function inner_copy_collapses_at_half_the_width(algo, assert) {
  assert.equal(algo.inner(square, 0.999).refused, undefined);
  assert.ok(algo.inner(square, 1.001).refused);
  const r = algo.inner(square, 0.5).pts;
  assert.ok(Math.hypot(r[0][0] - 0.5, r[0][1] - 0.5) < 1e-12);
}
export function a_chamfer_narrower_than_the_walls_turns_around(algo, assert) {
  // The chamfer's inner copy runs backwards once its neighbours' copies meet before it.
  const r = algo.inner(algo.SHAPES.chamfered, 0.35);
  assert.equal(r.refused, undefined);
  const big = algo.inner(algo.SHAPES.chamfered, 1.2);
  assert.match(big.refused, /e1/);
}
