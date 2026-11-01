// The closed form against an elimination that was actually run and counted,
// and the graph analysis against what the shapes must give.
export function dense_count_matches_a_counted_elimination(algo, assert) {
  for (const m of [1, 2, 3, 5, 8, 13, 40]) {
    assert.equal(algo.denseOps(m), algo.countedElimination(m).ops, `m = ${m}`);
  }
}
export function the_counted_elimination_solves_its_system(algo, assert) {
  // The diagonally dominant system 1/(1+i+j) + (m+1)I: check the triangular form's residual.
  const m = 12, r = algo.countedElimination(m);
  for (let i = 0; i < m; i++) {
    let s = 0;
    for (let j = i; j < m; j++) s += r.A[i][j] * r.x[j];
    assert.ok(Math.abs(s - r.A[i][m]) < 1e-10);
  }
}
export function dense_is_cubic(algo, assert) {
  const ratio = algo.denseOps(2000) / algo.denseOps(1000);
  assert.ok(Math.abs(ratio - 8) < 0.1, `${ratio}`);
}
export function the_plate_splits_into_one_group_per_screw(algo, assert) {
  const g = algo.groups(algo.assembly(100, "plate", 6));
  assert.equal(g.length, 100);
  assert.ok(g.every((s) => s === 1));
}
export function the_robot_is_one_group(algo, assert) {
  const a = algo.assembly(100, "robot", 6);
  assert.equal(a.bodies.length, 112);
  assert.equal(JSON.stringify(algo.groups(a)), "[112]");
}
export function the_robot_peels_down_to_its_links(algo, assert) {
  const p = algo.pendants(algo.assembly(100, "robot", 6), (b) => b[0] === "l");
  assert.equal(p.core, 6);
  assert.equal(p.pendants, 106);
}
export function grouping_is_free_on_the_plate_and_useless_on_the_robot(algo, assert) {
  const plate = algo.costs(500, "plate", 6), robot = algo.costs(500, "robot", 6);
  assert.equal(plate.grouped, plate.pendant);
  assert.equal(robot.grouped, robot.whole);
  assert.ok(robot.pendant < robot.whole / 1000);
}
