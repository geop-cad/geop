// Every point kept lies at the distance from the chain, and a notch narrower than
// twice the distance is passed over.
export function a_straight_line_offsets_by_the_distance(algo, assert) {
  const runs = algo.offset([[0, 0], [4, 0]], 1);
  assert.equal(runs.length, 1);
  assert.equal(JSON.stringify(runs[0]), JSON.stringify([[0, 1], [4, 1]]));
}

export function every_kept_point_is_at_the_distance_where_the_chain_is_not_ended(algo, assert) {
  const chain = [[0, 0], [3, 0], [3, 3]];
  for (const c of [chain, chain.slice().reverse()]) {
    for (const run of algo.offset(c, 0.5)) {
      for (const p of run) {
        const d = algo.chainDist(p, c);
        assert.ok(Math.abs(d - 0.5) < 1e-3, `${p} at ${d}`);
      }
    }
  }
}

export function a_turn_towards_the_offset_is_cut_at_the_crossing(algo, assert) {
  // A left turn, offset to the left: the two offsets meet at (2, 1).
  const runs = algo.offset([[0, 0], [3, 0], [3, 3]], 1);
  const pts = runs.flat();
  assert.ok(pts.some((p) => Math.hypot(p[0] - 2, p[1] - 1) < 1e-9), JSON.stringify(runs));
}

export function a_turn_away_gets_an_arc(algo, assert) {
  // A right turn, offset to the left: round the corner at radius 1.
  const runs = algo.offset([[0, 0], [3, 0], [3, -3]], 1);
  assert.equal(runs.length, 1);
  const arcPoints = runs[0].filter((p) => p[0] > 3 && p[1] > 0);
  assert.ok(arcPoints.length > 5);
  for (const p of arcPoints) assert.ok(Math.abs(Math.hypot(p[0] - 3, p[1]) - 1) < 1e-12);
}

export function a_notch_narrower_than_twice_the_distance_is_passed_over(algo, assert) {
  const chain = [[0, 3], [3.2, 3], [3.2, 1], [3.8, 1], [3.8, 3], [7, 3]];
  // Left of this chain is below it, in the notch's direction of turn: offset the reversed one.
  for (const c of [chain, chain.slice().reverse()]) {
    const runs = algo.offset(c, 0.45);
    for (const run of runs) for (let k = 0; k + 1 < run.length; k++) {
      const mid = [(run[k][0] + run[k + 1][0]) / 2, (run[k][1] + run[k + 1][1]) / 2];
      assert.ok(algo.chainDist(mid, c) > 0.45 * 0.995);
    }
  }
  // Above the chain, the offset runs straight across the notch's mouth from one side to the other.
  const above = algo.offset(chain, 0.45);
  const all = above.flat();
  assert.ok(all.some((p) => Math.abs(p[1] - 3.45) < 1e-9 && p[0] < 3));
  assert.ok(all.some((p) => Math.abs(p[1] - 3.45) < 1e-9 && p[0] > 4));
}
