// The six cases of splice_edge_into_face, each by what it must leave behind.
// The start: a 6 x 4 face (area 24), hole 0 of area 2 at x in [1,2], hole 1 of
// area 2 at x in [4,5], y in [1,3]; material 20. Vertex indices: outer 0-3 at
// (0,0) (6,0) (6,4) (0,4); hole 0 4-7 at (1,1) (1,3) (2,3) (2,1); hole 1 8-11 at
// (4,1) (4,3) (5,3) (5,1).
const close = (a, b) => Math.abs(a - b) < 1e-12;

export function neither_end_makes_a_hole(algo, assert) {
  const m = algo.start();
  const r = algo.splice(m, 0, null, null, [3, 1], [3, 3]);
  assert.match(r.kase, /hole/);
  assert.equal(m.faces.length, 1);
  assert.equal(m.faces[0].holes.length, 3);
  assert.equal(algo.ring(m, m.faces[0].holes[2]).length, 2);
  assert.ok(close(algo.totalArea(m), 20));
}
export function one_end_makes_a_spur(algo, assert) {
  const m = algo.start();
  const before = algo.ring(m, m.faces[0].outer).length;
  const r = algo.splice(m, 0, 0, null, null, [0.5, 0.5]);
  assert.match(r.kase, /spur/);
  assert.equal(m.faces.length, 1);
  assert.equal(algo.ring(m, m.faces[0].outer).length, before + 2);
  assert.ok(close(algo.totalArea(m), 20));
}
export function outer_to_outer_splits_the_face(algo, assert) {
  const m = algo.start();
  // From (3,0) would need a vertex there: add the split of the bottom side by
  // going corner to corner instead, past both holes on opposite sides.
  const r = algo.splice(m, 0, 0, 2, null, null);
  // The diagonal from (0,0) to (6,4) passes through hole 0's corners? y = 2x/3:
  // at x=1, y=.67 (below it), x=2, y=1.33 (inside hole 0): it is refused.
  assert.ok(r.refused);
  const n = algo.start();
  // Bridge along a free channel: from the top side's corner (6,4) to the bottom
  // corner (6,0) runs along a side, so it is refused too; add a new outer vertex
  // by making a face with one.
  const m2 = algo.newModel();
  m2.faces.push({ outer: -1, holes: [] });
  m2.faces[0].outer = algo.addLoop(m2, [[0, 0], [3, 0], [6, 0], [6, 4], [3, 4], [0, 4]], 0);
  m2.faces[0].holes.push(algo.addLoop(m2, [[1, 1], [1, 3], [2, 3], [2, 1]], 0));
  m2.faces[0].holes.push(algo.addLoop(m2, [[4, 1], [4, 3], [5, 3], [5, 1]], 0));
  const r2 = algo.splice(m2, 0, 1, 4, null, null);
  assert.match(r2.kase, /split into two/);
  assert.equal(m2.faces.length, 2);
  assert.ok(close(algo.faceArea(m2, 0) + algo.faceArea(m2, 1), 20));
  // Each half took the hole that lies in it.
  assert.equal(m2.faces[0].holes.length + m2.faces[1].holes.length, 2);
  assert.equal(m2.faces[0].holes.length, 1);
  assert.ok(close(algo.faceArea(m2, 0), 10));
  assert.ok(n);
}
export function two_holes_merge(algo, assert) {
  const m = algo.start();
  // (2,1) on hole 0 to (4,1) on hole 1.
  const r = algo.splice(m, 0, 7, 8, null, null);
  assert.match(r.kase, /merge/);
  assert.equal(m.faces.length, 1);
  assert.equal(m.faces[0].holes.length, 1);
  assert.equal(algo.ring(m, m.faces[0].holes[0]).length, 10);
  assert.ok(close(algo.totalArea(m), 20));
}
export function hole_absorbed_into_outer(algo, assert) {
  const m = algo.start();
  // (0,0) on the outer loop to (1,1) on hole 0.
  const r = algo.splice(m, 0, 0, 4, null, null);
  assert.match(r.kase, /absorbed/);
  assert.equal(m.faces.length, 1);
  assert.equal(m.faces[0].holes.length, 1);
  assert.equal(algo.ring(m, m.faces[0].outer).length, 10);
  assert.ok(close(algo.totalArea(m), 20));
}
export function one_hole_makes_a_new_face(algo, assert) {
  // A hole with a bite out of it: (1,1) (1,3) (3,3) (3,1) (2,2), the bite pointing
  // up from below; the chord from (1,1) to (3,1) lies in material and closes the bite.
  const m = algo.newModel();
  m.faces.push({ outer: -1, holes: [] });
  m.faces[0].outer = algo.addLoop(m, [[0, 0], [6, 0], [6, 4], [0, 4]], 0);
  m.faces[0].holes.push(algo.addLoop(m, [[1, 1], [1, 3], [3, 3], [3, 1], [2, 2]], 0));
  const r = algo.splice(m, 0, 4, 7, null, null);
  assert.match(r.kase, /new face/);
  assert.equal(m.faces.length, 2);
  // The new face is the triangle (1,1) (3,1) (2,2), of area 1, and is counter-clockwise.
  assert.ok(close(algo.signedArea(algo.ringPolygon(m, m.faces[1].outer)), 1));
  // The hole now goes round the square (1,1) (1,3) (3,3) (3,1), area 4, clockwise.
  assert.ok(close(algo.signedArea(algo.ringPolygon(m, m.faces[0].holes[0])), -4));
  assert.ok(close(algo.faceArea(m, 0), 20));
  assert.ok(close(algo.totalArea(m), 21));
}
export function coedges_stay_consistent(algo, assert) {
  const m = algo.start();
  algo.splice(m, 0, 7, 8, null, null);
  algo.splice(m, 0, 0, 4, null, null);
  m.cos.forEach((c, i) => {
    if (c.next < 0) return;
    assert.equal(m.cos[c.next].prev, i);
    assert.equal(c.b, m.cos[c.next].a);
  });
}
