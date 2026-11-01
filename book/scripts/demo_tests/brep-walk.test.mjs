// The half-edge structure's invariants, on each body the demo offers.
const bodies = (algo) => [algo.cube(), algo.prism(), algo.drilled()];

export function next_cycles_close_and_prev_inverts_next(algo, assert) {
  for (const m of bodies(algo)) {
    m.coedges.forEach((c, i) => {
      assert.equal(m.coedges[c.next].prev, i);
      assert.equal(m.coedges[c.prev].next, i);
      assert.equal(m.coedges[c.next].face, c.face);
      assert.ok(algo.loopOf(m, i).length >= 3);
    });
  }
}
export function a_coedge_ends_where_the_next_starts(algo, assert) {
  for (const m of bodies(algo)) {
    m.coedges.forEach((c, i) => assert.equal(algo.endVertex(m, i), algo.startVertex(m, c.next)));
  }
}
export function opposite_is_an_involution_with_opposed_sense(algo, assert) {
  for (const m of bodies(algo)) {
    m.coedges.forEach((c, i) => {
      const o = algo.opposite(m, i);
      assert.notEqual(o, i);
      assert.equal(algo.opposite(m, o), i);
      assert.notEqual(m.coedges[o].sense, c.sense);
      assert.notEqual(m.coedges[o].face, c.face);
    });
    assert.equal(m.coedges.length, 2 * m.edges.length);
  }
}
export function euler_counts(algo, assert) {
  const c = algo.euler(algo.cube());
  assert.deepEqual([c.V, c.E, c.F, c.L, c.chi], [8, 12, 6, 0, 2]);
  const p = algo.euler(algo.prism());
  assert.deepEqual([p.V, p.E, p.F, p.L, p.chi], [6, 9, 5, 0, 2]);
  const d = algo.euler(algo.drilled());
  assert.deepEqual([d.V, d.E, d.F, d.L, d.chi], [16, 24, 10, 2, 0]);
}
export function normals_point_outward(algo, assert) {
  // Each body is centred on the origin and every face plane misses it, so an
  // outward normal has a positive dot with a point of the face... except the
  // bore's faces, whose normals point into the bore, towards the axis.
  for (const m of bodies(algo)) {
    m.faces.forEach((f, i) => {
      const n = algo.faceNormal(m, i);
      const p = m.points[algo.startVertex(m, f.outer)];
      const dot = n[0] * p[0] + n[1] * p[1] + n[2] * p[2];
      if (f.name.startsWith("bore")) assert.ok(dot < 0, f.name);
      else assert.ok(dot > 0, f.name);
    });
  }
}
