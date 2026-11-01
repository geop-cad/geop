// The planar boolean against known areas, against a raster count on general
// shapes, and its keep table against every row of `keeps` in boolean.rs.
const near = (a, b, e = 1e-9) => Math.abs(a - b) <= e;
const square = (x, y, s) => [[x, y], [x + s, y], [x + s, y + s], [x, y + s]];

export function squares_overlapping(algo, assert) {
  const A = square(0, 0, 2), B = square(1, 1, 2);
  assert.ok(near(algo.boolean(A, B, "union").area, 7));
  assert.ok(near(algo.boolean(A, B, "intersection").area, 1));
  assert.ok(near(algo.boolean(A, B, "difference").area, 3));
  assert.ok(near(algo.boolean(A, B, "symmetric").area, 6));
}
export function disjoint_and_contained_empty_is_an_answer(algo, assert) {
  const A = square(0, 0, 1), far = square(5, 5, 1), big = square(-1, -1, 4);
  const inter = algo.boolean(A, far, "intersection");
  assert.equal(inter.loops.length, 0);
  assert.ok(near(inter.area, 0));
  const eaten = algo.boolean(A, big, "difference");
  assert.equal(eaten.loops.length, 0);
  // B inside A: the difference has the hole, a clockwise ring.
  const holed = algo.boolean(big, A, "difference");
  assert.equal(holed.loops.length, 2);
  assert.ok(near(holed.area, 15));
}
export function coincident_sides_are_kept_once(algo, assert) {
  // Two unit squares side by side share a side, opposed: a union drops both copies.
  const A = square(0, 0, 1), B = square(1, 0, 1);
  const u = algo.boolean(A, B, "union");
  assert.ok(near(u.area, 2));
  assert.equal(u.pieces.filter((p) => p.cls === "OnOppositeNormal").length, 2);
  assert.ok(u.pieces.filter((p) => p.cls === "OnOppositeNormal").every((p) => p.keep === "drop"));
  // Same square twice: every piece is along the other the same way, kept from a only.
  const same = algo.boolean(A, A, "union");
  assert.ok(near(same.area, 1));
  assert.equal(same.pieces.filter((p) => p.keep !== "drop").length, 4);
  assert.ok(near(algo.boolean(A, A, "difference").area, 0));
  // B's side lies along a's: a difference keeps a's copy of an opposed patch.
  assert.ok(near(algo.boolean(A, B, "difference").area, 1));
}
export function a_vertex_on_an_edge_splits_it(algo, assert) {
  const A = square(0, 0, 2), B = [[2, 1], [3, 0], [3, 2]];
  const r = algo.remesh(algo.ccw(A), algo.ccw(B), 3);
  assert.equal(r.log.onEdge, 1);
  assert.equal(r.a.length, 5);
  assert.ok(near(algo.boolean(A, B, "union").area, 5));
  assert.ok(near(algo.boolean(A, B, "intersection").area, 0));
}
function rasterArea(test, box, n = 700) {
  let count = 0;
  const [x0, y0, x1, y1] = box, dx = (x1 - x0) / n, dy = (y1 - y0) / n;
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    if (test([x0 + (i + 0.5137) * dx, y0 + (j + 0.4219) * dy])) count++;
  }
  return count * dx * dy;
}
export function concave_shapes_against_a_raster(algo, assert) {
  const A = [[0.5, 0.5], [3, 0.5], [3, 2.5], [1.75, 1.25], [0.5, 2.5]];
  const B = [[2, 1.5], [4.5, 1], [5, 3], [3, 3.5], [2, 2.5]];
  const inA = (p) => { const r = algo.pointIn(A, p); return !r.on && r.inside; };
  const inB = (p) => { const r = algo.pointIn(B, p); return !r.on && r.inside; };
  const box = [0, 0, 5.5, 4];
  const rules = {
    union: (p) => inA(p) || inB(p), intersection: (p) => inA(p) && inB(p),
    difference: (p) => inA(p) && !inB(p), symmetric: (p) => inA(p) !== inB(p)
  };
  for (const [op, rule] of Object.entries(rules)) {
    const a = algo.boolean(A, B, op).area, b = rasterArea(rule, box);
    assert.ok(near(a, b, 0.02), `${op}: ${a} vs ${b}`);
  }
}
export function the_table_is_the_kernels(algo, assert) {
  const rows = {
    union: { aOut: "keep", aIn: "drop", bOut: "keep", bIn: "drop", aSame: "keep", bSame: "drop", aOpp: "drop", bOpp: "drop" },
    intersection: { aOut: "drop", aIn: "keep", bOut: "drop", bIn: "keep", aSame: "keep", bSame: "drop", aOpp: "drop", bOpp: "drop" },
    difference: { aOut: "keep", aIn: "drop", bOut: "drop", bIn: "reversed", aSame: "drop", bSame: "drop", aOpp: "keep", bOpp: "drop" },
    symmetric: { aOut: "keep", aIn: "reversed", bOut: "keep", bIn: "reversed", aSame: "drop", bSame: "drop", aOpp: "drop", bOpp: "drop" }
  };
  const cls = { Out: "Outside", In: "Inside", Same: "OnSameNormal", Opp: "OnOppositeNormal" };
  for (const [op, row] of Object.entries(rows)) {
    for (const [k, want] of Object.entries(row)) {
      const fromA = k[0] === "a";
      assert.equal(algo.keeps(op, cls[k.slice(1)], fromA), want, `${op} ${k}`);
    }
  }
}
