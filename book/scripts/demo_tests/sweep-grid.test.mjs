// The grid built by the rules of skin.rs against the closed forms, against
// Euler-Poincare, and against "every edge used by exactly two faces".
function cases() {
  const out = [];
  for (const n of [3, 4, 5, 8]) {
    for (const hole of [false, true]) {
      for (const spans of [1, 2, 3, 4]) {
        out.push({ n, spans, closed: false, axisSide: false, hole });
        out.push({ n, spans, closed: false, axisSide: true, hole });
      }
      out.push({ n, spans: 4, closed: true, axisSide: false, hole });
      out.push({ n, spans: 4, closed: true, axisSide: true, hole });
    }
  }
  return out;
}
export function closed_forms_count_the_grid(algo, assert) {
  for (const p of cases()) {
    const g = algo.grid(p), f = algo.formula(p);
    for (const k of ["V", "E", "F", "R"]) assert.equal(g[k], f[k], `${k} of ${JSON.stringify(p)}`);
  }
}
export function every_edge_is_used_twice(algo, assert) {
  for (const p of cases()) assert.ok(algo.grid(p).manifold, JSON.stringify(p));
}
export function euler_poincare_holds(algo, assert) {
  for (const p of cases()) {
    const g = algo.grid(p);
    assert.ok(g.integral, JSON.stringify(p));
    assert.equal(g.V - g.E + g.F - g.R, 2 * (g.S - g.G), JSON.stringify(p));
  }
}
export function known_solids(algo, assert) {
  // A box: 8, 12, 6.
  const box = algo.grid({ n: 4, spans: 1, closed: false, axisSide: false, hole: false });
  assert.deepEqual([box.V, box.E, box.F, box.S, box.G], [8, 12, 6, 1, 0]);
  // A torus of a square section, a full turn clear of the axis: genus 1.
  const torus = algo.grid({ n: 4, spans: 4, closed: true, axisSide: false, hole: false });
  assert.deepEqual([torus.V, torus.E, torus.F, torus.S, torus.G], [16, 32, 16, 1, 1]);
  // A full turn of a square with a hole, clear of the axis: two shells, each a torus.
  const hollow = algo.grid({ n: 4, spans: 4, closed: true, axisSide: false, hole: true });
  assert.deepEqual([hollow.S, hollow.G], [2, 2]);
  // A side on the axis: one shell, genus 0; with a hole, a toroidal void.
  const lump = algo.grid({ n: 4, spans: 4, closed: true, axisSide: true, hole: false });
  assert.deepEqual([lump.S, lump.G], [1, 0]);
  const voided = algo.grid({ n: 4, spans: 4, closed: true, axisSide: true, hole: true });
  assert.deepEqual([voided.S, voided.G], [2, 1]);
}
