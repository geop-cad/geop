// Catmull-Clark's rules against what they must reproduce: on a regular grid, the
// refinement of the uniform bicubic B-spline, and its limit surface.
const close = (a, b, assert, what = "") => {
  for (let k = 0; k < 3; k++) assert.ok(Math.abs(a[k] - b[k]) < 1e-11, `${what} ${a} vs ${b}`);
};
const NU = 7, NV = 6;
const at = (i, j) => [Math.sin(i * 1.3) + 0.2 * i, Math.cos(j * 2.1) * (1 + 0.1 * i), 0.3 * i * j - j * j * 0.1];
const wrap = (x, n) => ((x % n) + n) % n;
const P = (i, j) => at(wrap(i, NU), wrap(j, NV));
// Σ a_r b_s P(i + r, j + s).
function tensor(i, j, a, b) {
  let s = [0, 0, 0];
  a.forEach((ar, r) => b.forEach((bs, q) => {
    const p = P(i + r, j + q);
    s = s.map((x, k) => x + ar * bs * p[k]);
  }));
  return s;
}
const VERTEX = [1 / 8, 6 / 8, 1 / 8], MID = [1 / 2, 1 / 2];

export function one_step_of_a_regular_grid_is_bicubic_refinement(algo, assert) {
  const mesh = algo.torus(NU, NV, at);
  const once = algo.subdivide(mesh);
  // Vertex points: (1 6 1)/8 each way, centred.
  for (let i = 0; i < NU; i++) for (let j = 0; j < NV; j++) {
    close(once.v[i * NV + j], tensor(i - 1, j - 1, VERTEX, VERTEX), assert, "vertex");
  }
  // Quad k = 0 of face (i, j) is [v, edge after, face point, edge before].
  mesh.f.forEach((face, f) => {
    const i = Math.floor(f / NV), j = f % NV, quad = once.f[4 * f];
    close(once.v[quad[1]], tensor(i, j - 1, MID, VERTEX), assert, "edge along i");
    close(once.v[quad[2]], tensor(i, j, MID, MID), assert, "face");
    close(once.v[quad[3]], tensor(i - 1, j, VERTEX, MID), assert, "edge along j");
  });
}
export function the_kernels_patches_on_a_regular_grid_are_the_bicubic_b_spline(algo, assert) {
  const mesh = algo.torus(NU, NV, at);
  const once = algo.subdivide(mesh);
  const net = algo.controlNet(once);
  // The limit corner at a cage vertex is the spline at that knot: (1 4 1)/6 each way.
  const L = [1 / 6, 4 / 6, 1 / 6];
  for (let i = 0; i < NU; i++) for (let j = 0; j < NV; j++) {
    close(net.corner[i * NV + j], tensor(i - 1, j - 1, L, L), assert, "limit");
  }
  // Inside a cage face the spline over u, v in [0, 1]: the cubic B-spline basis.
  const b = (t) => [(1 - t) ** 3 / 6, (3 * t ** 3 - 6 * t ** 2 + 4) / 6, (-3 * t ** 3 + 3 * t ** 2 + 3 * t + 1) / 6, t ** 3 / 6];
  for (const [fi, fj] of [[0, 0], [3, 2], [6, 5]]) {
    const f = fi * NV + fj;
    for (const [s, t] of [[0.2, 0.1], [0.4, 0.45], [0.05, 0.3]]) {
      // Patch 0 of cage face f covers the first half of the face each way.
      const patch = algo.patch(net, once, 4 * f);
      const got = algo.evalPatch(patch, s, t);
      const want = tensor(fi - 1, fj - 1, b(s / 2), b(t / 2));
      close(got, want, assert, "patch");
    }
  }
}
export function vertex_rule_is_the_textbook_form(algo, assert) {
  // (F + 2 R + (n - 3) V) / n with F, R the means of face points and edge midpoints.
  const mesh = algo.cube();
  const once = algo.subdivide(mesh);
  const v = mesh.v[6];
  const fan = algo.fans(mesh)[6];
  const n = fan.length;
  assert.equal(n, 3);
  let F = [0, 0, 0], R = [0, 0, 0];
  for (const [f, k] of fan) {
    const face = mesh.f[f];
    const fp = face.reduce((s, i) => s.map((x, a) => x + mesh.v[i][a] / 4), [0, 0, 0]);
    F = F.map((x, a) => x + fp[a] / n);
    const w = mesh.v[face[(k + 1) % 4]];
    R = R.map((x, a) => x + (v[a] + w[a]) / 2 / n);
  }
  const want = [0, 1, 2].map((a) => (F[a] + 2 * R[a] + (n - 3) * v[a]) / n);
  close(once.v[6], want, assert, "vertex");
}
export function a_creased_rim_stays_a_curve_of_its_own(algo, assert) {
  // The rim of the cube's top, all sharp: its vertices follow (1 6 1)/8 of the rim.
  const cage = algo.withCreases(algo.cube(), true);
  const once = algo.subdivide(cage);
  close(once.v[5], [(cage.v[4][0] + 6 * cage.v[5][0] + cage.v[6][0]) / 8, (cage.v[4][1] + 6 * cage.v[5][1] + cage.v[6][1]) / 8, 1], assert);
  // Its edge points are the edges' midpoints, on the plane of the rim.
  const edge = once.f.flat().filter((i) => i >= cage.v.length + cage.f.length);
  const onRim = edge.filter((i) => Math.abs(once.v[i][2] - 1) < 1e-12);
  assert.ok(onRim.length > 0);
}
export function patches_share_their_boundary_curves(algo, assert) {
  for (const cage of [algo.cube(), algo.lShape(), algo.withCreases(algo.lShape(), true)]) {
    const once = algo.subdivide(cage);
    const net = algo.controlNet(once);
    // Every patch's corners are the limit points, and each is a mesh corner.
    once.f.forEach((q, f) => {
      const patch = algo.patch(net, once, f);
      close(patch[0][0], net.corner[q[0]], assert);
      close(patch[3][0], net.corner[q[1]], assert);
      close(patch[3][3], net.corner[q[2]], assert);
      close(patch[0][3], net.corner[q[3]], assert);
    });
  }
}
export function an_l_block_is_closed_and_consistently_wound(algo, assert) {
  const cage = algo.lShape();
  // Every directed edge once, every edge in both directions.
  const seen = new Set();
  for (const face of cage.f) face.forEach((a, k) => {
    const d = `${a}>${face[(k + 1) % face.length]}`;
    assert.ok(!seen.has(d), `twice: ${d}`);
    seen.add(d);
  });
  for (const d of seen) {
    const [a, b] = d.split(">");
    assert.ok(seen.has(`${b}>${a}`), `open at ${d}`);
  }
  // The volume by the divergence theorem is positive: wound outward. Area of the L x height.
  let volume = 0;
  for (const face of cage.f) for (let k = 1; k + 1 < face.length; k++) {
    const [a, b, c] = [cage.v[face[0]], cage.v[face[k]], cage.v[face[k + 1]]];
    volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
  }
  const area = 3 * 1.2 + 1.2 * 1.8;
  assert.ok(Math.abs(volume - area * 1.6) < 1e-9, `volume ${volume}`);
}
