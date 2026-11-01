// The march stays on both surfaces, keeps its strides on the planes the corrector
// was told, closes the loop, and fails as Lecture 16 says when the stride is too big.
const a = 0.35, r = 0.5, R = 1;

export function every_point_lies_on_both_surfaces(algo, assert) {
  const m = algo.march(a, r, R, 0.2, 400);
  assert.equal(m.status, "closed");
  for (const s of m.steps) {
    const [ds, dc] = algo.offSurfaces(s.point, a, r, R);
    assert.ok(ds < 1e-10 && dc < 1e-10, `${ds} ${dc}`);
  }
}
export function the_tangent_is_square_to_both_normals(algo, assert) {
  const A = algo.sphere(R), B = algo.cylinder(a, r), q = algo.start(a, r, R, 0.6), t = algo.tangent(A, B, q);
  const cross = (u, v) => [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
  const dot = (u, v) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  const na = cross(A.du(q[0], q[1]), A.dv(q[0], q[1])), nb = cross(B.du(q[2], q[3]), B.dv(q[2], q[3]));
  assert.ok(Math.abs(dot(t, na)) < 1e-12 && Math.abs(dot(t, nb)) < 1e-12);
  assert.ok(Math.abs(dot(t, t) - 1) < 1e-12);
}
export function the_corrector_lands_on_the_plane_it_was_given(algo, assert) {
  const A = algo.sphere(R), B = algo.cylinder(a, r), q = algo.start(a, r, R, 0.6), dir = algo.tangent(A, B, q);
  const p = A.p(q[0], q[1]), target = [p[0] + 0.1 * dir[0], p[1] + 0.1 * dir[1], p[2] + 0.1 * dir[2]];
  const c = algo.corrector(A, B, q, target, dir);
  assert.equal(c.converged, true);
  const off = [c.point[0] - target[0], c.point[1] - target[1], c.point[2] - target[2]];
  assert.ok(Math.abs(off[0] * dir[0] + off[1] * dir[1] + off[2] * dir[2]) < 1e-11);
}
export function the_corrector_converges_fast(algo, assert) {
  const m = algo.march(a, r, R, 0.1, 400);
  for (const s of m.steps) if (s.residuals) assert.ok(s.iterations <= 6, `${s.iterations}`);
}
export function halving_the_stride_doubles_the_steps(algo, assert) {
  const n1 = algo.march(a, r, R, 0.1, 800).steps.length, n2 = algo.march(a, r, R, 0.05, 800).steps.length;
  assert.ok(Math.abs(n2 / n1 - 2) < 0.2, `${n1} ${n2}`);
}
export function too_long_a_stride_loses_the_curve(algo, assert) {
  // Some stride this long does not close: the corrector fails, or it runs out.
  const statuses = [0.7, 0.8, 0.9, 1.0, 1.1, 1.2].map((s) => algo.march(a, r, R, s, 400).status);
  assert.ok(statuses.some((s) => s !== "closed"), statuses.join("; "));
}
export function viviani_has_a_node_where_normals_are_parallel(algo, assert) {
  // a + r = R: the cylinder touches the sphere at (1, 0, 0), and there the normals agree.
  const A = algo.sphere(1), B = algo.cylinder(0.5, 0.5);
  assert.equal(algo.tangent(A, B, [0, 0, 0, 0]), null);
}
