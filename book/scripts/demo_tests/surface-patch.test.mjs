// Two rounds of de Casteljau against the double sum; partials against central
// differences; curvature against a surface of known curvature.
const samples = [0, 0.13, 0.5, 0.77, 1];
const close = (x, y, e, assert) => { for (let c = 0; c < 3; c++) assert.ok(Math.abs(x[c] - y[c]) < e, `${x} vs ${y}`); };

export function de_casteljau_twice_agrees_with_the_double_sum(a, assert) {
  const net = a.makeNet(a.PRESETS.dome);
  for (const u of samples) for (const v of samples) close(a.evaluate(net, u, v), a.bySum(net, u, v), 1e-12, assert);
}
export function either_order_gives_the_same_point(a, assert) {
  const net = a.makeNet(a.PRESETS.ripple);
  const swapped = net[0].map((_, j) => net.map((row) => row[j]));
  for (const u of samples) for (const v of samples) close(a.evaluate(net, u, v), a.evaluate(swapped, v, u), 1e-12, assert);
}
export function partials_match_central_differences(a, assert) {
  const net = a.makeNet(a.PRESETS.dome), h = 1e-5;
  for (const u of [0.2, 0.6]) for (const v of [0.3, 0.8]) {
    const du = [0, 1, 2].map((c) => (a.evaluate(net, u + h, v)[c] - a.evaluate(net, u - h, v)[c]) / (2 * h));
    const dv = [0, 1, 2].map((c) => (a.evaluate(net, u, v + h)[c] - a.evaluate(net, u, v - h)[c]) / (2 * h));
    close(a.partial(net, 1, 0, u, v), du, 1e-7, assert);
    close(a.partial(net, 0, 1, u, v), dv, 1e-7, assert);
    const duv = [0, 1, 2].map((c) => (a.partial(net, 0, 1, u + h, v)[c] - a.partial(net, 0, 1, u - h, v)[c]) / (2 * h));
    close(a.partial(net, 1, 1, u, v), duv, 1e-7, assert);
  }
}
export function curvature_of_a_hyperbolic_paraboloid(a, assert) {
  // z = xy over [0,1]^2 as a bicubic: x = u, y = v, z = uv, control heights i j / 9.
  const net = [];
  for (let i = 0; i < 4; i++) { net.push([]); for (let j = 0; j < 4; j++) net[i].push([i / 3, j / 3, (i / 3) * (j / 3)]); }
  for (const [u, v] of [[0, 0], [0.5, 0.5], [0.25, 0.75]]) {
    const f = a.forms(net, u, v), d = 1 + u * u + v * v;
    assert.ok(Math.abs(f.K + 1 / (d * d)) < 1e-12, `K ${f.K}`);
    // H of z = xy: -xy / (1 + x^2 + y^2)^(3/2).
    assert.ok(Math.abs(f.H + u * v / Math.pow(d, 1.5)) < 1e-12, `H ${f.H}`);
  }
}
export function flat_net_has_no_curvature_and_a_vertical_normal(a, assert) {
  const f = a.forms(a.makeNet(a.PRESETS.flat), 0.3, 0.7);
  assert.ok(Math.abs(f.K) < 1e-12 && Math.abs(f.H) < 1e-12);
  close(f.normal, [0, 0, 1], 1e-12, assert);
}
export function corners_are_the_nets_corners(a, assert) {
  const net = a.makeNet(a.PRESETS.ripple);
  close(a.evaluate(net, 0, 0), net[0][0], 1e-15, assert);
  close(a.evaluate(net, 1, 1), net[3][3], 1e-15, assert);
  close(a.evaluate(net, 1, 0), net[3][0], 1e-15, assert);
}
