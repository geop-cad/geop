// Every form must enclose the true point for every parameter in the box; the
// relative form must not grow with the distance from the origin, and A / W must.
const r = 3;
function trueArc(cx, cy, t) {
  const w = Math.SQRT2 / 2, a = (1 - t) * (1 - t), b = 2 * t * (1 - t) * w, e = t * t;
  return [cx + r * (a + b) / (a + b + e), cy + r * (b + e) / (a + b + e)];
}
const contains = (iv, x, slack = 0) => iv[0] - slack <= x && x <= iv[1] + slack;

export function every_form_encloses_the_curve_over_the_box(a, assert) {
  for (const d of [0.001, 1, 1000, 1e6]) {
    const pts = a.arcAt(d, -d / 2, d / 20, r);
    const box = [0.33 - 0.005, 0.33 + 0.005];
    for (const f of ["direct", "relative", "both", "meanValue"]) {
      const v = a[f](pts, box);
      for (let i = 0; i <= 20; i++) {
        const t = box[0] + (box[1] - box[0]) * i / 20, q = trueArc(d, -d / 2, t);
        // The double-precision sample itself carries rounding of a few places of d.
        const slack = 8 * Number.EPSILON * Math.max(d, 1);
        assert.ok(contains(v[0], q[0], slack) && contains(v[1], q[1], slack), `${f} d=${d} t=${t}: ${v[0]} ${q[0]}`);
      }
      assert.ok(contains(v[2], d / 20, 8 * Number.EPSILON * Math.max(d, 1)), `${f} z`);
    }
  }
}
export function the_arc_is_a_circle(a, assert) {
  const pts = a.arcAt(0, 0, 0, r);
  for (let i = 0; i <= 20; i++) {
    const v = a.relative(pts, a.I(i / 20));
    assert.ok(Math.abs(Math.hypot(a.mid(v[0]), a.mid(v[1])) - r) < 1e-14);
  }
}
export function direct_grows_with_distance_and_relative_does_not(a, assert) {
  const near = a.widthsAt(10, 0.33, 0.005, r), far = a.widthsAt(1e6, 0.33, 0.005, r);
  assert.ok(far.direct > 1000 * near.direct, `direct ${near.direct} ${far.direct}`);
  assert.ok(far.relative < 4 * near.relative, `relative ${near.relative} ${far.relative}`);
}
export function relative_is_about_as_wide_as_the_stretch(a, assert) {
  const w = a.widthsAt(1000, 0.33, 0.005, r);
  assert.ok(w.relative < 4 * w.stretch, `${w.relative} vs ${w.stretch}`);
}
export function flat_coordinate_stays_narrow_relative_to_a_point(a, assert) {
  const w = a.widthsAt(1000, 0.33, 0.005, r);
  assert.ok(w.relativeHeight < 1e-10, `${w.relativeHeight}`);
  assert.ok(w.directHeight > 0.1, `${w.directHeight}`);
}
export function the_intersection_is_never_wider_and_can_be_tighter_than_both(a, assert) {
  const pts = a.arcAt(1, -0.5, 0.05, r);
  let tighter = 0;
  for (let i = 1; i < 100; i++) {
    const t = i % 2 ? a.I(i / 100) : [i / 100 - 0.002, i / 100 + 0.002];
    const d = a.direct(pts, t), e = a.relative(pts, t), b = a.both(pts, t);
    for (let c = 0; c < 3; c++) {
      assert.ok(a.width(b[c]) <= Math.min(a.width(d[c]), a.width(e[c])));
      if (a.width(b[c]) < Math.min(a.width(d[c]), a.width(e[c]))) tighter++;
    }
  }
  assert.ok(tighter > 0);
}
