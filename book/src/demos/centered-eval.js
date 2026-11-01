// A quarter arc far from the origin, evaluated over a parameter box in real
// outward-rounded interval arithmetic: as A / W, relative to one of its control
// points (`spline::centered`), and by the mean value form. Pure functions in `algo`.
(function () {
  "use strict";

  var buf = new Float64Array(1), bits = new BigInt64Array(buf.buffer);
  function nextUp(x) {
    if (!isFinite(x)) return x;
    if (x === 0) return Number.MIN_VALUE;
    buf[0] = x; bits[0] += x > 0 ? 1n : -1n; return buf[0];
  }
  function nextDown(x) { return -nextUp(-x); }
  function I(lo, hi) { return [lo, hi === undefined ? lo : hi]; }
  function add(a, b) { return [nextDown(a[0] + b[0]), nextUp(a[1] + b[1])]; }
  function sub(a, b) { return [nextDown(a[0] - b[1]), nextUp(a[1] - b[0])]; }
  function mul(a, b) {
    var p = [a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]];
    return [nextDown(Math.min.apply(null, p)), nextUp(Math.max.apply(null, p))];
  }
  function div(a, b) { return mul(a, [nextDown(1 / b[1]), nextUp(1 / b[0])]); }
  function intersect(a, b) {
    var lo = Math.max(a[0], b[0]), hi = Math.min(a[1], b[1]);
    if (lo > hi) return a[1] - a[0] < b[1] - b[0] ? a : b;
    return [lo, hi];
  }
  function width(a) { return a[1] - a[0]; }
  function mid(a) { return (a[0] + a[1]) / 2; }
  function isExact(a, b) { return a[0] === b[0] && a[1] === b[1]; }
  // `Scalar::interpolate`.
  function interpolate(a, b, alpha) {
    if (isExact(a, b) && alpha[0] >= 0 && alpha[1] <= 1) return a;
    return intersect(add(a, mul(alpha, sub(b, a))), add(mul(sub(I(1), alpha), a), mul(alpha, b)));
  }
  function lerpV(a, b, alpha) { return a.map(function (x, c) { return interpolate(x, b[c], alpha); }); }
  function casteljau(points, t) {
    var l = points.slice();
    while (l.length > 1) {
      var n = [];
      for (var i = 0; i + 1 < l.length; i++) n.push(lerpV(l[i], l[i + 1], t));
      l = n;
    }
    return l[0];
  }
  // A quarter of a circle of radius r around (cx, cy) at height z, homogeneous
  // (x w, y w, z w, w), the end points taken with weight one as they are.
  function arcAt(cx, cy, z, r) {
    var w = Math.SQRT2 / 2;
    return [
      [I(cx + r), I(cy), I(z), I(1)],
      [mul(I(cx + r), I(w)), mul(I(cy + r), I(w)), mul(I(z), I(w)), I(w)],
      [I(cx), I(cy + r), I(z), I(1)]
    ];
  }
  // A / W, directly: three components.
  function direct(points, t) {
    var A = casteljau(points, t);
    return [0, 1, 2].map(function (c) { return div(A[c], A[3]); });
  }
  // `centered`: the points moved so that points[at]'s Cartesian position, made
  // sharp (a free choice), sits at the origin; the weight slot is not moved.
  function centered(points, at) {
    var first = points[at];
    var origin = [0, 1, 2].map(function (c) { return I(mid(div(first[c], first[3]))); });
    var moved = points.map(function (p) {
      return [sub(p[0], mul(p[3], origin[0])), sub(p[1], mul(p[3], origin[1])), sub(p[2], mul(p[3], origin[2])), p[3]];
    });
    return { moved: moved, origin: origin };
  }
  // origin + A / W, relative to the control point nearer the parameter.
  function relative(points, t) {
    var at = mid(t) > 0.5 ? points.length - 1 : 0;
    var c = centered(points, at), A = casteljau(c.moved, t);
    return [0, 1, 2].map(function (k) { return add(c.origin[k], div(A[k], A[3])); });
  }
  // The curve's `evaluate_into`: the two enclosures intersected.
  function both(points, t) {
    var a = direct(points, t), b = relative(points, t);
    return a.map(function (x, c) { return intersect(x, b[c]); });
  }
  // The derivative over t, by the quotient rule, relative to a control point:
  // differenced first, then evaluated.
  function tangentOver(points, t) {
    var at = mid(t) > 0.5 ? points.length - 1 : 0;
    var moved = centered(points, at).moved;
    var d = [];
    for (var i = 0; i + 1 < moved.length; i++) {
      d.push(moved[i].map(function (x, c) { return mul(I(moved.length - 1), sub(moved[i + 1][c], x)); }));
    }
    var dA = casteljau(d, t), A = casteljau(moved, t);
    return [0, 1, 2].map(function (k) { return div(sub(dA[k], mul(dA[3], div(A[k], A[3]))), A[3]); });
  }
  // The mean value form: the point at the box's middle plus the derivative over
  // the box times the half width. The kernel uses it on surfaces.
  function meanValue(points, t) {
    var m = I(mid(t)), d = sub(t, m), T = tangentOver(points, t), M = relative(points, m);
    return M.map(function (x, c) { return add(x, mul(T[c], d)); });
  }
  function maxWidth(v, n) {
    var w = 0;
    for (var c = 0; c < (n || 3); c++) w = Math.max(w, width(v[c]));
    return w;
  }
  // The true extent of the arc over t, in closed form from its angle: the rational
  // quarter's parameter gives angle θ(t) = atan2(w t (1-t) * 2 + t², ...).
  function trueStretch(r, lo, hi) {
    function pointAt(t) {
      var w = Math.SQRT2 / 2, a = (1 - t) * (1 - t), b = 2 * t * (1 - t) * w, e = t * t;
      return [r * (a + b) / (a + b + e), r * (b + e) / (a + b + e)];
    }
    var p = pointAt(lo), q = pointAt(hi);
    return Math.max(Math.abs(p[0] - q[0]), Math.abs(p[1] - q[1]));
  }
  // Widths of each form, at a distance D from the origin, for a box of the given
  // half width around t.
  function widthsAt(distance, t, half, r) {
    var points = arcAt(distance, -distance / 2, distance / 20, r);
    var box = half > 0 ? [t - half, t + half] : I(t);
    return {
      direct: maxWidth(direct(points, box)),
      directHeight: width(direct(points, box)[2]),
      relative: maxWidth(relative(points, box)),
      relativeHeight: width(relative(points, box)[2]),
      both: maxWidth(both(points, box)),
      mean: maxWidth(meanValue(points, box)),
      stretch: half > 0 ? trueStretch(r, t - half, t + half) : 0
    };
  }

  GeopBook.demo("centered-eval", function (ctx) {
    var D = GeopBook.draw;
    var logD = 3, half = 0.005;
    var stage = ctx.stage({ aspect: 1.45, maxHeight: 360, label: "Width of the enclosure against the arc's distance from the origin, logarithmic on both axes", draw: paint });
    var read = ctx.readout();
    function paint(g, w, h, c) {
      if (!read) return;
      var L = 42, R = 10, T = 8, B = 28;
      var xmin = -3, xmax = 6;
      var ymin = half > 0 ? -18 : -19, ymax = 7;
      function X(x) { return L + (w - L - R) * (x - xmin) / (xmax - xmin); }
      function Y(v) { var l = Math.log10(Math.max(v, 1e-30)); return T + (h - T - B) * (ymax - Math.max(ymin, Math.min(ymax, l))) / (ymax - ymin); }
      var e;
      g.strokeStyle = c.grid; g.lineWidth = 1;
      for (e = ymin + (((ymax - ymin) % 5)); e <= ymax; e += 5) {
        g.beginPath(); g.moveTo(L, Y(Math.pow(10, e))); g.lineTo(w - R, Y(Math.pow(10, e))); g.stroke();
        D.text(g, null, [L - 4, Y(Math.pow(10, e)) + 4], "1e" + e, c.muted, "right");
      }
      for (e = xmin; e <= xmax; e += 3) {
        g.beginPath(); g.moveTo(X(e), T); g.lineTo(X(e), h - B); g.stroke();
        D.text(g, null, [X(e), h - B + 12], "1e" + e, c.muted, "center");
      }
      D.text(g, null, [(L + w - R) / 2, h - 3], "distance of the arc from the origin", c.muted, "center");
      var keys = [["direct", c.red, "A / W"], ["relative", c.blue, "relative to a control point"], ["mean", c.green, "mean value form"]];
      if (half > 0) keys.push(["stretch", c.muted, "stretch of arc the box covers"]);
      keys.forEach(function (k) {
        var pts = [];
        for (var x = xmin; x <= xmax + 1e-9; x += 0.125) pts.push([X(x), Y(widthsAt(Math.pow(10, x), 0.33, half, 3)[k[0]])]);
        D.polyline(g, null, pts, k[1], 2, k[0] === "stretch" ? [4, 4] : undefined);
      });
      // The legend where the lines are not: below them for a box, above for a sharp t.
      var top = half > 0 ? h - B - 8 - 14 * (keys.length - 1) : T + 14;
      keys.forEach(function (k, i) { D.text(g, null, [L + 8, top + 14 * i], k[2], k[1]); });
      D.polyline(g, null, [[X(logD), T], [X(logD), h - B]], c.fg, 1, [2, 3]);
    }
    function update() {
      var d = Math.pow(10, logD), r = widthsAt(d, 0.33, half, 3);
      var f = function (x) { return x.toExponential(2); };
      read.set("arc of radius 3 centred " + d.toPrecision(3) + " from the origin, t in [" + (0.33 - half).toFixed(4) + ", " + (0.33 + half).toFixed(4) + "]\n" +
        "width in x or y:   A/W " + f(r.direct) + "   relative " + f(r.relative) + "   intersected " + f(r.both) + "   mean value " + f(r.mean) + "\n" +
        "width in height z: A/W " + f(r.directHeight) + "   relative " + f(r.relativeHeight) + "  (the arc is flat in z)");
      stage.redraw();
    }
    ctx.slider({ label: "distance", min: -3, max: 6, step: 0.05, value: logD, format: function (v) { return "1e" + v.toFixed(1); }, oninput: function (v) { logD = v; update(); } });
    ctx.select({
      label: "t box width", options: [["0.01", "0.01"], ["0.1", "0.1"], ["0.0001", "0.0001"], ["0", "0 (sharp t)"]], value: "0.01",
      oninput: function (v) { half = +v / 2; update(); }
    });
    update();
  }, {
    nextUp: nextUp, I: I, add: add, sub: sub, mul: mul, div: div, interpolate: interpolate, arcAt: arcAt, casteljau: casteljau,
    direct: direct, relative: relative, both: both, meanValue: meanValue, widthsAt: widthsAt, trueStretch: trueStretch, width: width, mid: mid
  });
})();
