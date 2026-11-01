// The offset of a cubic Bezier curve: O = S + d N, which is no polynomial or
// rational curve in general. Computed exactly point by point, fitted piecewise by
// cubics, with the fit's measured error (the kernel proves a bound instead of
// measuring one: see core/geop-core-geometry/src/nurb_surface/offset_fit.rs).
(function () {
  "use strict";
  function b(P, t) {
    var s = 1 - t, a = s * s * s, e = 3 * s * s * t, c = 3 * s * t * t, d = t * t * t;
    return [a * P[0][0] + e * P[1][0] + c * P[2][0] + d * P[3][0], a * P[0][1] + e * P[1][1] + c * P[2][1] + d * P[3][1]];
  }
  function d1(P, t) {
    var s = 1 - t;
    return [3 * (s * s * (P[1][0] - P[0][0]) + 2 * s * t * (P[2][0] - P[1][0]) + t * t * (P[3][0] - P[2][0])),
            3 * (s * s * (P[1][1] - P[0][1]) + 2 * s * t * (P[2][1] - P[1][1]) + t * t * (P[3][1] - P[2][1]))];
  }
  function d2(P, t) {
    var s = 1 - t;
    return [6 * (s * (P[2][0] - 2 * P[1][0] + P[0][0]) + t * (P[3][0] - 2 * P[2][0] + P[1][0])),
            6 * (s * (P[2][1] - 2 * P[1][1] + P[0][1]) + t * (P[3][1] - 2 * P[2][1] + P[1][1]))];
  }
  // Signed curvature, positive where the curve turns to its left.
  function kappa(P, t) {
    var a = d1(P, t), c = d2(P, t), l = Math.hypot(a[0], a[1]);
    return (a[0] * c[1] - a[1] * c[0]) / (l * l * l);
  }
  function normal(P, t) { var a = d1(P, t), l = Math.hypot(a[0], a[1]); return [-a[1] / l, a[0] / l]; }
  function offsetPoint(P, t, d) { var p = b(P, t), n = normal(P, t); return [p[0] + d * n[0], p[1] + d * n[1]]; }
  // O'(t) = S'(t) (1 - d kappa): the offset moves along the same tangent, scaled.
  function offsetDerivative(P, t, d) {
    var a = d1(P, t), f = 1 - d * kappa(P, t);
    return [a[0] * f, a[1] * f];
  }
  // Pieces of a C1 cubic through exact offset points and derivatives at n+1 parameters.
  function fit(P, d, n) {
    var pieces = [];
    for (var i = 0; i < n; i++) {
      var t0 = i / n, t1 = (i + 1) / n, h = t1 - t0;
      var p0 = offsetPoint(P, t0, d), p3 = offsetPoint(P, t1, d), m0 = offsetDerivative(P, t0, d), m1 = offsetDerivative(P, t1, d);
      pieces.push([p0, [p0[0] + m0[0] * h / 3, p0[1] + m0[1] * h / 3], [p3[0] - m1[0] * h / 3, p3[1] - m1[1] * h / 3], p3]);
    }
    return pieces;
  }
  // The largest distance between the fit and the exact offset at the same parameter,
  // over `m` samples per piece (a measurement, not a proof).
  function fitError(P, d, n, m) {
    var pieces = fit(P, d, n), worst = 0;
    for (var i = 0; i < n; i++) for (var k = 0; k <= m; k++) {
      var s = k / m, t = (i + s) / n, f = b(pieces[i], s), o = offsetPoint(P, t, d);
      worst = Math.max(worst, Math.hypot(f[0] - o[0], f[1] - o[1]));
    }
    return worst;
  }
  // Where the factor 1 - d kappa changes sign along the curve: the cusps of the offset.
  function cusps(P, d, samples) {
    var out = [], prev = 1 - d * kappa(P, 0);
    for (var i = 1; i <= samples; i++) {
      var t = i / samples, f = 1 - d * kappa(P, t);
      if ((f < 0) !== (prev < 0)) {
        var lo = (i - 1) / samples, hi = t, flo = prev;
        for (var it = 0; it < 40; it++) {
          var m = (lo + hi) / 2, fm = 1 - d * kappa(P, m);
          if ((fm < 0) === (flo < 0)) { lo = m; flo = fm; } else hi = m;
        }
        out.push((lo + hi) / 2);
      }
      prev = f;
    }
    return out;
  }

  GeopBook.demo("offset-curve", function (ctx) {
    var D = GeopBook.draw;
    var P = [[0.5, 0.9], [1.7, 3.6], [3.9, 3.5], [5.5, 1.2]];
    var dist = -0.6, n = 2, comb = true;
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 420, label: "A cubic curve, its exact offset and a fit to it", draw: paint });
    var view = ctx.view(stage, { x0: -0.6, x1: 6.6, y0: -1, y1: 4.4 });
    var handles = ctx.handles({ stage: stage, view: view, pts: P, onchange: update });
    var read = ctx.readout();
    function paint(g, w, h, c) {
      if (!view) return;
      view.update();
      var i, N = 240;
      if (comb) {
        var maxK = 0;
        for (i = 0; i <= N; i++) maxK = Math.max(maxK, Math.abs(kappa(P, i / N)));
        var sc = 0.9 / Math.max(maxK, 0.2), tips = [];
        for (i = 0; i <= N; i++) {
          var t = i / N, p = b(P, t), nn = normal(P, t), k = kappa(P, t);
          var tip = [p[0] - nn[0] * k * sc, p[1] - nn[1] * k * sc];
          tips.push(tip);
          if (i % 6 === 0) D.polyline(g, view, [p, tip], c.faint, 1);
        }
        D.polyline(g, view, tips, c.muted, 1);
      }
      D.polyline(g, view, P, c.faint, 1, [4, 4]);
      var base = [], off = [];
      for (i = 0; i <= N; i++) { base.push(b(P, i / N)); off.push(offsetPoint(P, i / N, dist)); }
      D.polyline(g, view, base, c.fg, 2.2);
      D.polyline(g, view, off, c.green, 3.5);
      var pieces = fit(P, dist, n), first = true;
      pieces.forEach(function (Q, idx) {
        var f = [];
        for (i = 0; i <= 50; i++) f.push(b(Q, i / 50));
        D.polyline(g, view, f, c.orange, 1.4);
        D.dot(g, view, Q[0], 3, c.orange);
        if (idx === pieces.length - 1) D.dot(g, view, Q[3], 3, c.orange);
      });
      cusps(P, dist, 400).forEach(function (t) { D.dot(g, view, offsetPoint(P, t, dist), 6, null, c.red); });
      P.forEach(function (p, i2) { D.dot(g, view, p, 6, handles && handles.selected === i2 ? c.accent : c.bg, c.fg); });
    }
    function update() {
      var cs = cusps(P, dist, 400), err = fitError(P, dist, n, 40);
      var min = Infinity;
      for (var i = 0; i <= 400; i++) min = Math.min(min, 1 - dist * kappa(P, i / 400));
      read.set("least of 1 - d kappa along the curve: " + min.toFixed(4) + (cs.length ? "   the offset has " + cs.length + " cusp" + (cs.length > 1 ? "s" : "") + " and folds" : "   the offset is regular") + "\n" +
        "fit of " + n + " cubic" + (n > 1 ? "s" : "") + ": largest miss measured " + err.toExponential(2) + (cs.length ? "" : ",   with twice the pieces " + fitError(P, dist, 2 * n, 40).toExponential(2)));
      stage.redraw();
    }
    ctx.slider({ label: "distance d", min: -2.5, max: 2.5, step: 0.01, value: dist, oninput: function (v) { dist = v; update(); } });
    ctx.slider({ label: "fit pieces", min: 1, max: 16, step: 1, value: n, format: function (v) { return "" + v; }, oninput: function (v) { n = v; update(); } });
    ctx.checkbox({ label: "curvature comb", value: comb, oninput: function (v) { comb = v; stage.redraw(); } });
    update();
  }, { b: b, kappa: kappa, offsetPoint: offsetPoint, offsetDerivative: offsetDerivative, fit: fit, fitError: fitError, cusps: cusps });
})();
