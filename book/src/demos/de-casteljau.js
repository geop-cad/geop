// De Casteljau's construction of a Bézier curve, its pieces when split at t,
// and the control polygon's convex hull. Pure functions in `algo`.
(function () {
  "use strict";

  function lerp(a, b, t) { return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]; }

  // Every level of the triangle: levels[0] are the control points, the last
  // level is the one point on the curve.
  function levels(pts, t) {
    var out = [pts.map(function (p) { return p.slice(); })];
    while (out[out.length - 1].length > 1) {
      var prev = out[out.length - 1], next = [];
      for (var i = 0; i + 1 < prev.length; i++) next.push(lerp(prev[i], prev[i + 1], t));
      out.push(next);
    }
    return out;
  }
  function evaluate(pts, t) { var l = levels(pts, t); return l[l.length - 1][0]; }

  // The sum over Bernstein polynomials, which de Casteljau must agree with.
  function binom(n, k) { var r = 1; for (var i = 1; i <= k; i++) r = r * (n - k + i) / i; return r; }
  function bernstein(pts, t) {
    var n = pts.length - 1, x = 0, y = 0;
    for (var i = 0; i <= n; i++) {
      var b = binom(n, i) * Math.pow(t, i) * Math.pow(1 - t, n - i);
      x += b * pts[i][0]; y += b * pts[i][1];
    }
    return [x, y];
  }
  // The control points of the two pieces: the two edges of the triangle.
  function split(pts, t) {
    var l = levels(pts, t);
    return { left: l.map(function (r) { return r[0]; }), right: l.map(function (r) { return r[r.length - 1]; }).reverse() };
  }
  // Andrew's monotone chain.
  function hull(pts) {
    var p = pts.slice().sort(function (a, b) { return a[0] - b[0] || a[1] - b[1]; });
    if (p.length < 3) return p;
    function cross(o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); }
    var lo = [], up = [], i;
    for (i = 0; i < p.length; i++) { while (lo.length > 1 && cross(lo[lo.length - 2], lo[lo.length - 1], p[i]) <= 0) lo.pop(); lo.push(p[i]); }
    for (i = p.length - 1; i >= 0; i--) { while (up.length > 1 && cross(up[up.length - 2], up[up.length - 1], p[i]) <= 0) up.pop(); up.push(p[i]); }
    lo.pop(); up.pop();
    return lo.concat(up);
  }

  GeopBook.demo("de-casteljau", function (ctx) {
    var D = GeopBook.draw;
    var pts = [[0.5, 0.6], [1.5, 3.2], [4.2, 3.6], [5.5, 0.8]];
    var t = 0.5, showHull = false, showSplit = false;
    var stage = ctx.stage({ aspect: 1.6, maxHeight: 420, label: "A Bézier curve with its control polygon and de Casteljau's construction", draw: paint });
    var view = ctx.view(stage, { x0: 0, x1: 6, y0: 0, y1: 4 });
    var handles = ctx.handles({ stage: stage, view: view, pts: pts, onchange: update });
    var read = ctx.readout();

    function paint(g, w, h, c) {
      if (!view) return;
      view.update();
      if (showHull) D.polygon(g, view, hull(pts), c.fillC, null);
      var curve = [];
      for (var i = 0; i <= 120; i++) curve.push(evaluate(pts, i / 120));
      D.polyline(g, view, pts, c.muted, 1.2, [4, 4]);
      if (showSplit) {
        var s = split(pts, t);
        D.polyline(g, view, s.left, c.blue, 1.5);
        D.polyline(g, view, s.right, c.orange, 1.5);
        s.left.forEach(function (p) { D.dot(g, view, p, 3.5, c.blue); });
        s.right.forEach(function (p) { D.dot(g, view, p, 3.5, c.orange); });
      } else {
        var ls = levels(pts, t);
        for (var k = 1; k < ls.length - 1; k++) {
          D.polyline(g, view, ls[k], c.series[(k - 1) % c.series.length], 1.5);
          ls[k].forEach(function (p) { D.dot(g, view, p, 3, c.series[(k - 1) % c.series.length]); });
        }
      }
      D.polyline(g, view, curve, c.fg, 2.2);
      var on = evaluate(pts, t);
      D.dot(g, view, on, 5, c.red);
      pts.forEach(function (p, i) {
        D.dot(g, view, p, 6, handles && handles.selected === i ? c.accent : c.bg, c.fg);
        D.text(g, view, [p[0] + 0.1, p[1] + 0.1], "P" + i, c.muted);
      });
    }
    function update() {
      var p = evaluate(pts, t), q = bernstein(pts, t);
      read.set("t = " + t.toFixed(3) + "   C(t) = (" + p[0].toFixed(4) + ", " + p[1].toFixed(4) + ")" +
        "   by the Bernstein sum: (" + q[0].toFixed(4) + ", " + q[1].toFixed(4) + ")");
      stage.redraw();
    }
    ctx.player({ duration: 5, loop: true, onframe: function (v) { t = v; update(); } }).set(0.5);
    ctx.button("Add point", function () {
      if (pts.length >= 8) return;
      // Degree elevation keeps the same curve: Q_i = (i P_{i-1} + (n+1-i) P_i) / (n+1).
      var n = pts.length - 1, q = [pts[0].slice()];
      for (var i = 1; i <= n; i++) {
        var a = i / (n + 1);
        q.push([a * pts[i - 1][0] + (1 - a) * pts[i][0], a * pts[i - 1][1] + (1 - a) * pts[i][1]]);
      }
      q.push(pts[n].slice());
      pts.length = 0; q.forEach(function (p) { pts.push(p); });
      update();
    });
    ctx.checkbox({ label: "convex hull", value: false, oninput: function (v) { showHull = v; stage.redraw(); } });
    ctx.checkbox({ label: "split at t", value: false, oninput: function (v) { showSplit = v; stage.redraw(); } });
    update();
  }, { levels: levels, evaluate: evaluate, bernstein: bernstein, split: split, hull: hull });
})();
