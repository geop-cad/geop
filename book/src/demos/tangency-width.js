// How wide an honest enclosure of a crossing is, near a tangency. The curve is
// y = t^2 - c and its height is only known to +-eps, the width the kernel's
// coordinates carry; the crossing with y = 0 is enclosed by a search over t using
// the hull of the interval Bernstein coefficients. Pure functions in `algo`.
(function () {
  "use strict";

  // The range of t^2 - c +- eps over [a, b], by its Bernstein coefficients on that
  // piece: a^2, a b, b^2 (a quadratic's blossom). The hull is a real enclosure.
  function range(a, b, c, eps) {
    var lo = Math.min(a * a, a * b, b * b) - c - eps, hi = Math.max(a * a, a * b, b * b) - c + eps;
    return [lo, hi];
  }
  function mayVanish(a, b, c, eps) { var r = range(a, b, c, eps); return r[0] <= 0 && r[1] >= 0; }

  // Every t in [-1, 1] that could be a zero: subdivide to isolate, then push each end
  // of each piece in by bisection, which is what a tangency makes slow to isolate.
  function enclose(c, eps) {
    var nodes = 0, pieces = [], stack = [[-1, 1]];
    var h = Math.max(1e-3 * Math.sqrt(eps), 1e-7);
    while (stack.length) {
      var s = stack.pop(); nodes++;
      if (!mayVanish(s[0], s[1], c, eps)) continue;
      if (s[1] - s[0] <= h) {
        var last = pieces[pieces.length - 1];
        if (last && Math.abs(last[1] - s[0]) < 1e-15) last[1] = s[1]; else pieces.push([s[0], s[1]]);
        continue;
      }
      var m = (s[0] + s[1]) / 2;
      stack.push([m, s[1]], [s[0], m]);
    }
    pieces.forEach(function (p) {
      var k;
      // Lower end: the smallest t a narrow box could still hold a zero at.
      var lo = p[0], hi = p[1];
      for (k = 0; k < 50 && hi - lo > 1e-15; k++) {
        var m = (lo + hi) / 2; nodes++;
        if (!mayVanish(lo, m, c, eps) && lo < m) lo = m; else hi = m;
      }
      var left = lo;
      lo = left; hi = p[1];
      for (k = 0; k < 50 && hi - lo > 1e-15; k++) {
        var n = (lo + hi) / 2; nodes++;
        if (!mayVanish(n, hi, c, eps) && n < hi) hi = n; else lo = n;
      }
      p[0] = left; p[1] = hi;
    });
    return { pieces: pieces, nodes: nodes };
  }
  // Exact answer for comparison: t^2 in [c - eps, c + eps].
  function exact(c, eps) {
    var hi = c + eps, lo = c - eps;
    if (hi < 0) return [];
    var R = Math.min(Math.sqrt(hi), 1);
    if (lo <= 0) return [[-R, R]];
    var r = Math.sqrt(lo);
    return [[-R, -r], [r, R]];
  }
  // Newton on t^2 - c from a seed: its iterations down to a step of 1e-14. On a double
  // root the error only halves per step, as one finds from e' = e / 2.
  function newtonIterations(c, seed) {
    var t = seed, k = 0;
    for (; k < 200; k++) {
      var f = t * t - c, d = 2 * t;
      if (d === 0) break;
      var dt = f / d;
      t -= dt;
      if (Math.abs(dt) < 1e-14) break;
    }
    return k + 1;
  }

  GeopBook.demo("tangency-width", function (ctx) {
    var D = GeopBook.draw;
    var c = 0.1, logEps = -6;
    var stage = ctx.stage({ aspect: 1.15, maxHeight: 560, label: "A parabola near the line y = 0, the crossing enclosures, and the width of the enclosure against the uncertainty on log axes", draw: paint });
    var read = ctx.readout();
    function paint(g, W, H, col) {
      var eps = Math.pow(10, logEps), split = H * 0.5;
      var fake = { w: W, h: split };
      var view = ctx.view(fake, { x0: -1.05, x1: 1.05, y0: -0.3, y1: 0.75 }, { free: true, pad: 8 });
      // Top: the curve, and its band.
      D.polyline(g, view, [[-1.05, 0], [1.05, 0]], col.muted, 1);
      var up = [], dn = [], mid = [];
      for (var i = 0; i <= 200; i++) {
        var t = -1 + i / 100;
        up.push([t, t * t - c + eps]); dn.unshift([t, t * t - c - eps]); mid.push([t, t * t - c]);
      }
      D.polygon(g, view, up.concat(dn), col.fillA, null);
      D.polyline(g, view, mid, col.blue, 2);
      var r = enclose(c, eps);
      r.pieces.forEach(function (p) {
        var x0 = view.x(p[0]), x1 = Math.max(view.x(p[1]), x0 + 3);
        g.fillStyle = col.red; g.fillRect(x0, view.y(0) - 3, x1 - x0, 6);
      });
      D.text(g, null, [8, 14], "y = t² - c, the band its uncertainty", col.muted);
      // Bottom: width against eps, log-log.
      var x0 = 42, w = W - x0 - 10, y0 = split + 8, h = H - y0 - 22;
      g.strokeStyle = col.faint; g.strokeRect(x0, y0, w, h);
      function X(le) { return x0 + (le + 14) / 14 * w; }
      function Y(lw) { return y0 + h - (lw + 16) / 16 * h; }
      g.fillStyle = col.muted; g.textAlign = "center"; g.font = "11px sans-serif";
      for (var e = -14; e <= 0; e += 2) g.fillText("1e" + e, X(e), y0 + h + 13);
      g.textAlign = "right";
      for (var q = -16; q <= 0; q += 4) g.fillText("1e" + q, x0 - 3, Y(q) + 4);
      g.strokeStyle = col.grid; g.beginPath();
      for (e = -14; e <= 0; e += 2) { g.moveTo(X(e), y0); g.lineTo(X(e), y0 + h); }
      for (q = -16; q <= 0; q += 4) { g.moveTo(x0, Y(q)); g.lineTo(x0 + w, Y(q)); }
      g.stroke();
      var curve = [], sq = [], lin = [];
      for (var le = -14; le <= -0.5; le += 0.5) {
        var pc = enclose(c, Math.pow(10, le)).pieces;
        var widest = 0;
        pc.forEach(function (p) { widest = Math.max(widest, p[1] - p[0]); });
        if (widest > 0) curve.push([X(le), Y(Math.log10(widest))]);
        sq.push([X(le), Y(Math.log10(2 * Math.sqrt(Math.pow(10, le))))]);
        lin.push([X(le), Y(le)]);
      }
      D.polyline(g, null, sq, col.green, 1, [4, 3]);
      D.polyline(g, null, lin, col.purple, 1, [4, 3]);
      D.polyline(g, null, curve, col.red, 2);
      D.dot(g, null, [X(logEps), r.pieces.length ? Y(Math.log10(Math.max.apply(null, r.pieces.map(function (p) { return p[1] - p[0]; })))) : y0], 4, col.red);
      g.font = "12px sans-serif";
      D.text(g, null, [x0 + 4, y0 + 13], "widest enclosure against ε", col.red);
      D.text(g, null, [x0 + 4, y0 + 27], "2√ε", col.green);
      D.text(g, null, [x0 + 4, y0 + 41], "ε", col.purple);
    }
    function update() {
      var eps = Math.pow(10, logEps), r = enclose(c, eps), ex = exact(c, eps), s = "";
      s += r.pieces.length + " enclosure(s) for c = " + c.toFixed(4) + ", ε = " + eps.toExponential(0) + ":\n";
      r.pieces.forEach(function (p, i) {
        s += "  [" + p[0].toFixed(8) + ", " + p[1].toFixed(8) + "], width " + (p[1] - p[0]).toExponential(2) + "\n";
      });
      if (!r.pieces.length) s += "  none: the search proved there is no crossing\n";
      s += "Search: " + r.nodes + " boxes. Newton on t² - c from t = 0.7, to a step of 1e-14: " + (c >= 0 ? newtonIterations(c, 0.7) + " iterations" : "no root to reach") + ".";
      read.set(s);
      stage.redraw();
    }
    ctx.slider({ label: "c", min: -0.05, max: 0.3, step: 0.0005, value: c, format: function (v) { return v.toFixed(4); }, oninput: function (v) { c = v; update(); } });
    ctx.button("Tangent: c = 0", function () { c = 0; cs.value = 0; update(); });
    var cs = { set value(v) { var input = ctx.root.querySelectorAll("input[type=range]")[0]; input.value = v; input.dispatchEvent(new Event("input")); } };
    ctx.slider({ label: "log₁₀ ε", min: -14, max: -1, step: 0.5, value: logEps, format: function (v) { return v.toFixed(1); }, oninput: function (v) { logEps = v; update(); } });
    update();
  }, { range: range, mayVanish: mayVanish, enclose: enclose, exact: exact, newtonIterations: newtonIterations });
})();
