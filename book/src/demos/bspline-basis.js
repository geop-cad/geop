// A B-spline: its basis functions by the Cox-de Boor recursion, its curve by
// de Boor's algorithm, and what dragging the knots does. Pure functions in `algo`.
(function () {
  "use strict";

  // N_{i,p}(t) by the Cox-de Boor recursion, half-open spans, 0/0 = 0. `last`
  // says t is the domain's end, where the last non-empty span is closed.
  function basis(i, p, knots, t, end) {
    if (p === 0) {
      if (knots[i] <= t && t < knots[i + 1]) return 1;
      // At the domain's end the span ending there counts.
      if (end !== undefined && t === end && knots[i] < knots[i + 1] && knots[i + 1] === end) return 1;
      return 0;
    }
    var a = 0, b = 0;
    var d1 = knots[i + p] - knots[i], d2 = knots[i + p + 1] - knots[i + 1];
    if (d1 > 0) a = (t - knots[i]) / d1 * basis(i, p - 1, knots, t, end);
    if (d2 > 0) b = (knots[i + p + 1] - t) / d2 * basis(i + 1, p - 1, knots, t, end);
    return a + b;
  }
  // The span k in [p, n] with knots[k] <= t < knots[k+1], as `find_span` in
  // core/geop-core-geometry/src/spline.rs; at the end, the last non-empty span.
  function findSpan(p, knots, n, t) {
    if (t >= knots[n + 1]) {
      for (var k = n; k >= p; k--) if (knots[k] < knots[n + 1]) return k;
      return p;
    }
    for (var j = p; j <= n; j++) if (knots[j] <= t && t < knots[j + 1]) return j;
    return p;
  }
  // Every level of de Boor's triangle at t in span: level r holds points r..p.
  function deBoorLevels(p, knots, pts, t, span) {
    var levels = [];
    var d = [];
    for (var j = 0; j <= p; j++) d.push(pts[span - p + j].slice());
    levels.push(d.map(function (q) { return q.slice(); }));
    for (var r = 1; r <= p; r++) {
      for (var i = p; i >= r; i--) {
        var idx = span - p + i;
        var den = knots[idx + p - r + 1] - knots[idx];
        var a = den > 0 ? (t - knots[idx]) / den : 0;
        d[i] = d[i - 1].map(function (v, c) { return v + a * (d[i][c] - v); });
      }
      levels.push(d.slice(r).map(function (q) { return q.slice(); }));
    }
    return levels;
  }
  function deBoor(p, knots, pts, t) {
    var span = findSpan(p, knots, pts.length - 1, t);
    var l = deBoorLevels(p, knots, pts, t, span);
    return l[p][0];
  }
  // The same point as the sum of basis functions times control points.
  function byBasis(p, knots, pts, t) {
    var end = knots[pts.length], x = 0, y = 0;
    for (var i = 0; i < pts.length; i++) {
      var b = basis(i, p, knots, t, end);
      x += b * pts[i][0]; y += b * pts[i][1];
    }
    return [x, y];
  }
  // The derivative's control points: p (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1}),
  // a B-spline of degree p-1 on the knots without their first and last.
  function derivativeSpline(p, knots, pts) {
    var q = [];
    for (var i = 0; i + 1 < pts.length; i++) {
      var den = knots[i + p + 1] - knots[i + 1];
      var f = den > 0 ? p / den : 0;
      q.push(pts[i + 1].map(function (v, c) { return f * (v - pts[i][c]); }));
    }
    return { degree: p - 1, knots: knots.slice(1, knots.length - 1), points: q };
  }
  // The distinct interior knots with their multiplicities and the continuity
  // C^{p-m} they give.
  function joints(p, knots, count) {
    var out = [], lo = knots[p], hi = knots[count];
    for (var i = p + 1; i < count; i++) {
      var k = knots[i];
      if (k <= lo || k >= hi) continue;
      if (out.length && out[out.length - 1].knot === k) out[out.length - 1].multiplicity++;
      else out.push({ knot: k, multiplicity: 1 });
    }
    out.forEach(function (j) { j.continuity = p - j.multiplicity; });
    return out;
  }
  // A clamped knot vector, uniform inside, on [0, 1].
  function clampedUniform(p, count) {
    var k = [], i;
    for (i = 0; i <= p; i++) k.push(0);
    var spans = count - p;
    for (i = 1; i < spans; i++) k.push(i / spans);
    for (i = 0; i <= p; i++) k.push(1);
    return k;
  }

  GeopBook.demo("bspline-basis", function (ctx) {
    var D = GeopBook.draw;
    var p = 3;
    var pts = [[0.4, 0.6], [1.1, 3.1], [2.3, 1.1], [3.2, 3.4], [4.3, 3.2], [5, 0.9], [5.8, 2.2]];
    var knots = clampedUniform(p, pts.length);
    var t = 0.42, showBoor = true;
    var knotHandles = [];

    var stage = ctx.stage({ aspect: 1.75, maxHeight: 360, label: "A B-spline curve with its control polygon", draw: paintCurve });
    var view = ctx.view(stage, { x0: 0, x1: 6.2, y0: 0.2, y1: 3.8 });
    var handles = ctx.handles({ stage: stage, view: view, pts: pts, onchange: update });

    var stage2 = ctx.stage({ height: 170, label: "The basis functions over the parameter, with the knots as handles below", draw: paintBasis });
    var view2 = ctx.view(stage2, { x0: -0.02, x1: 1.02, y0: -0.2, y1: 1.05 }, { free: true, pad: 8 });
    var kh = ctx.handles({
      stage: stage2, view: view2, pts: knotHandles, radius: 18,
      constrain: constrainKnot,
      onchange: function (i) { knots[p + 1 + i] = knotHandles[i][0]; update(); }
    });
    var read = ctx.readout();

    function rebuildKnotHandles() {
      knotHandles.length = 0;
      for (var i = p + 1; i < pts.length; i++) knotHandles.push([knots[i], -0.1]);
    }
    // A knot stays between its neighbours; dropped near one, it joins it, and
    // the multiplicity goes up.
    function constrainKnot(i, q) {
      var lo = i > 0 ? knotHandles[i - 1][0] : 0;
      var hi = i + 1 < knotHandles.length ? knotHandles[i + 1][0] : 1;
      var x = Math.max(lo, Math.min(hi, q[0]));
      var snap = 0.012;
      if (x - lo < snap) x = lo; else if (hi - x < snap) x = hi;
      return [x, -0.1];
    }
    function paintCurve(g, w, h, c) {
      if (!view || !handles) return;
      view.update();
      D.polyline(g, view, pts, c.muted, 1.2, [4, 4]);
      // The curve, one colour per knot span.
      var lo = knots[p], hi = knots[pts.length], k;
      for (k = p; k < pts.length; k++) {
        if (knots[k + 1] <= knots[k]) continue;
        var piece = [];
        for (var i = 0; i <= 40; i++) {
          var s = knots[k] + (knots[k + 1] - knots[k]) * i / 40;
          // A span's own polynomial, so a jump at a full knot is not bridged.
          piece.push(deBoorLevels(p, knots, pts, s, k)[p][0]);
        }
        D.polyline(g, view, piece, c.series[(k - p) % c.series.length], 2.4);
      }
      var span = findSpan(p, knots, pts.length - 1, t);
      if (showBoor && p > 0) {
        var ls = deBoorLevels(p, knots, pts, t, span);
        for (k = 0; k < p; k++) {
          D.polyline(g, view, ls[k], k === 0 ? c.faint : c.muted, k === 0 ? 3 : 1.3);
          if (k) ls[k].forEach(function (q) { D.dot(g, view, q, 2.5, c.fg); });
        }
      }
      pts.forEach(function (q, i) {
        var active = i >= span - p && i <= span;
        D.dot(g, view, q, active ? 6.5 : 5, handles.selected === i ? c.accent : c.bg, active ? c.red : c.fg);
      });
      D.dot(g, view, deBoor(p, knots, pts, t), 5, c.red);
    }
    function paintBasis(g, w, h, c) {
      if (!view2 || !read) return;
      view2.update();
      D.polyline(g, view2, [[0, 0], [1, 0]], c.muted, 1);
      var end = knots[pts.length];
      for (var i = 0; i < pts.length; i++) {
        var curve = [];
        for (var j = 0; j <= 200; j++) {
          var s = j / 200;
          curve.push([s, basis(i, p, knots, s, end)]);
        }
        D.polyline(g, view2, curve, c.series[i % c.series.length], 1.6);
      }
      D.polyline(g, view2, [[t, 0], [t, 1]], c.red, 1, [3, 3]);
      knotHandles.forEach(function (q, i) {
        D.polyline(g, view2, [[q[0], -0.03], [q[0], 0.03]], c.fg, 1.5);
        D.dot(g, view2, q, kh && kh.selected === i ? 6 : 5, c.accent, c.fg);
      });
    }
    function update() {
      var js = joints(p, knots, pts.length);
      var cont = js.map(function (j) {
        return "u=" + j.knot.toFixed(2) + " m=" + j.multiplicity + " " + (j.continuity < 0 ? "broken" : "C" + j.continuity);
      }).join("  ");
      var q = deBoor(p, knots, pts, t), b = byBasis(p, knots, pts, t);
      var span = findSpan(p, knots, pts.length - 1, t);
      read.set("degree " + p + ", span k=" + span + ", active P" + (span - p) + "..P" + span +
        "\nde Boor (" + q[0].toFixed(4) + ", " + q[1].toFixed(4) + ")   sum of N_i P_i (" + b[0].toFixed(4) + ", " + b[1].toFixed(4) + ")" +
        "\n" + (cont || "no interior knots: one Bézier span"));
      stage.redraw(); stage2.redraw();
    }
    rebuildKnotHandles();
    ctx.slider({ label: "t", min: 0, max: 1, step: 0.001, value: t, oninput: function (v) { t = v; update(); } });
    ctx.select({
      label: "degree", options: [["1", "1"], ["2", "2"], ["3", "3"], ["4", "4"], ["6", "6 (one span)"]], value: "3",
      oninput: function (v) { p = +v; knots = clampedUniform(p, pts.length); rebuildKnotHandles(); update(); }
    });
    ctx.button("Uniform knots", function () { knots = clampedUniform(p, pts.length); rebuildKnotHandles(); update(); });
    ctx.button("Triple knot", function () {
      // Pile the interior knots of the middle onto one value, as far as they go.
      knots = clampedUniform(p, pts.length);
      var count = knots.length, mid = 0.5, m = Math.min(p, pts.length - p - 1);
      var first = p + 1 + Math.floor((pts.length - p - 1 - m) / 2);
      for (var i = first; i < first + m; i++) knots[i] = mid;
      rebuildKnotHandles(); update();
    });
    ctx.checkbox({ label: "de Boor levels", value: true, oninput: function (v) { showBoor = v; stage.redraw(); } });
    update();
  }, {
    basis: basis, findSpan: findSpan, deBoorLevels: deBoorLevels, deBoor: deBoor, byBasis: byBasis,
    derivativeSpline: derivativeSpline, joints: joints, clampedUniform: clampedUniform
  });
})();
