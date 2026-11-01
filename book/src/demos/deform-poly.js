// A cubic Bezier curve in the (x, t) plane composed with a taper, a twist or a
// bend about the t axis, on its control points: products of Bernstein
// polynomials, no evaluation and no fitting, as core/geop-core-geometry/src/deform.rs
// does for a polynomial piece (all weights one).
(function () {
  "use strict";

  function binom(n, k) { var r = 1; for (var i = 1; i <= k; i++) r = r * (n - k + i) / i; return r; }

  // A polynomial by its Bernstein coefficients.
  // B_i^m B_k^n = C(m,i) C(n,k) / C(m+n,i+k) B_{i+k}^{m+n}.
  function product(a, b) {
    var m = a.length - 1, n = b.length - 1, out = [];
    for (var j = 0; j <= m + n; j++) out.push(0);
    for (var i = 0; i <= m; i++) for (var k = 0; k <= n; k++) {
      out[i + k] += a[i] * b[k] * binom(m, i) * binom(n, k) / binom(m + n, i + k);
    }
    return out;
  }
  function evalBernstein(c, u) {
    var p = c.slice();
    for (var r = p.length - 1; r > 0; r--) for (var i = 0; i < r; i++) p[i] = p[i] * (1 - u) + p[i + 1] * u;
    return p[0];
  }
  function addScaled(sum, scale, c) { for (var i = 0; i < sum.length; i++) sum[i] += scale * c[i]; }
  function constant(value, degree) { var out = []; for (var i = 0; i <= degree; i++) out.push(value); return out; }

  // The map: X = x a(t) + c1(t), T = x e(t) + c2(t) in the plane y = 0, each
  // coefficient constant + slope s + cos cos(rate s) + sin sin(rate s), s = t - start.
  function coefficient(constant_, slope, cos, sin, rate) { return { k: constant_ || 0, m: slope || 0, c: cos || 0, s: sin || 0, r: rate || 0 }; }
  function evalCoefficient(f, s) { return f.k + f.m * s + f.c * Math.cos(f.r * s) + f.s * Math.sin(f.r * s); }
  function isTrig(f) { return f.c !== 0 || f.s !== 0; }

  // AxialMap::taper, ::twist and ::bend, with x along the frame's first axis, y = 0.
  function makeMap(kind, start, end, amount) {
    var L = end - start;
    if (kind === "taper") {
      var slope = (amount - 1) / L;
      return { kind: kind, start: start, a: coefficient(1, slope), e: coefficient(), c1: coefficient(), c2: coefficient(start, 1) };
    }
    var rate = amount / L;
    if (kind === "twist") {
      // X = x cos(rate s): the point seen along y, the frame's second axis.
      return { kind: kind, start: start, a: coefficient(0, 0, 1, 0, rate), e: coefficient(), c1: coefficient(), c2: coefficient(start, 1) };
    }
    if (amount === 0) return { kind: kind, start: start, a: coefficient(1), e: coefficient(), c1: coefficient(), c2: coefficient(start, 1) };
    // Bend: c = (R (1 - cos), start + R sin), a = (cos, -sin).
    var R = L / amount;
    return {
      kind: kind, start: start, a: coefficient(0, 0, 1, 0, rate), e: coefficient(0, 0, 0, -1, rate),
      c1: coefficient(R, 0, -R, 0, rate), c2: coefficient(start, 0, 0, R, rate)
    };
  }
  function mapPoint(map, p) {
    var s = p[1] - map.start;
    return [p[0] * evalCoefficient(map.a, s) + evalCoefficient(map.c1, s), p[0] * evalCoefficient(map.e, s) + evalCoefficient(map.c2, s)];
  }

  // The interpolant of f over t in [mid - half, mid + half], in monomials of
  // sigma = (t - mid) / half: the linear part exactly, the trigonometric part
  // the cubic at the Chebyshev points, by Newton's divided differences.
  // error: (|cos| + |sin|) |rate half|^4 / 4! max|Π(σ - σ_j)|, and for
  // Chebyshev points of degree 3 that maximum is 1/8.
  function interpolant(f, start, mid, half, degree) {
    if (!isTrig(f)) {
      var out = [f.k + f.m * (mid - start), f.m * half];
      while (out.length < degree + 1) out.push(0);
      return { c: out, error: 0 };
    }
    var nodes = [], values = [], j;
    for (j = 0; j <= degree; j++) {
      nodes.push(Math.cos(Math.PI * (2 * j + 1) / (2 * (degree + 1))));
      values.push(evalCoefficient(f, mid + half * nodes[j] - start));
    }
    for (var level = 1; level <= degree; level++) {
      for (j = degree; j >= level; j--) values[j] = (values[j] - values[j - 1]) / (nodes[j] - nodes[j - level]);
    }
    var c = [values[degree]];
    for (j = degree - 1; j >= 0; j--) {
      var next = []; var n;
      for (n = 0; n <= c.length; n++) next.push(0);
      for (n = 0; n < c.length; n++) { next[n + 1] += c[n]; next[n] -= c[n] * nodes[j]; }
      next[0] += values[j];
      c = next;
    }
    var error = (Math.abs(f.c) + Math.abs(f.s)) * Math.pow(Math.abs(f.r) * half, degree + 1) / 24 * 0.125;
    return { c: c, error: error };
  }

  // One piece: control points [[x, t], ...], polynomial. Returns the image's
  // control points, of degree (d + 1) n, d = 1 for a taper and 3 otherwise, and
  // the proven bound on how far it can be from the true image.
  function composePiece(points, map) {
    var n = points.length - 1;
    var xs = points.map(function (p) { return p[0]; }), ts = points.map(function (p) { return p[1]; });
    var lo = Math.min.apply(null, ts), hi = Math.max.apply(null, ts);
    var mid = (lo + hi) / 2, half = (hi - lo) / 2;
    var trig = isTrig(map.a) || isTrig(map.e) || isTrig(map.c1) || isTrig(map.c2);
    var d = trig ? 3 : 1;
    // sigma(u) = (T(u) - mid) / half, or nothing where the piece has no reach.
    var sigma = ts.map(function (t) { return half > 0 ? (t - mid) / half : 0; });
    var one = constant(1, n);
    // sigma^k W^(d-k), every one of degree d n.
    var basis = [], k, power = [1];
    var units = [[1]];
    for (k = 1; k <= d; k++) units.push(product(units[k - 1], one));
    for (k = 0; k <= d; k++) {
      basis.push(product(power, units[d - k]));
      power = product(power, sigma);
    }
    function compose(f) {
      var ip = interpolant(f, map.start, mid, half, d), sum = constant(0, d * n);
      ip.c.forEach(function (c, i) { addScaled(sum, c, basis[i]); });
      return { poly: sum, error: ip.error };
    }
    var a = compose(map.a), e = compose(map.e), c1 = compose(map.c1), c2 = compose(map.c2);
    var xReach = Math.max.apply(null, xs.map(Math.abs));
    // x a(t) + c1(t) W, of degree (d + 1) n.
    var X = product(xs, a.poly), T = product(xs, e.poly);
    addScaled(X, 1, product(one, c1.poly));
    addScaled(T, 1, product(one, c2.poly));
    return {
      points: X.map(function (v, i) { return [v, T[i]]; }),
      degree: (d + 1) * n,
      bound: Math.hypot(a.error * xReach + c1.error, e.error * xReach + c2.error)
    };
  }

  function splitBezier(pts, c) {
    var left = [], right = [], row = pts.map(function (p) { return p.slice(); });
    while (row.length) {
      left.push(row[0]); right.unshift(row[row.length - 1]);
      var next = [];
      for (var i = 0; i + 1 < row.length; i++) next.push([row[i][0] * (1 - c) + row[i + 1][0] * c, row[i][1] * (1 - c) + row[i + 1][1] * c]);
      row = next;
    }
    return [left, right];
  }
  // The curve cut in `pieces` equal parameter stretches.
  function cut(points, pieces) {
    var out = [], rest = points;
    for (var i = pieces; i > 1; i--) {
      var s = splitBezier(rest, 1 / i);
      out.push(s[0]); rest = s[1];
    }
    out.push(rest);
    return out;
  }
  function evalCurve(points, u) {
    return [evalBernstein(points.map(function (p) { return p[0]; }), u), evalBernstein(points.map(function (p) { return p[1]; }), u)];
  }
  // Each piece composed, and how far the composed curve is measured to lie from
  // the true image, sampled at 64 parameters per piece (the sample only reports).
  function compose(points, map, pieces) {
    var parts = cut(points, pieces).map(function (p) { return { source: p, image: composePiece(p, map) }; });
    var bound = 0, measured = 0;
    parts.forEach(function (part) {
      bound = Math.max(bound, part.image.bound);
      for (var i = 0; i <= 64; i++) {
        var u = i / 64;
        var truth = mapPoint(map, evalCurve(part.source, u)), got = evalCurve(part.image.points, u);
        measured = Math.max(measured, Math.hypot(truth[0] - got[0], truth[1] - got[1]));
      }
    });
    return { parts: parts, bound: bound, measured: measured, degree: parts[0].image.degree };
  }

  GeopBook.demo("deform-poly", function (ctx) {
    var D = GeopBook.draw;
    var ZONE = [0, 4];
    var pts = [[0.3, 0], [1.6, 1.2], [-0.8, 2.8], [0.6, 4]];
    var kind = "bend", amount = { taper: 0.3, twist: Math.PI, bend: Math.PI / 2 }, pieces = 1, showPolygon = true;
    var stage = ctx.stage({ aspect: 1.25, maxHeight: 460, label: "A cubic curve and its image under a deformation about the vertical axis", draw: paint });
    var view = ctx.view(stage, { x0: -4.6, x1: 5.4, y0: -1.5, y1: 5 });
    var handles = ctx.handles({
      stage: stage, view: view, pts: pts, onchange: update,
      constrain: function (i, p) { return [Math.max(-2, Math.min(2, p[0])), Math.max(ZONE[0], Math.min(ZONE[1], p[1]))]; }
    });
    var read = ctx.readout();
    var kindSelect = ctx.select({
      label: "map", value: kind, options: [["taper", "taper (exact)"], ["twist", "twist"], ["bend", "bend"]],
      oninput: function (v) { kind = v; syncSlider(); update(); }
    });
    var slider = ctx.slider({
      label: "amount", min: -3, max: 3, step: 0.01, value: Math.PI / 2,
      format: function (v) { return kind === "taper" ? v.toFixed(2) : (v * 180 / Math.PI).toFixed(0) + "°"; },
      oninput: function (v) { amount[kind] = v; update(); }
    });
    ctx.select({ label: "pieces", value: "1", options: ["1", "2", "4", "8"], oninput: function (v) { pieces = +v; update(); } });
    ctx.checkbox({ label: "control polygon", value: true, oninput: function (v) { showPolygon = v; stage.redraw(); } });
    function syncSlider() {
      if (kind === "taper") { slider.el.min = 0.1; slider.el.max = 2.5; } else { slider.el.min = -3; slider.el.max = 3; }
      slider.value = amount[kind];
    }
    function result() { return compose(pts, makeMap(kind, ZONE[0], ZONE[1], amount[kind]), pieces); }

    function paint(g, w, h, c) {
      if (!view || !kindSelect) return;
      view.update();
      D.polyline(g, view, [[0, -1.5], [0, 5]], c.grid, 1);
      D.polyline(g, view, [[-4.6, ZONE[0]], [5.4, ZONE[0]]], c.faint, 1, [3, 4]);
      D.polyline(g, view, [[-4.6, ZONE[1]], [5.4, ZONE[1]]], c.faint, 1, [3, 4]);
      var r = result(), map = makeMap(kind, ZONE[0], ZONE[1], amount[kind]), i;
      // The source.
      var curve = [];
      for (i = 0; i <= 60; i++) curve.push(evalCurve(pts, i / 60));
      D.polyline(g, view, pts, c.muted, 1, [4, 4]);
      D.polyline(g, view, curve, c.muted, 2);
      // The composed pieces.
      r.parts.forEach(function (part, k) {
        var color = c.series[k % 2 === 0 ? 0 : 2];
        if (showPolygon) {
          D.polyline(g, view, part.image.points, c.faint, 1);
          part.image.points.forEach(function (p) { D.dot(g, view, p, 2.5, color); });
        }
        var image = [];
        for (var j = 0; j <= 60; j++) image.push(evalCurve(part.image.points, j / 60));
        D.polyline(g, view, image, color, 2.5);
      });
      // Samples of the true image, mapped one by one.
      for (i = 0; i <= 24; i++) D.dot(g, view, mapPoint(map, evalCurve(pts, i / 24)), 2.5, c.bg, c.orange);
      pts.forEach(function (p, k) { D.dot(g, view, p, 6, handles && handles.selected === k ? c.accent : c.bg, c.fg); });
    }
    function update() {
      var r = result();
      read.set("image: degree " + r.degree + " (" + (r.degree + 1) + " control points per piece)\n" +
        "proven bound " + r.bound.toExponential(2) + ", measured " + r.measured.toExponential(2) +
        (kind === "taper" ? "  (exact: only rounding)" : ""));
      stage.redraw();
    }
    update();
  }, { product: product, evalBernstein: evalBernstein, makeMap: makeMap, mapPoint: mapPoint, composePiece: composePiece, compose: compose, evalCurve: evalCurve, cut: cut, interpolant: interpolant, evalCoefficient: evalCoefficient });
})();
