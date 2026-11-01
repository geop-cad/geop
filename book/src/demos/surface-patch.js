// A bicubic tensor-product patch: two rounds of de Casteljau, partials, the
// normal, the fundamental forms and curvature. Pure functions in `algo`.
(function () {
  "use strict";

  function lerp(a, b, t) { return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]; }
  function casteljau(pts, t) {
    var l = pts.slice();
    while (l.length > 1) {
      var n = [];
      for (var i = 0; i + 1 < l.length; i++) n.push(lerp(l[i], l[i + 1], t));
      l = n;
    }
    return l[0];
  }
  // Net: net[i][j], i along u. First each column of constant j along u, giving
  // the control points of the iso-curve u = const in v; then that curve at v.
  function isoCurveAtU(net, u) {
    var out = [];
    for (var j = 0; j < net[0].length; j++) {
      out.push(casteljau(net.map(function (row) { return row[j]; }), u));
    }
    return out;
  }
  function evaluate(net, u, v) { return casteljau(isoCurveAtU(net, u), v); }
  function binom(n, k) { var r = 1; for (var i = 1; i <= k; i++) r = r * (n - k + i) / i; return r; }
  function bern(n, i, t) { return binom(n, i) * Math.pow(t, i) * Math.pow(1 - t, n - i); }
  // The double sum over the tensor-product basis.
  function bySum(net, u, v) {
    var p = net.length - 1, q = net[0].length - 1, out = [0, 0, 0];
    for (var i = 0; i <= p; i++) for (var j = 0; j <= q; j++) {
      var b = bern(p, i, u) * bern(q, j, v);
      for (var c = 0; c < 3; c++) out[c] += b * net[i][j][c];
    }
    return out;
  }
  // Partial derivatives up to order two, by differencing the net: S^(k,l) is
  // the net differenced k times along u, l times along v, scaled by the falling
  // factorials of the degrees, then evaluated.
  function differenced(net, k, l) {
    var n = net, i, j, c, m;
    for (m = 0; m < k; m++) {
      var f = n.length - 1, next = [];
      for (i = 0; i + 1 < n.length; i++) {
        var row = [];
        for (j = 0; j < n[0].length; j++) row.push([0, 1, 2].map(function (c2) { return f * (n[i + 1][j][c2] - n[i][j][c2]); }));
        next.push(row);
      }
      n = next;
    }
    for (m = 0; m < l; m++) {
      var g = n[0].length - 1;
      n = n.map(function (r) {
        var o = [];
        for (var a = 0; a + 1 < r.length; a++) o.push([0, 1, 2].map(function (c2) { return g * (r[a + 1][c2] - r[a][c2]); }));
        return o;
      });
    }
    return n;
  }
  function partial(net, k, l, u, v) { return evaluate(differenced(net, k, l), u, v); }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  // The first and second fundamental forms, the normal, and the curvatures:
  // K = (LN - M^2)/(EG - F^2), H = (LG - 2MF + NE) / (2 (EG - F^2)).
  function forms(net, u, v) {
    var su = partial(net, 1, 0, u, v), sv = partial(net, 0, 1, u, v);
    var suu = partial(net, 2, 0, u, v), suv = partial(net, 1, 1, u, v), svv = partial(net, 0, 2, u, v);
    var nn = cross(su, sv), len = Math.sqrt(dot(nn, nn));
    var n = [nn[0] / len, nn[1] / len, nn[2] / len];
    var E = dot(su, su), F = dot(su, sv), G = dot(sv, sv);
    var L = dot(suu, n), M = dot(suv, n), N = dot(svv, n);
    var det = E * G - F * F;
    return {
      su: su, sv: sv, normal: n, E: E, F: F, G: G, L: L, M: M, N: N,
      K: (L * N - M * M) / det,
      H: (L * G - 2 * M * F + N * E) / (2 * det)
    };
  }
  // A 4x4 net over [-1.5, 1.5]^2 at heights h(i, j).
  function makeNet(h) {
    var net = [];
    for (var i = 0; i < 4; i++) {
      var row = [];
      for (var j = 0; j < 4; j++) row.push([-1.5 + i, -1.5 + j, h(i, j)]);
      net.push(row);
    }
    return net;
  }
  var PRESETS = {
    dome: function (i, j) { return (i > 0 && i < 3 ? 0.9 : 0) + (j > 0 && j < 3 ? 0.9 : 0) - 0.5; },
    saddle: function (i, j) { return (i - 1.5) * (j - 1.5) * 0.5; },
    ripple: function (i, j) { return ((i + j) % 2 === 0 ? 0.6 : -0.6) * (i === 0 || i === 3 ? 0.5 : 1); },
    flat: function () { return 0; }
  };

  GeopBook.demo("surface-patch", function (ctx) {
    var D = GeopBook.draw;
    var net = makeNet(PRESETS.dome);
    var u = 0.35, v = 0.6, showNet = true, showIso = true, picked = [1, 1];

    var stage = ctx.stage({ aspect: 1.3, maxHeight: 420, label: "A bicubic surface patch with its control net", draw: paint });
    var cam = ctx.camera(stage, { yaw: 0.6, pitch: 0.95, scale: 0.21, distance: 12 });
    var read = ctx.readout();

    function P(q) { return cam.project(q); }
    function paint(g, w, h, c) {
      if (!cam || !read) return;
      var N = 16, grid = [], i, j;
      for (i = 0; i <= N; i++) { grid.push([]); for (j = 0; j <= N; j++) grid[i].push(evaluate(net, i / N, j / N)); }
      // Painter's order over the patch's little quads, shaded by the normal.
      var quads = [];
      for (i = 0; i < N; i++) for (j = 0; j < N; j++) {
        var a = grid[i][j], b = grid[i + 1][j], d = grid[i + 1][j + 1], e = grid[i][j + 1];
        var n = cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [e[0] - a[0], e[1] - a[1], e[2] - a[2]]);
        var l = Math.sqrt(dot(n, n)) || 1;
        var pa = P(a), pb = P(b), pd = P(d), pe = P(e);
        quads.push({ pts: [pa, pb, pd, pe], depth: pa[2] + pb[2] + pd[2] + pe[2], lit: Math.abs(n[2]) / l });
      }
      quads.sort(function (x, y) { return x.depth - y.depth; });
      var base = c.dark ? [108, 178, 255] : [80, 140, 210];
      quads.forEach(function (q) {
        var s = 0.35 + 0.65 * q.lit;
        var col = "rgba(" + Math.round(base[0] * s) + "," + Math.round(base[1] * s) + "," + Math.round(base[2] * s) + ",0.85)";
        D.polygon(g, null, q.pts, col, col, 0.5);
      });
      if (showNet) {
        for (i = 0; i < 4; i++) {
          D.polyline(g, null, net[i].map(P), c.muted, 1);
          D.polyline(g, null, net.map(function (r) { return P(r[i]); }), c.muted, 1);
        }
        net.forEach(function (r, ii) { r.forEach(function (q, jj) {
          var on = ii === picked[0] && jj === picked[1];
          D.dot(g, null, P(q), on ? 5.5 : 3, on ? c.accent : c.fg);
        }); });
      }
      // The two stages of de Casteljau.
      var iso = isoCurveAtU(net, u);
      if (showIso) {
        for (j = 0; j < 4; j++) {
          var col = [];
          for (i = 0; i <= 30; i++) col.push(P(casteljau(net.map(function (r) { return r[j]; }), i / 30)));
          D.polyline(g, null, col, c.faint, 1.5);
        }
        var isoCurve = [];
        for (i = 0; i <= 40; i++) isoCurve.push(P(casteljau(iso, i / 40)));
        D.polyline(g, null, iso.map(P), c.orange, 1.5, [4, 3]);
        D.polyline(g, null, isoCurve, c.orange, 2.4);
        iso.forEach(function (q) { D.dot(g, null, P(q), 4, c.orange); });
      }
      var S = evaluate(net, u, v), f = forms(net, u, v);
      D.polyline(g, null, [P(S), P([S[0] + f.su[0] * 0.15, S[1] + f.su[1] * 0.15, S[2] + f.su[2] * 0.15])], c.green, 2);
      D.polyline(g, null, [P(S), P([S[0] + f.sv[0] * 0.15, S[1] + f.sv[1] * 0.15, S[2] + f.sv[2] * 0.15])], c.purple, 2);
      D.polyline(g, null, [P(S), P([S[0] + f.normal[0], S[1] + f.normal[1], S[2] + f.normal[2]])], c.red, 2.2);
      D.dot(g, null, P(S), 5, c.red);
      var key = P([S[0] + f.normal[0], S[1] + f.normal[1], S[2] + f.normal[2]]);
      D.text(g, null, [key[0] + 6, key[1]], "n", c.red);
    }
    function update() {
      var S = evaluate(net, u, v), f = forms(net, u, v);
      var fx = function (x) { return x.toFixed(3); };
      read.set("S = (" + S.map(fx).join(", ") + ")\nE F G = " + fx(f.E) + " " + fx(f.F) + " " + fx(f.G) +
        "    L M N = " + fx(f.L) + " " + fx(f.M) + " " + fx(f.N) +
        "\nGaussian K = " + f.K.toFixed(4) + "   mean H = " + f.H.toFixed(4) +
        (Math.abs(f.K) < 1e-9 ? "   (developable or flat here)" : f.K > 0 ? "   (dome-like here)" : "   (saddle-like here)"));
      stage.redraw();
    }
    ctx.slider({ label: "u", min: 0, max: 1, step: 0.005, value: u, oninput: function (x) { u = x; update(); } });
    ctx.slider({ label: "v", min: 0, max: 1, step: 0.005, value: v, oninput: function (x) { v = x; update(); } });
    var height = ctx.slider({ label: "height of picked point", min: -2, max: 2, step: 0.01, value: net[1][1][2], oninput: function (x) { net[picked[0]][picked[1]][2] = x; update(); } });
    var opts = [];
    for (var i = 0; i < 4; i++) for (var j = 0; j < 4; j++) opts.push([i + "," + j, "P" + i + j]);
    ctx.select({
      label: "pick", options: opts, value: "1,1",
      oninput: function (x) { var s = x.split(","); picked = [+s[0], +s[1]]; height.value = net[picked[0]][picked[1]][2]; update(); }
    });
    ctx.select({
      label: "preset", options: ["dome", "saddle", "ripple", "flat"], value: "dome",
      oninput: function (x) { net = makeNet(PRESETS[x]); height.value = net[picked[0]][picked[1]][2]; update(); }
    });
    ctx.checkbox({ label: "net", value: true, oninput: function (x) { showNet = x; stage.redraw(); } });
    ctx.checkbox({ label: "iso-curve", value: true, oninput: function (x) { showIso = x; stage.redraw(); } });
    update();
  }, { evaluate: evaluate, bySum: bySum, isoCurveAtU: isoCurveAtU, partial: partial, forms: forms, makeNet: makeNet, PRESETS: PRESETS });
})();
