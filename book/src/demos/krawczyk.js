// Krawczyk's test for a point on a circle and on a line: draw the box X and its
// image K(X), in interval arithmetic with outward rounding. The same test the
// kernel runs, on a system small enough to see.
(function () {
  "use strict";

  // ---- intervals [lo, hi], each operation rounded outwards by a unit in the
  // last place (JavaScript cannot set the rounding mode, so widen by one ulp).
  function down(x) { if (x === 0) return -5e-324; return x - Math.abs(x) * 2.220446049250313e-16 - 5e-324; }
  function up(x) { if (x === 0) return 5e-324; return x + Math.abs(x) * 2.220446049250313e-16 + 5e-324; }
  function I(a, b) { return [a, b === undefined ? a : b]; }
  function iadd(a, b) { return [down(a[0] + b[0]), up(a[1] + b[1])]; }
  function isub(a, b) { return [down(a[0] - b[1]), up(a[1] - b[0])]; }
  function imul(a, b) {
    var p = [a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]];
    return [down(Math.min.apply(null, p)), up(Math.max.apply(null, p))];
  }
  function isubset(a, b) { return a[0] >= b[0] && a[1] <= b[1]; }
  function iabsmax(a) { return Math.max(Math.abs(a[0]), Math.abs(a[1])); }

  // The system: x² + y² - 1 = 0 and y - (m x + h) = 0.
  // f and its Jacobian, over interval arguments.
  function F(X, m, h) {
    var mi = I(m), hi = I(h);
    return [
      isub(iadd(imul(X[0], X[0]), imul(X[1], X[1])), I(1)),
      isub(isub(X[1], imul(mi, X[0])), hi)
    ];
  }
  function J(X, m) {
    var two = I(2);
    return [[imul(two, X[0]), imul(two, X[1])], [I(-m), I(1)]];
  }

  // The test, for a sharp centre c and half widths r. Y the inverse of J(c),
  // in plain numbers: any Y will do, the proof does not depend on it.
  function krawczyk(c, r, m, h) {
    var Xc = [I(c[0]), I(c[1])];
    var X = [[down(c[0] - r[0]), up(c[0] + r[0])], [down(c[1] - r[1]), up(c[1] + r[1])]];
    var Jm = J(Xc, m);
    var a = (Jm[0][0][0] + Jm[0][0][1]) / 2, b = (Jm[0][1][0] + Jm[0][1][1]) / 2, d = -m, e = 1;
    var det = a * e - b * d;
    if (det === 0 || !isFinite(1 / det)) return { invertible: false, X: X, det: det };
    var Y = [[e / det, -b / det], [-d / det, a / det]];
    var f = F(Xc, m, h);
    var JX = J(X, m);
    // Yf
    var yf = [0, 1].map(function (i) { return iadd(imul(I(Y[i][0]), f[0]), imul(I(Y[i][1]), f[1])); });
    // S = I - Y J(X)
    var S = [0, 1].map(function (i) {
      return [0, 1].map(function (j) {
        var yj = iadd(imul(I(Y[i][0]), JX[0][j]), imul(I(Y[i][1]), JX[1][j]));
        return isub(I(i === j ? 1 : 0), yj);
      });
    });
    var delta = [[down(-r[0]), up(r[0])], [down(-r[1]), up(r[1])]];
    var K = [0, 1].map(function (i) {
      var spread = iadd(imul(S[i][0], delta[0]), imul(S[i][1], delta[1]));
      return iadd(isub(I(c[i]), yf[i]), spread);
    });
    var norm = Math.max(iabsmax(S[0][0]) + iabsmax(S[0][1]), iabsmax(S[1][0]) + iabsmax(S[1][1]));
    var inside = isubset(K[0], X[0]) && isubset(K[1], X[1]);
    var disjoint = K[0][1] < X[0][0] || K[0][0] > X[0][1] || K[1][1] < X[1][0] || K[1][0] > X[1][1];
    var verdict = inside && norm < 1 ? "proven" : disjoint && norm < 1 ? "none" : "undecided";
    return { invertible: true, X: X, K: K, S: S, norm: norm, inside: inside, disjoint: disjoint, verdict: verdict, det: det };
  }

  // Newton from c, on plain numbers: the polish before a proof.
  function polish(c, m, h, steps) {
    var x = c.slice();
    for (var k = 0; k < (steps || 6); k++) {
      var f1 = x[0] * x[0] + x[1] * x[1] - 1, f2 = x[1] - m * x[0] - h;
      var a = 2 * x[0], b = 2 * x[1], d = -m, e = 1, det = a * e - b * d;
      if (det === 0) break;
      var nx = x[0] - (e * f1 - b * f2) / det, ny = x[1] - (-d * f1 + a * f2) / det;
      if (!isFinite(nx) || !isFinite(ny)) break;
      x = [nx, ny];
    }
    return x;
  }

  // Widening from a radius: what the kernel does, each radius twice the last,
  // at most `attempts` of them.
  function widen(c, m, h, attempts) {
    var r = Math.max(1e-14, 1e-14);
    for (var k = 0; k < (attempts || 40); k++) {
      var t = krawczyk(c, [r, r], m, h);
      if (t.invertible && t.verdict === "proven") return { radius: r, attempts: k + 1, test: t };

      r *= 2;
    }
    return { radius: null, attempts: attempts || 40 };
  }

  // The intersections of the circle and line, exactly, for drawing and tests.
  function roots(m, h) {
    var A = 1 + m * m, B = 2 * m * h, C = h * h - 1, disc = B * B - 4 * A * C;
    if (disc < 0) return [];
    var s = Math.sqrt(disc), x1 = (-B - s) / (2 * A), x2 = (-B + s) / (2 * A);
    return disc === 0 ? [[x1, m * x1 + h]] : [[x1, m * x1 + h], [x2, m * x2 + h]];
  }

  GeopBook.demo("krawczyk", function (ctx) {
    var D = GeopBook.draw;
    var state = { m: 0, h: 0.6, log: Math.log10(0.2), zoom: false, test: null };
    var pts = [[0.9, 0.5]];
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 380, label: "A circle, a line, a box X and its Krawczyk image K(X)", draw: paint });
    var view = ctx.view(stage, { x0: -1.8, x1: 1.8, y0: -1.3, y1: 1.3 });
    var read = ctx.readout();

    function recompute() {
      var r = Math.pow(10, state.log);
      state.test = krawczyk(pts[0], [r, r], state.m, state.h);
      var t = state.test;
      var lines = ["centre (" + pts[0][0].toPrecision(10) + ", " + pts[0][1].toPrecision(10) + ")   half width " + r.toExponential(1)];
      if (!t.invertible) lines.push("The Jacobian at the centre is exactly singular: no Y to take.");
      else {
        lines.push("‖I − Y J(X)‖∞ = " + t.norm.toPrecision(3) + (t.norm < 1 ? " < 1" : " ≥ 1"));
        lines.push("K(X) ⊆ X: " + (t.inside ? "yes" : "no"));
        lines.push(t.verdict === "proven" ? "PROVEN: X holds exactly one solution." :
          t.verdict === "none" ? "PROVEN: X holds no solution." :
          "UNDECIDED: this box proves nothing, either way.");
      }
      read.set(lines.join("\n"));
      if (state.zoom) {
        var R = Math.max(r * 2.2, 1e-14);
        if (t.invertible && t.K) R = Math.max(R, 1.1 * Math.max(Math.abs(t.K[0][0] - pts[0][0]), Math.abs(t.K[0][1] - pts[0][0]), Math.abs(t.K[1][0] - pts[0][1]), Math.abs(t.K[1][1] - pts[0][1])));
        R = Math.min(R, 20);
        view.box.x0 = pts[0][0] - R * 1.4; view.box.x1 = pts[0][0] + R * 1.4; view.box.y0 = pts[0][1] - R; view.box.y1 = pts[0][1] + R;
      } else { view.box.x0 = -1.8; view.box.x1 = 1.8; view.box.y0 = -1.3; view.box.y1 = 1.3; }
      stage.redraw();
    }

    function rect(g, X, fill, stroke, dash) {
      var x0 = view.x(X[0][0]), x1 = view.x(X[0][1]), y0 = view.y(X[1][1]), y1 = view.y(X[1][0]);
      var clamp = function (v, a) { return Math.max(-50, Math.min(a + 50, v)); };
      x0 = clamp(x0, stage.w); x1 = clamp(x1, stage.w); y0 = clamp(y0, stage.h); y1 = clamp(y1, stage.h);
      if (fill) { g.fillStyle = fill; g.fillRect(x0, y0, Math.max(x1 - x0, 1), Math.max(y1 - y0, 1)); }
      g.strokeStyle = stroke; g.lineWidth = 2; g.setLineDash(dash || []); g.strokeRect(x0, y0, Math.max(x1 - x0, 1), Math.max(y1 - y0, 1)); g.setLineDash([]);
    }

    function paint(g, w, h, c) {
      if (!view) return;
      view.update();
      var b = view.box;
      // Circle
      g.beginPath();
      g.arc(view.x(0), view.y(0), view.sx, 0, 2 * Math.PI);
      g.strokeStyle = c.orange; g.lineWidth = 2; g.stroke();
      // Line
      D.polyline(g, view, [[b.x0, state.m * b.x0 + state.h], [b.x1, state.m * b.x1 + state.h]], c.green, 2);
      roots(state.m, state.h).forEach(function (p) { D.dot(g, view, p, 4, c.fg); });
      var t = state.test;
      if (t) {
        var col = t.verdict === "proven" ? c.green : t.verdict === "none" ? c.red : c.muted;
        rect(g, t.X, c.fillA, c.blue);
        if (t.K) rect(g, t.K, null, col, [5, 4]);
      }
      D.dot(g, view, pts[0], 6, handles && handles.selected === 0 ? c.accent : c.bg, c.fg);
      D.text(g, null, [8, 16], "X solid, K(X) dashed", c.muted);
    }
    var handles = ctx.handles({ stage: stage, view: view, pts: pts, onchange: recompute });
    var lineSlider = ctx.slider({ label: "line height", min: -1.4, max: 1.4, step: 0.001, value: state.h, format: function (v) { return v.toFixed(3); }, oninput: function (v) { state.h = v; recompute(); } });
    ctx.slider({ label: "slope", min: -1, max: 1, step: 0.01, value: 0, oninput: function (v) { state.m = v; recompute(); } });
    var size = ctx.slider({ label: "box size", min: -13, max: 0.1, step: 0.05, value: state.log, format: function (v) { return "1e" + v.toFixed(1).replace("-", "\u2212"); }, oninput: function (v) { state.log = v; recompute(); } });
    ctx.button("Newton polish", function () { var q = polish(pts[0], state.m, state.h, 8); pts[0][0] = q[0]; pts[0][1] = q[1]; recompute(); });
    ctx.button("Widen until proven", function () {
      var w = widen(pts[0], state.m, state.h, 45);
      if (w.radius !== null) { state.log = Math.log10(w.radius); size.value = state.log; }
      recompute();
    });
    ctx.button("Tangent", function () {
      state.h = 1; state.m = 0; lineSlider.value = 1; pts[0][0] = 1e-4; pts[0][1] = 1; state.zoom = true; zoomBox.value = true; state.log = -5; size.value = -5; recompute();
    });
    var zoomBox = ctx.checkbox({ label: "zoom to the box", value: false, oninput: function (v) { state.zoom = v; recompute(); } });
    recompute();
  }, { krawczyk: krawczyk, polish: polish, widen: widen, roots: roots, iadd: iadd, isub: isub, imul: imul, F: F, J: J });
})();
