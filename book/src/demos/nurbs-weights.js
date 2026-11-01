// A rational quadratic: its weights, its lift into homogeneous space, the
// conic it is, and the exact circle. Pure functions in `algo`.
(function () {
  "use strict";

  // The homogeneous de Casteljau of (w x, w y, w) control points, projected.
  function evaluate(pts, weights, t) {
    var h = pts.map(function (p, i) { return [p[0] * weights[i], p[1] * weights[i], weights[i]]; });
    while (h.length > 1) {
      var next = [];
      for (var i = 0; i + 1 < h.length; i++) {
        next.push([h[i][0] + (h[i + 1][0] - h[i][0]) * t, h[i][1] + (h[i + 1][1] - h[i][1]) * t, h[i][2] + (h[i + 1][2] - h[i][2]) * t]);
      }
      h = next;
    }
    return [h[0][0] / h[0][2], h[0][1] / h[0][2]];
  }
  // The rational Bernstein sum, which it must agree with.
  function rationalSum(pts, weights, t) {
    var n = pts.length - 1, X = 0, Y = 0, W = 0, b = 1;
    for (var i = 0; i <= n; i++) {
      b = (i === 0 ? 1 : b * (n - i + 1) / i);
      var B = b * Math.pow(t, i) * Math.pow(1 - t, n - i);
      X += B * weights[i] * pts[i][0]; Y += B * weights[i] * pts[i][1]; W += B * weights[i];
    }
    return [X / W, Y / W];
  }
  // C' = (A' - W' C) / W, the quotient rule; for degree 2 Bézier.
  function derivative(pts, weights, t) {
    var h = pts.map(function (p, i) { return [p[0] * weights[i], p[1] * weights[i], weights[i]]; });
    var d0 = [0, 1, 2].map(function (c) { return 2 * (h[1][c] - h[0][c]); });
    var d1 = [0, 1, 2].map(function (c) { return 2 * (h[2][c] - h[1][c]); });
    var dA = [0, 1, 2].map(function (c) { return d0[c] + (d1[c] - d0[c]) * t; });
    var A = [0, 1, 2].map(function (c) { return (1 - t) * (1 - t) * h[0][c] + 2 * t * (1 - t) * h[1][c] + t * t * h[2][c]; });
    var C = [A[0] / A[2], A[1] / A[2]];
    return [(dA[0] - dA[2] * C[0]) / A[2], (dA[1] - dA[2] * C[1]) / A[2]];
  }
  // The shape invariant w1^2 / (w0 w2): which conic the curve is part of does not
  // change when the parameter is reparametrized, which rescales the weights.
  function conicType(weights) {
    var k = weights[1] * weights[1] / (weights[0] * weights[2]);
    if (Math.abs(k - 1) < 1e-9) return "parabola";
    return k < 1 ? "ellipse" : "hyperbola";
  }
  // The kernel's span of an arc through `step` degrees, `arc_span` in
  // core/geop-core-geometry/src/nurb_curve/helix.rs: control points of an arc of
  // radius r around the origin from `from` degrees on.
  function arcSpan(from, step, r) {
    var a = from * Math.PI / 180, b = (from + step) * Math.PI / 180, h = step * Math.PI / 360;
    var p0 = [r * Math.cos(a), r * Math.sin(a)], p2 = [r * Math.cos(b), r * Math.sin(b)];
    var out = 1 / (1 + Math.cos(2 * h));
    return {
      points: [p0, [out * (p0[0] + p2[0]), out * (p0[1] + p2[1])], p2],
      weights: [1, Math.cos(h), 1]
    };
  }
  // The largest |radius - r| over samples of the spans of a full circle.
  function circleError(spans, r, samples) {
    var worst = 0, step = 360 / spans;
    for (var s = 0; s < spans; s++) {
      var span = arcSpan(s * step, step, r);
      for (var i = 0; i <= samples; i++) {
        var q = evaluate(span.points, span.weights, i / samples);
        worst = Math.max(worst, Math.abs(Math.hypot(q[0], q[1]) - r));
      }
    }
    return worst;
  }
  // The residual of the conic through the five conditions: tangent at the two
  // ends and through the shoulder. For control points P0, P1, P2 written in
  // barycentric coordinates (a, b, c), a rational quadratic satisfies
  // b^2 w0 w2 = 4 a c w1^2, whichever weights it has.
  function conicResidual(pts, weights, q) {
    var P0 = pts[0], P1 = pts[1], P2 = pts[2];
    var det = (P1[0] - P0[0]) * (P2[1] - P0[1]) - (P1[1] - P0[1]) * (P2[0] - P0[0]);
    var b = ((q[0] - P0[0]) * (P2[1] - P0[1]) - (q[1] - P0[1]) * (P2[0] - P0[0])) / det;
    var c = ((P1[0] - P0[0]) * (q[1] - P0[1]) - (P1[1] - P0[1]) * (q[0] - P0[0])) / det;
    var a = 1 - b - c;
    return b * b * weights[0] * weights[2] - 4 * a * c * weights[1] * weights[1];
  }

  GeopBook.demo("nurbs-weights", function (ctx) {
    var D = GeopBook.draw;
    var pts = [[0.5, 0.5], [2.6, 3.2], [4.7, 0.5]];
    var weights = [1, 1, 1];
    var t = 0.5, spans = 4, mode = "free";

    var stage = ctx.stage({ aspect: 1.5, maxHeight: 340, label: "A rational quadratic curve in the plane", draw: paintPlane });
    var view = ctx.view(stage, { x0: 0, x1: 5.2, y0: 0, y1: 3.5 });
    var handles = ctx.handles({ stage: stage, view: view, pts: pts, active: function () { return mode === "free"; }, onchange: update });
    var stage3 = ctx.stage({ aspect: 1.5, maxHeight: 300, label: "The same curve lifted into homogeneous space", draw: paintLift });
    var cam = ctx.camera(stage3, { yaw: -0.5, pitch: 0.28, scale: 0.21, centre: [2.6, 1.5, 0.8], distance: 40 });
    var read = ctx.readout();

    // x and y run along the camera's plane, the weight axis upwards.
    function lifted(q, w) { return [q[0], q[1], w]; }

    function paintPlane(g, w, h, c) {
      if (!view || !read) return;
      view.update();
      if (mode === "circle") {
        var r = 1.6, centre = [2.6, 1.75], step = 360 / spans;
        for (var s = 0; s < spans; s++) {
          var span = arcSpan(s * step, step, r);
          var shift = function (q) { return [q[0] + centre[0], q[1] + centre[1]]; };
          var curve = [];
          for (var i = 0; i <= 40; i++) curve.push(shift(evaluate(span.points, span.weights, i / 40)));
          D.polyline(g, view, curve, c.series[s % c.series.length], 2.4);
          D.polyline(g, view, span.points.map(shift), c.muted, 1, [4, 4]);
          D.dot(g, view, shift(span.points[0]), 4, c.bg, c.fg);
          D.dot(g, view, shift(span.points[1]), 3.5, c.muted);
        }
        return;
      }
      D.polyline(g, view, pts, c.muted, 1.2, [4, 4]);
      var curve2 = [];
      for (var j = 0; j <= 120; j++) curve2.push(evaluate(pts, weights, j / 120));
      D.polyline(g, view, curve2, c.fg, 2.2);
      D.dot(g, view, evaluate(pts, weights, t), 5, c.red);
      pts.forEach(function (q, i) {
        D.dot(g, view, q, 6, handles && handles.selected === i ? c.accent : c.bg, c.fg);
        D.text(g, view, [q[0] + 0.08, q[1] + 0.08], "w=" + weights[i].toFixed(2), c.muted);
      });
    }
    function paintLift(g, w, hh, c) {
      if (!cam || !read) return;
      var P = cam.project;
      function line(a, b, col, width, dash) {
        var A = P(a), B = P(b);
        D.polyline(g, null, [A, B], col, width, dash);
      }
      var usePts = pts, useW = weights;
      if (mode === "circle") { var sp = arcSpan(0, 360 / spans, 1.6); usePts = sp.points.map(function (q) { return [q[0] + 2.6, q[1] + 1.75]; }); useW = sp.weights; }
      // The plane w = 1, as a square outline.
      [[0, 0], [5.2, 0], [5.2, 3.5], [0, 3.5]].forEach(function (q, i, all) {
        var n = all[(i + 1) % 4];
        line([q[0], q[1], 1], [n[0], n[1], 1], c.faint, 1);
      });
      D.text(g, null, P([5.25, 0, 1]), "w = 1", c.muted);
      var hpts = usePts.map(function (q, i) { return [q[0] * useW[i], q[1] * useW[i], useW[i]]; });
      // The polynomial curve in homogeneous space.
      var curveH = [], curveP = [];
      for (var i = 0; i <= 60; i++) {
        var s = i / 60, a = (1 - s) * (1 - s), b = 2 * s * (1 - s), e = s * s;
        var q = [0, 1, 2].map(function (k) { return a * hpts[0][k] + b * hpts[1][k] + e * hpts[2][k]; });
        curveH.push(P(q));
        curveP.push(P([q[0] / q[2], q[1] / q[2], 1]));
      }
      D.polyline(g, null, hpts.map(P), c.blue, 1.2, [4, 4]);
      D.polyline(g, null, curveH, c.blue, 2.2);
      D.polyline(g, null, curveP, c.fg, 2.2);
      // Rays from the origin through a control point and the current point.
      hpts.forEach(function (q, i) {
        line([0, 0, 0], q, c.faint, 1);
        D.dot(g, null, P(q), 4, c.blue);
        D.dot(g, null, P(lifted(usePts[i], 1)), 3, c.fg);
      });
      var A = (1 - t) * (1 - t), B = 2 * t * (1 - t), E = t * t;
      var cur = [0, 1, 2].map(function (k) { return A * hpts[0][k] + B * hpts[1][k] + E * hpts[2][k]; });
      line([0, 0, 0], cur, c.red, 1.2, [3, 3]);
      D.dot(g, null, P(cur), 4, c.blue);
      D.dot(g, null, P([cur[0] / cur[2], cur[1] / cur[2], 1]), 5, c.red);
      D.dot(g, null, P([0, 0, 0]), 3, c.fg);
      D.text(g, null, P([0, 0, 0]), " origin", c.muted);
    }
    function update() {
      var msg;
      if (mode === "circle") {
        var e = circleError(spans, 1.6, 2000), sp = arcSpan(0, 360 / spans, 1.6);
        msg = spans + " spans of " + (360 / spans) + " degrees, middle weight cos(" + (180 / spans) + " deg) = " + sp.weights[1].toFixed(16) +
          "\nlargest |radius - 1.6| over 2000 samples of each span: " + e.toExponential(2) + " (double precision rounding, no approximation)";
      } else {
        var k = weights[1] * weights[1] / (weights[0] * weights[2]);
        var q = evaluate(pts, weights, t), s = rationalSum(pts, weights, t);
        msg = "w1^2/(w0 w2) = " + k.toFixed(4) + "  so this is part of a " + conicType(weights) +
          "\nC(t) = (" + q[0].toFixed(4) + ", " + q[1].toFixed(4) + ")   rational Bernstein sum (" + s[0].toFixed(4) + ", " + s[1].toFixed(4) + ")";
      }
      read.set(msg);
      stage.redraw(); stage3.redraw();
    }
    var sl = [0, 1, 2].map(function (i) {
      return ctx.slider({ label: "w" + i, min: 0.05, max: 4, step: 0.01, value: weights[i], oninput: function (v) { weights[i] = v; mode = "free"; modeSel.value = "free"; update(); } });
    });
    ctx.slider({ label: "t", min: 0, max: 1, step: 0.001, value: t, oninput: function (v) { t = v; update(); } });
    var modeSel = ctx.select({
      label: "preset",
      options: [["free", "free weights"], ["quarter", "quarter circle"], ["parabola", "parabola"], ["hyperbola", "hyperbola"], ["reparam", "same curve"], ["circle4", "circle, 4 spans"], ["circle3", "circle, 3 spans"]],
      value: "free",
      oninput: function (v) {
        mode = "free"; spans = 4;
        if (v === "quarter") {
          // The kernel's quarter arc: P1 where the tangents meet, w1 = sqrt(2)/2.
          pts[0] = [4.6, 0.5]; pts[1] = [4.6, 2.5]; pts[2] = [2.6, 2.5];
          weights[0] = 1; weights[1] = Math.SQRT1_2; weights[2] = 1;
        } else if (v === "parabola") { weights[0] = 1; weights[1] = 1; weights[2] = 1; }
        else if (v === "hyperbola") { weights[0] = 1; weights[1] = 2; weights[2] = 1; }
        else if (v === "reparam") { weights[0] = 2; weights[1] = Math.SQRT2; weights[2] = 1; }
        else if (v === "circle4") { mode = "circle"; spans = 4; }
        else if (v === "circle3") { mode = "circle"; spans = 3; }
        sl.forEach(function (s, i) { s.value = weights[i]; });
        update();
      }
    });
    update();
  }, { evaluate: evaluate, rationalSum: rationalSum, derivative: derivative, conicType: conicType, arcSpan: arcSpan, circleError: circleError, conicResidual: conicResidual });
})();
