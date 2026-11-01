// Boehm's knot insertion, what it does to the control polygon, and (mode
// "width") what a wide split parameter does to a curve cut many times.
// Pure functions in `algo`.
(function () {
  "use strict";

  // ------------------------------------------------- real-valued B-splines
  function findSpan(p, knots, n, t) {
    if (t >= knots[n + 1]) { for (var k = n; k >= p; k--) if (knots[k] < knots[n + 1]) return k; return p; }
    for (var j = p; j <= n; j++) if (knots[j] <= t && t < knots[j + 1]) return j;
    return p;
  }
  function evalSpline(p, knots, pts, t) {
    var span = findSpan(p, knots, pts.length - 1, t), d = [], j;
    for (j = 0; j <= p; j++) d.push(pts[span - p + j].slice());
    for (var r = 1; r <= p; r++) {
      for (j = p; j >= r; j--) {
        var i = span - p + j, den = knots[i + p - r + 1] - knots[i];
        var a = den > 0 ? (t - knots[i]) / den : 0;
        d[j] = [d[j - 1][0] + a * (d[j][0] - d[j - 1][0]), d[j - 1][1] + a * (d[j][1] - d[j - 1][1])];
      }
    }
    return d[p];
  }
  // Boehm: Q_i = P_i (i <= k-p), a_i P_i + (1-a_i) P_{i-1} (k-p < i <= k), P_{i-1} (i > k).
  function insertKnot(p, knots, pts, t) {
    var k = findSpan(p, knots, pts.length - 1, t), Q = [], alphas = [];
    for (var i = 0; i <= pts.length; i++) {
      if (i <= k - p) Q.push(pts[i].slice());
      else if (i > k) Q.push(pts[i - 1].slice());
      else {
        var a = (t - knots[i]) / (knots[i + p] - knots[i]);
        alphas.push(a);
        Q.push([(1 - a) * pts[i - 1][0] + a * pts[i][0], (1 - a) * pts[i - 1][1] + a * pts[i][1]]);
      }
    }
    var K = knots.slice(0, k + 1).concat([t], knots.slice(k + 1));
    return { knots: K, points: Q, span: k, alphas: alphas };
  }
  // One round of midpoint subdivision: a new knot in the middle of every span.
  function refineAll(p, knots, pts) {
    var K = knots, Q = pts, mids = [];
    for (var i = p; i < pts.length; i++) if (knots[i + 1] > knots[i]) mids.push((knots[i] + knots[i + 1]) / 2);
    mids.forEach(function (m) { var r = insertKnot(p, K, Q, m); K = r.knots; Q = r.points; });
    return { knots: K, points: Q };
  }
  // The strong convex hull: on span k the curve lies in the hull of P_{k-p}..P_k.
  // Here as the box of those points, per non-empty span, and their total area.
  function spanBoxes(p, knots, pts) {
    var out = [];
    for (var k = p; k < pts.length; k++) {
      if (knots[k + 1] <= knots[k]) continue;
      var x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
      for (var i = k - p; i <= k; i++) {
        x0 = Math.min(x0, pts[i][0]); x1 = Math.max(x1, pts[i][0]);
        y0 = Math.min(y0, pts[i][1]); y1 = Math.max(y1, pts[i][1]);
      }
      out.push({ span: k, x0: x0, x1: x1, y0: y0, y1: y1 });
    }
    return out;
  }

  // ------------------------------------------------- outward-rounded intervals
  var buf = new Float64Array(1), bits = new BigInt64Array(buf.buffer);
  function nextUp(x) {
    if (!isFinite(x)) return x;
    if (x === 0) return Number.MIN_VALUE;
    buf[0] = x; bits[0] += x > 0 ? 1n : -1n; return buf[0];
  }
  function nextDown(x) { return -nextUp(-x); }
  // Intervals as [lo, hi], every operation rounded outward by one place, as
  // core/geop-core-math/src/scalars/mod.rs does.
  function I(lo, hi) { return [lo, hi === undefined ? lo : hi]; }
  function add(a, b) { return [nextDown(a[0] + b[0]), nextUp(a[1] + b[1])]; }
  function sub(a, b) { return [nextDown(a[0] - b[1]), nextUp(a[1] - b[0])]; }
  function mul(a, b) {
    var p = [a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]];
    return [nextDown(Math.min.apply(null, p)), nextUp(Math.max.apply(null, p))];
  }
  function div(a, b) {
    if (b[0] <= 0 && b[1] >= 0) throw new Error("division by an interval containing zero");
    return mul(a, [nextDown(1 / b[1]), nextUp(1 / b[0])]);
  }
  function intersect(a, b) {
    var lo = Math.max(a[0], b[0]), hi = Math.min(a[1], b[1]);
    if (lo > hi) return a[1] - a[0] < b[1] - b[0] ? a : b;
    return [lo, hi];
  }
  function width(a) { return a[1] - a[0]; }
  function mid(a) { return (a[0] + a[1]) / 2; }

  // `Scalar::interpolate`: both forms, intersected; `form` picks one to compare.
  function interpolate(a, b, alpha, form) {
    if (a[0] === b[0] && a[1] === b[1] && alpha[0] >= 0 && alpha[1] <= 1 && form !== "delta" && form !== "weights") return a;
    var viaDelta = add(a, mul(alpha, sub(b, a)));
    var viaWeights = add(mul(sub(I(1), alpha), a), mul(alpha, b));
    if (form === "delta") return viaDelta;
    if (form === "weights") return viaWeights;
    return intersect(viaDelta, viaWeights);
  }
  function lerpV(a, b, alpha, form) { return a.map(function (x, c) { return interpolate(x, b[c], alpha, form); }); }

  // A Bézier span on knots [a, b] of homogeneous interval control points.
  function quarterArc(r) {
    var w = Math.SQRT2 / 2;
    return {
      a: I(0), b: I(1),
      points: [
        [I(r), I(0), I(1)],
        [mul(I(r), I(w)), mul(I(r), I(w)), I(w)],
        [I(0), I(r), I(1)]
      ]
    };
  }
  function evalSpan(span, t, form) {
    var alpha = div(sub(t, span.a), sub(span.b, span.a));
    var l = span.points.slice();
    while (l.length > 1) {
      var n = [];
      for (var i = 0; i + 1 < l.length; i++) n.push(lerpV(l[i], l[i + 1], alpha, form));
      l = n;
    }
    return l[0];
  }
  // A piece's derivative by the quotient rule, over t.
  function tangentSpan(span, t, form) {
    var alpha = div(sub(t, span.a), sub(span.b, span.a));
    var s = sub(span.b, span.a), P = span.points;
    var d0 = P[0].map(function (x, c) { return sub(P[1][c], x); });
    var d1 = P[1].map(function (x, c) { return sub(P[2][c], x); });
    var dA = lerpV(d0, d1, alpha, form).map(function (x) { return div(mul(I(2), x), s); });
    var A = evalSpan(span, t, form);
    return [0, 1].map(function (c) { return div(sub(dA[c], mul(dA[2], div(A[c], A[2]))), A[2]); });
  }
  // Where a piece reaches the point of `angle` on the circle: Newton from a sharp
  // seed, as `refine_parameter_at_point`, the last step left unsharpened.
  function parameterAt(span, angle, r, form) {
    var q = [I(r * Math.cos(angle)), I(r * Math.sin(angle))];
    var t = I((mid(span.a) + mid(span.b)) / 2);
    var out = t;
    for (var it = 0; it < 12; it++) {
      var A = evalSpan(span, t, form);
      var c = [div(A[0], A[2]), div(A[1], A[2])];
      var d = tangentSpan(span, t, form);
      var num = add(mul(sub(c[0], q[0]), d[0]), mul(sub(c[1], q[1]), d[1]));
      var den = add(mul(d[0], d[0]), mul(d[1], d[1]));
      out = sub(t, div(num, den));
      t = I(mid(out));
    }
    return out;
  }
  // De Casteljau at alpha = (t - a) / (b - a); the right piece is kept, on [t, b].
  function cutRight(span, t, form) {
    var alpha = div(sub(t, span.a), sub(span.b, span.a));
    var P = span.points;
    var l1 = [lerpV(P[0], P[1], alpha, form), lerpV(P[1], P[2], alpha, form)];
    var m = lerpV(l1[0], l1[1], alpha, form);
    return { a: t, b: span.b, points: [m, l1[1], P[2]] };
  }
  function widest(span) {
    var w = 0;
    span.points.forEach(function (p) { p.forEach(function (x) { w = Math.max(w, width(x)); }); });
    return w;
  }
  // Cuts a quarter arc at `cuts` points along it, keeping the far piece each time,
  // the way a rim is cut by slots. keepWide: cut at the whole refined enclosure.
  // Sharp: at its middle. Returns the widest control point coordinate after each cut.
  function cutManyTimes(cuts, keepWide, form) {
    var span = quarterArc(4), history = [widest(span)];
    for (var k = 1; k <= cuts; k++) {
      var angle = (Math.PI / 2) * k / (cuts + 1);
      try {
        var t = parameterAt(span, angle, 4, form);
        span = cutRight(span, keepWide ? t : I(mid(t)), form);
        history.push(widest(span));
      } catch (e) {
        history.push(Infinity);
        for (; k < cuts; k++) history.push(Infinity);
        break;
      }
    }
    return history;
  }

  // ----------------------------------------------------------------- demos
  function insertDemo(ctx) {
    var D = GeopBook.draw;
    var p = 3;
    var start = [[0.4, 0.5], [1.2, 3.2], [2.8, 0.6], [3.6, 3.4], [5.1, 3.3], [5.8, 0.8]];
    var knots0 = [0, 0, 0, 0, 1 / 3, 2 / 3, 1, 1, 1, 1];
    var base = start.map(function (q) { return q.slice(); });
    var state = { knots: knots0.slice(), points: base.map(function (q) { return q.slice(); }) };
    var t = 0.45, showBoxes = true, lastOld = null;

    var stage = ctx.stage({ aspect: 1.6, maxHeight: 380, label: "A cubic B-spline and its control polygon, refined by knot insertion", draw: paint });
    var view = ctx.view(stage, { x0: 0, x1: 6.2, y0: 0.1, y1: 3.8 });
    var hd = ctx.handles({
      stage: stage, view: view, pts: base, active: function () { return state.points.length === base.length; },
      onchange: function () { state.points = base.map(function (q) { return q.slice(); }); state.knots = knots0.slice(); lastOld = null; update(); }
    });
    var read = ctx.readout();

    function curveDistance() {
      // Greatest distance from a control point of the refined polygon to the curve:
      // a sampled measure, for the readout only: to the chords between the samples,
      // since the samples themselves lie further apart than a refined polygon does
      // from the curve.
      var curve = [], i;
      for (i = 0; i <= 400; i++) curve.push(evalSpline(p, state.knots, state.points, i / 400));
      var worst = 0;
      state.points.forEach(function (q) {
        var best = 1e30;
        for (var s = 0; s < 400; s++) {
          var a = curve[s], dx = curve[s + 1][0] - a[0], dy = curve[s + 1][1] - a[1], len = dx * dx + dy * dy;
          var f = len > 0 ? Math.max(0, Math.min(1, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / len)) : 0;
          best = Math.min(best, Math.hypot(a[0] + f * dx - q[0], a[1] + f * dy - q[1]));
        }
        worst = Math.max(worst, best);
      });
      return worst;
    }
    function paint(g, w, h, c) {
      if (!view || !read) return;
      view.update();
      if (showBoxes) {
        spanBoxes(p, state.knots, state.points).forEach(function (b, i) {
          D.polygon(g, view, [[b.x0, b.y0], [b.x1, b.y0], [b.x1, b.y1], [b.x0, b.y1]], c.fillA, c.faint, 1);
        });
      }
      if (lastOld) D.polyline(g, view, lastOld, c.muted, 1, [2, 4]);
      D.polyline(g, view, state.points, c.orange, 1.5);
      var curve = [];
      for (var i = 0; i <= 200; i++) curve.push(evalSpline(p, state.knots, state.points, i / 200));
      D.polyline(g, view, curve, c.fg, 2.2);
      state.points.forEach(function (q) { D.dot(g, view, q, 3.2, c.orange); });
      if (state.points.length === base.length) {
        base.forEach(function (q, i) { D.dot(g, view, q, 6, hd && hd.selected === i ? c.accent : c.bg, c.fg); });
      }
      var on = evalSpline(p, state.knots, state.points, t);
      D.dot(g, view, on, 4.5, c.red);
    }
    function update(extra) {
      var boxes = spanBoxes(p, state.knots, state.points), area = 0;
      boxes.forEach(function (b) { area += (b.x1 - b.x0) * (b.y1 - b.y0); });
      read.set(state.points.length + " control points, " + boxes.length + " spans.  " +
        "Summed area of the spans' boxes: " + area.toFixed(3) + ".  Farthest control point from the curve: " + curveDistance().toFixed(4) + "." +
        (extra ? "\n" + extra : ""));
      stage.redraw();
    }
    ctx.slider({ label: "t", min: 0.01, max: 0.99, step: 0.001, value: t, oninput: function (v) { t = v; stage.redraw(); } });
    ctx.button("Insert at t", function () {
      if (state.points.length > 120) return;
      // A knot goes up to full multiplicity at most (degree + 1 copies would split it).
      var mult = state.knots.filter(function (k) { return k === t; }).length;
      if (mult >= p) { update("t already has multiplicity " + mult + ": the curve passes through a control point there."); return; }
      lastOld = state.points.map(function (q) { return q.slice(); });
      var r = insertKnot(p, state.knots, state.points, t);
      state.knots = r.knots; state.points = r.points;
      update("Inserted t = " + t.toFixed(3) + " in span " + r.span + ", alphas " + r.alphas.map(function (a) { return a.toFixed(3); }).join(", ") + ".");
    });
    ctx.button("Halve every span", function () {
      if (state.points.length > 120) return;
      lastOld = state.points.map(function (q) { return q.slice(); });
      var r = refineAll(p, state.knots, state.points);
      state.knots = r.knots; state.points = r.points;
      update();
    });
    ctx.button("Reset", function () {
      state.points = base.map(function (q) { return q.slice(); }); state.knots = knots0.slice(); lastOld = null; update();
    });
    ctx.checkbox({ label: "boxes of each span's points", value: true, oninput: function (v) { showBoxes = v; stage.redraw(); } });
    update();
  }

  function widthDemo(ctx) {
    var D = GeopBook.draw;
    var cuts = 40, form = "both", histories = null;
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 340, label: "Width of a quarter arc's control points against the number of cuts, on a logarithmic scale", draw: paint });
    var read = ctx.readout();
    function compute() {
      histories = { wide: cutManyTimes(cuts, true, form), sharp: cutManyTimes(cuts, false, form) };
      var w = histories.wide[cuts], s = histories.sharp[cuts];
      read.set("after " + cuts + " cuts, widest control point coordinate:\n  cut at the whole enclosure: " + (isFinite(w) ? w.toExponential(2) : "the search broke down") +
        "\n  cut at its sharp middle:    " + s.toExponential(2));
      stage.redraw();
    }
    function paint(g, w, h, c) {
      if (!histories) return;
      var L = 42, R = 10, T = 10, B = 26;
      var ymin = -17, ymax = 3;
      function X(k) { return L + (w - L - R) * k / cuts; }
      function Y(v) { var l = Math.log10(Math.max(v, 1e-30)); return T + (h - T - B) * (ymax - Math.max(ymin, Math.min(ymax, l))) / (ymax - ymin); }
      g.strokeStyle = c.grid; g.lineWidth = 1;
      for (var e = ymin + 1; e <= ymax; e += 4) {
        g.beginPath(); g.moveTo(L, Y(Math.pow(10, e))); g.lineTo(w - R, Y(Math.pow(10, e))); g.stroke();
        D.text(g, null, [L - 4, Y(Math.pow(10, e)) + 4], "1e" + e, c.muted, "right");
      }
      D.text(g, null, [(L + w - R) / 2, h - 8], "cuts", c.muted, "center");
      function line(hist, col) {
        var pts = [];
        for (var k = 0; k < hist.length; k++) { if (!isFinite(hist[k])) break; pts.push([X(k), Y(hist[k])]); }
        D.polyline(g, null, pts, col, 2);
      }
      line(histories.wide, c.red);
      line(histories.sharp, c.blue);
      D.text(g, null, [L + 8, T + 14], "cut at the whole enclosure", c.red);
      D.text(g, null, [L + 8, T + 30], "cut at its sharp middle", c.blue);
    }
    ctx.slider({ label: "cuts", min: 5, max: 60, step: 1, value: cuts, format: function (v) { return String(v); }, oninput: function (v) { cuts = v; compute(); } });
    ctx.select({
      label: "interpolate",
      options: [["both", "both forms, intersected"], ["delta", "a + α(b−a) only"], ["weights", "(1−α)a + αb only"]], value: "both",
      oninput: function (v) { form = v; compute(); }
    });
    compute();
  }

  GeopBook.demo("knot-insertion", function (ctx) {
    if (ctx.args.mode === "width") widthDemo(ctx); else insertDemo(ctx);
  }, {
    insertKnot: insertKnot, evalSpline: evalSpline, refineAll: refineAll, spanBoxes: spanBoxes,
    nextUp: nextUp, nextDown: nextDown, I: I, add: add, sub: sub, mul: mul, div: div, interpolate: interpolate,
    quarterArc: quarterArc, evalSpan: evalSpan, cutRight: cutRight, parameterAt: parameterAt, cutManyTimes: cutManyTimes, widest: widest
  });
})();
