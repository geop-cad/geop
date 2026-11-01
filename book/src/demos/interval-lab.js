// A function over an interval: its true range against what interval arithmetic
// makes of it, naively, by the mean value form, and piece by piece. The arithmetic
// is that of core/geop-core-math/src/scalars/mod.rs: nearest rounding, then one ulp
// outward, in the f64 of this browser.
(function () {
  "use strict";
  var f64 = new Float64Array(1), b64 = new BigUint64Array(f64.buffer);
  function up(x) {
    if (x === Infinity || x !== x) return x;
    if (x === 0) return 5e-324;
    f64[0] = x; b64[0] = x > 0 ? b64[0] + 1n : b64[0] - 1n;
    return f64[0];
  }
  function dn(x) { return -up(-x); }
  function pt(x) { return [x, x]; }
  function add(a, b) { return [dn(a[0] + b[0]), up(a[1] + b[1])]; }
  function sub(a, b) { return [dn(a[0] - b[1]), up(a[1] - b[0])]; }
  function mul(a, b) {
    var p = [a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]];
    return [dn(Math.min(p[0], p[1], p[2], p[3])), up(Math.max(p[0], p[1], p[2], p[3]))];
  }
  function div(a, b) {
    if (b[0] <= 0 && b[1] >= 0) throw new Error("divisor interval contains zero");
    return mul(a, [dn(1 / b[1]), up(1 / b[0])]);
  }
  function hasPeriodic(lo, hi, target) {
    var T = 2 * Math.PI, k = Math.ceil((lo - target) / T);
    return target + k * T <= hi;
  }
  function trig(x, f, maxAt, minAt) {
    if (x[1] - x[0] >= 2 * Math.PI) return [-1, 1];
    var a = f(x[0]), b = f(x[1]);
    var lo = dn(Math.min(a, b)), hi = up(Math.max(a, b));
    if (hasPeriodic(x[0], x[1], maxAt)) hi = 1;
    if (hasPeriodic(x[0], x[1], minAt)) lo = -1;
    return [Math.max(lo, -1), Math.min(hi, 1)];
  }
  function isin(x) { return trig(x, Math.sin, Math.PI / 2, -Math.PI / 2); }
  function icos(x) { return trig(x, Math.cos, 0, Math.PI); }
  var ONE = pt(1), TWO = pt(2), THREE = pt(3);

  // Each: its name, the expression in intervals as a programmer writes it, its
  // derivative, the same in plain numbers, the zeros of the derivative, the view.
  var fns = {
    self: { name: "x - x", f: function (X) { return sub(X, X); }, df: function () { return pt(0); }, g: function () { return 0; }, crit: function () { return []; }, dom: [-2, 2], y: [-2, 2] },
    parabola: { name: "x (1 - x)", f: function (X) { return mul(X, sub(ONE, X)); }, df: function (X) { return sub(ONE, mul(TWO, X)); }, g: function (x) { return x * (1 - x); }, crit: function () { return [0.5]; }, dom: [-0.5, 1.5], y: [-0.9, 0.9] },
    cubic: { name: "x^3 - 2x", f: function (X) { return sub(mul(mul(X, X), X), mul(TWO, X)); }, df: function (X) { return sub(mul(THREE, mul(X, X)), TWO); }, g: function (x) { return x * x * x - 2 * x; }, crit: function () { var r = Math.sqrt(2 / 3); return [-r, r]; }, dom: [-2, 2], y: [-5, 5] },
    ratio: { name: "x / (1 + x)", f: function (X) { return div(X, add(ONE, X)); }, df: function (X) { var w = add(ONE, X); return div(ONE, mul(w, w)); }, g: function (x) { return x / (1 + x); }, crit: function () { return []; }, dom: [0.1, 8], y: [-0.2, 1.6] },
    wave: { name: "sin x cos x", f: function (X) { return mul(isin(X), icos(X)); }, df: function (X) { var s = isin(X), c = icos(X); return sub(mul(c, c), mul(s, s)); }, g: function (x) { return Math.sin(x) * Math.cos(x); }, crit: function (lo, hi) { var o = []; for (var k = Math.ceil((lo - Math.PI / 4) / (Math.PI / 2)); Math.PI / 4 + k * Math.PI / 2 <= hi; k++) o.push(Math.PI / 4 + k * Math.PI / 2); return o; }, dom: [-3, 3], y: [-1.3, 1.3] }
  };
  // The range, from the ends and the zeros of the derivative inside: not a sample.
  function trueRange(fn, X) {
    var lo = Math.min(fn.g(X[0]), fn.g(X[1])), hi = Math.max(fn.g(X[0]), fn.g(X[1]));
    fn.crit(X[0], X[1]).forEach(function (c) {
      if (c >= X[0] && c <= X[1]) { var v = fn.g(c); lo = Math.min(lo, v); hi = Math.max(hi, v); }
    });
    return [lo, hi];
  }
  function naive(fn, X) { return fn.f(X); }
  // f(c) + f'(X) (X - c), with c the middle of X.
  function meanValue(fn, X) {
    var c = pt((X[0] + X[1]) / 2);
    return add(fn.f(c), mul(fn.df(X), sub(X, c)));
  }
  function pieces(fn, X, n, form) {
    var out = [], lo = X[0];
    for (var i = 1; i <= n; i++) {
      var hi = i === n ? X[1] : X[0] + (X[1] - X[0]) * i / n;
      out.push({ X: [lo, hi], Y: (form || naive)(fn, [lo, hi]) });
      lo = hi;
    }
    return out;
  }
  function hull(list) {
    var lo = Infinity, hi = -Infinity;
    list.forEach(function (p) { lo = Math.min(lo, p.Y[0]); hi = Math.max(hi, p.Y[1]); });
    return [lo, hi];
  }

  GeopBook.demo("interval-lab", function (ctx) {
    var D = GeopBook.draw;
    var key = ctx.args.fn || "parabola", fn = fns[key];
    var centre = (fn.dom[0] + fn.dom[1]) / 2, half = (fn.dom[1] - fn.dom[0]) / 16, n = 4;
    if (key === "parabola") { centre = 0.5; half = 0.1; }
    var stage = ctx.stage({ aspect: 1.15, maxHeight: 460, label: "A function, an input interval and its enclosures", draw: paint });
    var read = ctx.readout();
    function X() {
      var lo = Math.max(fn.dom[0], centre - half), hi = Math.min(fn.dom[1], centre + half);
      return [lo, hi];
    }
    function paint(g, w, h, c) {
      var I = X();
      var bars = [
        { name: "true range", Y: trueRange(fn, I), color: c.fg },
        { name: "naive", Y: naive(fn, I), color: c.red },
        { name: "mean value", Y: meanValue(fn, I), color: c.blue },
        { name: n + (n === 1 ? " piece" : " pieces"), Y: hull(pieces(fn, I, n)), color: c.green }
      ];
      // Top: the curve over the whole domain, the input shaded.
      var top = 8, bottom = Math.round(h * 0.6), left = 10, right = w - 10;
      function px(x) { return left + (x - fn.dom[0]) / (fn.dom[1] - fn.dom[0]) * (right - left); }
      function py(y) { return bottom - (clip(y) - fn.y[0]) / (fn.y[1] - fn.y[0]) * (bottom - top); }
      g.fillStyle = c.grid;
      g.fillRect(px(I[0]), top, px(I[1]) - px(I[0]), bottom - top);
      if (fn.y[0] < 0) D.polyline(g, null, [[left, py(0)], [right, py(0)]], c.faint, 1);
      var pts = [];
      for (var i = 0; i <= 300; i++) { var x = fn.dom[0] + (fn.dom[1] - fn.dom[0]) * i / 300; pts.push([px(x), py(fn.g(x))]); }
      D.polyline(g, null, pts, c.fg, 2);
      pieces(fn, I, n).forEach(function (p) {
        D.polygon(g, null, [[px(p.X[0]), py(p.Y[0])], [px(p.X[1]), py(p.Y[0])], [px(p.X[1]), py(p.Y[1])], [px(p.X[0]), py(p.Y[1])]], c.fillC, c.green, 1);
      });
      // Bottom: the four enclosures on one scale, fitted to the widest.
      var lo = Infinity, hi = -Infinity;
      bars.forEach(function (b) { lo = Math.min(lo, b.Y[0]); hi = Math.max(hi, b.Y[1]); });
      var pad = Math.max((hi - lo) * 0.08, 1e-12);
      lo -= pad; hi += pad;
      var y0 = bottom + 26, rowH = (h - y0 - 4) / bars.length;
      function bx(v) { return left + (v - lo) / (hi - lo) * (right - left); }
      g.font = "11px sans-serif";
      bars.forEach(function (b, k) {
        var y = y0 + k * rowH + rowH * 0.62;
        g.strokeStyle = b.color; g.lineWidth = 6; g.lineCap = "butt"; g.beginPath();
        g.moveTo(bx(b.Y[0]), y); g.lineTo(Math.max(bx(b.Y[1]), bx(b.Y[0]) + 2), y); g.stroke();
        g.lineCap = "round";
        D.text(g, null, [left, y - 6], b.name, b.color);
      });
      D.text(g, null, [right, bottom + 14], "from " + lo.toPrecision(4) + " to " + hi.toPrecision(4), c.muted, "right");
    }
    function clip(v) { return Math.max(fn.y[0], Math.min(fn.y[1], v)); }
    function fmt(Y) { return "[" + Y[0].toFixed(4) + ", " + Y[1].toFixed(4) + "] w=" + (Y[1] - Y[0]).toExponential(1); }
    function update() {
      var I = X();
      read.set(fn.name + " over [" + I[0].toFixed(4) + ", " + I[1].toFixed(4) + "]\n" +
        "true  " + fmt(trueRange(fn, I)) + "\n" +
        "naive " + fmt(naive(fn, I)) + "\n" +
        "mean  " + fmt(meanValue(fn, I)) + "\n" +
        "cut   " + fmt(hull(pieces(fn, I, n))));
      stage.redraw();
    }
    ctx.select({
      label: "f(x) =", value: key,
      options: Object.keys(fns).map(function (k) { return [k, fns[k].name]; }),
      oninput: function (v) {
        key = v; fn = fns[v];
        var span = fn.dom[1] - fn.dom[0];
        centre = (fn.dom[0] + fn.dom[1]) / 2;
        half = span / 16;
        cs.el.min = fn.dom[0]; cs.el.max = fn.dom[1]; cs.el.step = span / 400; cs.value = centre;
        ws.el.max = span / 2; ws.el.min = span / 2000; ws.el.step = span / 2000; ws.value = half;
        update();
      }
    });
    var cs = ctx.slider({ label: "centre", min: fn.dom[0], max: fn.dom[1], step: (fn.dom[1] - fn.dom[0]) / 400, value: centre, oninput: function (v) { centre = v; update(); } });
    var ws = ctx.slider({ label: "half width", min: (fn.dom[1] - fn.dom[0]) / 2000, max: (fn.dom[1] - fn.dom[0]) / 2, step: (fn.dom[1] - fn.dom[0]) / 2000, value: half, format: function (v) { return v.toFixed(3); }, oninput: function (v) { half = v; update(); } });
    ctx.slider({ label: "pieces", min: 1, max: 32, step: 1, value: n, format: function (v) { return String(v); }, oninput: function (v) { n = v; update(); } });
    update();
  }, { fns: fns, trueRange: trueRange, naive: naive, meanValue: meanValue, pieces: pieces, hull: hull, add: add, sub: sub, mul: mul, div: div, up: up, dn: dn });
})();
