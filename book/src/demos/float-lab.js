// Two ways rounding bites, measured in this browser's real floats: the order of a
// sum, and cancellation. Single precision is real too (Math.fround on every step).
(function () {
  "use strict";
  function round(prec, v) { return prec === "single" ? Math.fround(v) : v; }

  // 1 + 1/2 + ... + 1/N, largest term first.
  function forward(N, prec) {
    var s = 0;
    for (var n = 1; n <= N; n++) s = round(prec, s + round(prec, 1 / n));
    return s;
  }
  // The same terms, smallest first.
  function backward(N, prec) {
    var s = 0;
    for (var n = N; n >= 1; n--) s = round(prec, s + round(prec, 1 / n));
    return s;
  }
  // Neumaier's compensated sum in f64, the reference: it carries each addition's
  // rounding error, computed exactly by the two-sum identity.
  function compensated(N) {
    var s = 0, c = 0;
    for (var n = 1; n <= N; n++) {
      var v = 1 / n, t = s + v;
      if (Math.abs(s) >= Math.abs(v)) c += (s - t) + v; else c += (v - t) + s;
      s = t;
    }
    return s + c;
  }
  // (1 - cos x) / x^2, which tends to 1/2, as written ...
  function naive(x) { return (1 - Math.cos(x)) / (x * x); }
  // ... and with the half-angle identity, which subtracts nothing.
  function stable(x) { var s = Math.sin(x / 2); return 2 * s * s / (x * x); }

  GeopBook.demo("float-lab", function (ctx) {
    var D = GeopBook.draw;
    var lab = ctx.args.lab || "cancel";
    var prec = "single", sumCache = {}, x = 1e-6;
    var stage = ctx.stage({ aspect: 1.8, maxHeight: 340, label: "Measured rounding errors", draw: paint });
    var read = ctx.readout();

    function axes(g, w, h, c, xl, yl, yLo, yHi, xLo, xHi) {
      var pl = 44, pr = w - 10, pt = 12, pb = h - 26;
      var map = {
        x: function (v) { return pl + (v - xLo) / (xHi - xLo) * (pr - pl); },
        y: function (v) { return pb - (v - yLo) / (yHi - yLo) * (pb - pt); }
      };
      g.strokeStyle = c.grid; g.lineWidth = 1; g.beginPath();
      var i;
      for (i = Math.ceil(yLo); i <= yHi; i += 2) { g.moveTo(pl, map.y(i)); g.lineTo(pr, map.y(i)); }
      g.stroke();
      g.font = "11px sans-serif";
      for (i = Math.ceil(yLo); i <= yHi; i += 4) D.text(g, null, [pl - 4, map.y(i) + 4], "1e" + i, c.muted, "right");
      for (i = Math.ceil(xLo); i <= xHi; i += (xHi - xLo > 6 ? 2 : 1)) D.text(g, null, [map.x(i), pb + 14], "1e" + i, c.muted, "center");
      D.text(g, null, [pr, pt + 10], yl + " against " + xl, c.muted, "right");
      return map;
    }
    function paint(g, w, h, c) {
      if (lab === "sum") paintSum(g, w, h, c); else paintCancel(g, w, h, c);
    }
    function sums() {
      if (sumCache[prec]) return sumCache[prec];
      var out = [], Ns = [];
      for (var l = 1; l <= 6.001; l += 0.25) Ns.push(Math.round(Math.pow(10, l)));
      Ns.forEach(function (N) {
        var ref = compensated(N);
        out.push({ N: N, f: Math.abs(forward(N, prec) - ref) / ref, b: Math.abs(backward(N, prec) - ref) / ref, ref: ref, fv: forward(N, prec), bv: backward(N, prec) });
      });
      return (sumCache[prec] = out);
    }
    function paintSum(g, w, h, c) {
      var data = sums();
      var yLo = prec === "single" ? -9 : -18, yHi = prec === "single" ? -2 : -12;
      var m = axes(g, w, h, c, "N", "relative error", yLo, yHi, 1, 6);
      function line(key, col) {
        var pts = data.map(function (d) { var e = Math.max(d[key], Math.pow(10, yLo)); return [m.x(Math.log10(d.N)), m.y(Math.log10(e))]; });
        D.polyline(g, null, pts, col, 2);
        pts.forEach(function (p) { D.dot(g, null, p, 2.5, col); });
      }
      line("f", c.red); line("b", c.green);
      D.text(g, null, [m.x(1.1), m.y(yHi) + 22], "largest first", c.red);
      D.text(g, null, [m.x(1.1), m.y(yHi) + 36], "smallest first", c.green);
    }
    function paintCancel(g, w, h, c) {
      var m = axes(g, w, h, c, "x", "relative error", -18, 1, -10, 0);
      var pts = [], qts = [];
      for (var i = 0; i <= 600; i++) {
        var l = -10 + 10 * i / 600, xv = Math.pow(10, l);
        // The series 1/2 - x^2/24 + x^4/720 - x^6/40320, with terms enough where it is drawn.
        var x2 = xv * xv, truth = 0.5 - x2 / 24 + x2 * x2 / 720 - x2 * x2 * x2 / 40320;
        var e1 = Math.abs(naive(xv) - stable(xv)) / stable(xv);
        var e2 = Math.abs(stable(xv) - truth) / truth;
        pts.push([m.x(l), m.y(Math.log10(Math.max(e1, 1e-18)))]);
        if (l < -0.5) qts.push([m.x(l), m.y(Math.log10(Math.max(e2, 1e-18)))]);
      }
      D.polyline(g, null, pts, c.red, 1);
      D.polyline(g, null, qts, c.green, 1.5);
      var lx = Math.log10(x);
      g.strokeStyle = c.muted; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(m.x(lx), m.y(1)); g.lineTo(m.x(lx), m.y(-18)); g.stroke(); g.setLineDash([]);
      D.text(g, null, [m.x(-3), m.y(1) + 22], "(1 - cos x) / x^2", c.red);
      D.text(g, null, [m.x(-3), m.y(1) + 36], "2 sin^2(x/2) / x^2", c.green);
    }
    function update() {
      if (lab === "sum") {
        var d = sums(), last = d[d.length - 1];
        read.set("N = " + last.N + ", in " + (prec === "single" ? "f32" : "f64") + "\nlargest first:  " + last.fv + "\nsmallest first: " + last.bv + "\nreference:      " + last.ref);
      } else {
        read.set("x = " + x.toExponential(2) + "\n(1 - cos x) / x^2  = " + naive(x) + "\n2 sin^2(x/2) / x^2 = " + stable(x));
      }
      stage.redraw();
    }
    if (lab === "sum") {
      ctx.select({ label: "format", options: [["single", "f32 (single)"], ["double", "f64 (double)"]], value: prec, oninput: function (v) { prec = v; update(); } });
    } else {
      ctx.slider({ label: "log10 x", min: -10, max: 0, step: 0.01, value: -6, oninput: function (v) { x = Math.pow(10, v); update(); } });
    }
    update();
  }, { forward: forward, backward: backward, compensated: compensated, naive: naive, stable: stable });
})();
