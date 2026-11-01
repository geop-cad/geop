// Newton's method for (x + s)^3 = c in outward-rounded intervals, where c is
// (b + s)^3 for a boundary value b, so the true answer lies exactly on the boundary,
// and the data c carries the rounding of how it was computed. What the iteration
// keeps, and what it returns, decides how the answer is classified.
// The arithmetic is that of core/geop-core-math/src/scalars/mod.rs; the loop
// is a one-dimensional picture of NurbSurface::project and predictor_corrector_step.
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
  function add(a, b) { return [dn(a[0] + b[0]), up(a[1] + b[1])]; }
  function sub(a, b) { return [dn(a[0] - b[1]), up(a[1] - b[0])]; }
  function mul(a, b) {
    var p = [a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]];
    return [dn(Math.min(p[0], p[1], p[2], p[3])), up(Math.max(p[0], p[1], p[2], p[3]))];
  }
  function div(a, b) {
    if (b[0] <= 0 && b[1] >= 0) return null;
    return mul(a, [dn(1 / b[1]), up(1 / b[0])]);
  }
  function sharpen(a) { var m = (a[0] + a[1]) / 2; return [m, m]; }
  function cube(x) { return mul(mul(x, x), x); }

  // A shift no float sum with b is exact for, so that c is not exactly known.
  var S = [0.3, 0.3];
  // One Newton step for (x + s)^3 - c: x - ((x + s)^3 - c) / (3 (x + s)^2). Null where it fails.
  function step(x, c) {
    var y = add(x, S);
    var q = div(sub(cube(y), c), mul([3, 3], mul(y, y)));
    return q ? sub(x, q) : null;
  }

  // strategy "wide": every iterate seeds the next as it is.
  // strategy "sharp": every iterate is sharpened, the one returned too.
  // strategy "two": the seed is sharpened, the returned value is the update as computed.
  function solve(b, strategy, steps) {
    var c = cube(add([b, b], S));
    var seed = [b * 1.1, b * 1.1], answer = seed, widths = [], failedAt = -1;
    for (var k = 0; k < steps; k++) {
      var next = step(seed, c);
      if (!next) { failedAt = k; break; }
      if (strategy === "wide") { seed = next; answer = next; }
      else if (strategy === "sharp") { seed = sharpen(next); answer = seed; }
      else { seed = sharpen(next); answer = next; }
      widths.push(answer[1] - answer[0]);
    }
    return { answer: answer, widths: widths, failedAt: failedAt };
  }
  // Against a boundary that is the sharp value b, three-valued, as face_contains asks.
  function classify(answer, b) {
    if (answer[1] < b) return "inside";
    if (answer[0] > b) return "outside";
    return "on boundary";
  }
  // A fixed, repeatable set of boundaries in [0.5, 2).
  function boundaries(n) {
    var out = [], s = 0.5;
    for (var i = 0; i < n; i++) { s = (s + 0.6180339887498949) % 1; out.push(0.5 + 1.5 * s); }
    return out;
  }
  function survey(n, strategy, steps) {
    var r = { wrong: 0, contains: 0, n: n, failed: 0 };
    boundaries(n).forEach(function (b) {
      var s = solve(b, strategy, steps);
      if (s.failedAt >= 0) { r.failed++; return; }
      if (classify(s.answer, b) !== "on boundary") r.wrong++;
      if (s.answer[0] <= b && b <= s.answer[1]) r.contains++;
    });
    return r;
  }

  GeopBook.demo("sharpen-trap", function (ctx) {
    var D = GeopBook.draw;
    var b = 0.7, steps = 20;
    var names = { wide: "seed as it is", sharp: "sharpen everything", two: "two copies" };
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 420, label: "Newton's iterates' widths and the final answers against the boundary", draw: paint });
    var read = ctx.readout();
    function colorOf(s, c) { return s === "wide" ? c.orange : s === "sharp" ? c.red : c.green; }

    function paint(g, w, h, c) {
      // Top: the width of each iterate, log scale.
      var pl = 42, pr = w - 10, pt = 14, pb = Math.round(h * 0.56);
      var yLo = -17, yHi = 1, maxK = 60;
      function px(k) { return pl + (k / maxK) * (pr - pl); }
      function py(l) { return pb - (l - yLo) / (yHi - yLo) * (pb - pt); }
      g.strokeStyle = c.grid; g.lineWidth = 1; g.beginPath();
      for (var l = yLo + 1; l <= yHi; l += 4) { g.moveTo(pl, py(l)); g.lineTo(pr, py(l)); }
      g.stroke();
      g.font = "11px sans-serif";
      for (l = yLo + 1; l <= yHi; l += 4) D.text(g, null, [pl - 4, py(l) + 4], "1e" + l, c.muted, "right");
      D.text(g, null, [pr, pb + 13], "iteration (to " + maxK + ")", c.muted, "right");
      D.text(g, null, [pl + 2, pt - 2], "width of the iterate", c.muted);
      ["wide", "two"].forEach(function (s) {
        var r = solve(b, s, maxK), pts = r.widths.map(function (v, k) { return [px(k + 1), py(Math.log10(Math.max(v, 1e-17)))]; });
        D.polyline(g, null, pts, colorOf(s, c), 2);
        if (r.failedAt >= 0 && pts.length) D.dot(g, null, pts[pts.length - 1], 5, null, colorOf(s, c));
      });
      g.strokeStyle = c.muted; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(px(steps), pt); g.lineTo(px(steps), pb); g.stroke(); g.setLineDash([]);

      // Bottom: the answers around b, in ulps of b.
      var y0 = pb + 34, unit = up(Math.abs(b)) - Math.abs(b), span = 16;
      function sx(v) { return pl + (((v - b) / unit + span) / (2 * span)) * (pr - pl); }
      g.strokeStyle = c.fg; g.lineWidth = 2; g.beginPath(); g.moveTo(sx(b), y0 - 12); g.lineTo(sx(b), h - 8); g.stroke();
      D.text(g, null, [sx(b), y0 - 15], "boundary b", c.fg, "center");
      ["sharp", "two", "wide"].forEach(function (s, i) {
        var r = solve(b, s, steps), y = y0 + 10 + i * 24, col = colorOf(s, c);
        var lo = Math.max(pl, Math.min(pr, sx(r.answer[0]))), hi = Math.max(pl, Math.min(pr, sx(r.answer[1])));
        g.strokeStyle = col; g.lineWidth = 5; g.lineCap = "butt"; g.beginPath(); g.moveTo(lo - 1.5, y); g.lineTo(hi + 1.5, y); g.stroke(); g.lineCap = "round";
        D.text(g, null, [pl, y - 6], names[s], col);
      });
    }
    function line(s) {
      var r = solve(b, s, steps), v = survey(1000, s, steps), a = r.answer;
      return names[s] + ": " + (r.failedAt >= 0 ? "failed, a divisor could be zero" : classify(a, b) + ", width " + (a[1] - a[0]).toExponential(1)) +
        "\n  " + v.wrong + " of 1000 misclassified, " + v.failed + " failed";
    }
    function update() {
      read.set("b = " + b + ", after " + steps + " steps. The truth: on the boundary.\n" + line("sharp") + "\n" + line("two") + "\n" + line("wide"));
      stage.redraw();
    }
    ctx.slider({ label: "b", min: 0.5, max: 2, step: 0.001, value: b, format: function (v) { return v.toFixed(3); }, oninput: function (v) { b = v; update(); } });
    ctx.slider({ label: "steps", min: 1, max: 60, step: 1, value: steps, format: function (v) { return String(v); }, oninput: function (v) { steps = v; update(); } });
    update();
  }, { solve: solve, classify: classify, survey: survey, boundaries: boundaries, step: step });
})();
