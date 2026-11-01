// The orientation of three nearly collinear points, for a grid of p one ulp
// apart: as f64 computes it, as exact rational arithmetic (BigInt) has it, and as
// interval arithmetic rounded outward, the way Geop's Scalar does, can vouch for it.
(function () {
  "use strict";
  var f64 = new Float64Array(1), b64 = new BigUint64Array(f64.buffer);
  function nextUp(x) {
    if (x === Infinity || x !== x) return x;
    if (x === 0) return 5e-324;
    f64[0] = x; b64[0] = x > 0 ? b64[0] + 1n : b64[0] - 1n;
    return f64[0];
  }
  function nextDown(x) { return -nextUp(-x); }

  // A double as an exact integer times 2^exp.
  function exact(x) {
    f64[0] = x;
    var bits = b64[0];
    var sign = (bits >> 63n) ? -1n : 1n;
    var e = Number((bits >> 52n) & 0x7ffn);
    var frac = bits & 0xfffffffffffffn;
    if (e === 0) return { m: sign * frac, e: -1074 };
    return { m: sign * (frac | (1n << 52n)), e: e - 1075 };
  }
  // The exact sign of (q - p) x (r - p), all coordinates brought over one power of two.
  function orientExact(p, q, r) {
    var c = [p[0], p[1], q[0], q[1], r[0], r[1]].map(exact);
    var low = Math.min.apply(null, c.map(function (v) { return v.e; }));
    var n = c.map(function (v) { return v.m << BigInt(v.e - low); });
    var d = (n[2] - n[0]) * (n[5] - n[1]) - (n[3] - n[1]) * (n[4] - n[0]);
    return d > 0n ? 1 : d < 0n ? -1 : 0;
  }
  // As anyone would write it.
  function orientFloat(p, q, r) {
    var d = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    return d > 0 ? 1 : d < 0 ? -1 : 0;
  }
  // The same in outward-rounded intervals: 1 or -1 where the sign is proven, 0 "cannot tell".
  function isub(a, b) { return [nextDown(a[0] - b[1]), nextUp(a[1] - b[0])]; }
  function imul(a, b) {
    var v = [a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]];
    return [nextDown(Math.min(v[0], v[1], v[2], v[3])), nextUp(Math.max(v[0], v[1], v[2], v[3]))];
  }
  function orientInterval(p, q, r) {
    function pt(x) { return [x, x]; }
    var d = isub(
      imul(isub(pt(q[0]), pt(p[0])), isub(pt(r[1]), pt(p[1]))),
      imul(isub(pt(q[1]), pt(p[1])), isub(pt(r[0]), pt(p[0])))
    );
    return d[0] > 0 ? 1 : d[1] < 0 ? -1 : 0;
  }
  // p moved i ulps in x and j ulps in y.
  function offset(p, i, j) {
    var x = p[0], y = p[1], k;
    for (k = 0; k < i; k++) x = nextUp(x);
    for (k = 0; k < j; k++) y = nextUp(y);
    return [x, y];
  }
  function grid(p, q, r, n, fn) {
    var out = [], y = p[1];
    for (var j = 0; j < n; j++) {
      var row = [], x = p[0];
      for (var i = 0; i < n; i++) { row.push(fn([x, y], q, r)); x = nextUp(x); }
      out.push(row);
      y = nextUp(y);
    }
    return out;
  }

  GeopBook.demo("orient-grid", function (ctx) {
    var configs = {
      a: { p: [0.5, 0.5], q: [12, 12], r: [24, 24], label: "p=(0.5,0.5) q=(12,12) r=(24,24)" },
      b: { p: [0.5, 0.5], q: [3, 3], r: [100, 100], label: "p=(0.5,0.5) q=(3,3) r=(100,100)" },
      c: { p: [0.1, 0.1], q: [1.7, 1.7], r: [2.9, 2.9], label: "p=(0.1,0.1) q=(1.7,1.7) r=(2.9,2.9)" }
    };
    var config = "a", n = 64, mode = "float", cache = {};
    var stage = ctx.stage({ aspect: 1, maxHeight: 420, label: "The sign of the orientation test on a grid of nearby points", draw: paint });
    var read = ctx.readout();
    function data() {
      var key = config + ":" + n;
      if (!cache[key]) {
        var c = configs[config];
        cache[key] = { float: grid(c.p, c.q, c.r, n, orientFloat), exact: grid(c.p, c.q, c.r, n, orientExact), interval: grid(c.p, c.q, c.r, n, orientInterval) };
      }
      return cache[key];
    }
    function colorOf(s, c) { return s > 0 ? c.blue : s < 0 ? c.orange : c.faint; }
    function paint(g, w, h, c) {
      var d = data(), size = Math.min(w, h) - 8, cell = size / n;
      var x0 = (w - size) / 2, y0 = (h - size) / 2;
      for (var j = 0; j < n; j++) for (var i = 0; i < n; i++) {
        var s = mode === "wrong" ? 0 : d[mode][j][i];
        var wrong = d.float[j][i] !== d.exact[j][i];
        g.fillStyle = mode === "wrong" ? (wrong ? c.red : c.faint) : colorOf(s, c);
        // y up: row 0 at the bottom.
        g.fillRect(x0 + i * cell, y0 + (n - 1 - j) * cell, Math.ceil(cell), Math.ceil(cell));
      }
    }
    function update() {
      var d = data(), wrong = 0, lied = 0, undecided = 0;
      for (var j = 0; j < n; j++) for (var i = 0; i < n; i++) {
        if (d.float[j][i] !== d.exact[j][i]) wrong++;
        var v = d.interval[j][i];
        if (v === 0) undecided++; else if (v !== d.exact[j][i]) lied++;
      }
      read.set(configs[config].label + "\n" + n + " x " + n + " points p, each one ulp from the next.\n" +
        "f64 gives the wrong sign (or a false zero) at " + wrong + " of " + (n * n) + ".\n" +
        "Intervals: wrong at " + lied + ", \"cannot tell\" at " + undecided + ".");
      stage.redraw();
    }
    ctx.select({
      label: "show", value: mode,
      options: [["float", "sign in f64"], ["exact", "exact sign"], ["wrong", "where f64 is wrong"], ["interval", "interval verdict"]],
      oninput: function (v) { mode = v; update(); }
    });
    ctx.select({ label: "points", value: config, options: [["a", "q=(12,12), r=(24,24)"], ["b", "q=(3,3), r=(100,100)"], ["c", "p=(0.1,0.1), q=(1.7,1.7), r=(2.9,2.9)"]], oninput: function (v) { config = v; update(); } });
    ctx.select({ label: "grid", value: "64", options: [["16", "16"], ["64", "64"], ["128", "128"], ["256", "256"]], oninput: function (v) { n = +v; update(); } });
    update();
  }, { orientExact: orientExact, orientFloat: orientFloat, orientInterval: orientInterval, offset: offset, grid: grid });
})();
