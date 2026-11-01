// A planar four-bar linkage solved the way the kernel solves an assembly, in
// miniature: every moving link a body (x, y, angle), every pin two residuals, a
// drag a pull among where the pins hold. The freedom is rank arithmetic.
(function () {
  "use strict";

  // Bodies: {x,y,a} variables at 3i..3i+2. A pin ties local point pa of body a
  // (or a ground point if a < 0) to local point pb of body b.
  function placed(x, b, p) {
    if (b < 0) return { v: [p[0], p[1]], d: {} };
    var i = 3 * b, c = Math.cos(x[i + 2]), s = Math.sin(x[i + 2]);
    var wx = c * p[0] - s * p[1], wy = s * p[0] + c * p[1];
    var d = {}; d[i] = [1, 0]; d[i + 1] = [0, 1]; d[i + 2] = [-wy, wx];
    return { v: [x[i] + wx, x[i + 1] + wy], d: d };
  }

  function evaluate(pins, x) {
    var J = [], c = [], n = x.length;
    pins.forEach(function (pin) {
      var A = placed(x, pin.a, pin.pa), B = placed(x, pin.b, pin.pb);
      [0, 1].forEach(function (k) {
        var row = new Array(n).fill(0), i;
        for (i in A.d) row[i] += A.d[i][k];
        for (i in B.d) row[i] -= B.d[i][k];
        J.push(row); c.push(A.v[k] - B.v[k]);
      });
    });
    return { J: J, c: c };
  }

  // Householder with column pivoting on the columns of A (m by n): the factor Q,
  // the rank, the pivots.
  function qr(A, m, n, tol) {
    var R = A.map(function (r) { return r.slice(); }), i, j;
    var Q = []; for (i = 0; i < m; i++) { Q.push(new Array(m).fill(0)); Q[i][i] = 1; }
    var perm = []; for (j = 0; j < n; j++) perm.push(j);
    var rank = 0, pivots = [], first = 0;
    for (var k = 0; k < Math.min(m, n); k++) {
      var best = k, bn = -1;
      for (j = k; j < n; j++) { var s = 0; for (i = k; i < m; i++) s += R[i][j] * R[i][j]; if (s > bn) { bn = s; best = j; } }
      var len = Math.sqrt(Math.max(bn, 0));
      if (k === 0) first = len;
      pivots.push(len);
      if (len <= tol * Math.max(first, 1e-300) || len === 0) break;
      if (best !== k) { for (i = 0; i < m; i++) { var t = R[i][k]; R[i][k] = R[i][best]; R[i][best] = t; } t = perm[k]; perm[k] = perm[best]; perm[best] = t; }
      var alpha = R[k][k] > 0 ? -len : len, v = new Array(m).fill(0), vv = 0;
      for (i = k; i < m; i++) v[i] = R[i][k];
      v[k] -= alpha;
      for (i = k; i < m; i++) vv += v[i] * v[i];
      if (vv > 0) {
        for (j = k; j < n; j++) { var f = 0; for (i = k; i < m; i++) f += v[i] * R[i][j]; f = 2 * f / vv; for (i = k; i < m; i++) R[i][j] -= f * v[i]; }
        for (var q = 0; q < m; q++) { f = 0; for (i = k; i < m; i++) f += Q[q][i] * v[i]; f = 2 * f / vv; for (i = k; i < m; i++) Q[q][i] -= f * v[i]; }
      }
      rank++;
    }
    return { Q: Q, R: R, perm: perm, rank: rank, pivots: pivots };
  }
  function lstsq(A, b, n) {
    var m = A.length;
    if (!n || !m) return new Array(n || 0).fill(0);
    var f = qr(A, m, n, 1e-14), y = new Array(n).fill(0), c = new Array(m).fill(0), i, j;
    for (i = 0; i < m; i++) for (j = 0; j < m; j++) c[i] += f.Q[j][i] * b[j];
    for (i = f.rank - 1; i >= 0; i--) { var s = c[i]; for (j = i + 1; j < f.rank; j++) s -= f.R[i][j] * y[f.perm[j]]; y[f.perm[i]] = s / f.R[i][i]; }
    return y;
  }
  function transpose(J, n) { var At = []; for (var i = 0; i < n; i++) { At.push(new Array(J.length)); for (var j = 0; j < J.length; j++) At[i][j] = J[j][i]; } return At; }

  function rankInfo(J, n, tol) {
    if (!J.length) return { rank: 0, smallest: Infinity };
    var f = qr(transpose(J, n), n, J.length, tol === undefined ? 1e-7 : tol);
    return { rank: f.rank, smallest: f.pivots[f.rank - 1] };
  }

  // A step: shortest meeting the rows, then along the null space towards the pull.
  // The pull is on one point of one body: {b, p, to}.
  function solve(pins, x, pull, opts) {
    opts = opts || {};
    var n = x.length, its = opts.iterations || 30, damping = 1e-3, tol = 1e-7;
    for (var it = 0; it < its; it++) {
      var e = evaluate(pins, x), m = e.J.length, i, j, k;
      var f = qr(transpose(e.J, n), n, m, tol), r = f.rank;
      var A = [];
      for (i = 0; i < m; i++) { A.push(new Array(r)); for (k = 0; k < r; k++) { var s = 0; for (j = 0; j < n; j++) s += e.J[i][j] * f.Q[j][k]; A[i][k] = s; } }
      var y = lstsq(A, e.c.map(function (v) { return -v; }), r);
      var delta = new Array(n).fill(0);
      for (k = 0; k < r; k++) for (j = 0; j < n; j++) delta[j] += f.Q[j][k] * y[k];
      var free = n - r;
      if (pull && free > 0) {
        var P = placed(x, pull.b, pull.p), rows = [], rhs = [];
        [0, 1].forEach(function (a) {
          var row = new Array(free).fill(0), res = pull.to[a] - P.v[a];
          for (var ii in P.d) { for (var c = 0; c < free; c++) row[c] += P.d[ii][a] * f.Q[ii][r + c]; res -= P.d[ii][a] * delta[ii]; }
          rows.push(row); rhs.push(res);
        });
        for (j = 0; j < n; j++) { var row2 = new Array(free); for (k = 0; k < free; k++) row2[k] = damping * f.Q[j][r + k]; rows.push(row2); rhs.push(-damping * delta[j]); }
        var w = lstsq(rows, rhs, free);
        for (k = 0; k < free; k++) for (j = 0; j < n; j++) delta[j] += f.Q[j][r + k] * w[k];
      }
      var len = 0; for (j = 0; j < n; j++) len = Math.max(len, Math.abs(delta[j]));
      var lim = len > 0.15 ? 0.15 / len : 1;
      for (j = 0; j < n; j++) x[j] += lim * delta[j];
      if (len < 1e-12) break;
    }
    var end = evaluate(pins, x);
    var ri = rankInfo(end.J, n, tol);
    var res = end.c.reduce(function (M, v) { return Math.max(M, Math.abs(v)); }, 0);
    // Where one way to move is left: the null space's direction. The rocker's
    // turn per turn of the crank, if the crank turns at all.
    var motion = null;
    if (n - ri.rank === 1) {
      var fn = qr(transpose(end.J, n), n, end.J.length, tol);
      motion = []; for (j = 0; j < n; j++) motion.push(fn.Q[j][fn.rank]);
    }
    return { rank: ri.rank, smallest: ri.smallest, rows: end.J.length, vars: n, freedom: n - ri.rank, residual: res, motion: motion };
  }

  // ---- mechanisms. Link i spans its pivot (local origin) to (L, 0).
  function bar(x, i, ax, ay, bx, by) { x[3 * i] = ax; x[3 * i + 1] = ay; x[3 * i + 2] = Math.atan2(by - ay, bx - ax); }

  // Four-bar: ground pivots A, D; crank A-B (body 0), coupler B-C (body 1), rocker D-C (body 2).
  function mechanism(L1, L2, L3, L4, crank, extra) {
    var A = [0, 0], D_ = [L4, 0];
    var B = [A[0] + L1 * Math.cos(crank), A[1] + L1 * Math.sin(crank)];
    var dx = D_[0] - B[0], dy = D_[1] - B[1], d = Math.hypot(dx, dy);
    var a = (L2 * L2 - L3 * L3 + d * d) / (2 * d), h = Math.sqrt(Math.max(L2 * L2 - a * a, 0));
    var C = [B[0] + a * dx / d + h * dy / d, B[1] + a * dy / d - h * dx / d];
    if (C[1] < 0) C = [B[0] + a * dx / d - h * dy / d, B[1] + a * dy / d + h * dx / d];
    var x = new Array(extra ? 12 : 9);
    bar(x, 0, A[0], A[1], B[0], B[1]); bar(x, 1, B[0], B[1], C[0], C[1]); bar(x, 2, D_[0], D_[1], C[0], C[1]);
    var pins = [
      { a: -1, pa: A, b: 0, pb: [0, 0] },
      { a: 0, pa: [L1, 0], b: 1, pb: [0, 0] },
      { a: 1, pa: [L2, 0], b: 2, pb: [L3, 0] },
      { a: -1, pa: D_, b: 2, pb: [0, 0] }
    ];
    var result = { x: x, pins: pins, links: [[0, L1], [1, L2], [2, L3]], ground: [A, D_] };
    if (extra) { // a third crank, parallel to the other two, midway along the coupler
      var E = [L4 / 2, 0], F = [(B[0] + C[0]) / 2, (B[1] + C[1]) / 2];
      bar(x, 3, E[0], E[1], F[0], F[1]);
      pins.push({ a: -1, pa: E, b: 3, pb: [0, 0] });
      pins.push({ a: 1, pa: [L2 / 2, 0], b: 3, pb: [Math.hypot(F[0] - E[0], F[1] - E[1]), 0] });
      result.links.push([3, Math.hypot(F[0] - E[0], F[1] - E[1])]);
      result.ground.push(E);
    }
    return result;
  }
  // The crank angle where crank and coupler lie in one line, stretched: |AC| = L1 + L2.
  // The rocker is at its limit there: a dead point for whoever drives by the rocker.
  function deadCrank(L1, L2, L3, L4) {
    var r = L1 + L2, cos = (r * r + L4 * L4 - L3 * L3) / (2 * r * L4);
    return Math.acos(cos);
  }

  var PRESETS = {
    crank: { L: [1.0, 2.6, 2.3, 3.0], crank: 1.1, extra: false, label: "crank and rocker" },
    para: { L: [1.4, 3.0, 1.4, 3.0], crank: 1.1, extra: false, label: "parallelogram" },
    extra: { L: [1.4, 3.0, 1.4, 3.0], crank: 1.1, extra: true, label: "with a third crank" }
  };

  GeopBook.demo("four-bar", function (ctx) {
    var D = GeopBook.draw;
    var mech, result, preset = "crank", target = null;
    var pts = [[0, 0], [0, 0], [0, 0]];
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 380, label: "A four-bar linkage: three moving links on two ground pivots", draw: paint });
    var view = ctx.view(stage, { x0: -2, x1: 5, y0: -2.2, y1: 3.2 });
    var read = ctx.readout();
    var handleBody = [[0, 1], [1, 0.5], [2, 1]]; // [body, fraction along it]

    function syncPts() {
      handleBody.forEach(function (h, i) {
        var L = mech.links[h[0]][1], P = placed(mech.x, h[0], [L * h[1], 0]);
        pts[i][0] = P.v[0]; pts[i][1] = P.v[1];
      });
    }
    function report() {
      var r = result;
      var grubler = 3 * (r.vars / 3) - r.rows;
      var deficit = r.rows - r.rank, redundant = PRESETS[preset].extra ? 1 : 0;
      var kind = r.residual > 1e-6 ? "The pins cannot all hold here." :
        deficit > redundant ? "Singular: at this one configuration the rows lose rank. To first order there is a second way to move, the linkage can leave along either of two branches, and no enclosure can be proven." :
        redundant ? "Counting says " + grubler + " free; the rank says " + r.freedom + ". A row is redundant: everywhere, not at one point. The linkage still moves." :
        "Regular: one way to move.";
      var rate = "";
      if (r.motion && Math.abs(r.motion[2]) > 1e-9) rate = "   rocker turn per crank turn " + (r.motion[8] / r.motion[2]).toFixed(3);
      read.set("bodies " + r.vars / 3 + "   variables " + r.vars + "   pin rows " + r.rows + "   rank " + r.rank + "   freedom " + r.freedom + "\n" +
        "counting (3 per body, 2 per pin): " + grubler + "\n" +
        "smallest pivot " + r.smallest.toExponential(2) + "   largest residual " + r.residual.toExponential(1) + rate + "\n" + kind);
    }
    function load(p, crankAt) {
      preset = p;
      var s = PRESETS[p];
      mech = mechanism(s.L[0], s.L[1], s.L[2], s.L[3], crankAt === undefined ? s.crank : crankAt, s.extra);
      result = solve(mech.pins, mech.x, null, { iterations: 20 });
      syncPts(); report(); stage.redraw();
    }
    function paint(g, w, h, c) {
      if (!view || !mech) return;
      view.update();
      D.grid(g, view, 1, c.grid);
      mech.ground.forEach(function (p) { D.polygon(g, view, [[p[0] - 0.18, p[1] - 0.25], [p[0] + 0.18, p[1] - 0.25], p], c.faint, c.muted, 1); });
      mech.links.forEach(function (l, i) {
        var A = placed(mech.x, l[0], [0, 0]).v, B = placed(mech.x, l[0], [l[1], 0]).v;
        D.polyline(g, view, [A, B], [c.blue, c.green, c.orange, c.purple][i], 6);
        D.dot(g, view, A, 4, c.fg); D.dot(g, view, B, 4, c.fg);
      });
      if (target) D.dot(g, view, target, 3, c.muted);
      pts.forEach(function (p, i) { D.dot(g, view, p, 7, handles && handles.selected === i ? c.accent : c.bg, c.fg); });
    }
    var handles = ctx.handles({
      stage: stage, view: view, pts: pts, radius: 26,
      constrain: function (i, p) {
        var h = handleBody[i], L = mech.links[h[0]][1];
        target = p;
        result = solve(mech.pins, mech.x, { b: h[0], p: [L * h[1], 0], to: p });
        syncPts(); report();
        return null;
      },
      onrelease: function () { target = null; stage.redraw(); }
    });
    ctx.select({ label: "mechanism", value: "crank", options: Object.keys(PRESETS).map(function (k) { return [k, PRESETS[k].label]; }), oninput: function (v) { load(v); } });
    var selEl = ctx.root.querySelector("select");
    ctx.button("Rocker's limit", function () {
      var s = PRESETS.crank;
      load("crank", deadCrank(s.L[0], s.L[1], s.L[2], s.L[3]));
      selEl.value = "crank";
    });
    ctx.button("Flat parallelogram", function () { load("para", 0); selEl.value = "para"; });
    load("crank");
  }, { mechanism: mechanism, solve: solve, evaluate: evaluate, rankInfo: rankInfo, deadCrank: deadCrank, placed: placed, PRESETS: PRESETS });
})();
