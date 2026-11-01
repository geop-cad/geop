// A small but real sketch solver, in the shape of the kernel's: constraints as
// residuals with exact gradients (forward-mode dual numbers), each step the
// shortest one meeting the linearized constraints (Householder, pivoted) plus
// the part along their null space that serves the pull.
(function () {
  "use strict";

  // ---- dual numbers: a value and a sparse gradient over the variables
  function cst(v) { return { v: v, d: {} }; }
  function vary(x, i) { var d = {}; d[i] = 1; return { v: x[i], d: d }; }
  function comb(a, ka, b, kb, v) {
    var d = {}, k;
    for (k in a.d) d[k] = ka * a.d[k];
    for (k in b.d) d[k] = (d[k] || 0) + kb * b.d[k];
    return { v: v, d: d };
  }
  function add(a, b) { return comb(a, 1, b, 1, a.v + b.v); }
  function sub(a, b) { return comb(a, 1, b, -1, a.v - b.v); }
  function mul(a, b) { return comb(a, b.v, b, a.v, a.v * b.v); }
  function div(a, b) { return comb(a, 1 / b.v, b, -a.v / (b.v * b.v), a.v / b.v); }
  function sqrt(a) { var s = Math.sqrt(a.v); return comb(a, 0.5 / s, a, 0, s); }
  function scale(a, k) { return comb(a, k, a, 0, a.v * k); }

  // ---- the sketch: points (x, y) in x[2i], x[2i+1]; a circle's radius a variable too
  function point(x, i) { return [vary(x, 2 * i), vary(x, 2 * i + 1)]; }
  function norm2(u) { return sqrt(add(mul(u[0], u[0]), mul(u[1], u[1]))); }
  function diff(a, b) { return [sub(a[0], b[0]), sub(a[1], b[1])]; }

  // Each constraint: {kind, ...}, made into residuals, each a length (as the
  // kernel's are: an angle's is multiplied by the sketch's size).
  function residuals(c, x, size, sign) {
    var P = function (i) { return point(x, i); };
    switch (c.kind) {
      case "fix": var p = P(c.p); return [sub(p[0], cst(c.x)), sub(p[1], cst(c.y))];
      case "horizontal": return [sub(P(c.b)[1], P(c.a)[1])];
      case "vertical": return [sub(P(c.b)[0], P(c.a)[0])];
      case "distance": return [sub(norm2(diff(P(c.b), P(c.a))), cst(c.value))];
      case "perpendicular":
      case "parallel": {
        var u = diff(P(c.a1), P(c.a0)), w = diff(P(c.b1), P(c.b0));
        var lu = norm2(u), lw = norm2(w);
        var ux = div(u[0], lu), uy = div(u[1], lu), wx = div(w[0], lw), wy = div(w[1], lw);
        var r = c.kind === "parallel" ? sub(mul(ux, wy), mul(uy, wx)) : add(mul(ux, wx), mul(uy, wy));
        return [scale(r, size)];
      }
      case "tangent": { // the line (a0, a1) touches the circle of centre p, radius variable r
        var s = P(c.a0), e = P(c.a1), q = P(c.p), rad = vary(x, c.r);
        var dir = diff(e, s), l = norm2(dir), off = diff(q, s);
        var dist = div(sub(mul(dir[0], off[1]), mul(dir[1], off[0])), l);
        return [sub(scale(dist, sign[c.id] || 1), rad)];
      }
      case "radius": return [sub(vary(x, c.r), cst(c.value))];
    }
    throw new Error("unknown constraint " + c.kind);
  }

  function evaluate(cs, x, size, sign) {
    var rows = [], values = [], owner = [];
    cs.forEach(function (c, k) {
      residuals(c, x, size, sign).forEach(function (r) {
        var row = new Array(x.length).fill(0);
        for (var i in r.d) row[i] = r.d[i];
        rows.push(row); values.push(r.v); owner.push(k);
      });
    });
    return { J: rows, c: values, owner: owner };
  }

  // ---- linear algebra: Householder with column pivoting, of the columns of A.
  // Returns Q (full, m by m), R, the column order, and the rank: a column with
  // less left than `tol` of the largest is no pivot.
  function qr(A, m, n, tol) {
    var R = A.map(function (r) { return r.slice(); });
    var Q = []; for (var i = 0; i < m; i++) { Q.push(new Array(m).fill(0)); Q[i][i] = 1; }
    var perm = []; for (var j = 0; j < n; j++) perm.push(j);
    var rank = 0, first = 0;
    for (var k = 0; k < Math.min(m, n); k++) {
      var best = -1, bestN = -1;
      for (j = k; j < n; j++) {
        var s = 0; for (i = k; i < m; i++) s += R[i][j] * R[i][j];
        if (s > bestN) { bestN = s; best = j; }
      }
      var len = Math.sqrt(Math.max(bestN, 0));
      if (k === 0) first = len;
      if (len <= tol * Math.max(first, 1e-300) || len === 0) break;
      if (best !== k) { for (i = 0; i < m; i++) { var t = R[i][k]; R[i][k] = R[i][best]; R[i][best] = t; } t = perm[k]; perm[k] = perm[best]; perm[best] = t; }
      var alpha = R[k][k] > 0 ? -len : len; // away from cancelling
      var v = new Array(m).fill(0);
      for (i = k; i < m; i++) v[i] = R[i][k];
      v[k] -= alpha;
      var vv = 0; for (i = k; i < m; i++) vv += v[i] * v[i];
      if (vv > 0) {
        for (j = k; j < n; j++) {
          var f = 0; for (i = k; i < m; i++) f += v[i] * R[i][j];
          f = 2 * f / vv; for (i = k; i < m; i++) R[i][j] -= f * v[i];
        }
        for (var q = 0; q < m; q++) { // Q := Q H
          f = 0; for (i = k; i < m; i++) f += Q[q][i] * v[i];
          f = 2 * f / vv; for (i = k; i < m; i++) Q[q][i] -= f * v[i];
        }
      }
      rank++;
    }
    return { Q: Q, R: R, perm: perm, rank: rank };
  }

  // The y with |A y - b| least, A of full column rank, by Householder: no AᵀA.
  function lstsq(A, b, n) {
    var m = A.length;
    if (!n || !m) return new Array(n || 0).fill(0);
    var f = qr(A, m, n, 1e-14), y = new Array(n).fill(0), c = new Array(m).fill(0), i, j;
    for (i = 0; i < m; i++) for (j = 0; j < m; j++) c[i] += f.Q[j][i] * b[j];
    for (i = f.rank - 1; i >= 0; i--) {
      var s = c[i]; for (j = i + 1; j < f.rank; j++) s -= f.R[i][j] * y[f.perm[j]];
      y[f.perm[i]] = s / f.R[i][i];
    }
    return y;
  }

  // One step: delta = Q_range y (shortest, least squares over the rows) + Z w
  // (along the null space, serving the pulls and staying put).
  function step(J, c, n, pulls, x, damping, tol) {
    var m = J.length, i, j, k;
    var At = []; // Jᵀ, n by m
    for (i = 0; i < n; i++) { At.push(new Array(m)); for (j = 0; j < m; j++) At[i][j] = J[j][i]; }
    var f = m ? qr(At, n, m, tol) : { Q: identity(n), rank: 0 };
    var r = f.rank;
    // The rows in the basis of their range: A = J Q_r, m by r.
    var A = [];
    for (i = 0; i < m; i++) {
      A.push(new Array(r));
      for (k = 0; k < r; k++) { var s = 0; for (j = 0; j < n; j++) s += J[i][j] * f.Q[j][k]; A[i][k] = s; }
    }
    var y = lstsq(A, c.map(function (v) { return -v; }), r);
    var delta = new Array(n).fill(0);
    for (k = 0; k < r; k++) for (j = 0; j < n; j++) delta[j] += f.Q[j][k] * y[k];
    // Along Z = the rest of Q: least squares of the pulls, damped.
    var free = n - r;
    if (free > 0 && pulls.length) {
      var rows = [], rhs = [];
      pulls.forEach(function (p) {
        var row = new Array(free);
        for (k = 0; k < free; k++) row[k] = f.Q[p.i][r + k];
        rows.push(row); rhs.push(p.to - x[p.i] - delta[p.i]);
      });
      for (j = 0; j < n; j++) {
        var row2 = new Array(free);
        for (k = 0; k < free; k++) row2[k] = damping * f.Q[j][r + k];
        rows.push(row2); rhs.push(-damping * delta[j]);
      }
      var w = lstsq(rows, rhs, free);
      for (k = 0; k < free; k++) for (j = 0; j < n; j++) delta[j] += f.Q[j][r + k] * w[k];
    }
    return { delta: delta, rank: r };
  }
  function identity(n) { var q = []; for (var i = 0; i < n; i++) { q.push(new Array(n).fill(0)); q[i][i] = 1; } return q; }

  function maxAbs(v) { return v.reduce(function (m, a) { return Math.max(m, Math.abs(a)); }, 0); }

  // Solves in place. A tangency keeps the side it starts on, as the kernel's does.
  // pulls: [{i, to}]. Returns {iterations, residual, rank, dof, rows, log}.
  function solve(cs, x, opts) {
    opts = opts || {};
    var pulls = opts.pulls || [];
    var size = opts.size || 1;
    var tol = opts.tol === undefined ? 1e-8 : opts.tol;
    var maxStep = opts.maxStep || 0.5 * size;
    var sign = {};
    var at = evaluate(cs, x, size, {});
    cs.forEach(function (c) {
      if (c.kind === "tangent") {
        var ax = x[2 * c.a0], ay = x[2 * c.a0 + 1], bx = x[2 * c.a1] - ax, by = x[2 * c.a1 + 1] - ay;
        var d = (bx * (x[2 * c.p + 1] - ay) - by * (x[2 * c.p] - ax)) / Math.hypot(bx, by);
        sign[c.id] = d >= 0 ? 1 : -1;
      }
    });
    var log = [], it = 0, rank = 0;
    for (it = 0; it < (opts.maxIterations || 40); it++) {
      at = evaluate(cs, x, size, sign);
      log.push(maxAbs(at.c));
      var s = step(at.J, at.c, x.length, pulls, x, opts.damping === undefined ? 1e-4 : opts.damping, tol);
      var len = maxAbs(s.delta), k = len > maxStep ? maxStep / len : 1;
      for (var i = 0; i < x.length; i++) x[i] += k * s.delta[i];
      rank = s.rank;
      if (len < 1e-13 * size) { it++; break; }
    }
    at = evaluate(cs, x, size, sign);
    var f = at.J.length ? rankOf(at.J, tol) : 0;
    var res = maxAbs(at.c);
    return { iterations: it, residual: res, rank: f, dof: x.length - f, rows: at.J.length, log: log, conflict: res > 1e-7 * size, c: at.c, owner: at.owner };
  }
  function rankOf(J, tol) {
    var At = [], m = J.length, n = J[0].length;
    for (var i = 0; i < n; i++) { At.push(new Array(m)); for (var j = 0; j < m; j++) At[i][j] = J[j][i]; }
    return qr(At, n, m, tol === undefined ? 1e-8 : tol).rank;
  }

  // ---- the scene: a quadrilateral P0..P3 with shared corners, and a circle
  var NVARS = 11; // 4 corners, centre P4, radius x[10]
  function start() { return [1.0, 0.8, 4.6, 1.0, 4.3, 3.0, 0.8, 3.2, 3.0, 2.0, 0.7]; }

  var CATALOGUE = [
    { key: "fix", label: "P0 fixed at (1, 1)", c: { kind: "fix", p: 0, x: 1, y: 1 }, on: true },
    { key: "h0", label: "bottom horizontal", c: { kind: "horizontal", a: 0, b: 1 }, on: true },
    { key: "h2", label: "top horizontal", c: { kind: "horizontal", a: 3, b: 2 } },
    { key: "v1", label: "right vertical", c: { kind: "vertical", a: 1, b: 2 }, on: true },
    { key: "v3", label: "left vertical", c: { kind: "vertical", a: 0, b: 3 } },
    { key: "w", label: "width = 3.5", c: { kind: "distance", a: 0, b: 1, value: 3.5 } },
    { key: "h", label: "height = 2", c: { kind: "distance", a: 1, b: 2, value: 2 } },
    { key: "perp", label: "bottom ⊥ right (implied)", c: { kind: "perpendicular", a0: 0, a1: 1, b0: 1, b1: 2 } },
    { key: "par", label: "bottom ∥ top (implied)", c: { kind: "parallel", a0: 0, a1: 1, b0: 3, b1: 2 } },
    { key: "diag", label: "diagonal = 2 (conflicts)", c: { kind: "distance", a: 0, b: 2, value: 2 } },
    { key: "r", label: "radius = 0.7", c: { kind: "radius", r: 10, value: 0.7 } },
    { key: "t", label: "circle tangent to bottom", c: { id: "t", kind: "tangent", a0: 0, a1: 1, p: 4, r: 10 } }
  ];

  GeopBook.demo("sketch-solver", function (ctx) {
    var D = GeopBook.draw;
    var x = start();
    var active = {};
    CATALOGUE.forEach(function (e) { active[e.key] = !!e.on; });
    var pts = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]];
    var result = null, dragTarget = null;

    function constraints() { return CATALOGUE.filter(function (e) { return active[e.key]; }).map(function (e) { return e.c; }); }
    function syncPts() { for (var i = 0; i < 5; i++) { pts[i][0] = x[2 * i]; pts[i][1] = x[2 * i + 1]; } }

    var stage = ctx.stage({ aspect: 1.5, maxHeight: 380, label: "A quadrilateral and a circle, solved under the constraints ticked below", draw: paint });
    var view = ctx.view(stage, { x0: 0, x1: 6, y0: 0, y1: 4 });
    var read = ctx.readout();

    function resolve(pulls) {
      result = solve(constraints(), x, { pulls: pulls || [], size: 5 });
      syncPts();
      var kind;
      var m = result.rows, r = result.rank;
      if (result.conflict) kind = "CONFLICT: the rows cannot all vanish; this is the nearest the solve came";
      else if (m > r) kind = (m - r) + " redundant row" + (m - r > 1 ? "s" : "") + ": they say what the others say";
      else if (result.dof === 0) kind = "fully constrained";
      else kind = "under-constrained: " + result.dof + " way" + (result.dof > 1 ? "s" : "") + " left to move";
      read.set("variables " + NVARS + "   rows " + m + "   rank " + r + "   freedom " + result.dof + "\n" +
        "largest residual " + result.residual.toExponential(1) + "   steps " + result.iterations + "\n" +
        "per step: " + result.log.slice(0, 8).map(function (v) { return v.toExponential(0); }).join(" ") + "\n" + kind);
      stage.redraw();
    }

    function paint(g, w, h, c) {
      if (!view) return;
      view.update();
      D.grid(g, view, 1, c.grid);
      var col = result && result.conflict ? c.red : c.blue;
      D.polygon(g, view, pts.slice(0, 4), c.fillA, col, 2.2);
      var cx = view.x(pts[4][0]), cy = view.y(pts[4][1]);
      g.beginPath(); g.arc(cx, cy, Math.abs(x[10]) * view.sx, 0, 2 * Math.PI); g.strokeStyle = c.orange; g.lineWidth = 2; g.stroke();
      if (dragTarget) D.dot(g, view, dragTarget, 3, c.muted);
      pts.forEach(function (p, i) {
        D.dot(g, view, p, 6, handles && handles.selected === i ? c.accent : c.bg, c.fg);
        D.text(g, view, [p[0] + 0.1, p[1] + 0.1], i === 4 ? "C" : "P" + i, c.muted);
      });
    }

    var handles = ctx.handles({
      stage: stage, view: view, pts: pts,
      constrain: function (i, p) {
        dragTarget = p;
        resolve([{ i: 2 * i, to: p[0] }, { i: 2 * i + 1, to: p[1] }]);
        return null;
      },
      onrelease: function () { dragTarget = null; stage.redraw(); }
    });

    CATALOGUE.forEach(function (e) {
      ctx.checkbox({ label: e.label, value: !!e.on, oninput: function (v) { active[e.key] = v; resolve(); } });
    });
    ctx.button("Reset", function () {
      x = start();
      resolve();
    });
    resolve();
  }, { solve: solve, evaluate: evaluate, rankOf: rankOf, qr: qr, lstsq: lstsq, step: step, residuals: residuals, start: start, CATALOGUE: CATALOGUE });
})();
