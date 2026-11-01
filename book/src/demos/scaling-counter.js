// Counts the work of solving an assembly, three ways, on two benchmark shapes.
// The graph of mates is generated and analysed for real (union-find for the
// groups, peeling for the pendants); the cost of each dense system is the
// multiplications of Gaussian elimination, checked against a counted elimination
// in the demo's test. A model: the kernel's real solver does more per system.
(function () {
  "use strict";

  var VARS = 6; // a rigid body's freedoms

  // Multiplications and divisions of Gaussian elimination and one back
  // substitution on a dense m x m system, as countedElimination does them:
  // m(m+1)(2m+1)/6 + m(m-1)/2, with leading term m^3/3.
  function denseOps(m) {
    var ops = 0;
    for (var k = 0; k < m; k++) ops += (m - k - 1) * (m - k) + (m - k);
    return ops + m * (m + 1) / 2 - m;
  }

  // Elimination actually run, counting its operations: what denseOps claims.
  function countedElimination(m) {
    var A = [], i, j, k, ops = 0;
    for (i = 0; i < m; i++) { A.push([]); for (j = 0; j <= m; j++) A[i].push((i === j ? m + 1 : 0) + 1 / (1 + i + j)); }
    for (k = 0; k < m; k++) {
      for (i = k + 1; i < m; i++) {
        var f = A[i][k] / A[k][k]; ops += 1;
        for (j = k + 1; j <= m; j++) { A[i][j] -= f * A[k][j]; ops += 1; }
      }
    }
    var x = new Array(m);
    for (i = m - 1; i >= 0; i--) {
      var s = A[i][m];
      for (j = i + 1; j < m; j++) { s -= A[i][j] * x[j]; ops += 1; }
      x[i] = s / A[i][i]; ops += 1;
    }
    return { ops: ops, x: x, A: A };
  }

  // An assembly of `parts` screws. shape "plate": every screw on one fixed plate,
  // which has no variables. shape "robot": a chain of `links` links (each mated to
  // the one before), a bracket on each, and the screws on the brackets.
  function assembly(parts, shape, links) {
    var mates = [], bodies = [], i;
    if (shape === "plate") {
      for (i = 0; i < parts; i++) { bodies.push("s" + i); mates.push(["s" + i, null]); }
      return { bodies: bodies, mates: mates };
    }
    for (i = 0; i < links; i++) {
      bodies.push("l" + i);
      mates.push(i ? ["l" + i, "l" + (i - 1)] : ["l0", null]);
      bodies.push("b" + i);
      mates.push(["b" + i, "l" + i]);
    }
    for (i = 0; i < parts; i++) { bodies.push("s" + i); mates.push(["s" + i, "b" + (i % links)]); }
    return { bodies: bodies, mates: mates };
  }

  // Connected components by union-find; a mate to nothing (null) ties nothing.
  function groups(a) {
    var parent = {};
    function find(x) { while (parent[x] !== x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
    a.bodies.forEach(function (b) { parent[b] = b; });
    a.mates.forEach(function (m) { if (m[1] !== null) parent[find(m[0])] = find(m[1]); });
    var sizes = {};
    a.bodies.forEach(function (b) { var r = find(b); sizes[r] = (sizes[r] || 0) + 1; });
    return Object.keys(sizes).map(function (k) { return sizes[k]; });
  }

  // Pendants: repeatedly take off a body that only hangs off one other and that
  // nothing else hangs off, its own small solve afterwards. What is left is the core.
  function pendants(a, keep) {
    var hanging = {}, of = {};
    a.bodies.forEach(function (b) { hanging[b] = 0; });
    a.mates.forEach(function (m) { if (m[1] !== null) { hanging[m[1]] += 1; of[m[0]] = m[1]; } });
    var left = {}, count = 0;
    a.bodies.forEach(function (b) { left[b] = true; });
    var stack = a.bodies.filter(function (b) { return hanging[b] === 0 && !keep(b); });
    while (stack.length) {
      var b = stack.pop();
      delete left[b]; count += 1;
      var up = of[b];
      if (up !== undefined && --hanging[up] === 0 && !keep(up)) stack.push(up);
    }
    return { pendants: count, core: Object.keys(left).length };
  }

  // The operation counts of one solve under each strategy.
  function costs(parts, shape, links) {
    var a = assembly(parts, shape, links);
    var whole = denseOps(a.bodies.length * VARS);
    var grouped = groups(a).reduce(function (s, g) { return s + denseOps(g * VARS); }, 0);
    var keep = function (b) { return b[0] === "l"; };
    var p = pendants(a, keep);
    var hung = denseOps(p.core * VARS) + p.pendants * denseOps(VARS);
    return { bodies: a.bodies.length, whole: whole, grouped: grouped, pendant: hung, perStep: a.bodies.length * whole };
  }

  GeopBook.demo("scaling-counter", function (ctx) {
    var D = GeopBook.draw;
    var shape = "robot", parts = 200, links = 6;
    var series = [
      { key: "perStep", label: "one dense question per step", color: "red" },
      { key: "whole", label: "one dense system of every part", color: "orange" },
      { key: "grouped", label: "groups no mate ties together", color: "blue" },
      { key: "pendant", label: "groups and pendants", color: "green" }
    ];
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 400, label: "Operation counts against the number of parts, on logarithmic axes", draw: paint });
    var read = ctx.readout();
    function paint(g, w, h, c) {
      var ml = 44, mb = 28, mt = 8, mr = 8;
      var xs = [];
      for (var n = 10; n <= 2000; n = Math.ceil(n * 1.25)) xs.push(n);
      var data = xs.map(function (n) { return costs(n, shape, links); });
      var lo = 1e-3, hi = 1e15;
      var X = function (n) { return ml + (Math.log10(n) - 1) / (Math.log10(2000) - 1) * (w - ml - mr); };
      var Y = function (v) { return mt + (Math.log10(hi) - Math.log10(Math.max(v, lo))) / (Math.log10(hi) - Math.log10(lo)) * (h - mt - mb); };
      g.lineWidth = 1; g.strokeStyle = c.grid;
      [10, 100, 1000].forEach(function (n) {
        g.beginPath(); g.moveTo(X(n), mt); g.lineTo(X(n), h - mb); g.stroke();
        D.text(g, null, [X(n), h - mb + 15], String(n), c.muted, "center");
      });
      for (var e = 0; e <= 15; e += 3) {
        g.beginPath(); g.moveTo(ml, Y(Math.pow(10, e))); g.lineTo(w - mr, Y(Math.pow(10, e))); g.stroke();
        D.text(g, null, [ml - 5, Y(Math.pow(10, e)) + 4], "1e" + e, c.muted, "right");
      }
      D.text(g, null, [(ml + w - mr) / 2, h - 3], "parts", c.muted, "center");
      series.forEach(function (s) {
        var pts = xs.map(function (n, i) { return [X(n), Y(data[i][s.key])]; });
        D.polyline(g, null, pts, c[s.color], 2);
      });
      var at = costs(parts, shape, links);
      g.strokeStyle = c.muted; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(X(parts), mt); g.lineTo(X(parts), h - mb); g.stroke(); g.setLineDash([]);
      series.forEach(function (s) { D.dot(g, null, [X(parts), Y(at[s.key])], 4, c[s.color]); });
      var y = mt + 12;
      series.forEach(function (s) { g.fillStyle = c[s.color]; g.fillRect(ml + 8, y - 7, 10, 3); D.text(g, null, [ml + 22, y - 2], s.label, c.fg); y += 14; });
    }
    function update() {
      var a = costs(parts, shape, links), b = costs(parts * 2, shape, links);
      var f = function (v) { return v.toExponential(2); };
      read.set(a.bodies + " bodies: " +
        "\none question per step  " + f(a.perStep) + "  (x" + (b.perStep / a.perStep).toFixed(1) + " for twice the parts)" +
        "\none dense system       " + f(a.whole) + "  (x" + (b.whole / a.whole).toFixed(1) + ")" +
        "\ngroups                 " + f(a.grouped) + "  (x" + (b.grouped / a.grouped).toFixed(1) + ")" +
        "\ngroups and pendants    " + f(a.pendant) + "  (x" + (b.pendant / a.pendant).toFixed(1) + ")");
      stage.redraw();
    }
    ctx.select({ label: "benchmark", value: shape, options: [["plate", "screws on a fixed plate"], ["robot", "screws on a chain of links"]], oninput: function (v) { shape = v; update(); } });
    ctx.slider({ label: "parts", min: 10, max: 2000, step: 10, value: parts, format: function (v) { return String(v); }, oninput: function (v) { parts = v; update(); } });
    update();
  }, { denseOps: denseOps, countedElimination: countedElimination, assembly: assembly, groups: groups, pendants: pendants, costs: costs });
})();
