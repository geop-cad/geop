// A predictor-corrector march along where a sphere and a cylinder meet, by the
// same four-equation corrector `predictor_corrector_step` solves: S_a(u, v) = S_b(s, z)
// and the point on the plane through the prediction. In plain floating point, with a
// fixed step and a convergence test on the residual where the kernel has interval
// comparisons, and a sphere and cylinder given as formulas where it has NURBS.
// Pure functions in `algo`.
(function () {
  "use strict";

  function sphere(R) {
    return {
      p: function (u, v) { return [R * Math.cos(v) * Math.cos(u), R * Math.cos(v) * Math.sin(u), R * Math.sin(v)]; },
      du: function (u, v) { return [-R * Math.cos(v) * Math.sin(u), R * Math.cos(v) * Math.cos(u), 0]; },
      dv: function (u, v) { return [-R * Math.sin(v) * Math.cos(u), -R * Math.sin(v) * Math.sin(u), R * Math.cos(v)]; }
    };
  }
  function cylinder(a, r) {
    return {
      p: function (s, z) { return [a + r * Math.cos(s), r * Math.sin(s), z]; },
      du: function (s) { return [-r * Math.sin(s), r * Math.cos(s), 0]; },
      dv: function () { return [0, 0, 1]; }
    };
  }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function add(a, b, k) { return [a[0] + b[0] * k, a[1] + b[1] * k, a[2] + b[2] * k]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  function norm(a) { return Math.sqrt(dot(a, a)); }

  // Gaussian elimination with pivoting on a small dense system.
  function solve(A, b) {
    var n = b.length, M = A.map(function (r, i) { return r.concat([b[i]]); });
    for (var c = 0; c < n; c++) {
      var piv = c;
      for (var r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
      if (M[piv][c] === 0) return null;
      var t = M[c]; M[c] = M[piv]; M[piv] = t;
      for (r = c + 1; r < n; r++) {
        var f = M[r][c] / M[c][c];
        for (var k = c; k <= n; k++) M[r][k] -= f * M[c][k];
      }
    }
    var x = new Array(n);
    for (var i = n - 1; i >= 0; i--) {
      var s = M[i][n];
      for (var j = i + 1; j < n; j++) s -= M[i][j] * x[j];
      x[i] = s / M[i][i];
    }
    return x;
  }

  // The curve's tangent is square to both normals. Null where they are parallel.
  function tangent(A, B, q) {
    var na = cross(A.du(q[0], q[1]), A.dv(q[0], q[1])), nb = cross(B.du(q[2], q[3]), B.dv(q[2], q[3]));
    var t = cross(na, nb), l = norm(t);
    if (l < 1e-9 * norm(na) * norm(nb)) return null;
    return [t[0] / l, t[1] / l, t[2] / l];
  }

  var LAMBDA = 1e-12;
  var ITERATIONS = 20;
  // One predictor-corrector step: target = point + dir * step; solve
  // S_a - S_b = 0, dir . (S_a - target) = 0 by damped normal equations.
  function corrector(A, B, q, target, dir) {
    var x = q.slice(), history = [];
    for (var it = 0; it < ITERATIONS; it++) {
      var pa = A.p(x[0], x[1]), pb = B.p(x[2], x[3]), delta = sub(pa, pb), plane = dot(dir, sub(pa, target));
      var f = [delta[0], delta[1], delta[2], plane];
      var res = Math.max(Math.abs(f[0]), Math.abs(f[1]), Math.abs(f[2]), Math.abs(f[3]));
      history.push(res);
      if (res < 1e-12) return { q: x, point: pa, iterations: it, history: history, converged: true };
      var au = A.du(x[0], x[1]), av = A.dv(x[0], x[1]), bu = B.du(x[2], x[3]), bv = B.dv(x[2], x[3]);
      var J = [0, 1, 2].map(function (k) { return [au[k], av[k], -bu[k], -bv[k]]; });
      J.push([dot(dir, au), dot(dir, av), 0, 0]);
      var ata = [], atf = [];
      for (var r = 0; r < 4; r++) {
        ata.push([]);
        for (var c = 0; c < 4; c++) {
          var sum = r === c ? LAMBDA : 0;
          for (var row = 0; row < 4; row++) sum += J[row][r] * J[row][c];
          ata[r].push(sum);
        }
        var s = 0;
        for (row = 0; row < 4; row++) s += J[row][r] * f[row];
        atf.push(-s);
      }
      var d = solve(ata, atf);
      if (!d) return { q: x, point: pa, iterations: it, history: history, converged: false };
      x = [x[0] + d[0], x[1] + d[1], x[2] + d[2], x[3] + d[3]];
    }
    return { q: x, point: A.p(x[0], x[1]), iterations: ITERATIONS, history: history, converged: false };
  }

  // Where the two meet on the side facing +x, in the first quadrant of z: the seed
  // of the march. On the cylinder's angle s = s0, z from the sphere's equation.
  function start(a, r, R, s0) {
    var x = a + r * Math.cos(s0), y = r * Math.sin(s0), h = R * R - x * x - y * y;
    if (h < 0) return null;
    var z = Math.sqrt(h);
    return [Math.atan2(y, x), Math.asin(z / R), s0, z];
  }

  // March from the start until the curve closes on it, or until something goes wrong.
  function march(a, r, R, step, maxSteps) {
    var A = sphere(R), B = cylinder(a, r), q = start(a, r, R, 0.6);
    if (!q) return { steps: [], status: "the surfaces do not meet at the start" };
    var p0 = A.p(q[0], q[1]), point = p0, dir = tangent(A, B, q), steps = [], status = "ran out of steps";
    if (!dir) return { steps: [], status: "normals parallel at the start" };
    var dir0 = dir, travelled = 0;
    for (var i = 0; i < maxSteps; i++) {
      // Closing: the start lies ahead within this stride. (The kernel's trace
      // ends at a known vertex the same way, `candidate_within`.)
      var toStart = sub(p0, point);
      if (i > 1 && norm(toStart) <= step && dot(toStart, dir) > 0) {
        steps.push({ from: point, predicted: p0, point: p0, q: start(a, r, R, 0.6), iterations: 0, closed: true });
        status = "closed";
        break;
      }
      var predicted = add(point, dir, step);
      var c = corrector(A, B, q, predicted, dir);
      var entry = { from: point, predicted: predicted, point: c.point, q: c.q, iterations: c.iterations, converged: c.converged, residuals: c.history };
      steps.push(entry);
      if (!c.converged) { status = "the corrector did not converge in " + ITERATIONS + " iterations: lost the curve"; break; }
      var next = tangent(A, B, c.q);
      if (!next) { status = "normals parallel: no tangent to go on along (a node of the curve)"; break; }
      // The direction continues the last one, as in the kernel.
      var turned = dot(next, dir) < 0 ? [-next[0], -next[1], -next[2]] : next;
      entry.turn = Math.acos(Math.max(-1, Math.min(1, dot(turned, dir))));
      point = c.point; q = c.q; dir = turned;
    }
    return { steps: steps, status: status, start: p0, startQ: start(a, r, R, 0.6) };
  }

  // Sharp residual on both surfaces of a point of the trace: both implicit
  // equations hold. For the test, and for the readout's honesty.
  function offSurfaces(p, a, r, R) {
    return [Math.abs(norm(p) - R), Math.abs(Math.hypot(p[0] - a, p[1]) - r)];
  }

  GeopBook.demo("tracing", function (ctx) {
    var D = GeopBook.draw;
    var a = 0.35, r = 0.5, R = 1, step = 0.2, res = null, shown = 0, pcurve = true, timer = null;
    var stage = ctx.stage({ aspect: 1.05, maxHeight: 640, label: "A sphere and a cylinder, the traced intersection with predicted and corrected points, and its pcurve on the sphere", draw: paint });
    var cam = ctx.camera(stage, { yaw: 0.9, pitch: 0.95, scale: 0.34, distance: 7 });
    var read = ctx.readout();

    function recompute() {
      res = march(a, r, R, step, 400);
      shown = Math.min(shown, res.steps.length);
      stepSlider.el.max = Math.max(res.steps.length, 1);
      stepSlider.value = shown;
      describe(); stage.redraw();
    }
    function describe() {
      var s = "step " + shown + " of " + res.steps.length + ". The whole march: " + res.status + ".";
      var e = res.steps[shown - 1];
      if (e && e.residuals) {
        s += "\nthis step: " + e.iterations + " corrector iterations; residual " + e.residuals.slice(0, 6).map(function (v) { return v.toExponential(1); }).join(" -> ");
      }
      if (e && e.turn !== undefined) s += "\nthe tangent turned " + (e.turn * 180 / Math.PI).toFixed(1) + " degrees over it";
      read.set(s);
    }
    function paint(g, W, H, c) {
      if (!res) return;
      var split = pcurve ? H * 0.68 : H, P = cam.project;
      g.save(); g.beginPath(); g.rect(0, 0, W, split); g.clip();
      var realH = stage.h;
      stage.h = split;
      // Sphere: outline by meridians and parallels.
      var u, v, i;
      for (v = -60; v <= 60; v += 30) {
        var ring = [];
        for (i = 0; i <= 48; i++) { u = i / 48 * 2 * Math.PI; ring.push(P(sphere(R).p(u, v * Math.PI / 180))); }
        D.polyline(g, null, ring, c.faint, 1);
      }
      for (u = 0; u < 180; u += 30) {
        var mer = [];
        for (i = 0; i <= 48; i++) { v = i / 48 * 2 * Math.PI; var cu = Math.cos(u * Math.PI / 180), su = Math.sin(u * Math.PI / 180); mer.push(P([R * Math.cos(v) * cu, R * Math.cos(v) * su, R * Math.sin(v)])); }
        D.polyline(g, null, mer, c.faint, 1);
      }
      // Cylinder: its two rims and some rulings.
      var cyl = cylinder(a, r);
      [-1.2, 1.2].forEach(function (z) {
        var rim = [];
        for (i = 0; i <= 48; i++) rim.push(P(cyl.p(i / 48 * 2 * Math.PI, z)));
        D.polyline(g, null, rim, c.fillB.replace(/[\d.]+\)$/, "0.8)"), 1);
      });
      for (i = 0; i < 12; i++) {
        var sa = i / 12 * 2 * Math.PI;
        D.polyline(g, null, [P(cyl.p(sa, -1.2)), P(cyl.p(sa, 1.2))], c.fillB, 1);
      }
      // The trace so far.
      var path = [];
      for (i = 0; i < shown && i < res.steps.length; i++) {
        var e = res.steps[i];
        if (!path.length) path.push(P(e.from));
        path.push(P(e.point));
      }
      D.polyline(g, null, path, c.accent, 2.5);
      path.forEach(function (q) { D.dot(g, null, q, 2.5, c.accent); });
      var cur = res.steps[shown - 1];
      if (cur) {
        D.polyline(g, null, [P(cur.from), P(cur.predicted)], c.orange, 1.5, [4, 3]);
        D.dot(g, null, P(cur.predicted), 4, c.orange);
        D.polyline(g, null, [P(cur.predicted), P(cur.point)], c.red, 1.5);
        D.dot(g, null, P(cur.point), 4, c.red);
      }
      stage.h = realH;
      g.restore();
      D.text(g, null, [8, 16], "predicted", c.orange);
      D.text(g, null, [8, 32], "corrected", c.red);
      if (!pcurve) return;
      // The pcurve on the sphere: the march's (u, v).
      var y0 = split + 6, h = H - y0 - 16, x0 = 40, w = W - x0 - 10;
      g.strokeStyle = c.faint; g.strokeRect(x0, y0, w, h);
      function X(uu) { return x0 + (uu + Math.PI) / (2 * Math.PI) * w; }
      function Y(vv) { return y0 + h - (vv + Math.PI / 2) / Math.PI * h; }
      var uv = [];
      if (shown > 0 && res.startQ) uv.push([X(Math.atan2(Math.sin(res.startQ[0]), Math.cos(res.startQ[0]))), Y(res.startQ[1])]);
      for (i = 0; i < shown && i < res.steps.length; i++) {
        var q = res.steps[i].q;
        if (q) uv.push([X(Math.atan2(Math.sin(q[0]), Math.cos(q[0]))), Y(q[1])]);
      }
      var seg = [];
      uv.forEach(function (pt, k) {
        if (k && Math.abs(pt[0] - uv[k - 1][0]) > w / 2) { D.polyline(g, null, seg, c.accent, 2); seg = []; }
        seg.push(pt);
      });
      D.polyline(g, null, seg, c.accent, 2);
      uv.forEach(function (pt) { D.dot(g, null, pt, 2, c.accent); });
      D.text(g, null, [x0 + 4, y0 + 13], "pcurve on the sphere, (u, v)", c.muted);
      D.text(g, null, [x0, H - 3], "u = -π", c.muted);
      D.text(g, null, [x0 + w, H - 3], "π", c.muted, "right");
    }
    function show(v) { shown = v; stepSlider.value = v; describe(); stage.redraw(); }
    function stop() { if (timer) { clearInterval(timer); timer = null; playBtn.textContent = "Play"; } }
    var playBtn = ctx.button("Play", function () {
      if (timer) { stop(); return; }
      if (shown >= res.steps.length) show(0);
      playBtn.textContent = "Pause";
      timer = setInterval(function () {
        if (!ctx.visible) return;
        if (shown >= res.steps.length) { stop(); return; }
        show(shown + 1);
      }, 200);
    });
    ctx.button("Step", function () { stop(); show(Math.min(shown + 1, res.steps.length)); });
    ctx.button("All", function () { stop(); show(res.steps.length); });
    ctx.button("Reset", function () { stop(); show(0); });
    var stepSlider = ctx.slider({
      label: "at step", min: 0, max: 1, step: 1, value: 0, format: function (v) { return String(v); },
      oninput: function (v) { stop(); shown = v; describe(); stage.redraw(); }
    });
    ctx.slider({ label: "stride", min: 0.02, max: 1.2, step: 0.01, value: step, oninput: function (v) { step = v; recompute(); } });
    ctx.slider({ label: "cylinder offset a", min: 0.1, max: 0.5, step: 0.005, value: a, format: function (v) { return v.toFixed(3); }, oninput: function (v) { a = v; recompute(); } });
    ctx.checkbox({ label: "pcurve", value: true, oninput: function (v) { pcurve = v; stage.redraw(); } });
    recompute();
    show(res.steps.length);
  }, { sphere: sphere, cylinder: cylinder, tangent: tangent, corrector: corrector, start: start, march: march, offSurfaces: offSurfaces });
})();
