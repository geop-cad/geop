// Three views of a rolling-ball blend, each a computation:
//  mode "section": the ball in a cross-section between two quadratic Bezier curves
//    meeting at a corner, its centre found as the kernel's `meet` finds it: Gauss-Newton
//    on "r from each curve, along the normal at the foot point".
//  mode "rim": a ball rolled round the rim of a disc, the blend a piece of a torus.
//  mode "tilt": how a section interpolated through exact stations dips through the face
//    between them, f = into . n along the contact, and what a tilt does about it.
(function () {
  "use strict";
  function q(P, t) { var s = 1 - t; return [s * s * P[0][0] + 2 * s * t * P[1][0] + t * t * P[2][0], s * s * P[0][1] + 2 * s * t * P[1][1] + t * t * P[2][1]]; }
  function qd(P, t) { var s = 1 - t; return [2 * (s * (P[1][0] - P[0][0]) + t * (P[2][0] - P[1][0])), 2 * (s * (P[1][1] - P[0][1]) + t * (P[2][1] - P[1][1]))]; }
  function qdd(P) { return [2 * (P[2][0] - 2 * P[1][0] + P[0][0]), 2 * (P[2][1] - 2 * P[1][1] + P[0][1])]; }

  // The foot point of X on the quadratic: scan for a seed, then Newton on the
  // squared distance. In this demo the domain is carried past [0, 1] so a contact
  // past a curve's end is seen as such rather than clamped.
  function foot(P, X, tSeed) {
    var best = tSeed, bd = Infinity, i;
    if (tSeed === undefined) {
      for (i = -8; i <= 28; i++) {
        var t0 = i / 20, p0 = q(P, t0), d0 = (p0[0] - X[0]) * (p0[0] - X[0]) + (p0[1] - X[1]) * (p0[1] - X[1]);
        if (d0 < bd) { bd = d0; best = t0; }
      }
    }
    var t = best, dd = qdd(P);
    for (i = 0; i < 20; i++) {
      var p = q(P, t), d = qd(P, t), r = [p[0] - X[0], p[1] - X[1]];
      var f = r[0] * d[0] + r[1] * d[1], fp = d[0] * d[0] + d[1] * d[1] + r[0] * dd[0] + r[1] * dd[1];
      if (fp <= 0) fp = d[0] * d[0] + d[1] * d[1];
      var step = f / fp;
      t -= step;
      if (Math.abs(step) < 1e-14) break;
    }
    return t;
  }
  function normalOf(P, t, side) {
    var d = qd(P, t), l = Math.hypot(d[0], d[1]);
    return [-d[1] / l * side, d[0] / l * side];
  }
  // The ball of radius r touching A and B, on the given sides of each.
  // Returns { C, ta, tb, Ta, Tb, residual, iterations } or null.
  function ball(A, B, r, sideA, sideB, seed) {
    var C = seed.slice(), ta, tb, it;
    for (it = 0; it < 40; it++) {
      ta = foot(A, C); tb = foot(B, C);
      var pa = q(A, ta), pb = q(B, tb), na = normalOf(A, ta, sideA), nb = normalOf(B, tb, sideB);
      var ra = na[0] * (C[0] - pa[0]) + na[1] * (C[1] - pa[1]) - r;
      var rb = nb[0] * (C[0] - pb[0]) + nb[1] * (C[1] - pb[1]) - r;
      var det = na[0] * nb[1] - na[1] * nb[0];
      if (Math.abs(det) < 1e-9) return null;
      var dx = (ra * nb[1] - rb * na[1]) / det, dy = (na[0] * rb - nb[0] * ra) / det;
      C[0] -= dx; C[1] -= dy;
      if (Math.hypot(dx, dy) < 1e-13) break;
    }
    ta = foot(A, C); tb = foot(B, C);
    var Ta = q(A, ta), Tb = q(B, tb);
    var residual = Math.max(Math.abs(Math.hypot(C[0] - Ta[0], C[1] - Ta[1]) - r), Math.abs(Math.hypot(C[0] - Tb[0], C[1] - Tb[1]) - r));
    return { C: C, ta: ta, tb: tb, Ta: Ta, Tb: Tb, residual: residual, iterations: it + 1 };
  }
  // The side of each curve the other one lies on, and a seed along the bisector
  // at the shared corner A(0) = B(0).
  function setup(A, B, r) {
    var ta = qd(A, 0), tb = qd(B, 0), la = Math.hypot(ta[0], ta[1]), lb = Math.hypot(tb[0], tb[1]);
    ta = [ta[0] / la, ta[1] / la]; tb = [tb[0] / lb, tb[1] / lb];
    var cr = ta[0] * tb[1] - ta[1] * tb[0];
    var sideA = cr > 0 ? 1 : -1, sideB = cr > 0 ? -1 : 1;
    var bis = [ta[0] + tb[0], ta[1] + tb[1]], lb2 = Math.hypot(bis[0], bis[1]) || 1;
    var half = Math.acos(Math.max(-1, Math.min(1, ta[0] * tb[0] + ta[1] * tb[1]))) / 2;
    var dist = r / Math.max(Math.sin(half), 0.05);
    return { sideA: sideA, sideB: sideB, seed: [A[0][0] + bis[0] / lb2 * dist, A[0][1] + bis[1] / lb2 * dist] };
  }

  // ---- the tilt: a contact curve along t, the face's normal turning as
  // n(s) = (sin phi, 0, cos phi), phi = a sin(w s), and the section's direction into
  // the blend b(s) = (cos phi, 0, -sin phi), exactly tangent to the face.
  function phi(a, w, s) { return a * Math.sin(w * s); }
  function normal(a, w, s) { var p = phi(a, w, s); return [Math.sin(p), Math.cos(p)]; }
  function into(a, w, s) { var p = phi(a, w, s); return [Math.cos(p), -Math.sin(p)]; }
  // The C2 cubic spline through values at knots (end slopes by a one-sided difference): solves the tridiagonal for slopes.
  function splineSlopes(h, y) {
    var n = y.length - 1, a = new Array(n + 1), b = new Array(n + 1), c = new Array(n + 1), d = new Array(n + 1), i;
    b[0] = 1; c[0] = 0; b[n] = 1; a[n] = 0;
    if (n >= 4) {
      d[0] = (-25 * y[0] + 48 * y[1] - 36 * y[2] + 16 * y[3] - 3 * y[4]) / (12 * h);
      d[n] = (25 * y[n] - 48 * y[n - 1] + 36 * y[n - 2] - 16 * y[n - 3] + 3 * y[n - 4]) / (12 * h);
    } else {
      d[0] = (-3 * y[0] + 4 * y[1] - y[2]) / (2 * h);
      d[n] = (3 * y[n] - 4 * y[n - 1] + y[n - 2]) / (2 * h);
    }
    for (i = 1; i < n; i++) { a[i] = 1; b[i] = 4; c[i] = 1; d[i] = 3 * (y[i + 1] - y[i - 1]) / h; }
    for (i = 1; i <= n; i++) { var m = a[i] / b[i - 1]; b[i] -= m * c[i - 1]; d[i] -= m * d[i - 1]; }
    var out = new Array(n + 1);
    out[n] = d[n] / b[n];
    for (i = n - 1; i >= 0; i--) out[i] = (d[i] - c[i] * out[i + 1]) / b[i];
    return out;
  }
  function hermite(h, y, m, s) {
    var k = Math.min(y.length - 2, Math.max(0, Math.floor(s / h))), t = (s - k * h) / h, t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * y[k] + (t3 - 2 * t2 + t) * h * m[k] + (-2 * t3 + 3 * t2) * y[k + 1] + (t3 - t2) * h * m[k + 1];
  }
  // f(s) = (interpolated section direction) . n(s), at `samples`+1 points of [0, 1],
  // `spans` spans between stations, a tilt toward the normal at every station.
  function tiltProfile(a, w, spans, tilt, samples) {
    var h = 1 / spans, xs = [], zs = [], i;
    for (i = 0; i <= spans; i++) {
      var s = i * h, b = into(a, w, s), n = normal(a, w, s);
      xs.push(b[0] + tilt * n[0]); zs.push(b[1] + tilt * n[1]);
    }
    var mx = splineSlopes(h, xs), mz = splineSlopes(h, zs), out = [];
    for (i = 0; i <= samples; i++) {
      var u = i / samples, nn = normal(a, w, u);
      out.push(hermite(h, xs, mx, u) * nn[0] + hermite(h, zs, mz, u) * nn[1]);
    }
    return out;
  }
  // How many stretches of f lie below zero, away from the stations' rounding.
  function dips(f, eps) {
    var count = 0, inside = false;
    for (var i = 0; i < f.length; i++) {
      if (f[i] < -eps) { if (!inside) count++; inside = true; } else inside = false;
    }
    return count;
  }

  GeopBook.demo("rolling-ball", function (ctx) {
    var D = GeopBook.draw, mode = ctx.args.mode || "section";

    if (mode === "section") {
      var A = [[0.6, 0.5], [3.6, 0.2], [6.2, 1.2]], B = [[0.6, 0.5], [0.3, 2.2], [1.6, 3.9]];
      var r = 0.8, res = null;
      var pts = [A[0], A[1], A[2], B[1], B[2]];
      var stage = ctx.stage({ aspect: 1.5, maxHeight: 400, label: "A ball between two curves", draw: paintS });
      var view = ctx.view(stage, { x0: 0, x1: 6.6, y0: 0, y1: 4.4 });
      var handles = ctx.handles({
        stage: stage, view: view, pts: pts, onchange: updateS,
        constrain: function (i, p) { if (i === 0) { B[0][0] = p[0]; B[0][1] = p[1]; } return p; }
      });
      var read = ctx.readout();
      ctx.slider({ label: "radius r", min: 0.1, max: 1.8, step: 0.01, value: r, oninput: function (v) { r = v; updateS(); } });
      var showOffsets = ctx.checkbox({ label: "offset curves", value: true, oninput: function () { stage.redraw(); } });
      function offsetCurve(P, side) {
        var o = [];
        for (var i = 0; i <= 80; i++) { var t = -0.2 + 1.4 * i / 80, p = q(P, t), n = normalOf(P, t, side); o.push([p[0] + r * n[0], p[1] + r * n[1]]); }
        return o;
      }
      function paintS(g, w, h, c) {
        if (!view) return;
        view.update();
        var cv = function (P, a, b) { var o = []; for (var i = 0; i <= 80; i++) o.push(q(P, a + (b - a) * i / 80)); return o; };
        D.polyline(g, view, cv(A, 0, 1), c.fg, 2.4); D.polyline(g, view, cv(B, 0, 1), c.fg, 2.4);
        D.polyline(g, view, cv(A, 1, 1.25), c.faint, 1.5, [4, 4]); D.polyline(g, view, cv(B, 1, 1.25), c.faint, 1.5, [4, 4]);
        var S = setup(A, B, r);
        if (showOffsets && showOffsets.value) {
          D.polyline(g, view, offsetCurve(A, S.sideA), c.blue, 1.2, [5, 4]);
          D.polyline(g, view, offsetCurve(B, S.sideB), c.orange, 1.2, [5, 4]);
        }
        if (res) {
          g.beginPath(); g.arc(view.x(res.C[0]), view.y(res.C[1]), r * view.sx, 0, 2 * Math.PI);
          g.fillStyle = c.fillC; g.fill(); g.strokeStyle = c.green; g.lineWidth = 1.5; g.stroke();
          D.polyline(g, view, [res.C, res.Ta], c.muted, 1); D.polyline(g, view, [res.C, res.Tb], c.muted, 1);
          D.dot(g, view, res.C, 4, c.green); D.dot(g, view, res.Ta, 5, c.blue); D.dot(g, view, res.Tb, 5, c.orange);
          D.text(g, view, [res.C[0] + 0.08, res.C[1] + 0.08], "C", c.green);
          D.text(g, view, [res.Ta[0] + 0.06, res.Ta[1] - 0.22], "Ta", c.blue);
          D.text(g, view, [res.Tb[0] + 0.1, res.Tb[1]], "Tb", c.orange);
        }
        pts.forEach(function (p, i) { D.dot(g, view, p, i === 0 ? 6 : 5, handles && handles.selected === i ? c.accent : c.bg, c.fg); });
      }
      function updateS() {
        var S = setup(A, B, r);
        res = ball(A, B, r, S.sideA, S.sideB, S.seed);
        if (!res) read.set("The two normals could not be told apart: no ball found.");
        else {
          var bad = res.ta < 0 || res.ta > 1 || res.tb < 0 || res.tb > 1;
          read.set("C = (" + res.C[0].toFixed(4) + ", " + res.C[1].toFixed(4) + "), Gauss-Newton steps " + res.iterations + ", largest miss " + res.residual.toExponential(1) + "\n" +
            "contacts at ta = " + res.ta.toFixed(4) + ", tb = " + res.tb.toFixed(4) +
            (bad ? "\nA contact lies past the end of its curve: the ball is too large for this face, and the kernel refuses such a blend." : ""));
        }
        stage.redraw();
      }
      updateS();
      return;
    }

    if (mode === "rim") {
      var R = 2, rad = 0.6, th = 1.1;
      var st = ctx.stage({ aspect: 1.4, maxHeight: 400, label: "A ball rolled round the rim of a disc", draw: paintR });
      var cam = ctx.camera(st, { yaw: 0.4, pitch: 0.6, scale: 0.2, distance: 14 });
      var rd = ctx.readout();
      ctx.slider({ label: "ball radius", min: 0.15, max: 1.2, step: 0.01, value: rad, oninput: function (v) { rad = v; show(); } });
      ctx.player({ duration: 8, loop: true, label: "along the edge", onframe: function (v) { th = v * 2 * Math.PI; st.redraw(); } }).set(0.68);
      // The disc's top is z = 0, its wall radius R. Ball centre at radius R - r, height -r.
      function torusPoint(a, b) {
        // b in [0, pi/2]: from the contact on the top to the contact on the wall.
        var rho = R - rad + rad * Math.sin(b), z = -rad + rad * Math.cos(b);
        return [rho * Math.cos(a), rho * Math.sin(a), z];
      }
      function paintR(g, w, h, c) {
        if (!cam) return;
        var polys = [], i, j, N = 48;
        for (i = 0; i < N; i++) {
          var a0 = 2 * Math.PI * i / N, a1 = 2 * Math.PI * (i + 1) / N;
          // The wall below the fillet.
          var wall = [[R * Math.cos(a0), R * Math.sin(a0), -rad], [R * Math.cos(a1), R * Math.sin(a1), -rad], [R * Math.cos(a1), R * Math.sin(a1), -1.6], [R * Math.cos(a0), R * Math.sin(a0), -1.6]].map(cam.project);
          polys.push({ q: wall, fill: c.fillB });
          // The top inside the fillet.
          var r0 = R - rad;
          var top = [[r0 * Math.cos(a0), r0 * Math.sin(a0), 0], [r0 * Math.cos(a1), r0 * Math.sin(a1), 0], [0, 0, 0]].map(cam.project);
          polys.push({ q: top, fill: c.fillB });
          for (j = 0; j < 5; j++) {
            var b0 = (Math.PI / 2) * j / 5, b1 = (Math.PI / 2) * (j + 1) / 5;
            var tq = [torusPoint(a0, b0), torusPoint(a1, b0), torusPoint(a1, b1), torusPoint(a0, b1)].map(cam.project);
            polys.push({ q: tq, fill: c.fillA });
          }
        }
        polys.forEach(function (p) { p.d = p.q.reduce(function (s, v) { return s + v[2]; }, 0) / p.q.length; });
        polys.sort(function (x, y) { return x.d - y.d; });
        polys.forEach(function (p) { D.polygon(g, null, p.q, p.fill, null); });
        function circle(rho, z) { var o = []; for (var k = 0; k <= 96; k++) { var t = 2 * Math.PI * k / 96; o.push(cam.project([rho * Math.cos(t), rho * Math.sin(t), z])); } return o; }
        D.polyline(g, null, circle(R - rad, -rad), c.green, 1.5, [5, 4]); // the spine
        D.polyline(g, null, circle(R - rad, 0), c.blue, 2); // contact on the top
        D.polyline(g, null, circle(R, -rad), c.orange, 2); // contact on the wall
        D.polyline(g, null, circle(R, 0), c.faint, 1, [2, 4]); // the edge that was
        var ca = [(R - rad) * Math.cos(th), (R - rad) * Math.sin(th), -rad], cc = cam.project(ca);
        var size = rad * Math.min(st.w, st.h) * cam.scale * cam.distance / (cam.distance - cc[2]);
        g.beginPath(); g.arc(cc[0], cc[1], size, 0, 2 * Math.PI); g.fillStyle = c.fillC; g.fill(); g.strokeStyle = c.green; g.lineWidth = 1.2; g.stroke();
        var arc = []; for (var k = 0; k <= 16; k++) arc.push(cam.project(torusPoint(th, (Math.PI / 2) * k / 16)));
        D.polyline(g, null, arc, c.red, 3);
      }
      function show() {
        rd.set("spine (dashed): radius " + (R - rad).toFixed(2) + " at depth " + rad.toFixed(2) + ".  Contacts: on the top, radius " + (R - rad).toFixed(2) + "; on the wall, depth " + rad.toFixed(2) + ".");
        st.redraw();
      }
      show();
      return;
    }

    // mode "tilt"
    var spans = 6, logTilt = -3, on = false, a = 0.5, w0 = 5;
    var plot = ctx.stage({ aspect: 1.9, maxHeight: 300, label: "How far the section between stations leans into the face", draw: paintT });
    var out = ctx.readout();
    ctx.slider({ label: "spans", min: 3, max: 48, step: 1, value: spans, format: function (v) { return "" + v; }, oninput: function (v) { spans = v; showT(); } });
    ctx.slider({ label: "tilt 10^", min: -8, max: -1, step: 0.1, value: logTilt, format: function (v) { return v.toFixed(1); }, oninput: function (v) { logTilt = v; showT(); } });
    ctx.checkbox({ label: "tilt on", value: false, oninput: function (v) { on = v; showT(); } });
    function profile() { return tiltProfile(a, w0, spans, on ? Math.pow(10, logTilt) : 0, 600); }
    function paintT(g, W, H, c) {
      var f = profile(), max = 0, i;
      for (i = 0; i < f.length; i++) max = Math.max(max, Math.abs(f[i]));
      max = max || 1;
      var x = function (s) { return 8 + (W - 16) * s; }, y = function (v) { return H / 2 - (H / 2 - 16) * v / max; };
      g.fillStyle = c.fillB; g.fillRect(8, H / 2, W - 16, H / 2 - 4);
      g.strokeStyle = c.muted; g.lineWidth = 1; g.beginPath(); g.moveTo(8, H / 2); g.lineTo(W - 8, H / 2); g.stroke();
      for (i = 0; i <= spans; i++) D.dot(g, null, [x(i / spans), y(on ? Math.pow(10, logTilt) : 0)], 3.5, c.fg);
      D.polyline(g, null, f.map(function (v, k) { return [x(k / (f.length - 1)), y(v)]; }), c.blue, 2);
      g.fillStyle = c.muted; g.textAlign = "left"; g.textBaseline = "top";
      g.fillText("into · n: above the line, on the ball's side", 12, 6);
      g.textBaseline = "alphabetic"; g.fillText("below: through the face. Scale ±" + max.toExponential(1), 12, H - 10);
    }
    function showT() {
      var f = profile(), base = tiltProfile(a, w0, spans, 0, 600), err = 0;
      base.forEach(function (v) { err = Math.max(err, Math.abs(v)); });
      var d = dips(f, 1e-13);
      out.set("largest lean of the untilted section between stations: " + err.toExponential(2) + " rad\n" +
        (on ? "tilt " + Math.pow(10, logTilt).toExponential(1) + " rad: " : "no tilt: ") + d + " stretch" + (d === 1 ? "" : "es") + " through the face");
      plot.redraw();
    }
    showT();
  }, {
    q: q, ball: ball, setup: setup, foot: foot, tiltProfile: tiltProfile, dips: dips
  });
})();
