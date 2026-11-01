// What a solver's coordinates do to it: a link pinned to a target nearly a half
// turn away, solved by the same damped Gauss-Newton in two sets of variables.
// (1, w/2) normalized (the kernel's old pose variables) against a step of a twist
// from where the body is, through the exponential map (what it does now).
(function () {
  "use strict";

  // ---- quaternions (w, x, y, z)
  function qmul(a, b) {
    return [
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    ];
  }
  function qrotate(q, v) { // q unit
    var r = qmul(qmul(q, [0, v[0], v[1], v[2]]), [q[0], -q[1], -q[2], -q[3]]);
    return [r[1], r[2], r[3]];
  }
  function qnorm(q) { var n = Math.hypot(q[0], q[1], q[2], q[3]); return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]; }
  function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross3(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }

  // The rigid motion of a twist (omega, v), as Pose::exp's documentation has it:
  // with phi = |omega| / 2, s = sin(phi) / (2 phi), c = (cos(phi) - sin(phi)/phi) / (8 phi²),
  // rotation (cos phi, s omega), position 2 s cos(phi) v + (s² + 2 c cos(phi)) (omega·v) omega + 2 s² omega × v.
  function exp(omega, v) {
    var phi = Math.sqrt(dot3(omega, omega)) / 2;
    var s = phi < 1e-4 ? 0.5 * (1 - phi * phi / 6) : Math.sin(phi) / (2 * phi);
    var c = phi < 1e-2 ? -1 / 24 : (Math.cos(phi) - Math.sin(phi) / phi) / (8 * phi * phi);
    var cs = Math.cos(phi), ov = dot3(omega, v), ox = cross3(omega, v);
    var a = 2 * s * cs, b = (s * s + 2 * c * cs) * ov, d = 2 * s * s;
    return {
      rotation: [cs, s * omega[0], s * omega[1], s * omega[2]],
      position: [a * v[0] + b * omega[0] + d * ox[0], a * v[1] + b * omega[1] + d * ox[1], a * v[2] + b * omega[2] + d * ox[2]]
    };
  }

  // The old variables: the angle (about z) the rotation (1, w/2), normalized, stands for.
  function angleOfW(w) { return 2 * Math.atan(w / 2); }
  function dAngleDw(w) { return 1 / (1 + w * w / 4); }

  // The residual: the link's tip at unit distance, less where it is pinned: two
  // numbers, each a length. Its derivative by the angle is the tip's velocity.
  function tip(theta) { return [Math.cos(theta), Math.sin(theta)]; }

  // One run: minimize |r|² + damping² |step|² each step, a step taken only where
  // it lowers |r| definitely (by a little more than rounding). `coordinates`
  // gives the angle of a variable and how fast it moves it.
  function run(target, damping, mode, maxSteps) {
    var goal = tip(target);
    var q = [1, 0, 0, 0], w = 0;            // where the body is
    var mu = 0;                             // extra damping, grown when a step is turned down
    var trace = [{ theta: 0, w: 0 }];
    function angleNow() { return mode === "exp" ? 2 * Math.atan2(q[3], q[0]) : angleOfW(w); }
    var tol = 1e-9;
    for (var k = 0; k < maxSteps; k++) {
      var th = angleNow(), t = tip(th), r = [t[0] - goal[0], t[1] - goal[1]];
      var size = Math.hypot(r[0], r[1]);
      if (size < tol) break;
      var dThetaDVar = mode === "exp" ? 1 : dAngleDw(w);
      var J = [-Math.sin(th) * dThetaDVar, Math.cos(th) * dThetaDVar];
      var JtJ = J[0] * J[0] + J[1] * J[1], Jtr = J[0] * r[0] + J[1] * r[1];
      var taken = false;
      for (var tries = 0; tries < 30 && !taken; tries++) {
        var step = -Jtr / (JtJ + damping * damping + mu);
        var nq = q, nw = w, nth;
        if (mode === "exp") { nq = qnorm(qmul(q, [Math.cos(step / 2), 0, 0, Math.sin(step / 2)])); nth = 2 * Math.atan2(nq[3], nq[0]); }
        else { nw = w + step; nth = angleOfW(nw); }
        var nt = tip(nth), ns = Math.hypot(nt[0] - goal[0], nt[1] - goal[1]);
        if (ns < size * (1 - 1e-12)) { q = nq; w = nw; taken = true; mu = mu / 10; if (mu < 1e-16) mu = 0; }
        else mu = mu === 0 ? Math.max(JtJ, damping * damping, 1e-12) : mu * 10;
      }
      if (!taken) break;
      trace.push({ theta: angleNow(), w: w });
    }
    return { trace: trace, steps: trace.length - 1, final: trace[trace.length - 1], residual: (function () { var t = tip(trace[trace.length - 1].theta); return Math.hypot(t[0] - goal[0], t[1] - goal[1]); })() };
  }

  GeopBook.demo("pose-exp", function (ctx) {
    var D = GeopBook.draw;
    var target = 175 * Math.PI / 180, damping = 1e-4;
    var A = null, B = null, shown = 0, MAX = 200;
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 400, label: "A link turned to a target in two sets of variables, and the angle against the step", draw: paint });
    var read = ctx.readout();
    var player;

    function recompute() {
      A = run(target, damping, "quat", MAX);
      B = run(target, damping, "exp", MAX);
      stage.redraw();
      report();
    }
    function at(R, k) { return R.trace[Math.min(k, R.trace.length - 1)]; }
    function report() {
      var a = at(A, shown), b = at(B, shown);
      read.set("step " + shown + " of at most " + MAX + "\n" +
        "(1, w/2) normalized: angle " + (a.theta * 180 / Math.PI).toFixed(3) + "°   w = " + a.w.toFixed(2) +
        "   ends after " + A.steps + (A.residual < 1e-8 ? " steps, arrived" : " steps, off by " + A.residual.toExponential(1)) + "\n" +
        "twist through exp:   angle " + (b.theta * 180 / Math.PI).toFixed(3) + "°   ends after " + B.steps + (B.residual < 1e-8 ? " steps, arrived" : " steps, off by " + B.residual.toExponential(1)));
    }
    function dial(g, cx, cy, r, theta, title, c, colour) {
      g.beginPath(); g.arc(cx, cy, r, 0, 2 * Math.PI); g.strokeStyle = c.muted; g.lineWidth = 1; g.stroke();
      var gx = cx + r * Math.cos(target), gy = cy - r * Math.sin(target);
      D.dot(g, null, [gx, gy], 5, null, c.muted);
      g.beginPath(); g.moveTo(cx, cy); g.lineTo(cx + r * Math.cos(theta), cy - r * Math.sin(theta)); g.strokeStyle = colour; g.lineWidth = 4; g.stroke();
      D.dot(g, null, [cx, cy], 3, c.fg);
      D.text(g, null, [cx, cy + r + 15], title, c.fg, "center");
    }
    function paint(g, w, h, c) {
      if (!A) return;
      var top = Math.min(h * 0.42, w * 0.22), r = top * 0.4;
      g.font = "11px sans-serif";
      dial(g, w * 0.27, 10 + r, r, at(A, shown).theta, "(1, w/2) normalized", c, c.orange);
      dial(g, w * 0.73, 10 + r, r, at(B, shown).theta, "twist, exp", c, c.blue);
      // The chart: distance to the target, on a log scale, against the step.
      var x0 = 34, x1 = w - 8, y0 = 10 + 2 * r + 28, y1 = h - 18;
      var X = function (k) { return x0 + (x1 - x0) * k / MAX; };
      var yr = [1e-10, 10], Y = function (v) { var t = (Math.log10(Math.max(v, 1e-10)) - Math.log10(yr[0])) / (Math.log10(yr[1]) - Math.log10(yr[0])); return y1 - (y1 - y0) * t; };
      g.strokeStyle = c.faint; g.lineWidth = 1; g.beginPath();
      [1, 1e-3, 1e-6, 1e-9].forEach(function (v) { g.moveTo(x0, Y(v)); g.lineTo(x1, Y(v)); });
      g.stroke();
      [1, 1e-3, 1e-6, 1e-9].forEach(function (v) { D.text(g, null, [x0 - 3, Y(v) + 3], v === 1 ? "1" : "1e" + Math.round(Math.log10(v)), c.fg, "right"); });
      D.text(g, null, [x1, y1 + 13], "step " + MAX, c.fg, "right");
      D.text(g, null, [x0, y1 + 13], "0", c.fg);
      D.text(g, null, [x0 + 4, y0 + 10], "distance of the tip from where it is pinned", c.fg);
      function curve(R, col) {
        g.beginPath();
        R.trace.forEach(function (t, k) {
          var d = 2 * Math.abs(Math.sin((t.theta - target) / 2));
          if (k) g.lineTo(X(k), Y(d)); else g.moveTo(X(k), Y(d));
        });
        g.strokeStyle = col; g.lineWidth = 2; g.stroke();
      }
      curve(A, c.orange); curve(B, c.blue);
      g.beginPath(); g.moveTo(X(shown), y0); g.lineTo(X(shown), y1); g.strokeStyle = c.muted; g.setLineDash([3, 3]); g.lineWidth = 1; g.stroke(); g.setLineDash([]);
    }
    ctx.slider({ label: "target", min: 90, max: 179.9, step: 0.1, value: 175, format: function (v) { return v.toFixed(1) + "°"; }, oninput: function (v) { target = v * Math.PI / 180; recompute(); } });
    ctx.slider({ label: "damping", min: -6, max: -2, step: 0.1, value: -4, format: function (v) { return "1e" + v.toFixed(1).replace("-", "\u2212"); }, oninput: function (v) { damping = Math.pow(10, v); recompute(); } });
    player = ctx.player({ steps: MAX, duration: 8, label: "step", onframe: function (t) { shown = t; stage.redraw(); if (A) report(); } });
    recompute();
  }, { exp: exp, run: run, angleOfW: angleOfW, dAngleDw: dAngleDw, qmul: qmul, qrotate: qrotate, qnorm: qnorm, tip: tip });
})();
