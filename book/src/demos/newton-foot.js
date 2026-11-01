// Projecting a point onto a curve: Gauss-Newton against Newton on the squared
// distance, on a circle or an ellipse. The same two steps `NurbSurface::project`
// chooses between, one parameter down. Pure functions in `algo`.
(function () {
  "use strict";

  // A curve is C(t), C'(t), C''(t).
  function circle(R) {
    return {
      c: function (t) { return [R * Math.cos(t), R * Math.sin(t)]; },
      d: function (t) { return [-R * Math.sin(t), R * Math.cos(t)]; },
      dd: function (t) { return [-R * Math.cos(t), -R * Math.sin(t)]; }
    };
  }
  function ellipse(a, b) {
    return {
      c: function (t) { return [a * Math.cos(t), b * Math.sin(t)]; },
      d: function (t) { return [-a * Math.sin(t), b * Math.cos(t)]; },
      dd: function (t) { return [-a * Math.cos(t), -b * Math.sin(t)]; }
    };
  }
  function dot(u, v) { return u[0] * v[0] + u[1] * v[1]; }

  // The step of the first fundamental form alone: dt = r.C' / C'.C'.
  function gaussNewtonStep(curve, target, t) {
    var p = curve.c(t), r = [target[0] - p[0], target[1] - p[1]], d = curve.d(t);
    return dot(r, d) / dot(d, d);
  }
  // Newton's on f = |r|^2 / 2: f' = -r.C', f'' = C'.C' - r.C''. Where f'' is not
  // positive this is no step towards a minimum, and Gauss-Newton's is taken, as
  // `NurbSurface::project` does where its Hessian is not definitely positive definite.
  function newtonStep(curve, target, t) {
    var p = curve.c(t), r = [target[0] - p[0], target[1] - p[1]], d = curve.d(t), dd = curve.dd(t);
    var h = dot(d, d) - dot(r, dd);
    if (h > 0) return { step: dot(r, d) / h, newton: true };
    return { step: dot(r, d) / dot(d, d), newton: false };
  }
  function iterateGN(curve, target, t0, n) {
    var out = [t0], t = t0;
    for (var i = 0; i < n; i++) { t += gaussNewtonStep(curve, target, t); out.push(t); }
    return out;
  }
  function iterateNewton(curve, target, t0, n) {
    var out = [t0], used = [], t = t0;
    for (var i = 0; i < n; i++) { var s = newtonStep(curve, target, t); t += s.step; used.push(s.newton); out.push(t); }
    return { ts: out, used: used };
  }
  // The foot point nearest the target: the best of many samples, then Newton. Only
  // used to measure the iterates' errors in the readout, never as an iterate.
  function truth(curve, target) {
    var best = 0, bd = Infinity;
    for (var i = 0; i < 2000; i++) {
      var t = -Math.PI + 2 * Math.PI * i / 2000, p = curve.c(t), d = Math.hypot(p[0] - target[0], p[1] - target[1]);
      if (d < bd) { bd = d; best = t; }
    }
    for (var k = 0; k < 40; k++) best += newtonStep(curve, target, best).step;
    return best;
  }
  function wrap(e) { return Math.atan2(Math.sin(e), Math.cos(e)); }

  GeopBook.demo("newton-foot", function (ctx) {
    var D = GeopBook.draw;
    var target = [2.6, 1.5], pts = [target], shape = "circle", curve = circle(1), t0 = 0.2, n = 12;
    var stage = ctx.stage({ aspect: 1.2, maxHeight: 460, label: "A curve, a target point, and the foot points of the iterates of Gauss-Newton and of Newton", draw: paint });
    var view = ctx.view(stage, { x0: -3.4, x1: 3.4, y0: -2.6, y1: 2.6 });
    ctx.handles({ stage: stage, view: view, pts: pts, onchange: function () { update(); } });
    var read = ctx.readout();
    function paint(g, W, H, c) {
      if (!view) return;
      view.update();
      D.grid(g, view, 1, c.grid);
      var poly = [];
      for (var i = 0; i <= 160; i++) poly.push(curve.c(-Math.PI + 2 * Math.PI * i / 160));
      D.polyline(g, view, poly, c.fg, 2);
      function chain(ts, col, off) {
        var path = ts.map(function (t) { return curve.c(t); });
        ts.forEach(function (t, k) { D.polyline(g, view, [target, curve.c(t)], col, 0.5); });
        D.polyline(g, view, path, col, 1.2);
        path.forEach(function (p, k) { D.dot(g, view, p, k === path.length - 1 ? 5 : 3, col); });
      }
      chain(iterateGN(curve, target, t0, n), c.blue);
      chain(iterateNewton(curve, target, t0, n).ts, c.orange);
      D.dot(g, view, curve.c(t0), 5, c.bg, c.fg);
      D.dot(g, view, target, 6, c.red);
      D.text(g, view, [target[0] + 0.08, target[1] + 0.08], "target", c.muted);
      D.text(g, null, [8, 16], "Gauss-Newton", c.blue);
      D.text(g, null, [8, 32], "Newton", c.orange);
    }
    function update() {
      var star = truth(curve, target), g = iterateGN(curve, target, t0, n), N = iterateNewton(curve, target, t0, n);
      var dist = Math.hypot(curve.c(star)[0] - target[0], curve.c(star)[1] - target[1]);
      var s = "distance to the foot point " + dist.toFixed(3) + (shape === "circle" ? ", d / R = " + dist.toFixed(3) + " (Gauss-Newton's error is multiplied by about -d / R when outside)" : "") + "\n";
      s += " k   |error| Gauss-Newton   |error| Newton\n";
      for (var k = 0; k <= Math.min(n, 8); k++) {
        s += String(k).padStart(2) + "   " + Math.abs(wrap(g[k] - star)).toExponential(3).padStart(20) + "   " + Math.abs(wrap(N.ts[k] - star)).toExponential(3).padStart(14) + (k < N.used.length && !N.used[k] ? "  (next step GN)" : "") + "\n";
      }
      read.set(s);
      stage.redraw();
    }
    ctx.select({
      label: "curve", options: [["circle", "circle, R = 1"], ["ellipse", "ellipse 2 x 1"]], value: "circle",
      oninput: function (v) { shape = v; curve = v === "circle" ? circle(1) : ellipse(2, 1); update(); }
    });
    ctx.slider({ label: "seed t", min: -3.1, max: 3.1, step: 0.01, value: t0, oninput: function (v) { t0 = v; update(); } });
    ctx.slider({ label: "iterations", min: 1, max: 30, step: 1, value: n, format: function (v) { return String(v); }, oninput: function (v) { n = v; update(); } });
    update();
  }, { circle: circle, ellipse: ellipse, gaussNewtonStep: gaussNewtonStep, newtonStep: newtonStep, iterateGN: iterateGN, iterateNewton: iterateNewton, truth: truth, wrap: wrap });
})();
