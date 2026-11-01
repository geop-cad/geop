// Frames along a curve: the Frenet frame, a rotation-minimizing frame carried by
// the double reflection method (Wang, Juttler, Zheng, Liu 2008), and the frame
// a screw motion carries. All computed; `algo` is tested against the closed
// form: on a helix the rotation-minimizing frame turns against the screw frame
// at the rate of the torsion.
(function () {
  "use strict";
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function mul(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  function unit(a) { var l = Math.sqrt(dot(a, a)); return l ? mul(a, 1 / l) : a; }

  // A curve is { x(t), d1(t), d2(t), t0, t1 }: point, first and second derivative.
  function helix(r, pitch) {
    var c = pitch / (2 * Math.PI);
    return {
      x: function (t) { return [r * Math.cos(t), r * Math.sin(t), c * t]; },
      d1: function (t) { return [-r * Math.sin(t), r * Math.cos(t), c]; },
      d2: function (t) { return [-r * Math.cos(t), -r * Math.sin(t), 0]; }
    };
  }
  // A planar cubic with an inflection in the middle: x along, y = a (s^3 - s), in z = 0.
  function sCurve(a) {
    return {
      x: function (t) { return [t, a * (t * t * t - t), 0]; },
      d1: function (t) { return [1, a * (3 * t * t - 1), 0]; },
      d2: function (t) { return [0, 6 * a * t, 0]; }
    };
  }
  // The Frenet normal and binormal. null where the curvature vanishes.
  function frenet(curve, t) {
    var T = unit(curve.d1(t)), a = curve.d2(t);
    var n = sub(a, mul(T, dot(a, T)));
    if (Math.sqrt(dot(n, n)) < 1e-12) return null;
    n = unit(n);
    return { T: T, n: n, b: cross(T, n) };
  }
  // One step of double reflection, applied to v: exactly the formula of the paper.
  function reflectStep(v, x0, x1, t0, t1) {
    function refl(n, w) { var c = dot(n, n); return c === 0 ? w : sub(w, mul(n, 2 * dot(n, w) / c)); }
    var v1 = sub(x1, x0);
    var v2 = sub(t1, refl(v1, t0));
    return refl(v2, refl(v1, v));
  }
  // The vector v0 (square to the tangent at t0) carried to t1 in `steps` steps.
  function carried(curve, t0, t1, steps, v0) {
    var out = [v0], x = curve.x(t0), T = unit(curve.d1(t0)), v = v0;
    for (var i = 1; i <= steps; i++) {
      var t = t0 + (t1 - t0) * i / steps, x1 = curve.x(t), T1 = unit(curve.d1(t));
      v = reflectStep(v, x, x1, T, T1);
      out.push(v);
      x = x1; T = T1;
    }
    return out;
  }
  // The frame a screw motion carries on a helix: the radial direction, and the
  // binormal-like direction square to it and to the tangent.
  function screwVector(r, pitch, t) { return [Math.cos(t), Math.sin(t), 0]; }
  // The angle from u to v, about the axis T (all square to T).
  function angleAbout(T, u, v) { return Math.atan2(dot(cross(u, v), T), dot(u, v)); }
  // The closed form: the frame turns against the screw frame by theta sin(alpha).
  function analyticTwist(r, pitch, theta) {
    var c = pitch / (2 * Math.PI);
    return theta * c / Math.sqrt(r * r + c * c);
  }
  // The rotation-minimizing frame's twist against the screw frame, accumulated
  // over the parameters 0..theta, at each of `steps` steps.
  function twistAgainstScrew(r, pitch, theta, steps) {
    var h = helix(r, pitch), vs = carried(h, 0, theta, steps, screwVector(r, pitch, 0));
    var out = [0], total = 0, last = 0;
    for (var i = 1; i <= steps; i++) {
      var t = theta * i / steps, T = unit(h.d1(t));
      var a = angleAbout(T, screwVector(r, pitch, t), vs[i]);
      var d = a - last;
      while (d > Math.PI) d -= 2 * Math.PI;
      while (d < -Math.PI) d += 2 * Math.PI;
      total += d; last = a;
      out.push(total);
    }
    return out;
  }

  GeopBook.demo("helix-frames", function (ctx) {
    var D = GeopBook.draw;
    var shape = "helix", kind = "rmf", r = 1, pitch = 1.2, turns = 2, steps = 48;
    var stage = ctx.stage({ aspect: 1.35, maxHeight: 380, label: "Frames along a helix or a curve with an inflection", draw: paint });
    var cam = ctx.camera(stage, { yaw: 0.6, pitch: 0.3, scale: 0.2, distance: 16 });
    var plot = ctx.stage({ height: 130, label: "The twist of the rotation-minimizing frame against the screw frame", draw: paintPlot });
    plot.canvas.tabIndex = -1;
    var read = ctx.readout();

    function curveNow() { return shape === "helix" ? helix(r, pitch) : sCurve(0.6); }
    function range() { return shape === "helix" ? [0, turns * 2 * Math.PI] : [-1.6, 1.6]; }
    function frames() {
      var cv = curveNow(), R = range(), N = steps, out = [];
      var T0 = unit(cv.d1(R[0]));
      var start = shape === "helix" ? [1, 0, 0] : unit(cross(T0, [0, 0, 1]));
      var vs = carried(cv, R[0], R[1], N, start);
      for (var i = 0; i <= N; i++) {
        var t = R[0] + (R[1] - R[0]) * i / N, T = unit(cv.d1(t)), v;
        if (kind === "rmf") v = vs[i];
        else if (kind === "frenet") { var f = frenet(cv, t); v = f ? f.n : null; }
        else v = shape === "helix" ? screwVector(r, pitch, t) : unit(cross(T, [0, 0, 1]));
        out.push({ t: t, x: cv.x(t), T: T, v: v, b: v ? cross(T, v) : null });
      }
      return out;
    }
    function paint(g, w, h, c) {
      if (!cam) return;
      var fr = frames();
      var mid = shape === "helix" ? [0, 0, (pitch / (2 * Math.PI)) * range()[1] / 2] : [0, 0, 0];
      cam.centre = mid;
      cam.scale = shape === "helix" ? Math.min(0.2, 1.6 / Math.max(r * 2.6, (pitch * turns) * 1.1, 1) * 0.28) : 0.27;
      var size = shape === "helix" ? 0.35 * r : 0.3;
      // The ribbon the profile's first axis sweeps.
      var quads = [];
      for (var i = 0; i + 1 < fr.length; i++) {
        var a = fr[i], b = fr[i + 1];
        if (!a.v || !b.v) continue;
        var q = [a.x, add(a.x, mul(a.v, size)), add(b.x, mul(b.v, size)), b.x].map(cam.project);
        quads.push({ q: q, d: q[0][2] + q[1][2] + q[2][2] + q[3][2] });
      }
      quads.sort(function (x, y) { return x.d - y.d; });
      quads.forEach(function (f) { D.polygon(g, null, f.q, c.fillA, null); });
      D.polyline(g, null, fr.map(function (f) { return cam.project(f.x); }), c.fg, 2);
      var every = Math.max(1, Math.round(fr.length / 16));
      fr.forEach(function (f, k) {
        if (k % every || !f.v) return;
        var o = cam.project(f.x);
        D.polyline(g, null, [o, cam.project(add(f.x, mul(f.v, size)))], c.orange, 2);
        D.polyline(g, null, [o, cam.project(add(f.x, mul(f.b, size)))], c.green, 2);
        D.dot(g, null, o, 2.5, c.fg);
      });
      if (shape !== "helix" && kind === "frenet") {
        D.text(g, null, [w / 2, 16], "The Frenet normal flips at the inflection (t = 0), where it is undefined", c.red, "center");
      }
    }
    function paintPlot(g, w, h, c) {
      g.fillStyle = c.muted; g.textAlign = "left"; g.textBaseline = "top";
      if (shape !== "helix") { g.fillText("A planar curve: the rotation-minimizing frame does not turn about the tangent.", 8, 8); return; }
      var theta = turns * 2 * Math.PI;
      var tw = twistAgainstScrew(r, pitch, theta, steps);
      var max = Math.max(Math.abs(tw[tw.length - 1]), analyticTwist(r, pitch, theta), 0.1) * 1.1;
      // On a right-handed helix the frame lags the screw frame: plotted by size.
      var x = function (k) { return 44 + (w - 56) * k; }, y = function (v) { return h - 22 - (h - 36) * v / max; };
      g.strokeStyle = c.faint; g.lineWidth = 1; g.beginPath(); g.moveTo(44, y(0)); g.lineTo(w - 12, y(0)); g.moveTo(44, y(0)); g.lineTo(44, 8); g.stroke();
      g.fillText("twist (deg)", 8, 2);
      g.textAlign = "right"; g.fillText((max * 180 / Math.PI).toFixed(0), 40, 14);
      g.textAlign = "center"; g.fillText("turns along the helix", w / 2, h - 16);
      D.polyline(g, null, [[x(0), y(0)], [x(1), y(analyticTwist(r, pitch, theta))]], c.green, 4);
      D.polyline(g, null, tw.map(function (v, k) { return [x(k / steps), y(Math.abs(v))]; }), c.orange, 1.5);
    }
    function update() {
      var c = pitch / (2 * Math.PI), sa = c / Math.sqrt(r * r + c * c);
      var theta = turns * 2 * Math.PI, tw = twistAgainstScrew(r, pitch, theta, steps);
      var computed = Math.abs(tw[tw.length - 1]) * 180 / Math.PI, analytic = analyticTwist(r, pitch, theta) * 180 / Math.PI;
      read.set(shape === "helix" ?
        "pitch angle alpha = " + (Math.asin(sa) * 180 / Math.PI).toFixed(2) + " deg;  per turn the twist is 360 sin(alpha) = " + (360 * sa).toFixed(3) + " deg\n" +
        "double reflection (orange), " + steps + " steps: " + computed.toFixed(6) + " deg over " + turns + " turns;  torsion integral (green): " + analytic.toFixed(6) + " deg" :
        "");
      stage.redraw(); plot.redraw();
    }
    ctx.select({ label: "curve", value: shape, options: [["helix", "helix"], ["s", "curve with an inflection"]], oninput: function (v) { shape = v; update(); } });
    ctx.select({ label: "frame", value: kind, options: [["rmf", "rotation-minimizing"], ["frenet", "Frenet"], ["screw", "screw motion"]], oninput: function (v) { kind = v; update(); } });
    ctx.slider({ label: "radius", min: 0.3, max: 2, step: 0.05, value: r, oninput: function (v) { r = v; update(); } });
    ctx.slider({ label: "pitch", min: 0.2, max: 6, step: 0.1, value: pitch, oninput: function (v) { pitch = v; update(); } });
    ctx.slider({ label: "turns", min: 1, max: 4, step: 1, value: turns, format: function (v) { return "" + v; }, oninput: function (v) { turns = v; update(); } });
    ctx.slider({ label: "steps", min: 4, max: 96, step: 1, value: steps, format: function (v) { return "" + v; }, oninput: function (v) { steps = v; update(); } });
    update();
  }, {
    helix: helix, sCurve: sCurve, frenet: frenet, reflectStep: reflectStep, carried: carried,
    twistAgainstScrew: twistAgainstScrew, analyticTwist: analyticTwist, screwVector: screwVector,
    dot: dot, unit: unit, cross: cross, sub: sub
  });
})();
