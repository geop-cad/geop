// The bilinearly blended Coons patch of four cubic Bezier sides:
// S(u,v) = Lc(u,v) + Ld(u,v) - B(u,v). Drag a side's control points in 3-D
// (they move in the plane facing you); drag elsewhere to turn the view.
(function () {
  "use strict";
  function bez(p, t) {
    var s = 1 - t, a = s * s * s, b = 3 * s * s * t, c = 3 * s * t * t, d = t * t * t;
    return [a * p[0][0] + b * p[1][0] + c * p[2][0] + d * p[3][0],
            a * p[0][1] + b * p[1][1] + c * p[2][1] + d * p[3][1],
            a * p[0][2] + b * p[1][2] + c * p[2][2] + d * p[3][2]];
  }
  function lin(a, b, t) { return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]; }
  // c0: v = 0, c1: v = 1 (both in u); d0: u = 0, d1: u = 1 (both in v).
  function ruledU(s, u, v) { return lin(bez(s.c0, u), bez(s.c1, u), v); }
  function ruledV(s, u, v) { return lin(bez(s.d0, v), bez(s.d1, v), u); }
  function bilinear(s, u, v) {
    return lin(lin(s.c0[0], s.c0[3], u), lin(s.c1[0], s.c1[3], u), v);
  }
  function coons(s, u, v) {
    var a = ruledU(s, u, v), b = ruledV(s, u, v), c = bilinear(s, u, v);
    return [a[0] + b[0] - c[0], a[1] + b[1] - c[1], a[2] + b[2] - c[2]];
  }
  function initial() {
    var c00 = [-1.4, -1.4, 0], c10 = [1.4, -1.4, 0.5], c01 = [-1.4, 1.4, 0.6], c11 = [1.4, 1.4, -0.2];
    return {
      c0: [c00, [-0.5, -1.6, 1.1], [0.5, -1.2, -0.6], c10],
      c1: [c01, [-0.5, 1.2, 1.2], [0.5, 1.6, -0.9], c11],
      d0: [c00, [-1.6, -0.5, -0.8], [-1.2, 0.5, 1.3], c01],
      d1: [c10, [1.2, -0.5, 1.2], [1.6, 0.5, -0.8], c11]
    };
  }

  GeopBook.demo("coons-patch", function (ctx) {
    var D = GeopBook.draw;
    var s = initial(), mode = "coons", net = 14;
    var stage = ctx.stage({ aspect: 1.35, maxHeight: 420, label: "A Coons patch spanning four boundary curves", draw: paint });
    var cam = ctx.camera(stage, { yaw: 0.6, pitch: 0.75, scale: 0.21, distance: 12, orbit: false });
    var read = ctx.readout();

    // The handles: corners (shared by two sides) and the inner control points.
    function handles() {
      var out = [];
      out.push({ p: s.c0[0], c: true }, { p: s.c0[3], c: true }, { p: s.c1[0], c: true }, { p: s.c1[3], c: true });
      ["c0", "c1", "d0", "d1"].forEach(function (k) { out.push({ p: s[k][1] }, { p: s[k][2] }); });
      return out;
    }
    function surfacePoint(u, v) {
      if (mode === "lc") return ruledU(s, u, v);
      if (mode === "ld") return ruledV(s, u, v);
      if (mode === "b") return bilinear(s, u, v);
      return coons(s, u, v);
    }
    function paint(g, w, h, c) {
      if (!cam) return;
      var quads = [], i, j;
      for (i = 0; i < net; i++) for (j = 0; j < net; j++) {
        var q = [[i / net, j / net], [(i + 1) / net, j / net], [(i + 1) / net, (j + 1) / net], [i / net, (j + 1) / net]]
          .map(function (uv) { return cam.project(surfacePoint(uv[0], uv[1])); });
        quads.push({ q: q, d: q[0][2] + q[1][2] + q[2][2] + q[3][2] });
      }
      quads.sort(function (a, b) { return a.d - b.d; });
      quads.forEach(function (f) { D.polygon(g, null, f.q, c.fillA, c.faint, 0.6); });
      ["c0", "c1", "d0", "d1"].forEach(function (k, idx) {
        D.polyline(g, null, s[k].map(cam.project), c.faint, 1, [3, 3]);
        var pts = [];
        for (var t = 0; t <= 40; t++) pts.push(cam.project(bez(s[k], t / 40)));
        D.polyline(g, null, pts, c.series[idx], 2.4);
      });
      handles().forEach(function (hd) {
        D.dot(g, null, cam.project(hd.p), hd.c ? 6 : 4.5, hd === active ? c.accent : c.bg, c.fg);
      });
    }
    function update() {
      // Ask the surface for its misses along each side, the formula's whole point.
      var miss = 0;
      for (var t = 0; t <= 20; t++) {
        var u = t / 20;
        [[surfacePoint(u, 0), bez(s.c0, u)], [surfacePoint(u, 1), bez(s.c1, u)], [surfacePoint(0, u), bez(s.d0, u)], [surfacePoint(1, u), bez(s.d1, u)]]
          .forEach(function (pair) { miss = Math.max(miss, Math.hypot(pair[0][0] - pair[1][0], pair[0][1] - pair[1][1], pair[0][2] - pair[1][2])); });
      }
      read.set("furthest this surface lies from a side, over 21 points per side: " + miss.toFixed(6));
      stage.redraw();
    }
    // Picking in 3-D, in the plane facing the viewer; turning otherwise.
    var active = null, orbiting = null;
    function hit(sx, sy) {
      var best = null, bd = 22 * 22;
      handles().forEach(function (hd) {
        var q = cam.project(hd.p), d = (q[0] - sx) * (q[0] - sx) + (q[1] - sy) * (q[1] - sy);
        if (d < bd) { bd = d; best = hd; }
      });
      return best;
    }
    var canvas = stage.canvas;
    canvas.style.touchAction = "pan-y";
    function pos(ev) { var r = canvas.getBoundingClientRect(); return [ev.clientX - r.left, ev.clientY - r.top]; }
    canvas.addEventListener("pointerdown", function (ev) {
      var p = pos(ev); active = hit(p[0], p[1]);
      orbiting = active ? null : p;
      if (canvas.setPointerCapture) canvas.setPointerCapture(ev.pointerId);
      last = p; stage.redraw();
    });
    var last = null;
    canvas.addEventListener("pointermove", function (ev) {
      if (!last) return;
      var p = pos(ev), dx = p[0] - last[0], dy = p[1] - last[1]; last = p;
      if (active) {
        var cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw), cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
        var scale = Math.min(stage.w, stage.h) * cam.scale;
        var right = [cy, -sy, 0], up = [sp * sy, sp * cy, cp];
        for (var a = 0; a < 3; a++) active.p[a] += (right[a] * dx - up[a] * dy) / scale;
        update();
      } else if (orbiting) {
        cam.yaw += dx * 0.01; cam.pitch = Math.max(-1.5, Math.min(1.5, cam.pitch + dy * 0.01));
        stage.redraw();
      }
    });
    function up() { last = null; orbiting = null; }
    canvas.addEventListener("pointerup", up);
    canvas.addEventListener("pointercancel", up);
    canvas.addEventListener("touchstart", function (ev) {
      if (ev.touches.length !== 1) return;
      var r = canvas.getBoundingClientRect(), t = ev.touches[0];
      if (hit(t.clientX - r.left, t.clientY - r.top)) ev.preventDefault();
    }, { passive: false });
    ctx.select({ label: "show", value: mode, options: [["coons", "Coons: Lc + Ld - B"], ["lc", "Lc: ruled between c0, c1"], ["ld", "Ld: ruled between d0, d1"], ["b", "B: bilinear of the corners"]], oninput: function (v) { mode = v; update(); } });
    ctx.button("Reset", function () { s = initial(); active = null; update(); });
    update();
  }, { bez: bez, coons: coons, ruledU: ruledU, ruledV: ruledV, bilinear: bilinear, initial: initial });
})();
