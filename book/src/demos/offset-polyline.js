// Offsetting a chain of straight lines: corners that turn away get an arc,
// corners that turn towards are cut, and where the offset crosses itself, what
// lies nearer the chain than the distance is dropped. Arcs are drawn as short
// chords here; the kernel keeps them arcs and ties them by constraints.
(function () {
  "use strict";

  function sub(a, b) { return [a[0] - b[0], a[1] - b[1]]; }
  function unit(a) { var n = Math.hypot(a[0], a[1]) || 1; return [a[0] / n, a[1] / n]; }
  function cross(a, b) { return a[0] * b[1] - a[1] * b[0]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1]; }

  // Distance from p to the segment ab.
  function segDist(p, a, b) {
    var ab = sub(b, a), t = dot(sub(p, a), ab) / (dot(ab, ab) || 1);
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(p[0] - a[0] - t * ab[0], p[1] - a[1] - t * ab[1]);
  }
  function chainDist(p, chain) {
    var m = Infinity;
    for (var i = 0; i + 1 < chain.length; i++) m = Math.min(m, segDist(p, chain[i], chain[i + 1]));
    return m;
  }

  // The raw offset to the left by d: every piece's offset, joined at each corner
  // by an arc about the corner where it turns away (left turns are towards the
  // offset side), and by the two offsets' ends joined crudely where it turns
  // towards; the crossing that makes is cut by trim().
  function raw(chain, d, arcSteps) {
    var out = [];
    for (var i = 0; i + 1 < chain.length; i++) {
      var a = chain[i], b = chain[i + 1], t = unit(sub(b, a)), n = [-t[1], t[0]];
      var s = [a[0] + d * n[0], a[1] + d * n[1]], e = [b[0] + d * n[0], b[1] + d * n[1]];
      out.push(s, e);
      if (i + 2 < chain.length) {
        var t2 = unit(sub(chain[i + 2], b));
        if (cross(t, t2) < 0) { // turns away from the left: an arc round the corner
          var a0 = Math.atan2(n[1], n[0]), n2 = [-t2[1], t2[0]], a1 = Math.atan2(n2[1], n2[0]);
          while (a1 > a0) a1 -= 2 * Math.PI;
          var steps = arcSteps || 16;
          for (var k = 1; k < steps; k++) {
            var th = a0 + (a1 - a0) * k / steps;
            out.push([b[0] + d * Math.cos(th), b[1] + d * Math.sin(th)]);
          }
        }
      }
    }
    return out;
  }

  // Where segments pq and rs cross, at parameters strictly inside both.
  function crossing(p, q, r, s) {
    var u = sub(q, p), v = sub(s, r), den = cross(u, v);
    if (den === 0) return null;
    var w = sub(r, p), a = cross(w, v) / den, b = cross(w, u) / den;
    if (a <= 1e-12 || a >= 1 - 1e-12 || b <= 1e-12 || b >= 1 - 1e-12) return null;
    return { a: a, b: b, at: [p[0] + a * u[0], p[1] + a * u[1]] };
  }

  // Cut the raw offset at every crossing with itself, and keep the runs all of
  // which is at least d from the chain. Each run is one polyline.
  function trim(polyline, chain, d) {
    var n = polyline.length - 1, cuts = [];
    for (var i = 0; i < n; i++) cuts.push([]);
    for (i = 0; i < n; i++) {
      for (var j = i + 2; j < n; j++) {
        var c = crossing(polyline[i], polyline[i + 1], polyline[j], polyline[j + 1]);
        if (c) { cuts[i].push({ t: c.a, at: c.at }); cuts[j].push({ t: c.b, at: c.at }); }
      }
    }
    // The arcs here are chords, whose middles sag a little inside the distance:
    // 16 chords per turn away of at most half a turn sag by 1 - cos(pi/32) < 0.5%.
    var runs = [], current = [polyline[0]], tol = 0.005 * d;
    function close() { if (current.length > 1) runs.push(current); }
    for (i = 0; i < n; i++) {
      cuts[i].sort(function (x, y) { return x.t - y.t; });
      cuts[i].forEach(function (c) { current.push(c.at); close(); current = [c.at]; });
      current.push(polyline[i + 1]);
    }
    close();
    return runs.filter(function (run) {
      for (var k = 0; k + 1 < run.length; k++) {
        var mid = [(run[k][0] + run[k + 1][0]) / 2, (run[k][1] + run[k + 1][1]) / 2];
        if (chainDist(mid, chain) < d - tol) return false;
      }
      return true;
    });
  }
  function offset(chain, d, arcSteps) {
    return trim(raw(chain, d, arcSteps), chain, d);
  }

  GeopBook.demo("offset-polyline", function (ctx) {
    var D = GeopBook.draw;
    var chain = [[0.8, 3.2], [3.2, 3.2], [3.2, 1.2], [3.8, 1.2], [3.8, 3.2], [6.2, 3.2]];
    var d = 0.45, side = 1, showRaw = true;
    var stage = ctx.stage({ aspect: 1.6, maxHeight: 360, label: "A chain of lines and its offset, trimmed", draw: paint });
    var view = ctx.view(stage, { x0: 0, x1: 7, y0: 0, y1: 4.4 });
    var read = ctx.readout();
    function current() { return side > 0 ? chain : chain.slice().reverse(); }
    function paint(g, w, h, c) {
      if (!view) return;
      view.update();
      var src = current();
      if (showRaw) D.polyline(g, view, raw(src, d), c.faint, 1.4, [4, 3]);
      D.polyline(g, view, chain, c.fg, 2.4);
      var runs = offset(src, d);
      runs.forEach(function (run) { D.polyline(g, view, run, c.blue, 2.4); });
      chain.forEach(function (p, i) { D.dot(g, view, p, 6, handles && handles.selected === i ? c.accent : c.bg, c.fg); });
      read.set(runs.length + " run" + (runs.length === 1 ? "" : "s") + " kept at distance " + d.toFixed(2) + " from the chain.");
    }
    var handles = ctx.handles({ stage: stage, view: view, pts: chain, onchange: function () { stage.redraw(); } });
    ctx.slider({ label: "distance", min: 0.05, max: 1.2, step: 0.01, value: d, oninput: function (v) { d = v; stage.redraw(); } });
    ctx.button("Other side", function () { side = -side; stage.redraw(); });
    ctx.checkbox({ label: "show the uncut offset", value: true, oninput: function (v) { showRaw = v; stage.redraw(); } });
    stage.redraw();
  }, { raw: raw, trim: trim, offset: offset, chainDist: chainDist, crossing: crossing });
})();
