// Point in a trimmed face by ray parity, on loops of cubic pieces: the search
// keeps every piece of a curve whose control points' hull could meet the ray,
// as the kernel's does, and hits are the clusters left. A sharp ray meets a
// curve in crossings; a strip, a ray cast from a box, meets a bump it grazes in
// one cluster, and that flips the parity.
(function () {
  "use strict";

  // Cubic segment as 4 control points. The circle's cubic of kappa is within
  // 2.7e-4 of the true circle, which is not the kernel's exact arc, only a demo's.
  var KAPPA = 0.5522847498;
  function circle(cx, cy, r, ccw) {
    var k = KAPPA * r;
    var q = [
      [[cx + r, cy], [cx + r, cy + k], [cx + k, cy + r], [cx, cy + r]],
      [[cx, cy + r], [cx - k, cy + r], [cx - r, cy + k], [cx - r, cy]],
      [[cx - r, cy], [cx - r, cy - k], [cx - k, cy - r], [cx, cy - r]],
      [[cx, cy - r], [cx + k, cy - r], [cx + r, cy - k], [cx + r, cy]]
    ];
    if (ccw) return q;
    return q.reverse().map(function (s) { return s.slice().reverse(); });
  }
  function line(a, b) {
    return [a, [a[0] + (b[0] - a[0]) / 3, a[1] + (b[1] - a[1]) / 3], [a[0] + 2 * (b[0] - a[0]) / 3, a[1] + 2 * (b[1] - a[1]) / 3], b];
  }
  function trim(hole) {
    var outer = [line([0, 0], [4, 0]), line([4, 0], [4, 3]), line([4, 3], [0, 3]), line([0, 3], [0, 0])];
    return outer.concat(circle(hole[0], hole[1], 0.5, false));
  }
  function bez(s, t) {
    var u = 1 - t;
    return [
      u * u * u * s[0][0] + 3 * u * u * t * s[1][0] + 3 * u * t * t * s[2][0] + t * t * t * s[3][0],
      u * u * u * s[0][1] + 3 * u * u * t * s[1][1] + 3 * u * t * t * s[2][1] + t * t * t * s[3][1]
    ];
  }
  function splitBez(s, t) {
    function L(a, b) { return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]; }
    var a = L(s[0], s[1]), b = L(s[1], s[2]), c = L(s[2], s[3]), d = L(a, b), e = L(b, c), f = L(d, e);
    return [[s[0], a, d, f], [f, e, c, s[3]]];
  }
  // Every parameter of `s` that could be on the ray from `o` along unit `dir`,
  // past its start, widened to a strip of half-width w. The convex hull of the
  // control points is the proof that a discarded piece misses it.
  function hits(s, o, dir, w) {
    var found = [];
    (function go(seg, t0, t1, depth) {
      var min = Infinity, max = -Infinity, far = -Infinity;
      for (var i = 0; i < 4; i++) {
        var dx = seg[i][0] - o[0], dy = seg[i][1] - o[1];
        var d = dx * -dir[1] + dy * dir[0], along = dx * dir[0] + dy * dir[1];
        if (d < min) min = d;
        if (d > max) max = d;
        if (along > far) far = along;
      }
      if (min > w || max < -w || far < 0) return;
      if (depth >= 22) { found.push([t0, t1]); return; }
      var m = (t0 + t1) / 2, p = splitBez(seg, 0.5);
      go(p[0], t0, m, depth + 1);
      go(p[1], m, t1, depth + 1);
    })(s, 0, 1, 0);
    // Touching leaves are one cluster: the search's one hit.
    var clusters = [];
    found.forEach(function (b) {
      var last = clusters[clusters.length - 1];
      if (last && b[0] <= last[1]) last[1] = b[1]; else clusters.push(b.slice());
    });
    return clusters;
  }
  // Could the curve pass within eps of q? Subdivide while its box could hold q.
  function couldContain(seg, q, eps, depth) {
    depth = depth || 0;
    var x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    seg.forEach(function (p) { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); });
    if (q[0] < x0 - eps || q[0] > x1 + eps || q[1] < y0 - eps || q[1] > y1 + eps) return false;
    if (depth >= 16) return true;
    var p = splitBez(seg, 0.5);
    return couldContain(p[0], q, eps, depth + 1) || couldContain(p[1], q, eps, depth + 1);
  }
  // The classification: on a vertex, on a coedge, or by parity once a ray is clear.
  function classify(pieces, q, dir, w, eps) {
    var i;
    for (i = 0; i < pieces.length; i++) {
      if (Math.hypot(pieces[i][0][0] - q[0], pieces[i][0][1] - q[1]) <= eps) return { cls: "OnVertex", piece: i };
    }
    for (i = 0; i < pieces.length; i++) {
      if (couldContain(pieces[i], q, eps)) return { cls: "OnCoedge", piece: i };
    }
    // A ray that could pass through the start of a pcurve is rejected.
    for (i = 0; i < pieces.length; i++) {
      var dx = pieces[i][0][0] - q[0], dy = pieces[i][0][1] - q[1];
      if (dx * dir[0] + dy * dir[1] > 0 && Math.abs(dx * -dir[1] + dy * dir[0]) <= Math.max(w, eps)) {
        return { cls: "Rejected", piece: i };
      }
    }
    var count = 0, at = [];
    pieces.forEach(function (s) {
      hits(s, q, dir, w).forEach(function (c) { count++; at.push(bez(s, (c[0] + c[1]) / 2)); });
    });
    return { cls: count % 2 ? "Inside" : "Outside", count: count, at: at };
  }

  GeopBook.demo("ray-parity", function (ctx) {
    var D = GeopBook.draw;
    var q = [[2.35, 1.5]], hole = [[1.5, 1.5]], angle = 0.3, w = 0, EPS = 1e-3;
    var stage = ctx.stage({ aspect: 1.45, maxHeight: 400, label: "A trimmed face, a point in it and a ray from the point", draw: paint });
    var view = ctx.view(stage, { x0: -0.2, x1: 4.2, y0: -0.2, y1: 3.2 });
    var pts = [q[0], hole[0]];
    var handles = ctx.handles({ stage: stage, view: view, pts: pts, onchange: update });
    var read = ctx.readout();
    function dir() { return [Math.cos(angle), Math.sin(angle)]; }
    function paint(g, W, H, c) {
      if (!view) return;
      view.update();
      var pieces = trim(hole[0]), d = dir(), P = q[0];
      // The face.
      g.beginPath();
      [pieces.slice(0, 4), pieces.slice(4)].forEach(function (lp) {
        lp.forEach(function (s, k) {
          for (var i = 0; i <= 20; i++) {
            var p = bez(s, i / 20), x = view.x(p[0]), y = view.y(p[1]);
            if (i === 0 && k === 0) g.moveTo(x, y); else g.lineTo(x, y);
          }
        });
        g.closePath();
      });
      g.fillStyle = c.fillA; g.fill("evenodd");
      pieces.forEach(function (s) {
        var line = [];
        for (var i = 0; i <= 30; i++) line.push(bez(s, i / 30));
        D.polyline(g, view, line, c.fg, 1.5);
        D.dot(g, view, s[0], 2.5, c.fg);
      });
      var far = [P[0] + d[0] * 8, P[1] + d[1] * 8], n = [-d[1] * w, d[0] * w];
      if (w > 0) {
        D.polygon(g, view, [[P[0] + n[0], P[1] + n[1]], [far[0] + n[0], far[1] + n[1]], [far[0] - n[0], far[1] - n[1]], [P[0] - n[0], P[1] - n[1]]], c.fillB, null);
        D.polygon(g, view, [[P[0] - w, P[1] - w], [P[0] + w, P[1] - w], [P[0] + w, P[1] + w], [P[0] - w, P[1] + w]], null, c.orange, 1);
      }
      D.polyline(g, view, [P, far], c.red, 1.4);
      var sharp = classify(pieces, P, d, 0, EPS);
      (sharp.at || []).forEach(function (p) { D.dot(g, view, p, 4.5, c.red); });
      if (w > 0) {
        var strip = classify(pieces, P, d, w, EPS);
        (strip.at || []).forEach(function (p) { D.dot(g, view, p, 7, null, c.orange); });
      }
      D.dot(g, view, P, 6, c.bg, c.fg);
      D.dot(g, view, hole[0], 3, c.muted);
      D.text(g, view, [hole[0][0] + 0.07, hole[0][1] + 0.07], "drag the hole", c.muted);
    }
    function describe(r) {
      if (r.cls === "Rejected") return "rejected: the ray could pass through the start of a pcurve; a fresh direction is drawn";
      if (r.cls === "Inside" || r.cls === "Outside") return r.count + " hit(s), so " + r.cls;
      return r.cls;
    }
    function update() {
      var pieces = trim(hole[0]), d = dir();
      var sharp = classify(pieces, q[0], d, 0, EPS);
      var text = "sharp ray: " + describe(sharp);
      if (w > 0) {
        var strip = classify(pieces, q[0], d, w, EPS);
        text += "\nstrip of half-width " + w.toFixed(3) + ": " + describe(strip);
        if ((strip.cls === "Inside" || strip.cls === "Outside") && (sharp.cls === "Inside" || sharp.cls === "Outside") && strip.cls !== sharp.cls) {
          text += "   WRONG: a graze counted as a crossing";
        }
      }
      read.set(text);
      stage.redraw();
    }
    var slider = ctx.slider({ label: "ray direction", min: -3.14, max: 3.14, step: 0.002, value: angle, oninput: function (v) { angle = v; update(); } });
    var width = ctx.slider({ label: "query box half-width", min: 0, max: 0.3, step: 0.002, value: 0, format: function (v) { return v.toFixed(3); }, oninput: function (v) { w = v; update(); } });
    ctx.button("Aim at a vertex", function () {
      var ps = trim(hole[0]), best = null;
      ps.forEach(function (s) {
        var l = Math.hypot(s[0][0] - q[0][0], s[0][1] - q[0][1]);
        if (l > 0 && (!best || l < best[0])) best = [l, s[0]];
      });
      angle = Math.atan2(best[1][1] - q[0][1], best[1][0] - q[0][0]);
      slider.value = angle;
      update();
    });
    ctx.button("Graze the hole with a strip", function () {
      // The story of AGENTS.md: 0.44 from a hole's axis, rim 0.33, here scaled:
      // the ray passes a whisker outside the hole's rim, and a strip overlaps it.
      q[0][0] = hole[0][0] + 0.8; q[0][1] = hole[0][1];
      angle = Math.PI - Math.asin(0.53 / 0.8);
      slider.value = angle;
      width.value = 0.06;
      w = 0.06;
      update();
    });
    update();
    void handles;
  }, { circle: circle, line: line, trim: trim, bez: bez, hits: hits, classify: classify });
})();
