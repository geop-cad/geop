// A 2-D analogy of the one rebuild (lib/geop-lib-shell/src/rebuild.rs): a polygon
// whose sides lie on lines, as a solid's faces lie on surfaces. Change the lines; each
// corner moves to where its two lines meet now; each side is checked to keep running the
// way it ran. A side that turns around is refused, by name, as `turns_around` does.
(function () {
  "use strict";
  // A polygon is a list of points, counter-clockwise. Its "surfaces" are lines
  // { p: a point, n: unit outward normal }, one per side i (from pts[i] to pts[i+1]).
  function linesOf(pts) {
    return pts.map(function (a, i) {
      var b = pts[(i + 1) % pts.length], dx = b[0] - a[0], dy = b[1] - a[1], l = Math.hypot(dx, dy);
      return { p: a, n: [dy / l, -dx / l] };
    });
  }
  function moved(line, by) { return { p: [line.p[0] + line.n[0] * by, line.p[1] + line.n[1] * by], n: line.n }; }
  // Turned about a point of it, by an angle.
  function turned(line, about, angle) {
    var c = Math.cos(angle), s = Math.sin(angle);
    return { p: about, n: [c * line.n[0] - s * line.n[1], s * line.n[0] + c * line.n[1]] };
  }
  // Where two lines meet; null where they are parallel.
  function meet(a, b) {
    var det = a.n[0] * b.n[1] - a.n[1] * b.n[0];
    if (Math.abs(det) < 1e-12) return null;
    var ca = a.n[0] * a.p[0] + a.n[1] * a.p[1], cb = b.n[0] * b.p[0] + b.n[1] * b.p[1];
    return [(ca * b.n[1] - cb * a.n[1]) / det, (a.n[0] * cb - b.n[0] * ca) / det];
  }
  // The rebuild: corners where the lines around them meet, each checked against the
  // lines, each side checked to run the way it ran.
  // Returns { pts } or { refused: message, pts }.
  function rebuild(oldPts, lines) {
    var n = oldPts.length, out = [], i;
    for (i = 0; i < n; i++) {
      var prev = lines[(i + n - 1) % n], here = lines[i], p = meet(prev, here);
      if (!p) return { refused: "corner v" + i + ": the lines of sides e" + ((i + n - 1) % n) + " and e" + i + " no longer meet", pts: null };
      out.push(p);
    }
    for (i = 0; i < n; i++) {
      var a0 = oldPts[i], b0 = oldPts[(i + 1) % n], a1 = out[i], b1 = out[(i + 1) % n];
      var along = (b1[0] - a1[0]) * (b0[0] - a0[0]) + (b1[1] - a1[1]) * (b0[1] - a0[1]);
      if (!(along > 0)) {
        return { refused: "side e" + i + ": moved this far, it turns around between v" + i + " and v" + ((i + 1) % n) + ": a side it joins would vanish, or two would cross", pts: out };
      }
    }
    return { pts: out };
  }
  // Every side of the polygon moved in by t: the inner copy a shell makes.
  function inner(pts, t) {
    return rebuild(pts, linesOf(pts).map(function (l) { return moved(l, -t); }));
  }

  var SHAPES = {
    plate: [[0.8, 0.8], [5.2, 0.8], [5.2, 3.2], [0.8, 3.2]],
    bracket: [[0.6, 0.6], [5.4, 0.6], [5.4, 1.6], [2.2, 1.6], [2.2, 3.6], [0.6, 3.6]],
    chamfered: [[0.7, 0.7], [4.7, 0.7], [5.3, 1.3], [5.3, 3.3], [0.7, 3.3]],
    notch: [[0.6, 0.7], [2.3, 0.7], [2.7, 1.5], [3.3, 1.5], [3.7, 0.7], [5.4, 0.7], [5.4, 3.3], [0.6, 3.3]]
  };

  GeopBook.demo("face-move", function (ctx) {
    var D = GeopBook.draw;
    var shape = "bracket", op = "move", side = 3, by = 0.4, angle = 8;
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 380, label: "A polygon rebuilt around a moved side", draw: paint });
    var view = ctx.view(stage, { x0: 0, x1: 6, y0: 0, y1: 4.2 });
    var read = ctx.readout();
    function current() {
      var pts = SHAPES[shape], lines = linesOf(pts), n = pts.length, i = side % n;
      var ls = lines.slice(), res;
      if (op === "move") { ls[i] = moved(lines[i], by); res = rebuild(pts, ls); }
      else if (op === "draft") {
        var a = pts[i], b = pts[(i + 1) % n], mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
        ls[i] = turned(lines[i], mid, angle * Math.PI / 180); res = rebuild(pts, ls);
      } else { res = inner(pts, Math.max(by, 0)); ls = lines.map(function (l) { return moved(l, -Math.max(by, 0)); }); }
      return { pts: pts, lines: ls, res: res, i: i };
    }
    function paint(g, w, h, c) {
      if (!view) return;
      view.update();
      var cur = current(), pts = cur.pts, n = pts.length;
      // The lines: the surfaces, carried on past the sides.
      cur.lines.forEach(function (l, k) {
        var t = [-l.n[1], l.n[0]];
        var changed = op === "shell" || k === cur.i;
        if (!changed) return;
        D.polyline(g, view, [[l.p[0] - 8 * t[0], l.p[1] - 8 * t[1]], [l.p[0] + 8 * t[0], l.p[1] + 8 * t[1]]], c.orange, 1, [5, 5]);
      });
      D.polygon(g, view, pts, null, c.muted, 1.5);
      if (cur.res.pts && !cur.res.refused) {
        if (op === "shell") {
          D.polygon(g, view, pts, c.fillB, null);
          D.polygon(g, view, cur.res.pts, c.bg, c.blue, 2);
        } else {
          D.polygon(g, view, cur.res.pts, c.fillA, c.blue, 2);
        }
      } else if (cur.res.pts) {
        D.polygon(g, view, cur.res.pts, null, c.red, 2);
      }
      var shown = cur.res.pts || pts;
      shown.forEach(function (p, k) {
        D.dot(g, view, p, 3.5, cur.res.refused ? c.red : c.blue);
        D.text(g, view, [p[0] + 0.07, p[1] + 0.07], "v" + k, c.muted);
      });
      pts.forEach(function (a, k) {
        var b = pts[(k + 1) % n];
        D.text(g, view, [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + 0.06], "e" + k, op !== "shell" && k === cur.i ? c.orange : c.muted, "center");
      });
    }
    function update() {
      var cur = current();
      read.set(cur.res.refused ? "Refused: " + cur.res.refused : "Rebuilt: same corners, same sides, every corner where its lines meet.");
      stage.redraw();
    }
    ctx.select({ label: "shape", value: shape, options: Object.keys(SHAPES), oninput: function (v) { shape = v; sideSlider.value = sideSlider.value; update(); } });
    ctx.select({ label: "operation", value: op, options: [["move", "move a side"], ["draft", "tilt a side"], ["shell", "inner copy of all"]], oninput: function (v) { op = v; update(); } });
    var sideSlider = ctx.slider({ label: "side", min: 0, max: 7, step: 1, value: side, format: function (v) { return "e" + (v % SHAPES[shape].length); }, oninput: function (v) { side = v; update(); } });
    ctx.slider({ label: "distance", min: -2, max: 2.5, step: 0.01, value: by, oninput: function (v) { by = v; update(); } });
    ctx.slider({ label: "tilt", min: -60, max: 60, step: 1, value: angle, format: function (v) { return v + "°"; }, oninput: function (v) { angle = v; update(); } });
    update();
  }, { linesOf: linesOf, moved: moved, turned: turned, meet: meet, rebuild: rebuild, inner: inner, SHAPES: SHAPES });
})();
