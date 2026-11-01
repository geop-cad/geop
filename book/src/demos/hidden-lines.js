// Hidden-line removal in the three steps of ops/geop-ops-drawing/src/hidden_lines.rs:
// project every line, cut each where its projection crosses another's, decide each
// piece by a ray from its middle towards the eye. Convex solids only, in f64.
(function () {
  "use strict";

  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function add(a, b, s) { return [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s]; }

  // The eye's frame for a view: right, up, and the direction towards the eye.
  function viewFrame(yaw, pitch) {
    var cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
    return { right: [cy, -sy, 0], up: [sp * sy, sp * cy, cp], eye: [-cp * sy, -cp * cy, sp] };
  }
  function toPaper(view, p) { return [dot(p, view.right), dot(p, view.up)]; }

  // A convex solid: the planes bounding it (n . p <= d), a cylinder as well.
  // Its edges are 3-D segments.
  function box(lo, hi) {
    var planes = [
      { n: [1, 0, 0], d: hi[0] }, { n: [-1, 0, 0], d: -lo[0] },
      { n: [0, 1, 0], d: hi[1] }, { n: [0, -1, 0], d: -lo[1] },
      { n: [0, 0, 1], d: hi[2] }, { n: [0, 0, -1], d: -lo[2] }
    ];
    var c = function (i, j, k) { return [i ? hi[0] : lo[0], j ? hi[1] : lo[1], k ? hi[2] : lo[2]]; };
    var edges = [], i, j;
    for (i = 0; i < 2; i++) for (j = 0; j < 2; j++) {
      edges.push([c(0, i, j), c(1, i, j)]);
      edges.push([c(i, 0, j), c(i, 1, j)]);
      edges.push([c(i, j, 0), c(i, j, 1)]);
    }
    return { planes: planes, edges: function () { return edges; } };
  }
  // A wedge under a slanted plane: a box with one more half space, its corners found.
  function wedge(lo, hi) {
    var planes = box(lo, hi).planes.slice();
    // Through the top of the x = lo side and the bottom of the x = hi side.
    var nx = hi[2] - lo[2], nz = hi[0] - lo[0], len = Math.hypot(nx, nz);
    var n = [nx / len, 0, nz / len];
    planes.push({ n: n, d: dot(n, [lo[0], 0, hi[2]]) });
    var a = [lo[0], lo[1], lo[2]], b = [hi[0], lo[1], lo[2]], c = [lo[0], lo[1], hi[2]];
    var A = [lo[0], hi[1], lo[2]], B = [hi[0], hi[1], lo[2]], C = [lo[0], hi[1], hi[2]];
    var edges = [[a, b], [b, c], [c, a], [A, B], [B, C], [C, A], [a, A], [b, B], [c, C]];
    return { planes: planes, edges: function () { return edges; } };
  }
  // A cylinder about the z axis at (cx, cy): its two rims as polylines, and what
  // a curved face adds, its silhouette: the two generators where the normal is
  // square to the view, n(θ) . eye = 0.
  function cylinder(cx, cy, r, z0, z1) {
    var planes = [{ n: [0, 0, 1], d: z1 }, { n: [0, 0, -1], d: -z0 }];
    return {
      planes: planes, cyl: { cx: cx, cy: cy, r: r },
      edges: function (view) {
        var out = [], N = 64;
        function p(a, z) { return [cx + r * Math.cos(a), cy + r * Math.sin(a), z]; }
        for (var i = 0; i < N; i++) {
          var a = 2 * Math.PI * i / N, b = 2 * Math.PI * (i + 1) / N;
          out.push([p(a, z0), p(b, z0)]);
          out.push([p(a, z1), p(b, z1)]);
        }
        silhouetteAngles(view.eye).forEach(function (a) { out.push([p(a, z0), p(a, z1), "silhouette"]); });
        return out;
      }
    };
  }
  function silhouetteAngles(eye) {
    var a = Math.atan2(eye[1], eye[0]);
    return [a + Math.PI / 2, a - Math.PI / 2];
  }

  var TOL = 1e-9;
  // The stretch of the ray p + t e, t > 0, strictly inside the solid. The kernel
  // asks this with enclosures; these floats need a tolerance, which it does not.
  function rayInside(solid, p, e) {
    var t0 = TOL, t1 = 1e9;
    for (var i = 0; i < solid.planes.length; i++) {
      var pl = solid.planes[i], d = dot(pl.n, e), s = pl.d - dot(pl.n, p);
      if (Math.abs(d) < 1e-12) { if (s <= TOL) return null; continue; }
      var t = s / d;
      if (d > 0) t1 = Math.min(t1, t); else t0 = Math.max(t0, t);
    }
    if (solid.cyl) {
      var c = solid.cyl, ox = p[0] - c.cx, oy = p[1] - c.cy;
      var A = e[0] * e[0] + e[1] * e[1], B = ox * e[0] + oy * e[1], C = ox * ox + oy * oy - c.r * c.r;
      if (A < 1e-12) { if (C >= -TOL) return null; } else {
        var disc = B * B - A * C;
        if (disc <= 0) return null;
        var root = Math.sqrt(disc);
        t0 = Math.max(t0, (-B - root) / A);
        t1 = Math.min(t1, (-B + root) / A);
      }
    }
    return t1 > t0 + TOL ? [t0, t1] : null;
  }
  function hidden(solids, p, e) {
    return solids.some(function (s) { return rayInside(s, p, e) !== null; });
  }

  // Parameters along segment a-b on the paper where it crosses segment c-d.
  function crossing(a, b, c, d) {
    var r = [b[0] - a[0], b[1] - a[1]], s = [d[0] - c[0], d[1] - c[1]];
    var det = r[0] * s[1] - r[1] * s[0];
    if (Math.abs(det) < 1e-12) return null;
    var q = [c[0] - a[0], c[1] - a[1]];
    var t = (q[0] * s[1] - q[1] * s[0]) / det, u = (q[0] * r[1] - q[1] * r[0]) / det;
    return t > 1e-9 && t < 1 - 1e-9 && u > -1e-9 && u < 1 + 1e-9 ? t : null;
  }

  // The whole algorithm. Returns pieces { a, b (3-D), paper: [p, q], visible, silhouette } and the cuts.
  function hiddenLines(solids, view) {
    var lines = [];
    solids.forEach(function (s) { s.edges(view).forEach(function (e) { lines.push({ a: e[0], b: e[1], silhouette: e[2] === "silhouette", pa: toPaper(view, e[0]), pb: toPaper(view, e[1]) }); }); });
    var pieces = [], cuts = [];
    lines.forEach(function (line, i) {
      var ts = [0, 1];
      lines.forEach(function (other, j) {
        if (i === j) return;
        var t = crossing(line.pa, line.pb, other.pa, other.pb);
        if (t !== null) ts.push(t);
      });
      ts.sort(function (x, y) { return x - y; });
      for (var k = 0; k + 1 < ts.length; k++) {
        if (ts[k + 1] - ts[k] < 1e-9) continue;
        var m = (ts[k] + ts[k + 1]) / 2;
        var mid = [0, 1, 2].map(function (c) { return line.a[c] + (line.b[c] - line.a[c]) * m; });
        var at = function (t) { return [line.pa[0] + (line.pb[0] - line.pa[0]) * t, line.pa[1] + (line.pb[1] - line.pa[1]) * t]; };
        pieces.push({ paper: [at(ts[k]), at(ts[k + 1])], visible: !hidden(solids, mid, view.eye), silhouette: line.silhouette, mid: mid });
        if (k > 0) cuts.push(at(ts[k]));
      }
    });
    return { pieces: pieces, cuts: cuts, lines: lines };
  }

  GeopBook.demo("hidden-lines", function (ctx) {
    var D = GeopBook.draw;
    var yaw = 0.65, pitch = 0.55, showHidden = true, showCuts = false, sceneName = "block and cylinder";
    var scenes = {
      "block and cylinder": function () { return [box([-1.4, -0.9, -0.8], [0.9, 0.9, 0.4]), cylinder(1.1, 0.7, 0.55, -0.8, 1.2)]; },
      "block and wedge": function () { return [box([-1.2, -1, -0.8], [0.6, 1, 0.3]), wedge([0.1, -1.6, -0.8], [1.8, -0.2, 1.1])]; },
      "one box": function () { return [box([-1, -1, -1], [1, 1, 1])]; }
    };
    var stage = ctx.stage({ aspect: 1.3, maxHeight: 420, label: "Visible and hidden lines of solids, turned by dragging", draw: paint });
    ctx.drag(stage, function (dx, dy, x, y, phase) {
      if (phase !== "move") return;
      yaw += dx * 0.01;
      pitch = Math.max(-1.5, Math.min(1.5, pitch + dy * 0.01));
      update();
    });
    var read = ctx.readout();
    function paint(g, w, h, c) {
      var view = viewFrame(yaw, pitch), r = hiddenLines(scenes[sceneName](), view);
      var s = Math.min(w, h) * 0.2;
      function px(p) { return [w / 2 + p[0] * s, h / 2 - p[1] * s]; }
      r.pieces.forEach(function (piece) {
        var pts = [px(piece.paper[0]), px(piece.paper[1])];
        if (!piece.visible && showHidden) D.polyline(g, null, pts, c.muted, 1.3, [4, 3]);
      });
      r.pieces.forEach(function (piece) {
        var pts = [px(piece.paper[0]), px(piece.paper[1])];
        if (piece.visible) D.polyline(g, null, pts, piece.silhouette ? c.blue : c.fg, 1.8);
      });
      if (showCuts) r.cuts.forEach(function (q) { D.dot(g, null, px(q), 2.2, c.red); });
    }
    function update() {
      var r = hiddenLines(scenes[sceneName](), viewFrame(yaw, pitch));
      var vis = r.pieces.filter(function (p) { return p.visible; }).length;
      read.set(r.lines.length + " lines cut into " + r.pieces.length + " pieces: " + vis + " visible, " + (r.pieces.length - vis) + " hidden, one ray each.");
      stage.redraw();
    }
    ctx.select({ label: "scene", value: sceneName, options: Object.keys(scenes), oninput: function (v) { sceneName = v; update(); } });
    ctx.checkbox({ label: "hidden lines", value: true, oninput: function (v) { showHidden = v; stage.redraw(); } });
    ctx.checkbox({ label: "cuts", value: false, oninput: function (v) { showCuts = v; stage.redraw(); } });
    update();
  }, { viewFrame: viewFrame, toPaper: toPaper, box: box, wedge: wedge, cylinder: cylinder, silhouetteAngles: silhouetteAngles, hidden: hidden, hiddenLines: hiddenLines, rayInside: rayInside, crossing: crossing });
})();
