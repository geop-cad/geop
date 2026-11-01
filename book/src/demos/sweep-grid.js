// A sweep as a grid: every joint at every station is a vertex, every curve at
// every station an edge, every joint along every span a lateral edge, every
// curve through every span a wall. The rules are those of
// lib/geop-lib-sweep/src/skin.rs. `grid` builds the entities by those rules and
// counts them; `formula` is the closed form; a test checks they agree and
// that every edge is used by exactly two faces.
(function () {
  "use strict";

  // params: { n: joints of the outer loop, spans: m, closed: full turn or not,
  //           axisSide: first curve lies on the axis (its two joints poles),
  //           hole: a hole of 4 joints clear of the axis }
  function grid(p) {
    var stations = p.closed ? p.spans : p.spans + 1;
    var vertices = {}, edges = {}, faces = [];
    var loops = [{ n: p.n, axis: !!p.axisSide }];
    if (p.hole) loops.push({ n: 4, axis: false });
    function isPole(l, i) { return loops[l].axis && (i === 0 || i === 1); }
    function curveOnAxis(l, i) { return loops[l].axis && i === 0; }
    function V(l, i, s) { var k = isPole(l, i) ? "P" + l + "." + i : "J" + l + "." + i + "@" + s; vertices[k] = 1; return k; }
    function curveEdge(l, i, s) {
      var n = loops[l].n, a = V(l, i, s), b = V(l, (i + 1) % n, s);
      var key = curveOnAxis(l, i) ? "A" + l + "." + i : "C" + l + "." + i + "@" + s;
      edges[key] = [a, b];
      return key;
    }
    function lateral(l, i, q) {
      if (isPole(l, i)) return null;
      var key = "L" + l + "." + i + "@" + q;
      edges[key] = [V(l, i, q), V(l, i, (q + 1) % stations)];
      return key;
    }
    var l, i, s, q;
    for (l = 0; l < loops.length; l++) {
      var n = loops[l].n;
      for (i = 0; i < n; i++) {
        if (curveOnAxis(l, i)) continue; // sweeps no wall
        for (q = 0; q < p.spans; q++) {
          var next = (q + 1) % stations;
          var used = [curveEdge(l, i, q), curveEdge(l, i, next), lateral(l, i, q), lateral(l, (i + 1) % n, q)]
            .filter(function (e) { return e; });
          faces.push({ edges: used, rings: 0, kind: "wall" });
        }
      }
    }
    if (!p.closed) {
      [0, stations - 1].forEach(function (st) {
        var used = [];
        for (var a = 0; a < loops.length; a++) for (var b = 0; b < loops[a].n; b++) used.push(curveEdge(a, b, st));
        faces.push({ edges: used, rings: loops.length - 1, kind: "cap" });
      });
    }
    // Every edge used by exactly two faces, as BodySpec::check asks of a solid.
    var use = {};
    faces.forEach(function (f) { f.edges.forEach(function (e) { use[e] = (use[e] || 0) + 1; }); });
    var manifold = Object.keys(edges).every(function (e) { return use[e] === 2; });
    // Shells: faces sharing an edge are connected.
    var parent = faces.map(function (_, k) { return k; });
    function find(x) { while (parent[x] !== x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
    var first = {};
    faces.forEach(function (f, k) { f.edges.forEach(function (e) { if (first[e] === undefined) first[e] = k; else parent[find(k)] = find(first[e]); }); });
    var groups = {};
    faces.forEach(function (_, k) { var r = find(k); (groups[r] = groups[r] || []).push(k); });
    // Each shell's genus from its own Euler characteristic.
    var genus = 0, closedUp = true;
    Object.keys(groups).forEach(function (r) {
      var vs = {}, es = {}, f = groups[r].length, rings = 0;
      groups[r].forEach(function (k) {
        rings += faces[k].rings;
        faces[k].edges.forEach(function (e) { es[e] = 1; vs[edges[e][0]] = 1; vs[edges[e][1]] = 1; });
      });
      var chi = Object.keys(vs).length - Object.keys(es).length + f - rings;
      var g = (2 - chi) / 2;
      if (g < 0 || g !== Math.floor(g)) closedUp = false;
      genus += g;
    });
    var rings = faces.reduce(function (acc, f) { return acc + f.rings; }, 0);
    return {
      V: Object.keys(vertices).length, E: Object.keys(edges).length, F: faces.length, R: rings,
      S: Object.keys(groups).length, G: genus, manifold: manifold, integral: closedUp
    };
  }

  // The closed forms the lecture derives.
  function formula(p) {
    var s = p.closed ? p.spans : p.spans + 1, m = p.spans;
    var poles = p.axisSide ? 2 : 0, onAxis = p.axisSide ? 1 : 0, h = p.hole ? 4 : 0;
    var caps = p.closed ? 0 : 2, axisEdges = p.axisSide && !p.closed ? 1 : 0;
    return {
      V: (p.n - poles + h) * s + poles,
      E: (p.n - onAxis + h) * s + axisEdges + (p.n - poles + h) * m,
      F: (p.n - onAxis + h) * m + caps,
      R: p.hole && !p.closed ? 2 : 0
    };
  }

  // The outer profile in (r, z): n joints, in order, clear of the axis or with
  // the first side on it.
  function profile(n, axisSide) {
    var out = [], i;
    if (axisSide) {
      out.push([0, -1], [0, 1]);
      for (i = 0; i < n - 2; i++) {
        var a = Math.PI / 2 - Math.PI * (i + 0.5) / (n - 2);
        out.push([1.4 * Math.cos(a) + 0.15, Math.sin(a)]);
      }
    } else {
      for (i = 0; i < n; i++) {
        var b = 2 * Math.PI * i / n + Math.PI / n;
        out.push([1.8 + 0.8 * Math.cos(b), 0.8 * Math.sin(b)]);
      }
    }
    return out;
  }
  function holeLoop(axisSide) {
    var cx = axisSide ? 0.75 : 1.8, r = axisSide ? 0.22 : 0.3;
    return [[cx - r, -r], [cx - r, r], [cx + r, r], [cx + r, -r]];
  }

  GeopBook.demo("sweep-grid", function (ctx) {
    var D = GeopBook.draw;
    var mode = "extrude", n = 4, heights = 1, angle = 180, hole = false;
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 400, label: "A profile swept into a grid of vertices, edges and faces", draw: paint });
    var cam = ctx.camera(stage, { yaw: 0.7, pitch: 0.45, scale: 0.21, distance: 14 });
    var read = ctx.readout();

    function params() {
      var axisSide = mode === "revolve-axis" || mode === "turn-axis";
      var closed = mode === "turn" || mode === "turn-axis";
      var spans = mode === "extrude" ? heights : closed ? 4 : Math.ceil(angle / 90);
      return { n: Math.max(n, 3), spans: spans, closed: closed, axisSide: axisSide, hole: hole };
    }
    function place(p, along) {
      if (mode === "extrude") return [p[0] - 1.8, along * 2.4 - 1.2, p[1]];
      var a = along;
      return [p[0] * Math.cos(a), p[0] * Math.sin(a), p[1]];
    }
    function stationAlong(s, P) {
      if (mode === "extrude") return s / P.spans;
      if (P.closed) return s * Math.PI / 2;
      return (angle * Math.PI / 180) * s / P.spans;
    }
    function paint(g, w, h, c) {
      if (!cam) return;
      var P = params(), prof = profile(P.n, P.axisSide);
      var loops = [prof]; if (P.hole) loops.push(holeLoop(P.axisSide));
      var stations = P.closed ? P.spans : P.spans + 1;
      var polys = [];
      loops.forEach(function (lp, li) {
        lp.forEach(function (a, i) {
          if (li === 0 && P.axisSide && i === 0) return;
          var b = lp[(i + 1) % lp.length];
          for (var q = 0; q < P.spans; q++) {
            var s0 = stationAlong(q, P), s1 = stationAlong(q + 1, P);
            for (var k = 0; k < 6; k++) {
              var t0 = s0 + (s1 - s0) * k / 6, t1 = s0 + (s1 - s0) * (k + 1) / 6;
              var pts = [place(a, t0), place(b, t0), place(b, t1), place(a, t1)].map(cam.project);
              polys.push({ pts: pts, depth: (pts[0][2] + pts[1][2] + pts[2][2] + pts[3][2]) / 4, cap: false });
            }
          }
        });
      });
      if (!P.closed) {
        [0, P.spans].forEach(function (s) {
          var pts = prof.map(function (p) { return cam.project(place(p, stationAlong(s, P))); });
          var d = pts.reduce(function (acc, q) { return acc + q[2]; }, 0) / pts.length;
          polys.push({ pts: pts, depth: d, cap: true });
        });
      }
      polys.sort(function (x, y) { return x.depth - y.depth; });
      polys.forEach(function (f) { D.polygon(g, null, f.pts, f.cap ? c.fillB : c.fillA, null); });
      // Laterals.
      loops.forEach(function (lp, li) {
        lp.forEach(function (a, i) {
          if (li === 0 && P.axisSide && (i === 0 || i === 1)) return;
          var pts = [];
          var end = stationAlong(P.spans, P);
          for (var k = 0; k <= 40; k++) pts.push(cam.project(place(a, end * k / 40)));
          D.polyline(g, null, pts, c.muted, 1);
        });
      });
      // Station curves and vertices.
      for (var s = 0; s < stations; s++) {
        var al = stationAlong(s, P);
        loops.forEach(function (lp, li) {
          var pts = lp.map(function (p) { return cam.project(place(p, al)); });
          D.polygon(g, null, pts, null, c.fg, 1.5);
          pts.forEach(function (q, i) {
            var pole = li === 0 && P.axisSide && (i === 0 || i === 1);
            D.dot(g, null, q, pole ? 5 : 3, pole ? c.red : c.fg);
          });
        });
      }
    }
    function update() {
      var P = params(), G = grid(P), F = formula(P);
      var euler = G.V - G.E + G.F - G.R;
      read.set(
        "stations " + (P.closed ? P.spans : P.spans + 1) + ", spans " + P.spans + (P.closed ? " (closed)" : " (open)") + "\n" +
        "V = " + G.V + ", E = " + G.E + ", F = " + G.F + ", rings R = " + G.R + ", shells S = " + G.S + ", genus G = " + G.G + "\n" +
        "V - E + F - R = " + euler + ",  2(S - G) = " + 2 * (G.S - G.G) + (euler === 2 * (G.S - G.G) ? "  (Euler-Poincare holds)" : "  (does NOT hold)") + "\n" +
        "every edge used by two faces: " + (G.manifold ? "yes" : "no") +
        ";  closed forms give V " + F.V + ", E " + F.E + ", F " + F.F);
      stage.redraw();
    }
    ctx.select({
      label: "sweep", value: mode,
      options: [["extrude", "extrude"], ["revolve", "partial turn"], ["turn", "full turn"], ["revolve-axis", "partial, side on axis"], ["turn-axis", "full, side on axis"]],
      oninput: function (v) { mode = v; update(); }
    });
    ctx.checkbox({ label: "with a hole", value: false, oninput: function (v) { hole = v; update(); } });
    ctx.slider({ label: "joints", min: 3, max: 8, step: 1, value: 4, format: function (v) { return "" + v; }, oninput: function (v) { n = v; update(); } });
    ctx.slider({ label: "heights", min: 1, max: 4, step: 1, value: 1, format: function (v) { return "" + v; }, oninput: function (v) { heights = v; update(); } });
    ctx.slider({ label: "angle", min: 10, max: 350, step: 5, value: 180, format: function (v) { return v + "°"; }, oninput: function (v) { angle = v; update(); } });
    update();
  }, { grid: grid, formula: formula });
})();
