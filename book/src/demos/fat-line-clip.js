// Fat-line clipping of one Bézier curve against another (Sederberg and Nishita),
// step by step, and the chord band the kernel uses in place of the exact hull.
// Pure functions in `algo`.
(function () {
  "use strict";

  function lerp(a, b, t) { return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]; }
  function evaluate(pts, t) {
    var row = pts;
    while (row.length > 1) {
      var n = [];
      for (var i = 0; i + 1 < row.length; i++) n.push(lerp(row[i], row[i + 1], t));
      row = n;
    }
    return row[0];
  }
  function splitAt(p, t) {
    var left = [p[0]], right = [p[p.length - 1]], row = p;
    while (row.length > 1) {
      var n = [];
      for (var i = 0; i + 1 < row.length; i++) n.push(lerp(row[i], row[i + 1], t));
      left.push(n[0]); right.unshift(n[n.length - 1]);
      row = n;
    }
    return [left, right];
  }
  // The control points of the piece of a Bézier curve over [a, b] of its [0, 1].
  function restrict(p, a, b) {
    var r = a > 0 ? splitAt(p, a)[1] : p;
    return b < 1 ? splitAt(r, (b - a) / (1 - a))[0] : r;
  }

  // The fat line of a curve: the line through its end points as (n, c), unit n, so
  // that n.x + c is the signed distance of x from it; and the band [dmin, dmax] of
  // its control points' distances, which holds the whole curve by the convex hull.
  function fatLine(Q) {
    var A = Q[0], B = Q[Q.length - 1], dx = B[0] - A[0], dy = B[1] - A[1], l = Math.hypot(dx, dy);
    if (l === 0) return null;
    var n = [-dy / l, dx / l], c = -(n[0] * A[0] + n[1] * A[1]), lo = 0, hi = 0;
    Q.forEach(function (q) {
      var d = n[0] * q[0] + n[1] * q[1] + c;
      lo = Math.min(lo, d); hi = Math.max(hi, d);
    });
    // Plain floats round: without this margin, a band as thin as the last bit
    // rejects the very crossing it holds. Interval arithmetic rounds outward instead.
    var scale = 0;
    Q.forEach(function (q) { scale = Math.max(scale, Math.abs(q[0]), Math.abs(q[1])); });
    var margin = 8 * Number.EPSILON * scale;
    return { n: n, c: c, dmin: lo - margin, dmax: hi + margin };
  }
  // The distance from the line along a curve is the Bézier function whose
  // coefficients are its control points' distances; its graph has control points
  // (i / n, d_i), since the abscissae i / n reproduce t.
  function distances(P, line) {
    return P.map(function (p) { return line.n[0] * p[0] + line.n[1] * p[1] + line.c; });
  }
  function bernsteinValue(d, t) { return evaluate(d.map(function (v) { return [v, 0]; }), t)[0]; }

  // The exact clip: the parameters where the convex hull of (i / n, d_i) meets the
  // band. A convex polygon's extreme in t within a band is at a vertex in the band
  // or a crossing of a side with a band line; every pair of points is tried, as
  // contains/curve.md's Clip A does, so no hull has to be built.
  function hullClip(d, lo, hi) {
    var n = d.length - 1, tlo = Infinity, thi = -Infinity;
    function take(t) { tlo = Math.min(tlo, t); thi = Math.max(thi, t); }
    for (var i = 0; i <= n; i++) {
      var xi = i / n;
      if (d[i] >= lo && d[i] <= hi) take(xi);
      for (var j = i + 1; j <= n; j++) {
        [lo, hi].forEach(function (y) {
          if ((d[i] - y) * (d[j] - y) < 0) take(xi + (j / n - xi) * (y - d[i]) / (d[j] - d[i]));
        });
      }
    }
    return tlo <= thi ? [tlo, thi] : null;
  }

  // The kernel's clip, `fat_line_zeros`, in plain numbers: the coefficients are the
  // intervals d_i - [lo, hi]; the chord runs between the first and the last;
  // the band is the union of all offsets from it; a zero needs
  // t in xi_0 - (d_0 + band) / slope. Coarser than the hull, and needs no pairs.
  function chordClip(d, lo, hi) {
    var n = d.length - 1;
    function I(a, b) { return [Math.min(a, b), Math.max(a, b)]; }
    var D = d.map(function (v) { return [v - hi, v - lo]; });
    var slope = [D[n][0] - D[0][1], D[n][1] - D[0][0]];
    var off = [0, 0];
    for (var i = 0; i <= n; i++) {
      var xi = i / n;
      // D[i] - (D[0] + slope * xi), in interval arithmetic.
      var ls = [slope[0] * xi, slope[1] * xi];
      var o = [D[i][0] - D[0][1] - ls[1], D[i][1] - D[0][0] - ls[0]];
      off = [Math.min(off[0], o[0]), Math.max(off[1], o[1])];
    }
    var hull = [Math.min(D[0][0], D[n][0]) + off[0], Math.max(D[0][1], D[n][1]) + off[1]];
    if (hull[0] > 0 || hull[1] < 0) return null;
    if (slope[0] <= 0 && slope[1] >= 0) return [0, 1];
    var num = [D[0][0] + off[0], D[0][1] + off[1]];
    var qs = [num[0] / slope[0], num[0] / slope[1], num[1] / slope[0], num[1] / slope[1]];
    var a = -Math.max.apply(null, qs), b = -Math.min.apply(null, qs);
    var r = I(Math.max(a, 0), Math.min(b, 1));
    return a > 1 || b < 0 ? null : r;
  }

  // The search: pieces of A and B over parameter ranges. Clip one against the other's
  // fat line, then the other way; while a clip removes at least 20% of a piece, restrict
  // it and go on; else halve the longer range. Each entry of the log is one clip.
  function run(A, B, mode, tol, maxClips) {
    var clip = mode === "chord" ? chordClip : hullClip;
    var stack = [{ a: [0, 1], b: [0, 1], turn: 0 }], log = [], found = [], clips = 0, bisections = 0;
    while (stack.length && clips < maxClips) {
      var s = stack.pop();
      if (s.a[1] - s.a[0] <= tol && s.b[1] - s.b[0] <= tol) { found.push([s.a, s.b]); log.push({ state: s, action: "found" }); continue; }
      var which = s.turn, range = which ? s.b : s.a, other = which ? s.a : s.b;
      var P = restrict(which ? B : A, range[0], range[1]), Q = restrict(which ? A : B, other[0], other[1]);
      var line = fatLine(Q);
      clips++;
      if (!line) { stack.push({ a: s.a, b: s.b, turn: 1 - s.turn }); continue; }
      var d = distances(P, line), t = clip(d, line.dmin, line.dmax);
      var entry = { state: s, which: which, P: P, Q: Q, line: line, d: d, t: t, range: range };
      if (!t) { entry.action = "rejected"; log.push(entry); continue; }
      var w = range[1] - range[0], nr = [range[0] + t[0] * w, range[0] + t[1] * w];
      var next = { a: which ? s.a : nr, b: which ? nr : s.b };
      var removed = 1 - (t[1] - t[0]);
      if (removed >= 0.2) {
        entry.action = "restricted"; next.turn = 1 - which;
        stack.push(next);
      } else if (s.stalled) {
        // Clipping both ways removed too little: several solutions share this box.
        var ca = next.a, cb = next.b;
        entry.action = "halved"; bisections++;
        if (ca[1] - ca[0] >= cb[1] - cb[0]) {
          var m = (ca[0] + ca[1]) / 2;
          stack.push({ a: [m, ca[1]], b: cb, turn: 0 }, { a: [ca[0], m], b: cb, turn: 0 });
        } else {
          var mb = (cb[0] + cb[1]) / 2;
          stack.push({ a: ca, b: [mb, cb[1]], turn: 0 }, { a: ca, b: [cb[0], mb], turn: 0 });
        }
      } else {
        entry.action = "little progress"; next.turn = 1 - which; next.stalled = true;
        stack.push(next);
      }
      log.push(entry);
    }
    return { log: log, found: found, clips: clips, bisections: bisections, finished: !stack.length };
  }

  // Plain bisection with boxes, to the same width: how many pairs it looks at.
  function bisectionNodes(A, B, tol) {
    function box(p) {
      var b = [Infinity, Infinity, -Infinity, -Infinity];
      p.forEach(function (q) { b[0] = Math.min(b[0], q[0]); b[1] = Math.min(b[1], q[1]); b[2] = Math.max(b[2], q[0]); b[3] = Math.max(b[3], q[1]); });
      return b;
    }
    var stack = [[A, [0, 1], B, [0, 1]]], nodes = 0;
    while (stack.length && nodes < 200000) {
      var s = stack.pop(); nodes++;
      var ba = box(s[0]), bb = box(s[2]);
      if (ba[2] < bb[0] || bb[2] < ba[0] || ba[3] < bb[1] || bb[3] < ba[1]) continue;
      var wa = s[1][1] - s[1][0], wb = s[3][1] - s[3][0];
      if (wa <= tol && wb <= tol) continue;
      if (wa >= wb) {
        var h = splitAt(s[0], 0.5), m = (s[1][0] + s[1][1]) / 2;
        stack.push([h[0], [s[1][0], m], s[2], s[3]], [h[1], [m, s[1][1]], s[2], s[3]]);
      } else {
        var k = splitAt(s[2], 0.5), n = (s[3][0] + s[3][1]) / 2;
        stack.push([s[0], s[1], k[0], [s[3][0], n]], [s[0], s[1], k[1], [n, s[3][1]]]);
      }
    }
    return nodes;
  }

  // Lower and upper convex hull of the graph's control polygon, for drawing.
  function hullPoints(d) {
    var n = d.length - 1, pts = d.map(function (v, i) { return [i / n, v]; });
    function cross(o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); }
    var lo = [], up = [], i;
    for (i = 0; i <= n; i++) { while (lo.length > 1 && cross(lo[lo.length - 2], lo[lo.length - 1], pts[i]) <= 0) lo.pop(); lo.push(pts[i]); }
    for (i = n; i >= 0; i--) { while (up.length > 1 && cross(up[up.length - 2], up[up.length - 1], pts[i]) <= 0) up.pop(); up.push(pts[i]); }
    lo.pop(); up.pop();
    return lo.concat(up);
  }

  GeopBook.demo("fat-line-clip", function (ctx) {
    var D = GeopBook.draw;
    var A = [[0.3, 0.4], [1.6, 3.9], [3.6, 3.4], [5.6, 1.2]], B = [[0.4, 3.2], [2.1, 0.2], [3.9, 0.9], [5.5, 3.6]];
    var pts = A.concat(B), mode = "hull", shown = 0, res = null, bis = 0, timer = null;
    var stage = ctx.stage({ aspect: 0.95, maxHeight: 620, label: "Two curves, the fat line of one, and the distance function of the other with its hull and the clip interval", draw: paint });
    var box = { x0: 0, x1: 6, y0: 0, y1: 4 };
    var top = { h: 0, w: 0 };
    // The upper part of the stage is the plane of the curves.
    var fake = { get w() { return stage.w; }, get h() { return stage.h * 0.58; } };
    var view = ctx.view(fake, box, { pad: 10 });
    ctx.handles({ stage: stage, view: view, pts: pts, onchange: function () { recompute(); } });
    var read = ctx.readout();

    function recompute() {
      res = run(A, B, mode, 1e-9, 200);
      bis = bisectionNodes(A, B, 1e-9);
      if (shown > res.log.length - 1) shown = Math.max(res.log.length - 1, 0);
      stepSlider.el.max = Math.max(res.log.length - 1, 1);
      stepSlider.value = shown;
      describe(); stage.redraw();
    }
    function describe() {
      var e = res.log[shown];
      if (!e) { read.set("The curves' fat lines are apart: no crossing."); return; }
      var s = "clip " + (shown + 1) + " of " + res.log.length + ": ";
      if (e.action === "found") {
        s += "found: both ranges within 1e-9.\n  s in [" + e.state.a[0].toFixed(10) + ", " + e.state.a[1].toFixed(10) + "]\n  t in [" + e.state.b[0].toFixed(10) + ", " + e.state.b[1].toFixed(10) + "]";
      } else {
        var name = e.which ? "B" : "A";
        s += name + " against the fat line of " + (e.which ? "A" : "B") + ": " + e.action;
        if (e.t) s += ", kept " + ((e.t[1] - e.t[0]) * 100).toFixed(1) + "% of the piece";
        s += ".\n  s range width " + (e.state.a[1] - e.state.a[0]).toExponential(2) + ", t range width " + (e.state.b[1] - e.state.b[0]).toExponential(2);
      }
      s += "\nWhole search: " + res.clips + " clips, " + res.bisections + " halvings, " + res.found.length + " crossing(s). Plain bisection to the same width: " + bis + " pairs of boxes.";
      read.set(s);
    }
    function paint(g, W, H, c) {
      if (!res) return;
      view.update();
      var e = res.log[shown];
      function curve(P, col, w) {
        var poly = [];
        for (var i = 0; i <= 80; i++) poly.push(evaluate(P, i / 80));
        D.polyline(g, view, poly, col, w);
      }
      curve(A, c.faint, 1.5); curve(B, c.faint, 1.5);
      if (e && e.P) {
        var l = e.line, n = l.n, t = [n[1], -n[0]], base = [-l.c * n[0], -l.c * n[1]];
        // The band of the fat line, drawn long enough to cross the plane.
        var R = 20;
        function band(d) { return [base[0] + n[0] * d - t[0] * R, base[1] + n[1] * d - t[1] * R]; }
        function band2(d) { return [base[0] + n[0] * d + t[0] * R, base[1] + n[1] * d + t[1] * R]; }
        g.save(); g.beginPath(); g.rect(0, 0, W, fake.h); g.clip();
        D.polygon(g, view, [band(l.dmin), band2(l.dmin), band2(l.dmax), band(l.dmax)], c.fillC, c.green, 0.75);
        g.restore();
      }
      var pa = e ? restrict(A, e.state.a[0], e.state.a[1]) : A, pb = e ? restrict(B, e.state.b[0], e.state.b[1]) : B;
      curve(pa, c.blue, 3); curve(pb, c.orange, 3);
      A.forEach(function (p) { D.dot(g, view, p, 4.5, c.bg, c.blue); });
      B.forEach(function (p) { D.dot(g, view, p, 4.5, c.bg, c.orange); });
      res.found.forEach(function (f) { D.dot(g, view, evaluate(A, (f[0][0] + f[0][1]) / 2), 4, c.red); });

      // The lower part: the distance function over the piece's own [0, 1].
      var y0 = fake.h + 6, h = H - y0 - 16, x0 = 34, w = W - x0 - 10;
      g.strokeStyle = c.faint; g.lineWidth = 1; g.strokeRect(x0, y0, w, h);
      if (!e || !e.P) { D.text(g, null, [x0 + 6, y0 + 16], e ? "found" : "", c.muted); return; }
      var lo = Math.min.apply(null, e.d.concat([e.line.dmin, 0])), hi = Math.max.apply(null, e.d.concat([e.line.dmax, 0]));
      var pad = (hi - lo) * 0.1 || 1e-12; lo -= pad; hi += pad;
      function X(t) { return x0 + t * w; }
      function Y(v) { return y0 + h - (v - lo) / (hi - lo) * h; }
      g.fillStyle = c.fillC; g.fillRect(x0, Y(e.line.dmax), w, Y(e.line.dmin) - Y(e.line.dmax));
      if (e.t) { g.fillStyle = c.fillB; g.fillRect(X(e.t[0]), y0, X(e.t[1]) - X(e.t[0]), h); }
      var hp = hullPoints(e.d);
      D.polygon(g, null, hp.map(function (q) { return [X(q[0]), Y(q[1])]; }), c.fillA, c.blue, 1);
      var graph = [];
      for (var i = 0; i <= 60; i++) graph.push([X(i / 60), Y(bernsteinValue(e.d, i / 60))]);
      D.polyline(g, null, graph, c.fg, 2);
      e.d.forEach(function (v, k) { D.dot(g, null, [X(k / (e.d.length - 1)), Y(v)], 3.5, c.bg, c.blue); });
      if (e.t) {
        [e.t[0], e.t[1]].forEach(function (x) { D.polyline(g, null, [[X(x), y0], [X(x), y0 + h]], c.orange, 1.5); });
      }
      D.text(g, null, [x0 + 4, y0 + 13], "distance from the fat line, along the " + (e.which ? "orange" : "blue") + " piece", c.muted);
      D.text(g, null, [x0, H - 3], "t = 0", c.muted);
      D.text(g, null, [x0 + w, H - 3], "1", c.muted, "right");
    }
    function show(v) { shown = v; stepSlider.value = v; describe(); stage.redraw(); }
    function stop() { if (timer) { clearInterval(timer); timer = null; playBtn.textContent = "Play"; } }
    var playBtn = ctx.button("Play", function () {
      if (timer) { stop(); return; }
      if (shown >= res.log.length - 1) show(0);
      playBtn.textContent = "Pause";
      timer = setInterval(function () {
        if (!ctx.visible) return;
        if (shown >= res.log.length - 1) { stop(); return; }
        show(shown + 1);
      }, 900);
    });
    ctx.button("Next clip", function () { stop(); show(Math.min(shown + 1, res.log.length - 1)); });
    ctx.button("Reset", function () { stop(); show(0); });
    var stepSlider = ctx.slider({
      label: "clip", min: 0, max: 1, step: 1, value: 0, format: function (v) { return String(v + 1); },
      oninput: function (v) { stop(); shown = v; describe(); stage.redraw(); }
    });
    ctx.select({
      label: "clip with", options: [["hull", "exact hull"], ["chord", "chord band (kernel)"]], value: "hull",
      oninput: function (v) { stop(); mode = v; shown = 0; recompute(); }
    });
    recompute();
  }, { evaluate: evaluate, restrict: restrict, fatLine: fatLine, distances: distances, bernsteinValue: bernsteinValue, hullClip: hullClip, chordClip: chordClip, run: run, bisectionNodes: bisectionNodes });
})();
