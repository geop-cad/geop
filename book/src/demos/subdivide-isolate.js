// Branch and bound over pairs of Bézier pieces: a pair whose control-point boxes
// are apart holds no crossing, any other is cut, level by level. Simpler than the
// kernel, which also clips each pair (Lecture 13). Pure functions in `algo`.
(function () {
  "use strict";

  function lerp(a, b, t) { return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]; }

  // De Casteljau at one half: the control points of the two halves.
  function halve(pts) {
    var left = [pts[0]], right = [pts[pts.length - 1]], row = pts;
    while (row.length > 1) {
      var next = [];
      for (var i = 0; i + 1 < row.length; i++) next.push(lerp(row[i], row[i + 1], 0.5));
      left.push(next[0]); right.unshift(next[next.length - 1]);
      row = next;
    }
    return [left, right];
  }
  function evaluate(pts, t) {
    var row = pts;
    while (row.length > 1) {
      var next = [];
      for (var i = 0; i + 1 < row.length; i++) next.push(lerp(row[i], row[i + 1], t));
      row = next;
    }
    return row[0];
  }
  function box(pts) {
    var b = [Infinity, Infinity, -Infinity, -Infinity];
    pts.forEach(function (p) {
      b[0] = Math.min(b[0], p[0]); b[1] = Math.min(b[1], p[1]);
      b[2] = Math.max(b[2], p[0]); b[3] = Math.max(b[3], p[1]);
    });
    return b;
  }
  function boxesApart(a, b) { return a[2] < b[0] || b[2] < a[0] || a[3] < b[1] || b[3] < a[1]; }
  function boxSize(b) { return Math.max(b[2] - b[0], b[3] - b[1]); }
  function piece(pts, t0, t1) { return { pts: pts, t0: t0, t1: t1, box: box(pts) }; }

  // One level of the search. A pair is a node: its boxes apart, it is dropped;
  // both pieces small, it is a leaf; otherwise the larger piece is halved.
  function level(pairs, handoff) {
    var next = [], leaves = [], rejected = 0;
    pairs.forEach(function (pair) {
      var a = pair[0], b = pair[1];
      if (boxesApart(a.box, b.box)) { rejected++; return; }
      var sa = boxSize(a.box), sb = boxSize(b.box);
      if (sa <= handoff && sb <= handoff) { leaves.push(pair); return; }
      if (sa >= sb) {
        var ha = halve(a.pts), m = (a.t0 + a.t1) / 2;
        next.push([piece(ha[0], a.t0, m), b], [piece(ha[1], m, a.t1), b]);
      } else {
        var hb = halve(b.pts), n = (b.t0 + b.t1) / 2;
        next.push([a, piece(hb[0], b.t0, n)], [a, piece(hb[1], n, b.t1)]);
      }
    });
    return { next: next, leaves: leaves, rejected: rejected };
  }

  // The whole search: every level's pairs before they were looked at.
  function search(A, B, handoff, maxNodes) {
    var pairs = [[piece(A, 0, 1), piece(B, 0, 1)]], levels = [], leaves = [], nodes = 0, rejected = 0, exhausted = false;
    while (pairs.length) {
      if (nodes + pairs.length > maxNodes) {
        exhausted = true;
        levels.push({ pairs: pairs, nodes: nodes, rejected: rejected, leaves: leaves.slice(), exhausted: true });
        break;
      }
      levels.push({ pairs: pairs, nodes: nodes, rejected: rejected, leaves: leaves.slice() });
      nodes += pairs.length;
      var r = level(pairs, handoff);
      rejected += r.rejected;
      leaves = leaves.concat(r.leaves);
      pairs = r.next;
    }
    if (!exhausted) levels.push({ pairs: [], nodes: nodes, rejected: rejected, leaves: leaves.slice() });
    return { levels: levels, leaves: leaves, nodes: nodes, rejected: rejected, exhausted: exhausted };
  }

  // Leaves whose parameter boxes touch are one unresolved cluster, merged by union
  // (the kernel's `DisjointSet` does this).
  function clusters(leaves) {
    var parent = leaves.map(function (_, i) { return i; });
    function find(i) { while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; }
    var boxes = leaves.map(function (p) { return [p[0].t0, p[0].t1, p[1].t0, p[1].t1]; });
    // Sort by s, so each leaf is only compared with those overlapping it in s.
    var order = boxes.map(function (_, i) { return i; }).sort(function (x, y) { return boxes[x][0] - boxes[y][0]; });
    for (var oi = 0; oi < order.length; oi++) {
      var i = order[oi];
      for (var oj = oi + 1; oj < order.length; oj++) {
        var j = order[oj];
        if (boxes[j][0] > boxes[i][1]) break;
        if (boxes[j][2] <= boxes[i][3] && boxes[i][2] <= boxes[j][3]) parent[find(i)] = find(j);
      }
    }
    var out = {};
    boxes.forEach(function (b, i) {
      var r = find(i);
      if (!out[r]) out[r] = { s0: b[0], s1: b[1], t0: b[2], t1: b[3], leaves: 0 };
      var c = out[r];
      c.s0 = Math.min(c.s0, b[0]); c.s1 = Math.max(c.s1, b[1]);
      c.t0 = Math.min(c.t0, b[2]); c.t1 = Math.max(c.t1, b[3]);
      c.leaves++;
    });
    return Object.keys(out).map(function (k) { return out[k]; }).sort(function (x, y) { return x.s0 - y.s0; });
  }

  var PRESETS = {
    "three crossings": [[[0.3, 0.5], [1.6, 4.2], [3.8, -1.2], [5.7, 3.1]], [[0.2, 1.8], [2.0, 1.2], [4.0, 2.2], [5.8, 1.6]]],
    "tangent": [[[0.5, 0.5], [2.0, 3.5], [4.0, 3.5], [5.5, 0.5]], [[0.5, 2.75], [2.0, 2.75], [4.0, 2.75], [5.5, 2.75]]],
    "overlapping": [[[0.5, 0.5], [2.0, 3.5], [4.0, 3.5], [5.5, 0.5]], null],
    "apart": [[[0.5, 0.4], [2.0, 1.8], [4.0, 1.8], [5.5, 0.4]], [[0.5, 3.6], [2.0, 2.2], [4.0, 2.2], [5.5, 3.6]]]
  };
  // The overlap preset's second curve is a piece of the first, by de Casteljau.
  function subPiece(pts, a, b) {
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
    var right = splitAt(pts, a)[1];
    return splitAt(right, (b - a) / (1 - a))[0];
  }
  // Tilted, so that the common tangent runs along neither axis: a box then
  // overlaps its partner well before the curves come close (see the lecture).
  function tilt(q) {
    var x = q[0] - 3, y = q[1] - 2.2, c = Math.cos(0.35), s = Math.sin(0.35);
    return [3 + 0.8 * (c * x - s * y), 2.2 + 0.8 * (s * x + c * y)];
  }
  function preset(name) {
    var p = PRESETS[name];
    if (name === "tangent") return [p[0].map(tilt), p[1].map(tilt)];
    return [p[0].map(function (q) { return q.slice(); }), p[1] ? p[1].map(function (q) { return q.slice(); }) : subPiece(p[0], 0.25, 0.8)];
  }

  GeopBook.demo("subdivide-isolate", function (ctx) {
    var D = GeopBook.draw;
    var curves = preset("three crossings"), pts = [];
    function linkHandles() { pts.length = 0; curves[0].concat(curves[1]).forEach(function (p) { pts.push(p); }); }
    linkHandles();
    var handoff = 0.04, shown = 0, result = null;
    var stage = ctx.stage({ aspect: 1.55, maxHeight: 440, label: "Two Bézier curves, and the boxes of the pairs of pieces the search has not yet rejected", draw: paint });
    var view = ctx.view(stage, { x0: 0, x1: 6, y0: 0, y1: 4 });
    ctx.handles({ stage: stage, view: view, pts: pts, onchange: function () { recompute(); } });
    var read = ctx.readout();

    function recompute() {
      result = search(curves[0], curves[1], handoff, 20000);
      var last = result.levels.length - 1;
      if (shown > last) shown = last;
      levelSlider.el.max = last;
      levelSlider.value = shown;
      describe();
      stage.redraw();
    }
    function describe() {
      var l = result.levels[Math.min(shown, result.levels.length - 1)];
      var cs = clusters(l.leaves);
      var text = "level " + shown + " of " + (result.levels.length - 1) +
        ": " + l.pairs.length + " pairs alive, " + l.nodes + " nodes visited, " + l.rejected + " rejected, " + l.leaves.length + " leaves in " + cs.length + " cluster(s)";
      if (shown >= result.levels.length - 1) {
        if (result.exhausted) text += "\nBudget exhausted with pairs left: the result would be incomplete, and is an error.";
        else if (!cs.length) text += "\nEvery pair rejected: a proof that the curves do not meet.";
        else {
          text += "\nCandidates (s-range, width in s):";
          cs.slice(0, 5).forEach(function (c) { text += "\n  [" + c.s0.toFixed(4) + ", " + c.s1.toFixed(4) + "]  " + (c.s1 - c.s0).toExponential(1) + "  from " + c.leaves + " leaves"; });
          if (cs.length > 5) text += "\n  ...";
        }
      }
      read.set(text);
    }
    function paint(g, w, h, c) {
      if (!result) return;
      view.update();
      var l = result.levels[Math.min(shown, result.levels.length - 1)];
      function rect(b, fill, stroke) {
        var x = view.x(b[0]), y = view.y(b[3]), W = view.x(b[2]) - x, H = view.y(b[1]) - y;
        g.fillStyle = fill; g.fillRect(x, y, W, H);
        g.strokeStyle = stroke; g.lineWidth = 0.75; g.strokeRect(x, y, W, H);
      }
      var seenA = {}, seenB = {};
      l.pairs.slice(0, 400).forEach(function (p) {
        var ka = p[0].t0 + ":" + p[0].t1, kb = p[1].t0 + ":" + p[1].t1;
        if (!seenA[ka]) { seenA[ka] = 1; rect(p[0].box, c.fillA, c.blue); }
        if (!seenB[kb]) { seenB[kb] = 1; rect(p[1].box, c.fillB, c.orange); }
      });
      [0, 1].forEach(function (k) {
        var poly = [];
        for (var i = 0; i <= 100; i++) poly.push(evaluate(curves[k], i / 100));
        D.polyline(g, view, curves[k], c.faint, 1, [3, 3]);
        D.polyline(g, view, poly, k ? c.orange : c.blue, 2);
        curves[k].forEach(function (p) { D.dot(g, view, p, 5, c.bg, k ? c.orange : c.blue); });
      });
      clusters(l.leaves).forEach(function (cl) {
        // Draw the piece of the first curve the cluster covers.
        var poly = [];
        for (var i = 0; i <= 12; i++) poly.push(evaluate(curves[0], cl.s0 + (cl.s1 - cl.s0) * i / 12));
        D.polyline(g, view, poly, c.red, 4);
      });
    }

    // The number of levels depends on the curves, so the slider's range follows them.
    var timer = null, playBtn;
    function show(v) { shown = v; levelSlider.value = v; describe(); stage.redraw(); }
    function stop() { if (timer) { clearInterval(timer); timer = null; playBtn.textContent = "Play"; } }
    playBtn = ctx.button("Play", function () {
      if (timer) { stop(); return; }
      if (shown >= result.levels.length - 1) show(0);
      playBtn.textContent = "Pause";
      timer = setInterval(function () {
        if (!ctx.visible) return;
        if (shown >= result.levels.length - 1) { stop(); return; }
        show(shown + 1);
      }, 450);
    });
    ctx.button("Next level", function () { stop(); show(Math.min(shown + 1, result.levels.length - 1)); });
    ctx.button("Last level", function () { stop(); show(result.levels.length - 1); });
    ctx.button("Reset", function () { stop(); show(0); });
    var levelSlider = ctx.slider({
      label: "level", min: 0, max: 1, step: 1, value: 0,
      format: function (v) { return String(v); },
      oninput: function (v) { stop(); shown = v; describe(); stage.redraw(); }
    });
    ctx.select({
      label: "scene", options: Object.keys(PRESETS), value: "three crossings", oninput: function (name) {
        stop(); curves = preset(name); linkHandles(); shown = 0; recompute();
      }
    });
    ctx.slider({
      label: "handoff", min: 0.005, max: 0.3, step: 0.005, value: handoff, format: function (v) { return v.toFixed(3); },
      oninput: function (v) { handoff = v; recompute(); }
    });
    recompute();
  }, { evaluate: evaluate, halve: halve, box: box, search: search, clusters: clusters, preset: preset, subPiece: subPiece });
})();
