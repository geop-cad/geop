// A boolean of two polygons one dimension down, in the shape of the kernel's: a
// remesh that puts a vertex at every coincidence, in increasing dimension, then a
// classification of each piece by a point of it, then the keep table of
// lib/geop-lib-booleans/src/boolean.rs. An edge plays a face; there is no tracing
// to do in the plane. data-args '{"mode":"remesh"}' shows the pipeline step by step;
// the default shows the result.
(function () {
  "use strict";

  function signedArea(poly) {
    var s = 0;
    poly.forEach(function (p, i) { var q = poly[(i + 1) % poly.length]; s += p[0] * q[1] - q[0] * p[1]; });
    return s / 2;
  }
  function orient(a, b, c) { return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]); }
  function same(a, b) { return a[0] === b[0] && a[1] === b[1]; }
  function onSegment(a, b, p) {
    return orient(a, b, p) === 0 && (p[0] - a[0]) * (p[0] - b[0]) <= 0 && (p[1] - a[1]) * (p[1] - b[1]) <= 0;
  }
  // Is a polygon simple? A boolean's operands have to be solids.
  function isSimple(poly) {
    var n = poly.length;
    for (var i = 0; i < n; i++) {
      var a = poly[i], b = poly[(i + 1) % n];
      if (same(a, b)) return false;
      for (var j = i + 1; j < n; j++) {
        var c = poly[j], d = poly[(j + 1) % n];
        var adjacent = j === i + 1 || (i === 0 && j === n - 1);
        if (adjacent) {
          var shared = j === i + 1 ? b : a, far1 = j === i + 1 ? a : b, far2 = j === i + 1 ? d : c;
          if (orient(shared, far1, far2) === 0 && (far1[0] - shared[0]) * (far2[0] - shared[0]) + (far1[1] - shared[1]) * (far2[1] - shared[1]) > 0) return false;
          continue;
        }
        var d1 = orient(a, b, c), d2 = orient(a, b, d), d3 = orient(c, d, a), d4 = orient(c, d, b);
        if (d1 * d2 < 0 && d3 * d4 < 0) return false;
        if (onSegment(a, b, c) || onSegment(a, b, d) || onSegment(c, d, a) || onSegment(c, d, b)) return false;
      }
    }
    return true;
  }
  // A polygon's edges, counter-clockwise whichever way it was drawn: the
  // material on the left, as a solid's outward normal fixes it.
  function ccw(poly) { return signedArea(poly) < 0 ? poly.slice().reverse() : poly.slice(); }

  // Parameters along a-b at which it is to be split, for each kind of coincidence.
  function paramOn(a, b, p) {
    var dx = b[0] - a[0], dy = b[1] - a[1];
    return Math.abs(dx) > Math.abs(dy) ? (p[0] - a[0]) / dx : (p[1] - a[1]) / dy;
  }
  function crossing(a, b, c, d) {
    var d1 = orient(c, d, a), d2 = orient(c, d, b), d3 = orient(a, b, c), d4 = orient(a, b, d);
    if (d1 * d2 < 0 && d3 * d4 < 0) {
      var t = d1 / (d1 - d2);
      return { t: t, u: d3 / (d3 - d4), p: [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t] };
    }
    return null;
  }

  // The remesh, up to `stage`:
  // 1 vertices x vertices, 2 vertices x edges, 3 edges x edges.
  function remesh(A, B, stage) {
    var splitsA = A.map(function () { return []; }), splitsB = B.map(function () { return []; });
    var log = { merged: 0, onEdge: 0, crossed: 0 }, points = [];
    function note(list, t, p) {
      if (t <= 0 || t >= 1) return;
      if (list.some(function (s) { return s.t === t; })) return;
      list.push({ t: t, p: p });
    }
    function edge(P, i) { return [P[i], P[(i + 1) % P.length]]; }
    if (stage >= 1) {
      A.forEach(function (a) {
        B.forEach(function (b) { if (same(a, b)) { log.merged++; points.push({ p: a, kind: "merged" }); } });
      });
    }
    if (stage >= 2) {
      function vertexOnEdges(V, E, splits) {
        V.forEach(function (v) {
          E.forEach(function (_, i) {
            var e = edge(E, i);
            if (!same(v, e[0]) && !same(v, e[1]) && onSegment(e[0], e[1], v)) {
              note(splits[i], paramOn(e[0], e[1], v), v);
              log.onEdge++;
              points.push({ p: v, kind: "on edge" });
            }
          });
        });
      }
      vertexOnEdges(B, A, splitsA);
      vertexOnEdges(A, B, splitsB);
    }
    if (stage >= 3) {
      A.forEach(function (_, i) {
        B.forEach(function (_, j) {
          var e = edge(A, i), f = edge(B, j), x = crossing(e[0], e[1], f[0], f[1]);
          if (x) {
            note(splitsA[i], x.t, x.p);
            note(splitsB[j], x.u, x.p);
            log.crossed++;
            points.push({ p: x.p, kind: "crossing" });
          }
        });
      });
    }
    function pieces(P, splits, from) {
      var out = [];
      P.forEach(function (_, i) {
        var e = edge(P, i), s = splits[i].slice().sort(function (a, b) { return a.t - b.t; }), prev = e[0];
        s.forEach(function (x) { out.push({ a: prev, b: x.p, from: from }); prev = x.p; });
        out.push({ a: prev, b: e[1], from: from });
      });
      return out;
    }
    return { a: pieces(A, splitsA, "a"), b: pieces(B, splitsB, "b"), log: log, points: points };
  }

  // Against a polygon's boundary and by parity: a ray in a direction clear of
  // every vertex of the polygon (redrawn until one is, as the kernel's rays are).
  function pointIn(poly, p) {
    var i, n = poly.length;
    for (i = 0; i < n; i++) if (onSegment(poly[i], poly[(i + 1) % n], p)) return { on: true, edge: i };
    var attempts = 0;
    for (;;) {
      var ang = 0.7345 + 1.2345 * attempts++, dir = [Math.cos(ang), Math.sin(ang)], clear = true, count = 0;
      for (i = 0; i < n; i++) {
        var dx = poly[i][0] - p[0], dy = poly[i][1] - p[1];
        if (Math.abs(dx * -dir[1] + dy * dir[0]) < 1e-9 && dx * dir[0] + dy * dir[1] > 0) { clear = false; break; }
      }
      if (!clear) continue;
      for (i = 0; i < n; i++) {
        var a = poly[i], b = poly[(i + 1) % n];
        var ex = b[0] - a[0], ey = b[1] - a[1], den = dir[0] * ey - dir[1] * ex;
        if (den === 0) continue;
        var t = ((a[0] - p[0]) * ey - (a[1] - p[1]) * ex) / den, s = ((a[0] - p[0]) * dir[1] - (a[1] - p[1]) * dir[0]) / den;
        if (t > 0 && s >= 0 && s < 1) count++;
      }
      return { on: false, inside: count % 2 === 1, count: count };
    }
  }
  // The classes of FaceClassification, for one piece against the other polygon,
  // at the piece's middle. A piece on the other's boundary runs along it: the same
  // way (material on one side of both), or against it.
  function classify(piece, other) {
    var mid = [(piece.a[0] + piece.b[0]) / 2, (piece.a[1] + piece.b[1]) / 2];
    var r = pointIn(other, mid);
    if (!r.on) return r.inside ? "Inside" : "Outside";
    var e0 = other[r.edge], e1 = other[(r.edge + 1) % other.length];
    var dot = (piece.b[0] - piece.a[0]) * (e1[0] - e0[0]) + (piece.b[1] - piece.a[1]) * (e1[1] - e0[1]);
    return dot > 0 ? "OnSameNormal" : "OnOppositeNormal";
  }
  // The table `keeps` in boolean.rs.
  function keeps(op, cls, fromA) {
    switch (op) {
      case "union":
        if (cls === "Outside") return "keep";
        if (cls === "OnSameNormal" && fromA) return "keep";
        return "drop";
      case "intersection":
        if (cls === "Inside") return "keep";
        if (cls === "OnSameNormal" && fromA) return "keep";
        return "drop";
      case "difference":
        if ((cls === "Outside" || cls === "OnOppositeNormal") && fromA) return "keep";
        if (cls === "Inside" && !fromA) return "reversed";
        return "drop";
      case "symmetric":
        if (cls === "Outside") return "keep";
        if (cls === "Inside") return "reversed";
        return "drop";
    }
    throw new Error("no such operator: " + op);
  }
  // Keep, reverse or drop; then chain what is kept into loops, by following the
  // vertices (`assemble_solid`, one dimension down). A vertex a result touches
  // itself at has more than one way on: take the sharpest left turn away from it,
  // so that each ring stays simple.
  function key(p) { return p[0].toFixed(9) + "," + p[1].toFixed(9); }
  function assemble(kept) {
    var out = {}, used = kept.map(function () { return false; });
    kept.forEach(function (k, i) { var c = key(k.a); (out[c] = out[c] || []).push(i); });
    var loops = [];
    kept.forEach(function (start, i) {
      if (used[i]) return;
      var ring = [], cur = i, guard = 0;
      while (cur !== undefined && !used[cur] && guard++ < 10000) {
        used[cur] = true;
        ring.push(kept[cur].a);
        var at = kept[cur];
        var options = (out[key(at.b)] || []).filter(function (j) { return !used[j]; });
        if (options.length > 1) {
          var inAng = Math.atan2(at.b[1] - at.a[1], at.b[0] - at.a[0]);
          options.sort(function (x, y) {
            function turn(j) {
              var t = Math.atan2(kept[j].b[1] - kept[j].a[1], kept[j].b[0] - kept[j].a[0]) - inAng;
              while (t <= -Math.PI) t += 2 * Math.PI;
              while (t > Math.PI) t -= 2 * Math.PI;
              return t;
            }
            return turn(y) - turn(x);
          });
        }
        cur = options[0];
      }
      loops.push(ring);
    });
    return loops;
  }
  function boolean(A0, B0, op) {
    var A = ccw(A0), B = ccw(B0);
    var r = remesh(A, B, 3);
    var pieces = [];
    function run(list, other, fromA) {
      list.forEach(function (p) {
        var cls = classify(p, other), k = keeps(op, cls, fromA);
        pieces.push({ a: p.a, b: p.b, from: p.from, cls: cls, keep: k });
      });
    }
    run(r.a, B, true);
    run(r.b, A, false);
    var kept = pieces.filter(function (p) { return p.keep !== "drop"; }).map(function (p) {
      return p.keep === "reversed" ? { a: p.b, b: p.a } : { a: p.a, b: p.b };
    });
    var loops = assemble(kept);
    var area = 0;
    loops.forEach(function (l) { area += signedArea(l); });
    return { pieces: pieces, loops: loops, area: area, log: r.log };
  }

  var OPS = [["union", "union"], ["intersection", "intersection"], ["difference", "difference a - b"], ["symmetric", "symmetric difference"]];
  var GRID = 0.25;

  GeopBook.demo("boolean-2d", function (ctx) {
    var D = GeopBook.draw;
    var remeshMode = ctx.args.mode === "remesh";
    var A = [[0.5, 0.5], [3, 0.5], [3, 2.5], [0.5, 2.5]];
    var B = [[2, 1.5], [4.5, 1], [5, 3], [3, 3.5], [2, 2.5]];
    var op = "difference", step = remeshMode ? 0 : 4;
    var pts = A.concat(B);
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 400, label: "Two polygons and their boolean", draw: paint });
    var view = ctx.view(stage, { x0: 0, x1: 5.5, y0: 0, y1: 4 });
    var handles = ctx.handles({
      stage: stage, view: view, pts: pts, onchange: update,
      constrain: function (i, p) {
        // A grid, so that vertices can land exactly on each other and on edges.
        return [Math.max(0, Math.min(5.5, Math.round(p[0] / GRID) * GRID)), Math.max(0, Math.min(4, Math.round(p[1] / GRID) * GRID))];
      }
    });
    var read = ctx.readout();

    function paintPiece(g, p, color, width, dash) {
      D.polyline(g, view, [p.a, p.b], color, width, dash);
      // An arrow, the way the piece runs.
      var mx = (view.x(p.a[0]) + view.x(p.b[0])) / 2, my = (view.y(p.a[1]) + view.y(p.b[1])) / 2;
      var ang = Math.atan2(view.y(p.b[1]) - view.y(p.a[1]), view.x(p.b[0]) - view.x(p.a[0]));
      g.strokeStyle = color; g.lineWidth = Math.max(1, width - 0.5); g.setLineDash([]);
      g.beginPath(); g.moveTo(mx + 5 * Math.cos(ang), my + 5 * Math.sin(ang));
      g.lineTo(mx - 5 * Math.cos(ang - 0.6), my - 5 * Math.sin(ang - 0.6));
      g.moveTo(mx + 5 * Math.cos(ang), my + 5 * Math.sin(ang));
      g.lineTo(mx - 5 * Math.cos(ang + 0.6), my - 5 * Math.sin(ang + 0.6));
      g.stroke();
    }
    function paint(g, w, h, c) {
      if (!view) return;
      view.update();
      D.grid(g, view, 0.5, c.grid);
      var valid = isSimple(A) && isSimple(B);
      var classColor = { Inside: c.red, Outside: c.blue, OnSameNormal: c.green, OnOppositeNormal: c.purple };
      if (!valid) {
        D.polygon(g, view, A, null, c.muted, 1.5);
        D.polygon(g, view, B, null, c.muted, 1.5);
      } else if (remeshMode) {
        D.polygon(g, view, A, c.fillA, null);
        D.polygon(g, view, B, c.fillB, null);
        var Ac = ccw(A), Bc = ccw(B), r = remesh(Ac, Bc, Math.min(step, 3));
        var showClasses = step >= 4;
        function drawSet(list, other, base) {
          list.forEach(function (p) {
            paintPiece(g, p, showClasses ? classColor[classify(p, other)] : base, 2.2);
            if (step >= 2 && step < 4) { D.dot(g, view, p.a, 3, base); }
          });
        }
        drawSet(r.a, Bc, c.blue);
        drawSet(r.b, Ac, c.orange);
        r.points.forEach(function (q) {
          D.dot(g, view, q.p, 6, q.kind === "crossing" ? c.red : q.kind === "merged" ? c.green : c.purple, c.fg);
        });
      } else {
        var res = boolean(A, B, op);
        g.beginPath();
        res.loops.forEach(function (l) {
          l.forEach(function (p, i) { if (i) g.lineTo(view.x(p[0]), view.y(p[1])); else g.moveTo(view.x(p[0]), view.y(p[1])); });
          g.closePath();
        });
        g.fillStyle = c.fillC; g.fill("evenodd");
        res.pieces.forEach(function (p) {
          if (p.keep === "drop") paintPiece(g, p, classColor[p.cls], 1, [4, 4]);
        });
        res.pieces.forEach(function (p) {
          if (p.keep === "drop") return;
          paintPiece(g, p.keep === "reversed" ? { a: p.b, b: p.a } : p, classColor[p.cls], 2.8);
        });
      }
      pts.forEach(function (p, i) {
        D.dot(g, view, p, 5, handles && handles.selected === i ? c.accent : c.bg, i < A.length ? c.blue : c.orange);
      });
    }
    function update() {
      if (!isSimple(A) || !isSimple(B)) {
        read.set("A polygon crosses itself: that is no solid, and the kernel's operations would refuse it.");
        stage.redraw();
        return;
      }
      var Ac = ccw(A), Bc = ccw(B);
      if (remeshMode) {
        var r = remesh(Ac, Bc, Math.min(step, 3));
        var titles = ["the two polygons as given", "vertices x vertices: coincident vertices merged", "vertices x edges: an edge is split at a vertex of the other lying on it", "edges x edges: edges are split where they cross", "every piece classified, at its middle, by the other polygon: blue outside, red inside, green along it the same way, purple along it the other way"];
        read.set("step " + step + ": " + titles[step] +
          "\nmerged " + r.log.merged + ", on an edge " + r.log.onEdge + ", crossed " + r.log.crossed +
          "; polygon a in " + r.a.length + " pieces, b in " + r.b.length);
      } else {
        var res = boolean(A, B, op);
        var count = {};
        res.pieces.forEach(function (p) {
          var k = p.from + " " + p.cls + ": " + p.keep;
          count[k] = (count[k] || 0) + 1;
        });
        var a = signedArea(Ac), b = signedArea(Bc), inter = boolean(A, B, "intersection").area;
        var formula = { union: a + b - inter, intersection: inter, difference: a - inter, symmetric: a + b - 2 * inter }[op];
        var lines = Object.keys(count).sort().map(function (k) { return count[k] + " x " + k; });
        read.set(lines.join("\n") +
          "\nresult: " + res.loops.length + " loop(s), area " + res.area.toFixed(4) + (res.loops.length ? "" : " (empty: an answer, not a failure)") +
          "\nfrom the areas, with area(a) = " + a.toFixed(4) + ", area(b) = " + b.toFixed(4) + ", area(a and b) = " + inter.toFixed(4) + ": " + formula.toFixed(4));
      }
      stage.redraw();
    }
    if (remeshMode) {
      ctx.player({ steps: 4, duration: 8, label: "step", onframe: function (t) { step = t; update(); } });
    } else {
      ctx.select({ label: "operator", value: op, options: OPS, oninput: function (v) { op = v; update(); } });
    }
    update();
  }, { signedArea: signedArea, isSimple: isSimple, remesh: remesh, pointIn: pointIn, classify: classify, keeps: keeps, assemble: assemble, boolean: boolean, ccw: ccw });
})();
