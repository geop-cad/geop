// Splicing an edge into a face, on a real half-edge structure in (u, v), by the
// same rewiring and the same case analysis as Model::splice_edge_into_face:
// neither end on the face, one end, or both, and then which boundaries they are
// on. Edges here are straight; the kernel's pcurves need not be.
(function () {
  "use strict";

  // ---- the structure
  function newModel() { return { pts: [], cos: [], faces: [] }; }
  function addPoint(m, p) { m.pts.push(p.slice()); return m.pts.length - 1; }
  function addCoedge(m, a, b, face) { m.cos.push({ a: a, b: b, next: -1, prev: -1, face: face }); return m.cos.length - 1; }
  function link(m, x, y) { m.cos[x].next = y; m.cos[y].prev = x; }
  function ring(m, c) {
    var out = [c], n = m.cos[c].next;
    while (n !== c) { out.push(n); n = m.cos[n].next; if (out.length > m.cos.length + 1) throw new Error("a ring does not close"); }
    return out;
  }
  function addLoop(m, points, face) {
    var ids = points.map(function (p) { return addPoint(m, p); });
    var cs = ids.map(function (v, i) { return addCoedge(m, v, ids[(i + 1) % ids.length], face); });
    cs.forEach(function (c, i) { link(m, c, cs[(i + 1) % cs.length]); });
    return cs[0];
  }
  // A square face, counter-clockwise, with two square holes, clockwise.
  function start() {
    var m = newModel();
    m.faces.push({ outer: -1, holes: [] });
    m.faces[0].outer = addLoop(m, [[0, 0], [6, 0], [6, 4], [0, 4]], 0);
    m.faces[0].holes.push(addLoop(m, [[1, 1], [1, 3], [2, 3], [2, 1]], 0));
    m.faces[0].holes.push(addLoop(m, [[4, 1], [4, 3], [5, 3], [5, 1]], 0));
    return m;
  }

  // ---- geometry of a ring: straight here, in the kernel loop_signed_area
  function ringPolygon(m, c) { return ring(m, c).map(function (k) { return m.pts[m.cos[k].a]; }); }
  function signedArea(poly) {
    var s = 0;
    poly.forEach(function (p, i) { var q = poly[(i + 1) % poly.length]; s += p[0] * q[1] - q[0] * p[1]; });
    return s / 2;
  }
  function ringInside(m, c, p) {
    // Parity along +x, half-open in y, so that a shared vertex counts once.
    var inside = false;
    ring(m, c).forEach(function (k) {
      var a = m.pts[m.cos[k].a], b = m.pts[m.cos[k].b];
      if ((a[1] > p[1]) !== (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])) inside = !inside;
    });
    return inside;
  }
  function faceInside(m, f, p) {
    var face = m.faces[f];
    return ringInside(m, face.outer, p) && !face.holes.some(function (h) { return ringInside(m, h, p); });
  }
  function orient(a, b, c) { return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]); }
  function onSegment(a, b, p) {
    return orient(a, b, p) === 0 && (p[0] - a[0]) * (p[0] - b[0]) <= 0 && (p[1] - a[1]) * (p[1] - b[1]) <= 0;
  }
  // Does segment p-q touch segment a-b anywhere but at a shared end?
  function touches(p, q, a, b) {
    var same = function (u, v) { return u[0] === v[0] && u[1] === v[1]; };
    var sp = same(p, a) || same(p, b), sq = same(q, a) || same(q, b);
    if (sp && sq) return true;
    if (sp || sq) {
      var free = sp ? q : p, far = same(sp ? p : q, a) ? b : a, at = sp ? p : q;
      return orient(at, far, free) === 0 && (far[0] - at[0]) * (free[0] - at[0]) + (far[1] - at[1]) * (free[1] - at[1]) > 0;
    }
    var d1 = orient(p, q, a), d2 = orient(p, q, b), d3 = orient(a, b, p), d4 = orient(a, b, q);
    if (d1 * d2 < 0 && d3 * d4 < 0) return true;
    return onSegment(p, q, a) || onSegment(p, q, b) || onSegment(a, b, p) || onSegment(a, b, q);
  }

  // Which boundary of its face a coedge's ring is: "outer" or the hole's index.
  function boundaryOf(m, c) {
    var f = m.faces[m.cos[c].face], r = ring(m, c);
    if (r.indexOf(f.outer) >= 0) return { kind: "outer" };
    for (var i = 0; i < f.holes.length; i++) if (r.indexOf(f.holes[i]) >= 0) return { kind: "hole", index: i };
    throw new Error("a coedge on no boundary of its face");
  }
  // Every coedge of the face arriving at vertex v (coedges_ending_at).
  function arriving(m, f, v) {
    var out = [];
    m.cos.forEach(function (c, i) { if (c.face === f && c.b === v) out.push(i); });
    return out;
  }
  // choose_corner: material fills the angle counter-clockwise from the way the
  // boundary leaves to the way it arrived from.
  function angleFrom(a, b) { var t = Math.atan2(b[1], b[0]) - Math.atan2(a[1], a[0]); return t < 0 ? t + 2 * Math.PI : t; }
  function chooseCorner(m, list, v, towards) {
    if (list.length < 2) return list[0];
    var at = m.pts[v], d = [towards[0] - at[0], towards[1] - at[1]];
    for (var i = 0; i < list.length; i++) {
      var c = m.cos[list[i]];
      var from = m.pts[c.a], to = m.pts[m.cos[c.next].b];
      var out = [to[0] - at[0], to[1] - at[1]], back = [from[0] - at[0], from[1] - at[1]];
      var sweep = angleFrom(out, back);
      if (sweep === 0) sweep = 2 * Math.PI; // A spur's tip: the whole way round.
      if (angleFrom(out, d) < sweep) return list[i];
    }
    return list[0];
  }
  function reassignRing(m, c, f) { ring(m, c).forEach(function (k) { m.cos[k].face = f; }); }

  // new_face_from_ring: `r` becomes the outer loop of a new face, and the old
  // face's holes are sorted between the two by a point of each.
  function newFaceFromRing(m, f, r) {
    var nf = m.faces.length, old = m.faces[f];
    m.faces.push({ outer: r, holes: [] });
    reassignRing(m, r, nf);
    var stay = [];
    old.holes.forEach(function (h) {
      var a = m.pts[m.cos[h].a], b = m.pts[m.cos[h].b];
      var mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      // A point on the ring itself is no point inside it (face_contains: OnCoedge).
      var onIt = ring(m, r).some(function (k) { return onSegment(m.pts[m.cos[k].a], m.pts[m.cos[k].b], mid); });
      if (!onIt && ringInside(m, r, mid)) { m.faces[nf].holes.push(h); reassignRing(m, h, nf); } else stay.push(h);
    });
    old.holes = stay;
    return nf;
  }

  // Splices an edge from p to q into the face. vp and vq are vertex indices, or
  // null for a point not yet on the face. Returns what happened, or a refusal.
  function splice(m, f, vp, vq, P, Q) {
    var p = vp !== null ? m.pts[vp] : P, q = vq !== null ? m.pts[vq] : Q;
    if (p[0] === q[0] && p[1] === q[1]) return { refused: "the two ends are one point" };
    var i;
    for (i = 0; i < m.cos.length; i++) {
      var c = m.cos[i];
      if (touches(p, q, m.pts[c.a], m.pts[c.b])) {
        return { refused: "the new edge would cross or run along an edge already there (a remesh splits at every crossing before it splices)" };
      }
    }
    if (!faceInside(m, f, [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2])) return { refused: "the new edge does not lie in the face" };
    if (vp === null) vp = addPoint(m, P);
    if (vq === null) vq = addPoint(m, Q);
    var aList = arriving(m, f, vp), bList = arriving(m, f, vq);
    var a = aList.length ? chooseCorner(m, aList, vp, q) : null;
    var b = bList.length ? chooseCorner(m, bList, vq, p) : null;
    var fwd = addCoedge(m, vp, vq, f), rev = addCoedge(m, vq, vp, f);
    if (a === null && b === null) {
      link(m, fwd, rev); link(m, rev, fwd);
      m.faces[f].holes.push(fwd);
      return { kase: "neither end on the face: a new two-coedge ring floating inside it, a hole", newFace: null };
    }
    if (a !== null && b === null) {
      var an = m.cos[a].next;
      link(m, a, fwd); link(m, fwd, rev); link(m, rev, an);
      return { kase: "one end on the face: a spur, out along the edge and back, in that vertex's own loop", newFace: null };
    }
    if (a === null && b !== null) {
      var bn0 = m.cos[b].next;
      link(m, b, rev); link(m, rev, fwd); link(m, fwd, bn0);
      return { kase: "one end on the face: a spur, out along the edge and back, in that vertex's own loop", newFace: null };
    }
    var la = boundaryOf(m, a), lb = boundaryOf(m, b);
    var aNext = m.cos[a].next, bNext = m.cos[b].next;
    link(m, a, fwd); link(m, fwd, bNext); link(m, b, rev); link(m, rev, aNext);
    var face = m.faces[f];
    if (la.kind === "outer" && lb.kind === "outer") {
      // split_face: both rings bound material.
      face.outer = fwd;
      var nf = newFaceFromRing(m, f, rev);
      return { kase: "both on the outer loop: the face is split into two faces", newFace: nf };
    }
    if (la.kind === "hole" && lb.kind === "hole" && la.index === lb.index) {
      // The ring of positive area bounds material, the other stays a hole.
      var area = signedArea(ringPolygon(m, fwd));
      var material = area > 0 ? fwd : rev, voidRing = area > 0 ? rev : fwd;
      face.holes[la.index] = voidRing;
      var nf2 = newFaceFromRing(m, f, material);
      return { kase: "both on one hole: the ring of positive area is a new face, the other stays a hole", newFace: nf2 };
    }
    if (la.kind === "hole" && lb.kind === "hole") {
      face.holes[la.index] = fwd;
      face.holes.splice(lb.index, 1);
      return { kase: "on two different holes: they merge into one hole", newFace: null };
    }
    var j = la.kind === "hole" ? la.index : lb.index;
    face.outer = fwd;
    face.holes.splice(j, 1);
    return { kase: "on a hole and the outer loop: the hole is absorbed into the outer loop", newFace: null };
  }
  // The material area of a face: its rings' signed areas add up, holes negative.
  function faceArea(m, f) {
    var face = m.faces[f], s = signedArea(ringPolygon(m, face.outer));
    face.holes.forEach(function (h) { s += signedArea(ringPolygon(m, h)); });
    return s;
  }
  function totalArea(m) { var s = 0; m.faces.forEach(function (_, f) { s += faceArea(m, f); }); return s; }

  GeopBook.demo("splice-cases", function (ctx) {
    var D = GeopBook.draw;
    var m = start(), picked = null, message = "";
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 400, label: "A face in the parameter plane with its loops", draw: paint });
    var view = ctx.view(stage, { x0: -0.2, x1: 6.2, y0: -0.2, y1: 4.2 });
    var read = ctx.readout();

    function paint(g, w, h, c) {
      if (!view) return;
      view.update();
      D.grid(g, view, 1, c.grid);
      var fills = [c.fillA, c.fillB, c.fillC];
      m.faces.forEach(function (face, fi) {
        g.beginPath();
        [face.outer].concat(face.holes).forEach(function (r) {
          ringPolygon(m, r).forEach(function (p, i) {
            if (i) g.lineTo(view.x(p[0]), view.y(p[1])); else g.moveTo(view.x(p[0]), view.y(p[1]));
          });
          g.closePath();
        });
        g.fillStyle = fills[fi % fills.length];
        g.fill("evenodd");
      });
      // Each coedge shifted to its left, into its face's material.
      m.cos.forEach(function (co, i) {
        if (co.next < 0) return;
        var a = m.pts[co.a], b = m.pts[co.b], dx = b[0] - a[0], dy = b[1] - a[1], l = Math.hypot(dx, dy);
        var ux = dx / l, uy = dy / l, s = 0.07, inset = Math.min(0.2, l / 3);
        var A = [a[0] + ux * inset - uy * s, a[1] + uy * inset + ux * s], B = [b[0] - ux * inset - uy * s, b[1] - uy * inset + ux * s];
        D.polyline(g, view, [A, B], c.series[co.face % c.series.length], 1.5);
        var ang = Math.atan2(-(B[1] - A[1]), B[0] - A[0]), bx = view.x(B[0]), by = view.y(B[1]);
        g.beginPath(); g.moveTo(bx, by); g.lineTo(bx - 7 * Math.cos(ang - 0.45), by - 7 * Math.sin(ang - 0.45));
        g.moveTo(bx, by); g.lineTo(bx - 7 * Math.cos(ang + 0.45), by - 7 * Math.sin(ang + 0.45)); g.stroke();
      });
      m.pts.forEach(function (p, i) {
        var on = m.cos.some(function (co) { return co.a === i && co.next >= 0; });
        if (on) D.dot(g, view, p, picked && picked.v === i ? 7 : 4, picked && picked.v === i ? c.red : c.fg);
      });
      if (picked && picked.v === null) D.dot(g, view, picked.p, 6, c.red);
    }
    function summary() {
      var lines = m.faces.map(function (face, f) {
        return "face " + f + ": outer ring of " + ring(m, face.outer).length + " coedges, " + face.holes.length + " hole(s), area " + faceArea(m, f).toFixed(2);
      });
      read.set((message ? message + "\n" : "") + (picked ? "first end picked: pick the second\n" : "") + lines.join("\n") + "\ntotal " + totalArea(m).toFixed(2));
      stage.redraw();
    }
    stage.canvas.addEventListener("click", function (ev) {
      var r = stage.canvas.getBoundingClientRect(), sx = ev.clientX - r.left, sy = ev.clientY - r.top;
      view.update();
      var v = null, best = 22 * 22;
      m.pts.forEach(function (p, i) {
        if (!m.cos.some(function (co) { return co.a === i && co.next >= 0; })) return;
        var d = Math.pow(view.x(p[0]) - sx, 2) + Math.pow(view.y(p[1]) - sy, 2);
        if (d < best) { best = d; v = i; }
      });
      var w = view.inv(sx, sy);
      // Free points on a half-unit grid, so that everything stays exact.
      var p = v !== null ? m.pts[v] : [Math.round(w[0] * 2) / 2, Math.round(w[1] * 2) / 2];
      var pick = { v: v, p: p };
      if (!picked) { picked = pick; message = ""; summary(); return; }
      var first = picked; picked = null;
      // The face they are on: the one the new edge's midpoint is inside.
      var mid = [(first.p[0] + pick.p[0]) / 2, (first.p[1] + pick.p[1]) / 2], f = -1;
      m.faces.forEach(function (_, k) { if (f < 0 && faceInside(m, k, mid)) f = k; });
      if (f < 0) { message = "refused: the new edge does not lie in any face"; summary(); return; }
      var res = splice(m, f, first.v, pick.v, first.p, pick.p);
      message = res.refused ? "refused: " + res.refused : "case: " + res.kase;
      summary();
    });
    ctx.button("Start over", function () { m = start(); picked = null; message = ""; summary(); });
    summary();
  }, { start: start, splice: splice, ring: ring, signedArea: signedArea, ringPolygon: ringPolygon, faceArea: faceArea, totalArea: totalArea, faceInside: faceInside, addPoint: addPoint, newModel: newModel, addLoop: addLoop });
})();
