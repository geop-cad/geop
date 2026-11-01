// Walking a real half-edge structure: vertices, edges, coedges with next, prev
// and sense, faces with an outer loop and holes. The structure is built from a
// list of faces, each a loop of vertex indices, the way a BodySpec describes a
// body: the opposite coedge is not stored, it is found through the edge.
(function () {
  "use strict";

  // points: [x,y,z][]; faces: {name, outer:[vertex...], holes:[[vertex...]...]}.
  function build(points, faces) {
    var edges = [], coedges = [], outFaces = [], byKey = {};
    function addLoop(loop, face) {
      var ids = [];
      for (var i = 0; i < loop.length; i++) {
        var a = loop[i], b = loop[(i + 1) % loop.length];
        var key = Math.min(a, b) + "-" + Math.max(a, b), e = byKey[key];
        var sense;
        if (e === undefined) {
          e = edges.length;
          byKey[key] = e;
          edges.push({ start: a, end: b });
          sense = "Forward";
        } else {
          sense = edges[e].start === a ? "Forward" : "Reversed";
        }
        ids.push(coedges.length);
        coedges.push({ edge: e, sense: sense, next: -1, prev: -1, face: face });
      }
      for (var k = 0; k < ids.length; k++) {
        coedges[ids[k]].next = ids[(k + 1) % ids.length];
        coedges[ids[k]].prev = ids[(k + ids.length - 1) % ids.length];
      }
      return ids[0];
    }
    faces.forEach(function (f, fi) {
      var outer = addLoop(f.outer, fi);
      var holes = (f.holes || []).map(function (h) { return addLoop(h, fi); });
      outFaces.push({ name: f.name, outer: outer, holes: holes });
    });
    return { points: points, edges: edges, coedges: coedges, faces: outFaces };
  }
  function startVertex(m, c) { var co = m.coedges[c], e = m.edges[co.edge]; return co.sense === "Forward" ? e.start : e.end; }
  function endVertex(m, c) { var co = m.coedges[c], e = m.edges[co.edge]; return co.sense === "Forward" ? e.end : e.start; }
  // The kernel keeps no opposite pointer: coedges_of_edge gives both.
  function coedgesOfEdge(m, e) {
    var out = [];
    m.coedges.forEach(function (c, i) { if (c.edge === e) out.push(i); });
    return out;
  }
  function opposite(m, c) {
    var both = coedgesOfEdge(m, m.coedges[c].edge);
    return both[0] === c ? both[1] : both[0];
  }
  function loopOf(m, c) {
    var out = [c], n = m.coedges[c].next;
    while (n !== c) { out.push(n); n = m.coedges[n].next; if (out.length > m.coedges.length) throw new Error("loop does not close"); }
    return out;
  }
  // V - E + F - L: 2(S - G) for S shells of total genus G.
  function euler(m) {
    var L = 0;
    m.faces.forEach(function (f) { L += f.holes.length; });
    return { V: m.points.length, E: m.edges.length, F: m.faces.length, L: L, chi: m.points.length - m.edges.length + m.faces.length - L };
  }
  function faceNormal(m, f) {
    var loop = loopOf(m, m.faces[f].outer), n = [0, 0, 0];
    // Newell's formula: the normal from the loop, right-handed about its direction.
    loop.forEach(function (c) {
      var a = m.points[startVertex(m, c)], b = m.points[endVertex(m, c)];
      n[0] += (a[1] - b[1]) * (a[2] + b[2]);
      n[1] += (a[2] - b[2]) * (a[0] + b[0]);
      n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    });
    var l = Math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) || 1;
    return [n[0] / l, n[1] / l, n[2] / l];
  }

  function cube() {
    var p = [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]];
    return build(p, [
      { name: "bottom", outer: [0, 3, 2, 1] }, { name: "top", outer: [4, 5, 6, 7] },
      { name: "front", outer: [0, 1, 5, 4] }, { name: "right", outer: [1, 2, 6, 5] },
      { name: "back", outer: [2, 3, 7, 6] }, { name: "left", outer: [3, 0, 4, 7] }
    ]);
  }
  function prism() {
    var p = [[-1, -1, -1], [1, -1, -1], [0, 1, -1], [-1, -1, 1], [1, -1, 1], [0, 1, 1]];
    return build(p, [
      { name: "bottom", outer: [0, 2, 1] }, { name: "top", outer: [3, 4, 5] },
      { name: "side a", outer: [0, 1, 4, 3] }, { name: "side b", outer: [1, 2, 5, 4] }, { name: "side c", outer: [2, 0, 3, 5] }
    ]);
  }
  // A block with a square hole right through: two faces with a hole loop, genus 1.
  function drilled() {
    var a = 1, b = 0.45, p = [];
    [-1, 1].forEach(function (z) {
      [[-a, -a], [a, -a], [a, a], [-a, a], [-b, -b], [b, -b], [b, b], [-b, b]].forEach(function (q) { p.push([q[0], q[1], z]); });
    });
    // Bottom 0-7, top 8-15; outer corners first, then the hole's.
    return build(p, [
      { name: "bottom", outer: [0, 3, 2, 1], holes: [[4, 5, 6, 7]] },
      { name: "top", outer: [8, 9, 10, 11], holes: [[12, 15, 14, 13]] },
      { name: "front", outer: [0, 1, 9, 8] }, { name: "right", outer: [1, 2, 10, 9] },
      { name: "back", outer: [2, 3, 11, 10] }, { name: "left", outer: [3, 0, 8, 11] },
      { name: "bore front", outer: [5, 4, 12, 13] }, { name: "bore right", outer: [6, 5, 13, 14] },
      { name: "bore back", outer: [7, 6, 14, 15] }, { name: "bore left", outer: [4, 7, 15, 12] }
    ]);
  }
  var BODIES = { cube: cube, prism: prism, drilled: drilled };

  GeopBook.demo("brep-walk", function (ctx) {
    var D = GeopBook.draw;
    var m = cube(), cur = m.faces[1].outer, trail = [];
    var stage = ctx.stage({ aspect: 1.45, maxHeight: 400, label: "A solid with its coedges drawn as arrows inside each face", draw: paint });
    var cam = ctx.camera(stage, { yaw: 0.6, pitch: 0.55, scale: 0.27, distance: 9 });
    var read = ctx.readout();

    function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
    // A coedge's arrow, shifted to its left: into the material of its face.
    function arrow(c) {
      var a = m.points[startVertex(m, c)], b = m.points[endVertex(m, c)];
      var n = faceNormal(m, m.coedges[c].face);
      var d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
      var l = Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
      d = [d[0] / l, d[1] / l, d[2] / l];
      var left = cross(n, d), inset = 0.11, trim = 0.16;
      function at(s) {
        return [a[0] + d[0] * s + left[0] * inset, a[1] + d[1] * s + left[1] * inset, a[2] + d[2] * s + left[2] * inset];
      }
      return [at(trim), at(l - trim)];
    }
    function front(f) {
      // Signed area of the projected outer loop; y points down on screen.
      var loop = loopOf(m, m.faces[f].outer), s = 0, pts = loop.map(function (c) { return cam.project(m.points[startVertex(m, c)]); });
      pts.forEach(function (p, i) { var q = pts[(i + 1) % pts.length]; s += p[0] * q[1] - q[0] * p[1]; });
      return s < 0;
    }
    function paint(g, w, h, c) {
      if (!cam) return;
      var order = m.faces.map(function (f, i) { return i; }).sort(function (x, y) { return (front(x) ? 1 : 0) - (front(y) ? 1 : 0); });
      var curFace = m.coedges[cur].face;
      order.forEach(function (fi) {
        var f = m.faces[fi], isFront = front(fi);
        var rings = [f.outer].concat(f.holes);
        g.beginPath();
        rings.forEach(function (r) {
          loopOf(m, r).forEach(function (co, i) {
            var p = cam.project(m.points[startVertex(m, co)]);
            if (i) g.lineTo(p[0], p[1]); else g.moveTo(p[0], p[1]);
          });
          g.closePath();
        });
        g.fillStyle = fi === curFace ? c.fillB : (isFront ? c.fillA : "rgba(0,0,0,0)");
        g.fill("evenodd");
        g.strokeStyle = isFront ? c.fg : c.faint; g.lineWidth = 1; g.stroke();
      });
      order.forEach(function (fi) {
        var isFront = front(fi);
        var f = m.faces[fi];
        [f.outer].concat(f.holes).forEach(function (r) {
          loopOf(m, r).forEach(function (co) {
            var seg = arrow(co), a = cam.project(seg[0]), b = cam.project(seg[1]);
            var active = co === cur, color = active ? c.red : (trail.indexOf(co) >= 0 ? c.orange : (isFront ? c.muted : c.faint));
            if (!isFront && !active) return;
            g.strokeStyle = color; g.lineWidth = active ? 3 : 1.3;
            g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
            var ang = Math.atan2(b[1] - a[1], b[0] - a[0]), s = active ? 9 : 6;
            g.beginPath(); g.moveTo(b[0], b[1]);
            g.lineTo(b[0] - s * Math.cos(ang - 0.45), b[1] - s * Math.sin(ang - 0.45));
            g.moveTo(b[0], b[1]);
            g.lineTo(b[0] - s * Math.cos(ang + 0.45), b[1] - s * Math.sin(ang + 0.45));
            g.stroke();
          });
        });
      });
      m.points.forEach(function (p, i) {
        var q = cam.project(p);
        D.dot(g, null, q, 2.5, c.fg);
        if (w > 420) D.text(g, null, [q[0] + 5, q[1] - 4], "V" + i, c.muted);
      });
    }
    function update() {
      var co = m.coedges[cur], e = euler(m), loop = loopOf(m, cur);
      var f = m.faces[co.face];
      var which = f.outer !== undefined && loopOf(m, f.outer).indexOf(cur) >= 0 ? "outer" : "a hole";
      read.set("coedge C" + cur + ": " + co.sense + " on edge E" + co.edge + " (V" + m.edges[co.edge].start + " to V" + m.edges[co.edge].end + "), runs V" + startVertex(m, cur) + " to V" + endVertex(m, cur) +
        "\nface: " + f.name + ", on its " + which + " loop of " + loop.length + " coedges" +
        "\nnext C" + co.next + ", prev C" + co.prev + ", other coedge of the edge C" + opposite(m, cur) + " on " + m.faces[m.coedges[opposite(m, cur)].face].name +
        "\nV - E + F - L = " + e.V + " - " + e.E + " + " + e.F + " - " + e.L + " = " + e.chi + (e.chi === 2 ? "  (one shell, genus 0)" : "  (one shell, genus " + (2 - e.chi) / 2 + ")"));
      stage.redraw();
    }
    function go(c) { trail.push(cur); if (trail.length > 8) trail.shift(); cur = c; update(); }
    ctx.button("next", function () { go(m.coedges[cur].next); });
    ctx.button("prev", function () { go(m.coedges[cur].prev); });
    ctx.button("other side", function () { go(opposite(m, cur)); });
    ctx.select({
      label: "body", value: "cube",
      options: [["cube", "cube"], ["prism", "prism"], ["drilled", "block with a through hole"]],
      oninput: function (v) { m = BODIES[v](); cur = m.faces[1].outer; trail = []; update(); }
    });
    update();
  }, { build: build, cube: cube, prism: prism, drilled: drilled, startVertex: startVertex, endVertex: endVertex, opposite: opposite, loopOf: loopOf, euler: euler, faceNormal: faceNormal, coedgesOfEdge: coedgesOfEdge });
})();
