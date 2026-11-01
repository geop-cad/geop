// Catmull-Clark subdivision with sharp creases, by the rules of
// ops/geop-ops-subd/src/subdivide.rs, and one step followed by Loop and
// Schaefer's bicubic patches, as ops/geop-ops-subd/src/limit.rs builds them.
// A mesh is { v: [[x,y,z],...], f: [[i,j,k,l],...], sharp: Set of "a-b" }, closed.
(function () {
  "use strict";

  function key(a, b) { return a < b ? a + "-" + b : b + "-" + a; }
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  function weighted(terms) {
    var s = [0, 0, 0];
    terms.forEach(function (t) { s = add(s, scale(t[1], t[0])); });
    return s;
  }

  // Each vertex's corners, ordered round it: a corner [f, k] has v = f[k]; the next
  // one round is across the edge to the corner's previous vertex (as Topology has it).
  function fans(mesh) {
    var directed = {}, n = mesh.v.length, out = [];
    mesh.f.forEach(function (face, f) {
      face.forEach(function (v, k) { directed[v + ">" + face[(k + 1) % face.length]] = [f, k]; });
    });
    var firstCorner = {};
    mesh.f.forEach(function (face, f) { face.forEach(function (v, k) { if (!(v in firstCorner)) firstCorner[v] = [f, k]; }); });
    for (var v = 0; v < n; v++) {
      var fan = [], c = firstCorner[v];
      if (!c) { out.push(fan); continue; }
      var start = c;
      do {
        fan.push(c);
        var face = mesh.f[c[0]], prev = face[(c[1] + face.length - 1) % face.length];
        c = directed[v + ">" + prev];
        if (!c) throw new Error("the mesh is not closed at vertex " + v);
      } while (c[0] !== start[0] || c[1] !== start[1]);
      out.push(fan);
    }
    return out;
  }
  function nextOf(mesh, c) { var face = mesh.f[c[0]]; return face[(c[1] + 1) % face.length]; }

  // Smooth: no sharp edge or one; crease: two; corner: three or more.
  function vertexKind(mesh, fan, v) {
    var ends = [];
    fan.forEach(function (c) { var w = nextOf(mesh, c); if (mesh.sharp.has(key(v, w))) ends.push(w); });
    if (ends.length < 2) return { kind: "smooth" };
    if (ends.length === 2) return { kind: "crease", a: ends[0], b: ends[1] };
    return { kind: "corner" };
  }

  function subdivide(mesh) {
    var p = mesh.v, faces = mesh.f, allFans = fans(mesh);
    var facePoints = faces.map(function (face) {
      var s = [0, 0, 0];
      face.forEach(function (i) { s = add(s, p[i]); });
      return scale(s, 1 / face.length);
    });
    var v = p.map(function (pv, i) {
      var fan = allFans[i], k = vertexKind(mesh, fan, i);
      if (k.kind === "corner") return pv.slice();
      if (k.kind === "crease") return scale(weighted([[1, p[k.a]], [6, pv], [1, p[k.b]]]), 1 / 8);
      var n = fan.length, terms = [[n * (n - 2), pv]];
      fan.forEach(function (c) { terms.push([1, facePoints[c[0]]]); terms.push([1, p[nextOf(mesh, c)]]); });
      return scale(weighted(terms), 1 / (n * n));
    });
    var pos = v.slice(), facePoint = facePoints.map(function (q) { pos.push(q); return pos.length - 1; });
    var edgeFaces = {};
    faces.forEach(function (face, f) {
      face.forEach(function (a, k) {
        var e = key(a, face[(k + 1) % face.length]);
        (edgeFaces[e] = edgeFaces[e] || []).push(f);
      });
    });
    var edgePoint = {};
    Object.keys(edgeFaces).forEach(function (e) {
      var ab = e.split("-").map(Number), a = ab[0], b = ab[1];
      var q = mesh.sharp.has(e)
        ? scale(add(p[a], p[b]), 0.5)
        : scale(weighted([[1, p[a]], [1, p[b]], [1, facePoints[edgeFaces[e][0]]], [1, facePoints[edgeFaces[e][1]]]]), 1 / 4);
      edgePoint[e] = pos.length;
      pos.push(q);
    });
    var quads = [];
    faces.forEach(function (face, f) {
      var n = face.length;
      face.forEach(function (w, k) {
        var after = face[(k + 1) % n], before = face[(k + n - 1) % n];
        quads.push([w, edgePoint[key(w, after)], facePoint[f], edgePoint[key(before, w)]]);
      });
    });
    var sharp = new Set();
    mesh.sharp.forEach(function (e) {
      var ab = e.split("-").map(Number), m = edgePoint[e];
      sharp.add(key(ab[0], m)); sharp.add(key(ab[1], m));
    });
    return { v: pos, f: quads, sharp: sharp, cageVertices: p.length };
  }

  // The normal of a smooth vertex's limit tangent plane, by the eigenvector masks.
  function tangentNormal(mesh, fan, i) {
    var n = fan.length, p = mesh.v, a = 1 + Math.cos(2 * Math.PI / n) + Math.cos(Math.PI / n) * Math.sqrt(2 * (9 + Math.cos(2 * Math.PI / n)));
    function tangent(trig) {
      var s = [0, 0, 0];
      fan.forEach(function (c, j) {
        var e = sub(p[nextOf(mesh, c)], p[i]), d = sub(p[mesh.f[c[0]][(c[1] + 2) % 4]], p[i]);
        var t0 = trig(2 * Math.PI * j / n), t1 = trig(2 * Math.PI * (j + 1) / n);
        s = add(add(s, scale(e, a * t0)), scale(d, t0 + t1));
      });
      return s;
    }
    return cross(tangent(Math.cos), tangent(Math.sin));
  }

  // The shared control points: limit.rs's control_net, for a closed quad mesh.
  function controlNet(mesh) {
    var p = mesh.v, faces = mesh.f, allFans = fans(mesh);
    var kinds = p.map(function (_, i) { return vertexKind(mesh, allFans[i], i); });
    // How many faces the sector of each corner has, between two sharp edges.
    var weight = faces.map(function () { return [0, 0, 0, 0]; });
    p.forEach(function (_, v) {
      var fan = allFans[v], m = fan.length, kind = kinds[v].kind;
      if (kind === "smooth") { fan.forEach(function (c) { weight[c[0]][c[1]] = m; }); return; }
      function cut(i) { var c = fan[i], face = faces[c[0]]; return mesh.sharp.has(key(v, face[(c[1] + 3) % 4])); }
      var start = 0, i;
      for (i = 0; i < m; i++) if (cut(i)) { start = (i + 1) % m; break; }
      var run = [];
      for (var step = 0; step < m; step++) {
        i = (start + step) % m;
        run.push(i);
        if (step + 1 === m || cut(i)) {
          run.forEach(function (j) {
            weight[fan[j][0]][fan[j][1]] = (kind === "crease" ? 2 : 4) * run.length;
          });
          run = [];
        }
      }
    });
    var interior = faces.map(function (face, f) {
      return [0, 1, 2, 3].map(function (k) {
        var n = weight[f][k];
        return scale(weighted([[n, p[face[k]]], [2, p[face[(k + 1) % 4]]], [2, p[face[(k + 3) % 4]]], [1, p[face[(k + 2) % 4]]]]), 1 / (n + 5));
      });
    });
    var corner = p.map(function (pv, v) {
      var k = kinds[v];
      if (k.kind === "corner") return pv.slice();
      if (k.kind === "crease") return scale(weighted([[1, p[k.a]], [4, pv], [1, p[k.b]]]), 1 / 6);
      var fan = allFans[v], n = fan.length, terms = [[n * n, pv]];
      fan.forEach(function (c) { terms.push([4, p[nextOf(mesh, c)]]); terms.push([1, p[faces[c[0]][(c[1] + 2) % 4]]]); });
      return scale(weighted(terms), 1 / (n * (n + 5)));
    });
    var directed = {};
    faces.forEach(function (face, f) { face.forEach(function (a, k) { directed[a + ">" + face[(k + 1) % 4]] = [f, k]; }); });
    var edge = {}, normals = {};
    Object.keys(directed).forEach(function (d) {
      var ab = d.split(">").map(Number), from = ab[0], to = ab[1];
      var sharp = mesh.sharp.has(key(from, to));
      var q;
      if (sharp && kinds[from].kind !== "smooth") {
        q = scale(weighted([[2, p[from]], [1, p[to]]]), 1 / 3);
      } else {
        var c1 = directed[from + ">" + to], c2 = directed[to + ">" + from];
        // The face's interior point nearest `from`.
        var at = function (c) {
          return faces[c[0]][c[1]] === from ? interior[c[0]][c[1]] : interior[c[0]][(c[1] + 1) % 4];
        };
        var average = scale(add(at(c1), at(c2)), 0.5);
        var n = allFans[from].length;
        if (kinds[from].kind === "smooth" && n !== 4) {
          var normal = normals[from] || (normals[from] = tangentNormal(mesh, allFans[from], from));
          var off = sub(average, corner[from]);
          q = sub(average, scale(normal, dot(off, normal) / dot(normal, normal)));
        } else q = average;
      }
      edge[d] = q;
    });
    return { corner: corner, edge: edge, interior: interior };
  }
  // ControlNet::patch.
  function patch(net, mesh, f) {
    var q = mesh.f[f];
    function l(i) { return net.corner[q[i]]; }
    function e(a, b) { return net.edge[q[a] + ">" + q[b]]; }
    function n(i) { return net.interior[f][i]; }
    return [
      [l(0), e(0, 3), e(3, 0), l(3)],
      [e(0, 1), n(0), n(3), e(3, 2)],
      [e(1, 0), n(1), n(2), e(2, 3)],
      [l(1), e(1, 2), e(2, 1), l(2)]
    ];
  }
  function bez(c, t) {
    var a = c.map(function (x) { return x.slice(); });
    for (var r = 3; r > 0; r--) for (var i = 0; i < r; i++) a[i] = [a[i][0] * (1 - t) + a[i + 1][0] * t, a[i][1] * (1 - t) + a[i + 1][1] * t, a[i][2] * (1 - t) + a[i + 1][2] * t];
    return a[0];
  }
  function evalPatch(P, u, v) {
    return bez(P.map(function (row) { return bez(row, v); }), u);
  }
  // Each patch sampled into a grid of quads, for drawing.
  function patchesToMesh(mesh, steps) {
    var net = controlNet(mesh), v = [], f = [];
    mesh.f.forEach(function (_, face) {
      var P = patch(net, mesh, face), base = v.length, i, j;
      for (i = 0; i <= steps; i++) for (j = 0; j <= steps; j++) v.push(evalPatch(P, i / steps, j / steps));
      for (i = 0; i < steps; i++) for (j = 0; j < steps; j++) {
        var a = base + i * (steps + 1) + j;
        // First side runs along i from corner 0 to 1; last from 0 to 3 along j: same turn as the quad.
        f.push([a, a + steps + 1, a + steps + 2, a + 1]);
      }
    });
    return { v: v, f: f };
  }

  // Cages. Faces counter-clockwise from outside.
  function cube() {
    var v = [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]];
    var f = [[3, 2, 1, 0], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
    return { v: v, f: f, sharp: new Set(), creaseCandidates: [[4, 5], [5, 6], [6, 7], [7, 4]] };
  }
  // An extruded L, its outline of seven corners (one straight, at 6),
  function lShape() {
    var ring = [[-1.5, -1.5], [1.5, -1.5], [1.5, -0.3], [-0.3, -0.3], [-0.3, 1.5], [-1.5, 1.5]];
    var extra = [[-1.5, -0.3]]; // on the outline, for the split of the caps
    var v = [];
    var outline = [ring[0], ring[1], ring[2], ring[3], ring[4], ring[5], extra[0]];
    outline.forEach(function (p) { v.push([p[0], p[1], -0.8]); });
    outline.forEach(function (p) { v.push([p[0], p[1], 0.8]); });
    var n = outline.length, f = [];
    for (var i = 0; i < n; i++) { var j = (i + 1) % n; f.push([i, j, j + n, i + n]); }
    // Each cap a quad and a pentagon, split along 6-3.
    f.push([6, 3, 4, 5].reverse());
    f.push([0, 6, 3, 2, 1]);
    f.push([6 + n, 5 + n, 4 + n, 3 + n].reverse());
    f.push([0 + n, 1 + n, 2 + n, 3 + n, 6 + n]);
    return { v: v, f: f, sharp: new Set(), creaseCandidates: [[n, n + 1], [n + 1, n + 2], [n + 2, n + 3], [n + 3, n + 4], [n + 4, n + 5], [n + 5, n + 6], [n + 6, n]] };
  }
  // A closed regular grid of quads, on a torus, for the test.
  function torus(nu, nv, at) {
    var v = [], f = [];
    for (var i = 0; i < nu; i++) for (var j = 0; j < nv; j++) v.push(at(i, j));
    function id(i, j) { return ((i + nu) % nu) * nv + ((j + nv) % nv); }
    for (i = 0; i < nu; i++) for (j = 0; j < nv; j++) f.push([id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)]);
    return { v: v, f: f, sharp: new Set() };
  }
  function withCreases(cage, on) {
    var sharp = new Set();
    if (on) cage.creaseCandidates.forEach(function (e) { sharp.add(key(e[0], e[1])); });
    return { v: cage.v, f: cage.f, sharp: sharp, creaseCandidates: cage.creaseCandidates };
  }

  GeopBook.demo("catmull-clark", function (ctx) {
    var D = GeopBook.draw;
    var level = 2, mode = "cc", cageName = "cube", creased = false, showCage = true;
    var stage = ctx.stage({ aspect: 1.25, maxHeight: 440, label: "A control cage and its subdivision surface, orbited by dragging", draw: paint });
    var cam = ctx.camera(stage, { yaw: 0.6, pitch: 0.55, scale: 0.26, distance: 9 });
    var read = ctx.readout();
    var shown = null, cage = null;

    function rebuild() {
      cage = withCreases(cageName === "cube" ? cube() : lShape(), creased);
      var m = cage;
      if (mode === "patches") {
        var once = subdivide(cage);
        shown = patchesToMesh(once, 8);
        var regular = 0, all = fans(once);
        all.forEach(function (fan) { if (fan.length !== 4) regular++; });
        var nCage = cage.f.filter(function (x) { return x.length === 4; }).length;
        var nOther = cage.f.length - nCage;
        read.set(once.f.length + " bicubic Bézier patches (sampled here at 8 x 8), from one step.\n" +
          "As a B-rep: " + nCage + " quad faces, each one bicubic B-spline of 7 x 7 control points" +
          (nOther ? ", and " + (once.f.length - 4 * nCage) + " faces from " + nOther + " n-gons" : "") + ".\n" +
          regular + " extraordinary vertices (not of four faces): near them the patches only approximate the limit.");
      } else {
        for (var i = 0; i < level; i++) m = subdivide(m);
        shown = m;
        read.set("level " + level + ": " + m.v.length + " vertices, " + m.f.length + " faces" + (level === 0 && cage.f.some(function (x) { return x.length !== 4; }) ? " (not all quads yet)" : ""));
      }
      stage.redraw();
    }
    function paint(g, w, h, c) {
      if (!shown) return;
      var P = shown.v.map(cam.project), order = [];
      var cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw), cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
      var base = c.dark ? [120, 170, 235] : [90, 145, 210];
      shown.f.forEach(function (face) {
        // Newell's normal in the world, and whether it faces the eye on the paper.
        var n = [0, 0, 0], area = 0, depth = 0;
        for (var i = 0; i < face.length; i++) {
          var a = shown.v[face[i]], b = shown.v[face[(i + 1) % face.length]];
          n = add(n, cross(a, b));
          var A = P[face[i]], B = P[face[(i + 1) % face.length]];
          area += A[0] * B[1] - B[0] * A[1];
          depth += A[2];
        }
        if (area >= 0) return;
        var len = Math.sqrt(dot(n, n)) || 1;
        var y1 = (sy * n[0] + cy * n[1]) / len;
        var toward = -(cp * y1 - sp * n[2] / len);
        order.push([depth / face.length, face, 0.3 + 0.7 * Math.max(0, toward)]);
      });
      order.sort(function (x, y) { return x[0] - y[0]; });
      order.forEach(function (o) {
        var col = "rgb(" + Math.round(base[0] * o[2]) + "," + Math.round(base[1] * o[2]) + "," + Math.round(base[2] * o[2]) + ")";
        var pts = o[1].map(function (i) { return P[i]; });
        D.polygon(g, null, pts, col, col, 0.6);
        if (mode !== "patches" && level <= 2) D.polygon(g, null, pts, null, c.faint, 0.5);
      });
      if (showCage && cage) {
        var q = cage.v.map(cam.project), seen = {};
        cage.f.forEach(function (face) {
          face.forEach(function (a, k) {
            var b = face[(k + 1) % face.length], e = key(a, b);
            if (seen[e]) return; seen[e] = 1;
            var sharp = cage.sharp.has(e);
            D.polyline(g, null, [q[a], q[b]], sharp ? c.red : c.orange, sharp ? 2 : 1.2, sharp ? [] : [4, 3]);
          });
        });
        q.forEach(function (x) { D.dot(g, null, x, 2.5, c.orange); });
      }
    }
    ctx.select({ label: "cage", value: cageName, options: [["cube", "box"], ["l", "L block"]], oninput: function (v) { cageName = v; rebuild(); } });
    ctx.select({
      label: "show", value: mode, options: [["cc", "Catmull-Clark level"], ["patches", "one step, then patches"]],
      oninput: function (v) { mode = v; rebuild(); }
    });
    ctx.slider({ label: "level", min: 0, max: 4, step: 1, value: level, format: function (v) { return String(v); }, oninput: function (v) { level = v; rebuild(); } });
    ctx.checkbox({ label: "crease the top rim", value: false, oninput: function (v) { creased = v; rebuild(); } });
    ctx.checkbox({ label: "cage", value: true, oninput: function (v) { showCage = v; stage.redraw(); } });
    rebuild();
  }, { subdivide: subdivide, controlNet: controlNet, patch: patch, evalPatch: evalPatch, torus: torus, cube: cube, lShape: lShape, withCreases: withCreases, fans: fans, patchesToMesh: patchesToMesh, key: key });
})();
