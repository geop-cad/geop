// Draws a mesh the real kernel made (see scripts/build_models.py), orbited by
// dragging. data-args='{"model":"rounded_box","example":"rounded_box"}': the
// file in assets/models, and the example it can be opened as in the web app.
(function () {
  "use strict";
  GeopBook.demo("model-viewer", function (ctx) {
    var name = ctx.args.model;
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 420, label: "The mesh of " + name + " made by the Geop kernel", draw: paint });
    var cam = ctx.camera(stage, { yaw: ctx.args.yaw === undefined ? 0.7 : ctx.args.yaw, pitch: ctx.args.pitch === undefined ? 0.5 : ctx.args.pitch, scale: 0.8, distance: 6 });
    var mesh = null, normals = null, scale = 1, status = "loading " + name + "…";
    var wire = false;

    function paint(g, w, h, c) {
      if (!mesh) { GeopBook.draw.text(g, null, [w / 2, h / 2], status, c.muted, "center"); return; }
      var pos = mesh.positions, idx = mesh.indices, n = idx.length / 3;
      var P = new Float64Array(pos.length);
      for (var i = 0; i < pos.length; i += 3) {
        var q = cam.project([pos[i] * scale, pos[i + 1] * scale, pos[i + 2] * scale]);
        P[i] = q[0]; P[i + 1] = q[1]; P[i + 2] = q[2];
      }
      var order = [];
      for (var t = 0; t < n; t++) {
        var a = idx[3 * t] * 3, b = idx[3 * t + 1] * 3, e = idx[3 * t + 2] * 3;
        // Facing the viewer: the screen's y points down, so outward is clockwise.
        var area = (P[b] - P[a]) * (P[e + 1] - P[a + 1]) - (P[b + 1] - P[a + 1]) * (P[e] - P[a]);
        if (area >= 0) continue;
        order.push([P[a + 2] + P[b + 2] + P[e + 2], t]);
      }
      order.sort(function (x, y) { return x[0] - y[0]; });
      var cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw), cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
      var base = c.dark ? [120, 170, 235] : [70, 125, 195];
      for (var k = 0; k < order.length; k++) {
        var T = order[k][1];
        var A = idx[3 * T] * 3, B = idx[3 * T + 1] * 3, C = idx[3 * T + 2] * 3;
        // A light at the viewer: the normal's component towards the camera.
        var nx = normals[3 * T], ny = normals[3 * T + 1], nz = normals[3 * T + 2];
        var y1 = sy * nx + cy * ny;
        var toward = -(cp * y1 - sp * nz);
        var shade = 0.3 + 0.7 * Math.max(0, toward);
        var col = "rgb(" + Math.round(base[0] * shade) + "," + Math.round(base[1] * shade) + "," + Math.round(base[2] * shade) + ")";
        g.beginPath(); g.moveTo(P[A], P[A + 1]); g.lineTo(P[B], P[B + 1]); g.lineTo(P[C], P[C + 1]); g.closePath();
        g.fillStyle = col; g.fill();
        g.strokeStyle = wire ? c.bg : col; g.lineWidth = wire ? 0.4 : 0.6; g.stroke();
      }
    }
    fetch((typeof path_to_root === "string" ? path_to_root : "") + "assets/models/" + name + ".json").then(function (r) {
      if (!r.ok) throw new Error(r.status);
      return r.json();
    }).then(function (m) {
      var p = m.positions, lo = [1e30, 1e30, 1e30], hi = [-1e30, -1e30, -1e30], i, a;
      for (i = 0; i < p.length; i += 3) for (a = 0; a < 3; a++) { lo[a] = Math.min(lo[a], p[i + a]); hi[a] = Math.max(hi[a], p[i + a]); }
      var size = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) || 1;
      scale = 1 / size;
      cam.centre = [(lo[0] + hi[0]) / 2 * scale, (lo[1] + hi[1]) / 2 * scale, (lo[2] + hi[2]) / 2 * scale];
      var idx = m.indices, N = new Float64Array(idx.length);
      for (var t = 0; t < idx.length / 3; t++) {
        var A = idx[3 * t] * 3, B = idx[3 * t + 1] * 3, C = idx[3 * t + 2] * 3;
        var ux = p[B] - p[A], uy = p[B + 1] - p[A + 1], uz = p[B + 2] - p[A + 2];
        var vx = p[C] - p[A], vy = p[C + 1] - p[A + 1], vz = p[C + 2] - p[A + 2];
        var nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        var l = Math.sqrt(nx * nx + ny * ny + nz * nz) || 1;
        N[3 * t] = nx / l; N[3 * t + 1] = ny / l; N[3 * t + 2] = nz / l;
      }
      mesh = m; normals = N;
      readout.set(m.triangles + " triangles, as the kernel meshed the part. Drag to turn it.");
      stage.redraw();
    }).catch(function (e) {
      // A page opened from the disk (file://) may not fetch. Say so.
      status = "This model is only shown when the book is served over http (" + e.message + ").";
      stage.redraw();
    });
    ctx.checkbox({ label: "triangles", value: false, oninput: function (v) { wire = v; stage.redraw(); } });
    var readout = ctx.readout();
    if (ctx.args.example) {
      ctx.note('<a href="https://app.geop-cad.dev/?example=' + encodeURIComponent(ctx.args.example) + '" target="_blank" rel="noopener">Open this example in the editor</a>: the real kernel, as WebAssembly, in your browser.');
    }
  });
})();
