// One bend between two flanges, seen across the bend, and the flat strip it
// comes from. BA = angle (r + K T), as Sheet::developed_length has it, in
// ops/geop-ops-sheetmetal/src/sheet.rs.
(function () {
  "use strict";

  // Bend allowance: the developed length, the length of the neutral arc.
  function bendAllowance(angle, r, T, K) { return angle * (r + K * T); }
  // Outside setback: from the bend's tangent line to where the outsides would
  // meet were the bend sharp. Inside setback likewise.
  function outsideSetback(angle, r, T) { return (r + T) * Math.tan(angle / 2); }
  function insideSetback(angle, r) { return r * Math.tan(angle / 2); }
  // Bend deduction: what the two outer-sharp lengths overstate the flat by.
  function bendDeduction(angle, r, T, K) { return 2 * outsideSetback(angle, r, T) - bendAllowance(angle, r, T, K); }
  // Flat length for flanges A, B measured to the outer sharp corner, or
  // (tangent) the flat parts only.
  function flatLength(A, B, angle, r, T, K) { return A + B - bendDeduction(angle, r, T, K); }

  // The section: a profile of the flanges and bend at offset d from the inside.
  // Flange one runs along -x to the tangent point at the origin, the bend turns
  // up by `angle` about the centre (0, r); flange two runs on from its end.
  function section(angle, r, d, lengthA, lengthB, steps) {
    var pts = [[-lengthA, -d]], R = r + d, cx = 0, cy = r;
    for (var i = 0; i <= steps; i++) {
      var a = angle * i / steps;
      pts.push([cx + R * Math.sin(a), cy - R * Math.cos(a)]);
    }
    var endX = cx + R * Math.sin(angle), endY = cy - R * Math.cos(angle);
    pts.push([endX + lengthB * Math.cos(angle), endY + lengthB * Math.sin(angle)]);
    return pts;
  }
  function polyLength(pts) {
    var s = 0;
    for (var i = 1; i < pts.length; i++) s += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
    return s;
  }

  GeopBook.demo("bend-allowance", function (ctx) {
    var D = GeopBook.draw;
    var p = { angle: Math.PI / 2, r: 1, T: 1, K: 0.44 };
    var A = 4, B = 3.2; // the flanges to the outer sharp corner
    var stage = ctx.stage({ aspect: 1.25, maxHeight: 460, label: "A bend in section, its neutral arc, and the flat strip", draw: paint });
    var view = ctx.view(stage, { x0: -5, x1: 7, y0: -5.2, y1: 6.2 }, { pad: 6 });
    var read = ctx.readout();
    function set(key) { return function (v) { p[key] = v; update(); }; }
    ctx.slider({ label: "angle", min: 0.2, max: 2.9, step: 0.01, value: p.angle, format: function (v) { return (v * 180 / Math.PI).toFixed(0) + "°"; }, oninput: set("angle") });
    ctx.slider({ label: "radius r", min: 0.1, max: 2.5, step: 0.05, value: p.r, oninput: set("r") });
    ctx.slider({ label: "thickness T", min: 0.2, max: 2, step: 0.05, value: p.T, oninput: set("T") });
    ctx.slider({ label: "K", min: 0, max: 1, step: 0.01, value: p.K, oninput: set("K") });

    function paint(g, w, h, c) {
      if (!view || !read) return;
      view.update();
      var ossb = outsideSetback(p.angle, p.r, p.T);
      var flatA = Math.max(A - ossb, 0.01), flatB = Math.max(B - ossb, 0.01);
      var inner = section(p.angle, p.r, 0, flatA, flatB, 48);
      var outer = section(p.angle, p.r, p.T, flatA, flatB, 48);
      var neutral = section(p.angle, p.r, p.K * p.T, flatA, flatB, 48);
      var shift = 1.2; // room for the flat strip beneath
      function up(q) { return [q[0], q[1] + shift]; }
      var band = inner.concat(outer.slice().reverse()).map(up);
      D.polygon(g, view, band, c.fillA, c.blue, 1.5);
      D.polyline(g, view, neutral.map(up), c.orange, 2);
      // The bend's region: the two radii at its ends.
      var ends = [[[0, p.r], [0, -p.T]], [[0, p.r], [(p.r + p.T) * Math.sin(p.angle), p.r - (p.r + p.T) * Math.cos(p.angle)]]];
      ends.forEach(function (e) { D.polyline(g, view, e.map(up), c.muted, 1, [3, 3]); });
      D.dot(g, view, up([0, p.r]), 3, c.muted);

      // The flat strip, to the same scale.
      var ba = bendAllowance(p.angle, p.r, p.T, p.K);
      var y0 = -5, y1 = -5 + p.T;
      var x0 = -flatA, x1 = 0, x2 = ba, x3 = ba + flatB;
      var cx = (x3 + x0) / 2 - 1; // centred under the section
      function at(x, y) { return [x - cx, y]; }
      D.polygon(g, view, [at(x0, y0), at(x3, y0), at(x3, y1), at(x0, y1)], c.fillA, c.blue, 1.5);
      D.polygon(g, view, [at(x1, y0), at(x2, y0), at(x2, y1), at(x1, y1)], c.fillB, c.orange, 1.5);
      D.polyline(g, view, [at(x0, y0 + p.K * p.T), at(x3, y0 + p.K * p.T)], c.orange, 1.5);
      D.text(g, view, at((x1 + x2) / 2, y1 + 0.2), "BA", c.orange, "center");
    }
    function update() {
      var ba = bendAllowance(p.angle, p.r, p.T, p.K), bd = bendDeduction(p.angle, p.r, p.T, p.K);
      read.set("neutral radius r + K T = " + (p.r + p.K * p.T).toFixed(3) +
        "\nbend allowance BA = " + ba.toFixed(3) +
        "\noutside setback = " + outsideSetback(p.angle, p.r, p.T).toFixed(3) +
        "   bend deduction = " + bd.toFixed(3) +
        "\nflanges " + A + " and " + B + " to the outer sharp corner: flat length " + flatLength(A, B, p.angle, p.r, p.T, p.K).toFixed(3));
      stage.redraw();
    }
    update();
  }, { bendAllowance: bendAllowance, outsideSetback: outsideSetback, insideSetback: insideSetback, bendDeduction: bendDeduction, flatLength: flatLength, section: section, polyLength: polyLength });
})();
