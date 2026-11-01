// Which way a loop runs: the area by Green's theorem, integrated exactly from the
// spline by Gauss-Legendre, against a polygon of eight samples per pcurve, which is
// what the kernel used to do. Everything is a cubic spline in (u, v) here.
(function () {
  "use strict";

  // Three-point Gauss-Legendre on [0, 1]: exact to degree 5, which is the degree
  // of u v' - v u' on one cubic span.
  var GL = [[0.5 - Math.sqrt(0.6) / 2, 5 / 18], [0.5, 8 / 18], [0.5 + Math.sqrt(0.6) / 2, 5 / 18]];

  function point(s, t) {
    var u = 1 - t;
    return [
      u * u * u * s[0][0] + 3 * u * u * t * s[1][0] + 3 * u * t * t * s[2][0] + t * t * t * s[3][0],
      u * u * u * s[0][1] + 3 * u * u * t * s[1][1] + 3 * u * t * t * s[2][1] + t * t * t * s[3][1]
    ];
  }
  function tangent(s, t) {
    var u = 1 - t, a = 3 * u * u, b = 6 * u * t, c = 3 * t * t;
    return [
      a * (s[1][0] - s[0][0]) + b * (s[2][0] - s[1][0]) + c * (s[3][0] - s[2][0]),
      a * (s[1][1] - s[0][1]) + b * (s[2][1] - s[1][1]) + c * (s[3][1] - s[2][1])
    ];
  }
  // A loop is a list of pcurves, a pcurve a list of cubic spans.
  function greenArea(loop, rule) {
    var r = rule || GL, sum = 0;
    loop.forEach(function (pc) {
      pc.forEach(function (s) {
        r.forEach(function (g) {
          var p = point(s, g[0]), d = tangent(s, g[0]);
          sum += g[1] * (p[0] * d[1] - p[1] * d[0]);
        });
      });
    });
    return sum / 2;
  }
  // n points along each pcurve's parameter, its end left out as the next starts there.
  function samplePolygon(loop, n) {
    var poly = [];
    loop.forEach(function (pc) {
      for (var i = 0; i < n; i++) {
        var x = i / n * pc.length, k = Math.min(Math.floor(x), pc.length - 1);
        poly.push(point(pc[k], x - k));
      }
    });
    return poly;
  }
  function polygonArea(poly) {
    var s = 0;
    poly.forEach(function (p, i) { var q = poly[(i + 1) % poly.length]; s += p[0] * q[1] - q[0] * p[1]; });
    return s / 2;
  }
  function lineSpan(a, b) {
    return [a, [a[0] + (b[0] - a[0]) / 3, a[1] + (b[1] - a[1]) / 3], [a[0] + 2 * (b[0] - a[0]) / 3, a[1] + 2 * (b[1] - a[1]) / 3], b];
  }
  // A cubic Hermite span of y(x) between x0 and x1, x linear.
  function hermite(x0, y0, m0, x1, y1, m1) {
    var h = x1 - x0;
    return [[x0, y0], [x0 + h / 3, y0 + m0 * h / 3], [x0 + 2 * h / 3, y1 - m1 * h / 3], [x1, y1]];
  }
  // A thin sliver: straight along y = 0, and back along a pcurve of `periods`
  // wiggles, y(x) = a + b cos(2 pi periods x) in the middle and pinned to 0 at its
  // ends. Sampled once per period it reads a + b, while the true mean is a. A fit
  // of many spans looked at in few samples: that is how the sign came out wrong.
  function sliver(a, b, periods) {
    var f = function (x) { return a + b * Math.cos(2 * Math.PI * periods * x); };
    var df = function (x) { return -2 * Math.PI * periods * b * Math.sin(2 * Math.PI * periods * x); };
    var spans = [], per = 8 * periods, x1 = 1;
    // From (1, 0) back to (0, 0), pinned to 0 at the two ends.
    for (var i = 0; i < per; i++) {
      var hi = 1 - i / per, lo = 1 - (i + 1) / per;
      var yhi = hi >= x1 ? 0 : f(hi), mhi = hi >= x1 ? 0 : df(hi);
      var ylo = lo <= 0 ? 0 : f(lo), mlo = lo <= 0 ? 0 : df(lo);
      spans.push(hermite(hi, yhi, mhi, lo, ylo, mlo));
    }
    return [[lineSpan([0, 0], [1, 0])], spans];
  }
  function blob() {
    // Two pcurves of one cubic span each: draggable.
    return [[[[0.1, 0.3], [0.35, 0.05], [0.7, 0.05], [0.9, 0.3]]], [[[0.9, 0.3], [0.8, 0.75], [0.3, 0.8], [0.1, 0.3]]]];
  }

  GeopBook.demo("loop-area", function (ctx) {
    var D = GeopBook.draw;
    var mode = "blob", loop = blob(), n = 8;
    var stage = ctx.stage({ aspect: 1.5, maxHeight: 360, label: "A closed loop of spline curves and the polygon sampled along it", draw: paint });
    var box = { x0: -0.03, x1: 1.03, y0: -0.05, y1: 0.85 };
    var view = ctx.view(stage, box);
    var pts = [];
    function linkPts() {
      pts.length = 0;
      if (mode === "blob") {
        // The loop's joints are shared, so they are one handle each.
        var A = loop[0][0], B = loop[1][0];
        B[3] = A[0]; B[0] = A[3];
        pts.push(A[0], A[1], A[2], A[3], B[1], B[2]);
      }
    }
    var handles = ctx.handles({ stage: stage, view: view, pts: pts, onchange: update });
    var read = ctx.readout();
    function paint(g, w, h, c) {
      if (!view) return;
      view.update();
      if (mode !== "blob") {
        box.y0 = -0.12; box.y1 = 0.12;
      } else {
        box.y0 = -0.05; box.y1 = 0.85;
      }
      view.update();
      var curve = [];
      loop.forEach(function (pc) {
        pc.forEach(function (s) { for (var i = 0; i <= 16; i++) curve.push(point(s, i / 16)); });
      });
      D.polygon(g, view, curve, greenArea(loop) > 0 ? c.fillA : c.fillB, c.fg, 1.8);
      var poly = samplePolygon(loop, n);
      D.polygon(g, view, poly, null, c.red, 1.4);
      poly.forEach(function (p) { D.dot(g, view, p, 3, c.red); });
      if (mode === "blob") {
        D.polyline(g, view, loop[0][0], c.muted, 1, [3, 3]);
        D.polyline(g, view, loop[1][0], c.muted, 1, [3, 3]);
        pts.forEach(function (p) { D.dot(g, view, p, 5.5, c.bg, c.fg); });
      }
    }
    function update() {
      if (mode === "blob") { loop[1][0][3] = loop[0][0][0]; loop[1][0][0] = loop[0][0][3]; }
      var exact = greenArea(loop), poly = polygonArea(samplePolygon(loop, n));
      var sign = function (x) { return x > 0 ? "counter-clockwise" : x < 0 ? "clockwise" : "none"; };
      read.set("Green, Gauss-Legendre on each span: " + exact.toFixed(6) + "  (" + sign(exact) + ")" +
        "\npolygon of " + n + " samples per pcurve: " + poly.toFixed(6) + "  (" + sign(poly) + ")" +
        "\nerror of the polygon: " + Math.abs(exact - poly).toExponential(2) +
        (exact * poly < 0 ? "\nThe polygon has the wrong sign, and nothing in it says so." : ""));
      stage.redraw();
    }
    ctx.select({
      label: "loop", value: "blob",
      options: [["blob", "two cubic spans"], ["sliver", "a sliver: a fit of 64 spans"]],
      oninput: function (v) {
        mode = v;
        loop = v === "blob" ? blob() : sliver(0.02, -0.05, 8);
        linkPts();
        update();
      }
    });
    ctx.slider({ label: "samples per pcurve", min: 2, max: 64, step: 1, value: 8, format: function (v) { return String(v); }, oninput: function (v) { n = v; update(); } });
    linkPts();
    update();
    void handles;
  }, { GL: GL, point: point, tangent: tangent, greenArea: greenArea, samplePolygon: samplePolygon, polygonArea: polygonArea, lineSpan: lineSpan, sliver: sliver, blob: blob });
})();
