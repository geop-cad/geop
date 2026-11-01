// The real IEEE 754 numbers near a chosen value: where they are, how far apart,
// and what the bits of one are. Nothing is simulated: the floats drawn are the
// ones this browser computes with (Float64Array, Float32Array).
(function () {
  "use strict";
  var f64 = new Float64Array(1), b64 = new BigUint64Array(f64.buffer);
  var f32 = new Float32Array(1), b32 = new Uint32Array(f32.buffer);

  function nextUp64(x) {
    if (x === Infinity || x !== x) return x;
    if (x === 0) return 5e-324;
    f64[0] = x;
    b64[0] = x > 0 ? b64[0] + 1n : b64[0] - 1n;
    return f64[0];
  }
  function nextUp32(x) {
    x = Math.fround(x);
    if (x === Infinity || x !== x) return x;
    if (x === 0) { b32[0] = 1; return f32[0]; }
    f32[0] = x;
    b32[0] = x > 0 ? b32[0] + 1 : b32[0] - 1;
    return f32[0];
  }
  // The distance from x to the next number above it.
  function ulp64(x) { x = Math.abs(x); return nextUp64(x) - x; }
  function ulp32(x) { x = Math.abs(Math.fround(x)); return nextUp32(x) - x; }

  // sign, biased exponent and fraction fields, and the value 2^e the fraction scales.
  function fields64(x) {
    f64[0] = x;
    var bits = b64[0];
    var sign = Number(bits >> 63n);
    var exponent = Number((bits >> 52n) & 0x7ffn);
    var fraction = bits & 0xfffffffffffffn;
    return { sign: sign, exponent: exponent, fraction: fraction, unbiased: exponent === 0 ? -1022 : exponent - 1023, subnormal: exponent === 0 };
  }
  function fields32(x) {
    f32[0] = x;
    var bits = b32[0];
    var exponent = (bits >>> 23) & 0xff;
    return { sign: bits >>> 31, exponent: exponent, fraction: BigInt(bits & 0x7fffff), unbiased: exponent === 0 ? -126 : exponent - 127, subnormal: exponent === 0 };
  }

  GeopBook.demo("float-ruler", function (ctx) {
    var D = GeopBook.draw;
    var prec = "double", exponent = 0, mantissa = 1;
    var x = 1;
    function value() {
      var v = mantissa * Math.pow(10, exponent);
      return prec === "double" ? v : Math.fround(v);
    }
    var nextUp = function (v) { return prec === "double" ? nextUp64(v) : nextUp32(v); };
    var ulp = function (v) { return prec === "double" ? ulp64(v) : ulp32(v); };

    var stage = ctx.stage({ aspect: 1.5, maxHeight: 360, label: "The floating point numbers next to a value, and how their spacing grows", draw: paint });
    var read = ctx.readout();

    function paint(g, w, h, c) {
      x = value();
      // Top: a ruler of the floats around x, positions exact in units of ulp.
      var top = 34, left = 16, right = w - 16;
      var u = ulp(x);
      var span = 4.5;
      function sx(k) { return left + (k + span) / (2 * span) * (right - left); }
      g.strokeStyle = c.muted; g.lineWidth = 1; g.beginPath(); g.moveTo(left, top); g.lineTo(right, top); g.stroke();
      var v = x;
      for (var k = 0; k <= 4; k++) {
        tick(g, sx((v - x) / u), top, k === 0 ? c.red : c.fg, k === 0 ? 16 : 10);
        v = nextUp(v);
      }
      // Below x the spacing can be half, at a power of two: walk down for real.
      v = x;
      for (k = 1; k <= 9; k++) {
        v = -nextUp(-v);
        var pos = (v - x) / u;
        if (pos < -span) break;
        tick(g, sx(pos), top, c.fg, 10);
      }
      g.font = "12px sans-serif";
      D.text(g, null, [sx(0), top - 14], "x", c.red, "center");
      D.text(g, null, [(left + right) / 2, top + 24], "each tick is a number the format holds", c.muted, "center");

      // Bottom: ulp against x over sixteen times either way, on log2 axes: a staircase.
      var y0 = top + 56, y1 = h - 24, pl = 58, pr = w - 12;
      var lx = Math.log2(x), lu = Math.log2(u);
      var xLo = lx - 4, xHi = lx + 4, yLo = lu - 4.5, yHi = lu + 4.5;
      function px(l) { return pl + (l - xLo) / (xHi - xLo) * (pr - pl); }
      function py(l) { return y1 - (l - yLo) / (yHi - yLo) * (y1 - y0); }
      g.strokeStyle = c.grid; g.lineWidth = 1; g.beginPath();
      for (var ly = Math.ceil(yLo); ly <= yHi; ly += 2) { g.moveTo(pl, py(ly)); g.lineTo(pr, py(ly)); }
      g.stroke();
      var pts = [];
      for (var i = 0; i <= 800; i++) {
        var l = xLo + (xHi - xLo) * i / 800;
        var val = Math.pow(2, l);
        pts.push([px(l), py(Math.log2(ulp(val)))]);
      }
      D.polyline(g, null, pts, c.blue, 1.8);
      D.dot(g, null, [px(lx), py(lu)], 4.5, c.red);
      g.font = "11px sans-serif";
      D.text(g, null, [pl, y0 - 6], "ulp, doubling per step", c.muted);
      D.text(g, null, [pr, y1 + 14], "x, from x/16 to 16x (log scale)", c.muted, "right");
      for (ly = Math.ceil(yLo); ly <= yHi; ly += 2) D.text(g, null, [pl - 4, py(ly) + 4], Math.pow(2, ly).toExponential(0), c.muted, "right");
    }
    function tick(g, x0, y, color, len) {
      g.strokeStyle = color; g.lineWidth = 1.5; g.beginPath(); g.moveTo(x0, y - len / 2); g.lineTo(x0, y + len / 2); g.stroke();
    }
    function update() {
      x = value();
      var f = prec === "double" ? fields64(x) : fields32(x);
      var u = ulp(x);
      read.set(
        "x        = " + x.toPrecision(prec === "double" ? 17 : 9) + "\n" +
        "next up  = " + nextUp(x).toPrecision(prec === "double" ? 17 : 9) + "\n" +
        "ulp(x)   = " + u.toExponential(3) + "\nulp(x)/x = " + (u / x).toExponential(2) + "\n" +
        "sign " + f.sign + "  exponent 2^" + f.unbiased + "  fraction 0x" + f.fraction.toString(16)
      );
      stage.redraw();
    }
    ctx.select({ label: "format", options: [["double", "f64 (double)"], ["single", "f32 (single)"]], value: prec, oninput: function (v) { prec = v; update(); } });
    var presets = ctx.select({
      label: "jump to", options: [["1,0", "1"], ["1,-1", "0.1"], ["1,3", "1000"], ["1,5", "100000"], ["9.007199254740992,15", "2^53"], ["1,16", "1e16"], ["2,0", "2 (a power of two)"]],
      oninput: function (v) {
        var p = v.split(",");
        mantissa = +p[0]; exponent = +p[1];
        slider.value = exponent + Math.log10(mantissa);
        update();
      }
    });
    var slider = ctx.slider({
      label: "log10 x", min: -12, max: 20, step: 0.01, value: 0,
      oninput: function (v) { exponent = Math.floor(v); mantissa = Math.pow(10, v - exponent); update(); }
    });
    update();
  }, { nextUp64: nextUp64, nextUp32: nextUp32, ulp64: ulp64, ulp32: ulp32, fields64: fields64, fields32: fields32 });
})();
