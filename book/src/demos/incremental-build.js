// A small model of the runner's memo table: nodes keyed by (target, step), each
// recording what it read; a node is reused iff its arguments are equal and every
// read holds now; a node made again into an equal value keeps its version.
// The rules are those of StepContext::produce (core/geop-core-program/src/part.rs);
// the "geometry" here is arithmetic standing in for a make.
(function () {
  "use strict";

  // ---- the model of the runner -------------------------------------------
  var stamp = 0;
  function newVersion() { stamp += 1; return stamp; }

  function emptyPart() { return { nodes: {}, makes: 0, log: [] }; }

  // The node of `name` a step at position `at` sees: the latest before it.
  function visible(part, order, name, at) {
    for (var i = at - 1; i >= 0; i--) {
      var n = part.nodes[name + "@" + order[i]];
      if (n) return n;
    }
    return null;
  }

  function holds(part, order, read, at, inputs) {
    if (read.kind === "input") return inputs[read.name] === read.value;
    var n = visible(part, order, read.name, at);
    if (read.kind === "absent") return n === null;
    return !!n && n.step === read.producer && n.version === read.version;
  }

  function same(a, b) { return JSON.stringify(a) === JSON.stringify(b); }

  // One run. `program` is a list of { id, target, args, make(read, args) }.
  // Returns the status of each step: "reused", "made", or "kept" (made again,
  // into an equal value, so its version stayed and readers are reused).
  function run(part, program, inputs) {
    var order = program.map(function (s) { return s.id; });
    var status = {};
    program.forEach(function (step, at) {
      var key = step.target + "@" + step.id;
      var old = part.nodes[key];
      if (old && same(old.args, step.args) && old.reads.every(function (r) { return holds(part, order, r, at, inputs); })) {
        status[step.id] = "reused";
        return;
      }
      var reads = [];
      var api = {
        input: function (name) { reads.push({ kind: "input", name: name, value: inputs[name] }); return inputs[name]; },
        target: function (name) {
          var n = visible(part, order, name, at);
          if (!n) { reads.push({ kind: "absent", name: name }); return null; }
          reads.push({ kind: "target", name: name, producer: n.step, version: n.version });
          return n.value;
        }
      };
      var value = step.make(api, step.args);
      part.makes += 1;
      var keep = old && same(old.value, value);
      status[step.id] = keep ? "kept" : "made";
      part.nodes[key] = { step: step.id, target: step.target, args: step.args, reads: reads, value: value, version: keep ? old.version : newVersion() };
    });
    // Nodes of steps no longer in the program are dropped at the end of a run.
    Object.keys(part.nodes).forEach(function (k) { if (order.indexOf(part.nodes[k].step) < 0) delete part.nodes[k]; });
    return status;
  }

  // What the nodes hold, by target and step: what must equal a build from scratch.
  function values(part) {
    var out = {};
    Object.keys(part.nodes).sort().forEach(function (k) { out[k] = part.nodes[k].value; });
    return out;
  }

  // The plate of the lecture: a sketch rounded to a whole width, a box, a hole.
  function plateProgram(order) {
    var steps = {
      outline: { id: "outline", target: "sketch:outline", args: {}, make: function (r) { return { w: Math.round(r.input("width")) }; } },
      hole_sketch: { id: "hole_sketch", target: "sketch:hole", args: {}, make: function (r) { return { r: r.input("radius") }; } },
      box: { id: "box", target: "model", args: {}, make: function (r) {
        var s = r.target("sketch:outline");
        return { volume: s ? s.w * s.w * r.input("depth") : null };
      } },
      hole: { id: "hole", target: "model", args: {}, make: function (r) {
        var m = r.target("model"), s = r.target("sketch:hole");
        return { volume: m && s ? m.volume - Math.PI * s.r * s.r * r.input("depth") : null };
      } },
      label: { id: "label", target: "label", args: {}, make: function (r) { return { text: r.input("title") }; } }
    };
    return (order || ["outline", "hole_sketch", "box", "hole", "label"]).map(function (id) { return steps[id]; });
  }

  GeopBook.demo("incremental-build", function (ctx) {
    var D = GeopBook.draw;
    var inputs = { width: 2, depth: 1, radius: 0.4, title: 1 };
    var order = ["outline", "hole_sketch", "box", "hole", "label"];
    var part = emptyPart();
    var status = {};
    var total = 0;
    var stage = ctx.stage({ height: 270, label: "The steps of a program and the state of their nodes after the last run", draw: paint });
    var read = ctx.readout();

    function rerun() {
      var program = plateProgram(order);
      var before = part.makes;
      status = run(part, program, inputs);
      total = part.makes - before;
      var scratch = emptyPart();
      run(scratch, program, inputs);
      var v = values(part), w = values(scratch);
      var agree = same(v, w);
      var model = part.nodes["model@hole"], box = part.nodes["model@box"];
      var last = order.indexOf("hole") > order.indexOf("box") ? model : box;
      read.set("makes this run: " + total + " of " + program.length + " steps\n" +
        "the same program from scratch makes " + scratch.makes + " and holds " + (agree ? "equal" : "DIFFERENT") + " nodes (assert_incremental's check)\n" +
        "volume of the model: " + (last && last.value.volume !== null ? last.value.volume.toFixed(4) : "none"));
      stage.redraw();
    }
    function paint(g, w, h, c) {
      var names = { made: ["made", c.orange], kept: ["made, equal: same version", c.purple], reused: ["reused", c.green] };
      var rowH = 44, left = 8, top = 8;
      var program = plateProgram(order);
      var yOf = {};
      program.forEach(function (s, i) { yOf[s.id] = top + i * rowH; });
      // What each node read: lines from its producer's row.
      var gutter = Math.min(70, w * 0.16);
      program.forEach(function (s, i) {
        var node = part.nodes[s.target + "@" + s.id];
        if (!node) return;
        var bump = 0;
        node.reads.forEach(function (r) {
          if (r.kind !== "target") return;
          var y0 = yOf[r.producer] + 16, y1 = yOf[s.id] + 16;
          bump += 9;
          g.beginPath(); g.moveTo(left + gutter, y0);
          g.bezierCurveTo(left + gutter - 26 - bump, y0, left + gutter - 26 - bump, y1, left + gutter, y1);
          g.strokeStyle = c.muted; g.lineWidth = 1.2; g.stroke();
          D.dot(g, null, [left + gutter, y1], 2.5, c.muted);
        });
      });
      program.forEach(function (s) {
        var y = yOf[s.id], node = part.nodes[s.target + "@" + s.id];
        var st = names[status[s.id]] || ["", c.muted];
        var x = left + gutter + 4, bw = w - x - 8;
        g.fillStyle = c.faint; g.fillRect(x, y, bw, 34);
        g.fillStyle = st[1]; g.fillRect(x, y, 5, 34);
        D.text(g, null, [x + 11, y + 14], s.id + "   " + s.target, c.fg);
        var inputsRead = node ? node.reads.filter(function (r) { return r.kind === "input"; }).map(function (r) { return r.name; }).join(", ") : "";
        D.text(g, null, [x + 11, y + 28], st[0] + (node ? "  v" + node.version : "") + (inputsRead && bw > 300 ? "  reads " + inputsRead : ""), c.muted);
      });
    }
    function slider(key, label, min, max, step) {
      ctx.slider({ label: label, min: min, max: max, step: step, value: inputs[key], oninput: function (v) { inputs[key] = v; rerun(); } });
    }
    slider("width", "width", 1, 4, 0.1);
    slider("depth", "depth", 0.5, 2, 0.1);
    slider("radius", "hole r", 0.1, 0.5, 0.05);
    slider("title", "title", 1, 5, 1);
    ctx.button("Swap the two sketches", function () {
      var a = order.indexOf("outline"), b = order.indexOf("hole_sketch");
      order[a] = "hole_sketch"; order[b] = "outline";
      rerun();
    });
    ctx.button("Run again", rerun);
    rerun();
  }, { emptyPart: emptyPart, run: run, values: values, plateProgram: plateProgram });
})();
