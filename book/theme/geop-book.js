// The lectures' runtime: math, and what the interactive demos share.
//
// Math is written in the Markdown as code spans, `$x$` and `$$x$$`, so that
// the Markdown parser leaves it alone; here KaTeX replaces them.
//
// A demo is a <figure class="demo" data-demo="name"> in a page and a file
// book/src/demos/name.js calling GeopBook.demo("name", init, algo). This file
// loads that script the first time such a figure comes near the viewport and
// calls init(ctx) with the helpers below. See book/AUTHORING.md.
//
// No dependencies, no network: every demo works from a saved copy of the book.
(function () {
  "use strict";

  // ---------------------------------------------------------------- math
  function renderMath(root) {
    if (!window.katex) return;
    root.querySelectorAll("code").forEach(function (code) {
      if (code.closest("pre")) return;
      var text = code.textContent;
      var display = /^\$\$[\s\S]*\$\$$/.test(text) && text.length > 4;
      var inline = !display && /^\$[^$]+\$$/.test(text);
      if (!display && !inline) return;
      var tex = display ? text.slice(2, -2) : text.slice(1, -1);
      var out = document.createElement("span");
      out.className = display ? "gb-math-display" : "gb-math-inline";
      try {
        katex.render(tex, out, { displayMode: display, throwOnError: true, strict: "ignore" });
      } catch (e) {
        out = document.createElement("code");
        out.textContent = text + "  [" + e.message + "]";
        out.style.color = "#c0392b";
        console.error("KaTeX: " + e.message + " in: " + text);
      }
      code.replaceWith(out);
    });
  }

  // ------------------------------------------------------------- colours
  function cssVar(name, fallback) {
    var v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    return v || fallback;
  }
  function parseColor(c) {
    var probe = document.createElement("canvas").getContext("2d");
    probe.fillStyle = "#000"; probe.fillStyle = c;
    var s = probe.fillStyle;
    if (s[0] === "#") return [parseInt(s.slice(1, 3), 16), parseInt(s.slice(3, 5), 16), parseInt(s.slice(5, 7), 16)];
    var m = s.match(/[\d.]+/g);
    return m ? m.slice(0, 3).map(Number) : [255, 255, 255];
  }
  function colors() {
    
    var bg = cssVar("--bg", "#fff");
    var fg = cssVar("--fg", "#000");
    var c = parseColor(bg);
    var dark = 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2] < 128;
    return dark ? {
      dark: true, bg: bg, fg: fg, muted: "rgba(220,225,235,0.55)", faint: "rgba(220,225,235,0.18)",
      grid: "rgba(220,225,235,0.10)", accent: "#6cb2ff", blue: "#6cb2ff", orange: "#ffa94d",
      green: "#5fd38d", red: "#ff7b7b", purple: "#c9a2ff", teal: "#4fd1c5", yellow: "#f6d860",
      fillA: "rgba(108,178,255,0.22)", fillB: "rgba(255,169,77,0.22)", fillC: "rgba(95,211,141,0.22)",
      series: ["#6cb2ff", "#ffa94d", "#5fd38d", "#ff7b7b", "#c9a2ff", "#4fd1c5", "#f6d860"]
    } : {
      dark: false, bg: bg, fg: fg, muted: "rgba(40,45,60,0.6)", faint: "rgba(40,45,60,0.18)",
      grid: "rgba(40,45,60,0.08)", accent: "#1f6fc5", blue: "#1f6fc5", orange: "#d9730d",
      green: "#1e8e4e", red: "#cc3333", purple: "#7b3fc4", teal: "#0f8f85", yellow: "#a88600",
      fillA: "rgba(31,111,197,0.16)", fillB: "rgba(217,115,13,0.16)", fillC: "rgba(30,142,78,0.16)",
      series: ["#1f6fc5", "#d9730d", "#1e8e4e", "#cc3333", "#7b3fc4", "#0f8f85", "#a88600"]
    };
  }

  // ------------------------------------------------------------- context
  var reduced = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)");
  function reducedMotion() { return !!(reduced && reduced.matches); }

  function makeContext(root, args) {
    var body = document.createElement("div");
    body.className = "gb-body";
    var caption = root.querySelector("figcaption");
    if (caption) root.insertBefore(body, caption); else root.appendChild(body);
    var controlsEl = null;
    var redrawers = [];
    var visible = true;

    function controls() {
      if (!controlsEl) {
        controlsEl = document.createElement("div");
        controlsEl.className = "gb-controls";
        body.appendChild(controlsEl);
      }
      return controlsEl;
    }

    var ctx = {
      root: root,
      args: args,
      colors: colors,
      get reducedMotion() { return reducedMotion(); },
      get visible() { return visible; },
      _setVisible: function (v) { visible = v; },
      _redrawAll: function () { redrawers.forEach(function (r) { r(); }); },

      // A canvas that follows the width of the page, sharp on any screen.
      // opts: { height: px, aspect: width/height, maxHeight: px, draw(g, w, h, colors) }
      stage: function (opts) {
        opts = opts || {};
        var wrap = document.createElement("div");
        wrap.className = "gb-stage";
        var canvas = document.createElement("canvas");
        canvas.tabIndex = 0;
        if (opts.label) { canvas.setAttribute("role", "img"); canvas.setAttribute("aria-label", opts.label); }
        wrap.appendChild(canvas);
        body.appendChild(wrap);
        var g = canvas.getContext("2d");
        var pending = false;
        var stage = {
          canvas: canvas, g: g, w: 0, h: 0, draw: opts.draw || function () {},
          redraw: function () {
            if (pending) return;
            pending = true;
            requestAnimationFrame(function () { pending = false; stage.paint(); });
          },
          paint: function () {
            var dpr = window.devicePixelRatio || 1;
            var w = Math.max(wrap.clientWidth, 120);
            var h = opts.height ? opts.height : Math.round(w / (opts.aspect || 1.6));
            if (opts.maxHeight && h > opts.maxHeight) h = opts.maxHeight;
            if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
              canvas.width = Math.round(w * dpr);
              canvas.height = Math.round(h * dpr);
              canvas.style.height = h + "px";
            }
            stage.w = w; stage.h = h;
            g.setTransform(dpr, 0, 0, dpr, 0, 0);
            var c = colors();
            g.clearRect(0, 0, w, h);
            g.font = "12px sans-serif";
            g.lineJoin = "round"; g.lineCap = "round";
            stage.draw(g, w, h, c);
          }
        };
        redrawers.push(stage.paint);
        if (window.ResizeObserver) new ResizeObserver(function () { stage.redraw(); }).observe(wrap);
        stage.paint();
        return stage;
      },

      // Maps the world box into a stage, equal scale on both axes (y up) unless
      // opts.free, centred; recomputed at each call, so call it in draw too.
      view: function (stage, box, opts) {
        opts = opts || {};
        var pad = opts.pad === undefined ? 12 : opts.pad;
        var v = {
          box: box,
          sx: 1, sy: 1, ox: 0, oy: 0,
          update: function () {
            var w = Math.max(stage.w, 1), h = Math.max(stage.h, 1);
            var bw = box.x1 - box.x0, bh = box.y1 - box.y0;
            var sx = (w - 2 * pad) / bw, sy = (h - 2 * pad) / bh;
            if (!opts.free) sx = sy = Math.min(sx, sy);
            v.sx = sx; v.sy = sy;
            v.ox = (w - bw * sx) / 2 - box.x0 * sx;
            v.oy = (h + bh * sy) / 2 + box.y0 * sy;
          },
          x: function (x) { return v.ox + x * v.sx; },
          y: function (y) { return v.oy - y * v.sy; },
          px: function (p) { return [v.ox + p[0] * v.sx, v.oy - p[1] * v.sy]; },
          inv: function (sx, sy) { return [(sx - v.ox) / v.sx, (v.oy - sy) / v.sy]; }
        };
        v.update();
        return v;
      },

      // Draggable points. pts: array of [x, y] (mutated in place).
      // opts: { view, stage, radius, onchange(i), constrain(i, p), active(i) }.
      // Keyboard: focus the canvas, n / p choose a point, arrows move it.
      handles: function (opts) {
        var stage = opts.stage, view = opts.view, pts = opts.pts, canvas = stage.canvas;
        var hit = (opts.radius || 22);
        var dragging = -1;
        var h = { selected: -1, hover: -1, pts: pts };
        function pos(ev) {
          var r = canvas.getBoundingClientRect();
          return [ev.clientX - r.left, ev.clientY - r.top];
        }
        function nearest(sx, sy) {
          var best = -1, bd = hit * hit;
          view.update();
          for (var i = 0; i < pts.length; i++) {
            if (opts.active && !opts.active(i)) continue;
            var dx = view.x(pts[i][0]) - sx, dy = view.y(pts[i][1]) - sy, d = dx * dx + dy * dy;
            if (d < bd) { bd = d; best = i; }
          }
          return best;
        }
        function moveTo(i, p) {
          var q = opts.constrain ? opts.constrain(i, p) : p;
          if (q) { pts[i][0] = q[0]; pts[i][1] = q[1]; }
          if (opts.onchange) opts.onchange(i);
          stage.redraw();
        }
        canvas.addEventListener("pointerdown", function (ev) {
          var p = pos(ev), i = nearest(p[0], p[1]);
          if (i < 0) return;
          dragging = i; h.selected = i;
          if (canvas.setPointerCapture) canvas.setPointerCapture(ev.pointerId);
          ev.preventDefault();
          stage.redraw();
        });
        canvas.addEventListener("pointermove", function (ev) {
          var p = pos(ev);
          if (dragging >= 0) {
            view.update();
            moveTo(dragging, view.inv(p[0], p[1]));
          } else {
            var i = nearest(p[0], p[1]);
            if (i !== h.hover) { h.hover = i; canvas.style.cursor = i >= 0 ? "grab" : ""; stage.redraw(); }
          }
        });
        function up() { dragging = -1; if (opts.onrelease) opts.onrelease(); }
        canvas.addEventListener("pointerup", up);
        canvas.addEventListener("pointercancel", up);
        // A touch on a handle drags it; a touch elsewhere scrolls the page.
        canvas.addEventListener("touchstart", function (ev) {
          if (ev.touches.length !== 1) return;
          var r = canvas.getBoundingClientRect(), t = ev.touches[0];
          if (nearest(t.clientX - r.left, t.clientY - r.top) >= 0) ev.preventDefault();
        }, { passive: false });
        canvas.addEventListener("keydown", function (ev) {
          if (!pts.length) return;
          if (ev.key === "n" || ev.key === "p") {
            h.selected = ((h.selected < 0 ? -1 : h.selected) + (ev.key === "n" ? 1 : pts.length - 1)) % pts.length;
            stage.redraw(); ev.preventDefault(); return;
          }
          var d = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, 1], ArrowDown: [0, -1] }[ev.key];
          if (!d || h.selected < 0) return;
          view.update();
          var step = 4 / view.sx, p = pts[h.selected];
          moveTo(h.selected, [p[0] + d[0] * step, p[1] + d[1] * (4 / view.sy)]);
          ev.preventDefault();
        });
        return h;
      },

      // Drag anywhere on the stage (for orbiting, sliding). fn(dx, dy, x, y, phase)
      // A vertical touch still scrolls the page unless opts.claim is set.
      drag: function (stage, fn, opts) {
        var c = stage.canvas, last = null;
        if (opts && opts.claim) c.style.touchAction = "none"; else c.style.touchAction = "pan-y";
        c.addEventListener("pointerdown", function (ev) {
          var r = c.getBoundingClientRect();
          last = [ev.clientX, ev.clientY];
          if (c.setPointerCapture) c.setPointerCapture(ev.pointerId);
          fn(0, 0, ev.clientX - r.left, ev.clientY - r.top, "start");
        });
        c.addEventListener("pointermove", function (ev) {
          if (!last) return;
          var r = c.getBoundingClientRect();
          var dx = ev.clientX - last[0], dy = ev.clientY - last[1];
          last = [ev.clientX, ev.clientY];
          fn(dx, dy, ev.clientX - r.left, ev.clientY - r.top, "move");
          stage.redraw();
        });
        function up() { last = null; }
        c.addEventListener("pointerup", up);
        c.addEventListener("pointercancel", up);
      },

      // A 3-D view, orbited by dragging, by a perspective camera at a distance.
      // opts: { yaw, pitch, scale (px per unit at the centre, relative to min(w,h)), centre, distance }
      // project([x,y,z]) -> [sx, sy, depth], depth larger is nearer the viewer.
      camera: function (stage, opts) {
        opts = opts || {};
        var cam = {
          yaw: opts.yaw === undefined ? 0.7 : opts.yaw,
          pitch: opts.pitch === undefined ? 0.5 : opts.pitch,
          scale: opts.scale || 0.35,
          centre: opts.centre || [0, 0, 0],
          distance: opts.distance || 8,
          project: function (p) {
            var x = p[0] - cam.centre[0], y = p[1] - cam.centre[1], z = p[2] - cam.centre[2];
            var cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw), cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
            // Turn about z (up), then tilt about the screen's x.
            var x1 = cy * x - sy * y, y1 = sy * x + cy * y;
            var depth = -(cp * y1 - sp * z) ;
            var up = sp * y1 + cp * z;
            var f = cam.distance / (cam.distance - depth);
            var s = Math.min(stage.w, stage.h) * cam.scale * f;
            return [stage.w / 2 + x1 * s, stage.h / 2 - up * s, depth];
          }
        };
        if (opts.orbit !== false) {
          ctx.drag(stage, function (dx, dy, x, y, phase) {
            if (phase !== "move") return;
            cam.yaw += dx * 0.01;
            cam.pitch = Math.max(-1.5, Math.min(1.5, cam.pitch + dy * 0.01));
          });
        }
        return cam;
      },

      // ---- controls
      slider: function (o) {
        var label = document.createElement("label");
        var text = document.createElement("span"); text.textContent = o.label;
        var input = document.createElement("input");
        input.type = "range"; input.min = o.min; input.max = o.max; input.step = o.step || (o.max - o.min) / 200;
        input.value = o.value;
        var value = document.createElement("span"); value.className = "gb-value";
        var fmt = o.format || function (v) { return (+v).toFixed(2); };
        function show() { value.textContent = fmt(+input.value); }
        input.addEventListener("input", function () { show(); if (o.oninput) o.oninput(+input.value); });
        show();
        label.appendChild(text); label.appendChild(input); label.appendChild(value);
        controls().appendChild(label);
        return { el: input, get value() { return +input.value; }, set value(v) { input.value = v; show(); } };
      },
      select: function (o) {
        var label = document.createElement("label"); label.className = "gb-compact";
        var text = document.createElement("span"); text.textContent = o.label || "";
        var sel = document.createElement("select");
        o.options.forEach(function (opt) {
          var e = document.createElement("option");
          if (Array.isArray(opt)) { e.value = opt[0]; e.textContent = opt[1]; } else { e.value = opt; e.textContent = opt; }
          sel.appendChild(e);
        });
        if (o.value !== undefined) sel.value = o.value;
        sel.addEventListener("change", function () { if (o.oninput) o.oninput(sel.value); });
        label.appendChild(text); label.appendChild(sel);
        controls().appendChild(label);
        return { el: sel, get value() { return sel.value; }, set value(v) { sel.value = v; } };
      },
      checkbox: function (o) {
        var label = document.createElement("label"); label.className = "gb-compact";
        var input = document.createElement("input"); input.type = "checkbox"; input.checked = !!o.value;
        var text = document.createElement("span"); text.textContent = o.label;
        input.addEventListener("change", function () { if (o.oninput) o.oninput(input.checked); });
        label.appendChild(input); label.appendChild(text);
        controls().appendChild(label);
        return { el: input, get value() { return input.checked; }, set value(v) { input.checked = v; } };
      },
      button: function (text, fn) {
        var b = document.createElement("button"); b.type = "button"; b.textContent = text;
        b.addEventListener("click", fn);
        controls().appendChild(b);
        return b;
      },
      // A text box under the stage for numbers; set(text) replaces its text.
      readout: function (initial) {
        var el = document.createElement("div"); el.className = "gb-readout";
        el.setAttribute("aria-live", "polite");
        if (initial) el.textContent = initial;
        body.appendChild(el);
        return { el: el, set: function (t) { el.textContent = t; } };
      },

      // Play / pause / step over a time t in [0, 1] (or integer steps).
      // opts: { duration: seconds, steps: integer count (then t is 0..steps, stepping
      // by 1), loop: bool, onframe(t), autoplay: bool, label }
      // Never moves by itself under prefers-reduced-motion, and only while on screen.
      player: function (o) {
        var t = 0, playing = false, last = 0;
        var discrete = o.steps !== undefined;
        var max = discrete ? o.steps : 1;
        var period = (o.duration || 4) * 1000;
        var playBtn = ctx.button("Play", toggle);
        var stepBtn = ctx.button("Step", step);
        ctx.button("Reset", function () { setPlaying(false); set(0); });
        var slider = ctx.slider({
          label: o.label || (discrete ? "step" : "t"), min: 0, max: max, step: discrete ? 1 : 0.001, value: 0,
          format: discrete ? function (v) { return v + "/" + max; } : undefined,
          oninput: function (v) { setPlaying(false); t = v; o.onframe(t); }
        });
        function set(v) { t = v; slider.value = v; o.onframe(t); }
        function setPlaying(p) {
          playing = p; playBtn.textContent = p ? "Pause" : "Play";
          playBtn.setAttribute("aria-pressed", p ? "true" : "false");
          if (p) { last = performance.now(); requestAnimationFrame(tick); }
        }
        function toggle() {
          if (!playing && t >= max) set(0);
          setPlaying(!playing);
        }
        function step() {
          setPlaying(false);
          var nt = discrete ? Math.min(max, Math.floor(t) + 1) : Math.min(1, t + 0.02);
          if (nt >= max && t >= max) nt = 0;
          set(nt);
        }
        function tick(now) {
          if (!playing) return;
          var dt = now - last;
          if (!ctx.visible) { last = now; requestAnimationFrame(tick); return; }
          if (discrete) {
            if (dt >= period / Math.max(max, 1)) {
              last = now;
              if (t >= max) {
                if (o.loop) set(0); else { setPlaying(false); return; }
              } else set(t + 1);
            }
          } else {
            last = now;
            var nt = t + dt / period;
            if (nt >= 1) {
              if (o.loop) nt = nt - 1; else { set(1); setPlaying(false); return; }
            }
            set(nt);
          }
          requestAnimationFrame(tick);
        }
        if (o.autoplay && !reducedMotion()) setPlaying(true);
        return {
          get t() { return t; }, set: set, play: function () { setPlaying(true); },
          pause: function () { setPlaying(false); }, get playing() { return playing; }
        };
      },

      // A note that the demo has a live version in the real kernel's web app.
      note: function (html) {
        var el = document.createElement("div"); el.className = "gb-readout"; el.innerHTML = html;
        body.appendChild(el);
        return el;
      }
    };
    return ctx;
  }

  // Line and text drawing every demo wants.
  var draw = {
    polyline: function (g, view, pts, color, width, dash) {
      if (!pts.length) return;
      g.beginPath();
      for (var i = 0; i < pts.length; i++) {
        var x = view ? view.x(pts[i][0]) : pts[i][0], y = view ? view.y(pts[i][1]) : pts[i][1];
        if (i) g.lineTo(x, y); else g.moveTo(x, y);
      }
      g.strokeStyle = color; g.lineWidth = width || 1.5;
      g.setLineDash(dash || []);
      g.stroke();
      g.setLineDash([]);
    },
    polygon: function (g, view, pts, fill, stroke, width) {
      if (!pts.length) return;
      g.beginPath();
      for (var i = 0; i < pts.length; i++) {
        var x = view ? view.x(pts[i][0]) : pts[i][0], y = view ? view.y(pts[i][1]) : pts[i][1];
        if (i) g.lineTo(x, y); else g.moveTo(x, y);
      }
      g.closePath();
      if (fill) { g.fillStyle = fill; g.fill(); }
      if (stroke) { g.strokeStyle = stroke; g.lineWidth = width || 1.5; g.stroke(); }
    },
    dot: function (g, view, p, r, color, stroke) {
      var x = view ? view.x(p[0]) : p[0], y = view ? view.y(p[1]) : p[1];
      g.beginPath(); g.arc(x, y, r || 4, 0, 2 * Math.PI);
      if (color) { g.fillStyle = color; g.fill(); }
      if (stroke) { g.strokeStyle = stroke; g.lineWidth = 1.5; g.stroke(); }
    },
    text: function (g, view, p, s, color, align, baseline) {
      var x = view ? view.x(p[0]) : p[0], y = view ? view.y(p[1]) : p[1];
      g.fillStyle = color; g.textAlign = align || "left"; g.textBaseline = baseline || "alphabetic";
      g.fillText(s, x, y);
    },
    // Grid lines at the integer multiples of `step` inside the view's box.
    grid: function (g, view, step, color) {
      var b = view.box;
      g.strokeStyle = color; g.lineWidth = 1; g.beginPath();
      for (var x = Math.ceil(b.x0 / step) * step; x <= b.x1 + 1e-9; x += step) { g.moveTo(view.x(x), view.y(b.y0)); g.lineTo(view.x(x), view.y(b.y1)); }
      for (var y = Math.ceil(b.y0 / step) * step; y <= b.y1 + 1e-9; y += step) { g.moveTo(view.x(b.x0), view.y(y)); g.lineTo(view.x(b.x1), view.y(y)); }
      g.stroke();
    }
  };

  // ------------------------------------------------------------- loading
  var registry = {};
  var waiting = {};
  var algos = {};

  function pathToRoot() { return typeof path_to_root === "string" ? path_to_root : ""; }

  function start(root) {
    var name = root.getAttribute("data-demo");
    var init = registry[name];
    var pending = root.querySelector(".gb-pending");
    if (pending) pending.remove();
    var args = {};
    try { args = JSON.parse(root.getAttribute("data-args") || "{}"); } catch (e) { console.error(e); }
    var ctx = makeContext(root, args);
    roots.push(ctx);
    if (window.IntersectionObserver) {
      new IntersectionObserver(function (es) { ctx._setVisible(es[es.length - 1].isIntersecting); }).observe(root);
    }
    try {
      init(ctx);
    } catch (e) {
      var err = document.createElement("div"); err.className = "gb-error";
      err.textContent = "This demo failed to start: " + e.message;
      root.appendChild(err);
      console.error("demo " + name + ": ", e);
    }
  }
  var roots = [];

  function load(root) {
    var name = root.getAttribute("data-demo");
    if (registry[name]) { start(root); return; }
    if (waiting[name]) { waiting[name].push(root); return; }
    waiting[name] = [root];
    var s = document.createElement("script");
    s.src = pathToRoot() + "demos/" + name + ".js";
    s.onload = function () {
      if (!registry[name]) console.error("demos/" + name + ".js did not call GeopBook.demo(\"" + name + "\", ...)");
      else waiting[name].forEach(start);
      delete waiting[name];
    };
    s.onerror = function () { console.error("could not load demos/" + name + ".js"); };
    document.body.appendChild(s);
  }

  function scan() {
    var figures = Array.prototype.slice.call(document.querySelectorAll("figure.demo[data-demo]"));
    figures.forEach(function (f) {
      var p = document.createElement("div"); p.className = "gb-pending"; p.textContent = "loading demo…";
      f.insertBefore(p, f.firstChild);
    });
    if (window.IntersectionObserver) {
      var io = new IntersectionObserver(function (entries) {
        entries.forEach(function (e) {
          if (e.isIntersecting) { io.unobserve(e.target); load(e.target); }
        });
      }, { rootMargin: "400px" });
      figures.forEach(function (f) { io.observe(f); });
    } else {
      figures.forEach(load);
    }
  }

  // mdBook sets the theme by a class on <html>; a demo draws in its colours.
  new MutationObserver(function () {
    roots.forEach(function (c) { c._redrawAll(); });
  }).observe(document.documentElement, { attributes: true, attributeFilter: ["class"] });
  if (window.matchMedia) {
    var scheme = window.matchMedia("(prefers-color-scheme: dark)");
    if (scheme.addEventListener) scheme.addEventListener("change", function () { roots.forEach(function (c) { c._redrawAll(); }); });
  }

  window.GeopBook = {
    // init(ctx) builds the demo in ctx.root. algo is an object of the demo's pure
    // functions, kept apart from any drawing so scripts/test_demo_math.mjs can check them.
    demo: function (name, init, algo) { registry[name] = init; if (algo) algos[name] = algo; },
    draw: draw,
    algos: algos
  };

  renderMath(document);
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", scan);
  else scan();
})();
