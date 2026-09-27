// Anonymous usage statistics for the landing page — the counterpart of the
// app's web/src/analytics.ts, with the same guarantees: PostHog's EU cloud
// in cookieless mode, nothing stored on the visitor's device, no session
// replay, no autocapture. Only the events below are sent, and none carries
// anything the visitor typed.
//
// - `$pageview`, once.
// - `section_viewed` {section}: a section was on screen long enough to be
//   read (once per section per page load) — which parts people read.
// - `example_viewed` {name}: an example card was shown in the carousel.
// - `carousel_navigated` {direction}: the carousel's arrows were used.
// - `link_clicked` {target, section, example?}: a link or button was
//   followed — the app, an example, sponsoring, GitHub, the book, the docs,
//   the contact email, the privacy page.
//
// Off unless the deploy substituted the PostHog key below (see
// .github/workflows/deploy-landing.yml), and off for anyone who sends Do
// Not Track / Global Privacy Control or opted out on privacy.html. The
// privacy notice describes exactly this; change the two together.

const KEY = "__POSTHOG_KEY__";
const OPT_OUT_KEY = "geop-analytics-opt-out";
/** How long a section must stay in view to count as read. */
const DWELL_MS = 1500;

function optedOut() {
  if (navigator.doNotTrack === "1" || navigator.globalPrivacyControl === true) return true;
  try {
    return localStorage.getItem(OPT_OUT_KEY) === "1";
  } catch {
    return false;
  }
}

function start() {
  const posthog = window.posthog;
  posthog.init(KEY, {
    api_host: "https://eu.i.posthog.com",
    cookieless_mode: "always",
    persistence: "memory",
    person_profiles: "never",
    autocapture: false,
    capture_pageview: true,
    capture_pageleave: false,
    disable_session_recording: true,
    capture_heatmaps: false,
    capture_dead_clicks: false,
    capture_exceptions: false,
    capture_performance: false,
    disable_surveys: true,
    advanced_disable_flags: true,
    disable_external_dependency_loading: true,
  });
  const track = (event, properties, options) => posthog.capture(event, properties, options);

  // The section an element sits in: a section's id, or its first class for
  // the two without one (the hero with the examples, and the sponsor ask).
  const sectionOf = (el) => {
    if (el.closest("header.site")) return "nav";
    if (el.closest("footer.site")) return "footer";
    const section = el.closest("section");
    return section ? section.id || section.classList[0] || "unknown" : "unknown";
  };

  // ── sections read ─────────────────────────────────────────────────────
  const timers = new Map();
  const seen = new Set();
  const onRead = []; // called with a section's name once it counts as read
  const sections = new IntersectionObserver(
    (entries) => {
      for (const entry of entries) {
        const name = sectionOf(entry.target);
        if (seen.has(name)) continue;
        // In view: most of the section, or — for one taller than the
        // screen — most of the screen.
        const screen = entry.rootBounds ? entry.intersectionRect.height / entry.rootBounds.height : 0;
        const inView = entry.isIntersecting && (entry.intersectionRatio >= 0.6 || screen >= 0.5);
        if (inView && !timers.has(name)) {
          timers.set(
            name,
            setTimeout(() => {
              seen.add(name);
              track("section_viewed", { section: name });
              onRead.forEach((f) => f(name));
            }, DWELL_MS),
          );
        } else if (!inView && timers.has(name)) {
          clearTimeout(timers.get(name));
          timers.delete(name);
        }
      }
    },
    { threshold: [0, 0.25, 0.5, 0.6, 0.75, 1] },
  );
  document.querySelectorAll("main section").forEach((s) => sections.observe(s));

  // ── examples looked at ────────────────────────────────────────────────
  // A card counts as seen when it is showing in the carousel *and* the
  // carousel's section has been read: a card "in view" of a carousel no one
  // has scrolled to has not been seen by anyone.
  const carousel = document.getElementById("example-carousel");
  if (carousel) {
    const home = sectionOf(carousel);
    const showing = new Set();
    const reported = new Set();
    const report = () => {
      if (!seen.has(home)) return;
      for (const name of showing) {
        if (reported.has(name)) continue;
        reported.add(name);
        track("example_viewed", { name });
      }
    };
    const cards = new IntersectionObserver(
      (entries) => {
        for (const entry of entries) {
          const name = entry.target.dataset.example;
          if (entry.isIntersecting) showing.add(name);
          else showing.delete(name);
        }
        report();
      },
      { root: carousel, threshold: 0.6 },
    );
    carousel.querySelectorAll("[data-example]").forEach((card) => cards.observe(card));
    onRead.push((name) => name === home && report());
  }
  document.querySelectorAll(".carousel-nav").forEach((button) =>
    button.addEventListener("click", () =>
      track("carousel_navigated", { direction: button.classList.contains("prev") ? "prev" : "next" }),
    ),
  );

  // ── links followed ────────────────────────────────────────────────────
  const targetOf = (a) => {
    if (a.hasAttribute("data-email-link")) return { target: "contact" };
    const url = new URL(a.href, location.href);
    if (url.hostname === "app.geop-cad.dev") {
      const example = url.searchParams.get("example");
      return example ? { target: "app_example", example } : { target: "app" };
    }
    if (url.hostname === "ko-fi.com") return { target: "sponsor" };
    if (url.hostname === "github.com") return { target: "github" };
    if (url.hostname === "book.geop-cad.dev") return { target: "book" };
    if (url.hostname === "docs.geop-cad.dev") return { target: "docs" };
    if (url.pathname.endsWith("privacy.html")) return { target: "privacy" };
    if (url.hash && url.pathname === location.pathname) return { target: "anchor", anchor: url.hash.slice(1) };
    return { target: "other" };
  };
  document.addEventListener(
    "click",
    (e) => {
      const a = e.target instanceof Element ? e.target.closest("a[href]") : null;
      if (!a) return;
      // Sent at once rather than batched: most of these leave the page.
      track("link_clicked", { ...targetOf(a), section: sectionOf(a) }, { send_instantly: true });
    },
    { capture: true },
  );
}

if (!KEY.startsWith("__") && !optedOut()) {
  const script = document.createElement("script");
  script.src = "vendor/posthog/array.no-external.js";
  script.onload = start;
  document.head.appendChild(script);
}
