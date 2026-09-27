// Anonymous usage statistics, through PostHog's EU cloud in cookieless mode.
//
// What this is for: knowing which operations people use and — most of all —
// which ones fail on real geometry. What it deliberately is not: a way to
// recognize anyone. Nothing is stored in the browser (no cookie, no
// localStorage id), PostHog counts visitors with a server-side hash that is
// rotated daily, no session replay, no autocapture of clicks or text, and
// no program content ever leaves the page — events carry an operation's
// *kind*, never its arguments, step ids or sketches.
//
// Off unless `VITE_POSTHOG_KEY` is set at build time, and off for anyone
// who sends Do Not Track / Global Privacy Control or opts out in the
// privacy dialog. The privacy notice (homepage/privacy.html) describes
// exactly this; change the two together.

import type { PostHog } from "posthog-js";
import type { ProgramEdit, RunResult } from "./geop";

const KEY = import.meta.env.VITE_POSTHOG_KEY as string | undefined;
const HOST = "https://eu.i.posthog.com";
/** The viewer's own choice, kept in their browser only — strictly necessary to honour it. */
const OPT_OUT_KEY = "geop-analytics-opt-out";

type Properties = Record<string, string | number | boolean>;

/** PostHog, once loaded. Loaded at most once per page: it cannot be re-initialized. */
let client: PostHog | null = null;
/** This page load's own record of an opt-out, so it holds even where storage is blocked. */
let optedOutNow: boolean | null = null;
/**
 * Events from before PostHog finished loading (it is imported lazily, and
 * e.g. an example opened from the URL fires before that). Only ever filled
 * while statistics are on, and dropped when they are turned off.
 */
let pending: [string, Properties | undefined][] = [];

function signalsOptOut(): boolean {
  const nav = navigator as Navigator & { globalPrivacyControl?: boolean };
  return nav.doNotTrack === "1" || nav.globalPrivacyControl === true;
}

function storedOptOut(): boolean {
  try {
    return localStorage.getItem(OPT_OUT_KEY) === "1";
  } catch {
    return false;
  }
}

/** Why statistics are off, or `null` if they are on. */
export function analyticsStatus(): "unconfigured" | "browser" | "opted-out" | null {
  if (!KEY) return "unconfigured";
  if (signalsOptOut()) return "browser";
  if (optedOutNow ?? storedOptOut()) return "opted-out";
  return null;
}

/** Start collecting, unless anything says not to. Loads PostHog only then. */
export async function initAnalytics(): Promise<void> {
  if (client || analyticsStatus() !== null) return;
  const { default: posthog } = await import("posthog-js");
  if (client) return;
  posthog.init(KEY!, {
    api_host: HOST,
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
    // No remote config or feature flags, and no extra scripts fetched at
    // runtime: what runs is exactly what is configured here.
    advanced_disable_flags: true,
    disable_external_dependency_loading: true,
  });
  client = posthog;
  flush();
}

/**
 * Turn statistics off (and remember that in this browser), or back on.
 *
 * PostHog's own `opt_out_capturing` is ignored in cookieless mode, and with
 * autocapture off nothing is sent except through `track` — so `track` is
 * where an opt-out takes effect, from the moment it is made.
 */
export function setAnalyticsOptOut(optOut: boolean) {
  optedOutNow = optOut;
  try {
    if (optOut) localStorage.setItem(OPT_OUT_KEY, "1");
    else localStorage.removeItem(OPT_OUT_KEY);
  } catch {
    // Storage blocked: the choice holds for this page load only.
  }
  if (optOut) pending = [];
  else void initAnalytics();
}

function flush() {
  if (!client || analyticsStatus() !== null) return;
  for (const [event, properties] of pending) client.capture(event, properties);
  pending = [];
}

function track(event: string, properties?: Properties) {
  if (analyticsStatus() !== null) return;
  pending.push([event, properties]);
  flush();
}

/** A program edit: what kind, and which operation — nothing the user typed. */
export function trackEdit(edit: ProgramEdit) {
  track("program_edit", {
    edit: edit.edit,
    ...("operation" in edit ? { operation: edit.operation } : {}),
  });
}

/**
 * Every step a run reports as failed, once per distinct failure per page
 * load — a program that keeps failing the same way is re-run on every edit
 * after it, and should count once. Only the kernel's root error message
 * goes along: it names what went wrong in the kernel, not what was built.
 */
const reportedFailures = new Set<string>();
export function trackFailures(result: RunResult, operations: string[]) {
  result.results.forEach((step, i) => {
    if (!step.error) return;
    const error = rootError(step.error);
    const operation = operations[i] ?? "unknown";
    const key = `${operation}:${error}`;
    if (reportedFailures.has(key)) return;
    reportedFailures.add(key);
    track("step_failed", { operation, error });
  });
}

/**
 * The `RootError:` line of a `GeopError`'s display, without its context
 * chain and backtrace, and with every entity name redacted: names are built
 * from step ids the user typed (`"extrude(box,end)"`), so they stay here.
 */
function rootError(display: string): string {
  const root = display.match(/RootError: (.*)/)?.[1] ?? display.split("\n")[0];
  return root
    .replace(/"(?:[^"\\]|\\.)*"/g, "<name>")
    .replace(/\b[a-z_]+\([^)]*\)/g, "<name>")
    .slice(0, 200);
}

export function trackExample(name: string) {
  track("example_opened", { name });
}

export function trackFile(action: "saved" | "loaded") {
  track(`program_${action}`);
}
