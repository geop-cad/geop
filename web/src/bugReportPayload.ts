import type { Program, Report, ReportedStep } from "./geop";

/** What a bug report is made of, besides what the kernel says of the session. */
export interface BugReportInput {
  /** What the user wrote, exactly as typed. */
  description: string;
  /** The program as the app has it: sent where the kernel did not answer. */
  program: Program;
  /** What the kernel says of the session (`report`), or `null` where it could not say. */
  report: Report | null;
  /** The last error the app showed, beside the step's own. */
  lastError: string | null;
  /** The commit the app, and its kernel, were built from, if the build knew it. */
  commit: string | null;
  url: string;
  userAgent: string;
  timestamp: string;
}

/**
 * What `/api/bug-report` is sent: the description as typed; the program;
 * the step being edited — its operation, its arguments, its error as its
 * dialog showed it and the field that error concerns — which a failing
 * step never was committed with; the kernel's version and commit; and
 * where it was sent from.
 */
export interface BugReportPayload {
  description: string;
  program: Program;
  step: ReportedStep | null;
  /** The kernel's version, if it answered, and the commit it was built from, if known. */
  kernel: { version: string | null; commit: string | null };
  meta: {
    url: string;
    userAgent: string;
    timestamp: string;
    lastError: string | null;
  };
}

/** The payload of a bug report made of `input`. */
export function bugReportPayload(input: BugReportInput): BugReportPayload {
  const { description, program, report, lastError, commit, url, userAgent, timestamp } = input;
  return {
    description,
    program: report?.program ?? program,
    step: report?.step ?? null,
    kernel: { version: report?.version ?? null, commit },
    meta: { url, userAgent, timestamp, lastError },
  };
}
