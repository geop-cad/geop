// What the kernel's panels send, by the key the kernel sends them under: the
// states `ProgramPanels` shows, and what else reads them.

import type { JointInfo, MateFreedom } from "./geop";

/** What the joints panel shows — see `geop_ops_assembly::JointsState`. */
export interface JointsState {
  /** The joints of the part shown, each coordinate settable. */
  joints: JointInfo[];
  /** How free its placed parts are, if it has any. */
  freedom: MateFreedom | null;
  /** A smallest set of its constraints that cannot all hold together: none where they all hold. */
  conflicting: string[];
}

/** What the joints panel is sent — see `geop_ops_assembly::JointsCommand`. */
export type JointsCommand = { command: "set"; parameter: string; value: number };

/** A motion study as the motion panel lists it — see `geop_ops_assembly::StudyState`. */
export interface StudyState {
  /** The id of its step. */
  name: string;
  /** When it starts and ends, in seconds. */
  start: number;
  end: number;
  frames: number;
  /** The joint coordinates it drives. */
  drivers: string[];
}

/** Two solids found overlapping at a time of a motion. */
export interface Collision {
  time: number;
  a: string;
  b: string;
}

/** What the motion panel shows — see `geop_ops_assembly::MotionState`. */
export interface MotionState {
  studies: StudyState[];
  /** The study played last, and the time it was shown at. */
  shown: [string, number] | null;
  /** What the last check of a study found. */
  checked: { study: string; collisions: Collision[] } | null;
}

/** What the motion panel is sent — see `geop_ops_assembly::MotionCommand`. */
export type MotionCommand =
  | { command: "seek"; study: string; time: number }
  | { command: "check"; study: string };
