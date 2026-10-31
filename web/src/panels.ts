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

/** A table a program is looked up in — see `geop_ops_model::parameters::TableChoice`. */
export interface TableChoice {
  step: string;
  /** The parameter it is looked up by: `configuration` for the program's configurations. */
  input: string;
  rows: string[];
  /** The row built. */
  selected: string;
}

/** What the configurations panel shows — see `geop_ops_model::parameters::ConfigurationsState`. */
export interface ConfigurationsState {
  /** Each table the program is looked up in, its configurations first. */
  tables: TableChoice[];
}

/** What the configurations panel is sent — see `geop_ops_model::parameters::ConfigurationsCommand`. */
export type ConfigurationsCommand =
  | { command: "select"; input: string; row: string }
  | { command: "add"; name?: string | null }
  | { command: "suppress"; step: string };

/** The key of the configurations panel. */
export const CONFIGURATIONS = "configurations";

/**
 * The panel command suppressing the step `step` in the configuration
 * built — or running it again, if it is suppressed: what the timeline's
 * button and Ctrl+E send.
 */
export function suppressStep(step: string): { key: string; payload: ConfigurationsCommand } {
  return { key: CONFIGURATIONS, payload: { command: "suppress", step } };
}
