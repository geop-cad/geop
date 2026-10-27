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
