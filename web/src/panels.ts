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
