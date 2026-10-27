import { Fragment, type ReactNode } from "react";
import type { Update } from "./geop";
import { JointsPanel } from "./JointsPanel";
import { MotionPanel } from "./MotionPanel";
import type { JointsCommand, JointsState, MotionCommand, MotionState } from "./panels";

/** What a panel is shown with: its state — none before the kernel sent it — and how it sends its commands. */
interface PanelProps<State, Payload> {
  state: State | undefined;
  /** Whether the kernel is ready. */
  enabled: boolean;
  /** Whether no step is edited. */
  idle: boolean;
  /** Send the panel's command; what the kernel shows after it. */
  send: (payload: Payload) => Promise<Update | null>;
  /** Open the step `id` for editing: what a panel does for what a step's dialog edits. */
  openStep: (id: string) => void;
}

/** A panel, shown with what the kernel sent under its key. */
type PanelView = (props: PanelProps<unknown, unknown>) => ReactNode;

/** `view`, shown with what the kernel sent under its key as the state it reads. */
function panel<State, Payload>(view: (props: PanelProps<State, Payload>) => ReactNode): PanelView {
  return (props) => view({ ...props, state: props.state as State | undefined });
}

/**
 * Each panel the kernel may send, by key, in the order shown: with what
 * `panels.ts` reads of their states, the one place the app names a panel.
 * A panel of a key not here is not shown.
 */
const PANELS: Record<string, PanelView> = {
  joints: panel(({ state, enabled, idle, send }: PanelProps<JointsState, JointsCommand>) => {
    const joints = state?.joints ?? [];
    const conflicting = state?.conflicting ?? [];
    if (joints.length === 0 && conflicting.length === 0) return null;
    return (
      <>
        <h2>{joints.length > 0 ? "Joints" : "Mates"}</h2>
        <JointsPanel
          joints={joints}
          conflicting={conflicting}
          enabled={enabled && idle}
          onSet={(parameter, value) => send({ command: "set", parameter, value })}
        />
      </>
    );
  }),
  motion: panel(({ state, enabled, idle, send }: PanelProps<MotionState, MotionCommand>) => {
    if (!state || state.studies.length === 0) return null;
    return (
      <>
        <h2>Motion</h2>
        <MotionPanel
          state={state}
          enabled={enabled && idle}
          onSeek={(study, time) => send({ command: "seek", study, time })}
          onCheck={(study) => void send({ command: "check", study })}
        />
      </>
    );
  }),
};

interface Props {
  /** What each panel shows, by key, as the kernel sent it. */
  panels: Record<string, unknown>;
  enabled: boolean;
  idle: boolean;
  /** Send the panel `key` its command `payload`; what the kernel shows after it. */
  onCommand: (key: string, payload: unknown) => Promise<Update | null>;
  /** Open the step `id` for editing. */
  onOpenStep: (id: string) => void;
}

/** The program's panels, each by the component its key maps to. */
export function ProgramPanels({ panels, enabled, idle, onCommand, onOpenStep }: Props) {
  return (
    <>
      {Object.entries(PANELS).map(([key, Panel]) => (
        <Fragment key={key}>
          {Panel({ state: panels[key], enabled, idle, send: (payload) => onCommand(key, payload), openStep: onOpenStep })}
        </Fragment>
      ))}
    </>
  );
}
