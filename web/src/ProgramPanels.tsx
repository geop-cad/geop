import { Fragment, type ReactNode } from "react";
import type { JointInfo, Material, MateFreedom, Parameters, ParamValue } from "./geop";
import { JointsPanel } from "./JointsPanel";
import { ParametersPanel } from "./ParametersPanel";

/** What the parameters panel shows — see `geop_ops_parameters::ParametersPanelState`. */
interface ParametersState {
  /** What the parameters resolve to, by name — numbers, a table's row and columns, the colour — and why those that do not resolve fail. */
  parameters: {
    /** What the program's parameters and properties steps define. */
    defined: Parameters;
    values: Record<string, ParamValue>;
    errors: Record<string, string>;
  };
  /** What reads each parameter, by name: the steps and other parameters whose formulas do — what fails if it goes. */
  parameter_uses: Record<string, string[]>;
  /** The materials a part's can be picked from. */
  materials: Material[];
}

/** What the parameters panel is sent — see `geop_ops_parameters::ParametersCommand`. */
type ParametersCommand =
  | { command: "set_parameters"; parameters: Parameters }
  | { command: "rename_parameter"; from: string; to: string };

/** What the joints panel shows — see `geop_ops_assembly::JointsState`. */
interface JointsState {
  /** The joints of the part shown, each coordinate settable. */
  joints: JointInfo[];
  /** How free its placed parts are, if it has any. */
  freedom: MateFreedom | null;
  /** A smallest set of its constraints that cannot all hold together: none where they all hold. */
  conflicting: string[];
}

/** What the joints panel is sent — see `geop_ops_assembly::JointsCommand`. */
type JointsCommand = { command: "set"; parameter: string; value: number };

/** What a panel is shown with: its state — none before the kernel sent it — and how it sends its commands. */
interface PanelProps<State, Payload> {
  state: State | undefined;
  /** Whether the kernel is ready. */
  enabled: boolean;
  /** Whether no step is edited. */
  idle: boolean;
  send: (payload: Payload) => Promise<unknown> | void;
}

/** A panel, shown with what the kernel sent under its key. */
type PanelView = (props: PanelProps<unknown, unknown>) => ReactNode;

/** `view`, shown with what the kernel sent under its key as the state it reads. */
function panel<State, Payload>(view: (props: PanelProps<State, Payload>) => ReactNode): PanelView {
  return (props) => view({ ...props, state: props.state as State | undefined });
}

/**
 * Each panel the kernel may send, by key, in the order shown: the one
 * place the app names a panel. A panel of a key not here is not shown.
 */
const PANELS: Record<string, PanelView> = {
  parameters: panel(({ state, enabled, send }: PanelProps<ParametersState, ParametersCommand>) => (
    <>
      <h2>Parameters</h2>
      <ParametersPanel
        parameters={state?.parameters.defined ?? {}}
        resolved={state?.parameters ?? { values: {}, errors: {} }}
        enabled={enabled}
        materials={state?.materials ?? []}
        uses={state?.parameter_uses ?? {}}
        onChange={(parameters) => send({ command: "set_parameters", parameters })}
        onRename={(from, to) => send({ command: "rename_parameter", from, to })}
      />
    </>
  )),
  joints: panel(({ state, enabled, idle, send }: PanelProps<JointsState, JointsCommand>) => {
    const joints = state?.joints ?? [];
    if (joints.length === 0 && Object.keys(state?.freedom?.parts ?? {}).length === 0) return null;
    return (
      <>
        <h2>{joints.length > 0 ? "Joints" : "Freedom"}</h2>
        <JointsPanel
          joints={joints}
          freedom={state?.freedom ?? null}
          conflicting={state?.conflicting ?? []}
          enabled={enabled && idle}
          onSet={(parameter, value) => send({ command: "set", parameter, value })}
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
  /** Send the panel `key` its command `payload`. */
  onCommand: (key: string, payload: unknown) => Promise<unknown> | void;
}

/** The program's panels, each by the component its key maps to. */
export function ProgramPanels({ panels, enabled, idle, onCommand }: Props) {
  return (
    <>
      {Object.entries(PANELS).map(([key, Panel]) => (
        <Fragment key={key}>
          {Panel({ state: panels[key], enabled, idle, send: (payload) => onCommand(key, payload) })}
        </Fragment>
      ))}
    </>
  );
}
