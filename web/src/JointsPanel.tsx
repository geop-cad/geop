import type { JointInfo, MateFreedom } from "./geop";

interface Props {
  joints: JointInfo[];
  freedom: MateFreedom | null;
  enabled: boolean;
  /** The joint coordinate `parameter` is set to `value`: the parts move to it. */
  onSet: (parameter: string, value: number) => void;
}

/** What a joint's name says of it: `add_part(fore,m1)` is the forearm's mate `m1`. */
function label(name: string): string {
  const m = /^add_part\(([^,]+),([^)]+)\)$/.exec(name);
  return m ? `${m[1]} ${m[2]}` : name;
}

/**
 * The program's joints, each coordinate set by typing it — or, limited, on a
 * slider between its limits — and how free each placed part still is.
 */
export function JointsPanel({ joints, freedom, enabled, onSet }: Props) {
  return (
    <div className="parameters">
      {joints.flatMap((joint) =>
        joint.values.map((v) => {
          const unit = v.motion === "angle" ? "°" : "";
          const give = (e: { currentTarget: HTMLInputElement }) => {
            const value = Number(e.currentTarget.value);
            if (Number.isFinite(value) && value !== v.value)
              onSet(v.parameter, value);
          };
          return (
            <div
              className="parameter-row"
              key={v.parameter}
              title={`${joint.kind} joint ${joint.name}: ${v.motion}${joint.between.length > 0 ? ` — ${joint.between.join(" ↔ ")}` : ""}`}
            >
              <span className="parameter-name">{label(joint.name)}</span>
              {v.min != null && v.max != null && (
                <input
                  type="range"
                  className="parameter-formula"
                  min={v.min}
                  max={v.max}
                  step={v.motion === "angle" ? 1 : 0.01}
                  value={v.value}
                  disabled={!enabled}
                  onChange={(e) =>
                    onSet(v.parameter, Number(e.currentTarget.value))
                  }
                />
              )}
              <input
                key={v.value}
                type="number"
                className="parameter-formula"
                defaultValue={Number(v.value.toFixed(4))}
                disabled={!enabled}
                onBlur={give}
                onKeyDown={(e) => {
                  if (e.key === "Enter") give(e);
                }}
              />
              <span className="parameter-value">{unit || v.motion}</span>
            </div>
          );
        }),
      )}
      {freedom && freedom.conflicting.length > 0 && (
        <div className="parameter-row failed">
          <span className="parameter-value">
            cannot all hold: {freedom.conflicting.join(", ")}
          </span>
        </div>
      )}
      {freedom &&
        Object.entries(freedom.parts).map(([part, dof]) => (
          <div
            className="parameter-row"
            key={part}
            title="How many independent ways it can still move"
          >
            <span className="parameter-name">{part}</span>
            <span className="parameter-value">
              {dof === 0 ? "held fast" : `${dof} DOF`}
            </span>
          </div>
        ))}
    </div>
  );
}
