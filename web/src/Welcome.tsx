import { FEATURED, title } from "./examples";
import { formatShortcut } from "./shortcuts";

interface Props {
  /** The example parts the kernel has, by name. */
  examples: string[];
  /** The example assemblies (several files) it has, by name. */
  workspaceExamples: string[];
  onExample: (name: string) => void;
  onWorkspaceExample: (name: string) => void;
  /** The key of the operation `kind`, or of the app's `action`, as the kernel names it, if it has one. */
  keyOf: (kindOrAction: string) => string | null;
}

/** What an empty part shows over the view: how to start, and examples to open. */
export function Welcome({ examples, workspaceExamples, onExample, onWorkspaceExample, keyOf }: Props) {
  /** ` (S)`: the key of `kind`, as the platform writes it. */
  const key = (kind: string) => {
    const k = keyOf(kind);
    return k ? <kbd>{formatShortcut(k)}</kbd> : null;
  };
  const featured = FEATURED.flatMap((name) =>
    examples.includes(name)
      ? [{ name, open: () => onExample(name) }]
      : workspaceExamples.includes(name)
        ? [{ name, open: () => onWorkspaceExample(name) }]
        : [],
  );
  return (
    <div className="welcome">
      <div className="welcome-card">
        <h2>Start a part</h2>
        <p>
          Pick <strong>Sketch</strong> {key("add_sketch")} in the ribbon and click a plane of the origin, then{" "}
          <strong>Extrude</strong> {key("extrude")} what you drew. Every step stays editable: change a dimension and the
          part rebuilds.
        </p>
        <p className="welcome-keys">
          {key("palette")} finds any command · {key("keys")} lists the keys · {key("fit")} frames the part
        </p>
        {featured.length > 0 && (
          <>
            <h3>Or open an example</h3>
            <div className="welcome-examples">
              {featured.map(({ name, open }) => (
                <button key={name} onClick={open}>
                  {title(name)}
                </button>
              ))}
            </div>
          </>
        )}
        <p className="welcome-more">More under File ▸ Example parts.</p>
      </div>
    </div>
  );
}
