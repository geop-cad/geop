import { FEATURED, title } from "./examples";

interface Props {
  /** The example parts the kernel has, by name. */
  examples: string[];
  /** The example assemblies (several files) it has, by name. */
  workspaceExamples: string[];
  onExample: (name: string) => void;
  onWorkspaceExample: (name: string) => void;
}

/** What an empty part shows over the view: how to start, and examples to open. */
export function Welcome({ examples, workspaceExamples, onExample, onWorkspaceExample }: Props) {
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
          Pick <strong>Sketch</strong> in the ribbon and click a plane of the origin, then <strong>Extrude</strong>{" "}
          what you drew. Every step stays editable: change a dimension and the part rebuilds.
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
