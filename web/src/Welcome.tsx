import { useEffect, useRef, type KeyboardEvent } from "react";
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
  onClose: () => void;
  /** Told how many pixels at the bottom of the view the welcome lies over, as that changes; 0 once it is gone. */
  onCover: (pixels: number) => void;
}

/**
 * What an empty part shows over the view: how to start, and examples to
 * open — the arrows move between them, Enter opens one, Escape closes
 * the welcome.
 */
export function Welcome({ examples, workspaceExamples, onExample, onWorkspaceExample, keyOf, onClose, onCover }: Props) {
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
  const list = useRef<HTMLDivElement>(null);
  const card = useRef<HTMLDivElement>(null);
  // The first example has the focus, unless something else took it: the
  // keys reach the examples at once.
  useEffect(() => {
    if (document.activeElement === document.body || document.activeElement == null)
      list.current?.querySelector<HTMLButtonElement>("button")?.focus({ preventScroll: true });
  }, []);
  // What it covers of the view, from the card's top down: the view centres
  // itself above (see `SceneViewer`'s `covered`), so the origin, which a
  // first sketch is started on, is never under it.
  const onCoverRef = useRef(onCover);
  onCoverRef.current = onCover;
  useEffect(() => {
    const el = card.current;
    const view = el?.closest(".viewport");
    if (!el || !view) return;
    const measure = () =>
      onCoverRef.current(Math.max(0, Math.round(view.getBoundingClientRect().bottom - el.getBoundingClientRect().top)));
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(el);
    observer.observe(view);
    return () => {
      observer.disconnect();
      onCoverRef.current(0);
    };
  }, []);

  const onKey = (e: KeyboardEvent<HTMLDivElement>) => {
    if (e.key === "Escape") {
      e.preventDefault();
      onClose();
      return;
    }
    const buttons = [...(list.current?.querySelectorAll<HTMLButtonElement>("button") ?? [])];
    const at = buttons.indexOf(document.activeElement as HTMLButtonElement);
    if (at < 0) return;
    // Left and right step by one, up and down by a row: as many as share the first's.
    const row = buttons.filter((b) => b.offsetTop === buttons[0].offsetTop).length;
    const step = { ArrowLeft: -1, ArrowRight: 1, ArrowUp: -row, ArrowDown: row }[e.key];
    if (step == null) return;
    e.preventDefault();
    buttons[Math.max(0, Math.min(buttons.length - 1, at + step))].focus();
  };

  return (
    <div className="welcome">
      <div className="welcome-card" role="dialog" aria-label="Welcome" onKeyDown={onKey} ref={card}>
        <button className="welcome-close" aria-label="Close" title="Close (Esc)" onClick={onClose}>
          ×
        </button>
        <h2>Start a part</h2>
        <p>
          Pick <strong>Sketch</strong> {key("add_sketch")} in the ribbon and click a plane of the origin, then{" "}
          <strong>Extrude</strong> {key("extrude")} what you drew: every step stays editable.
          {featured.length > 0 && " Or open an example:"}
        </p>
        {featured.length > 0 && (
          <div className="welcome-examples" ref={list}>
            {featured.map(({ name, open }) => (
              <button key={name} className="welcome-example" onClick={open}>
                <span className="welcome-example-name">{title(name)}</span>
              </button>
            ))}
          </div>
        )}
        <p className="welcome-more">
          More under File ▸ Example parts · {key("palette")} finds any command · {key("keys")} lists the keys ·{" "}
          {key("fit")} frames the part
        </p>
      </div>
    </div>
  );
}
