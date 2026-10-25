import type { AnswerRow, Block, PickedElement, Inspection, InspectorInfo, ToolInfo } from "./geop";
import { Icon } from "./icons";
import type { Section } from "./section";

interface Props {
  enabled: boolean;
  /** The tools the kernel offers to take in hand — the drag tool — none of them an operation: a button each. */
  tools: ToolInfo[];
  /** The key of the tool in hand, if one is. */
  inHand: string | null;
  /** Take the tool `key` in hand, or put the one in hand down. */
  onTool: (key: string | null) => void;
  /** The inspectors the kernel offers: a button each. */
  inspectors: InspectorInfo[];
  /** The inspector whose pick tool is in hand, if one is. */
  picking: string | null;
  /** Take the pick tool of `inspector` in hand, or put it down. */
  onPicking: (inspector: string | null) => void;
  /** The pick tool's answer about its picks, while it is in hand. */
  picked: Inspection | null;
  /** The last answers of the inspectors asked of the whole part, by kind — until the part changes. */
  answers: Record<string, Inspection>;
  /** Whether a question is being answered. */
  busy: boolean;
  /** Ask `inspector` about the part as drawn, told `options`. */
  onInspect: (inspector: string, options?: unknown) => void;
  /** Save what `exporter` writes, told `options`. */
  onExport: (exporter: string, options?: unknown) => void;
  section: Section | null;
  onSection: (section: Section | null) => void;
  /** How big the part is: how far a section can move. */
  size: number;
  /** Light these entities in the view, or nothing. */
  onLight: (entities: PickedElement[]) => void;
}

/** What an answer's parts are given: what to do on a button, and what to light. */
interface Handlers {
  /** The inspector that answered: what its buttons ask again. */
  kind: string;
  enabled: boolean;
  busy: boolean;
  onInspect: (inspector: string, options?: unknown) => void;
  onExport: (exporter: string, options?: unknown) => void;
  onLight: (entities: PickedElement[]) => void;
}

/** What a row does while pointed at: says all it says, and lights what it is of. */
function pointing(row: { detail?: string | null; light: PickedElement[] }, onLight: (entities: PickedElement[]) => void) {
  return {
    title: row.detail ?? undefined,
    onMouseEnter: row.light.length > 0 ? () => onLight(row.light) : undefined,
    onMouseLeave: row.light.length > 0 ? () => onLight([]) : undefined,
  };
}

function TableRow({ row, onLight }: { row: AnswerRow; onLight: (entities: PickedElement[]) => void }) {
  return (
    <tr className={row.tone} {...pointing(row, onLight)}>
      {row.label != null && <th>{row.label}</th>}
      {row.cells.map((cell, i) => (
        <td key={i}>{cell}</td>
      ))}
    </tr>
  );
}

/** One part of an answer, as the kernel built it. */
function BlockView({ block, h }: { block: Block; h: Handlers }) {
  switch (block.type) {
    case "text":
      return <p className={block.tone === "error" ? "error" : "hint"}>{block.text}</p>;
    case "list":
      return (
        <ul className="inspect-list">
          {block.items.map((item, i) => (
            <li key={i} className={item.tone} {...pointing(item, h.onLight)}>
              {[item.label, ...item.cells].filter((c) => c != null).join(" ")}
            </li>
          ))}
        </ul>
      );
    case "table":
      return (
        <table className="inspect-table">
          {block.head.length > 0 && (
            <thead>
              <tr>
                {block.head.map((cell, i) => (
                  <th key={i}>{cell}</th>
                ))}
              </tr>
            </thead>
          )}
          <tbody>
            {block.rows.map((row, i) => (
              <TableRow key={i} row={row} onLight={h.onLight} />
            ))}
          </tbody>
          {block.foot.length > 0 && (
            <tfoot>
              {block.foot.map((row, i) => (
                <TableRow key={i} row={row} onLight={h.onLight} />
              ))}
            </tfoot>
          )}
        </table>
      );
    case "group":
      return (
        <details open={block.open}>
          <summary {...pointing({ detail: block.title, light: block.light }, h.onLight)}>
            {block.title} {block.detail && <span className="hint">{block.detail}</span>}
          </summary>
          {block.blocks.map((b, i) => (
            <BlockView key={i} block={b} h={h} />
          ))}
        </details>
      );
    case "actions":
      return (
        <div className="inspect-row">
          {block.actions.map((action, i) => (
            <button
              key={i}
              className={["small", action.chosen ? "active" : ""].join(" ")}
              disabled={!h.enabled || (action.does.type === "inspect" && h.busy)}
              title={action.doc}
              onClick={() =>
                action.does.type === "inspect"
                  ? h.onInspect(h.kind, action.does.options)
                  : h.onExport(action.does.exporter, action.does.options)
              }
            >
              {action.label}
            </button>
          ))}
        </div>
      );
  }
}

/** An inspector's answer: as it is shown, or — showing nothing — its report as it comes. */
function AnswerView({ inspection, h }: { inspection: Inspection; h: Omit<Handlers, "kind"> }) {
  const blocks = inspection.shown.blocks;
  if (blocks.length === 0) return <pre className="inspect-report">{JSON.stringify(inspection.report, null, 2)}</pre>;
  return (
    <>
      {blocks.map((block, i) => (
        <BlockView key={i} block={block} h={{ ...h, kind: inspection.kind }} />
      ))}
    </>
  );
}

/**
 * The inspect tools: the tools the kernel offers to take in hand — the
 * drag tool, which moves placed parts as their mates let them — the
 * inspectors, those that pick as tools to take in hand, the others as
 * questions about the part as drawn, and a section view. None of them is a
 * step: a tool and an inspection add nothing to the program, and a section
 * only changes the view. Every answer is shown as the kernel built it
 * ([[Answer]]), whatever the inspector.
 */
export function InspectPanel({
  enabled,
  tools,
  inHand,
  onTool,
  inspectors,
  picking,
  onPicking,
  picked,
  answers,
  busy,
  onInspect,
  onExport,
  section,
  onSection,
  size,
  onLight,
}: Props) {
  const plane = picked?.shown.plane ?? null;
  const h = { enabled, busy, onInspect, onExport, onLight };
  return (
    <div className="inspect">
      <div className="inspect-tools">
        {tools.map((tool) => {
          const active = inHand === tool.key;
          return (
            <button
              key={tool.key}
              className={["op-button", active ? "active" : ""].join(" ")}
              disabled={!enabled}
              title={tool.doc}
              onClick={() => onTool(active ? null : tool.key)}
            >
              {tool.icon && <Icon name={tool.icon} />}
              <span>{tool.label}</span>
            </button>
          );
        })}
        {inspectors.map((inspector) => {
          const picks = inspector.picks != null;
          const active = picks && picking === inspector.kind;
          return (
            <button
              key={inspector.kind}
              className={["op-button", active ? "active" : ""].join(" ")}
              disabled={!enabled || (!picks && busy)}
              title={inspector.doc}
              onClick={() =>
                (picks ? onPicking(active ? null : inspector.kind) : onInspect(inspector.kind))
              }
            >
              {inspector.icon && <Icon name={inspector.icon} />}
              <span>{inspector.label}</span>
            </button>
          );
        })}
        <button
          className={["op-button", section ? "active" : ""].join(" ")}
          disabled={!enabled}
          title="Section view: cut the view along a plane"
          onClick={() => onSection(section ? null : { axis: "z", offset: 0, flip: false, plane: null })}
        >
          <Icon name="section" />
          <span>Section</span>
        </button>
      </div>

      {picked && (
        <div className="inspect-result">
          <AnswerView inspection={picked} h={h} />
        </div>
      )}

      {section && (
        <div className="inspect-result">
          <div className="inspect-row">
            {(["x", "y", "z", "picked"] as const).map((axis) => (
              <button
                key={axis}
                className={["small", section.axis === axis ? "active" : ""].join(" ")}
                disabled={axis === "picked" && !plane}
                title={axis === "picked" ? "The plane of the face or datum plane picked to inspect" : `Across ${axis}`}
                onClick={() => onSection({ ...section, axis, offset: 0, plane: axis === "picked" ? plane : null })}
              >
                {axis === "picked" ? "Picked plane" : axis.toUpperCase()}
              </button>
            ))}
            <button className="small" title="Cut away the other side" onClick={() => onSection({ ...section, flip: !section.flip })}>
              Flip
            </button>
          </div>
          <input
            type="range"
            min={-size / 2}
            max={size / 2}
            step={size / 500}
            value={section.offset}
            aria-label="Where the section cuts"
            onChange={(e) => onSection({ ...section, offset: Number(e.target.value) })}
          />
          <span className="hint">{section.offset.toFixed(2)} mm</span>
        </div>
      )}

      {Object.entries(answers).map(([kind, inspection]) => (
        <div key={kind} className="inspect-result">
          <AnswerView inspection={inspection} h={h} />
        </div>
      ))}
    </div>
  );
}
