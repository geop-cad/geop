import type {
  Bom,
  BomLine,
  Bounded,
  EntityRef,
  ExporterInfo,
  Inspection,
  InspectorInfo,
  InterferenceReport,
  MassReport,
  MassSummary,
  Measurement,
} from "./geop";
import { Icon } from "./icons";
import type { Section } from "./section";

interface Props {
  enabled: boolean;
  /** The inspectors the kernel offers: a button each. */
  inspectors: InspectorInfo[];
  /** The exporters it offers: a bill of materials is saved by `bom`'s, if there is one. */
  exporters: ExporterInfo[];
  /** The inspector whose pick tool is in hand, if one is. */
  pickTool: string | null;
  /** Take the pick tool of `inspector` in hand, or put it down. */
  onPickTool: (inspector: string | null) => void;
  /** The pick tool's answer about its picks, while it is in hand. */
  picked: Inspection | null;
  /** The last answers of the inspectors asked of the whole part, by kind — until the part changes. */
  answers: Record<string, unknown>;
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
  onLight: (entities: EntityRef[]) => void;
}

/** A bounded number as shown: its value, and ± its error when that shows at this precision. */
function show(b: Bounded, digits = 3): string {
  const value = b.value.toFixed(digits);
  return b.error >= 0.5 * 10 ** -digits ? `${value} ± ${b.error.toExponential(1)}` : value;
}

/** A mass in kilograms, or grams below one. */
function showMass(b: Bounded): string {
  return b.value < 1 ? `${show({ value: b.value * 1000, error: b.error * 1000 })} g` : `${show(b)} kg`;
}

/** What a line of a bill of materials is, in full: shown when hovered. */
function lineDetails(line: BomLine): string {
  if (line.kind === "wire") {
    return [`cable ${line.cable}`, `${line.colour}`, `Ø${line.diameter} mm`, `cut to ${line.cut_length.value.toFixed(1)} mm`].join("\n");
  }
  return [
    line.file,
    line.parameters,
    line.assumed ? "No material given: weighed as water" : line.material,
    line.thickness.length > 0 ? `sheet ${line.thickness.join(", ")} mm` : "",
    line.unit_mass ? `${showMass(line.unit_mass)} each` : "",
    line.error ?? "",
  ]
    .filter((s) => s !== "")
    .join("\n");
}

/** A bill of materials: per line its item number, quantity, name and designation, and its total mass or length. */
function BomTable({ bom }: { bom: Bom }) {
  return (
    <table className="inspect-table bom-table">
      <thead>
        <tr>
          <th>#</th>
          <th>Qty</th>
          <th>Part</th>
          <th className="bom-number">Total</th>
        </tr>
      </thead>
      <tbody>
        {bom.lines.map((line) => (
          <tr key={line.item} title={lineDetails(line)}>
            <td>{line.item}</td>
            <td>{line.quantity}</td>
            <td style={{ paddingLeft: `${Math.max(0, line.level - 1) * 0.8}rem` }}>
              <div className="bom-name">{line.name}</div>
              {line.designation && <div className="hint">{line.designation}</div>}
              {line.kind === "part" && line.assumed && <div className="hint">no material</div>}
            </td>
            <td className="bom-number">
              {line.kind === "wire"
                ? `${line.total_length.value.toFixed(0)} mm`
                : line.total_mass
                  ? showMass(line.total_mass)
                  : "?"}
            </td>
          </tr>
        ))}
      </tbody>
      {bom.total_mass && (
        <tfoot>
          <tr>
            <th colSpan={3}>Total</th>
            <td className="bom-number">{showMass(bom.total_mass)}</td>
          </tr>
        </tfoot>
      )}
    </table>
  );
}

function Summary({ summary }: { summary: MassSummary }) {
  const axes = ["x", "y", "z"];
  return (
    <table className="inspect-table">
      <tbody>
        <tr>
          <th>Mass</th>
          <td>{showMass(summary.mass)}</td>
        </tr>
        <tr>
          <th>Volume</th>
          <td>{show(summary.volume)} mm³</td>
        </tr>
        <tr>
          <th>Area</th>
          <td>{show(summary.area)} mm²</td>
        </tr>
        <tr>
          <th>Centre</th>
          <td>{summary.center.map((c) => show(c)).join(", ")} mm</td>
        </tr>
        <tr>
          <th title="About the centre of mass, along the world's axes">Inertia</th>
          <td>
            {summary.inertia.map((row, i) => (
              <div key={i}>
                {axes[i]}: {row.map((c) => c.value.toPrecision(5)).join("  ")}
              </div>
            ))}
            <span className="hint">kg·mm²</span>
          </td>
        </tr>
        <tr>
          <th>Principal</th>
          <td>
            {summary.principal_moments.map((m, i) => (
              <div key={i} title={`about (${summary.principal_axes[i].map((x) => x.toFixed(3)).join(", ")})`}>
                {show(m, 4)}
              </div>
            ))}
            <span className="hint">kg·mm²</span>
          </td>
        </tr>
      </tbody>
    </table>
  );
}

/** What the measure tool's picks measure. */
function MeasurementView({ measurement }: { measurement: Measurement }) {
  return (
    <>
      {measurement.entities.length === 0 && <p className="hint">Click a vertex, edge, face or datum; a second one to measure between them.</p>}
      {measurement.entities.map((e, i) => (
        <div key={i} className="inspect-entity" title={"name" in e ? e.name : ""}>
          {e.type} {"name" in e ? e.name : ""}
        </div>
      ))}
      {measurement.error && <p className="error">{measurement.error}</p>}
      <table className="inspect-table">
        <tbody>
          {measurement.values.map((v, i) => (
            <tr key={i}>
              <th>{v.label}</th>
              <td>
                {show(v)} {v.unit}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </>
  );
}

/** The mass properties of every solid, and of all of them. */
function MassView({ mass, onLight }: { mass: MassReport; onLight: (entities: EntityRef[]) => void }) {
  return (
    <>
      {mass.bodies.length === 0 && <p className="hint">No solids.</p>}
      {mass.total && mass.bodies.length > 1 && (
        <details open>
          <summary>All {mass.bodies.length} solids</summary>
          <Summary summary={mass.total} />
        </details>
      )}
      {mass.bodies.map((body) => (
        <details key={body.name} open={mass.bodies.length === 1}>
          <summary title={body.name} onMouseEnter={() => onLight([solid(body.name)])} onMouseLeave={() => onLight([])}>
            {body.name} <span className="hint">{body.material}</span>
          </summary>
          {body.assumed && <p className="hint">No material given: weighed as water. Set one under Parameters.</p>}
          {body.error && <p className="error">{body.error}</p>}
          {body.properties && <Summary summary={body.properties} />}
        </details>
      ))}
    </>
  );
}

/** Which solids overlap, and which touch. */
function InterferenceView({ report, onLight }: { report: InterferenceReport; onLight: (entities: EntityRef[]) => void }) {
  return (
    <>
      {report.found.length === 0 && report.unchecked.length === 0 && (
        <p className="hint">
          No interference among {report.solids} solid{report.solids === 1 ? "" : "s"}.
        </p>
      )}
      <ul className="inspect-list">
        {report.found.map((f, i) => (
          <li key={i} className={f.contact} onMouseEnter={() => onLight([solid(f.a), solid(f.b)])} onMouseLeave={() => onLight([])}>
            {f.a} × {f.b}: {f.contact === "overlap" ? `overlap ${f.volume ? show(f.volume) : "?"} mm³` : "touch"}
          </li>
        ))}
        {report.unchecked.map((u, i) => (
          <li key={`u${i}`} className="unchecked" title={u.error}>
            {u.a} × {u.b}: could not be checked
          </li>
        ))}
      </ul>
    </>
  );
}

/** The solid `name`, to light. */
function solid(name: string): EntityRef {
  return { type: "Solid", name };
}

/**
 * The inspect tools: the inspectors the kernel offers — those that pick
 * as tools to take in hand, the others as questions about the part as
 * drawn — and a section view. None of them is a step: inspecting changes
 * nothing, and a section only changes the view. The answers the app knows
 * are shown as such ([[Inspection]]); any other as it comes.
 */
export function InspectPanel({
  enabled,
  inspectors,
  exporters,
  pickTool,
  onPickTool,
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
  const measurement = picked?.kind === "measure" ? (picked.report as Measurement) : null;
  const bom = answers.bom as Bom | undefined;
  return (
    <div className="inspect">
      <div className="inspect-tools">
        {inspectors.map((inspector) => {
          const picks = inspector.picks != null;
          const active = picks && pickTool === inspector.kind;
          return (
            <button
              key={inspector.kind}
              className={["op-button", active ? "active" : ""].join(" ")}
              disabled={!enabled || (!picks && busy)}
              title={inspector.doc}
              onClick={() =>
                (picks ? onPickTool(active ? null : inspector.kind) : onInspect(inspector.kind))
              }
            >
              <Icon name={inspector.kind} />
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
          {measurement ? <MeasurementView measurement={measurement} /> : <pre className="inspect-report">{JSON.stringify(picked.report, null, 2)}</pre>}
        </div>
      )}

      {section && (
        <div className="inspect-result">
          <div className="inspect-row">
            {(["x", "y", "z", "picked"] as const).map((axis) => (
              <button
                key={axis}
                className={["small", section.axis === axis ? "active" : ""].join(" ")}
                disabled={axis === "picked" && !measurement?.plane}
                title={axis === "picked" ? "The plane of the face or datum plane picked with Measure" : `Across ${axis}`}
                onClick={() =>
                  onSection({ ...section, axis, offset: 0, plane: axis === "picked" ? (measurement?.plane ?? null) : null })
                }
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

      {Object.entries(answers).map(([kind, report]) => (
        <div key={kind} className="inspect-result">
          {kind === "mass_properties" ? (
            <MassView mass={report as MassReport} onLight={onLight} />
          ) : kind === "interference" ? (
            <InterferenceView report={report as InterferenceReport} onLight={onLight} />
          ) : kind === "bom" ? (
            <>
              <div className="inspect-row">
                {(["flat", "indented"] as const).map((structure) => (
                  <button
                    key={structure}
                    className={["small", bom?.structure === structure ? "active" : ""].join(" ")}
                    disabled={!enabled || busy}
                    title={structure === "flat" ? "Every part once, counted in the whole assembly" : "By sub-assembly, counted per assembly"}
                    onClick={() => onInspect("bom", { structure })}
                  >
                    {structure === "flat" ? "Flat" : "Indented"}
                  </button>
                ))}
                {exporters.some((e) => e.kind === "bom") && (
                  <button
                    className="small"
                    disabled={!enabled}
                    title="Save the bill of materials as a CSV file"
                    onClick={() => onExport("bom", { structure: bom?.structure ?? "flat" })}
                  >
                    Save CSV
                  </button>
                )}
              </div>
              {(report as Bom).lines.length === 0 ? <p className="hint">Nothing to list.</p> : <BomTable bom={report as Bom} />}
            </>
          ) : (
            <pre className="inspect-report">{JSON.stringify(report, null, 2)}</pre>
          )}
        </div>
      ))}
    </div>
  );
}
