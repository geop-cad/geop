import type { Bounded, EntityRef, InterferenceReport, MassReport, MassSummary, Measurement } from "./geop";
import { Icon } from "./icons";
import type { Section } from "./section";

interface Props {
  enabled: boolean;
  measuring: boolean;
  onMeasure: (on: boolean) => void;
  measurement: Measurement | null;
  mass: MassReport | null;
  interference: InterferenceReport | null;
  /** Whether a question is being answered. */
  busy: boolean;
  onQuery: (query: "mass_properties" | "interference") => void;
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

/**
 * The inspect tools: measure what is picked, the mass properties and the
 * interference of the part as drawn, and a section view. None of them is a
 * step: measuring and asking change nothing, and a section only changes the
 * view.
 */
export function InspectPanel({
  enabled,
  measuring,
  onMeasure,
  measurement,
  mass,
  interference,
  busy,
  onQuery,
  section,
  onSection,
  size,
  onLight,
}: Props) {
  const solid = (name: string): EntityRef => ({ type: "Solid", name });
  return (
    <div className="inspect">
      <div className="inspect-tools">
        <button
          className={["op-button", measuring ? "active" : ""].join(" ")}
          disabled={!enabled}
          title="Measure: click up to two vertices, edges, faces or datums"
          onClick={() => onMeasure(!measuring)}
        >
          <Icon name="measure" />
          <span>Measure</span>
        </button>
        <button
          className="op-button"
          disabled={!enabled || busy}
          title="Mass properties of every solid, of its part's material"
          onClick={() => onQuery("mass_properties")}
        >
          <Icon name="mass" />
          <span>Mass</span>
        </button>
        <button
          className="op-button"
          disabled={!enabled || busy}
          title="Which solids overlap, and which touch"
          onClick={() => onQuery("interference")}
        >
          <Icon name="interference" />
          <span>Interference</span>
        </button>
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

      {measuring && measurement && (
        <div className="inspect-result">
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

      {mass && (
        <div className="inspect-result">
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
        </div>
      )}

      {interference && (
        <div className="inspect-result">
          {interference.found.length === 0 && interference.unchecked.length === 0 && (
            <p className="hint">
              No interference among {interference.solids} solid{interference.solids === 1 ? "" : "s"}.
            </p>
          )}
          <ul className="inspect-list">
            {interference.found.map((f, i) => (
              <li
                key={i}
                className={f.contact}
                onMouseEnter={() => onLight([solid(f.a), solid(f.b)])}
                onMouseLeave={() => onLight([])}
              >
                {f.a} × {f.b}: {f.contact === "overlap" ? `overlap ${f.volume ? show(f.volume) : "?"} mm³` : "touch"}
              </li>
            ))}
            {interference.unchecked.map((u, i) => (
              <li key={`u${i}`} className="unchecked" title={u.error}>
                {u.a} × {u.b}: could not be checked
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}
