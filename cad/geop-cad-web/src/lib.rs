//! WebAssembly bindings for the browser editor.
//!
//! The editor edits a `geop_ops_parts::Program`, and only through this
//! module: [`update_program`] applies a [`ProgramEdit`] with
//! [`Program::update`] — the one way any editor changes a program — and
//! [`run_program`] builds it with a [`ProgramRunner`], as far as the editor
//! asks (while a step in the middle is being edited, the steps after it
//! need not run). [`operation_schemas`] describes every operation, so the
//! editor builds its forms from them instead of knowing each by hand.
//!
//! Everything crosses the boundary as JSON: programs and edits in their own
//! serde format, scenes as flat number arrays a `three.js` viewer consumes
//! directly. Entities are named, never numbered — a pick returns the name
//! of what was hit, which is what a program step refers to it by.

use std::cell::RefCell;

use geop_cad_base::pick::{
    PickFilter, PickKind, Ray, SketchTargets, pick as pick_model, pick_sketch,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Color10, CoordinateSystem},
    scalars::{Scalar, scal_in_f64::ScalInF64},
    vector::Vector3,
};
use geop_core_part::{DatumKind, Part, RefId};
use geop_core_sketch::{CurveKind, PointId, Sketch, SolveReport};
use geop_core_topology::{EdgeId, FaceId, SolidId, VertexId};
use geop_ops_parts::{
    EntityRef, PartOperation, Program, ProgramEdit, ProgramRunner, StepHandle, StepResult,
    examples,
    operation::{inspect_selection, resolve_plane},
};
use geop_ops_rasterize::{RasterizedModel, rasterize_model_tagged};
use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

/// The scalar type used for every wasm-exposed part.
type S = ScalInF64;

thread_local! {
    /// The program being edited. Changed only by [`update_program`].
    static PROGRAM: RefCell<Program> = RefCell::new(Program::new());
    /// Builds [`PROGRAM`]. Picks and sketch planes resolve against the part
    /// of its most recent run — never a preview's, so nothing can refer to
    /// geometry that only a not-yet-made edit would create.
    static COMMITTED: RefCell<ProgramRunner<S>> = RefCell::new(ProgramRunner::new());
    /// How the part of `COMMITTED`'s most recent run is drawn: sampled once
    /// per run, and what every pick tests against — so a pick is cheap
    /// enough for hovering, and hits exactly what is on screen.
    static VIEW: RefCell<Option<View>> = const { RefCell::new(None) };
    /// Builds previews: the program with one edit not made yet. Its own
    /// cache, so re-previewing as a slider moves replays only the edited
    /// step.
    static PREVIEW: RefCell<ProgramRunner<S>> = RefCell::new(ProgramRunner::new());
}

fn to_js_err(e: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&e.to_string())
}

fn from_json<T: for<'de> Deserialize<'de>>(json: &str, what: &str) -> GeopResult<T> {
    serde_json::from_str(json).map_err(|e| GeopError::new(format!("reading {what}: {e}")))
}

fn to_json(value: &impl Serialize) -> GeopResult<String> {
    serde_json::to_string(value).map_err(|e| GeopError::new(format!("writing JSON: {e}")))
}

// ── scene ────────────────────────────────────────────────────────────────────

/// Samples per edge and per parametric direction of a face for the scene
/// the browser renders. A face's grid is refined further where its own
/// curvature asks for it (see `geop_ops_rasterize::grid`), so this is the
/// floor, not the ceiling.
const SCENE_RESOLUTION: usize = 24;

/// Sketch curves in the scene: profile geometry and construction geometry.
const SKETCH_COLOR: u32 = 0xffa040;
const CONSTRUCTION_COLOR: u32 = 0x808080;

/// A part as it is drawn: its topology sampled into points, polylines and
/// triangles, and its sketches outlined. What is drawn is also what picks
/// test against.
struct View {
    raster: RasterizedModel<S>,
    sketches: SketchTargets<S>,
}

impl View {
    fn of(part: &Part<S>) -> GeopResult<Self> {
        Ok(View {
            raster: rasterize_model_tagged(part.topology(), SCENE_RESOLUTION)?,
            sketches: SketchTargets::of(part),
        })
    }
}

#[derive(Serialize)]
struct FaceTag {
    name: String,
    /// The name of the solid the face bounds.
    solid: Option<String>,
}

#[derive(Serialize)]
struct SceneJson {
    /// `[x, y, z, colorHex]` per point.
    points: Vec<[f64; 4]>,
    /// Per point: the name of the vertex it draws.
    point_names: Vec<String>,
    /// `[x0, y0, z0, x1, y1, z1, colorHex]` per line segment.
    lines: Vec<[f64; 7]>,
    /// `[ax, ay, az, bx, by, bz, cx, cy, cz, colorHex]` per triangle.
    triangles: Vec<[f64; 10]>,
    /// The surface normal at each of a triangle's three corners,
    /// `[nax, nay, naz, nbx, …, ncz]`, one entry per entry of `triangles`.
    /// These are the kernel's own normals, not normals averaged from the
    /// mesh, so a curved face shades as the surface it approximates instead
    /// of as the facets it was cut into.
    normals: Vec<[f64; 9]>,
    /// Per triangle: the index in `faces` of the face it belongs to — so a
    /// viewer can highlight a face or a solid by name.
    triangle_faces: Vec<u32>,
    faces: Vec<FaceTag>,
    /// Per line: the index in `sketch_names` of the sketch it belongs to,
    /// or `-1` for an edge of the model.
    line_sketches: Vec<i32>,
    sketch_names: Vec<String>,
    /// Per line: the index in `edge_names` of the edge of the model it
    /// draws, or `-1` for a sketch's.
    line_edges: Vec<i32>,
    edge_names: Vec<String>,
}

fn f3(p: &Vector3<S>) -> [f64; 3] {
    [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()]
}

/// `view` of `part` as scene JSON, tagged with the names of what it shows.
fn scene_of(part: &Part<S>, view: &View) -> SceneJson {
    let mut scene = SceneJson {
        points: Vec::new(),
        point_names: Vec::new(),
        lines: Vec::new(),
        triangles: Vec::new(),
        normals: Vec::new(),
        triangle_faces: Vec::new(),
        faces: Vec::new(),
        line_sketches: Vec::new(),
        sketch_names: Vec::new(),
        line_edges: Vec::new(),
        edge_names: Vec::new(),
    };
    let hex = |c: Color10| c.to_hex() as f64;

    let name = |id: RefId| part.name_of(id).unwrap_or_default().to_string();
    for (&id, p) in &view.raster.vertices {
        let [x, y, z] = f3(p);
        scene.points.push([x, y, z, hex(Color10::DarkGray)]);
        scene.point_names.push(name(id.into()));
    }
    /// Whose a line is: a sketch's or an edge's, by index.
    enum Owner {
        Sketch(i32),
        Edge(i32),
    }
    let push_line = |scene: &mut SceneJson, a: [f64; 3], b: [f64; 3], color: f64, owner: &Owner| {
        scene
            .lines
            .push([a[0], a[1], a[2], b[0], b[1], b[2], color]);
        let (sketch, edge) = match owner {
            Owner::Sketch(i) => (*i, -1),
            Owner::Edge(i) => (-1, *i),
        };
        scene.line_sketches.push(sketch);
        scene.line_edges.push(edge);
    };
    for (&id, polyline) in &view.raster.edges {
        let owner = Owner::Edge(scene.edge_names.len() as i32);
        scene.edge_names.push(name(id.into()));
        for w in polyline.windows(2) {
            push_line(&mut scene, f3(&w[0]), f3(&w[1]), hex(Color10::Gray), &owner);
        }
    }

    let mut faces: Vec<_> = view.raster.faces.iter().collect();
    faces.sort_by_key(|(id, _)| id.0);
    for (&face, tris) in faces {
        let index = scene.faces.len() as u32;
        let solid = geop_cad_base::pick::solid_of_face(part.topology(), face);
        scene.faces.push(FaceTag {
            name: part.name_of(face).unwrap_or_default().to_string(),
            solid: solid.and_then(|s| part.name_of(s)).map(str::to_string),
        });
        for t in tris {
            let [a, b, c] = [f3(&t.a), f3(&t.b), f3(&t.c)];
            scene.triangles.push([
                a[0],
                a[1],
                a[2],
                b[0],
                b[1],
                b[2],
                c[0],
                c[1],
                c[2],
                hex(Color10::Blue),
            ]);
            let n = t.vertex_normals.unwrap_or([t.normal; 3]).map(|n| f3(&n));
            scene.normals.push([
                n[0][0], n[0][1], n[0][2], n[1][0], n[1][1], n[1][2], n[2][0], n[2][1], n[2][2],
            ]);
            scene.triangle_faces.push(index);
        }
    }

    for target in &view.sketches.0 {
        let owner = Owner::Sketch(scene.sketch_names.len() as i32);
        scene
            .sketch_names
            .push(part.name_of(target.sketch).unwrap_or_default().to_string());
        let frame = Frame::of(&target.plane);
        for (_, construction, polyline) in &target.curves {
            let color = if *construction {
                CONSTRUCTION_COLOR
            } else {
                SKETCH_COLOR
            } as f64;
            for w in polyline.windows(2) {
                push_line(
                    &mut scene,
                    frame.to_world(w[0]),
                    frame.to_world(w[1]),
                    color,
                    &owner,
                );
            }
        }
    }
    scene
}

/// Install a panic hook that forwards Rust panics to the JS console with a
/// proper stack trace, instead of an opaque "unreachable executed" trap.
/// Call once, right after the wasm module is instantiated.
#[wasm_bindgen]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

// ── operations and programs ──────────────────────────────────────────────────

/// Every operation and what it takes (JSON array of
/// `geop_ops_parts::OperationSchema`).
#[wasm_bindgen]
pub fn operation_schemas() -> Result<String, JsValue> {
    to_json(&PartOperation::schemas()).map_err(to_js_err)
}

/// The program being edited (JSON `Program`).
#[wasm_bindgen]
pub fn program() -> Result<String, JsValue> {
    PROGRAM.with(|p| p.borrow().to_json()).map_err(to_js_err)
}

fn update_program_inner(edit_json: &str) -> GeopResult<String> {
    let edit: ProgramEdit = from_json(edit_json, "program edit")?;
    let id = PROGRAM.with(|p| p.borrow_mut().update(edit))?;
    to_json(&id)
}

/// Apply an edit (JSON `ProgramEdit`) to the program — the only way it
/// changes. Returns the id of the step it inserted, changed or moved (JSON
/// string, or `null`). A rejected edit changes nothing.
#[wasm_bindgen]
pub fn update_program(edit_json: &str) -> Result<String, JsValue> {
    update_program_inner(edit_json).map_err(to_js_err)
}

/// The example programs (JSON array of `{name, program}`), to start from.
#[wasm_bindgen]
pub fn example_programs() -> Result<String, JsValue> {
    #[derive(Serialize)]
    struct Example {
        name: &'static str,
        program: Program,
    }
    let all: Vec<Example> = examples::all()
        .into_iter()
        .map(|(name, program)| Example { name, program })
        .collect();
    to_json(&all).map_err(to_js_err)
}

/// A sketch of the part, for choosing it (and a line of it) as an argument.
#[derive(Serialize)]
struct SketchJson {
    name: String,
    /// Every line: `{id, construction}`.
    lines: Vec<LineJson>,
    frame: Frame,
}

/// A datum of the part, for drawing and picking it.
#[derive(Serialize)]
struct DatumJson {
    name: String,
    kind: DatumKind,
    frame: Frame,
}

#[derive(Serialize)]
struct LineJson {
    id: u64,
    construction: bool,
}

#[derive(Serialize)]
struct RunJson {
    /// One per step that ran; the last may be the failure that stopped it.
    results: Vec<StepResult>,
    scene: SceneJson,
    /// The part's solids, oldest first.
    solids: Vec<String>,
    sketches: Vec<SketchJson>,
    /// The part's datums, oldest first.
    datums: Vec<DatumJson>,
    /// Every handle of every step that ran — the editor picks which to
    /// offer.
    handles: Vec<StepHandle>,
}

fn run_json(runner: &ProgramRunner<S>, view: &View) -> GeopResult<String> {
    let (part, results) = (runner.part(), runner.results());
    let mut solids: Vec<SolidId> = part.topology().solids.keys().copied().collect();
    solids.sort_by_key(|s| s.0);
    let sketches = part
        .sketches()
        .map(|(id, placed)| SketchJson {
            name: part.name_of(id).unwrap_or_default().to_string(),
            lines: placed
                .sketch
                .curves
                .iter()
                .filter(|(_, c)| matches!(c.kind, CurveKind::Line { .. }))
                .map(|(&id, c)| LineJson {
                    id: id.0,
                    construction: c.construction,
                })
                .collect(),
            frame: Frame::of(&placed.plane),
        })
        .collect();
    to_json(&RunJson {
        results: results.to_vec(),
        scene: scene_of(part, view),
        solids: solids
            .into_iter()
            .filter_map(|s| part.name_of(s).map(str::to_string))
            .collect(),
        sketches,
        datums: part
            .datums()
            .map(|(id, datum)| DatumJson {
                name: part.name_of(id).unwrap_or_default().to_string(),
                kind: datum.kind,
                frame: Frame::of(&datum.frame),
            })
            .collect(),
        handles: runner.handles()?,
    })
}

fn run_program_inner(stop_json: &str) -> GeopResult<String> {
    let stop: Option<usize> = from_json(stop_json, "stop")?;
    PROGRAM.with(|program| {
        COMMITTED.with(|runner| {
            let mut runner = runner.borrow_mut();
            runner.run(&program.borrow(), stop);
            let view = View::of(runner.part())?;
            let json = run_json(&runner, &view);
            VIEW.with(|v| *v.borrow_mut() = Some(view));
            json
        })
    })
}

/// Build the program, stopping after `stop` steps (JSON number, or `null`
/// for all of them), and return `{results, scene, solids, sketches}`.
/// Rebuilds only from the first step that changed since the last run.
#[wasm_bindgen]
pub fn run_program(stop_json: &str) -> Result<String, JsValue> {
    run_program_inner(stop_json).map_err(to_js_err)
}

fn preview_program_inner(edit_json: &str, stop_json: &str) -> GeopResult<String> {
    let edit: ProgramEdit = from_json(edit_json, "program edit")?;
    let stop: Option<usize> = from_json(stop_json, "stop")?;
    let mut program = PROGRAM.with(|p| p.borrow().clone());
    program.update(edit)?;
    PREVIEW.with(|runner| {
        let mut runner = runner.borrow_mut();
        runner.run(&program, stop);
        run_json(&runner, &View::of(runner.part())?)
    })
}

/// Like [`run_program`], for the program with `edit` applied — without
/// applying it: what the edit would build, to preview while it is being
/// made.
#[wasm_bindgen]
pub fn preview_program(edit_json: &str, stop_json: &str) -> Result<String, JsValue> {
    preview_program_inner(edit_json, stop_json).map_err(to_js_err)
}

/// A sketch plane in plain `f64`, for display: sketch `(x, y)` lies at
/// `origin + x u + y v`.
#[derive(Serialize)]
struct Frame {
    origin: [f64; 3],
    u: [f64; 3],
    v: [f64; 3],
    normal: [f64; 3],
}

impl Frame {
    fn of(cs: &CoordinateSystem<S>) -> Self {
        let f = |p: &Vector3<S>| [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()];
        Frame {
            origin: f(cs.origin()),
            u: f(cs.u()),
            v: f(cs.v()),
            normal: f(cs.w()),
        }
    }

    fn to_world(&self, p: [f64; 2]) -> [f64; 3] {
        [0, 1, 2].map(|k| self.origin[k] + p[0] * self.u[k] + p[1] * self.v[k])
    }
}

// ── picking ──────────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct PickHitJson {
    kind: &'static str,
    /// The name of what was hit.
    name: String,
    point: [f64; 3],
    t: f64,
    /// For a face or solid hit, the name of the solid it belongs to.
    solid: Option<String>,
}

fn parse_pick_filter(filter: &str) -> GeopResult<PickFilter> {
    match filter {
        "vertex" => Ok(PickFilter::Vertex),
        "edge" => Ok(PickFilter::Edge),
        "face" => Ok(PickFilter::Face),
        "solid" => Ok(PickFilter::Solid),
        "any" => Ok(PickFilter::Any),
        other => Err(GeopError::new(format!(
            "pick_ray: unknown filter {other:?}, expected one of vertex/edge/face/solid/any/sketch"
        ))),
    }
}

fn pick_ray_inner(ray: Ray<S>, filter: &str, tolerance: f64) -> GeopResult<String> {
    // Nothing has been built yet, so there is nothing to hit.
    let hit = COMMITTED.with(|runner| {
        VIEW.with(|view| -> GeopResult<Option<PickHitJson>> {
            let (runner, view) = (runner.borrow(), view.borrow());
            let Some(view) = view.as_ref() else {
                return Ok(None);
            };
            let part = runner.part();
            if filter == "sketch" {
                return Ok(pick_sketch(&view.sketches, ray, tolerance).and_then(|h| {
                    Some(PickHitJson {
                        kind: "sketch",
                        name: part.name_of(h.sketch)?.to_string(),
                        point: f3(&h.point),
                        t: h.t.to_f64(),
                        solid: None,
                    })
                }));
            }
            let model = part.topology();
            let Some(h) = pick_model(
                model,
                &view.raster,
                ray,
                parse_pick_filter(filter)?,
                tolerance,
            )?
            else {
                return Ok(None);
            };
            let (kind, id, solid) = match h.kind {
                PickKind::Vertex => ("vertex", RefId::Vertex(VertexId(h.id)), None),
                PickKind::Edge => ("edge", RefId::Edge(EdgeId(h.id)), None),
                PickKind::Face => (
                    "face",
                    RefId::Face(FaceId(h.id)),
                    geop_cad_base::pick::solid_of_face(model, FaceId(h.id)),
                ),
                PickKind::Solid => ("solid", RefId::Solid(SolidId(h.id)), Some(SolidId(h.id))),
            };
            let name_of = |id: RefId| part.name_of(id).map(str::to_string);
            Ok(name_of(id).map(|name| PickHitJson {
                kind,
                name,
                point: f3(&h.point),
                t: h.t.to_f64(),
                solid: solid.and_then(|s| name_of(s.into())),
            }))
        })
    })?;
    to_json(&hit)
}

/// Cast a ray (`origin` = `(ox, oy, oz)`, `dir` = `(dx, dy, dz)`, not
/// required to be unit length) against the part of the most recent
/// [`run_program`], as its scene draws it — cheap enough to call on every
/// pointer move, to show what a click would pick — and return the nearest entity matching `filter`
/// (`"vertex" | "edge" | "face" | "solid" | "sketch"`, or `"any"` for the
/// smallest visible vertex, edge or face) within `tolerance`
/// (world-space distance; only meaningful for vertices, edges and sketch
/// curves — a sketch is also hit anywhere inside its closed regions) as
/// JSON `{kind, name, point, t, solid}` — `null` if nothing was hit.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn pick_ray(
    ox: f64,
    oy: f64,
    oz: f64,
    dx: f64,
    dy: f64,
    dz: f64,
    filter: &str,
    tolerance: f64,
) -> Result<String, JsValue> {
    let v3 = |x: f64, y: f64, z: f64| Vector3::from_array([x, y, z].map(S::from_f64));
    let ray = Ray {
        origin: v3(ox, oy, oz),
        dir: v3(dx, dy, dz),
    };
    pick_ray_inner(ray, filter, tolerance).map_err(to_js_err)
}

// ── sketching ────────────────────────────────────────────────────────────────

fn sketch_plane_inner(plane_json: &str) -> GeopResult<String> {
    let plane: EntityRef = from_json(plane_json, "sketch plane")?;
    let cs = COMMITTED.with(|runner| resolve_plane(runner.borrow().part(), &plane))?;
    to_json(&Frame::of(&cs))
}

/// The frame of a sketch plane (JSON `EntityRef`: `{"type": "Plane",
/// "normal": "Z"}`, `{"type": "Face", "name": "<name>"}` or `{"type":
/// "Datum", "name": "<name>"}`) in the part of the most recent
/// [`run_program`] — the part a sketch step inserted there sees.
#[wasm_bindgen]
pub fn sketch_plane(plane_json: &str) -> Result<String, JsValue> {
    sketch_plane_inner(plane_json).map_err(to_js_err)
}

fn inspect_selection_inner(selection_json: &str) -> GeopResult<String> {
    let selection: Vec<EntityRef> = from_json(selection_json, "selection")?;
    let fit = COMMITTED.with(|runner| inspect_selection(runner.borrow().part(), &selection));
    to_json(&fit)
}

/// What each entity of a selection (JSON array of `EntityRef`) can be used
/// as in the part of the most recent [`run_program`], and which datum
/// constructions fit it: JSON `{roles: [[role, ...], ...], fits: [method,
/// ...]}` (see `geop_ops_parts::operation::SelectionFit`).
#[wasm_bindgen]
pub fn inspect_selection_fit(selection_json: &str) -> Result<String, JsValue> {
    inspect_selection_inner(selection_json).map_err(to_js_err)
}

#[derive(Serialize)]
struct SolveJson {
    sketch: Sketch,
    report: SolveReport,
    /// Every closed region as `[outer, ...holes]`, each a polyline in sketch
    /// coordinates — for shading the profile while sketching.
    regions: Vec<Vec<Vec<[f64; 2]>>>,
    /// Why the curves do not form regions, if they do not.
    regions_error: Option<String>,
}

fn solve_sketch_inner(sketch_json: &str, drags_json: &str) -> GeopResult<String> {
    let mut sketch: Sketch = from_json(sketch_json, "sketch")?;
    let drags: Vec<(u64, f64, f64)> = from_json(drags_json, "drags")?;
    let drags: Vec<(PointId, [f64; 2])> = drags
        .into_iter()
        .map(|(p, x, y)| (PointId(p), [x, y]))
        .collect();
    let report = sketch.solve_with_drag(&drags)?;
    let positions = sketch.positions();
    let (regions, regions_error) = match sketch.regions() {
        Ok(regions) => (
            regions
                .iter()
                .map(|r| {
                    std::iter::once(&r.outer)
                        .chain(&r.holes)
                        .map(|l| l.polyline(&sketch, &positions))
                        .collect()
                })
                .collect(),
            None,
        ),
        Err(e) => (Vec::new(), Some(e.to_string())),
    };
    to_json(&SolveJson {
        sketch,
        report,
        regions,
        regions_error,
    })
}

/// Solve a sketch (JSON `Sketch`), optionally dragging points towards
/// targets (`drags_json`: `[[pointId, x, y], ...]`), and return the solved
/// sketch with its `SolveReport` and region outlines.
#[wasm_bindgen]
pub fn solve_sketch(sketch_json: &str, drags_json: &str) -> Result<String, JsValue> {
    solve_sketch_inner(sketch_json, drags_json).map_err(to_js_err)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn load(program: &Program) {
        let edit = ProgramEdit::Replace {
            program: program.clone(),
        };
        update_program_inner(&serde_json::to_string(&edit).unwrap()).unwrap();
    }

    /// The schemas the editor builds its forms from: every operation, with
    /// its arguments' kinds.
    #[test]
    fn schemas_list_every_operation() {
        let schemas: serde_json::Value =
            serde_json::from_str(&to_json(&PartOperation::schemas()).unwrap()).unwrap();
        let kinds: Vec<&str> = schemas
            .as_array()
            .unwrap()
            .iter()
            .map(|s| s["kind"].as_str().unwrap())
            .collect();
        assert!(kinds.contains(&"extrude"), "{kinds:?}");
        assert_eq!(schemas[1]["args"][1]["kind"]["type"], "number");
    }

    /// The editor's loop: edits as JSON, a run of the program, a pick by
    /// name, a stop part way.
    #[test]
    fn edits_runs_and_picks_by_name() {
        load(&examples::box_with_drill_hole());
        let inserted = update_program_inner(
            r#"{"edit": "insert", "index": 4, "operation": "extrude",
                "args": {"sketch": "outline", "distance": -0.5}}"#,
        )
        .unwrap();
        assert_eq!(inserted, r#""extrude1""#);

        let run: serde_json::Value =
            serde_json::from_str(&run_program_inner("null").unwrap()).unwrap();
        let results = run["results"].as_array().unwrap();
        assert_eq!(results.len(), 5);
        assert!(results.iter().all(|r| r["error"].is_null()), "{results:?}");
        assert_eq!(
            run["solids"],
            serde_json::json!(["extrude(hole)", "extrude(extrude1)"])
        );
        assert!(!run["scene"]["triangles"].as_array().unwrap().is_empty());
        assert_eq!(run["sketches"][0]["name"], "outline");
        // Every step's handles come along, feature and sketch alike.
        let handles = run["handles"].as_array().unwrap();
        assert!(
            handles
                .iter()
                .any(|h| h["step"] == "box" && h["arg"] == serde_json::json!(["distance"]))
        );
        assert!(handles.iter().any(|h| h["group"] == "sketch"));
        // Every triangle and line says what it belongs to, by name.
        let scene = &run["scene"];
        assert_eq!(
            scene["triangle_faces"].as_array().unwrap().len(),
            scene["triangles"].as_array().unwrap().len()
        );
        assert_eq!(
            scene["line_sketches"].as_array().unwrap().len(),
            scene["lines"].as_array().unwrap().len()
        );
        let top = scene["faces"]
            .as_array()
            .unwrap()
            .iter()
            .find(|f| f["name"] == "extrude(box,end)")
            .expect("the box's top is drawn");
        assert_eq!(top["solid"], "extrude(hole)");
        assert!(
            scene["sketch_names"]
                .as_array()
                .unwrap()
                .contains(&"outline".into())
        );
        assert_eq!(run["sketches"][0]["lines"].as_array().unwrap().len(), 4);

        // Straight down onto the top of the box, away from the hole.
        let ray = Ray {
            origin: Vector3::from_array([0.3, 0.3, 5.0].map(S::from_f64)),
            dir: Vector3::from_array([0.0, 0.0, -1.0].map(S::from_f64)),
        };
        let hit: serde_json::Value =
            serde_json::from_str(&pick_ray_inner(ray, "face", 0.0).unwrap()).unwrap();
        assert_eq!(hit["name"], "extrude(box,end)");
        assert_eq!(hit["solid"], "extrude(hole)");

        // The outline sketch lies under the box: seen from below, it is
        // hit anywhere inside the square it bounds.
        let up = Ray {
            origin: Vector3::from_array([1.0, 1.0, -5.0].map(S::from_f64)),
            dir: Vector3::from_array([0.0, 0.0, 1.0].map(S::from_f64)),
        };
        let hit: serde_json::Value =
            serde_json::from_str(&pick_ray_inner(up, "sketch", 0.01).unwrap()).unwrap();
        assert_eq!(hit["name"], "outline");
        assert_eq!(hit["kind"], "sketch");

        // Back in time: only the box's two steps.
        let run: serde_json::Value =
            serde_json::from_str(&run_program_inner("2").unwrap()).unwrap();
        assert_eq!(run["results"].as_array().unwrap().len(), 2);
        assert_eq!(run["solids"], serde_json::json!(["extrude(box)"]));
    }

    /// A preview builds the program with an edit, but leaves the program as
    /// it was; a rejected edit reports why.
    #[test]
    fn previews_do_not_edit_the_program() {
        load(&examples::box_with_drill_hole());
        let before = PROGRAM.with(|p| p.borrow().clone());
        let edit = r#"{"edit": "update", "id": "hole", "operation": "extrude",
                       "args": {"sketch": "hole_sketch", "distance": -0.25}}"#;
        let run: serde_json::Value =
            serde_json::from_str(&preview_program_inner(edit, "null").unwrap()).unwrap();
        assert!(
            run["results"]
                .as_array()
                .unwrap()
                .iter()
                .all(|r| r["error"].is_null())
        );
        assert_eq!(PROGRAM.with(|p| p.borrow().clone()), before);

        let err = update_program_inner(r#"{"edit": "remove", "id": "nope"}"#).unwrap_err();
        assert!(format!("{err:?}").contains("nope"), "{err:?}");
    }

    /// A sketch plane on a base plane, without any part: what a new sketch
    /// on an empty program asks for.
    #[test]
    fn sketch_plane_json_reports_a_frame() {
        let out = sketch_plane_inner(r#"{"type": "Plane", "normal": "X"}"#).unwrap();
        let frame: serde_json::Value = serde_json::from_str(&out).unwrap();
        assert_eq!(frame["normal"], serde_json::json!([1.0, 0.0, 0.0]));
        assert_eq!(frame["u"], serde_json::json!([0.0, 1.0, 0.0]));
    }

    /// Datums come with every run, and a selection is told what it can be
    /// used as and what can be built from it; any entity can be picked for
    /// one, by name.
    #[test]
    fn datums_selections_and_picking_anything() {
        load(&examples::boss_on_reference_plane());
        let run: serde_json::Value =
            serde_json::from_str(&run_program_inner("null").unwrap()).unwrap();
        let lifted = &run["datums"][0];
        assert_eq!(lifted["name"], "lifted");
        assert_eq!(lifted["kind"], "plane");
        assert_eq!(
            lifted["frame"]["normal"],
            serde_json::json!([0.0, 0.0, 1.0])
        );
        let scene = &run["scene"];
        assert_eq!(
            scene["line_edges"].as_array().unwrap().len(),
            scene["lines"].as_array().unwrap().len()
        );
        assert_eq!(
            scene["point_names"].as_array().unwrap().len(),
            scene["points"].as_array().unwrap().len()
        );

        let fit: serde_json::Value = serde_json::from_str(
            &inspect_selection_inner(
                r#"[{"type": "Datum", "name": "lifted"}, {"type": "Axis", "axis": "Z"}]"#,
            )
            .unwrap(),
        )
        .unwrap();
        assert_eq!(fit["roles"], serde_json::json!([["plane"], ["line"]]));
        let fits = fit["fits"].as_array().unwrap();
        assert!(fits.contains(&"line_plane".into()), "{fits:?}");
        assert!(!fits.contains(&"offset".into()), "{fits:?}");

        // Right next to the box's corner, from above: the corner.
        let ray = Ray {
            origin: Vector3::from_array([2.005, 0.005, 5.0].map(S::from_f64)),
            dir: Vector3::from_array([0.0, 0.0, -1.0].map(S::from_f64)),
        };
        let hit: serde_json::Value =
            serde_json::from_str(&pick_ray_inner(ray, "any", 0.02).unwrap()).unwrap();
        assert_eq!(hit["kind"], "vertex");
        assert_eq!(hit["name"], "extrude(box,outline,p1,end)");
    }

    /// Solving with a drag: the same call the sketch editor makes on every
    /// pointer move, including the region outlines it shades.
    #[test]
    fn solve_sketch_json_drags_and_reports_regions() {
        let sketch = r#"{
            "points": {
                "0": {"x": 0.0, "y": 0.0}, "1": {"x": 1.0, "y": 0.0},
                "2": {"x": 1.0, "y": 1.0}, "3": {"x": 0.0, "y": 1.0}
            },
            "curves": {
                "4": {"type": "Line", "start": 0, "end": 1, "construction": false},
                "5": {"type": "Line", "start": 1, "end": 2, "construction": false},
                "6": {"type": "Line", "start": 2, "end": 3, "construction": false},
                "7": {"type": "Line", "start": 3, "end": 0, "construction": false}
            },
            "constraints": {
                "8": {"type": "Fix", "point": 0, "x": 0.0, "y": 0.0},
                "9": {"type": "Horizontal", "line": 4},
                "10": {"type": "Vertical", "line": 7}
            },
            "next_id": 11
        }"#;
        let out = solve_sketch_inner(sketch, "[[2, 3.0, 2.0]]").unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&out).unwrap();
        assert!(parsed["report"]["converged"].as_bool().unwrap(), "{out}");
        // The dragged corner is free, so it lands on the cursor.
        let p2 = &parsed["sketch"]["points"]["2"];
        assert!((p2["x"].as_f64().unwrap() - 3.0).abs() < 1e-6, "{p2}");
        assert!((p2["y"].as_f64().unwrap() - 2.0).abs() < 1e-6, "{p2}");
        // One region, one (outer) loop, no holes.
        assert_eq!(parsed["regions"].as_array().unwrap().len(), 1);
        assert_eq!(parsed["regions"][0].as_array().unwrap().len(), 1);
        assert!(parsed["regions_error"].is_null());
    }
}
