//! WebAssembly bindings for the browser editor.
//!
//! The editor edits a `geop_cad_base::Program`, and only through this
//! module: [`update_program`] applies a [`ProgramEdit`] with
//! [`Program::update`] — the one way any editor changes a program — and
//! [`run_program`] builds it with a [`ProgramRunner`], as far as the editor
//! asks (while a step in the middle is being edited, the steps after it
//! need not run).
//!
//! A step is edited through [`edit_step`]: the editor sends what the user
//! did, and gets back the step's new arguments, the session to send with
//! the next event, and what to show — a dialog and visuals (see
//! `geop_ops::ui`). Every decision, from what a click picks to what a sketch
//! snaps to, is made there; the editor renders and forwards input.
//!
//! Everything crosses the boundary as JSON: programs, edits and events in
//! their own serde format, scenes as flat number arrays a `three.js` viewer
//! consumes directly. Entities are named, never numbered.

use std::cell::RefCell;

use geop_cad_base::{PartOperation, Program, ProgramEdit, ProgramRunner, examples};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Color10,
    scalars::{Scalar, scal_in_f64::ScalInF64},
    vector::Vector3,
};
use geop_ops::{
    EditContext, EntityRef, Operations, StepResult,
    ui::{Event, Extent, PartView, Presentation},
};
use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

/// The scalar type used for every wasm-exposed part.
type S = ScalInF64;

thread_local! {
    /// The program being edited. Changed only by [`update_program`].
    static PROGRAM: RefCell<Program> = RefCell::new(Program::new());
    /// Builds [`PROGRAM`]. A step is edited against the part of its most
    /// recent run — never a preview's, so nothing can refer to geometry
    /// that only a not-yet-made edit would create.
    static COMMITTED: RefCell<ProgramRunner<S>> = RefCell::new(ProgramRunner::new());
    /// How the part of `COMMITTED`'s most recent run is drawn: made once per
    /// run, and what every pick tests against — so a pick is cheap enough
    /// for hovering, and hits exactly what is on screen.
    static VIEW: RefCell<Option<PartView<S>>> = const { RefCell::new(None) };
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

/// Sketch curves in the scene: profile geometry and construction geometry.
const SKETCH_COLOR: u32 = 0xffa040;
const CONSTRUCTION_COLOR: u32 = 0x808080;

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

/// `view` as scene JSON, tagged with the names of what it shows.
fn scene_of(view: &PartView<S>) -> SceneJson {
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
    let xyz = |p: &Vector3<S>| [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()];
    for v in &view.vertices {
        let [x, y, z] = xyz(&v.at);
        scene.points.push([x, y, z, hex(Color10::DarkGray)]);
        scene.point_names.push(v.name.clone());
    }
    let push_line = |scene: &mut SceneJson,
                     a: Vector3<S>,
                     b: Vector3<S>,
                     color: f64,
                     sketch: i32,
                     edge: i32| {
        let (a, b) = (xyz(&a), xyz(&b));
        scene
            .lines
            .push([a[0], a[1], a[2], b[0], b[1], b[2], color]);
        scene.line_sketches.push(sketch);
        scene.line_edges.push(edge);
    };
    for e in &view.edges {
        let index = scene.edge_names.len() as i32;
        scene.edge_names.push(e.name.clone());
        for w in e.polyline.windows(2) {
            push_line(&mut scene, w[0], w[1], hex(Color10::Gray), -1, index);
        }
    }
    for f in &view.faces {
        let index = scene.faces.len() as u32;
        scene.faces.push(FaceTag {
            name: f.name.clone(),
            solid: f.solid.clone(),
        });
        for (triangle, normals) in f.triangles.iter().zip(&f.normals) {
            let [a, b, c] = triangle.map(|p| xyz(&p));
            let n = normals.map(|p| xyz(&p));
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
            scene.normals.push([
                n[0][0], n[0][1], n[0][2], n[1][0], n[1][1], n[1][2], n[2][0], n[2][1], n[2][2],
            ]);
            scene.triangle_faces.push(index);
        }
    }
    for sketch in &view.sketches {
        let index = scene.sketch_names.len() as i32;
        scene.sketch_names.push(sketch.name.clone());
        for curve in &sketch.curves {
            let color = if curve.construction {
                CONSTRUCTION_COLOR
            } else {
                SKETCH_COLOR
            } as f64;
            for w in curve.polyline.windows(2) {
                let (a, b) = (sketch.plane.uv_to_xyz(&w[0]), sketch.plane.uv_to_xyz(&w[1]));
                push_line(&mut scene, a, b, color, index, -1);
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

/// Every operation the editor offers (JSON array of
/// `geop_ops::OperationInfo`: `{kind, label, doc}`).
#[wasm_bindgen]
pub fn operation_infos() -> Result<String, JsValue> {
    to_json(&PartOperation::infos()).map_err(to_js_err)
}

/// The program being edited (JSON `Program`).
#[wasm_bindgen]
pub fn program() -> Result<String, JsValue> {
    PROGRAM.with(|p| p.borrow().to_json()).map_err(to_js_err)
}

/// A step of the program, as a list of steps shows it.
#[derive(Serialize)]
struct StepInfo {
    id: String,
    kind: &'static str,
    label: &'static str,
    /// Its arguments in one line.
    summary: String,
}

/// Every step of the program, as a list of steps shows it (JSON array of
/// `{id, kind, label, summary}`).
#[wasm_bindgen]
pub fn describe_program() -> Result<String, JsValue> {
    let steps: Vec<StepInfo> = PROGRAM.with(|p| {
        p.borrow()
            .steps
            .iter()
            .map(|s| StepInfo {
                id: s.id.clone(),
                kind: s.operation.kind(),
                label: s.operation.label(),
                summary: s.operation.summary(),
            })
            .collect()
    });
    to_json(&steps).map_err(to_js_err)
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

#[derive(Serialize)]
struct RunJson<'a> {
    /// One per step that ran; the last may be the failure that stopped it.
    results: &'a [StepResult],
    scene: SceneJson,
    /// The part's datums, oldest first.
    datums: &'a [geop_ops::ui::view::ViewDatum<S>],
    /// Where the drawing is and how big: datum planes and axes are drawn
    /// this big around the point of them nearest its center.
    extent: Extent<S>,
    /// The sketches and datums the steps that ran build on: an editor hides
    /// them, since what was made from them shows them now.
    references: Vec<EntityRef>,
}

fn run_json(runner: &ProgramRunner<S>, view: &PartView<S>) -> GeopResult<String> {
    to_json(&RunJson {
        results: runner.results(),
        scene: scene_of(view),
        datums: &view.datums,
        extent: view.extent,
        references: runner.references(),
    })
}

fn run_program_inner(stop_json: &str) -> GeopResult<String> {
    let stop: Option<usize> = from_json(stop_json, "stop")?;
    PROGRAM.with(|program| {
        COMMITTED.with(|runner| {
            let mut runner = runner.borrow_mut();
            runner.run(&program.borrow(), stop);
            let view = PartView::of(runner.part())?;
            let json = run_json(&runner, &view);
            VIEW.with(|v| *v.borrow_mut() = Some(view));
            json
        })
    })
}

/// Build the program, stopping after `stop` steps (JSON number, or `null`
/// for all of them), and return `{results, scene, datums, extent,
/// references}`. Rebuilds only from the first step that changed since the
/// last run. The part it builds is what [`edit_step`] edits against.
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
        run_json(&runner, &PartView::of(runner.part())?)
    })
}

/// Like [`run_program`], for the program with `edit` applied — without
/// applying it: what the edit would build, to preview while it is being
/// made.
#[wasm_bindgen]
pub fn preview_program(edit_json: &str, stop_json: &str) -> Result<String, JsValue> {
    preview_program_inner(edit_json, stop_json).map_err(to_js_err)
}

// ── editing a step ───────────────────────────────────────────────────────────

/// What [`edit_step`] is asked.
#[derive(Deserialize)]
struct EditRequest {
    /// The step as it is — or, for a new step, only `kind`.
    #[serde(default)]
    operation: Option<PartOperation>,
    #[serde(default)]
    kind: Option<String>,
    /// The session the last edit returned; `null` to start afresh.
    #[serde(default)]
    session: serde_json::Value,
    /// What the user did; `null` for only what to show.
    #[serde(default)]
    event: Option<Event<S>>,
}

/// What [`edit_step`] answers.
#[derive(Serialize)]
struct EditResponse {
    /// The step's operation with its new arguments: `{operation, args}`.
    operation: PartOperation,
    session: serde_json::Value,
    presentation: Presentation<S>,
}

fn edit_step_inner(request_json: &str) -> GeopResult<String> {
    let request: EditRequest = from_json(request_json, "edit request")?;
    COMMITTED.with(|runner| {
        VIEW.with(|view| {
            let runner = runner.borrow();
            let part = runner.part();
            let mut view = view.borrow_mut();
            if view.is_none() {
                *view = Some(PartView::of(part)?);
            }
            let view = view.as_ref().expect("made above");
            let operation = match (request.operation, request.kind) {
                (Some(operation), _) => operation,
                (None, Some(kind)) => PartOperation::new_step(&kind, part)?,
                (None, None) => {
                    return Err(GeopError::new("edit request needs an operation or a kind"));
                }
            };
            let ctx = EditContext { part, view };
            let edited = operation.edit(&ctx, request.session, request.event.as_ref());
            to_json(&EditResponse {
                operation: edited.args,
                session: edited.session,
                presentation: edited.presentation,
            })
        })
    })
}

/// Edit a step against the part of the most recent [`run_program`] — the
/// part the step is applied to. The request is JSON `{operation, session,
/// event}` — `{kind}` in place of `operation` starts a new step of that
/// kind — and the answer JSON `{operation, session, presentation}`: the
/// step with its new arguments, the session to send with the next event,
/// and what to show (see `geop_ops::ui`).
#[wasm_bindgen]
pub fn edit_step(request_json: &str) -> Result<String, JsValue> {
    edit_step_inner(request_json).map_err(to_js_err)
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

    fn json(s: &str) -> serde_json::Value {
        serde_json::from_str(s).unwrap()
    }

    /// The editor's loop: edits as JSON, a run of the program, a stop part
    /// way.
    #[test]
    fn edits_and_runs() {
        load(&examples::box_with_drill_hole());
        let inserted = update_program_inner(
            r#"{"edit": "insert", "index": 4, "operation": "extrude",
                "args": {"sketch": "outline", "distance": -0.5}}"#,
        )
        .unwrap();
        assert_eq!(inserted, r#""extrude1""#);

        let run = json(&run_program_inner("null").unwrap());
        let results = run["results"].as_array().unwrap();
        assert_eq!(results.len(), 5);
        assert!(results.iter().all(|r| r["error"].is_null()), "{results:?}");
        assert!(!run["scene"]["triangles"].as_array().unwrap().is_empty());
        assert!(
            run["references"]
                .as_array()
                .unwrap()
                .contains(&serde_json::json!({"type": "Sketch", "name": "outline"}))
        );
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

        let steps = json(&describe_program().unwrap());
        assert_eq!(steps[4]["label"], "Extrude");
        assert!(
            steps[4]["summary"]
                .as_str()
                .unwrap()
                .contains("distance=-0.50")
        );

        // Back in time: only the box's two steps.
        let run = json(&run_program_inner("2").unwrap());
        assert_eq!(run["results"].as_array().unwrap().len(), 2);
    }

    /// A preview builds the program with an edit, but leaves the program as
    /// it was; a rejected edit reports why.
    #[test]
    fn previews_do_not_edit_the_program() {
        load(&examples::box_with_drill_hole());
        let before = PROGRAM.with(|p| p.borrow().clone());
        let edit = r#"{"edit": "update", "id": "hole", "operation": "extrude",
                       "args": {"sketch": "hole_sketch", "distance": -0.25}}"#;
        let run = json(&preview_program_inner(edit, "null").unwrap());
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

    /// Datums come with every run, as the viewer draws them.
    #[test]
    fn datums_come_with_runs() {
        load(&examples::boss_on_reference_plane());
        let run = json(&run_program_inner("null").unwrap());
        assert_eq!(run["datums"][0]["name"], "origin");
        assert_eq!(run["datums"][0]["kind"], "frame");
        let lifted = &run["datums"][1];
        assert_eq!(lifted["name"], "lifted");
        assert_eq!(lifted["kind"], "plane");
        assert_eq!(lifted["frame"]["w"], serde_json::json!([0.0, 0.0, 1.0]));
        assert!(run["extent"]["size"].as_f64().unwrap() >= 1.0);
    }

    /// A step is started, shown and edited through JSON, its session
    /// carried from one call to the next: a new sketch picks its plane,
    /// then draws a line, clicked as rays.
    #[test]
    fn steps_are_edited_as_json() {
        load(&Program::new());
        run_program_inner("null").unwrap();
        let start = json(&edit_step_inner(r#"{"kind": "add_sketch"}"#).unwrap());
        assert_eq!(start["operation"]["operation"], "add_sketch");
        assert_eq!(start["presentation"]["dialog"][0]["key"], "plane");
        assert!(start["presentation"]["focus"].is_null());

        let mut step = start;
        let mut send = |event: serde_json::Value| {
            let request = serde_json::json!({
                "operation": step["operation"],
                "session": step["session"],
                "event": event,
            });
            step = json(&edit_step_inner(&request.to_string()).unwrap());
            step.clone()
        };
        let drawing =
            send(serde_json::json!({"type": "dialog", "key": "draw", "value": {"type": "press"}}));
        assert_eq!(
            drawing["presentation"]["focus"]["w"],
            serde_json::json!([0.0, 0.0, 1.0])
        );
        send(serde_json::json!({"type": "key", "key": "l"}));
        let click = |x: f64, y: f64| {
            serde_json::json!({
                "type": "click",
                "pointer": {
                    "ray": {"origin": [x, y, 10.0], "dir": [0.0, 0.0, -1.0]},
                    "reach": {"type": "tube", "radius": 0.009},
                },
            })
        };
        send(click(0.5, 0.5));
        let drawn = send(click(1.5, 0.5));
        let curves = drawn["operation"]["args"]["sketch"]["curves"]
            .as_object()
            .unwrap();
        assert_eq!(curves.len(), 1);
        let visuals = drawn["presentation"]["visuals"].as_array().unwrap();
        assert!(visuals.iter().any(|v| v["shape"] == "polyline"));
    }
}
