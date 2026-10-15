//! The inspect tools on whole programs: the mass properties of assemblies,
//! each placed part of its own material where it is placed, interference
//! between placed parts — and, ignored for their time, every example's
//! mass properties checked against themselves.

use geop_core_editor::output::Bounded;
use geop_core_math::{
    scalars::{Ring, ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops_assembly::solids::placed_solids;
use geop_ops_booleans::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};
use geop_ops_extrude_revolve::shapes::cube::cube_solid;
use geop_ops_inspect::{
    interference::bounding_box,
    mass::{MassSummary, mass_report},
};
use geop_ops_model::Namer;

use crate::registry::testing::Built;
use crate::{Command, Editor, examples};

/// The editor with the workspace example `name` loaded, as the browser
/// loads one.
fn workspace(name: &str) -> Editor {
    let mut editor = crate::editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: name.into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor
}

/// The answer of the inspector `kind`, asked of the part as drawn the way
/// the inspect panel asks: as the front end reads it.
fn ask(editor: &mut Editor, kind: &str) -> serde_json::Value {
    let update = editor.handle(Command::Inspect {
        inspector: kind.into(),
        options: serde_json::Value::Null,
    });
    assert!(update.error.is_none(), "{kind}: {:?}", update.error);
    let inspection = update.inspection.expect("an answer");
    assert_eq!(inspection.kind, kind);
    inspection.report
}

/// `json` as a number shown with its bounds.
fn bounded(json: &serde_json::Value) -> Bounded {
    Bounded {
        value: json["value"].as_f64().expect("a value"),
        error: json["error"].as_f64().expect("an error"),
    }
}

/// Two plates placed in an assembly, both aluminium as their file says:
/// each weighed where it is placed, the total their sum, its centre
/// between theirs. Asked through the editor, as the inspect panel asks.
#[test]
fn an_assembly_weighs_its_placed_parts() {
    let mut editor = workspace("parametric_plates");
    let report = ask(&mut editor, "mass_properties");
    let bodies = report["bodies"].as_array().expect("bodies");
    assert_eq!(bodies.len(), 2, "{report}");
    let mut sum = 0.0;
    let mut moment = 0.0;
    for body in bodies {
        let name = body["name"].as_str().unwrap();
        assert!(name.contains('/'), "{name} is of a placed part");
        assert_eq!(body["material"], "Aluminium 6061");
        assert_eq!(body["assumed"], false);
        let properties = &body["properties"];
        let (volume, mass) = (bounded(&properties["volume"]), bounded(&properties["mass"]));
        // Aluminium: 2.7e-6 kg per mm³.
        assert!(
            Bounded {
                value: volume.value * 2.7e-6,
                error: volume.error * 2.7e-6 + 1e-15,
            }
            .contains(mass.value)
        );
        sum += mass.value;
        moment += mass.value * bounded(&properties["center"][0]).value;
    }
    let total = &report["total"];
    let (mass, x) = (bounded(&total["mass"]), bounded(&total["center"][0]));
    assert!((mass.value - sum).abs() <= mass.error + 1e-12);
    assert!((x.value - moment / sum).abs() <= x.error + 1e-9);

    // Placed apart, they do not interfere.
    let report = ask(&mut editor, "interference");
    assert_eq!(report["solids"], 2);
    assert_eq!(report["unchecked"], serde_json::json!([]), "{report}");
    let found = report["found"].as_array().unwrap();
    assert!(found.iter().all(|f| f["contact"] != "overlap"), "{report}");
}

/// The checks every solid's mass properties pass: converged, a positive
/// volume, a symmetric inertia tensor whose principal moments are positive
/// and satisfy the triangle inequality, as every body's do.
fn check_mass(name: &str, summary: &MassSummary, failures: &mut Vec<String>) {
    let mut fail = |why: String| failures.push(format!("{name}: {why}"));
    if !summary.converged {
        fail("did not converge".into());
    }
    if summary.volume.value - summary.volume.error <= 0.0 {
        fail(format!("volume {:?}", summary.volume));
    }
    for a in 0..3 {
        for b in 0..3 {
            let (x, y) = (summary.inertia[a][b], summary.inertia[b][a]);
            if (x.value - y.value).abs() > x.error + y.error {
                fail(format!("I[{a}][{b}] = {x:?} but I[{b}][{a}] = {y:?}"));
            }
        }
    }
    let [i1, i2, i3] = summary.principal_moments;
    if i1.value - i1.error <= 0.0 {
        fail(format!("a principal moment {i1:?} is not positive"));
    }
    if i1.value + i2.value + i1.error + i2.error < i3.value - i3.error {
        fail(format!("principal moments {i1:?} + {i2:?} < {i3:?}"));
    }
}

/// Every example part: every solid's mass properties pass [`check_mass`],
/// and its volume is that of its two pieces — cut across by a box, by
/// booleans, each piece's volume integrated on its own.
#[test]
#[ignore = "slow: every example's mass properties, and booleans halving each solid — run with `cargo test -- --ignored`"]
fn example_mass_properties_are_consistent() {
    let mut failures = Vec::new();
    for (example, program) in examples::all() {
        let part = program().built().unwrap();
        let reader = part.current();
        let report = mass_report(&reader).unwrap();
        for (body, placed) in report.bodies.iter().zip(placed_solids(&reader).unwrap()) {
            let name = format!("{example}/{}", body.name);
            let Some(summary) = &body.properties else {
                failures.push(format!("{name}: {:?}", body.error));
                continue;
            };
            check_mass(&name, summary, &mut failures);
            // The part of its box below a plane across x, a little beyond
            // it elsewhere. Not through the centre of mass: of a symmetric
            // part, that is where its circles' seams are, and a cut placed
            // on an existing edge asks the boolean for a degenerate crossing
            // (see `AGENTS.md`) — so a fraction of the width no construction
            // chooses.
            assert!(placed.pose.is_none(), "{name}: an example places no part");
            let faces = placed.model.topology().solid_faces(placed.solid).unwrap();
            let bounds = bounding_box(&placed, &faces).unwrap();
            let lo = |k: usize| bounds[k].lower().to_f64() - 1.0;
            let hi = |k: usize| bounds[k].upper().to_f64() + 1.0;
            let cut = bounds[0].lower().to_f64() + 0.437_281 * bounds[0].width().to_f64();
            let halves = [BooleanOp::Intersection, BooleanOp::Difference].map(|op| {
                let mut scratch = placed.model.clone().into_owned();
                let knife = cube_solid(
                    &mut scratch,
                    "knife",
                    Vector3::from_array([lo(0), lo(1), lo(2)].map(S::from_f64)),
                    Vector3::from_array([cut, hi(1), hi(2)].map(S::from_f64)),
                )?;
                let namer = Namer::new("halve", "half")?;
                let half = boolean(
                    &mut scratch,
                    &namer,
                    placed.solid,
                    knife,
                    op,
                    RemeshParams::default(),
                )?;
                let Some(half) = half else {
                    return Ok(S::ZERO);
                };
                Ok(scratch.topology().mass_properties(half)?.volume)
            });
            match halves {
                [Ok(a), Ok(b)] => {
                    let sum = Bounded::of(a.add(b));
                    let whole = summary.volume;
                    if (sum.value - whole.value).abs() > sum.error + whole.error {
                        failures.push(format!(
                            "{name}: halves {sum:?} do not add up to the whole {whole:?}"
                        ));
                    }
                }
                [a, b] => failures.push(format!(
                    "{name}: halving failed: {:?}",
                    [a.err(), b.err()]
                        .into_iter()
                        .flatten()
                        .map(|e: geop_core_math::geop_error::GeopError| e.to_string())
                        .collect::<Vec<_>>()
                )),
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// The linkages' bars, stacked each on the one it is pinned to, touch but
/// never overlap; neither do the pin and the plate it is placed in.
#[test]
#[ignore = "slow: interference of every pair of the assembly examples' parts — run with `cargo test -- --ignored`"]
fn assembly_examples_do_not_overlap() {
    for name in ["chain", "four_bar", "pin_in_plate"] {
        let report = ask(&mut workspace(name), "interference");
        assert_eq!(
            report["unchecked"],
            serde_json::json!([]),
            "{name}: {report:#}"
        );
        let found = report["found"].as_array().unwrap();
        let overlaps: Vec<_> = found.iter().filter(|f| f["contact"] == "overlap").collect();
        assert!(overlaps.is_empty(), "{name}: {overlaps:#?}");
    }
}
