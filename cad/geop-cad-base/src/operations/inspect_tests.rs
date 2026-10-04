//! The inspect tools on whole programs: the mass properties of assemblies,
//! each placed part of its own material where it is placed, interference
//! between placed parts — and, ignored for their time, every example's
//! mass properties checked against themselves.

use geop_core_math::scalars::{Ring, ScalInF64 as S, Scalar};
use geop_core_math::vector::Vector3;
use geop_ops::{Namer, NoFiles, Part};
use geop_ops_booleans::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};
use geop_ops_extrude_revolve::shapes::cube::cube_solid;
use geop_ops_inspect::{
    Bounded,
    bodies::placed_solids,
    interference::Contact,
    mass::{MassSummary, mass_report},
};

use crate::{
    Command, Editor, examples,
    inspect::{Inspection, Query},
};

/// The editor with the workspace example `name` loaded, as the browser
/// loads one.
fn workspace(name: &str) -> Editor<S> {
    let mut editor = Editor::new();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: name.into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor
}

/// Two plates placed in an assembly, both aluminium as their file says:
/// each weighed where it is placed, the total their sum, its centre
/// between theirs. Asked through the editor, as the inspect panel asks.
#[test]
fn an_assembly_weighs_its_placed_parts() {
    let mut editor = workspace("parametric_plates");
    let update = editor.handle(Command::Inspect {
        query: Query::MassProperties,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let Some(Inspection::MassProperties(report)) = update.inspection else {
        panic!("no mass properties: {:?}", update.inspection);
    };
    assert_eq!(report.bodies.len(), 2, "{report:?}");
    let mut sum = 0.0;
    let mut moment = 0.0;
    for body in &report.bodies {
        assert!(body.name.contains('/'), "{} is of a placed part", body.name);
        assert_eq!(body.material, "Aluminium 6061");
        assert!(!body.assumed);
        let properties = body.properties.as_ref().expect("weighed");
        // Aluminium: 2.7e-6 kg per mm³.
        assert!(
            Bounded {
                value: properties.volume.value * 2.7e-6,
                error: properties.volume.error * 2.7e-6 + 1e-15,
            }
            .contains(properties.mass.value)
        );
        sum += properties.mass.value;
        moment += properties.mass.value * properties.center[0].value;
    }
    let total = report.total.expect("a total");
    assert!((total.mass.value - sum).abs() <= total.mass.error + 1e-12);
    assert!((total.center[0].value - moment / sum).abs() <= total.center[0].error + 1e-9);

    // Placed apart, they do not interfere.
    let update = editor.handle(Command::Inspect {
        query: Query::Interference,
    });
    let Some(Inspection::Interference(report)) = update.inspection else {
        panic!("no interference: {:?}", update.inspection);
    };
    assert_eq!(report.solids, 2);
    assert!(report.unchecked.is_empty(), "{report:?}");
    assert!(
        report.found.iter().all(|f| f.contact != Contact::Overlap),
        "{report:?}"
    );
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

/// The sheet-metal bracket's mass properties converge on every face — its
/// bends' faces among them, whose integrals once stalled.
#[test]
fn sheet_metal_bracket_mass_properties_converge() {
    let part: Part<S> = examples::sheet_metal_bracket().build(&NoFiles).unwrap();
    for placed in placed_solids(&part).unwrap() {
        let mass = placed.mass_properties().unwrap();
        let unresolved: Vec<&str> = mass
            .unresolved
            .iter()
            .map(|&f| part.name_of(f).unwrap_or("?"))
            .collect();
        assert!(
            unresolved.is_empty(),
            "{}: not resolved on {unresolved:?}",
            placed.name
        );
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
        let part: Part<S> = program.build(&NoFiles).unwrap();
        let report = mass_report(&part).unwrap();
        for (body, placed) in report.bodies.iter().zip(placed_solids(&part).unwrap()) {
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
            let bounds = placed.bounding_box().unwrap();
            let lo = |k: usize| bounds[k].lower().to_f64() - 1.0;
            let hi = |k: usize| bounds[k].upper().to_f64() + 1.0;
            let cut = bounds[0].lower().to_f64() + 0.437_281 * bounds[0].width().to_f64();
            let halves = [BooleanOp::Intersection, BooleanOp::Difference].map(|op| {
                let mut scratch = part.clone();
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
                Ok(scratch.topology().mass_properties(half, S::ONE)?.volume)
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
        let mut editor = workspace(name);
        let update = editor.handle(Command::Inspect {
            query: Query::Interference,
        });
        assert!(update.error.is_none(), "{name}: {:?}", update.error);
        let Some(Inspection::Interference(report)) = update.inspection else {
            panic!("{name}: no interference: {:?}", update.inspection);
        };
        assert!(report.unchecked.is_empty(), "{name}: {report:#?}");
        let overlaps: Vec<_> = report
            .found
            .iter()
            .filter(|f| f.contact == Contact::Overlap)
            .collect();
        assert!(overlaps.is_empty(), "{name}: {overlaps:#?}");
    }
}
