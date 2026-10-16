//! Wire harnesses in assemblies, edited as a front end edits them: routes
//! between connectors on placed parts follow those parts wherever their
//! mates let them be moved, and the bundles they sweep are drawn, never
//! weighed.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::{StepEditEvent, Value},
    editor::{Command, Editor},
    output::Bounded,
    registry::Registry,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{DatumComponent, FrameAxis},
    scalars::{Ring, ScalInF64 as S, Scalar},
};
use geop_core_program::{
    program::Program,
    values::{Design, StateValue, StateValues, pose_state},
};
use geop_ops_assembly::{
    AddPartArgs, Kind, Mate, MateEntity, examples::pose, solids::placed_solids,
};
use geop_ops_datums::{ORIGIN, component_name, datum};
use geop_ops_model::read_model;
use geop_ops_reference_geometry::Selected;

use crate::{RouteArgs, Wire, WireSize, cable};

/// What these tests edit: plates drawn and extruded, placed and mated in
/// an assembly, routed between, weighed and listed.
fn registry() -> GeopResult<Registry> {
    let r = Registry::default();
    let r = geop_ops_model::register(r)?;
    let r = geop_ops_sketch::register(r)?;
    let r = geop_ops_datums::register(r)?;
    let r = geop_ops_reference_geometry::register(r)?;
    let r = geop_ops_extrude_revolve::register(r)?;
    let r = geop_ops_assembly::register(r)?;
    let r = crate::register(r)?;
    let r = geop_ops_inspect::register(r)?;
    let r = geop_ops_bom::register(r)?;
    geop_ops_parameters::register(r)
}

/// An editor of [`registry`] on [`assembly`], saved as `assembly.geop`,
/// with the plate it places beside it as `plate.geop` — the example box with a hole
/// drilled into it — which loads with every step built.
fn editor() -> Editor {
    let plate = geop_ops_extrude_revolve::examples::box_with_drill_hole();
    let files = BTreeMap::from([("plate.geop".to_string(), plate.to_json().unwrap())]);
    let mut editor = Editor::new(registry().unwrap(), files);
    let update = editor.handle(Command::Load {
        program: assembly(),
        path: Some("assembly.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let steps = update.program.unwrap().steps;
    assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");
    editor
}

/// An assembly of two plates: `a` fixed at the origin, `b` 4 along `x`,
/// its `xy` plane mated onto `a`'s, so it slides in that plane — and a
/// cable from `a`'s frame, leaving up its `z` axis, into `b`'s, arriving
/// down its `z` axis: a half circle over the gap between them.
fn assembly() -> Program {
    let mut program = Program::new();
    program.push(
        "a",
        AddPartArgs {
            file: "plate.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    let xy = |instance: &str| {
        MateEntity::of(
            instance,
            Selected::Datum(datum(component_name(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            ))),
        )
    };
    program.push(
        "b",
        AddPartArgs {
            file: "plate.geop".into(),
            mates: BTreeMap::from([(
                "m1".into(),
                Mate::constraint(Kind::Coincident, vec![xy("b"), xy("a")]),
            )]),
            ..Default::default()
        },
    );
    program.push(
        "cable",
        RouteArgs {
            through: vec![
                Selected::Datum(datum(ORIGIN)).behind("a"),
                Selected::Datum(datum(ORIGIN)).behind("b"),
            ],
            wires: vec![Wire {
                size: WireSize::Diameter(0.2),
                ..Wire::new("signal")
            }],
            fill: 1.0,
            bend_factor: 5.0,
            service_loop: 0.0,
        },
    );
    let at = |x: f64| StateValue::Pose(pose([x, 0.0, 0.0], [0.0; 3]));
    program.state = StateValues::from([(pose_state("a"), at(0.0)), (pose_state("b"), at(4.0))]);
    program
}

/// The cable's length in what `editor` built.
fn cable_length(editor: &Editor) -> Design {
    cable(&editor.part(), "route(cable)").unwrap().length
}

/// Moving the mated plate in the editor — its placement's `x` typed in —
/// reroutes the cable: the half circle over the gap grows with it, from
/// `pi 4 / 2` to `pi 6 / 2`.
#[test]
fn moving_a_mated_part_reroutes_the_cable() {
    let mut editor = editor();
    let half_circle = |gap: f64| S::PI.mul(S::from_f64(gap / 2.0));
    let before = cable_length(&editor);
    assert!(before.could_be_equal(half_circle(4.0)), "{before:?}");

    editor.handle(Command::Open { id: "b".into() });
    editor.handle(Command::Event {
        event: StepEditEvent::Dialog {
            key: "x".into(),
            value: Value::Number(6.0),
        },
    });
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let steps = update.program.unwrap().steps;
    assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");
    let after = cable_length(&editor);
    assert!(after.could_be_equal(half_circle(6.0)), "{after:?}");
    // The bundle's end cap went with the plate: centred on its frame.
    let reader = editor.part();
    let part = read_model(&reader).unwrap();
    let end = part.face_id("route(cable,end)").unwrap();
    let model = part.topology();
    let centre: f64 = model
        .iterate_face_coedges(end)
        .map(|c| model.coedge_start_vertex(c).unwrap().point[0].to_f64())
        .sum::<f64>()
        / 4.0;
    assert!((centre - 6.0).abs() < 1e-9, "{centre}");
}

/// A route's swept bundle is how its cable is drawn, not a body: weighing
/// the part, checking it for interference and its bill of materials all
/// leave it out — the bill orders its wires instead — so the mass the
/// inspector reports is the bill's.
#[test]
fn a_cable_bundle_is_no_body_to_weigh_or_check() {
    let editor = editor();
    let view = &editor.part();
    let solids: Vec<String> = (placed_solids(view).unwrap().into_iter())
        .map(|s| s.name)
        .collect();
    assert!(!solids.iter().any(|s| s.contains("route")), "{solids:?}");

    let mass = geop_ops_inspect::mass_report(view).unwrap();
    let names: Vec<&str> = mass.bodies.iter().map(|b| b.name.as_str()).collect();
    assert_eq!(names.len(), 2, "the plates alone: {names:?}");
    let interference = geop_ops_inspect::interference_report(view).unwrap();
    assert_eq!(interference.solids, 2, "{:?}", interference.found);

    let bill = geop_ops_bom::bom(view, "assembly.geop", geop_ops_bom::Structure::Flat).unwrap();
    let total = |b: Bounded| b.value;
    let weighed = mass.total.as_ref().unwrap().mass.value;
    assert!(
        (total(bill.total_mass.unwrap()) - weighed).abs() < 1e-12,
        "{:?} against {weighed}",
        bill.total_mass
    );
    let wires = (bill.lines.iter())
        .filter(|l| matches!(l.kind, geop_ops_bom::LineKind::Wire { .. }))
        .count();
    assert_eq!(wires, 1, "{:?}", bill.lines);
}
