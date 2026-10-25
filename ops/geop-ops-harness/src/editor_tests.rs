//! Wire harnesses in assemblies, edited as a front end edits them: routes
//! between connectors on placed parts follow those parts wherever their
//! mates let them be moved, and the bundles they sweep are drawn, never
//! weighed. A route on a part is picked by clicks, its wires chosen in the
//! dialog.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::{Button, Control, StepEditEvent, Style, Tone, Value},
    editor::{Command, Editor},
    output::Bounded,
    registry::Registry,
    selectable::Selectable,
    testing::{dialog, editor_with, pointer},
    view::drawn,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{DatumComponent, FrameAxis},
    scalars::Scalar,
};
use geop_core_program::{
    program::Program,
    values::{StateValue, StateValues, pose_state},
};
use geop_lib_fixtures::drilled_box;
use geop_ops_assembly::{AddPartArgs, Kind, Mate, examples::pose};
use geop_ops_model::datums::{ORIGIN, component_name, datum};
use geop_ops_model::mate_entity::MateEntity;
use geop_ops_model::placed::solids::placed_solids;
use geop_ops_model::read_model;
use geop_ops_model::selection::Selected;

use crate::{RouteArgs, Wire, WireSize, cable};

/// What these tests edit: given plates, placed and mated in an assembly,
/// routed between, weighed and listed.
fn registry() -> GeopResult<Registry> {
    let r = Registry::default();
    let r = geop_ops_model::register(r)?;
    let r = geop_lib_fixtures::register(r)?;
    let r = geop_ops_assembly::register(r)?;
    let r = crate::register(r)?;
    let r = geop_ops_inspect::register(r)?;
    let r = geop_ops_bom::register(r)?;
    Ok(r)
}

/// An editor of [`registry`] on [`assembly`], saved as `assembly.geop`,
/// with the plate it places beside it as `plate.geop` — the box with a hole
/// drilled into it (see [`drilled_box`]) — which loads with every step built.
fn editor() -> Editor {
    let plate = drilled_box();
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
fn cable_length(editor: &Editor) -> Scalar {
    cable(&editor.part(), "route(cable)").unwrap().length
}

/// Moving the mated plate in the editor — its placement's `x` typed in —
/// reroutes the cable: the half circle over the gap grows with it, from
/// `pi 4 / 2` to `pi 6 / 2`.
#[test]
fn moving_a_mated_part_reroutes_the_cable() {
    let mut editor = editor();
    let half_circle = |gap: f64| Scalar::PI.mul(Scalar::from_f64(gap / 2.0));
    let before = cable_length(&editor);
    assert!(before.could_be_equal(half_circle(4.0)), "{before:?}");

    editor.handle(Command::Open {
        id: "b".into(),
        placed: None,
    });
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

    let bill = geop_ops_model::placed::bom::bom(
        view,
        "assembly.geop",
        geop_ops_model::placed::bom::Structure::Flat,
    )
    .unwrap();
    let total = |b: Bounded| b.value;
    let weighed = mass.total.as_ref().unwrap().mass.value;
    assert!(
        (total(bill.total_mass.unwrap()) - weighed).abs() < 1e-12,
        "{:?} against {weighed}",
        bill.total_mass
    );
    let wires = (bill.lines.iter())
        .filter(|l| matches!(l.kind, geop_ops_model::placed::bom::LineKind::Wire { .. }))
        .count();
    assert_eq!(wires, 1, "{:?}", bill.lines);
}

/// A new route picks what it runs through by clicks, in order: down
/// through the box's hole — its rim on top, its rim at the bottom, both
/// clips — and out to the box's far bottom corner. Too tight for the
/// default hookup wire, it is refused, the bend drawn as failed; a thinner
/// wire, chosen in the dialog, fits, and once built the dialog reports the
/// route's length and every wire's cut length.
#[test]
fn new_route_is_picked_and_its_wires_chosen() {
    let mut editor = editor_with(registry().unwrap(), drilled_box());
    let update = editor.handle(Command::New {
        kind: "route".into(),
    });
    let step = update.step.expect("the route is edited");
    let pickable = &step.presentation.pickable;
    assert!(
        [drawn::VERTEX, drawn::EDGE]
            .iter()
            .all(|k| pickable.contains(&k.to_string())),
        "{pickable:?}"
    );
    assert_eq!(step.missing, ["through"]);
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    // The hole, 0.4 round `(1, 1)` and 0.5 deep, seen at a slant: its rim
    // on the box's top, then its rim on its floor, through the hole — both
    // halfway between the vertices that split each circle.
    let rim = 1.0 + 0.4 * std::f64::consts::FRAC_1_SQRT_2;
    editor.handle(click(pointer(
        [rim + 4.0, rim + 4.0, 1.0 + 5.0],
        [-4.0, -4.0, -5.0],
    )));
    editor.handle(click(pointer(
        [rim - 0.3, rim - 0.3, 1.5],
        [0.3, 0.3, -1.0],
    )));
    // The box's bottom corner at `(2, 2, 0)`, from below.
    let update = editor.handle(click(pointer([2.0, 2.0, -10.0], [0.0, 0.0, 1.0])));
    let step = update.step.expect("the route is edited");
    let Some(Control::Reference(through)) = step.presentation.dialog.get("through") else {
        panic!("what the route runs through is picked");
    };
    // The rims as edges, the corner as its vertex: a point, which leaves
    // the route free to choose its way.
    let picked: Vec<Selected> = (through.elements())
        .map(|e| Selected::from_picked(e).expect("an element of the part"))
        .collect();
    assert!(
        matches!(
            picked.as_slice(),
            [Selected::Edge(_), Selected::Edge(_), Selected::Vertex(_)]
        ),
        "{picked:?}"
    );
    // The default wire bundles 1.4 across, which may bend no tighter than
    // 7.2: the turn out to the corner is far tighter.
    let error = step.error.expect("too tight a bend is refused");
    assert!(
        error.contains("may bend no tighter than radius 7.18"),
        "{error}"
    );
    assert!(error.contains("between point 2"), "{error}");
    assert!(
        step.presentation
            .visuals
            .iter()
            .any(|v| v.key.starts_with("route:") && v.style == Style::Failed),
        "the bend too tight is drawn as failed"
    );

    // The wire, selected, given a diameter of its own: 0.1 across.
    editor.handle(dialog("wire:0", Value::Press));
    editor.handle(dialog("wire_size", Value::Choice("diameter".into())));
    let update = editor.handle(dialog("wire_diameter", Value::Number(0.1)));
    let step = update.step.expect("the route is edited");
    assert_eq!(step.error, None);
    let update = editor.handle(dialog("service_loop", Value::Number(0.25)));
    assert_eq!(update.step.expect("the route is edited").error, None);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert!(
        editor
            .handle(Command::Show)
            .scene
            .unwrap()
            .part
            .solids
            .contains(&"route(route1)".into()),
        "the bundle is drawn"
    );

    let cable = cable(&editor.part(), "route(route1)")
        .expect("the route's cable")
        .clone();
    // Straight down the hole, 0.5, then a single arc out to the corner.
    assert!(cable.length.to_f64() > 0.5 + 1.5, "{:?}", cable.length);
    let cut = cable.wires[0].cut_length.to_f64() - cable.length.to_f64();
    assert!((cut - 0.5).abs() < 1e-9, "{cut}");
    let update = editor.handle(Command::Open {
        id: "route1".into(),
        placed: None,
    });
    let presentation = update.step.expect("the route is edited").presentation;
    let Some(Control::Text { text, tone }) = presentation.dialog.get("report") else {
        panic!("what was built is reported: {:?}", presentation.dialog);
    };
    assert_eq!(*tone, Tone::Success);
    assert!(
        text.starts_with(&format!("Route {:.1} long", cable.length.to_f64())),
        "{text}"
    );
    let Some(Control::List { items, .. }) = presentation.dialog.get("wires") else {
        panic!("the wires are listed");
    };
    let detail = items[0].detail.as_deref().unwrap();
    let cut_length = cable.wires[0].cut_length.to_f64();
    assert!(
        detail.ends_with(&format!("cut {cut_length:.1}")),
        "{detail}"
    );
}
