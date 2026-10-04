//! Wire harnesses in assemblies: routes between connectors on placed
//! parts follow those parts wherever their mates let them be moved.

use std::collections::BTreeMap;

use geop_core_math::{
    primitives::{DatumComponent, FrameAxis},
    scalars::{Ring, ScalInF64 as S, Scalar},
};
use geop_ops::{
    EntityRef,
    assembly::{Kind, Mate},
    part::{ParamValue, State, pose_parameter},
    ui::{StepEditEvent, Value},
};
use geop_ops_assembly::AddPartArgs;
use geop_ops_harness::{RouteArgs, Wire, WireSize};

use crate::examples::pose;
use crate::{Command, Editor, Program, examples};

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
            fixed: true,
            ..Default::default()
        },
    );
    let xy = |instance: &str| {
        EntityRef::datum_component(
            format!("{instance}/origin"),
            DatumComponent::Plane(FrameAxis::Z),
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
            through: vec![EntityRef::datum("a/origin"), EntityRef::datum("b/origin")],
            wires: vec![Wire {
                size: WireSize::Diameter(0.2),
                ..Wire::new("signal")
            }],
            fill: 1.0,
            bend_factor: 5.0,
            service_loop: 0.0,
        },
    );
    let at = |x: f64| ParamValue::Pose(pose([x, 0.0, 0.0], [0.0; 3]));
    program.state = State::from([
        (pose_parameter("a"), at(0.0)),
        (pose_parameter("b"), at(4.0)),
    ]);
    program
}

/// The cable's length in what `editor` built.
fn cable_length(editor: &Editor<S>) -> S {
    editor.part().cable("route(cable)").unwrap().length
}

/// Moving the mated plate in the editor — its placement's `x` typed in —
/// reroutes the cable: the half circle over the gap grows with it, from
/// `pi 4 / 2` to `pi 6 / 2`.
#[test]
fn moving_a_mated_part_reroutes_the_cable() {
    let mut editor = Editor::<S>::new();
    let files = BTreeMap::from([(
        "plate.geop".to_string(),
        Some(examples::box_with_drill_hole().to_json().unwrap()),
    )]);
    let update = editor.handle(Command::Files { files });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Load {
        program: assembly(),
        path: Some("assembly.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let steps = update.program.unwrap().steps;
    assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");
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
    let part = editor.part();
    let end = part.face_id("route(cable,end)").unwrap();
    let model = part.topology();
    let centre: f64 = model
        .iterate_face_coedges(end)
        .map(|c| model.coedge_start_vertex(c).unwrap().point[0].to_f64())
        .sum::<f64>()
        / 4.0;
    assert!((centre - 6.0).abs() < 1e-9, "{centre}");
}
