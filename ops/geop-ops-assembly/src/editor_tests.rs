//! Parts placed as an editor places them: the step placing one edited
//! through the [`geop_core_editor::editor::Editor`], which solves the mates after every edit
//! with what the step pulls. Standard parts placed from the workspace's
//! files, joints dragged and set in the joints panel, placed parts moved
//! by the gizmo, listed in the structure, and written as STL.

use geop_core_program::{
    element::{ElementReference, PickedElement},
    placed::Located,
    program::Program,
    testing::{args as step_args, edit_args},
    values::{Setting, StateValue, pose_state},
};
use std::collections::BTreeMap;

use crate::{AddPartArgs, JointsCommand, JointsPanel, Kind, Mate, MateKind, program_tests::editor};
use geop_core_editor::{
    editing::{Axis, Control, GizmoPart, Pointer, StepEditEvent, Tone, Value},
    editor::{Editor, ProgramState, Update},
    output::Content,
    registry::{Panel, StructureItem},
    selectable::Selectable,
};
use geop_core_math::{
    primitives::{DatumComponent, FrameAxis, Pose},
    scalars::Scalar,
    vector::Vector3,
};
use geop_lib_fixtures::{PrimitiveArgs, Shape, drilled_box};
use geop_ops_model::datums::{ORIGIN, component_name, datums};
use geop_ops_model::mate_entity::MateEntity;
use geop_ops_model::placed::placed_parts;
use geop_ops_model::selection::Selected;
use geop_ops_model::{element, testing::assert_close};

use geop_core_editor::{
    editor::Command,
    testing::{Stepping, dialog, drag_gizmo, from_above, handled, pointer},
};

/// The unit cube from the origin to `(1, 1, 1)`, as a program.
fn cube() -> Program {
    let mut program = Program::new();
    program.push("c", PrimitiveArgs::block([0.0; 3], [1.0; 3]));
    program
}

/// A step placing `cube.geop`, held fixed if `fixed`, with the mates
/// `mates`.
fn args(fixed: bool, mates: &[(&str, MateKind, [MateEntity; 2])]) -> AddPartArgs {
    let args = AddPartArgs {
        file: "cube.geop".into(),
        mates: (mates.iter())
            .map(|(id, kind, entities)| {
                let mate = Mate {
                    kind: *kind,
                    entities: entities.to_vec(),
                    joints: Vec::new(),
                };
                (id.to_string(), mate)
            })
            .collect(),
        ..Default::default()
    };
    if fixed { args.fixed() } else { args }
}

/// The datum `name` of the part placed as `instance`.
fn datum(instance: &str, name: String) -> MateEntity {
    MateEntity::of(
        instance,
        Selected::Datum(ElementReference::new(datums(), name)),
    )
}

/// The frame of the part placed as `instance`.
fn origin(instance: &str) -> MateEntity {
    datum(instance, ORIGIN.to_string())
}

/// The `xy` plane of the frame of the part placed as `instance`.
fn base(instance: &str) -> MateEntity {
    datum(
        instance,
        component_name(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
    )
}

/// The datum `name` of the part placed as `part`, as a pick names it.
fn placed_datum(part: &str, name: &str) -> PickedElement {
    datum(part, name.into()).picked()
}

/// The face `name` of the part placed as `part`, as a pick names it.
fn placed_face(part: &str, name: &str) -> PickedElement {
    MateEntity::of(part, Selected::Face(element(name))).picked()
}

/// The command setting the value `parameter` a step offers a user to set
/// to `value`, as the joints panel sends it.
fn set_joint(parameter: &str, value: f64) -> Command {
    let parameter = parameter.into();
    Command::panel::<JointsPanel>(&JointsCommand::Set { parameter, value })
        .expect("a joint's value is plain data")
}

/// What the joints panel shows of `program`.
fn joints_panel(program: &ProgramState) -> &serde_json::Value {
    &program.panels[JointsPanel::KEY]
}

fn n(x: f64) -> Scalar {
    Scalar::from_f64(x)
}

/// Where `pose` puts the point `p`.
fn apply(pose: &Pose, p: [f64; 3]) -> [f64; 3] {
    let q = pose.apply(&Vector3::from_array(p.map(n)));
    [0, 1, 2].map(|k| q[k].to_f64())
}

/// Looking straight down at `(x, y, z)`.
fn down_at(x: f64, y: f64, z: f64) -> Pointer {
    pointer([x, y, z + 10.0], [0.0, 0.0, -1.0])
}

/// The cube placed as `a`, fixed at the origin, and then as `b` by
/// `placed`, in a workspace holding `cube.geop`: the step placing `b`
/// opened.
fn placing(placed: AddPartArgs) -> Stepping {
    let mut editor = editor();
    let files = BTreeMap::from([("cube.geop".to_string(), Some(cube().to_json().unwrap()))]);
    handled(&mut editor, Command::Files { files });
    let mut program = Program::new();
    program.push("a", args(true, &[]));
    program.push("b", placed);
    handled(
        &mut editor,
        Command::Load {
            program,
            path: Some("assembly.geop".into()),
        },
    );
    let shown = handled(
        &mut editor,
        Command::Open {
            id: "b".into(),
            placed: None,
        },
    )
    .step
    .expect("the step is edited");
    Stepping { editor, shown }
}

/// Where the part placed as `instance` is, as the editor last ran the
/// program.
fn moved_to(step: &Stepping, instance: &str) -> Pose {
    let reader = step.editor.part();
    let placed = placed_parts(&reader).unwrap();
    let placed = (placed.iter()).find(|p| p.name == instance);
    placed
        .unwrap_or_else(|| panic!("no part {instance}"))
        .located
        .world
}

/// Adding a mate in the editor arms its entities to be picked; once it has
/// both, picking is done.
#[test]
fn adding_a_mate_arms_its_entities() {
    let mut step = placing(args(false, &[]));
    step.dialog("add_mate", Value::Choice("coincident".into()));
    let Some(Control::Reference(entities)) = step.presentation().dialog.get("mate:m1:entities")
    else {
        panic!("the new mate's entities are a reference field");
    };
    assert!(entities.armed, "a new mate waits for its entities");

    let pair = vec![origin("b"), origin("a")];
    let picked: Vec<PickedElement> = pair.iter().map(Selectable::picked).collect();
    step.dialog("mate:m1:entities", Value::Elements(picked));
    assert!(
        step.presentation().dialog.get("mate:m1:entities").is_none(),
        "done picking"
    );
    assert_eq!(step.args::<AddPartArgs>().mates["m1"].entities, pair);
}

/// Typing where the part goes pulls it there, turned as typed.
#[test]
fn the_placement_fields_pull_the_part_there() {
    let mut step = placing(args(false, &[]));
    step.dialog("x", Value::Number(2.0));
    step.dialog("rotation_z", Value::Number(90.0));
    let pose = moved_to(&step, "b");
    assert_close(apply(&pose, [0.0; 3]), [2.0, 0.0, 0.0], 1e-6);
    assert_close(apply(&pose, [1.0, 0.0, 0.0]), [2.0, 1.0, 0.0], 1e-6);
}

/// A fixed part goes where it is dragged: no mate moves it, so the solve
/// moves it alone.
#[test]
fn a_fixed_part_goes_where_it_is_dragged() {
    let mut step = placing(args(true, &[]));
    step.send(StepEditEvent::Hover {
        pointer: down_at(0.5, 0.5, 1.0),
        shift: false,
    });
    step.send(StepEditEvent::Drag {
        from: down_at(0.5, 0.5, 1.0),
        to: down_at(2.5, 0.0, 1.0),
        done: true,
        shift: false,
    });
    let pose = moved_to(&step, "b");
    assert_close(apply(&pose, [0.0; 3]), [2.0, -0.5, 0.0], 1e-6);
}

/// Mates that cannot all hold are listed as such.
#[test]
fn mates_that_cannot_hold_are_said_so() {
    let distance = |value| MateKind::Constraint(Kind::Distance { value: n(value) });
    let mates = args(
        false,
        &[
            ("m1", distance(1.0), [base("b"), base("a")]),
            ("m2", distance(2.0), [base("b"), base("a")]),
        ],
    );
    let step = placing(mates);
    let Some(Control::List { items, .. }) = step.presentation().dialog.get("mates") else {
        panic!("the mates are a list");
    };
    assert!(items.iter().any(|i| i.tone == Tone::Error), "{items:?}");
}

/// Placing the plate, its parameters are offered as what they are: its
/// colour to pick, its numbers on sliders over their ranges, its screw
/// from a table to search — showing what the plate is built with here,
/// the depth following the width given — and what is set there is what
/// the plate is placed with.
#[test]
fn placing_a_part_offers_its_parameters() {
    let mut editor = editor();
    let plate = geop_ops_extrude_revolve::examples::parametric_plate();
    let files = BTreeMap::from([("plate.geop".to_string(), Some(plate.to_json().unwrap()))]);
    assert!(editor.handle(Command::Files { files }).error.is_none());
    handled(
        &mut editor,
        Command::Load {
            program: crate::examples::plates_assembly(),
            path: Some("plates.geop".into()),
        },
    );
    let update = editor.handle(Command::Open {
        id: "large".into(),
        placed: None,
    });
    let dialog = update.step.unwrap().presentation.dialog;
    assert!(matches!(
        dialog.get("parameter:color"),
        Some(Control::Color { value, .. }) if value == "#3e7bd0"
    ));
    let Some(Control::Number(width)) = dialog.get("parameter:width") else {
        panic!("width on a slider")
    };
    assert_eq!((width.value, width.range), (5.0, Some([1.0, 10.0])));
    let Some(Control::Number(depth)) = dialog.get("parameter:depth") else {
        panic!("depth on a slider")
    };
    assert!((depth.value - 2.5).abs() < 1e-9, "{}", depth.value);
    let Some(Control::Select {
        options,
        searchable,
        value,
        ..
    }) = dialog.get("parameter:screw")
    else {
        panic!("the screw from its table")
    };
    assert!(*searchable && value == "M6" && options.len() == 4);

    handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Dialog {
                key: "parameter:screw".into(),
                value: Value::Choice("M3".into()),
            },
        },
    );
    handled(&mut editor, Command::Commit);
    let args: AddPartArgs = step_args(&editor.program().steps[1]);
    assert_eq!(args.parameters["screw"], Setting::Text("M3".into()));
}

/// The number a program's state gives `name`.
fn state_number(editor: &Editor, name: &str) -> f64 {
    match editor.program().state.get(name) {
        Some(StateValue::Number(v)) => v.to_f64(),
        other => panic!("{name} is no number: {other:?}"),
    }
}

/// How far the placed part `instance` is turned about `z`, in degrees.
fn turn_of(editor: &Editor, instance: &str) -> f64 {
    match editor.program().state.get(&pose_state(instance)) {
        Some(StateValue::Pose(pose)) => pose.euler_degrees()[2],
        other => panic!("{instance} has no pose: {other:?}"),
    }
}

/// The arm's forearm turns on a revolute joint limited to ±150°: dragged
/// round past the limit by a point of it, it turns about the joint's axis
/// and stops at 150° — the joint's angle, in the program's state, exactly
/// the limit, and shown in the step's dialog. Set in the dialog, the joint
/// turns the forearm there; set in the program's panel, the wrist turns the
/// hand.
#[test]
fn a_revolute_joint_is_dragged_up_to_its_limit() {
    let mut editor = editor();
    let update = handled(
        &mut editor,
        Command::LoadWorkspaceExample {
            name: "arm".into(),
            folder: None,
        },
    );
    let elbow = "add_part(fore,m1).angle";
    assert_eq!(state_number(&editor, elbow), 30.0);
    let program = update.program.unwrap();
    let joints = joints_panel(&program);
    assert_eq!(joints["joints"].as_array().unwrap().len(), 2, "{joints:?}");
    let freedom = &joints["freedom"];
    assert_eq!(freedom["parts"]["upper"], 0);
    assert_eq!(freedom["parts"]["fore"], 1);
    assert_eq!(freedom["parts"]["hand"], 2);

    handled(
        &mut editor,
        Command::Open {
            id: "fore".into(),
            placed: None,
        },
    );
    // Grabbed halfway along, where nothing lies on it, and dragged round
    // the elbow at (3, 0) to 170°, looking down.
    let down = |[x, y]: [f64; 2]| from_above(x, y);
    let round = |degrees: f64| {
        let a = degrees.to_radians();
        [3.0 + 1.5 * a.cos(), 1.5 * a.sin()]
    };
    let mut done = false;
    for (k, angle) in [60.0, 100.0, 140.0, 170.0].into_iter().enumerate() {
        done = k == 3;
        handled(
            &mut editor,
            Command::Event {
                event: StepEditEvent::Drag {
                    from: down(round(30.0)),
                    to: down(round(angle)),
                    done,
                    shift: false,
                },
            },
        );
    }
    assert!(done);
    handled(&mut editor, Command::Commit);
    assert_eq!(state_number(&editor, elbow), 150.0);
    assert!((turn_of(&editor, "fore") - 150.0).abs() < 1e-9);
    editor.handle(Command::Open {
        id: "fore".into(),
        placed: None,
    });
    let step = editor.handle(dialog("mate:m1", Value::Press)).step.unwrap();
    let Some(Control::Number(angle)) = step.presentation.dialog.get("mate:m1:angle") else {
        panic!("the joint's angle is shown");
    };
    assert_eq!(angle.value, 150.0);
    assert!(step.presentation.dialog.get("mate:m1:max").is_some());

    // Set in the dialog: the forearm turns there.
    handled(&mut editor, dialog("mate:m1:angle", Value::Number(-60.0)));
    handled(&mut editor, Command::Commit);
    assert_eq!(state_number(&editor, elbow), -60.0);
    let fore = turn_of(&editor, "fore");
    assert!(
        (fore + 60.0).abs() < 1e-9,
        "{fore} {:?}",
        editor.program().state
    );

    // Set in the program's panel: the hand turns, the forearm stays.
    let wrist = "add_part(hand,m1).angle";
    let update = handled(&mut editor, set_joint(wrist, 90.0));
    assert_eq!(state_number(&editor, wrist), 90.0);
    assert!((turn_of(&editor, "fore") + 60.0).abs() < 1e-9);
    assert!((turn_of(&editor, "hand") - 30.0).abs() < 1e-9);
    let program = update.program.unwrap();
    let joints = &joints_panel(&program)["joints"];
    assert_eq!(joints[1]["values"][0]["value"], 90.0);
    assert_eq!(joints[1]["values"][0]["max"], 120.0);
}

/// A new part step's file explorer lists every file of the workspace by
/// its path relative to the program — the programs, which it takes, and
/// the rest, which it does not, and the standard parts — but not the
/// program itself, and offers no upload; a program in a folder, chosen by
/// its path, is placed.
#[test]
fn a_part_is_placed_from_a_folder_through_the_file_explorer() {
    let mut editor = editor();
    let plate = crate::examples::metric_plate().to_json().unwrap();
    let files = [
        ("parts/bracket/plate.geop".to_string(), Some(plate)),
        ("parts/notes.txt".to_string(), Some("notes".to_string())),
    ]
    .into();
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("assembly.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);

    let update = editor.handle(Command::New {
        kind: "add_part".into(),
    });
    let step = update.step.unwrap();
    let Some(Control::File { files, upload, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is chosen in an explorer");
    };
    let (standard, own): (Vec<_>, Vec<_>) =
        (files.iter()).partition(|f| f.path.starts_with("std:"));
    let own: Vec<(&str, bool)> = (own.iter())
        .map(|f| (f.path.as_str(), f.accepted))
        .collect();
    assert_eq!(
        own,
        [
            ("parts/bracket/plate.geop", true),
            ("parts/notes.txt", false)
        ]
    );
    assert!(!standard.is_empty() && standard.iter().all(|f| f.accepted));
    assert!(upload.is_empty(), "{upload:?}");

    let update = editor.handle(dialog(
        "file",
        Value::Choice("parts/bracket/plate.geop".into()),
    ));
    let step = update.step.unwrap();
    assert!(step.error.is_none(), "{:?}", step.error);
    let Some(Control::File { value, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is chosen in an explorer");
    };
    assert_eq!(value, "parts/bracket/plate.geop");
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let scene = editor.handle(Command::Show).scene.expect("the scene");
    assert_eq!(scene.part.instances.len(), 1, "the plate is placed");
}

/// A standard screw is placed as any part: offered among the files, its
/// size picked from its table, and mated by its datums — its axis on the
/// hole's, the underside of its head on the plate.
#[test]
fn a_standard_screw_is_placed_in_a_plate() {
    let screw = "std:iso4762_socket_head_cap_screw.geop";
    let (plate, hole) = (
        crate::examples::metric_plate(),
        crate::examples::metric_plate_hole(),
    );
    let mut editor = editor();
    let files = [("plate.geop".to_string(), Some(plate.to_json().unwrap()))].into();
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("assembly.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);

    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice("plate.geop".into())));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let update = editor.handle(Command::New {
        kind: "add_part".into(),
    });
    let step = update.step.unwrap();
    let Some(Control::File { files, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is chosen in an explorer");
    };
    assert!(
        (files.iter()).any(|f| f.path == screw && f.accepted),
        "{files:?}"
    );
    editor.handle(dialog("file", Value::Choice(screw.into())));
    let update = editor.handle(dialog("parameter:size", Value::Choice("M4x12".into())));
    assert!(update.error.is_none(), "{:?}", update.error);
    let mates = [
        (
            "m1",
            "concentric",
            vec![placed_datum("part2", "axis"), placed_face("part1", &hole)],
        ),
        (
            "m2",
            "coincident",
            vec![
                placed_datum("part2", "seat"),
                placed_face("part1", "extrude(plate,end)"),
            ],
        ),
    ];
    for (id, kind, entities) in mates {
        editor.handle(dialog("add_mate", Value::Choice(kind.into())));
        let update = editor.handle(dialog(
            &format!("mate:{id}:entities"),
            Value::Elements(entities),
        ));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let program = editor.program();
    let args: crate::AddPartArgs = step_args(&program.steps[1]);
    assert_eq!(
        args.parameters.get("size"),
        Some(&Setting::Text("M4x12".into()))
    );
    assert!(crate::solving::check(&editor.part()).unwrap().converged);
    let Some(StateValue::Pose(pose)) = program.state.get(&pose_state("part2")) else {
        panic!("the screw has a pose");
    };
    // Its head on the plate's top, its tip 12 below, down the hole.
    let v = |p: [f64; 3]| Vector3::from_array(p.map(Scalar::from_f64));
    for (local, want) in [
        ([0.0, 0.0, 0.0], [20.0, 20.0, 5.0]),
        ([0.0, 0.0, -12.0], [20.0, 20.0, -7.0]),
    ] {
        let got = pose.apply(&v(local));
        let got = [0, 1, 2].map(|k| got[k].to_f64());
        assert!(
            (0..3).all(|k| (got[k] - want[k]).abs() < 1e-6),
            "{local:?} is at {got:?}, not {want:?}"
        );
    }
}

/// A linear guide is placed from the standard parts as the front end does:
/// the rail, its size and length picked in its dialog, then a carriage of
/// the same size on it by a slider joint between their `axis` datums —
/// which leaves it the one motion along the rail, and puts it on the rail.
#[test]
fn a_carriage_slides_on_a_standard_rail() {
    let (rail, carriage) = ("std:linear_rail.geop", "std:linear_carriage.geop");
    let mut editor = editor();
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("guide.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);

    let update = editor.handle(Command::New {
        kind: "add_part".into(),
    });
    let step = update.step.unwrap();
    let Some(Control::File { files, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is chosen in an explorer");
    };
    for file in [rail, carriage, "std:spur_gear.geop", "std:servo_sg90.geop"] {
        assert!(
            (files.iter()).any(|f| f.path == file && f.accepted),
            "{file}: {files:?}"
        );
    }
    editor.handle(dialog("file", Value::Choice(rail.into())));
    editor.handle(dialog("fixed", Value::Bool(true)));
    editor.handle(dialog("parameter:size", Value::Choice("MGN9".into())));
    let update = editor.handle(dialog("parameter:length", Value::Number(300.0)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice(carriage.into())));
    editor.handle(dialog("parameter:size", Value::Choice("MGN9H".into())));
    editor.handle(dialog("add_mate", Value::Choice("slider".into())));
    let update = editor.handle(dialog(
        "mate:m1:entities",
        Value::Elements(vec![
            placed_datum("part1", "axis"),
            placed_datum("part2", "axis"),
        ]),
    ));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let program = editor.program();
    let args: crate::AddPartArgs = step_args(&program.steps[0]);
    assert_eq!(args.parameters.get("length"), Some(&Setting::Number(300.0)));
    let part = editor.part();
    assert!(crate::solving::check(&part).unwrap().converged);
    let joints = crate::solving::settings(&part);
    assert_eq!(joints.len(), 1, "{joints:?}");
    assert_eq!(joints[0].kind, "Slider");
    // On the rail: its axis on the rail's — the carriage drawn sitting on a
    // rail at the origin, so its pose turns nothing and moves it only along
    // the rail.
    // A part the solve left where it was drawn has no pose in the state.
    let pose = match program.state.get(&pose_state("part2")) {
        Some(StateValue::Pose(pose)) => *pose,
        None => Pose::identity(),
        other => panic!("the carriage's pose is {other:?}"),
    };
    let origin = pose.apply(&Vector3::zero());
    let origin = [0, 1, 2].map(|k| origin[k].to_f64());
    assert!(
        origin[0].abs() < 1e-6 && origin[1].abs() < 1e-6,
        "{origin:?}"
    );
}

/// "Download STL", as the File menu offers it: the exporters the program
/// state lists, the one picked asked for by its kind. It writes every
/// solid of the part shown as a binary STL mesh, named after the program's
/// file — the placed parts' solids too, so the pin in the plate adds to the
/// plate's triangles. An exporter there is none of is refused, naming those
/// there are; so is a format the exporter does not write.
#[test]
fn a_part_is_exported_as_stl_with_its_placed_parts() {
    /// The triangles of a binary STL file, checked against its length.
    fn triangles(update: Update) -> u32 {
        assert!(update.error.is_none(), "{:?}", update.error);
        let export = update.export.expect("a file is written");
        assert_eq!(export.media_type, "model/stl");
        let Content::Bytes(bytes) = export.content else {
            panic!("a binary file");
        };
        assert!(!bytes.starts_with(b"solid"), "a binary header");
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap());
        assert_eq!(bytes.len(), 84 + 50 * count as usize);
        count
    }
    let export = |exporter: &str, format: Option<&str>, options| Command::Export {
        exporter: exporter.into(),
        format: format.map(str::to_string),
        options,
        date: String::new(),
    };
    let stl = || export("stl", None, serde_json::Value::Null);
    let mut editor = editor();
    let update = handled(
        &mut editor,
        Command::Load {
            program: drilled_box(),
            path: None,
        },
    );
    let offered = update.program.unwrap().exporters;
    assert!(
        offered
            .iter()
            .any(|e| e.kind == "stl" && e.formats == ["stl"]),
        "{offered:?}"
    );
    let plate = triangles(editor.handle(stl()));
    assert!(plate > 0);

    editor.handle(Command::LoadWorkspaceExample {
        name: "pin_in_plate".into(),
        folder: None,
    });
    let json = (editor
        .handle_json(r#"{"command": "export", "exporter": "stl", "date": "2026-10-11"}"#))
    .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert_eq!(update["export"]["name"], "assembly.stl");
    assert!(update["export"]["bytes"].is_string(), "{update}");
    let assembly = triangles(editor.handle(stl()));
    assert!(
        assembly > plate,
        "{assembly} triangles, the plate alone {plate}"
    );
    // Only the pin, as the exporter's own options say.
    let options = serde_json::json!({ "solids": ["/pin:extrude(pin)"] });
    let pin = triangles(editor.handle(export("stl", None, options)));
    assert!(pin < assembly, "{pin} of {assembly}");

    let update = editor.handle(export("gltf", None, serde_json::Value::Null));
    let error = update.error.expect("refused");
    assert!(
        error.contains("no exporter `gltf`") && error.contains("stl"),
        "{error}"
    );
    assert!(update.export.is_none());
    let update = editor.handle(export("stl", Some("obj"), serde_json::Value::Null));
    assert!(update.error.expect("refused").contains("writes no `obj`"));
}

/// The structure tree of an assembly, as the front end lists it: the
/// placed parts, and the mates their steps list — named, with the entities
/// they are between, and never drawn.
#[test]
fn an_assembly_lists_its_mates() {
    let mut editor = editor();
    let update = handled(
        &mut editor,
        Command::LoadWorkspaceExample {
            name: "pin_in_plate".into(),
            folder: None,
        },
    );
    let structure = update.scene.expect("a scene").structure;
    let of = |kind: &str| -> Vec<&StructureItem> {
        structure.iter().filter(|i| i.kind == kind).collect()
    };
    let parts: Vec<&str> = of("part").iter().map(|i| i.name.as_str()).collect();
    assert!(parts.contains(&"pin"), "{structure:?}");
    let mates = of("mate");
    let names: Vec<&str> = mates.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(
        names,
        [
            "add_part(plate,fixed)",
            "add_part(pin,m1)",
            "add_part(pin,m2)"
        ],
        "{structure:?}"
    );
    assert!(mates.iter().all(|mate| mate.visible.is_none()));
    // The plate held by its own frame; the pin's faces against the hole's.
    let between: Vec<usize> = mates.iter().map(|mate| mate.elements.len()).collect();
    assert_eq!(between, [1, 2, 2], "{mates:?}");
    assert_eq!(
        mates[2].elements,
        [
            placed_face("pin", "extrude(pin,start)"),
            placed_face("plate", "extrude(hole,end)")
        ]
    );
}

/// Placed parts are moved by a gizmo at their origin: a fixed block turned
/// a quarter about `z` by its ring, and a block on it slid along its `z`
/// axis by its `z` arrow — the slider joint between their frames solved as
/// it is dragged.
#[test]
fn placed_parts_are_moved_and_turned_by_the_gizmo() {
    let block = drilled_box().to_json().unwrap();
    let mut editor = editor();
    let files = [("block.geop".to_string(), Some(block))].into();
    assert!(editor.handle(Command::Files { files }).error.is_none());
    editor.handle(Command::Load {
        program: Program::new(),
        path: Some("guide.geop".into()),
    });
    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice("block.geop".into())));
    handled(&mut editor, Command::Commit);
    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice("block.geop".into())));
    editor.handle(dialog("add_mate", Value::Choice("slider".into())));
    editor.handle(dialog(
        "mate:m1:entities",
        Value::Elements(vec![
            placed_datum("part1", "origin"),
            placed_datum("part2", "origin"),
        ]),
    ));
    handled(&mut editor, Command::Commit);
    // A part nothing solved for yet is at the origin.
    let pose = |editor: &Editor, id: &str| match editor.program().state.get(&pose_state(id)) {
        Some(StateValue::Pose(pose)) => *pose,
        None => Pose::identity(),
        other => panic!("{id}: {other:?}"),
    };
    let apply = |pose: &Pose, p: [f64; 3]| {
        let at = pose.apply(&Vector3::from_array(p.map(Scalar::from_f64)));
        [0, 1, 2].map(|k| at[k].to_f64())
    };
    let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-6);

    // The second block, slid 10 along the first's `z` axis by its `z`
    // arrow, seen from the side.
    let start = apply(&pose(&editor, "part2"), [0.0; 3]);
    let update = editor.handle(Command::Open {
        id: "part2".into(),
        placed: None,
    });
    let step = update.step.expect("the second block is edited");
    let gizmo = step.presentation.gizmo.expect("a placed part has a gizmo");
    assert!(gizmo.modes.translate && gizmo.modes.rotate);
    assert!(
        step.presentation
            .dialog
            .get(geop_core_editor::step_editor::GIZMO_ORIENTATION)
            .is_some()
    );
    let side = |z: f64| pointer([start[0], start[1] - 100.0, z], [0.0, 1.0, 0.0]);
    drag_gizmo(
        &mut editor,
        GizmoPart::Move(Axis::Z),
        side(start[2] + 0.05),
        side(start[2] + 10.055),
        side(start[2] + 5.0),
        false,
    );
    assert!(editor.handle(Command::Commit).error.is_none());
    let slid = apply(&pose(&editor, "part2"), [0.0; 3]);
    assert!(
        close(slid, [start[0], start[1], start[2] + 10.0]),
        "{start:?} slid to {slid:?}"
    );

    // The first block, fixed, turned a quarter about `z` by its ring, seen
    // from above: it goes where it is turned, and the second turns with it.
    editor.handle(Command::Open {
        id: "part1".into(),
        placed: None,
    });
    let reach = 0.009;
    let round = |degrees: f64| {
        let (s, c) = degrees.to_radians().sin_cos();
        let r = 7.0 * reach;
        pointer([r * c, r * s, 100.0], [0.0, 0.0, -1.0])
    };
    drag_gizmo(
        &mut editor,
        GizmoPart::Turn(Axis::Z),
        round(45.0),
        round(137.0),
        round(90.0),
        false,
    );
    assert!(editor.handle(Command::Commit).error.is_none());
    let turned = pose(&editor, "part1");
    assert!(
        close(apply(&turned, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]),
        "{turned:?}"
    );
    assert!(crate::solving::check(&editor.part()).unwrap().converged);
}

/// Moving the pointer over an assembly draws no part: neither the
/// assembly, which was drawn when it was built, nor the parts it places,
/// each drawn once where it is first placed.
#[test]
fn hovering_over_an_assembly_draws_no_part() {
    let mut editor = editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let hover = |editor: &mut Editor| {
        let drawn = geop_core_editor::view::parts_drawn();
        for k in 0..5 {
            let update = editor.handle(Command::Event {
                event: StepEditEvent::Hover {
                    pointer: from_above(0.1 * k as f64, 0.0),
                    shift: false,
                },
            });
            assert!(update.error.is_none(), "{:?}", update.error);
        }
        geop_core_editor::view::parts_drawn() - drawn
    };
    assert_eq!(hover(&mut editor), 0);
    // Nor while a part is placed, which picks in what its step builds.
    let update = editor.handle(Command::Open {
        id: "fore".into(),
        placed: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(hover(&mut editor), 0);
}

/// A file left and opened again takes up its steps as they were built:
/// the drilled box, then an assembly placing it, then the box again,
/// builds the box's steps once, not twice. Edited in between, it is built
/// again from the step that changed.
///
/// Only that each load builds is checked: how many versions a run built
/// was counted by `Part::built`, which geop-core-program no longer has.
#[test]
fn a_file_opened_again_is_not_built_again() {
    let part = drilled_box();
    let mut assembly = Program::new();
    assembly.push(
        "box",
        crate::AddPartArgs {
            file: "part.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    let mut editor = editor();
    let files = [
        ("part.geop".to_string(), Some(part.to_json().unwrap())),
        (
            "assembly.geop".to_string(),
            Some(assembly.to_json().unwrap()),
        ),
    ]
    .into();
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let load = |editor: &mut Editor, program: &Program, path: &str| {
        let update = handled(
            editor,
            Command::Load {
                program: program.clone(),
                path: Some(path.into()),
            },
        );
        let steps = update.program.expect("the program is sent").steps;
        assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");
    };
    load(&mut editor, &part, "part.geop");
    load(&mut editor, &assembly, "assembly.geop");
    load(&mut editor, &part, "part.geop");

    // Its hole drilled deeper: from that step on.
    let mut deeper = part.clone();
    let last = deeper.steps.len() - 1;
    edit_args(&mut deeper.steps[last], |hole: &mut PrimitiveArgs| {
        hole.shape = Shape::Cylinder {
            base: [1.0, 1.0, 0.2],
            axis: FrameAxis::Z,
            radius: 0.4,
            height: 1.3,
        };
    });
    assert_ne!(deeper.steps[last], part.steps[last], "the hole changed");
    load(&mut editor, &deeper, "part.geop");
}

/// The drag tool is no operation: an editor offers it among its tools —
/// where the inspectors are, beside what asks about the part — by its key,
/// label, doc and icon alone, and not among the operations a step can be.
#[test]
fn the_drag_tool_is_offered_among_the_tools_not_the_operations() {
    let mut editor = editor();
    let program = handled(&mut editor, Command::Show)
        .program
        .expect("the program is sent");
    let drag = (program.tools.iter())
        .find(|tool| tool.key == "drag")
        .expect("the drag tool is offered");
    assert_eq!(drag.label, "Drag");
    assert!(drag.icon.is_some());
    assert_eq!(
        serde_json::to_value(drag).unwrap(),
        serde_json::json!({
            "key": "drag",
            "label": "Drag",
            "doc": drag.doc,
            "icon": drag.icon,
        }),
        "a tool says nothing of where a toolbar would put it"
    );
    assert!(
        (program.operations.iter()).all(|op| op.kind != "drag" && op.info.label != "Drag"),
        "the drag tool is not an operation"
    );
    handled(
        &mut editor,
        Command::Tool {
            key: Some("drag".into()),
        },
    );
    let program = handled(&mut editor, Command::Show).program.unwrap();
    assert_eq!(program.tool.as_deref(), Some("drag"));
}

/// Editing a part placed before another: the other leaves the scene while
/// the step is edited, and shows again after, without its component sent
/// again — the viewer keeps it. Moving the part edited sends where it is,
/// and nothing of what it looks like.
#[test]
fn a_part_placed_after_the_one_edited_is_sent_once() {
    let mut editor = editor();
    let files = BTreeMap::from([
        ("cube.geop".to_string(), Some(cube().to_json().unwrap())),
        (
            "block.geop".to_string(),
            Some(drilled_box().to_json().unwrap()),
        ),
    ]);
    handled(&mut editor, Command::Files { files });
    let mut program = Program::new();
    program.push("a", args(true, &[]));
    let block = AddPartArgs {
        file: "block.geop".into(),
        ..Default::default()
    };
    program.push("b", block.fixed());
    let path = Some("assembly.geop".into());
    let loaded = handled(&mut editor, Command::Load { program, path });
    let scene = loaded.scene.expect("the assembly is drawn");
    assert_eq!(
        scene.components.len(),
        2,
        "the cube and the block, once each"
    );

    let open = Command::Open {
        id: "a".into(),
        placed: None,
    };
    let opened = handled(&mut editor, open.clone());
    let scene = opened.scene.expect("the part as far as `a`");
    assert_eq!(scene.removed, ["b"]);
    let cancelled = handled(&mut editor, Command::Cancel);
    let shown = cancelled.scene.expect("the block shows again");
    let names: Vec<&str> = shown.instances.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(names, ["b"]);
    assert!(shown.forgotten.is_empty());
    assert!(shown.components.is_empty(), "the block is not sent again");

    handled(&mut editor, open);
    let moved = handled(&mut editor, dialog("x", Value::Number(3.0)));
    let scene = moved.scene.expect("the cube is moved");
    assert!(
        scene.components.is_empty(),
        "nothing is sent of what it looks like"
    );
    let names: Vec<&str> = scene.instances.iter().map(|i| i.name.as_str()).collect();
    assert!(names.contains(&"a"), "{names:?}");
}

/// The handle of the step `step`'s field `key`, as `step` shows it.
fn handle_of(step: &geop_core_editor::editor::StepState, key: &str) -> [f64; 3] {
    (step.presentation.visuals.iter())
        .find_map(|v| match &v.shape {
            geop_core_editor::editing::Shape::Handle { at, .. } if v.key == key => {
                Some(at.to_f64_array())
            }
            _ => None,
        })
        .unwrap_or_else(|| panic!("no handle for {key}"))
}

/// A step of a part placed in the program is edited where the part is
/// placed: listed under the step placing it, opened there, its handles
/// drawn where the part is, hovered there, and the placed part built anew
/// as it is edited. Committed, the part's file is handed back to keep, and
/// the program placing it is as it was; cancelled, the file is as it was.
#[test]
fn a_step_of_a_placed_part_is_edited_where_it_is_placed() {
    let part = geop_ops_extrude_revolve::examples::box_with_drill_hole();
    let files = BTreeMap::from([("box.geop".to_string(), Some(part.to_json().unwrap()))]);
    // The step on its own, in its own file: where its handle is.
    let mut alone = editor();
    handled(
        &mut alone,
        Command::Files {
            files: files.clone(),
        },
    );
    let path = Some("box.geop".into());
    handled(
        &mut alone,
        Command::Load {
            program: part.clone(),
            path,
        },
    );
    let opened = handled(
        &mut alone,
        Command::Open {
            id: "box".into(),
            placed: None,
        },
    );
    let own = handle_of(&opened.step.expect("the extrusion is edited"), "distance");

    let mut editor = editor();
    handled(&mut editor, Command::Files { files });
    let mut program = Program::new();
    let placing = AddPartArgs {
        file: "box.geop".into(),
        ..Default::default()
    };
    program.push("a", placing.fixed());
    let at = Pose::identity().with_position(Vector3::from_array([10.0, 0.0, 0.0].map(n)));
    program.state.insert(pose_state("a"), StateValue::Pose(at));
    let path = Some("assembly.geop".into());
    let loaded = handled(
        &mut editor,
        Command::Load {
            program: program.clone(),
            path,
        },
    );
    let structure = loaded.scene.expect("the assembly is drawn").structure;
    let item = (structure.iter())
        .find(|item| item.name == "a")
        .expect("the part is listed");
    assert_eq!(
        item.label.as_deref(),
        Some("box: a"),
        "called after its file too"
    );
    let listed = &loaded.program.expect("the program is sent").steps[0];
    let [placed] = &listed.placed[..] else {
        panic!("the step places one part: {:?}", listed.placed);
    };
    assert_eq!(
        (placed.instance.as_str(), placed.file.as_str()),
        ("a", "box.geop")
    );
    let ids: Vec<&str> = placed.steps.iter().map(|s| s.id.as_str()).collect();
    assert_eq!(ids, ["outline", "box", "hole_sketch", "hole"]);

    let open = Command::Open {
        id: "box".into(),
        placed: Some("a".into()),
    };
    let opened = handled(&mut editor, open.clone());
    let step = opened.step.expect("the placed part's extrusion is edited");
    assert_eq!(step.label, "Extrude");
    let handle = handle_of(&step, "distance");
    assert_close(handle, [own[0] + 10.0, own[1], own[2]], 1e-9);
    let editing = opened.program.expect("the program is sent").steps[0].placed[0].steps[1].editing;
    assert!(editing, "the step is shown edited where it is listed");
    // Hovered where the part is placed, the handle is grabbed.
    let over = pointer([handle[0], handle[1], handle[2] + 5.0], [0.0, 0.0, -1.0]);
    let hovered = handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Hover {
                pointer: over,
                shift: false,
            },
        },
    );
    assert!(
        hovered.step.expect("still edited").presentation.grab,
        "the handle is under the pointer"
    );

    let edited = handled(&mut editor, dialog("distance", Value::Number(2.0)));
    let scene = edited.scene.expect("the placed part is built anew");
    assert_eq!(scene.components.len(), 1, "its new look");
    assert!(
        edited.written.is_none(),
        "nothing is kept before the commit"
    );
    let committed = handled(&mut editor, Command::Commit);
    let [written] = &committed.written.clone().unwrap_or_default()[..] else {
        panic!("one file is kept: {:?}", committed.written);
    };
    assert_eq!(written.path, "box.geop");
    let extrusion: geop_ops_extrude_revolve::ExtrudeArgs =
        step_args(written.program.step("box").unwrap());
    assert_eq!(
        extrusion.extent,
        geop_ops_extrude_revolve::Extents::blind(2.0)
    );
    assert_eq!(
        editor.program(),
        &program,
        "the program placing it is as it was"
    );

    handled(&mut editor, open.clone());
    handled(&mut editor, dialog("distance", Value::Number(3.0)));
    handled(&mut editor, Command::Cancel);
    let reopened = handled(&mut editor, open);
    let distance = reopened
        .step
        .expect("edited again")
        .presentation
        .dialog
        .get("distance")
        .cloned();
    assert!(
        matches!(distance, Some(Control::Number(number)) if number.value == 2.0),
        "cancelled, the file is as committed"
    );
}

/// What the scene of `editor` hides, sent afresh.
fn scene_hidden(editor: &mut Editor) -> Vec<String> {
    let update = handled(editor, Command::Show);
    update.scene.expect("a scene is sent on show").hidden
}

/// The placed parts' datums are drawn only to be picked: placing a part,
/// they are not shown — until a mate waits for its entities, which a
/// datum can be, and once it has them, they go again.
#[test]
fn placing_a_part_shows_the_datums_only_while_a_mate_picks() {
    let mut step = placing(args(false, &[]));
    let origins = ["a", "b"].map(|part| Located::in_instance(part, ORIGIN).key());
    let hidden = scene_hidden(&mut step.editor);
    for origin in &origins {
        assert!(hidden.contains(origin), "{origin} shown: {hidden:?}");
    }
    step.dialog("add_mate", Value::Choice("coincident".into()));
    let hidden = scene_hidden(&mut step.editor);
    for origin in &origins {
        assert!(!hidden.contains(origin), "{origin} hidden: {hidden:?}");
    }
    let picked = [origin("b"), origin("a")].map(|e| e.picked()).to_vec();
    step.dialog("mate:m1:entities", Value::Elements(picked));
    let hidden = scene_hidden(&mut step.editor);
    for origin in &origins {
        assert!(hidden.contains(origin), "{origin} shown: {hidden:?}");
    }
}

/// A placed part hidden is not picked, and hides nothing behind it: the
/// cube placed as `b` stood above `a`, and hidden, a mate's pick looking
/// down through it lights `a`'s top, and a click picks what is lit.
#[test]
fn a_hidden_part_is_not_picked_through() {
    let mut step = placing(args(true, &[]));
    step.dialog("z", Value::Number(2.0));
    handled(
        &mut step.editor,
        Command::Visibility {
            name: "b".into(),
            visible: false,
        },
    );
    step.dialog("add_mate", Value::Choice("coincident".into()));
    let at = down_at(0.5, 0.5, 4.0);
    step.send(StepEditEvent::Hover {
        pointer: at,
        shift: false,
    });
    let lit = step.presentation().highlights.clone();
    assert_eq!(lit.len(), 1, "{lit:?}");
    assert_eq!(lit[0].target, Located::in_instance("a", "model"), "{lit:?}");
    step.send(StepEditEvent::Click {
        pointer: at,
        button: Default::default(),
        double: false,
        shift: false,
    });
    let mate = &step.args::<AddPartArgs>().mates["m1"];
    let picked: Vec<PickedElement> = mate.entities.iter().map(Selectable::picked).collect();
    assert_eq!(picked, lit, "a click picks what the hover lit");
}
