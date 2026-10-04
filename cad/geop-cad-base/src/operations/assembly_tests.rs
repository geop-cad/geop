//! Assemblies: programs placing the parts other program files build — built
//! from a workspace of files, and edited as a front end drives the editor.

use std::collections::BTreeMap;

use geop_core_math::{
    primitives::{Pose, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{
    Design, EntityRef, PartDescription,
    operation::Aspects,
    ui::{Control, Pointer, Reach, StepEditEvent, Value},
};
use geop_ops_assembly::AddPartArgs;

use geop_ops::part::{ParamValue, pose_parameter};
use geop_ops_datums::{AddDatumArgs, Construction};

use crate::examples::{n, pose};
use crate::{Command, Editor, PartOperation, Program, Step, Workspace, examples};

/// The files of the `pin_in_plate` example but its assembly, by path.
fn parts() -> BTreeMap<String, String> {
    BTreeMap::from([
        (
            "plate.geop".into(),
            examples::box_with_drill_hole().to_json().unwrap(),
        ),
        ("pin.geop".into(), examples::pin().to_json().unwrap()),
    ])
}

fn point(part: &geop_ops::Part<S>, entity: &EntityRef) -> [f64; 3] {
    let p = Aspects::of(entity, part).unwrap().point.unwrap();
    [0, 1, 2].map(|k| p[k].to_f64())
}

#[track_caller]
fn assert_close(a: [f64; 3], b: [f64; 3], tol: f64) {
    assert!(
        (0..3).all(|k| (a[k] - b[k]).abs() < tol),
        "{a:?} is not within {tol} of {b:?}"
    );
}

/// The pin is mated into the plate's hole: on the hole's axis, its bottom
/// on the hole's bottom — the plate is 1 thick and the hole 0.5 deep,
/// around `(1, 1)` — where the program's state puts it.
#[test]
fn the_pin_is_mated_into_the_plate() {
    let workspace = Workspace::<S>::new(parts());
    let program = examples::pin_in_plate_assembly();
    let part = program.build(&workspace.scope("assembly.geop")).unwrap();
    part.check_names().unwrap();
    assert!(part.check_mates(|_| true).unwrap().converged);
    let origin = |instance: &str| EntityRef::datum(format!("{instance}/origin"));
    assert_close(point(&part, &origin("plate")), [0.0; 3], 1e-12);
    assert_close(point(&part, &origin("pin")), [1.0, 1.0, 0.5], 1e-7);

    let description = PartDescription::of(&part).unwrap();
    assert_eq!(
        description.instances.keys().collect::<Vec<_>>(),
        ["pin", "plate"]
    );
    assert_eq!(description.instances["pin"].file, "pin.geop");
    assert_eq!(description.mates, ["add_part(pin,m1)", "add_part(pin,m2)"]);
}

/// A program placing a file that places it back — directly or through
/// others — would never finish building: it is refused, naming the cycle.
#[test]
fn files_must_not_place_each_other_in_a_cycle() {
    let placing = |file: &str| {
        let mut program = Program::new();
        program.push(
            "placed",
            AddPartArgs {
                file: file.into(),
                fixed: true,
                flexible: false,
                mates: BTreeMap::new(),
                ..Default::default()
            },
        );
        program
    };
    let workspace = Workspace::<S>::new(BTreeMap::from([
        ("a.geop".into(), placing("b.geop").to_json().unwrap()),
        ("b.geop".into(), placing("sub/../a.geop").to_json().unwrap()),
        ("c.geop".into(), placing("c.geop").to_json().unwrap()),
    ]));
    let error = |program: Program, file: &str| match program.build::<S>(&workspace.scope(file)) {
        Ok(_) => panic!("{file} built"),
        Err(e) => e.to_string(),
    };
    let err = error(placing("a.geop"), "main.geop");
    assert!(err.contains("a.geop -> b.geop -> a.geop"), "{err}");
    // Built before, a file is not built again — and still cannot be
    // placed by a file it places.
    let err = error(placing("b.geop"), "a.geop");
    assert!(err.contains("places it back"), "{err}");
    let err = error(placing("c.geop"), "c.geop");
    assert!(err.contains("c.geop -> c.geop"), "{err}");
}

fn editor_on(program: Program) -> Editor<S> {
    let mut editor = Editor::new();
    let files = parts().into_iter().map(|(p, t)| (p, Some(t))).collect();
    let update = editor.handle(Command::Files { files });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Load {
        program,
        path: Some("assembly.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor
}

fn dialog(key: &str, value: Value) -> Command<S> {
    Command::Event {
        event: StepEditEvent::Dialog {
            key: key.into(),
            value,
        },
    }
}

/// A new part step offers the other files to place; placing one draws its
/// part, every entity named behind the step's id.
#[test]
fn the_editor_places_parts_from_its_files() {
    let mut editor = editor_on(Program::new());
    let update = editor.handle(Command::New {
        kind: "add_part".into(),
    });
    let step = update.step.unwrap();
    let Some(Control::Select { options, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is chosen from a list");
    };
    let files: Vec<&str> = options.iter().map(|o| o.value.as_str()).collect();
    assert_eq!(files, ["", "pin.geop", "plate.geop"]);

    // What a viewer keeps: every component it was sent.
    let mut known = BTreeMap::new();
    let mut keep = |update: &crate::Update<S>| {
        if let Some(scene) = &update.scene {
            known.extend(scene.components.clone());
        }
    };
    keep(&editor.handle(dialog("file", Value::Choice("plate.geop".into()))));
    let update = editor.handle(Command::Commit);
    keep(&update);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(update.program.unwrap().steps[0].id, "part1");
    let scene = update.scene.unwrap();
    // Drawn by reference: the placed part where it is, and its component's
    // view, sent once.
    assert!(scene.part.faces.is_empty());
    let [instance] = &scene.part.instances[..] else {
        panic!("one part is placed: {:?}", scene.part.instances);
    };
    assert_eq!(instance.name, "part1");
    let component = &known[&instance.component];
    assert!(!component.faces.is_empty());
    assert!(component.datums.iter().any(|d| d.name == "origin"));
    assert!(
        component.sketches.is_empty(),
        "what it was drawn with is its own"
    );
    // Sent again only when the viewer starts afresh.
    let shown = editor.handle(Command::Show).scene.unwrap();
    assert!(shown.components.contains_key(&instance.component));
}

/// The highest point the scene draws of the instance `instance`: its
/// component's view, where the instance is.
fn top(editor: &mut Editor<S>, instance: &str) -> f64 {
    let scene = editor.handle(Command::Show).scene.unwrap();
    let placed = scene
        .part
        .instances
        .iter()
        .find(|i| i.name == instance)
        .expect("the instance is drawn");
    scene.components[&placed.component]
        .faces
        .iter()
        .flat_map(|f| f.triangles.iter().flatten())
        .map(|p| placed.frame.to_xyz(p)[2].to_f64())
        .fold(f64::NEG_INFINITY, f64::max)
}

/// When a file a program places changes, what it places is built anew.
#[test]
fn a_changed_file_is_placed_anew() {
    let mut editor = editor_on(examples::pin_in_plate_assembly());
    assert!((top(&mut editor, "pin") - 2.5).abs() < 1e-6);
    let mut longer = examples::pin();
    let PartOperation::Extrude(extrude) = &mut longer.steps[1].operation else {
        panic!("the pin is extruded");
    };
    extrude.extent = geop_ops_extrude_revolve::Extents::blind(3.0);
    let update = editor.handle(Command::Files {
        files: BTreeMap::from([("pin.geop".into(), Some(longer.to_json().unwrap()))]),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    assert!((top(&mut editor, "pin") - 3.5).abs() < 1e-6);
}

/// Looking straight down at `(x, y)`.
fn down(x: f64, y: f64) -> Pointer<S> {
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    Pointer {
        ray: Ray::try_new(v([x, y, 10.0]), v([0.0, 0.0, -1.0])).unwrap(),
        reach: Reach::Tube {
            radius: S::from_f64(0.009),
        },
    }
}

/// A placed part is dragged by the editor: grabbed where the pointer hits
/// it, moved in the plane facing the eye — and, with no mates, it goes
/// where it is dragged.
#[test]
fn a_placed_part_is_dragged() {
    let mut program = examples::pin_in_plate_assembly();
    let PartOperation::AddPart(pin) = &mut program.steps[1].operation else {
        panic!("the pin is placed");
    };
    pin.mates.clear();
    program
        .state
        .insert(pose_parameter("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    let update = editor.handle(Command::Open { id: "pin".into() });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: down(3.5, 1.0),
            to: down(4.5, 1.5),
            done: true,
            shift: false,
        },
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(Command::Commit);
    // As closely as a drag follows the pointer: the turn resistance lets the
    // pin lag by at most a ten-thousandth of the way.
    let pin = pose_of(editor.program(), "pin");
    assert_close(position(&pin), [4.5, 1.5, 0.0], 1e-4);
}

/// A fixed part goes where it is dragged, every event of the drag measured
/// from where it was grabbed — not added up, one on top of the other.
#[test]
fn a_fixed_part_follows_a_drag_of_several_events() {
    let mut program = examples::pin_in_plate_assembly();
    let PartOperation::AddPart(pin) = &mut program.steps[1].operation else {
        panic!("the pin is placed");
    };
    pin.mates.clear();
    pin.fixed = true;
    program
        .state
        .insert(pose_parameter("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    let update = editor.handle(Command::Open { id: "pin".into() });
    assert!(update.error.is_none(), "{:?}", update.error);
    for (to, done) in [((4.0, 1.0), false), ((4.5, 1.0), false), ((5.0, 1.0), true)] {
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: down(3.5, 1.0),
                to: down(to.0, to.1),
                done,
                shift: false,
            },
        });
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    editor.handle(Command::Commit);
    let pin = pose_of(editor.program(), "pin");
    assert_close(position(&pin), [5.0, 1.0, 0.0], 1e-9);
}

/// Where `pose` puts its body's origin.
fn position(pose: &Pose<Design>) -> [f64; 3] {
    let p = pose.position();
    [0, 1, 2].map(|k| p[k].to_f64())
}

/// Where `program`'s state puts the part placed as `instance`.
fn pose_of(program: &Program, instance: &str) -> Pose<Design> {
    match program.state.get(&pose_parameter(instance)) {
        Some(ParamValue::Pose(pose)) => *pose,
        other => panic!("{instance} has no pose: {other:?}"),
    }
}

fn at(position: [f64; 3]) -> ParamValue {
    ParamValue::Pose(pose(position, [0.0; 3]))
}

/// A program whose parts are not where its mates hold them — its
/// state stale, say, after a file it places changed — is solved as it
/// is loaded, from where they are: they move there, and the program keeps
/// where.
#[test]
fn a_program_is_solved_where_its_mates_do_not_hold() {
    let mut program = examples::pin_in_plate_assembly();
    program
        .state
        .insert(pose_parameter("pin"), at([3.5, 1.0, 0.0]));
    let editor = editor_on(program);
    assert_close(
        position(&pose_of(editor.program(), "pin")),
        [1.0, 1.0, 0.5],
        1e-7,
    );
    assert_close(
        position(&pose_of(editor.program(), "plate")),
        [0.0; 3],
        1e-12,
    );
}

/// A later step's mates move a part placed earlier — here the earlier part
/// is free and the later one fixed — and every step sees the earlier part
/// where they put it: a reference point built on it in a step between the
/// two is where the part ends up, not where it was first put. (The plate
/// may turn about the pin's axis; where it turns to is the solve's
/// choice.)
#[test]
fn a_later_mate_moves_an_earlier_part_for_every_step() {
    let mut program = examples::pin_in_plate_assembly();
    let PartOperation::AddPart(plate) = &mut program.steps[0].operation else {
        panic!("the plate is placed");
    };
    plate.fixed = false;
    let PartOperation::AddPart(pin) = &mut program.steps[1].operation else {
        panic!("the pin is placed");
    };
    pin.fixed = true;
    program.steps.insert(
        1,
        Step {
            id: "mark".into(),
            operation: AddDatumArgs {
                selection: vec![EntityRef::datum("plate/origin")],
                construction: Construction::Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            }
            .into(),
        },
    );
    program
        .state
        .insert(pose_parameter("plate"), at([2.0, -1.0, 0.0]));
    let editor = editor_on(program);
    // The pin stays; the plate comes to it.
    assert_close(
        position(&pose_of(editor.program(), "pin")),
        [1.0, 1.0, 0.5],
        1e-12,
    );
    let plate = pose_of(editor.program(), "plate");
    assert!(position(&plate) != [2.0, -1.0, 0.0], "the plate moved");
    let workspace = Workspace::<S>::new(parts());
    let part = editor
        .program()
        .build(&workspace.scope("assembly.geop"))
        .unwrap();
    assert!(part.check_mates(|_| true).unwrap().converged);
    assert_close(
        point(&part, &EntityRef::datum("mark")),
        position(&plate),
        1e-12,
    );
}

/// Editing a step, a mate added moves the part the step places — not the
/// part it is mated to, though that one is not fixed either.
#[test]
fn a_mate_added_moves_the_part_of_its_step() {
    let mut program = examples::pin_in_plate_assembly();
    let PartOperation::AddPart(plate) = &mut program.steps[0].operation else {
        panic!("the plate is placed");
    };
    plate.fixed = false;
    let PartOperation::AddPart(pin) = &mut program.steps[1].operation else {
        panic!("the pin is placed");
    };
    let mates = std::mem::take(&mut pin.mates);
    program
        .state
        .insert(pose_parameter("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    editor.handle(Command::Open { id: "pin".into() });
    for (id, mate) in &mates {
        editor.handle(dialog(
            "add_mate",
            Value::Choice(format!("{:?}", mate.kind).to_lowercase()),
        ));
        let update = editor.handle(dialog(
            &format!("mate:{id}:entities"),
            Value::Entities(mate.entities.clone()),
        ));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_close(
        position(&pose_of(editor.program(), "plate")),
        [0.0; 3],
        1e-12,
    );
    assert_close(
        position(&pose_of(editor.program(), "pin")),
        [1.0, 1.0, 0.5],
        1e-7,
    );
}

/// An example of several files adds its files: the update lists them, the
/// first one edited — and the history of the file edited before is not this
/// one's to undo.
#[test]
fn an_example_of_several_files_adds_its_files() {
    let mut editor = Editor::<S>::new();
    editor.handle(Command::LoadExample {
        name: "box_with_drill_hole".into(),
    });
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "pin_in_plate".into(),
        folder: Some("examples/pin_in_plate".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let files = update.files.unwrap();
    let paths: Vec<&str> = files.iter().map(|f| f.path.as_str()).collect();
    assert_eq!(
        paths,
        [
            "examples/pin_in_plate/assembly.geop",
            "examples/pin_in_plate/plate.geop",
            "examples/pin_in_plate/pin.geop"
        ]
    );
    let program = update.program.unwrap();
    assert_eq!(
        program.path.as_deref(),
        Some("examples/pin_in_plate/assembly.geop")
    );
    assert!(!program.can_undo);
    assert_eq!(
        program.workspace_examples,
        ["pin_in_plate", "chain", "parametric_plates", "four_bar"]
    );
    let part = update.scene.unwrap().part;
    let instances: Vec<&str> = part.instances.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(instances, ["plate", "pin"]);
    assert!(editor.handle(Command::Show).files.is_none());
}

/// Switching to another file, the editor keeps the program it leaves as
/// that file's: the file switched to places it as it was edited.
#[test]
fn a_file_left_is_placed_as_it_was_edited() {
    let mut editor = Editor::<S>::new();
    editor.handle(Command::Load {
        program: examples::pin(),
        path: Some("pin.geop".into()),
    });
    let mut assembly = Program::new();
    assembly.push(
        "pin",
        AddPartArgs {
            file: "pin.geop".into(),
            fixed: true,
            flexible: false,
            mates: BTreeMap::new(),
            ..Default::default()
        },
    );
    let update = editor.handle(Command::Load {
        program: assembly,
        path: Some("assembly.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = update.program.unwrap();
    assert!(
        program.steps[0].error.is_none(),
        "{:?}",
        program.steps[0].error
    );
    assert!((top(&mut editor, "pin") - 2.0).abs() < 1e-6);
}

/// How long the editor takes to answer one event of a drag of a mated part
/// — the pin, turned about the hole's axis — including the update sent
/// back as JSON. A drag sends one per frame, so this has to stay well
/// below a second.
#[test]
fn a_drag_is_answered_quickly() {
    let mut editor = editor_on(examples::pin_in_plate_assembly());
    editor.handle(Command::Open { id: "pin".into() });
    let start = std::time::Instant::now();
    let events = 10;
    for i in 0..events {
        let update = editor.handle_json(
            &serde_json::to_string(&serde_json::json!({
                "command": "event",
                "event": {
                    "type": "drag",
                    "from": {"ray": {"origin": [1.3, 1.0, 10.0], "dir": [0.0, 0.0, -1.0]},
                             "reach": {"type": "tube", "radius": 0.009}},
                    "to": {"ray": {"origin": [1.3, 1.0 + 0.02 * i as f64, 10.0], "dir": [0.0, 0.0, -1.0]},
                           "reach": {"type": "tube", "radius": 0.009}},
                    "done": i + 1 == events,
                },
            }))
            .unwrap(),
        );
        assert!(update.is_ok());
    }
    let per_event = start.elapsed() / events;
    assert!(
        per_event < std::time::Duration::from_millis(50),
        "a drag event took {per_event:?}"
    );
}

/// `hinge.geop`: the plate, fixed, and the pin on the axis of its hole —
/// free to slide along it and turn about it — and `top.geop`, which
/// places the hinge, fixed, flexibly or not, and holds the pin's top 3
/// above its own base.
fn hinge(flexible: bool) -> (BTreeMap<String, Option<String>>, Program) {
    let mut hinge = examples::pin_in_plate_assembly();
    let PartOperation::AddPart(pin) = &mut hinge.steps[1].operation else {
        panic!("the pin is placed");
    };
    pin.mates.remove("m2");
    let mut files: BTreeMap<String, Option<String>> =
        parts().into_iter().map(|(p, t)| (p, Some(t))).collect();
    files.insert("hinge.geop".into(), Some(hinge.to_json().unwrap()));
    let mut top = Program::new();
    top.push(
        "hinge",
        AddPartArgs {
            file: "hinge.geop".into(),
            fixed: true,
            flexible,
            mates: BTreeMap::from([(
                "m1".into(),
                geop_ops::assembly::Mate {
                    kind: geop_ops::assembly::MateKind::Distance { value: n(3.0) },
                    entities: vec![
                        EntityRef::Face {
                            name: "hinge/pin/extrude(pin,end)".into(),
                        },
                        EntityRef::datum_component(
                            geop_ops::ORIGIN,
                            geop_core_math::primitives::DatumComponent::Plane(
                                geop_core_math::primitives::FrameAxis::Z,
                            ),
                        ),
                    ],
                },
            )]),
            ..Default::default()
        },
    );
    (files, top)
}

fn editor_with(files: BTreeMap<String, Option<String>>, program: Program, path: &str) -> Editor<S> {
    let mut editor = Editor::new();
    editor.handle(Command::Files { files });
    let update = editor.handle(Command::Load {
        program,
        path: Some(path.into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor
}

/// Placed flexibly, the parts of a sub-assembly are the program's to move:
/// its mate slides the hinge's pin up — the pin's pose a parameter of the
/// program, relative to the hinge — and the hinge is drawn with it there.
#[test]
fn a_flexible_sub_assembly_moves_its_parts() {
    let (files, program) = hinge(true);
    let mut editor = editor_with(files, program, "top.geop");
    let pin = pose_of(editor.program(), "hinge/pin");
    assert_close(position(&pin), [1.0, 1.0, 1.0], 1e-7);
    assert_close(
        position(&pose_of(editor.program(), "hinge")),
        [0.0; 3],
        1e-12,
    );
    let scene = editor.handle(Command::Show).scene.unwrap();
    let names: Vec<&str> = scene
        .part
        .instances
        .iter()
        .map(|i| i.name.as_str())
        .collect();
    assert_eq!(names, ["hinge", "hinge/plate", "hinge/pin"]);
    let drawn = &scene.part.instances[2].frame;
    assert!((drawn.origin()[2].to_f64() - 1.0).abs() < 1e-7);
    assert!((top(&mut editor, "hinge/pin") - 3.0).abs() < 1e-6);
}

/// Placed rigid, a sub-assembly moves as one: its pin stays where the hinge
/// puts it, and the mate cannot hold.
#[test]
fn a_rigid_sub_assembly_moves_as_one() {
    let (files, program) = hinge(false);
    let editor = editor_with(files.clone(), program, "top.geop");
    assert!(!editor.program().state.contains_key("hinge/pin.pose"));
    let workspace = Workspace::<S>::new(
        files
            .into_iter()
            .filter_map(|(path, text)| Some((path, text?)))
            .collect(),
    );
    let part = editor
        .program()
        .build(&workspace.scope("top.geop"))
        .unwrap();
    assert!(!part.check_mates(|_| true).unwrap().converged);
}

/// With the drag tool in hand and no step edited, any placed part is
/// dragged where its mates let it: hovered, it is lit and a press grabs
/// it; dragged, the program's state follows; undone, the whole drag
/// goes at once. A fixed part stays put.
#[test]
fn the_drag_tool_drags_any_placed_part() {
    let mut program = examples::pin_in_plate_assembly();
    let PartOperation::AddPart(pin) = &mut program.steps[1].operation else {
        panic!("the pin is placed");
    };
    pin.mates.clear();
    program
        .state
        .insert(pose_parameter("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    let update = editor.handle(Command::DragTool { on: true });
    assert!(update.program.unwrap().drag_tool);

    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: down(3.5, 1.0),
            shift: false,
        },
    });
    let tool = update.tool.unwrap();
    assert!(tool.grab, "a press grabs the pin");
    assert!(
        matches!(&tool.visuals[0].shape, geop_ops::ui::Shape::Instance { name } if name == "pin")
    );

    for (i, done) in [(1, false), (2, true)] {
        editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: down(3.5, 1.0),
                to: down(3.5 + 0.5 * f64::from(i), 1.0),
                done,
                shift: false,
            },
        });
    }
    assert_close(
        position(&pose_of(editor.program(), "pin")),
        [4.5, 1.0, 0.0],
        1e-4,
    );
    editor.handle(Command::Undo);
    assert_close(
        position(&pose_of(editor.program(), "pin")),
        [3.5, 1.0, 0.0],
        1e-12,
    );

    // The plate is fixed: it is not offered, and a drag of it changes
    // nothing.
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: down(0.2, 0.2),
            shift: false,
        },
    });
    assert!(!update.tool.unwrap().grab);
    let before = editor.program().clone();
    editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: down(0.2, 0.2),
            to: down(1.2, 0.2),
            done: true,
            shift: false,
        },
    });
    assert_eq!(editor.program(), &before);
}

/// Where `program`'s state puts every placed part, by parameter.
fn poses(program: &Program) -> Vec<(String, Pose<Design>)> {
    program
        .state
        .iter()
        .filter_map(|(name, value)| match value {
            ParamValue::Pose(pose) => Some((name.clone(), *pose)),
            _ => None,
        })
        .collect()
}

/// The longest an event of a drag may take to answer, in a release build:
/// a few frames — natively, so a browser, several times slower, still keeps
/// up. A debug build is not timed.
const MAX_EVENT_MILLIS: u64 = if cfg!(debug_assertions) { u64::MAX } else { 50 };

/// Drags the last link of [`examples::chain_assembly`] — grabbed at `start`
/// — through `path`, checking after every event that no link jumps or
/// drifts off: each may slide along its pin and turn about it, which
/// nothing holds, but only as the pins need.
fn drive_chain(
    start: [f64; 2],
    path: &[[f64; 2]],
) -> (
    Program,
    Vec<(
        std::time::Duration,
        usize,
        Option<geop_ops::assembly::MateReport>,
    )>,
) {
    let files = BTreeMap::from([(
        "link.geop".to_string(),
        Some(examples::link().to_json().unwrap()),
    )]);
    let mut editor = editor_with(files.clone(), examples::chain_assembly(), "chain.geop");
    let workspace = Workspace::<S>::new(
        files
            .into_iter()
            .map(|(path, text)| (path, text.unwrap()))
            .collect::<BTreeMap<_, _>>(),
    );
    let library = workspace.scope("chain.geop");
    editor.handle(Command::DragTool { on: true });
    // Seen from in front and above — so the plane a drag moves in, facing
    // the eye, is tilted, and the pointer pulls along the pins too.
    let eye = |x: f64, y: f64| {
        let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
        Pointer {
            ray: Ray::try_new(v([x, y - 8.0, 8.2]), v([0.0, 8.0, -8.0])).unwrap(),
            reach: Reach::Tube {
                radius: S::from_f64(0.009),
            },
        }
    };
    let mut previous = poses(editor.program());
    let mut times = Vec::new();
    for (i, to) in path.iter().enumerate() {
        let started = std::time::Instant::now();
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: eye(start[0], start[1]),
                to: eye(to[0], to[1]),
                done: i + 1 == path.len(),
                shift: false,
            },
        });
        times.push((started.elapsed(), i, editor.dragged().cloned()));
        assert!(update.error.is_none(), "{:?}", update.error);
        // Every drag leaves the mates holding — or the editor would solve
        // them all over again before it answers.
        let check = editor
            .program()
            .build(&library)
            .unwrap()
            .check_mates(|_| true)
            .unwrap();
        assert!(check.converged, "event {i} (to {to:?}): {check:?}");
        let now = poses(editor.program());
        for ((name, p), (_, q)) in now.iter().zip(&previous) {
            let (pp, qp) = (position(p), position(q));
            let jump = (0..3).map(|k| (pp[k] - qp[k]).powi(2)).sum::<f64>().sqrt();
            assert!(
                jump < 1.0,
                "event {i} (to {to:?}): {name} jumped {jump} from {qp:?} {:?}° to {pp:?} {:?}°",
                q.euler_degrees(),
                p.euler_degrees()
            );
            assert!(
                pp.iter().all(|c| c.abs() < 20.0),
                "event {i} (to {to:?}): {name} at {p:?}"
            );
        }
        previous = now;
    }
    // Every event is answered within a frame or two — not only on average:
    // one slow event is a visible stall.
    (editor.program().clone(), times)
}

/// [`drive_chain`], every event answered quickly — not only on average: one
/// slow event is a visible stall.
fn drag_chain(start: [f64; 2], path: &[[f64; 2]]) {
    let (_, mut times) = drive_chain(start, path);
    times.sort_by_key(|(time, ..)| *time);
    let (time, event, report) = &times[times.len() - 1];
    assert!(
        *time < std::time::Duration::from_millis(MAX_EVENT_MILLIS),
        "event {event} (to {:?}) took {time:?}, its solve {report:?}; half took at most {:?}, \
         nine in ten {:?}",
        path[*event],
        times[times.len() / 2].0,
        times[times.len() * 9 / 10].0,
    );
}

/// Points `steps` of the way around the circle about `center` of radius
/// `radius`, from `from` radians on, turning `turns` times.
fn circle(center: [f64; 2], radius: f64, from: f64, turns: f64, steps: usize) -> Vec<[f64; 2]> {
    (1..=steps)
        .map(|i| {
            let angle = from + turns * std::f64::consts::TAU * i as f64 / steps as f64;
            [
                center[0] + radius * angle.cos(),
                center[1] + radius * angle.sin(),
            ]
        })
        .collect()
}

/// Dragging the last link of a chain around in circles, grabbed near its
/// far hole, keeps every link near.
#[test]
fn dragging_a_chain_keeps_every_link_near() {
    drag_chain([8.5, 0.0], &circle([6.5, 0.0], 2.0, 0.0, 1.9, 60));
}

/// The last link folded back over the one before it — turned half a turn
/// about their pin — and then dragged around in circles: still, no link
/// drifts off along its pin.
#[test]
fn dragging_a_folded_chain_keeps_every_link_near() {
    drag_chain([8.5, 0.0], &folded_path());
}

/// The path of [`dragging_a_folded_chain_keeps_every_link_near`]: a half
/// circle that folds the last link back over the one before, then circles
/// near the fold.
fn folded_path() -> Vec<[f64; 2]> {
    let mut path = circle([6.0, 0.0], 2.5, 0.0, 0.5, 30);
    path.extend(circle([4.5, 0.0], 1.0, std::f64::consts::PI, 3.0, 120));
    path
}

/// How the solve of every event of a drag of the chain along `path` goes,
/// from where the editor left the chain after the events before it: its
/// drag solved directly — grabbed where the pointer first hit the link,
/// pulled to where the event's pointer meets the plane facing the eye (see
/// `drive_chain`) — and the mates checked where it put the links, as the
/// editor does after a drag (they must hold there, or it solves them all
/// over again).
fn solves_along(path: &[[f64; 2]]) -> Vec<geop_ops::assembly::MateReport> {
    let files = BTreeMap::from([("link.geop".to_string(), examples::link().to_json().unwrap())]);
    let workspace = Workspace::<S>::new(files);
    let library = workspace.scope("chain.geop");
    let mut program = examples::chain_assembly();
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    path.iter()
        .map(|to| {
            let part = program.build(&library).unwrap();
            let drag = geop_ops::assembly::Drag {
                parameter: "link3.pose".into(),
                local: v([2.5, 0.0, 0.2]),
                target: v([to[0], to[1] / 2.0, 0.2 + to[1] / 2.0]),
            };
            let (moved, report) = part.solve_mates(None, &[drag]).unwrap();
            program.state.extend(moved);
            let check = program
                .build(&library)
                .unwrap()
                .check_mates(|_| true)
                .unwrap();
            assert!(
                check.converged,
                "after {to:?}: {check:?}, solved: {report:?}"
            );
            report
        })
        .collect()
}

/// Folded, the links are in line — a singular configuration, where the
/// mates' Jacobian loses rank and Newton slows — and the drags into, out of
/// and around it still take few steps, in one phase, the mates holding where
/// it ends (see [`solves_along`]): dragging through the fold stays smooth.
#[test]
fn drags_around_the_fold_take_few_steps() {
    let path = folded_path();
    let reports = solves_along(&path);
    let (event, worst) = reports
        .iter()
        .enumerate()
        .max_by_key(|(_, r)| r.iterations)
        .unwrap();
    assert!(
        worst.iterations <= 30 && reports.iter().all(|r| r.phases.len() == 1),
        "event {event} (to {:?}): {worst:?}; {} of {} events took more than 20 steps",
        path[event],
        reports.iter().filter(|r| r.iterations > 20).count(),
        reports.len()
    );
}

/// A placed part's datums are its own: drawn only while a part is being
/// placed, for its mates to pick — hidden once it is.
#[test]
fn placed_datums_show_only_while_placing() {
    let mut editor = editor_on(examples::pin_in_plate_assembly());
    let hidden = |update: crate::Update<S>| update.scene.unwrap().hidden;
    let shown = hidden(editor.handle(Command::Show));
    assert!(shown.contains(&"pin/origin".to_string()), "{shown:?}");
    assert!(shown.contains(&"plate/origin".to_string()), "{shown:?}");
    let placing = hidden(editor.handle(Command::Open { id: "pin".into() }));
    assert!(
        !placing.iter().any(|name| name.contains('/')),
        "{placing:?}"
    );
}

/// Opened first thing — every file, then the assembly, as the browser does
/// — every placed part comes with the mesh to draw it by.
#[test]
fn an_assembly_opened_first_sends_its_parts() {
    let mut editor = Editor::<S>::new();
    let files: BTreeMap<String, Option<String>> = examples::workspaces()
        .into_iter()
        .find(|(name, _)| *name == "pin_in_plate")
        .unwrap()
        .1
        .into_iter()
        .map(|(path, program)| (path.to_string(), Some(program.to_json().unwrap())))
        .collect();
    let mut known = BTreeMap::new();
    let mut keep = |update: crate::Update<S>| {
        assert!(update.error.is_none(), "{:?}", update.error);
        if let Some(scene) = update.scene {
            known.extend(scene.components);
            Some(scene.part)
        } else {
            None
        }
    };
    keep(editor.handle(Command::Files { files }));
    let part = keep(editor.handle(Command::Load {
        program: examples::pin_in_plate_assembly(),
        path: Some("assembly.geop".into()),
    }))
    .expect("the assembly is drawn");
    assert_eq!(part.instances.len(), 2);
    for instance in &part.instances {
        assert!(
            known.contains_key(&instance.component),
            "{instance:?} not sent"
        );
    }
}

/// The workspace example `name`, as files and the program of its first.
fn workspace_example(name: &str) -> (BTreeMap<String, String>, Program) {
    let files = examples::workspaces()
        .into_iter()
        .find(|(n, _)| *n == name)
        .unwrap()
        .1;
    let program = files[0].1.clone();
    let files = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    (files, program)
}

/// The linkages' bars lie stacked, each on the one it is pinned to — so no
/// two cut into each other — and their mates hold where the examples put
/// them.
#[test]
fn linkage_examples_are_stacked_and_hold() {
    for (name, layers) in [
        (
            "chain",
            vec![("link1", 0.0), ("link2", 0.2), ("link3", 0.4)],
        ),
        (
            "four_bar",
            vec![
                ("ground", 0.0),
                ("crank", 0.2),
                ("rocker", 0.2),
                ("coupler", 0.4),
            ],
        ),
    ] {
        let (files, program) = workspace_example(name);
        let part = program
            .build(&Workspace::<S>::new(files).scope(&format!("{name}.geop")))
            .unwrap();
        let report = part.check_mates(|_| true).unwrap();
        assert!(report.converged, "{name}: {report:?}");
        for (instance, z) in layers {
            let at = position(&pose_of(&program, instance));
            assert!((at[2] - z).abs() < 1e-9, "{name}: {instance} at {at:?}");
        }
    }
}

/// The four-bar's crank turns all the way round — the coupler and the
/// rocker following, the mates holding at every step.
#[test]
fn the_four_bar_crank_turns_all_the_way_round() {
    let (files, mut program) = workspace_example("four_bar");
    let workspace = Workspace::<S>::new(files);
    let library = workspace.scope("four_bar.geop");
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    for step in 1..=36 {
        let angle = (60.0 + 10.0 * f64::from(step)).to_radians();
        let part = program.build(&library).unwrap();
        let drag = geop_ops::assembly::Drag {
            parameter: "crank.pose".into(),
            local: v([1.5, 0.0, 0.1]),
            target: v([1.5 * angle.cos(), 1.5 * angle.sin(), 0.3]),
        };
        let (moved, report) = part.solve_mates(None, &[drag]).unwrap();
        assert!(report.converged, "step {step}: {report:?}");
        program.state.extend(moved);
        let tip = pose_of(&program, "crank").apply(&v([1.5, 0.0, 0.0]));
        let tip = [0, 1, 2].map(|k| tip[k].to_f64());
        assert_close(tip, [1.5 * angle.cos(), 1.5 * angle.sin(), 0.2], 1e-3);
        let check = program
            .build(&library)
            .unwrap()
            .check_mates(|_| true)
            .unwrap();
        assert!(check.converged, "step {step}: {check:?}");
    }
}
