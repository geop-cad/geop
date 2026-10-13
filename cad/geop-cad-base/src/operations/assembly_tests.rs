//! Assemblies: programs placing the parts other program files build — built
//! from a workspace of files, and edited as a front end drives the editor.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::{
    editing::{Button, Control, Pointer, Pull, Reach, Shape, StepEditEvent, Value},
    selectable::Selectable,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{DatumComponent, FrameAxis, Pose, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{element::ElementReference, program::ProgramStep};
use geop_ops_add_datum::{AddDatumArgs, Construction, Selected};
use geop_ops_assembly::{
    AddPartArgs, Library, MateEntity, PartPatternArgs, PatternAxis, Resolved, component_of,
    mate_name,
    placed::locate,
    solving::{self, Freedom, Report, Rerun, SettingsState},
};
use geop_ops_datums::{ORIGIN, component_name, datum};
use geop_ops_extrude_revolve::ExtrudeArgs;
use geop_ops_model::{Design, element, read_model, testing::assert_close};
use geop_ops_parameters::{Settings, StateValue, StateValues, pose_state};
use geop_ops_pattern::spacing::Spacing;

use crate::examples::{n, pose};
use crate::{
    Command, Editor, Program, examples,
    operations::{
        self, Runner,
        testing::{dialog, edit_args, example_editor, from_above, handled},
        workspace,
    },
};

/// What a program built, as a run left it, the program with the state it
/// ran with, and the numbers its parameters gave: what the tests ask of it.
struct Built {
    runner: Runner,
    program: Program,
    inputs: BTreeMap<String, Design>,
}

/// `program` built with `library`, or why its first step that fails does.
fn build(program: &Program, library: &Rc<dyn Library>) -> GeopResult<Built> {
    let runner = operations::build(program, "", library.clone())?;
    Ok(Built {
        runner,
        program: program.clone(),
        inputs: program.inputs(&Settings::new()),
    })
}

impl Built {
    fn view(&self) -> geop_ops_parameters::PartReader<'_> {
        self.runner.part.current()
    }

    /// The program, to run again by a solve.
    fn rerun(&mut self) -> Rerun<'_> {
        Rerun {
            runner: &mut self.runner,
            program: &self.program.program,
            inputs: &self.inputs,
        }
    }

    fn check_names(&self) {
        read_model(&self.view()).unwrap().check_names().unwrap();
    }

    fn check_mates(&self) -> GeopResult<Report> {
        solving::check(&self.view())
    }

    /// The mates solved with `pull`: where they hold, a true solution near
    /// there is enclosed — every value moved — at no more than the solve's
    /// cost.
    fn solve_mates(&mut self, pull: Pull<Design>) -> GeopResult<(StateValues, Report)> {
        let state = self.program.state.clone();
        let (moved, report) = solving::solve(&mut self.rerun(), &state, &[pull], None)?;
        if report.converged {
            assert!(report.unproven.is_empty(), "{:?}", report.unproven);
            for name in moved.keys() {
                assert!(
                    report.enclosed.contains_key(name),
                    "{name} moved, not enclosed"
                );
            }
        }
        // Enclosing what a drag solved costs no more than solving it: the
        // rows it ended at, a Newton step, a box and its check.
        let [solve, enclose] = report.evaluations;
        assert!(
            enclose <= solve.max(4),
            "{enclose} evaluations to enclose a solve of {solve}"
        );
        Ok((moved, report))
    }

    fn solve_joints(&mut self, set: &[(&str, f64)]) -> GeopResult<(StateValues, Report)> {
        let set: BTreeMap<String, Design> =
            (set.iter()).map(|(p, v)| (p.to_string(), n(*v))).collect();
        let state = self.program.state.clone();
        solving::set(&mut self.rerun(), &state, &set)
    }

    fn joints(&self) -> GeopResult<Vec<SettingsState>> {
        Ok(solving::settings(&self.view()))
    }

    fn mate_freedom(&mut self) -> GeopResult<Freedom> {
        let state = self.program.state.clone();
        solving::freedom(&mut self.rerun(), &state, &Default::default())
    }

    /// Whether the part placed as `name` is held where it is.
    fn holds(&self, name: &str) -> bool {
        let reader = self.view();
        (reader.targets_of::<Resolved>().into_iter()).any(|target| {
            matches!(
                reader.retrieve_element(&ElementReference::whole(target)),
                Ok(Resolved::Fixed { body }) if body == name
            )
        })
    }

    /// The parts placed in it — not in those — by name, with their files.
    fn instances(&self) -> BTreeMap<String, String> {
        let reader = self.view();
        (reader.instances().into_iter())
            .filter_map(|(name, instance)| {
                Some((name.to_string(), component_of(instance).ok()?.file.clone()))
            })
            .collect()
    }

    /// Its mates, by name.
    fn mates(&self) -> Vec<String> {
        let mut names: Vec<String> = (self.view().targets_of::<Resolved>().into_iter())
            .filter_map(|target| mate_name(target.name()).map(str::to_string))
            .collect();
        names.sort();
        names
    }

    /// The point `of` takes of what `entity` is — see
    /// [`geop_ops_add_datum::Shape`] — where the part has it: of a placed
    /// part, where it is placed.
    fn placed(
        &self,
        entity: &MateEntity,
        of: impl Fn(&geop_ops_add_datum::Shape) -> Option<Vector3<Design>>,
    ) -> [f64; 3] {
        let reader = self.view();
        let p = of(&entity.shape(&reader).unwrap()).expect("the entity has that point");
        let p = match entity.part() {
            "" => p,
            part => locate(&reader, part).unwrap().world.apply(&p),
        };
        [0, 1, 2].map(|k| p[k].to_f64())
    }
}

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

fn point(part: &Built, entity: &MateEntity) -> [f64; 3] {
    part.placed(entity, |shape| shape.point)
}

/// The pin is mated into the plate's hole: on the hole's axis, its bottom
/// on the hole's bottom — the plate is 1 thick and the hole 0.5 deep,
/// around `(1, 1)` — where the program's state puts it.
#[test]
fn the_pin_is_mated_into_the_plate() {
    let workspace = workspace(parts());
    let program = examples::pin_in_plate_assembly();
    let part = build(&program, &workspace.scope("assembly.geop")).unwrap();
    part.check_names();
    assert!(part.check_mates().unwrap().converged);
    let origin = |instance: &str| MateEntity::of(instance, Selected::Datum(datum(ORIGIN)));
    assert_close(point(&part, &origin("plate")), [0.0; 3], 1e-12);
    assert_close(point(&part, &origin("pin")), [1.0, 1.0, 0.5], 1e-7);

    let instances = part.instances();
    assert_eq!(instances.keys().collect::<Vec<_>>(), ["pin", "plate"]);
    assert_eq!(instances["pin"], "pin.geop");
    assert_eq!(
        part.mates(),
        [
            "add_part(pin,m1)",
            "add_part(pin,m2)",
            "add_part(plate,fixed)"
        ]
    );
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
                mates: BTreeMap::new(),
                ..Default::default()
            }
            .fixed(),
        );
        program
    };
    let workspace = workspace(BTreeMap::from([
        ("a.geop".into(), placing("b.geop").to_json().unwrap()),
        ("b.geop".into(), placing("sub/../a.geop").to_json().unwrap()),
        ("c.geop".into(), placing("c.geop").to_json().unwrap()),
    ]));
    let error = |program: Program, file: &str| match build(&program, &workspace.scope(file)) {
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

fn editor_on(program: Program) -> Editor {
    let mut editor = Editor::new();
    let files = parts().into_iter().map(|(p, t)| (p, Some(t))).collect();
    handled(&mut editor, Command::Files { files });
    handled(
        &mut editor,
        Command::Load {
            program,
            path: Some("assembly.geop".into()),
        },
    );
    editor
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
    // The workspace's own, then the standard parts.
    let (own, standard) = files.split_at(3);
    assert_eq!(own, ["", "pin.geop", "plate.geop"]);
    assert!(!standard.is_empty() && standard.iter().all(|f| f.starts_with("std:")));

    // What a viewer keeps: every component it was sent.
    let mut known = BTreeMap::new();
    let mut keep = |update: &crate::Update| {
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
fn top(editor: &mut Editor, instance: &str) -> f64 {
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
    edit_args(&mut longer.program.steps[1], |extrude: &mut ExtrudeArgs| {
        extrude.extent = geop_ops_extrude_revolve::Extents::blind(3.0)
    });
    handled(
        &mut editor,
        Command::Files {
            files: BTreeMap::from([("pin.geop".into(), Some(longer.to_json().unwrap()))]),
        },
    );
    assert!((top(&mut editor, "pin") - 3.5).abs() < 1e-6);
}

/// A placed part is dragged by the editor: grabbed where the pointer hits
/// it, moved in the plane facing the eye — and, with no mates, it goes
/// where it is dragged.
#[test]
fn a_placed_part_is_dragged() {
    let mut program = examples::pin_in_plate_assembly();
    edit_args(&mut program.program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear()
    });
    program
        .state
        .insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    handled(&mut editor, Command::Open { id: "pin".into() });
    handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Drag {
                from: from_above(3.5, 1.0),
                to: from_above(4.5, 1.5),
                done: true,
                shift: false,
            },
        },
    );
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
    edit_args(&mut program.program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear();
        pin.set_fixed(true);
    });
    program
        .state
        .insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    handled(&mut editor, Command::Open { id: "pin".into() });
    for (to, done) in [((4.0, 1.0), false), ((4.5, 1.0), false), ((5.0, 1.0), true)] {
        handled(
            &mut editor,
            Command::Event {
                event: StepEditEvent::Drag {
                    from: from_above(3.5, 1.0),
                    to: from_above(to.0, to.1),
                    done,
                    shift: false,
                },
            },
        );
    }
    editor.handle(Command::Commit);
    let pin = pose_of(editor.program(), "pin");
    assert_close(position(&pin), [5.0, 1.0, 0.0], 1e-9);
}

/// Adding a fixed mate in the step's dialog holds the part the step
/// places, like the checkbox did: it goes where it is dragged, and the
/// program survives being saved and read back.
#[test]
fn a_fixed_mate_added_in_the_dialog_holds_the_part() {
    let mut program = examples::pin_in_plate_assembly();
    edit_args(&mut program.program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear()
    });
    program
        .state
        .insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    handled(&mut editor, Command::Open { id: "pin".into() });
    handled(
        &mut editor,
        dialog("add_mate", Value::Choice("fixed".into())),
    );
    handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Drag {
                from: from_above(3.5, 1.0),
                to: from_above(5.0, 1.0),
                done: true,
                shift: false,
            },
        },
    );
    editor.handle(Command::Commit);
    assert_close(
        position(&pose_of(editor.program(), "pin")),
        [5.0, 1.0, 0.0],
        1e-9,
    );
    let program = editor.program().clone();
    assert_eq!(
        Program::from_json(&program.to_json().unwrap())
            .unwrap()
            .program,
        program.program
    );
    let part = build(&program, &workspace(parts()).scope("assembly.geop")).unwrap();
    assert!(part.holds("pin") && part.holds("plate"));
}

/// Where `pose` puts its body's origin.
fn position(pose: &Pose<Design>) -> [f64; 3] {
    let p = pose.position();
    [0, 1, 2].map(|k| p[k].to_f64())
}

/// Where `program`'s state puts the part placed as `instance`: with no
/// pose there — nothing ever moved it — at the origin.
fn pose_of(program: &Program, instance: &str) -> Pose<Design> {
    match program.state.get(&pose_state(instance)) {
        Some(StateValue::Pose(pose)) => *pose,
        None => Pose::identity(),
        other => panic!("{instance} has no pose: {other:?}"),
    }
}

fn at(position: [f64; 3]) -> StateValue {
    StateValue::Pose(pose(position, [0.0; 3]))
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
        .insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
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
/// is free and the later one fixed: the earlier part ends up where they
/// put it, for every step — a datum made on it in between too. (The plate
/// may turn about the pin's axis; where it turns to is the solve's
/// choice.)
#[test]
fn a_later_mate_moves_an_earlier_part_for_every_step() {
    let mut program = examples::pin_in_plate_assembly();
    edit_args(&mut program.program.steps[0], |plate: &mut AddPartArgs| {
        plate.set_fixed(false)
    });
    edit_args(&mut program.program.steps[1], |pin: &mut AddPartArgs| {
        pin.set_fixed(true)
    });
    let mark = AddDatumArgs {
        selection: vec![Selected::Datum(datum(ORIGIN)).behind("plate")],
        construction: Construction::Point {
            x: 0.0.into(),
            y: 0.0.into(),
            z: 0.0.into(),
        },
    };
    let mark = ProgramStep::of("mark", &mark).unwrap();
    program.program.steps.insert(1, mark);
    program
        .state
        .insert(pose_state("plate"), at([2.0, -1.0, 0.0]));
    let editor = editor_on(program);
    // The pin stays; the plate comes to it.
    assert_close(
        position(&pose_of(editor.program(), "pin")),
        [1.0, 1.0, 0.5],
        1e-12,
    );
    let plate = pose_of(editor.program(), "plate");
    assert!(position(&plate) != [2.0, -1.0, 0.0], "the plate moved");
    let part = build(editor.program(), &workspace(parts()).scope("assembly.geop")).unwrap();
    assert!(part.check_mates().unwrap().converged);
    assert_close(
        point(&part, &MateEntity::on(Selected::Datum(datum("mark")))),
        position(&plate),
        1e-12,
    );
}

/// Editing a step, a mate added moves the part the step places — not the
/// part it is mated to, though that one is not fixed either.
#[test]
fn a_mate_added_moves_the_part_of_its_step() {
    let mut program = examples::pin_in_plate_assembly();
    edit_args(&mut program.program.steps[0], |plate: &mut AddPartArgs| {
        plate.set_fixed(false)
    });
    let mut mates = BTreeMap::new();
    edit_args(&mut program.program.steps[1], |pin: &mut AddPartArgs| {
        mates = std::mem::take(&mut pin.mates)
    });
    program
        .state
        .insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    editor.handle(Command::Open { id: "pin".into() });
    for (id, mate) in &mates {
        editor.handle(dialog(
            "add_mate",
            Value::Choice(mate.kind.label().to_lowercase()),
        ));
        handled(
            &mut editor,
            dialog(
                &format!("mate:{id}:entities"),
                Value::Elements(mate.entities.iter().map(Selectable::picked).collect()),
            ),
        );
    }
    handled(&mut editor, Command::Commit);
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
    let mut editor = example_editor("box_with_drill_hole");
    let update = handled(
        &mut editor,
        Command::LoadWorkspaceExample {
            name: "pin_in_plate".into(),
            folder: Some("examples/pin_in_plate".into()),
        },
    );
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
        [
            "press",
            "pin_in_plate",
            "chain",
            "parametric_plates",
            "four_bar",
            "arm",
            "bolted_plate"
        ]
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
    let mut editor = Editor::new();
    editor.handle(Command::Load {
        program: examples::pin(),
        path: Some("pin.geop".into()),
    });
    let mut assembly = Program::new();
    assembly.push(
        "pin",
        AddPartArgs {
            file: "pin.geop".into(),
            mates: BTreeMap::new(),
            ..Default::default()
        }
        .fixed(),
    );
    let update = handled(
        &mut editor,
        Command::Load {
            program: assembly,
            path: Some("assembly.geop".into()),
        },
    );
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
/// places the hinge, fixed, and holds the pin's top 3 above its own base.
fn hinge() -> (BTreeMap<String, Option<String>>, Program) {
    let mut hinge = examples::pin_in_plate_assembly();
    edit_args(&mut hinge.program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.remove("m2");
    });
    let mut files: BTreeMap<String, Option<String>> =
        parts().into_iter().map(|(p, t)| (p, Some(t))).collect();
    files.insert("hinge.geop".into(), Some(hinge.to_json().unwrap()));
    let mut top = Program::new();
    top.push(
        "hinge",
        AddPartArgs {
            file: "hinge.geop".into(),
            mates: BTreeMap::from([(
                "m1".into(),
                geop_ops_assembly::Mate::constraint(
                    geop_ops_assembly::Kind::Distance { value: n(3.0) },
                    vec![
                        MateEntity::of("hinge/pin", Selected::Face(element("extrude(pin,end)"))),
                        MateEntity::on(Selected::Datum(datum(component_name(
                            ORIGIN,
                            DatumComponent::Plane(FrameAxis::Z),
                        )))),
                    ],
                ),
            )]),
            ..Default::default()
        }
        .fixed(),
    );
    (files, top)
}

fn editor_with(files: BTreeMap<String, Option<String>>, program: Program, path: &str) -> Editor {
    let mut editor = Editor::new();
    editor.handle(Command::Files { files });
    handled(
        &mut editor,
        Command::Load {
            program,
            path: Some(path.into()),
        },
    );
    editor
}

/// The parts of a sub-assembly are the program's to move:
/// its mate slides the hinge's pin up — the pin's pose a parameter of the
/// program, relative to the hinge — and the hinge is drawn with it there.
#[test]
fn a_flexible_sub_assembly_moves_its_parts() {
    let (files, program) = hinge();
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

/// With the drag tool in hand and no step edited, any placed part is
/// dragged where its mates let it: hovered, it is lit and a press grabs
/// it; dragged, the program's state follows; undone, the whole drag
/// goes at once. A fixed part stays put.
#[test]
fn the_drag_tool_drags_any_placed_part() {
    let mut program = examples::pin_in_plate_assembly();
    edit_args(&mut program.program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear()
    });
    program
        .state
        .insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    let update = editor.handle(Command::DragTool { on: true });
    assert!(update.program.unwrap().drag_tool);

    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: from_above(3.5, 1.0),
            shift: false,
        },
    });
    let tool = update.tool.unwrap();
    assert!(tool.grab, "a press grabs the pin");
    assert!(
        matches!(&tool.visuals[0].shape, Shape::Instance { name } if name == "pin")
    );

    for (i, done) in [(1, false), (2, true)] {
        editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: from_above(3.5, 1.0),
                to: from_above(3.5 + 0.5 * f64::from(i), 1.0),
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
            pointer: from_above(0.2, 0.2),
            shift: false,
        },
    });
    assert!(!update.tool.unwrap().grab);
    let before = editor.program().clone();
    editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: from_above(0.2, 0.2),
            to: from_above(1.2, 0.2),
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
            StateValue::Pose(pose) => Some((name.clone(), *pose)),
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
) -> (Program, Vec<(std::time::Duration, usize, Option<Report>)>) {
    let files = BTreeMap::from([(
        "link.geop".to_string(),
        Some(examples::link().to_json().unwrap()),
    )]);
    let mut editor = editor_with(files.clone(), examples::chain_assembly(), "chain.geop");
    let workspace = workspace(
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
        let check = build(editor.program(), &library)
            .unwrap()
            .check_mates()
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
fn solves_along(path: &[[f64; 2]]) -> Vec<Report> {
    let files = BTreeMap::from([("link.geop".to_string(), examples::link().to_json().unwrap())]);
    let workspace = workspace(files);
    let library = workspace.scope("chain.geop");
    let mut program = examples::chain_assembly();
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    path.iter()
        .map(|to| {
            let mut part = build(&program, &library).unwrap();
            let drag = Pull::Point {
                parameter: "link3.pose".into(),
                local: v([2.5, 0.0, 0.2]),
                target: v([to[0], to[1] / 2.0, 0.2 + to[1] / 2.0]),
            };
            let (moved, report) = part.solve_mates(drag).unwrap();
            program.state.extend(moved);
            let check = build(&program, &library).unwrap().check_mates().unwrap();
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
    let hidden = |update: crate::Update| update.scene.unwrap().hidden;
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
    let mut editor = Editor::new();
    let files: BTreeMap<String, Option<String>> = examples::workspaces()
        .into_iter()
        .find(|(name, _)| *name == "pin_in_plate")
        .unwrap()
        .1()
    .into_iter()
    .map(|(path, program)| (path.to_string(), Some(program.to_json().unwrap())))
    .collect();
    let mut known = BTreeMap::new();
    let mut keep = |update: crate::Update| {
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
        .1();
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
        ("arm", vec![("upper", 0.0), ("fore", 0.2), ("hand", 0.4)]),
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
        let part = build(&program, &workspace(files).scope(&format!("{name}.geop"))).unwrap();
        let report = part.check_mates().unwrap();
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
    let workspace = workspace(files);
    let library = workspace.scope("four_bar.geop");
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    for step in 1..=36 {
        let angle = (60.0 + 10.0 * f64::from(step)).to_radians();
        let mut part = build(&program, &library).unwrap();
        let drag = Pull::Point {
            parameter: "crank.pose".into(),
            local: v([1.5, 0.0, 0.1]),
            target: v([1.5 * angle.cos(), 1.5 * angle.sin(), 0.3]),
        };
        let (moved, report) = part.solve_mates(drag).unwrap();
        assert!(report.converged, "step {step}: {report:?}");
        program.state.extend(moved);
        let tip = pose_of(&program, "crank").apply(&v([1.5, 0.0, 0.0]));
        let tip = [0, 1, 2].map(|k| tip[k].to_f64());
        assert_close(tip, [1.5 * angle.cos(), 1.5 * angle.sin(), 0.2], 1e-3);
        let check = build(&program, &library).unwrap().check_mates().unwrap();
        assert!(check.converged, "step {step}: {check:?}");
    }
}

/// The arm's joints, as its program places them: their angles where its
/// state puts them, their limits, and how free each link is — the upper arm
/// fixed, the forearm turning on one joint, the hand on two.
#[test]
fn the_arm_lists_its_joints_and_how_free_its_links_are() {
    let (files, program) = workspace_example("arm");
    let mut part = build(&program, &workspace(files).scope("arm.geop")).unwrap();
    let joints = part.joints().unwrap();
    let values: Vec<_> = joints
        .iter()
        .map(|j| (j.name.as_str(), j.kind, j.values[0].value, j.values[0].max))
        .collect();
    assert_eq!(
        values,
        [
            ("add_part(fore,m1)", "Revolute", 30.0, Some(150.0)),
            ("add_part(hand,m1)", "Revolute", -45.0, Some(120.0)),
        ]
    );
    let freedom = part.mate_freedom().unwrap();
    assert_eq!(
        freedom.parts.into_iter().collect::<Vec<_>>(),
        [
            ("fore".to_string(), 1),
            ("hand".to_string(), 2),
            ("upper".to_string(), 0)
        ]
    );
    assert_eq!(freedom.total, 2);
    assert!(part.check_mates().unwrap().converged);
}

/// Dragged round past its limit, the forearm turns about its joint and
/// stops at the limit: the joint's angle is the limit exactly, and the
/// report says so. The hand, free on its own joint, stops at that one's.
#[test]
fn the_arm_is_dragged_up_to_its_limit() {
    let (files, mut program) = workspace_example("arm");
    let workspace = workspace(files);
    let library = workspace.scope("arm.geop");
    let mut part = build(&program, &library).unwrap();
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    let a = 170.0_f64.to_radians();
    let drag = Pull::Point {
        parameter: "fore.pose".into(),
        local: v([1.5, 0.0, 0.2]),
        target: v([3.0 + 1.5 * a.cos(), 1.5 * a.sin(), 0.4]),
    };
    let (moved, report) = part.solve_mates(drag).unwrap();
    assert!(report.converged, "{report:?}");
    // The hand keeps as much of its turn in the world as its own joint
    // allows: turned the other way by as much as the forearm turned, it
    // would be past its limit too.
    assert_eq!(
        report.at_limit,
        ["add_part(fore,m1).angle", "add_part(hand,m1).angle"],
        "{report:?} {moved:?}"
    );
    assert_eq!(
        moved["add_part(hand,m1).angle"],
        StateValue::Number(n(-120.0))
    );
    assert_eq!(
        moved["add_part(fore,m1).angle"],
        StateValue::Number(n(150.0))
    );
    program.state.extend(moved);
    let fore = pose_of(&program, "fore");
    assert!((fore.euler_degrees()[2] - 150.0).abs() < 1e-9, "{fore:?}");
    assert_close(position(&fore), [3.0, 0.0, 0.2], 1e-9);
    let check = build(&program, &library).unwrap().check_mates().unwrap();
    assert!(check.converged, "{check:?}");
}

/// Six pins round the plate's hole: the pin placed at 1 from the hole's
/// axis, and a pattern of 6, 60° apart, round it — the hole's face picked
/// in the plate placed, round its axis where the plate is. Each copy is a
/// placed part of its own, named by the pattern, its entities behind its
/// name; moved, the pin takes its copies with it, and the plate moved
/// takes the axis.
#[test]
fn six_pins_are_patterned_round_a_hole() {
    let workspace = workspace(parts());
    let library = workspace.scope("assembly.geop");
    let mut program = Program::new();
    program.push(
        "plate",
        AddPartArgs {
            file: "plate.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    program.push(
        "pin",
        AddPartArgs {
            file: "pin.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    program.push(
        "pins",
        PartPatternArgs {
            part: "pin".into(),
            // The plate's hole, round about (1, 1) in it.
            axis: Some(PatternAxis::Round(
                element("extrude(hole,hole_sketch,c1)").behind("plate"),
            )),
            reversed: false,
            layout: geop_ops_assembly::Layout::Circular {
                angle: Spacing::step(60.0),
            },
            count: 6.0.into(),
        },
    );
    program.state = BTreeMap::from([
        (pose_state("plate"), at([0.0; 3])),
        (pose_state("pin"), at([2.0, 1.0, 0.0])),
    ]);
    let check = |program: &Program, centre: f64, radius: f64| {
        let part = build(program, &library).unwrap();
        part.check_names();
        let instances = part.instances();
        assert_eq!(instances.len(), 7);
        for k in 1..6 {
            let copy = format!("part_pattern(pins,{k})");
            assert_eq!(instances[&copy], "pin.geop");
            let origin = MateEntity::of(&copy, Selected::Datum(datum(ORIGIN)));
            let a = (60.0 * k as f64).to_radians();
            assert_close(
                point(&part, &origin),
                [centre + radius * a.cos(), 1.0 + radius * a.sin(), 0.0],
                1e-9,
            );
        }
        // A copy's faces are its own entities, pickable by name.
        let top = MateEntity::of(
            "part_pattern(pins,3)",
            Selected::Face(element("extrude(pin,end)")),
        );
        let plane = part.placed(&top, |shape| shape.plane.as_ref().map(|plane| *plane.origin()));
        assert!((plane[2] - 2.0).abs() < 1e-9);
    };
    check(&program, 1.0, 1.0);
    program
        .state
        .insert(pose_state("pin"), at([3.0, 1.0, 0.0]));
    check(&program, 1.0, 2.0);
    program
        .state
        .insert(pose_state("plate"), at([10.0, 0.0, 0.0]));
    program
        .state
        .insert(pose_state("pin"), at([12.0, 1.0, 0.0]));
    check(&program, 11.0, 1.0);

    // Refused, saying why: a part no step placed, a pattern of one.
    edit_args(
        &mut program.program.steps[2],
        |pattern: &mut PartPatternArgs| pattern.part = "nut".into(),
    );
    let err = build(&program, &library).err().unwrap().to_string();
    assert!(err.contains("no part placed so far"), "{err}");
    edit_args(
        &mut program.program.steps[2],
        |pattern: &mut PartPatternArgs| {
            pattern.part = "pin".into();
            pattern.count = 1.0.into();
        },
    );
    let err = build(&program, &library).err().unwrap().to_string();
    assert!(err.contains("needs 2 or more"), "{err}");
}

/// Two links on revolute joints at the ends of a fixed one, coupled as a
/// pair of gears 2:1 turning opposite ways: setting the first to 90° turns
/// the second to -45°.
#[test]
fn a_gear_coupling_turns_the_driven_link_by_its_ratio() {
    use geop_ops_assembly::{CouplingKind, JointKind, Mate, Transmission};
    let (files, _) = workspace_example("arm");
    let workspace = workspace(files);
    let library = workspace.scope("gears.geop");
    let joint = |on_hole: usize, link: &str| {
        Mate::joint(
            JointKind::Revolute {
                min: None,
                max: None,
            },
            vec![
                examples::link_rim("base", on_hole, "end"),
                examples::link_rim(link, 0, "start"),
            ],
        )
    };
    let mut program = Program::new();
    let placed = |fixed: bool, mates: Vec<(&str, Mate)>| {
        let args = AddPartArgs {
            file: "link.geop".into(),
            mates: mates
                .into_iter()
                .map(|(id, m)| (id.to_string(), m))
                .collect(),
            ..Default::default()
        };
        if fixed { args.fixed() } else { args }
    };
    program.push("base", placed(true, Vec::new()));
    program.push("driver", placed(false, vec![("m1", joint(0, "driver"))]));
    let gear = Mate::coupling(
        CouplingKind {
            transmission: Transmission::Gear,
            value: n(2.0),
            reverse: true,
        },
        vec!["add_part(driver,m1)".into(), "add_part(driven,m1)".into()],
    );
    program.push(
        "driven",
        placed(false, vec![("m1", joint(1, "driven")), ("m2", gear)]),
    );
    program.state = BTreeMap::from([
        (pose_state("base"), at([0.0; 3])),
        (pose_state("driver"), at([0.0, 0.0, 0.2])),
        (pose_state("driven"), at([3.0, 0.0, 0.2])),
    ]);
    // The mate is written flat, by the coupling's own type.
    let json = program.to_json().unwrap();
    let gear = &program.program.steps[2].args["mates"]["m2"];
    assert_eq!(
        (&gear["type"], &gear["value"], &gear["reverse"]),
        (&"gear".into(), &2.0.into(), &true.into()),
        "{json}"
    );
    assert_eq!(Program::from_json(&json).unwrap().program, program.program);

    let driver = "add_part(driver,m1).angle";
    let mut part = build(&program, &library).unwrap();
    let (moved, report) = part.solve_joints(&[(driver, 90.0)]).unwrap();
    assert!(report.converged, "{report:?}");
    assert_eq!(moved[driver], StateValue::Number(n(90.0)));
    let StateValue::Number(driven) = moved["add_part(driven,m1).angle"] else {
        panic!("the driven joint's angle");
    };
    assert!((driven.to_f64() + 45.0).abs() < 1e-9, "{driven:?}");
    program.state.extend(moved);
    let turned = pose_of(&program, "driven").euler_degrees()[2];
    assert!((turned + 45.0).abs() < 1e-9, "{turned}");
    assert!(
        build(&program, &library)
            .unwrap()
            .check_mates()
            .unwrap()
            .converged
    );
}

/// A file built once is placed as it was built for as long as it is the
/// same: its build is kept, by the values its parameters were built with,
/// and placed again rather than built again. Writing it as it was changes
/// nothing; writing it otherwise forgets the build.
#[test]
fn a_kept_build_is_placed_without_building_its_file() {
    let text = examples::box_with_drill_hole().to_json().unwrap();
    let files = BTreeMap::from([("part.geop".to_string(), text.clone())]);
    let workspace = workspace(files);
    let placed = || {
        (workspace.scope("assembly.geop"))
            .component("part.geop", &Settings::new(), &StateValues::new())
            .expect("the part is placed")
    };
    let first = placed();
    assert!(Rc::ptr_eq(&first, &placed()), "kept, not built again");
    assert!(!workspace.write("part.geop", Some(text)));
    assert!(Rc::ptr_eq(&first, &placed()), "written as it was");
    assert!(workspace.write("part.geop", Some("not a program".into())));
    assert!(
        (workspace.scope("assembly.geop"))
            .component("part.geop", &Settings::new(), &StateValues::new())
            .is_err(),
        "written, the file is built again"
    );
}

/// A joint is made from frames, picked on the two parts: the pin's wall
/// and the hole's wall each give one on their axis, half way along it, and
/// a revolute joint between them puts the pin in the hole, free to turn
/// and, as the frames are half way along, no lower than its middle at the
/// hole's — where the editor's dialog lists it with its coordinate.
#[test]
fn a_joint_between_frames_on_two_walls_puts_the_pin_in_the_hole() {
    let mut program = examples::pin_in_plate_assembly();
    let mut mates = BTreeMap::new();
    edit_args(&mut program.program.steps[1], |pin: &mut AddPartArgs| {
        mates = std::mem::take(&mut pin.mates)
    });
    program
        .state
        .insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    editor.handle(Command::Open { id: "pin".into() });
    editor.handle(dialog("add_mate", Value::Choice("revolute".into())));
    let frames = mates["m1"].entities.iter().map(Selectable::picked).collect();
    let update = handled(
        &mut editor,
        dialog("mate:m1:entities", Value::Elements(frames)),
    );
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = editor.handle(dialog("mate:m1", Value::Press)).step.unwrap();
    assert!(
        matches!(
            step.presentation.dialog.get("mate:m1:angle"),
            Some(Control::Number(_))
        ),
        "a revolute joint has its angle to set"
    );
    handled(&mut editor, Command::Commit);
    // On the hole's axis: the pin's frame is 1 up its 2, the hole's 0.75
    // up the plate, so the pin's origin is 0.25 below the plate's.
    assert_close(
        position(&pose_of(editor.program(), "pin")),
        [1.0, 1.0, -0.25],
        1e-7,
    );
}

/// A coupling is made of joints already made, and offers those that do what
/// it needs — a gear two that turn — each said by what it joins; with none
/// that does, it says to add one first.
#[test]
fn a_coupling_offers_the_joints_that_move_as_it_needs() {
    let files = BTreeMap::from([(
        "link.geop".to_string(),
        Some(examples::link().to_json().unwrap()),
    )]);
    let mut editor = Editor::new();
    editor.handle(Command::Files { files });
    handled(
        &mut editor,
        Command::Load {
            program: examples::arm_assembly(),
            path: Some("arm.geop".into()),
        },
    );
    editor.handle(Command::Open { id: "hand".into() });
    let step = editor
        .handle(dialog("add_mate", Value::Choice("gear".into())))
        .step
        .unwrap();
    let dialog_of = &step.presentation.dialog;
    for field in ["mate:m2:joint0", "mate:m2:joint1"] {
        let Some(Control::Select { options, .. }) = dialog_of.get(field) else {
            panic!("{field} is chosen from a list");
        };
        let labels: Vec<&str> = options.iter().map(|o| o.label.as_str()).collect();
        assert_eq!(labels.len(), 3, "{labels:?}");
        assert!(
            labels[1].contains("Revolute") && labels[1].contains('↔'),
            "{labels:?}"
        );
    }
    // A rack and its pinion needs one that slides: there is none.
    let step = editor
        .handle(dialog("add_mate", Value::Choice("rack_pinion".into())))
        .step
        .unwrap();
    let Some(Control::Text { text, .. }) = step.presentation.dialog.get("mate:m3:no_joint1") else {
        panic!("it says there is no joint that slides");
    };
    assert!(text.contains("Slider"), "{text}");
}

/// Clicking the way the front end does: with a joint's frames to pick, a
/// hover over a face lights it and draws the frame a click would put there,
/// as three arrows; a click picks it. A revolute joint between the frames
/// on the pin's top and the plate's top then stands the pin on its head in
/// the middle of the plate, its top at the plate's top.
#[test]
#[ignore = "the step editor lights a hovered frame pick but no longer draws it as a triad: \
            `geop_ops::ui::step` drew one for every `EntityRef::Frame` hovered or picked, and \
            neither `cad/geop-cad-base/src/step_editor.rs` nor `Selectable` has a way to draw \
            what a pick is (the highlight and the joint the clicks make pass)"]
fn a_joint_is_made_by_clicking_where_its_frames_go() {
    let mut program = examples::pin_in_plate_assembly();
    edit_args(&mut program.program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear()
    });
    program
        .state
        .insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    editor.handle(Command::Open { id: "pin".into() });
    editor.handle(dialog("add_mate", Value::Choice("revolute".into())));
    // The frames to pick are waited for: nothing to press first.
    let armed = editor.handle(Command::Event {
        event: StepEditEvent::Leave,
    });
    assert_eq!(
        armed.step.unwrap().presentation.pickable,
        MateEntity::picked_from()
    );
    let event = |event| Command::Event { event };

    // Not through the pin's origin, a frame datum that is picked first.
    let update = editor.handle(event(StepEditEvent::Hover {
        pointer: from_above(3.6, 1.0),
        shift: false,
    }));
    let presentation = update.step.unwrap().presentation;
    // Lit as the frame a click there puts on the pin's top.
    let pin_top = geop_ops_view::FramePick {
        on: MateEntity::of("pin", Selected::Face(element("extrude(pin,end)"))).picked(),
        at: None,
    }
    .picked();
    assert!(
        presentation.highlights.contains(&pin_top),
        "{:?}",
        presentation.highlights
    );
    for (x, y) in [(3.6, 1.0), (0.3, 0.3)] {
        let click = StepEditEvent::Click {
            pointer: from_above(x, y),
            button: Button::Primary,
            double: false,
            shift: false,
        };
        handled(&mut editor, event(click));
    }
    handled(&mut editor, Command::Commit);
    assert_close(
        position(&pose_of(editor.program(), "pin")),
        [1.0, 1.0, -1.0],
        1e-7,
    );
    // The hover drew the frame a click would put, as three arrows.
    let triad = presentation
        .visuals
        .iter()
        .find_map(|v| match &v.shape {
            Shape::Triad { at, axes } => Some((at, axes)),
            _ => None,
        })
        .expect("the frame a click would put is drawn");
    assert_close(
        [0, 1, 2].map(|k| triad.0[k].to_f64()),
        [3.5, 1.0, 2.0],
        1e-9,
    );
    assert!((triad.1[2][2].to_f64().abs() - 1.0).abs() < 1e-9, "z up");
}
