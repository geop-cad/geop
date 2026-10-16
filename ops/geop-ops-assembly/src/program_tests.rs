//! Assemblies: programs placing the parts other program files build — built
//! from a workspace of files, and edited as a front end drives the editor.

use geop_core_editor::{
    editor::{Command, Editor, Rerun, Update},
    registry::Registry,
    testing::{dialog, from_above, handled},
};
use geop_core_program::{
    element::ElementReference,
    numeric::{ParameterValue, read_parameter_values},
    part::Part,
    program::{Program, ProgramStep},
    reader::Reader,
    runner::Runner,
    values::{Design, Setting, StateValue, StateValues, pose_state},
    workspace::{self, FileStore, Files, Location},
};
use geop_lib_stdlib::WithStandardParts;
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
use geop_core_program::testing::{assert_incremental, assert_round_trips, edit_sequence};
use geop_core_sketch::PointId;
use geop_ops_datums::{ORIGIN, component_name, datum};
use geop_ops_extrude_revolve::{
    ExtrudeArgs,
    examples::{box_with_drill_hole, parametric_plate},
    testing::{edit_args, model_of},
};
use geop_ops_model::{Point, element, read_model, testing::assert_close};
use geop_ops_parameters::{COLOR, instances, read_properties};
use geop_ops_pattern::spacing::Spacing;
use geop_ops_reference_geometry::{AddDatumArgs, Construction, Selected};
use geop_ops_sketch::{figures::n, sketch_element};

use crate::{
    AddPartArgs, DragTool, MateEntity, PartPatternArgs, PatternAxis, Resolved, examples,
    examples::pose,
    mate_name,
    placed::{Placed, locate, placed_parts},
    solving::{self, Freedom, Report, SettingsState},
};

/// The operations the tests of assemblies write their programs in — the
/// parts sketched, extruded, rounded and mirrored, the standard parts'
/// patterns and parameters — with everything assemblies offer an editor,
/// their examples among it.
pub(crate) fn registry() -> Registry {
    let registry = (|| -> GeopResult<Registry> {
        let r = Registry::default();
        let r = geop_ops_model::register(r)?;
        let r = geop_ops_sketch::register(r)?;
        let r = geop_ops_datums::register(r)?;
        let r = geop_ops_reference_geometry::register(r)?;
        let r = geop_ops_extrude_revolve::register(r)?;
        let r = geop_ops_fillet::register(r)?;
        let r = geop_ops_pattern::register(r)?;
        let r = crate::register(r)?;
        geop_ops_parameters::register(r)
    })();
    registry.expect("the operations each have a name of their own")
}

/// An editor of [`registry`], the standard parts beside the files it is
/// sent.
pub(crate) fn editor() -> Editor {
    Editor::new(registry(), WithStandardParts(BTreeMap::new()))
}

/// A workspace of `files` beside the standard parts, whose programs are
/// written in the operations of [`registry`].
pub(crate) type Workspace<F = BTreeMap<String, String>> = Files<WithStandardParts<F>>;

/// The workspace of `files`, beside the standard parts (see
/// [`Workspace`]).
pub(crate) fn workspace<F: FileStore + 'static>(files: F) -> Rc<Workspace<F>> {
    Workspace::new(WithStandardParts(files), Rc::new(registry().operations()))
}

/// A part a program built, and the runner that built it, to run it again.
pub(crate) struct BuiltPart {
    pub(crate) runner: Runner,
    pub(crate) part: Part,
}

/// Builds `program`, the file `file` of `workspace`, with its parameters
/// as it defines them.
pub(crate) fn build_file(
    program: &Program,
    file: &str,
    workspace: Rc<dyn workspace::Workspace>,
) -> GeopResult<BuiltPart> {
    let mut runner = Runner::new(registry().operations());
    runner.workspace = Some(workspace);
    runner.location = Location {
        file: workspace::resolve("", file),
        within: Vec::new(),
    };
    let mut part = Part::default();
    program.run(&runner, &mut part)?;
    Ok(BuiltPart { runner, part })
}

/// An editor of [`registry`] on the example `name`.
fn example_editor(name: &str) -> Editor {
    let mut editor = editor();
    handled(&mut editor, Command::LoadExample { name: name.into() });
    editor
}

/// What a program built, as a run left it, the program with the state it
/// ran with, and the numbers its parameters gave: what the tests ask of it.
struct Built {
    runner: Runner,
    part: Part,
    program: Program,
}

/// `program` built in `workspace`, or why its first step that fails does.
fn build<F: FileStore + 'static>(
    program: &Program,
    workspace: &Rc<Workspace<F>>,
) -> GeopResult<Built> {
    let BuiltPart { runner, part } = build_file(program, "", workspace.clone())?;
    Ok(Built {
        runner,
        part,
        program: program.clone(),
    })
}

impl Built {
    fn view(&self) -> geop_core_program::reader::Reader<'_> {
        self.part.current()
    }

    /// The program, to run again by a solve.
    fn rerun(&mut self) -> Rerun<'_> {
        Rerun {
            runner: &self.runner,
            part: &mut self.part,
            program: &self.program,
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
        (instances(&self.view()).unwrap().into_iter())
            .map(|(name, instance)| (name.to_string(), instance.file.clone()))
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
    /// [`geop_ops_reference_geometry::Shape`] — where the part has it: of a placed
    /// part, where it is placed.
    fn placed(
        &self,
        entity: &MateEntity,
        of: impl Fn(&geop_ops_reference_geometry::Shape) -> Option<Vector3<Design>>,
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
            box_with_drill_hole().to_json().unwrap(),
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
    let part = build(&program, &workspace).unwrap();
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
    let error = |program: Program, file: &str| match build(&program, &workspace) {
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
    let mut editor = editor();
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
    let mut keep = |update: &Update| {
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
    edit_args(&mut longer.steps[1], |extrude: &mut ExtrudeArgs| {
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
    edit_args(&mut program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear()
    });
    program.state.insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
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
    edit_args(&mut program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear();
        pin.set_fixed(true);
    });
    program.state.insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
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
    edit_args(&mut program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear()
    });
    program.state.insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
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
            .steps,
        program.steps
    );
    let part = build(&program, &workspace(parts())).unwrap();
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
    program.state.insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
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
    edit_args(&mut program.steps[0], |plate: &mut AddPartArgs| {
        plate.set_fixed(false)
    });
    edit_args(&mut program.steps[1], |pin: &mut AddPartArgs| {
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
    program.steps.insert(1, mark);
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
    let part = build(editor.program(), &workspace(parts())).unwrap();
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
    edit_args(&mut program.steps[0], |plate: &mut AddPartArgs| {
        plate.set_fixed(false)
    });
    let mut mates = BTreeMap::new();
    edit_args(&mut program.steps[1], |pin: &mut AddPartArgs| {
        mates = std::mem::take(&mut pin.mates)
    });
    program.state.insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
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
    let mut editor = example_editor("pin");
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
    let mut editor = editor();
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
    edit_args(&mut hinge.steps[1], |pin: &mut AddPartArgs| {
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
                crate::Mate::constraint(
                    crate::Kind::Distance { value: n(3.0) },
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
    let mut editor = editor();
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

/// A mate on a part of a placed part's part is solved from where the files
/// have them: the hinge's pin slides up to it.
#[test]
fn a_mate_on_a_part_of_a_sub_assembly_is_solved() {
    let (files, top) = hinge();
    let files: BTreeMap<String, String> = (files.into_iter())
        .filter_map(|(f, t)| Some((f, t?)))
        .collect();
    let mut built = build(&top, &workspace(files)).unwrap();
    let solvable = solving::solved_states(&built.view()).unwrap();
    assert!(solvable.contains(&pose_state("hinge/pin")), "{solvable:?}");
    let constraints = geop_ops_constraints::Constraints::of(&built.view()).unwrap();
    let mate = constraints.get("add_part(hinge,m1)").unwrap();
    let by: Vec<&String> = mate.jacobian.keys().collect();
    assert!(by.contains(&&pose_state("hinge/pin")), "{by:?}");
    let report = built.check_mates().unwrap();
    assert!(!report.converged, "the pin is not up yet");
    let (moved, report) = solving::solve(&mut built.rerun(), &top.state, &[], None).unwrap();
    assert!(report.converged, "{report:?}");
    let pin = match moved.get(&pose_state("hinge/pin")) {
        Some(StateValue::Pose(pose)) => *pose,
        other => panic!("the pin did not move: {other:?}, {:?}", moved.keys()),
    };
    assert_close(position(&pin), [1.0, 1.0, 1.0], 1e-7);
}

/// The parts of a sub-assembly are the program's to move:
/// its mate slides the hinge's pin up — the pin's pose a parameter of the
/// program, relative to the hinge — and the hinge is drawn with it there.
#[test]
fn a_flexible_sub_assembly_moves_its_parts() {
    let (files, program) = hinge();
    let mut editor = editor_with(files, program, "top.geop");
    let state = &editor.program().state;
    assert!(
        state.contains_key(&pose_state("hinge/pin")),
        "the pin's pose is kept with the program: {:?}",
        state.keys().collect::<Vec<_>>()
    );
    let loaded = pose_of(editor.program(), "hinge/pin");
    let shown = editor.handle(Command::Show);
    let errors: Vec<_> = (shown.program.iter())
        .flat_map(|program| &program.steps)
        .filter_map(|step| step.error.as_ref())
        .collect();
    assert!(errors.is_empty(), "{errors:?}");
    assert_close(position(&loaded), [1.0, 1.0, 1.0], 1e-7);
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
    edit_args(&mut program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear()
    });
    program.state.insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    let update = editor.handle(Command::Tool {
        key: Some("drag".into()),
    });
    assert_eq!(update.program.unwrap().tool.as_deref(), Some("drag"));

    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: from_above(3.5, 1.0),
            shift: false,
        },
    });
    let tool = update.tool.unwrap();
    assert!(tool.grab, "a press grabs the pin");
    assert!(matches!(&tool.visuals[0].shape, Shape::Instance { name } if name == "pin"));

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
    let library = workspace.clone();
    editor.handle(Command::Tool {
        key: Some("drag".into()),
    });
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
        let dragged = editor
            .tool::<DragTool>()
            .and_then(DragTool::dragged)
            .cloned();
        times.push((started.elapsed(), i, dragged));
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
    let library = workspace.clone();
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
    let hidden = |update: Update| update.scene.unwrap().hidden;
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
    let mut editor = editor();
    let files: BTreeMap<String, Option<String>> = examples::workspaces()
        .into_iter()
        .find(|(name, _)| *name == "pin_in_plate")
        .unwrap()
        .1()
    .into_iter()
    .map(|(path, program)| (path.to_string(), Some(program.to_json().unwrap())))
    .collect();
    let mut known = BTreeMap::new();
    let mut keep = |update: Update| {
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
        let part = build(&program, &workspace(files)).unwrap();
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
    let library = workspace.clone();
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
    let mut part = build(&program, &workspace(files)).unwrap();
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
    let library = workspace.clone();
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
    let library = workspace.clone();
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
            layout: crate::Layout::Circular {
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
        let plane = part.placed(&top, |shape| {
            shape.plane.as_ref().map(|plane| *plane.origin())
        });
        assert!((plane[2] - 2.0).abs() < 1e-9);
    };
    check(&program, 1.0, 1.0);
    program.state.insert(pose_state("pin"), at([3.0, 1.0, 0.0]));
    check(&program, 1.0, 2.0);
    program
        .state
        .insert(pose_state("plate"), at([10.0, 0.0, 0.0]));
    program
        .state
        .insert(pose_state("pin"), at([12.0, 1.0, 0.0]));
    check(&program, 11.0, 1.0);

    // Refused, saying why: a part no step placed, a pattern of one.
    edit_args(&mut program.steps[2], |pattern: &mut PartPatternArgs| {
        pattern.part = "nut".into()
    });
    let err = build(&program, &library).err().unwrap().to_string();
    assert!(err.contains("no part placed so far"), "{err}");
    edit_args(&mut program.steps[2], |pattern: &mut PartPatternArgs| {
        pattern.part = "pin".into();
        pattern.count = 1.0.into();
    });
    let err = build(&program, &library).err().unwrap().to_string();
    assert!(err.contains("needs 2 or more"), "{err}");
}

/// What a part pattern reads is recorded: moving the plate whose hole it
/// turns the pins round makes its copies again, moving the pin it copies —
/// which the copies follow by their placement, not by what the pattern
/// read — makes nothing, and running it again unchanged makes nothing.
#[test]
fn a_part_pattern_is_made_again_only_when_what_it_reads_changes() {
    let workspace = workspace(parts());
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
            axis: Some(PatternAxis::Round(
                element("extrude(hole,hole_sketch,c1)").behind("plate"),
            )),
            reversed: false,
            layout: crate::Layout::Circular {
                angle: Spacing::step(60.0),
            },
            count: 6.0.into(),
        },
    );
    program.state = BTreeMap::from([
        (pose_state("plate"), at([0.0; 3])),
        (pose_state("pin"), at([2.0, 1.0, 0.0])),
    ]);
    let mut built = build(&program, &workspace).unwrap();
    let mut run = |program: &Program| {
        let before = built.runner.counts().makes;
        program.run(&built.runner, &mut built.part).unwrap();
        built.runner.counts().makes - before
    };
    assert_eq!(run(&program), 0, "nothing changed");

    program.state.insert(pose_state("pin"), at([3.0, 1.0, 0.0]));
    assert_eq!(
        run(&program),
        0,
        "the pin moved, which the pattern does not read"
    );

    program
        .state
        .insert(pose_state("plate"), at([1.0, 0.0, 0.0]));
    assert!(
        run(&program) > 0,
        "the plate moved, round whose hole the pins turn"
    );
    let rebuilt = build(&program, &workspace).unwrap();
    let (reader, anew) = (built.part.current(), rebuilt.view());
    for k in 1..6 {
        let name = format!("part_pattern(pins,{k})");
        assert_eq!(
            geop_ops_parameters::located(&reader, &name).unwrap(),
            geop_ops_parameters::located(&anew, &name).unwrap(),
            "{name} is where building anew puts it"
        );
    }
}

/// Two links on revolute joints at the ends of a fixed one, coupled as a
/// pair of gears 2:1 turning opposite ways: setting the first to 90° turns
/// the second to -45°.
#[test]
fn a_gear_coupling_turns_the_driven_link_by_its_ratio() {
    use crate::{CouplingKind, JointKind, Mate, Transmission};
    let (files, _) = workspace_example("arm");
    let workspace = workspace(files);
    let library = workspace.clone();
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
    let gear = &program.steps[2].args["mates"]["m2"];
    assert_eq!(
        (&gear["type"], &gear["value"], &gear["reverse"]),
        (&"gear".into(), &2.0.into(), &true.into()),
        "{json}"
    );
    assert_eq!(Program::from_json(&json).unwrap().steps, program.steps);

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

/// A part is built from its file as the file is: written as it was,
/// nothing changes; written otherwise, the file is read again.
#[test]
fn a_part_is_built_from_its_file_as_written() {
    let text = box_with_drill_hole().to_json().unwrap();
    let files = BTreeMap::from([("part.geop".to_string(), text.clone())]);
    let workspace = workspace(files);
    let placed =
        || geop_core_program::workspace::Workspace::program(&*workspace, "part.geop").map(|_| ());
    assert!(placed().is_ok(), "the part is placed");
    assert!(!workspace.write("part.geop", Some(text)));
    assert!(placed().is_ok(), "written as it was");
    assert!(workspace.write("part.geop", Some("not a program".into())));
    assert!(placed().is_err(), "written, the file is read again");
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
    edit_args(&mut program.steps[1], |pin: &mut AddPartArgs| {
        mates = std::mem::take(&mut pin.mates)
    });
    program.state.insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let mut editor = editor_on(program);
    editor.handle(Command::Open { id: "pin".into() });
    editor.handle(dialog("add_mate", Value::Choice("revolute".into())));
    let frames = mates["m1"]
        .entities
        .iter()
        .map(Selectable::picked)
        .collect();
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
    let mut editor = editor();
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
            neither `geop_core_editor::step_editor` nor `Selectable` has a way to draw \
            what a pick is (the highlight and the joint the clicks make pass)"]
fn a_joint_is_made_by_clicking_where_its_frames_go() {
    let mut program = examples::pin_in_plate_assembly();
    edit_args(&mut program.steps[1], |pin: &mut AddPartArgs| {
        pin.mates.clear()
    });
    program.state.insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
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
    let pin_top = geop_core_editor::view::FramePick {
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

/// The `bolted_plate` example holds together: the screw down the hole, the
/// nut under the plate, upside down — its base on the plate's bottom.
#[test]
fn the_bolted_plate_holds_together() {
    let files = BTreeMap::from([(
        "plate.geop".to_string(),
        examples::metric_plate().to_json().unwrap(),
    )]);
    let runner = build_file(
        &examples::bolted_plate(),
        "bolted_plate.geop",
        workspace(files),
    )
    .unwrap();
    let view = runner.part.current();
    assert!(solving::check(&view).unwrap().converged);
    let v = |p: [f64; 3]| Vector3::from_array(p.map(Design::from_f64));
    for (instance, local, want) in [
        ("screw", [0.0, 0.0, -12.0], [20.0, 20.0, -7.0]),
        ("nut", [0.0, 0.0, 3.2], [20.0, 20.0, -3.2]),
    ] {
        let located = locate(&view, instance).unwrap();
        let got = located.world.apply(&v(local));
        let got = [0, 1, 2].map(|k| got[k].to_f64());
        assert!(
            (0..3).all(|k| (got[k] - want[k]).abs() < 1e-6),
            "{instance} {local:?} is at {got:?}, not {want:?}"
        );
    }
}

/// The examples whose files take long to build: the press, rounded, and
/// the bolted plate, of standard parts.
const SLOW_EXAMPLES: [&str; 2] = ["press", "bolted_plate"];

/// Every file of the examples of assemblies, `slow` or not (see
/// [`SLOW_EXAMPLES`]) — of one made of several files, and an example
/// program as the one file `part.geop` — with the workspace of its
/// example's files, saved: what `check` is asked of in turn.
fn for_every_example_file(
    slow: bool,
    check: impl Fn(Rc<dyn workspace::Workspace>, &str, &Program),
) {
    let set = examples::set();
    let programs = (set.programs.iter()).map(|(name, make)| (*name, vec![("part.geop", make())]));
    let workspaces = (set.workspaces.iter()).map(|(name, make)| (*name, make()));
    for (name, files) in workspaces.chain(programs) {
        if SLOW_EXAMPLES.contains(&name) != slow {
            continue;
        }
        let saved: BTreeMap<String, String> = (files.iter())
            .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
            .collect();
        let workspace: Rc<dyn workspace::Workspace> = workspace(saved);
        for (file, program) in &files {
            check(workspace.clone(), file, program);
        }
    }
}

/// Every file of the examples of assemblies `slow` or not is saved and
/// read back as the same program, which builds — beside the other files of
/// its example — the very same part (see [`assert_round_trips`]).
fn assert_examples_round_trip(slow: bool) {
    let operations = Rc::new(registry().operations());
    for_every_example_file(slow, |workspace, file, program| {
        assert_round_trips(operations.clone(), Some((workspace, file)), program);
    });
}

/// [`assert_examples_round_trip`] on the examples quick to build.
#[test]
fn examples_round_trip() {
    assert_examples_round_trip(false);
}

/// [`assert_examples_round_trip`] on the [`SLOW_EXAMPLES`].
#[test]
#[ignore = "slow: the press and the bolted plate, built twice — run with `cargo test -- --ignored`"]
fn slow_examples_round_trip() {
    assert_examples_round_trip(true);
}

/// The edits [`assert_examples_incremental`] runs a file of an example
/// through: those of every example ([`edit_sequence`]), and each part it
/// places moved half a unit along `x` and put back.
fn edited(original: &Program) -> Vec<Program> {
    let mut programs = edit_sequence(original);
    for (name, value) in &original.state {
        if let StateValue::Pose(placed) = value {
            let [x, y, z] = position(placed);
            let turn = placed.euler_degrees();
            let mut moved = original.clone();
            let to = pose([x + 0.5, y, z], turn);
            moved.state.insert(name.clone(), StateValue::Pose(to));
            programs.extend([moved, original.clone()]);
        }
    }
    programs
}

/// Every file of the examples of assemblies `slow` or not, edited as
/// [`edited`] says, builds incrementally what it builds from scratch (see
/// [`assert_incremental`]).
fn assert_examples_incremental(slow: bool) {
    let operations = Rc::new(registry().operations());
    for_every_example_file(slow, |workspace, file, program| {
        assert_incremental(
            operations.clone(),
            Some((workspace, file)),
            &edited(program),
        );
    });
}

/// [`assert_examples_incremental`] on the examples quick to build.
#[test]
fn examples_build_incrementally() {
    assert_examples_incremental(false);
}

/// [`assert_examples_incremental`] on the [`SLOW_EXAMPLES`].
#[test]
#[ignore = "slow: the press and the bolted plate, built several times over — run with `cargo test -- --ignored`"]
fn slow_examples_build_incrementally() {
    assert_examples_incremental(true);
}

/// The press handle's rim rounded: a bar 236 long whose rim lies 225 from
/// the origin. The blend face's control points were 8e-8 wide, and a ray
/// grazing it at its silhouette could not be resolved (see
/// `geop_ops_drawing`'s `press_from_the_iso_view_draws`): the cylinder's normal
/// was taken at the bar's far end and turned around an axis known to
/// 1e-12, and the weight was `r / |E - C|`, with the corner `E`'s width in
/// it twice. What is left is the weight's width, from the faces'
/// directions, times the distance from the origin.
#[test]
fn press_handle_rim_fillet_is_narrow() {
    let (_, files) = (examples::workspaces().into_iter())
        .find(|(n, _)| *n == "press")
        .unwrap();
    let (_, handle) = (files().into_iter())
        .find(|(path, _)| *path == "handle.geop")
        .unwrap();
    let built = build_file(&handle, "handle.geop", workspace(BTreeMap::new())).unwrap();
    let part = model_of(&built.part);
    let face = part
        .face_id("fillet(fillet1,revolve(revolve1,sketch2,p5,q3),fillet,q3)")
        .unwrap();
    let surface = &part.topology().get_face(face).unwrap().surface;
    let widest = surface
        .control_points
        .iter()
        .flat_map(|p| (0..4).map(move |k| p[k].width().to_f64()))
        .fold(0.0, f64::max);
    assert!(widest < 3e-9, "{widest:e}: {:?}", surface.control_points);
}

/// The value of the parameter `name` the part `view` shows was built with:
/// a table's row, a number, a table's column (`table.column`) — or, for
/// [`COLOR`], its colour.
fn input(view: &Reader, name: &str) -> Option<Setting> {
    if name == COLOR {
        return read_properties(view).ok()?.color.clone().map(Setting::Text);
    }
    let values = read_parameter_values(view).ok()?;
    let (_, value) = values.0.iter().find(|(n, _)| n == name)?;
    match value {
        ParameterValue::Number(n) => Some(Setting::Number(*n)),
        ParameterValue::Text(row) => Some(Setting::Text(row.clone())),
    }
}

/// Placed by another program, the plate is built with the values given
/// there: wider, with a larger hole, in another colour — and as it is
/// where nothing is given.
#[test]
fn placed_parts_take_the_parameters_given() {
    let workspace = workspace(BTreeMap::from([(
        "plate.geop".to_string(),
        parametric_plate().to_json().unwrap(),
    )]));
    let program = examples::plates_assembly();
    let part = build_file(&program, "plates.geop", workspace).unwrap().part;
    let view = part.current();
    // An element of a placed part, where it is placed.
    let corner = |instance: &str| {
        let located = locate(&view, instance).unwrap();
        let component = located.part.clone();
        let reference = sketch_element::<Point>("outline", PointId(2));
        let p = located
            .world
            .apply(&component.retrieve_element(&reference).unwrap());
        [p[0].to_f64(), p[1].to_f64()]
    };
    let small = corner("small");
    let large = corner("large");
    assert!(
        (small[0] - 4.0).abs() < 1e-9 && (small[1] - 2.0).abs() < 1e-9,
        "{small:?}"
    );
    // 5 wide, and so 2.5 deep; placed 3 along y.
    assert!(
        (large[0] - 5.0).abs() < 1e-9 && (large[1] - 5.5).abs() < 1e-9,
        "{large:?}"
    );
    let placed = placed_parts(&view).unwrap();
    let color = |part: &Placed| input(&part.located.part.clone(), COLOR);
    let colors: Vec<Option<Setting>> = placed.iter().map(color).collect();
    let text = |c: &str| Some(Setting::Text(c.into()));
    assert!(colors.contains(&text("#3e7bd0")), "{colors:?}");
    assert!(colors.contains(&text("#d0893e")), "{colors:?}");
    // The table's row and the number given are what the large plate was
    // built with.
    let large = placed.iter().find(|p| color(p) == text("#3e7bd0")).unwrap();
    let large = large.located.part.clone();
    assert_eq!(input(&large, "screw"), text("M6"));
    let clearance = match input(&large, "screw.clearance") {
        Some(Setting::Number(v)) => Some(v),
        _ => None,
    };
    assert_eq!(clearance, Some(0.66));
}

/// The triangles of the STL mesh of `part`, which `runner` built from
/// `program`: of the solids named `solids`, of all if none are.
fn stl_triangles(
    runner: &Runner,
    part: &mut Part,
    program: &Program,
    solids: &[&str],
) -> GeopResult<usize> {
    use geop_core_editor::output::{Exporter, Output};

    let mut output = Output {
        runner,
        part,
        file: program,
        editing: None,
        date: "",
    };
    let options = crate::stl::StlOptions {
        solids: solids.iter().map(|s| s.to_string()).collect(),
        ..Default::default()
    };
    let export = crate::stl::Stl::export(&mut output, "stl", options)?;
    let bytes = export.bytes();
    let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    assert_eq!(bytes.len(), 84 + 50 * count);
    Ok(count)
}

/// An assembly is meshed with the parts it places, read from the files
/// beside it, each where it is placed, as `geop compile assembly.geop`
/// writes it: its triangles are the plate's and the pin's; one solid of a
/// placed part is chosen by its name behind the instance's. Its state stale,
/// the pin's mates are reported as not holding, and solved for, it is
/// meshed as before.
#[test]
fn an_assembly_is_meshed_from_its_files() {
    let mut files = parts();
    let assembly = examples::pin_in_plate_assembly();
    files.insert("assembly.geop".into(), assembly.to_json().unwrap());
    let workspace = workspace(files.clone());
    let mut sizes = BTreeMap::new();
    for (file, text) in &files {
        let program = Program::from_json(text).unwrap();
        let BuiltPart { runner, mut part } = build_file(&program, file, workspace.clone()).unwrap();
        let triangles = stl_triangles(&runner, &mut part, &program, &[]).unwrap();
        sizes.insert(file.as_str(), triangles);
    }
    assert_eq!(
        sizes["assembly.geop"],
        sizes["plate.geop"] + sizes["pin.geop"]
    );
    let BuiltPart { runner, mut part } =
        build_file(&assembly, "assembly.geop", workspace.clone()).unwrap();
    let pin = stl_triangles(&runner, &mut part, &assembly, &["pin/extrude(pin)"]).unwrap();
    assert_eq!(pin, sizes["pin.geop"]);

    let mut stale = assembly;
    stale.state.insert(pose_state("pin"), at([3.5, 1.0, 0.0]));
    let BuiltPart { runner, mut part } =
        build_file(&stale, "assembly.geop", workspace.clone()).unwrap();
    let report = solving::check(&part.current()).unwrap();
    assert!(!report.converged);
    assert_eq!(report.failed, ["add_part(pin,m1)", "add_part(pin,m2)"]);
    let mut rerun = Rerun {
        runner: &runner,
        part: &mut part,
        program: &stale,
    };
    let (moved, report) = solving::solve(&mut rerun, &stale.state, &[], None).unwrap();
    assert!(report.converged, "{report:?}");
    stale.state.extend(moved);
    stale.run(&runner, &mut part).unwrap();
    assert!(solving::check(&part.current()).unwrap().converged);
    let triangles = stl_triangles(&runner, &mut part, &stale, &[]).unwrap();
    assert_eq!(triangles, sizes["assembly.geop"]);
}

/// A solid asked for by a name no solid of the part has is refused, naming
/// those there are.
#[test]
fn an_unknown_solid_names_the_known_ones() {
    let program = box_with_drill_hole();
    let BuiltPart { runner, mut part } =
        build_file(&program, "box.geop", workspace(BTreeMap::new())).unwrap();
    let err = stl_triangles(&runner, &mut part, &program, &["nothing"]).unwrap_err();
    assert!(err.to_string().contains("extrude(hole)"), "{err}");
}
