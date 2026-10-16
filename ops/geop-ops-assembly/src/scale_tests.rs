//! Large assemblies: a workspace of a plate with hundreds of screws and
//! standoffs placed on it, through a few levels of sub-assemblies — what a
//! robot is. Editing one file rebuilds only what places it, a placed part
//! is drawn once however often it is placed, and the timings of the whole
//! (an ignored test) are printed.

use crate::{
    examples::pose,
    program_tests::{Workspace, build_file, editor, registry},
};
use geop_core_editor::{
    editor::{Command, Editor, Update},
    testing::handled,
};
use geop_core_program::{
    part::Part,
    program::Program,
    values::{Design, StateValue, StateValues, pose_state},
    workspace::{FileStore, FileStoreMut},
};
use geop_ops_extrude_revolve::testing::{extrusion, on_origin, sketch};
use geop_ops_sketch::figures::{circle, rectangle, solved};
use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet},
    rc::Rc,
    time::Instant,
};

use crate::{
    AddPartArgs, Kind, Mate, MateEntity, Resolved,
    placed::{locate, placed_parts},
    solving::check as check_mates,
};
use geop_core_editor::editing::{Pointer, Reach, StepEditEvent};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{FrameAxis, Pose, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_lib_stdlib::WithStandardParts;
use geop_ops_extrude_revolve::Extents;
use geop_ops_model::Combine;
use geop_ops_model::{element, read_model};
use geop_ops_reference_geometry::Selected;
use geop_ops_sketch::Sketch;

/// A part of one extrusion `height` tall, of a sketch on the origin's Z
/// plane: its faces are `extrude(<id>,start)` below and `extrude(<id>,end)`
/// above.
fn extruded(id: &str, drawn: Sketch, height: f64) -> Program {
    let mut program = Program::new();
    let sketch_id = format!("{id}_sketch");
    program.push(
        sketch_id.clone(),
        sketch(on_origin(FrameAxis::Z), solved(drawn)),
    );
    program.push(
        id,
        extrusion(&sketch_id, Extents::blind(height), Combine::NewBody),
    );
    program
}

/// A round part of `radius`, `height` tall.
fn round(id: &str, radius: f64, height: f64) -> Program {
    let mut sketch = Sketch::new();
    circle(&mut sketch, [0.0, 0.0], radius);
    extruded(id, sketch, height)
}

/// A 10 x 10 plate, 0.2 thick.
fn plate() -> Program {
    let mut sketch = Sketch::new();
    rectangle(&mut sketch, [0.0, 0.0], 10.0, 10.0);
    extruded("plate", sketch, 0.2)
}

/// The face `face` of the part placed as `instance` — `plate/extrude(…)`,
/// of a part placed in it.
fn face(instance: &str, face: &str) -> MateEntity {
    let (part, face) = match face.split_once('/') {
        Some((inner, face)) => (format!("{instance}/{inner}"), face),
        None => (instance.to_string(), face),
    };
    MateEntity::of(&part, Selected::Face(element(face)))
}

/// Placed from `file`, its face `below` lying on the face `on`.
fn resting(file: &str, below: MateEntity, on: MateEntity) -> AddPartArgs {
    AddPartArgs {
        file: file.into(),
        mates: BTreeMap::from([(
            "m1".into(),
            Mate::constraint(Kind::Coincident, vec![below, on]),
        )]),
        ..Default::default()
    }
}

/// A program placing `plate.geop` fixed as `plate`, then every part of
/// `parts` — `(id, file, bottom face, position)` — lying on the plate's top
/// or on the face named, where its state puts each.
fn assembly(parts: Vec<(String, &str, MateEntity, MateEntity, [f64; 3])>) -> Program {
    let mut program = Program::new();
    program.push(
        "plate",
        AddPartArgs {
            file: "plate.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    let mut state = StateValues::from([(
        pose_state("plate"),
        StateValue::Pose(pose([0.0; 3], [0.0; 3])),
    )]);
    for (id, file, below, on, at) in parts {
        state.insert(pose_state(&id), StateValue::Pose(pose(at, [0.0; 3])));
        program.push(id, resting(file, below, on));
    }
    program.state = state;
    program
}

/// The program files of an assembly of `boards` boards and `screws` more
/// screws on a plate, by path; the top one is `robot.geop`. A module is a
/// plate with 10 screws on it; a board a plate with 4 standoffs and 4
/// modules on them: 44 parts. So the robot places `44 * boards + screws`
/// screws and standoffs, besides the plates.
fn robot(boards: usize, screws: usize) -> BTreeMap<String, String> {
    let top = |id: &str| face(id, "extrude(plate,end)");
    let screw = |id: String, x: f64, y: f64, z: f64| {
        let below = face(&id, "extrude(screw,start)");
        (id, "screw.geop", below, top("plate"), [x, y, z])
    };
    let module = assembly(
        (0..10)
            .map(|i| screw(format!("screw{i}"), 0.5 + i as f64 * 0.9, 0.5, 0.2))
            .collect(),
    );
    let mut board_parts: Vec<_> = (0..4)
        .map(|i| {
            let id = format!("standoff{i}");
            let below = face(&id, "extrude(standoff,start)");
            let at = [1.0 + 8.0 * (i % 2) as f64, 1.0 + 8.0 * (i / 2) as f64, 0.2];
            (id, "standoff.geop", below, top("plate"), at)
        })
        .collect();
    board_parts.extend((0..4).map(|i| {
        let id = format!("module{i}");
        let below = face(&id, "plate/extrude(plate,start)");
        let on = face("standoff0", "extrude(standoff,end)");
        (id, "module.geop", below, on, [0.0, i as f64 * 2.5, 2.2])
    }));
    let board = assembly(board_parts);
    let mut robot_parts: Vec<_> = (0..boards)
        .map(|i| {
            let id = format!("board{i}");
            let below = face(&id, "plate/extrude(plate,start)");
            (
                id,
                "board.geop",
                below,
                top("plate"),
                [(i + 1) as f64 * 11.0, 0.0, 0.2],
            )
        })
        .collect();
    robot_parts.extend((0..screws).map(|i| {
        let (x, y) = ((i % 20) as f64 * 0.5 + 0.25, (i / 20) as f64 * 0.5 + 0.25);
        screw(format!("screw{i}"), x, y, 0.2)
    }));
    let robot = assembly(robot_parts);
    [
        ("robot.geop", robot),
        ("board.geop", board),
        ("module.geop", module),
        ("plate.geop", plate()),
        ("screw.geop", round("screw", 0.15, 1.0)),
        ("standoff.geop", round("standoff", 0.25, 2.0)),
    ]
    .into_iter()
    .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
    .collect()
}

/// Files in memory that count how often each is read: a file is read to
/// be built, so the count is how often it was built.
#[derive(Default)]
struct Counted {
    files: BTreeMap<String, String>,
    reads: RefCell<BTreeMap<String, usize>>,
}

impl Counted {
    fn take(&self) -> BTreeMap<String, usize> {
        std::mem::take(&mut self.reads.borrow_mut())
    }
}

impl FileStore for Counted {
    fn read(&self, path: &str) -> GeopResult<String> {
        *self.reads.borrow_mut().entry(path.to_string()).or_default() += 1;
        self.files.read(path)
    }

    fn list(&self) -> Vec<String> {
        self.files.list()
    }
}

impl FileStoreMut for Counted {
    fn write(&mut self, path: &str, text: Option<String>) {
        self.files.write(path, text);
    }
}

/// The files the robot places, however deep.
fn files_placed(part: &Part) -> BTreeSet<String> {
    (placed_parts(&part.current()).unwrap().into_iter())
        .map(|placed| placed.located.instance.file.clone())
        .collect()
}

/// Removing one of the robot's own screws sends no other placed part's view
/// again: what the solve after it moves is the robot's to move, and a part
/// run over a box of its poses and back is drawn as it was.
#[test]
fn removing_a_screw_sends_no_other_part_again() {
    let (mut editor, _) = robot_editor(1, 20);
    let before = editor.program().state.clone();
    let last = editor.program().steps.last().unwrap().id.clone();
    let update = handled(&mut editor, Command::Remove { id: last });
    let scene = update.scene.expect("the screw is gone");
    let after = &editor.program().state;
    let changed: Vec<&String> = (after.keys().chain(before.keys()))
        .filter(|name| before.get(*name) != after.get(*name))
        .collect();
    let sent: Vec<&str> = (scene.instances.iter())
        .filter(|instance| scene.components.contains_key(&instance.component))
        .map(|instance| instance.name.as_str())
        .collect();
    assert!(
        scene.components.is_empty(),
        "sent again: {} views, of {sent:?}; the state changed in {changed:?}",
        scene.components.len()
    );
}

/// An edited file is placed as it is now, however deep it is placed;
/// saving a file unchanged changes nothing.
///
/// TODO: Editing one file should build again only it and the files placing
/// it, however deep, once the parts of no state are shared between
/// instances (see `geop_core_program::part::Part`): every instance of the
/// file builds its part again now.
#[test]
fn an_edited_file_is_placed_as_it_is_now_however_deep() {
    let mut files = robot(2, 3);
    files.remove("robot.geop");
    let workspace = Workspace::<Counted>::new(
        WithStandardParts(Counted {
            files,
            ..Default::default()
        }),
        Rc::new(registry().operations()),
    );
    let robot = robot(2, 3).remove("robot.geop").unwrap();
    // The robot itself is the program being built, not a file read.
    let built = |workspace: &Rc<Workspace<Counted>>| {
        let program = Program::from_json(&robot).unwrap();
        build_in(workspace, &program)
    };
    assert_eq!(files_placed(&built(&workspace)).len(), 5);

    // The same text again: nothing changed.
    let screw = workspace.files().files["screw.geop"].clone();
    assert!(!workspace.write("screw.geop", Some(screw)));

    // A longer screw: the module places it, and the board the module.
    let longer = round("screw", 0.15, 1.5).to_json().unwrap();
    assert!(workspace.write("screw.geop", Some(longer)));
    let part = built(&workspace);
    // It is the longer screw that is placed, however deep.
    for screw in ["screw0", "board1/module3/screw9"] {
        let reader = part.current();
        let located = locate(&reader, screw).unwrap();
        let heights: BTreeSet<_> = read_model(&located.part.clone())
            .unwrap()
            .topology()
            .vertices
            .values()
            .map(|v| v.point[2].to_f64().to_bits())
            .collect();
        assert!(heights.contains(&1.5f64.to_bits()), "{screw}: {heights:?}");
    }
}

/// Moving one screw of an assembly builds again the step that places it and
/// what reads where it is, not the placing of the others: and the part is
/// what a build from scratch makes.
#[test]
fn moving_a_part_builds_again_only_what_reads_where_it_is() {
    let mut files = robot(0, 30);
    let robot = Program::from_json(&files.remove("robot.geop").unwrap()).unwrap();
    let workspace = Workspace::<Counted>::new(
        WithStandardParts(Counted {
            files,
            ..Default::default()
        }),
        Rc::new(registry().operations()),
    );
    let resolved = crate::mates_resolved();
    let mut built = build_file(&robot, "robot.geop", workspace.clone()).unwrap();
    assert_eq!(
        crate::mates_resolved() - resolved,
        31,
        "the plate's fixed mate, and each of the 30 screws' mates"
    );
    workspace.files().take();

    let mut moved = robot.clone();
    moved.state.insert(
        pose_state("screw7"),
        StateValue::Pose(pose([4.0, 3.0, 0.2], [0.0; 3])),
    );
    let resolved = crate::mates_resolved();
    let output = built.runner.run(&mut built.part, &moved, None);
    assert!(output.step_errors.is_empty(), "{:?}", output.step_errors);
    assert_eq!(
        crate::mates_resolved() - resolved,
        0,
        "a part moved resolves no mate again: only screw7's constraint reads where it is"
    );
    assert_eq!(
        workspace.files().take(),
        BTreeMap::new(),
        "and builds no file"
    );

    let scratch = build_in(&workspace, &moved);
    let at = |part: &Part| -> Pose<Design> { locate(&part.current(), "screw7").unwrap().world };
    assert_eq!(at(&built.part), at(&scratch));
}

/// `program`, the file `robot.geop` of `workspace`, built from scratch, its
/// mates holding as its state has its parts.
fn build_in(workspace: &Rc<Workspace<Counted>>, program: &Program) -> Part {
    let part = build_file(program, "robot.geop", workspace.clone())
        .unwrap()
        .part;
    assert!(check_mates(&part.current()).unwrap().converged);
    part
}

/// What the editor sends, as the front end gets it: its size in bytes,
/// and of what in it.
fn size(update: &Update) -> String {
    assert!(update.error.is_none(), "{:?}", update.error);
    let json = serde_json::to_value(update).unwrap();
    let len = |v: &serde_json::Value| v.to_string().len();
    let mut parts: Vec<String> = json
        .as_object()
        .unwrap()
        .iter()
        .filter(|(_, v)| !v.is_null())
        .flat_map(|(k, v)| match (k.as_str(), v.as_object()) {
            ("scene", Some(scene)) => scene
                .iter()
                .map(|(k, v)| format!("scene.{k} {}", len(v)))
                .collect(),
            _ => vec![format!("{k} {}", len(v))],
        })
        .collect();
    parts.sort();
    let moved = update.scene.as_ref().map_or(0, |s| s.instances.len());
    format!(
        "{} bytes ({}), {moved} placed parts sent",
        len(&json),
        parts.join(", ")
    )
}

/// A pointer straight down onto `(x, y)`.
fn down_onto(x: f64, y: f64) -> Pointer<S> {
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    Pointer {
        ray: Ray::try_new(v([x, y, 50.0]), v([0.0, 0.0, -1.0])).unwrap(),
        reach: Reach::Tube {
            radius: S::from_f64(0.01),
        },
    }
}

/// The editor editing the robot of `boards` boards and `screws` screws,
/// with every file sent.
fn robot_editor(boards: usize, screws: usize) -> (Editor, Update) {
    let mut files = robot(boards, screws);
    let robot = Program::from_json(&files.remove("robot.geop").unwrap()).unwrap();
    let mut editor = editor();
    handled(
        &mut editor,
        Command::Files {
            files: files.into_iter().map(|(p, t)| (p, Some(t))).collect(),
        },
    );
    let update = editor.handle(Command::Load {
        program: robot,
        path: Some("robot.geop".into()),
    });
    (editor, update)
}

/// Editing the robot, the way the front end does: a leaf file changed is
/// drawn again, and saved unchanged, nothing is. A view is sent once for
/// every part drawn alike, however often it is placed.
///
/// TODO: only the screw and what places it built again, once the parts of
/// no state are shared between instances (see
/// `geop_core_program::part::Part`): every screw is built on its own now.
#[test]
fn a_changed_leaf_is_drawn_again() {
    let (mut editor, update) = robot_editor(1, 20);
    let scene = update.scene.expect("the robot is drawn");
    // The plate, the board and its 53 (its plate, 4 standoffs, 4 modules of
    // a plate and 10 screws each), and 20 screws.
    assert_eq!(scene.instances.len(), 1 + 1 + 53 + 20);
    let sent = |instance: &geop_core_editor::view::ViewInstance| {
        scene.components.contains_key(&instance.component)
    };
    assert!(scene.instances.iter().all(sent));
    let screws: BTreeSet<&str> = (scene.instances.iter())
        .filter(|instance| instance.name.starts_with("screw"))
        .map(|instance| instance.component.as_str())
        .collect();
    assert_eq!(screws.len(), 1, "the robot's screws are drawn alike");

    let longer = round("screw", 0.15, 1.5).to_json().unwrap();
    let update = editor.handle(Command::Files {
        files: BTreeMap::from([("screw.geop".into(), Some(longer.clone()))]),
    });
    let scene = update.scene.expect("the screws changed");
    assert!(!scene.all && scene.removed.is_empty());
    assert!(!scene.components.is_empty());

    // Saved again unchanged: nothing to show.
    let update = editor.handle(Command::Files {
        files: BTreeMap::from([("screw.geop".into(), Some(longer))]),
    });
    assert!(update.scene.is_none(), "nothing changed");
}

/// Seconds `f` takes, and what it returns.
fn timed<T>(f: impl FnOnce() -> T) -> (f64, T) {
    let start = Instant::now();
    let out = f();
    (start.elapsed().as_secs_f64(), out)
}

/// The robot, timed: loading it, editing a leaf file and the top file, a
/// hover and a drag of the drag tool, each with the size of the update the
/// front end is sent.
#[test]
#[ignore = "slow: timings of a robot of 500 and 2000 placed parts — run with `cargo test -- --ignored`"]
fn robot_timings() {
    for (boards, screws) in [(10, 60), (40, 240)] {
        let parts = 44 * boards + screws;
        let (load, (mut editor, update)) = timed(|| robot_editor(boards, screws));
        println!(
            "robot of {parts} parts: load {load:.3} s, {}",
            size(&update)
        );

        let longer = round("screw", 0.15, 1.5).to_json().unwrap();
        let (leaf, update) = timed(|| {
            editor.handle(Command::Files {
                files: BTreeMap::from([("screw.geop".into(), Some(longer.clone()))]),
            })
        });
        println!("  leaf edit {leaf:.3} s, {}", size(&update));

        let (same, update) = timed(|| {
            editor.handle(Command::Files {
                files: BTreeMap::from([("screw.geop".into(), Some(longer.clone()))]),
            })
        });
        println!("  unchanged save {same:.3} s, {}", size(&update));

        let last = editor.program().steps.last().unwrap().id.clone();
        let (top, update) = timed(|| editor.handle(Command::Remove { id: last }));
        println!("  top-level edit {top:.3} s, {}", size(&update));

        editor.handle(Command::Tool {
            key: Some("drag".into()),
        });
        let (hover, update) = timed(|| {
            editor.handle(Command::Event {
                event: StepEditEvent::Hover {
                    pointer: down_onto(0.25, 0.25),
                    shift: false,
                },
            })
        });
        println!("  hover {hover:.4} s, {}", size(&update));

        let (drag, update) = timed(|| {
            editor.handle(Command::Event {
                event: StepEditEvent::Drag {
                    from: down_onto(0.25, 0.25),
                    to: down_onto(0.35, 0.3),
                    done: false,
                    shift: false,
                },
            })
        });
        println!("  drag {drag:.3} s, {}", size(&update));
    }
}

/// Editing the robot asks its whole assembly something a few times per
/// edit — to solve its mates, to check them — and never once per step: a
/// step list of every placed part, each asking the whole assembly how
/// free its part is, made a drag of a 2000-part robot take minutes. Counted
/// in mates resolved (see [`crate::mates_resolved`]), not
/// timed, so it shows at a modest size: per mate of the assembly — those
/// of the parts placed in it included — an edit resolves a few, however
/// many there are.
///
/// Nor once per screw: each is a group of the system no mate ties to the
/// others, and the groups are solved, enclosed and measured in lockstep,
/// one run of the program answering all of them (see
/// [`geop_core_math::solvers::system`]). Asked one group at a time, a
/// question cost a run of every step per group, and a drag of a 2000-part
/// robot took seconds. Counted in program runs (see
/// [`geop_ops_constraints::solve::runs`]): four times the screws, the same
/// runs.
#[test]
fn an_edit_resolves_each_mate_a_few_times() {
    let mut runs = Vec::new();
    for screws in [20, 80] {
        let resolved = |f: &mut dyn FnMut()| {
            let before = (crate::mates_resolved(), geop_ops_constraints::solve::runs());
            f();
            (
                crate::mates_resolved() - before.0,
                geop_ops_constraints::solve::runs() - before.1,
            )
        };
        let mut editor = None;
        let (load, load_runs) = resolved(&mut || editor = Some(robot_editor(1, screws).0));
        let mut editor = editor.unwrap();
        // Its own and those of every part placed in it, however deep: each
        // placed part is built, and its mates resolved, on its own.
        let part = editor.part();
        let placed = placed_parts(&part).unwrap();
        let mates = part.targets_of::<Resolved>().len()
            + (placed.iter())
                .map(|p| p.located.part.clone().targets_of::<Resolved>().len())
                .sum::<usize>();
        let (drag, drag_runs) = resolved(&mut || {
            editor.handle(Command::Tool {
                key: Some("drag".into()),
            });
            handled(
                &mut editor,
                Command::Event {
                    event: StepEditEvent::Drag {
                        from: down_onto(0.25, 0.25),
                        to: down_onto(0.35, 0.3),
                        done: true,
                        shift: false,
                    },
                },
            );
        });
        let last = editor.program().steps.last().unwrap().id.clone();
        let (remove, remove_runs) = resolved(&mut || {
            handled(&mut editor, Command::Remove { id: last.clone() });
        });
        runs.push((screws, [load_runs, drag_runs, remove_runs]));
        // Now 3, 7 and 3 per mate; once per step, it was 7, 14 and 7 per
        // mate with 21 mates, and 86, 173 and 84 with 81.
        for (edit, resolved) in [("load", load), ("drag", drag), ("remove", remove)] {
            assert!(
                resolved <= 10 * mates,
                "a {edit} of a robot of {mates} mates resolved {resolved}"
            );
        }
    }
    let [(_, few), (_, many)] = runs[..] else {
        unreachable!()
    };
    assert!(
        (0..3).all(|k| many[k] <= few[k]),
        "the program runs of a load, a drag and a remove grow with the screws: {runs:?}"
    );
}

/// A robot of two boards and 20 screws, driven as the front end does: the
/// drag tool takes the screw under the pointer, and dragging it sends where
/// that screw went — not the hundred other placed parts, which stay where
/// they are, nor any part's looks again.
#[test]
fn dragging_one_of_many_placed_parts_sends_only_it() {
    let (mut editor, _) = robot_editor(2, 20);
    let before = editor.program().state.clone();
    editor.handle(Command::Tool {
        key: Some("drag".into()),
    });
    // The screws stand in a row along x at y = 0.25, from x = 0.25 on.
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: down_onto(5.25, 0.25),
            shift: false,
        },
    });
    let tool = update.tool.expect("the drag tool is in hand");
    assert!(tool.grab, "a press over a screw grabs it");
    let update = handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Drag {
                from: down_onto(5.25, 0.25),
                to: down_onto(5.35, 0.3),
                done: true,
                shift: false,
            },
        },
    );
    let scene = update.scene.expect("the screw moved");
    assert!(
        scene.components.is_empty() && scene.removed.is_empty() && !scene.all,
        "sent again: {:?}, removed: {:?}, all: {}",
        scene.components.keys().collect::<Vec<_>>(),
        scene.removed,
        scene.all
    );
    let after = &editor.program().state;
    let moved: Vec<&String> = after
        .keys()
        .filter(|name| before.get(*name) != after.get(*name))
        .collect();
    assert_eq!(moved, ["screw10.pose"]);
    let sent: Vec<&str> = scene.instances.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(sent, ["screw10"]);
}
