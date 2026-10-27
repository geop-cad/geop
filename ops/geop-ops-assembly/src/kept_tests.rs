//! Assemblies kept between processes (see `geop_core_program::kept`): a
//! robot of placed parts, sub-assemblies of placed parts in it, reopened
//! from what an earlier process kept, makes none of its parts' models or
//! drawings again — and an editor opening it meshes no face.

use std::{collections::BTreeMap, rc::Rc, time::Instant};

use geop_core_editor::{
    editor::{Command, Editor},
    testing::handled,
};
use geop_core_program::{
    cache::Cache,
    placed::Located,
    program::Program,
    testing::{Shelf, assert_reopens},
    values::{StateValue, StateValues, pose_state},
    workspace::{Files, Workspace},
};
use geop_lib_fixtures::PrimitiveArgs;
use geop_lib_stdlib::WithStandardParts;
use geop_ops_model::{element, mate_entity::MateEntity, selection::Selected};

use crate::{AddPartArgs, Kind, Mate, examples::pose, program_tests::registry};

/// A program of one given solid `id`.
fn part(id: &str, solid: PrimitiveArgs) -> Program {
    let mut program = Program::new();
    program.push(id, solid);
    program
}

/// The face `face` of the part placed as `instance`.
fn face(instance: &str, face: &str) -> MateEntity {
    let face = Located::of_key(face).unwrap();
    MateEntity::of(
        instance,
        Selected::Face(element(face.name())).behind(face.path()),
    )
}

/// A plate, `plate.geop` fixed, with each of `parts` — its file, and where
/// it is — standing on it, its face `below` on the plate's top.
fn on_a_plate(parts: Vec<(String, &str, &str, [f64; 3])>) -> Program {
    let mut program = Program::new();
    let plate = AddPartArgs {
        file: "plate.geop".into(),
        ..Default::default()
    };
    program.push("plate", plate.fixed());
    let mut state = StateValues::from([(
        pose_state("plate"),
        StateValue::Pose(pose([0.0; 3], [0.0; 3])),
    )]);
    for (id, file, below, at) in parts {
        let on = face("plate", "block(plate,end)");
        let mate = Mate::constraint(Kind::Coincident, vec![face(&id, below), on]);
        state.insert(pose_state(&id), StateValue::Pose(pose(at, [0.0; 3])));
        let args = AddPartArgs {
            file: file.into(),
            mates: BTreeMap::from([("m1".into(), mate)]),
            ..Default::default()
        };
        program.push(id, args);
    }
    program.state = state;
    program
}

/// The files of a robot of `modules` modules — a plate with 10 screws —
/// and `screws` screws of its own, on a plate: `11 * modules + screws + 1`
/// parts placed, of four programs. The top one is `robot.geop`.
fn robot(modules: usize, screws: usize) -> BTreeMap<String, String> {
    let screw = |k: usize, at: [f64; 3]| {
        (
            format!("screw{k}"),
            "screw.geop",
            "cylinder(screw,start)",
            at,
        )
    };
    let module = on_a_plate(
        (0..10)
            .map(|k| screw(k, [0.5 + k as f64 * 0.9, 0.5, 0.2]))
            .collect(),
    );
    let mut parts: Vec<_> = (0..modules)
        .map(|k| {
            let at = [(k % 4) as f64 * 11.0, (k / 4) as f64 * 11.0, 0.2];
            (
                format!("module{k}"),
                "module.geop",
                "/plate:block(plate,start)",
                at,
            )
        })
        .collect();
    parts
        .extend((0..screws).map(|k| screw(k, [(k % 20) as f64 * 0.5, (k / 20) as f64 * 0.5, 0.2])));
    [
        ("robot.geop", on_a_plate(parts)),
        ("module.geop", module),
        (
            "plate.geop",
            part("plate", PrimitiveArgs::block([0.0; 3], [10.0, 10.0, 0.2])),
        ),
        (
            "screw.geop",
            part("screw", PrimitiveArgs::cylinder_z([0.0; 3], 0.15, 1.0)),
        ),
    ]
    .into_iter()
    .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
    .collect()
}

/// The workspace of `files`, its cache `cache`.
fn workspace(files: &BTreeMap<String, String>, cache: Cache) -> Rc<dyn Workspace> {
    let operations = Rc::new(registry().operations());
    Files::with_cache(WithStandardParts(files.clone()), operations, cache)
}

/// A robot reopened makes none of what it kept: not the models of its
/// placed parts, nor their drawings, in the robot or in a module — and
/// builds the same robot. Each placed program is put together again of
/// what it kept, once for every distinct build.
#[test]
fn a_robot_reopened_makes_none_of_its_parts_again() {
    let files = robot(3, 5);
    let operations = Rc::new(registry().operations());
    let [first, again] = assert_reopens(operations, |cache| workspace(&files, cache), "robot.geop");
    assert!(first.kept_makes > 0);
    assert_eq!(again.kept_makes, 0);
    assert!(again.restored >= first.kept_makes, "{again:?} of {first:?}");
}

/// An editor opening the robot of `modules` modules and `screws` screws
/// with the cache `shelf` keeps: the faces it meshed, and the seconds it
/// took.
fn open(shelf: &Shelf, modules: usize, screws: usize) -> (usize, f64) {
    let mut files = robot(modules, screws);
    let robot = Program::from_json(&files.remove("robot.geop").unwrap()).unwrap();
    let start = Instant::now();
    let meshed = geop_core_rasterize::faces_meshed();
    let mut editor = Editor::with_cache(
        registry(),
        WithStandardParts(BTreeMap::new()),
        shelf.cache(),
    );
    let files = files.into_iter().map(|(p, t)| (p, Some(t))).collect();
    handled(&mut editor, Command::Files { files });
    let path = Some("robot.geop".into());
    handled(
        &mut editor,
        Command::Load {
            program: robot,
            path,
        },
    );
    (
        geop_core_rasterize::faces_meshed() - meshed,
        start.elapsed().as_secs_f64(),
    )
}

/// An editor reopening a robot another had open meshes no face.
#[test]
fn a_robot_reopened_in_an_editor_meshes_no_face() {
    let shelf = Shelf::of("kept");
    let (meshed, _) = open(&shelf, 2, 5);
    assert!(meshed > 0);
    let (meshed_again, _) = open(&shelf, 2, 5);
    assert_eq!(meshed_again, 0);
}

/// A robot is meshed once for every distinct part in it, however often each
/// is placed: four times the screws and modules, the same faces meshed.
#[test]
fn a_robot_is_meshed_once_for_every_distinct_part() {
    let (few, _) = open(&Shelf::of("few"), 1, 5);
    let (many, _) = open(&Shelf::of("many"), 4, 20);
    assert_eq!(few, many);
}

/// The robot of 500 and 2000 placed parts opened in an editor with no cache
/// and with the cache the first kept, timed, with the size of the cache.
#[test]
#[ignore = "slow: timings of robots of 500 and 2000 placed parts — run with `cargo test -- --ignored`"]
fn robot_reopen_timings() {
    for (modules, screws) in [(40, 60), (160, 240)] {
        let shelf = Shelf::of("kept");
        let parts = 11 * modules + screws + 1;
        let (meshed, cold) = open(&shelf, modules, screws);
        let (meshed_again, warm) = open(&shelf, modules, screws);
        let kept: usize = shelf.files.borrow().values().map(String::len).sum();
        println!(
            "robot of {parts} parts: open {cold:.3} s ({meshed} faces meshed), reopened {warm:.3} s \
             ({meshed_again} faces meshed), {kept} bytes kept"
        );
    }
}
