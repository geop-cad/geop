//! Builds kept between processes: a process reopening a program another
//! built, with the cache file it left, makes none of what was kept, unless
//! what it was made from changed — a step, an input, a file it read.

use std::{collections::BTreeMap, rc::Rc};

use crate::{
    part::Part,
    program::{Program, ProgramStep},
    runner::{Counts, Runner},
    test::{
        Running, add_two_datums, add_two_frame, frame, input_frame, inputs, instances::load,
        middle_datum, middle_frame, no_state, ok, registry, shift_frame,
    },
    testing::Shelf,
    values::Settings,
    workspace::{Files, Location, Workspace},
};

/// The file of the program every test here builds.
const FILE: &str = "part.geop";

/// A process of the kernel, building the program in [`FILE`] of `files`
/// with the cache kept on `shelf`.
fn process(shelf: &Shelf, files: &BTreeMap<String, String>) -> (Rc<Files>, Running) {
    let registry = Rc::new(registry());
    let workspace = Files::with_cache(files.clone(), registry.clone(), shelf.cache());
    let location = Location {
        file: FILE.to_string(),
        within: Vec::new(),
    };
    let runner = Runner::of(registry, Some(workspace.clone()), location);
    let running = Running {
        runner,
        part: Part::default(),
    };
    (workspace, running)
}

/// Builds the program in [`FILE`] in a fresh process, with `inputs`, and
/// writes what it kept: what it did, and the part.
fn build(shelf: &Shelf, files: &BTreeMap<String, String>, inputs: &Settings) -> (Counts, Running) {
    let (workspace, mut running) = process(shelf, files);
    let program = workspace.program(FILE).unwrap();
    ok(running.run(&program, inputs, &no_state()));
    workspace.cache().unwrap().flush();
    (running.runner.counts(), running)
}

/// The files of a workspace whose program [`FILE`] is `steps`.
fn files(steps: Vec<ProgramStep>) -> BTreeMap<String, String> {
    let program = Program::from_steps(steps).to_json().unwrap();
    BTreeMap::from([(FILE.to_string(), program)])
}

/// Two frames, one halfway between them, moved, and datums besides, which
/// are not kept: what reads them is made from what another process made
/// again.
fn frames(shift: f64) -> Vec<ProgramStep> {
    vec![
        add_two_frame("a", [0.0; 3], [0.0, 0.0, 2.0]),
        add_two_datums("d", [0.0; 3], [4.0, 0.0, 0.0]),
        middle_datum("dm", "d_1", "d_2"),
        middle_frame("m", "a_1", "a_2"),
        shift_frame("s", "m", shift),
    ]
}

/// What one process kept is read back by the next: it makes none of the
/// frames — the datums, which are not kept, it makes again — and builds
/// the same part.
#[test]
fn reopening_a_program_makes_nothing_it_kept() {
    let shelf = Shelf::of("1");
    let files = files(frames(1.0));
    let (first, built) = build(&shelf, &files, &inputs(&[]));
    // `a_1`, `a_2`, `m`, and `m` moved.
    assert_eq!(first.kept_makes, 4);
    assert!(shelf.files.borrow().contains_key("part.geop.geopc"));

    let (again, rebuilt) = build(&shelf, &files, &inputs(&[]));
    assert_eq!((again.kept_makes, again.restored), (0, 4), "{again:?}");
    assert_eq!(again.makes, first.makes - first.kept_makes);
    assert_eq!(rebuilt.part.differences(&built.part), Vec::<String>::new());
    assert_eq!(frame(&rebuilt, "m"), frame(&built, "m"));
}

/// A step edited makes again what it makes, and what reads that; what the
/// steps before it made is read back.
#[test]
fn an_edited_step_makes_again_only_what_it_changed() {
    let shelf = Shelf::of("1");
    build(&shelf, &files(frames(1.0)), &inputs(&[]));
    let (edited, _) = build(&shelf, &files(frames(2.0)), &inputs(&[]));
    assert_eq!((edited.kept_makes, edited.restored), (1, 3), "{edited:?}");
    // Both are kept now: going back makes nothing.
    let (back, _) = build(&shelf, &files(frames(1.0)), &inputs(&[]));
    assert_eq!((back.kept_makes, back.restored), (0, 4), "{back:?}");
}

/// What another build of the kernel kept may have been made by other code:
/// it is made again.
#[test]
fn another_build_of_the_kernel_makes_everything_again() {
    let shelf = Shelf::of("1");
    let files = files(frames(1.0));
    build(&shelf, &files, &inputs(&[]));
    let other = Shelf {
        build: "2".to_string(),
        ..shelf.clone()
    };
    let (again, _) = build(&other, &files, &inputs(&[]));
    assert_eq!((again.kept_makes, again.restored), (4, 0), "{again:?}");
}

/// A node that read an input is made again when the input is other, and
/// read back when it is as it was.
#[test]
fn a_kept_node_is_checked_against_the_inputs_it_read() {
    let shelf = Shelf::of("1");
    let files = files(vec![input_frame("i", "x", "w"), shift_frame("s", "x", 1.0)]);
    build(&shelf, &files, &inputs(&[("w", 1.0)]));
    let (other, _) = build(&shelf, &files, &inputs(&[("w", 2.0)]));
    assert_eq!((other.kept_makes, other.restored), (2, 0), "{other:?}");
    let (same, _) = build(&shelf, &files, &inputs(&[("w", 1.0)]));
    assert_eq!((same.kept_makes, same.restored), (0, 2), "{same:?}");
}

/// A part placed from another file is kept in that file's cache file: a
/// process reopening the program places it without making any of it, and
/// one whose placed file changed makes again what changed in it, and what
/// reads that.
#[test]
fn a_placed_part_is_kept_beside_its_own_file() {
    let shelf = Shelf::of("1");
    let placing = |bolt: [f64; 3]| {
        let mut files = files(vec![
            load("bolt", vec![add_two_frame("a", bolt, [0.0, 0.0, 2.0])]),
            middle_frame("m", "/bolt:a_1", "/bolt:a_2"),
        ]);
        let bolt = vec![add_two_frame("a", bolt, [0.0, 0.0, 2.0])];
        files.insert(
            "bolt.geop".into(),
            Program::from_steps(bolt).to_json().unwrap(),
        );
        files
    };
    let (first, _) = build(&shelf, &placing([0.0; 3]), &inputs(&[]));
    assert_eq!(first.kept_makes, 3);
    assert!(shelf.files.borrow().contains_key("bolt.geop.geopc"));

    let (again, _) = build(&shelf, &placing([0.0; 3]), &inputs(&[]));
    assert_eq!((again.kept_makes, again.restored), (0, 3), "{again:?}");
    // The placed part itself is put together again, of what it kept.
    assert_eq!((again.placed, again.builds), (1, 1));

    let (moved, rebuilt) = build(&shelf, &placing([2.0, 0.0, 0.0]), &inputs(&[]));
    // The bolt's first frame, and the frame halfway that reads it.
    assert_eq!((moved.kept_makes, moved.restored), (2, 1), "{moved:?}");
    crate::test::assert_encloses(frame(&rebuilt, "m"), [1.0, 0.0, 1.0]);
}
