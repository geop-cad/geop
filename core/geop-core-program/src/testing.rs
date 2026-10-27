//! What tests of operations need of a program (feature `testing`): its
//! steps and their arguments as their types, a runner of it, and that it
//! builds the same incrementally and after a round trip through its file.
#![allow(
    clippy::panic,
    clippy::expect_used,
    clippy::indexing_slicing,
    reason = "a test fails by panicking"
)]

use std::rc::Rc;

use serde::{Serialize, de::DeserializeOwned};

use crate::{
    operation::StepArgs,
    operation_registry::OperationRegistry,
    part::Part,
    program::{Program, ProgramStep},
    runner::{RunOutput, Runner},
    workspace::{Location, Workspace},
};

// ── Steps and their arguments ───────────────────────────────────────────────

/// The step `id`, `args` for the operation they are for.
pub fn as_step<A: StepArgs>(id: &str, args: A) -> ProgramStep {
    ProgramStep::of(id, &args).expect("a step's arguments are plain data")
}

/// The step `id` of `program`.
#[track_caller]
pub fn step<'a>(program: &'a mut Program, id: &str) -> &'a mut ProgramStep {
    let index = program.index_of(id).unwrap_or_else(|e| panic!("{e}"));
    &mut program.steps[index]
}

/// The arguments of `step`, as their type `A`.
#[track_caller]
pub fn args<A: DeserializeOwned>(step: &ProgramStep) -> A {
    serde_json::from_value(step.args.clone())
        .unwrap_or_else(|e| panic!("step {:?} holds no such arguments: {e}", step.id))
}

/// Changes the arguments of `step`, as their type `A`, by `edit`.
pub fn edit_args<A: Serialize + DeserializeOwned>(
    step: &mut ProgramStep,
    edit: impl FnOnce(&mut A),
) {
    let mut args: A = args(step);
    edit(&mut args);
    step.args = serde_json::to_value(args).expect("arguments are plain data");
}

// ── Running programs ────────────────────────────────────────────────────────

/// A runner of the operations of `registry`, running the file `file` of
/// `workspace`, if one is given.
pub fn runner(
    registry: Rc<OperationRegistry>,
    workspace: Option<(Rc<dyn Workspace>, &str)>,
) -> Runner {
    let location = Location {
        file: (workspace.as_ref()).map_or_else(String::new, |(_, file)| file.to_string()),
        within: Vec::new(),
    };
    Runner::of(
        registry,
        workspace.map(|(workspace, _)| workspace),
        location,
    )
}

/// The failed steps of `output`, and why.
fn errors(output: RunOutput) -> Vec<(String, String)> {
    (output.step_errors.into_iter())
        .map(|(step, e)| (step, e.to_string()))
        .collect()
}

/// Runs `programs` one after another on one part, as an editor runs a
/// program edited again and again, and each on a fresh part by a fresh
/// runner, from scratch — its placed parts built anew, not taken from the
/// first runner's build cache; and asserts that after each the two parts
/// hold equal nodes of every target (see [`Part::differences`]), and that
/// their steps fail alike. A node reused for reading what did not change
/// would otherwise show here as a target that differs. The programs are
/// run in the operations of `registry`, as the file `file` of `workspace`,
/// if one is given.
#[track_caller]
pub fn assert_incremental(
    registry: Rc<OperationRegistry>,
    workspace: Option<(Rc<dyn Workspace>, &str)>,
    programs: &[Program],
) {
    let file = workspace.as_ref().map(|(_, file)| file.to_string());
    let incremental = runner(registry.clone(), workspace.clone());
    let mut part = Part::default();
    for (k, program) in programs.iter().enumerate() {
        let ran = errors(incremental.run(&mut part, program, None));
        let mut scratch = Part::default();
        let built =
            errors(runner(registry.clone(), workspace.clone()).run(&mut scratch, program, None));
        assert_eq!(
            ran, built,
            "{file:?}, program {k}: its steps fail otherwise when run again"
        );
        let differences = part.differences(&scratch);
        assert!(
            differences.is_empty(),
            "{file:?}, program {k}: run again, it builds otherwise than from scratch: \
             {differences:?}"
        );
    }
}

/// Builds `program`, every step without an error, saves it, loads it back
/// — the same program, saved the same again — and builds what it loaded,
/// by a fresh runner: asserts that the two parts hold equal nodes of every
/// target (see [`Part::differences`]), and returns the first. The program
/// is run in the operations of `registry`, as the file `file` of
/// `workspace`, if one is given.
#[track_caller]
pub fn assert_round_trips(
    registry: Rc<OperationRegistry>,
    workspace: Option<(Rc<dyn Workspace>, &str)>,
    program: &Program,
) -> Part {
    let file = workspace.as_ref().map(|(_, file)| file.to_string());
    let build = |program: &Program| {
        let mut part = Part::default();
        let failed =
            errors(runner(registry.clone(), workspace.clone()).run(&mut part, program, None));
        assert!(failed.is_empty(), "{file:?}: steps fail: {failed:?}");
        part
    };
    let built = build(program);
    let json = program
        .to_json()
        .unwrap_or_else(|e| panic!("{file:?}: not saved: {e}"));
    let loaded = Program::from_json(&json).unwrap_or_else(|e| panic!("{file:?}: not loaded: {e}"));
    assert_eq!(&loaded, program, "{file:?}: loaded, it is another program");
    let again = loaded
        .to_json()
        .unwrap_or_else(|e| panic!("{file:?}: not saved again: {e}"));
    assert_eq!(again, json, "{file:?}: saved again, it reads otherwise");
    let differences = built.differences(&build(&loaded));
    assert!(
        differences.is_empty(),
        "{file:?}: loaded, it builds otherwise: {differences:?}"
    );
    built
}

/// Edits of `original` to run [`assert_incremental`] through, as a user
/// makes them: as it is, its last step dropped, put back, its first two
/// steps swapped — a failure, where the second reads the first — and put
/// back.
pub fn edit_sequence(original: &Program) -> Vec<Program> {
    let mut programs = vec![original.clone()];
    if let Some(last) = original.steps.len().checked_sub(1) {
        let mut dropped = original.clone();
        dropped.steps.remove(last);
        programs.extend([dropped, original.clone()]);
    }
    if original.steps.len() >= 2 {
        let mut swapped = original.clone();
        swapped.steps.swap(0, 1);
        programs.extend([swapped, original.clone()]);
    }
    programs
}

// ── Keeping builds between processes ────────────────────────────────────────

/// Cache files in memory, as a store on disk keeps them, of the build
/// `build`: what a test of what one process keeps for the next shares
/// between the two (see [`crate::cache`], [`crate::kept`]).
#[derive(Clone, Default)]
pub struct Shelf {
    pub build: String,
    pub files: Rc<std::cell::RefCell<std::collections::BTreeMap<String, String>>>,
}

impl Shelf {
    /// An empty shelf of the build `build`.
    pub fn of(build: &str) -> Self {
        Self {
            build: build.to_string(),
            ..Self::default()
        }
    }

    /// A cache kept on it.
    pub fn cache(&self) -> crate::cache::Cache {
        crate::cache::Cache::stored(Box::new(self.clone()))
    }
}

impl crate::cache::CacheStore for Shelf {
    fn build(&self) -> &str {
        &self.build
    }

    fn read(&self, path: &str) -> Option<String> {
        self.files.borrow().get(path).cloned()
    }

    fn write(&self, path: &str, text: &str) {
        (self.files.borrow_mut()).insert(path.to_string(), text.to_string());
    }
}

/// Builds the program in the file `file` of the workspace `workspace` makes
/// of a cache, in the operations of `registry`, in one process — its cache
/// kept on a shelf — then in another, from what the first kept (see
/// [`crate::kept`]): asserts that the second makes none of the targets kept
/// between processes, and builds what the first built (see
/// [`Part::differences`]). Returns what each process did.
#[track_caller]
pub fn assert_reopens(
    registry: Rc<OperationRegistry>,
    workspace: impl Fn(crate::cache::Cache) -> Rc<dyn Workspace>,
    file: &str,
) -> [crate::runner::Counts; 2] {
    let shelf = Shelf::of("kept");
    let process = || {
        let workspace = workspace(shelf.cache());
        let program = (workspace.program(file)).unwrap_or_else(|e| panic!("{file}: {e}"));
        let runner = runner(registry.clone(), Some((workspace.clone(), file)));
        let mut part = Part::default();
        let failed = errors(runner.run(&mut part, &program, None));
        if let Some(cache) = workspace.cache() {
            cache.flush();
        }
        (runner.counts(), part, failed)
    };
    let (first, built, failed) = process();
    let kept = shelf.files.borrow().clone();
    let (second, reopened, failed_again) = process();
    assert_eq!(
        failed, failed_again,
        "{file}: reopened, its steps fail otherwise"
    );
    assert_eq!(
        second.kept_makes,
        0,
        "{file}: reopened, it makes again what it kept: {second:?}, first {first:?}: {:#?}",
        not_kept(&reopened, file, &kept, &shelf.build)
    );
    let differences = reopened.differences(&built);
    assert!(
        differences.is_empty(),
        "{file}: reopened, it builds otherwise: {differences:?}"
    );
    [first, second]
}

/// Every node of a kept type of `part`, built from the file `file` — and of
/// the parts placed in it, from theirs — that the cache files `kept`, of
/// the build `build`, keep no record of, and why: the reads in which the
/// records of its target with its arguments differ from how it was made.
fn not_kept(
    part: &Part,
    file: &str,
    kept: &std::collections::BTreeMap<String, String>,
    build: &str,
) -> Vec<String> {
    let mut found = Vec::new();
    let mut parts = vec![(file.to_string(), part)];
    while let Some((path, part)) = parts.pop() {
        for (_, _, placed) in part.placed() {
            parts.push((placed.location.file.clone(), &placed.part));
        }
        let text = kept.get(&crate::cache::cache_path(&path));
        let builds = crate::kept::Builds::read(text.map(String::as_str), build);
        for (name, node) in part.kept_nodes() {
            let Some((derivation, digest)) = &node.kept else {
                continue;
            };
            let records = builds.records(&node.step, name);
            if records
                .iter()
                .any(|r| r.digest == *digest && r.derivation == **derivation)
            {
                continue;
            }
            let differing: Vec<String> = (records.iter())
                .filter(|r| r.derivation.args == derivation.args)
                .flat_map(|r| {
                    (r.derivation.reads.iter())
                        .filter(|read| !derivation.reads.contains(read))
                        .map(|(read, _)| format!("{read:?}"))
                })
                .collect();
            found.push(format!(
                "{path}: `{name}` of step `{}`: {} records, {} with its arguments, \
                 reading otherwise: {differing:?}; it read {:?}",
                node.step,
                records.len(),
                (records.iter())
                    .filter(|r| r.derivation.args == derivation.args)
                    .count(),
                derivation.reads,
            ));
        }
    }
    found
}
