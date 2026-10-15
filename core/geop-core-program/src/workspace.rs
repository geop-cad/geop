//! [`Workspace`]: where the programs a program places, and the files its
//! steps read, come from — the one thing a run reads that is not given to
//! it as inputs or state, and not built by its steps.
//!
//! A program never knows where it is. The runner running it does
//! ([`Location`]): the file it is in, and the files placing it. A step
//! reads through the runner ([`crate::program_runner::ProgramRunner`]), in
//! its body, which runs in every run, and passes what it read to the
//! targets it defines as arguments: a target is made again exactly when
//! what it read changed.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::GeopResult;

use crate::program::Program;

/// Programs and other files, by path.
pub trait Workspace<S> {
    /// The program in the file at `path`, and the state it was saved with:
    /// the same each time it is asked for while the file does not change.
    fn program(&self, path: &str) -> GeopResult<Loaded<S>>;

    /// The text of the file at `path`: a STEP file a step imports. The same
    /// each time it is asked for while the file does not change.
    fn read(&self, path: &str) -> GeopResult<Rc<str>>;

    /// Every file there is, by path.
    fn list(&self) -> Vec<String>;

    /// The path of the file the file at `from` names `name`.
    fn resolve(&self, from: &str, name: &str) -> String;

    /// How the file at `from` names the file at `path`.
    fn relative(&self, from: &str, path: &str) -> String;

    /// How many times a file changed: what a part reading files, carried
    /// over from its last run, would read otherwise once this does.
    fn revision(&self) -> u64;
}

/// A program as a file holds it: its steps, and the state it was saved
/// with — what it is run with where the part placing it gives none.
pub struct Loaded<S> {
    pub program: Rc<Program>,
    pub state: Rc<BTreeMap<String, S>>,
}

// By hand, since a derive would ask the same of `S`.
impl<S> Clone for Loaded<S> {
    fn clone(&self) -> Self {
        Self {
            program: self.program.clone(),
            state: self.state.clone(),
        }
    }
}

/// Where the program a runner runs is: the file it is in, and the files
/// placing it, outermost first — which it must not place again, or the
/// files would place themselves. The runner's to know, never the
/// program's.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Location {
    pub file: String,
    pub within: Vec<String>,
}
