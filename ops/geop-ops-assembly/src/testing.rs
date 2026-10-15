//! What the tests of what reads placed parts share: parts made in Rust as
//! the programs of a workspace ([`Shelf`]), each giving its model
//! ([`GivenModel`]) and placing others ([`Place`]).

use geop_core_program::{
    operation::Operation,
    operation_registry::OperationRegistry,
    part::StepContext,
    program::{Program, ProgramStep},
    runner::Runner,
    values::{Design, Settings, pose_state},
    workspace::{Location, Workspace},
};
use std::{cell::RefCell, collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_ops_model::{NamedModel, SolidNotes, model, solid_notes};
use geop_ops_parameters::{DefineParameters, LookupTable, PartProperties, Placement};
use serde::{Deserialize, Serialize};

use crate::placed::define_placed;

thread_local! {
    /// The models [`given_model`] keeps, with the notes on their solids.
    static MODELS: RefCell<Vec<(NamedModel<Design>, Option<SolidNotes>)>> =
        const { RefCell::new(Vec::new()) };
}

/// Gives the part a model made in Rust, and the notes on its solids: one
/// [`given_model`] kept.
pub struct GivenModel;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GivenModelArgs {
    key: usize,
}

impl Operation for GivenModel {
    const KIND: &'static str = "given_model";
    type Args = GivenModelArgs;

    fn run(ctx: &mut StepContext, args: &GivenModelArgs) -> GeopResult<()> {
        let (given, notes) = (MODELS.with(|models| models.borrow().get(args.key).cloned()))
            .ok_or_else(|| GeopError::new(format!("no model {} was given", args.key)))?;
        ctx.produce(&model(), &given, |_, given| Ok(given.clone()))?;
        if let Some(notes) = &notes {
            ctx.produce(&solid_notes(), notes, |_, notes| Ok(notes.clone()))?;
        }
        Ok(())
    }
}

/// The step `id` giving the part `model`, and `notes` on its solids.
pub fn given_model(id: &str, model: NamedModel<Design>, notes: Option<SolidNotes>) -> ProgramStep {
    let key = MODELS.with(|models| {
        let mut models = models.borrow_mut();
        models.push((model, notes));
        models.len() - 1
    });
    ProgramStep::new::<GivenModel>(id, &GivenModelArgs { key }).expect("plain arguments")
}

/// Places the part the program in `file` builds, its parameters set to
/// `inputs`, where its pose state puts it: what a placed part is, without
/// mates.
pub struct Place;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlaceArgs {
    file: String,
    inputs: Settings,
}

impl Operation for Place {
    const KIND: &'static str = "place";
    type Args = PlaceArgs;

    fn run(ctx: &mut StepContext, args: &PlaceArgs) -> GeopResult<()> {
        let step = ctx.step();
        let placement = Placement::State(pose_state(step));
        define_placed(ctx, step, &args.file, &args.inputs, placement)
    }
}

/// The step `id` placing the part of `file`, its parameters set to
/// `inputs`.
pub fn place(id: &str, file: &str, inputs: &Settings) -> ProgramStep {
    let args = PlaceArgs {
        file: file.into(),
        inputs: inputs.clone(),
    };
    ProgramStep::new::<Place>(id, &args).expect("plain arguments")
}

/// Programs by file, as a workspace holds them: the parts a test places.
#[derive(Default)]
pub struct Shelf(BTreeMap<String, Rc<Program>>);

impl Shelf {
    /// It with `program` as the file `file`.
    pub fn with(mut self, file: &str, program: Program) -> Self {
        self.0.insert(file.to_string(), Rc::new(program));
        self
    }
}

impl Workspace for Shelf {
    fn program(&self, path: &str) -> GeopResult<Rc<Program>> {
        (self.0.get(path).cloned()).ok_or_else(|| GeopError::new(format!("there is no {path:?}")))
    }

    fn read(&self, path: &str) -> GeopResult<Rc<str>> {
        Err(GeopError::new(format!("{path:?} is no file to read")))
    }

    fn list(&self) -> Vec<String> {
        self.0.keys().cloned().collect()
    }

    fn resolve(&self, _: &str, name: &str) -> String {
        name.to_string()
    }

    fn relative(&self, _: &str, path: &str) -> String {
        path.to_string()
    }

    /// A shelf does not change: one revision.
    fn revision(&self, _: &str) -> u64 {
        0
    }
}

/// The operations of the programs on a shelf: the parameters and
/// properties steps, a model given and a part placed.
pub fn registry() -> OperationRegistry {
    (OperationRegistry::default().with(DefineParameters))
        .and_then(|r| r.with(LookupTable))
        .and_then(|r| r.with(PartProperties))
        .and_then(|r| r.with(GivenModel))
        .and_then(|r| r.with(Place))
        .expect("each has a name of its own")
}

/// A runner of [`registry`]'s operations, reading `shelf`, running the
/// program of the file `file`.
pub fn runner(shelf: Shelf, file: &str) -> Runner {
    let mut runner = Runner::new(registry());
    runner.workspace = Some(Rc::new(shelf));
    runner.location = Location {
        file: file.to_string(),
        within: Vec::new(),
    };
    runner
}
