//! [`Editable`]: what an [`Operation`] is to an editor — how it is offered,
//! the arguments of a new step, the form a step is edited through and what
//! the user's events do to it — and [`Editors`], the operations an editor
//! offers, which also gives the program runner its registry.

use std::{any::Any, collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    operation::{ErasedOperation, Operation, args, saved},
    operation_registry::OperationRegistry,
    part::{Part, Reader},
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
    values::Design,
};
use serde::Serialize;

use crate::editing::{Event, Form, hit::Scene};

/// What an operation is to an editor: its short name and what it does —
/// what an editor offers it by — and where and how prominently the editor
/// offers it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct OperationInfo {
    pub label: &'static str,
    pub doc: &'static str,
    pub group: OperationGroup,
    pub tier: OperationTier,
    /// Not yet robust enough to offer by default: an editor's reduced mode
    /// leaves it out.
    pub experimental: bool,
    /// Whether its reference fields pick in the part as the step builds it,
    /// rather than as it was before: a placed part's mates pick elements of
    /// the part it places.
    pub picks_built: bool,
    /// The extensions of the files its steps read that are no programs —
    /// data a program keeps beside it, a STEP file: what a front end keeps
    /// in a workspace with the programs, and sends the editor.
    pub reads: &'static [&'static str],
    /// What it is shown as: SVG on a 20 × 20 grid, drawn in the current
    /// colour — strokes `currentColor`, unfilled unless it says otherwise —
    /// that a front end puts in an `<svg>` of its own. None to show its
    /// label.
    pub icon: Option<&'static str>,
}

impl OperationInfo {
    /// An operation offered by default, whose reference fields pick in the
    /// part as it was before the step.
    pub const fn new(
        label: &'static str,
        doc: &'static str,
        group: OperationGroup,
        tier: OperationTier,
    ) -> Self {
        Self {
            label,
            doc,
            group,
            tier,
            experimental: false,
            picks_built: false,
            reads: &[],
            icon: None,
        }
    }

    /// The same, but left out of an editor's reduced mode.
    pub const fn experimental(self) -> Self {
        Self {
            experimental: true,
            ..self
        }
    }

    /// The same, but picking in the part as the step builds it.
    pub const fn picks_built(self) -> Self {
        Self {
            picks_built: true,
            ..self
        }
    }

    /// The same, its steps reading files of the extensions `reads`.
    pub const fn reads(self, reads: &'static [&'static str]) -> Self {
        Self { reads, ..self }
    }

    /// The same, shown as `icon` (see [`OperationInfo::icon`]).
    pub const fn icon(self, icon: &'static str) -> Self {
        Self {
            icon: Some(icon),
            ..self
        }
    }
}

/// An operation as an editor lists it: the name its steps are stored
/// under, and its [`OperationInfo`].
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Listed {
    pub kind: &'static str,
    #[serde(flatten)]
    pub info: &'static OperationInfo,
}

/// What an operation works on, as an editor groups its operations: the
/// toolbar's sections, in this order — the order of a part's workflow.
/// Serialized as the group's name, as it is shown.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub enum OperationGroup {
    /// What a part is besides its shape: its parameters, and what it is
    /// called, made of and coloured.
    Part,
    /// Sketches, and the reference geometry they are drawn on.
    Sketch,
    /// What makes a solid of a sketch: extrusions, revolutions, sweeps and
    /// lofts.
    Solid,
    /// Features that change a solid: holes, rounds, walls, threads.
    Features,
    /// Whole bodies: patterned, mirrored, combined, cut, moved.
    #[serde(rename = "Pattern & bodies")]
    Bodies,
    /// Faces standing on their own, and freeform shapes.
    Surface,
    /// Bodies of bent sheet, and their flat patterns.
    #[serde(rename = "Sheet metal")]
    SheetMetal,
    /// Placed parts, and what runs between them.
    Assembly,
    /// What is made of a part rather than in it: drawings, exports.
    Output,
}

/// How prominently an editor offers an operation within its group, from
/// the few used most to the rarely used: the order the group lists them in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub enum OperationTier {
    /// One of the few used most: a big button.
    Big,
    /// Used often: a small button.
    Small,
    /// Used rarely: only in its group's menu.
    Menu,
}

/// What a step is edited against: its id — what everything it builds is
/// named after — the runner it is run by, which reads the files around its
/// program, the part as it was before it, the part as the step built it
/// when it last ran, if it did, and, for an event, the part as the editor
/// draws it — what a pointer is over, without drawing it again.
pub struct Context<'a> {
    pub step: &'a str,
    pub runner: &'a ProgramRunner,
    pub before: Reader<'a>,
    pub built: Option<Reader<'a>>,
    /// The part as drawn for picks (see [`Scene`]): given with every event
    /// an editor passes on, and nothing else.
    pub scene: Option<&'a dyn Scene<Design>>,
}

/// An [`Operation`] an editor offers: everything it keeps while a step is
/// edited is its editing state, and everything it shows the step's form.
pub trait Editable: Operation {
    const INFO: OperationInfo;
    /// What the editor keeps about a step while it is edited, beyond its
    /// arguments, starting from the default whenever a step is opened.
    type EditingState: Default + 'static;

    /// The arguments of a new step: sensible defaults, taking the part it
    /// is inserted into into account.
    fn new_args(context: &Context) -> Self::Args;

    /// What the editor shows while the step is edited. Never fails: an
    /// editor needs the form most exactly when the arguments do not build,
    /// so whatever is wrong is said in it.
    fn presentation(
        context: &Context,
        args: &Self::Args,
        editing: &Self::EditingState,
    ) -> Form<Design>;

    /// The step's arguments and editing state after `event`.
    fn process_user_event(
        context: &Context,
        args: &Self::Args,
        editing: &Self::EditingState,
        event: Event<Design>,
    ) -> (Self::Args, Self::EditingState);
}

/// A step's arguments, as saved, and its editing state after an event.
pub struct Edited {
    pub args: serde_json::Value,
    pub editing: Box<dyn Any>,
}

/// An [`Editable`] with its types erased, so that operations of all kinds
/// can be kept in one [`Editors`]. Arguments come as saved.
pub trait ErasedEditable: ErasedOperation {
    fn info(&self) -> &'static OperationInfo;
    fn new_args(&self, context: &Context) -> GeopResult<serde_json::Value>;
    fn initial_editing(&self) -> Box<dyn Any>;
    fn presentation(
        &self,
        context: &Context,
        args: &serde_json::Value,
        editing: &dyn Any,
    ) -> GeopResult<Form<Design>>;
    fn process_user_event(
        &self,
        context: &Context,
        args: &serde_json::Value,
        editing: &dyn Any,
        event: Event<Design>,
    ) -> GeopResult<Edited>;
}

fn editing<O: Editable>(editing: &dyn Any) -> GeopResult<&O::EditingState> {
    editing.downcast_ref().ok_or_else(|| {
        GeopError::new(format!(
            "operation `{}` was given an editing state that is not `{}`",
            O::KIND,
            std::any::type_name::<O::EditingState>()
        ))
    })
}

impl<O: Editable> ErasedEditable for O {
    fn info(&self) -> &'static OperationInfo {
        &O::INFO
    }

    fn new_args(&self, context: &Context) -> GeopResult<serde_json::Value> {
        saved::<O>(&<O as Editable>::new_args(context))
    }

    fn initial_editing(&self) -> Box<dyn Any> {
        Box::new(O::EditingState::default())
    }

    fn presentation(
        &self,
        context: &Context,
        args: &serde_json::Value,
        editing: &dyn Any,
    ) -> GeopResult<Form<Design>> {
        Ok(<O as Editable>::presentation(
            context,
            &self::args::<O>(args)?,
            self::editing::<O>(editing)?,
        ))
    }

    fn process_user_event(
        &self,
        context: &Context,
        args: &serde_json::Value,
        editing: &dyn Any,
        event: Event<Design>,
    ) -> GeopResult<Edited> {
        let (args, editing) = <O as Editable>::process_user_event(
            context,
            &self::args::<O>(args)?,
            self::editing::<O>(editing)?,
            event,
        );
        Ok(Edited {
            args: saved::<O>(&args)?,
            editing: Box::new(editing),
        })
    }
}

/// The operations an editor offers, by name — and, for the runner its
/// programs run on, the same as an [`OperationRegistry`]
/// ([`Editors::operations`]).
#[derive(Default)]
pub struct Editors(BTreeMap<&'static str, Rc<dyn ErasedEditable>>);

/// The position of the step `step` in `program`.
fn position(program: &Program, step: &str) -> GeopResult<usize> {
    (program.steps.iter().position(|s| s.id == step))
        .ok_or_else(|| GeopError::new(format!("step `{step}` is not in the program")))
}

/// A step being edited: its arguments as saved, its operation, and what
/// it is edited against.
struct Step<'a> {
    args: &'a serde_json::Value,
    operation: &'a dyn ErasedEditable,
    context: Context<'a>,
}

impl Editors {
    /// Adds `operation`. Fails if an operation of its name is already there.
    pub fn with<O: Editable>(mut self, operation: O) -> GeopResult<Self> {
        if self.0.contains_key(O::KIND) {
            return Err(GeopError::new(format!(
                "operation `{}` is registered twice",
                O::KIND
            )));
        }
        self.0.insert(O::KIND, Rc::new(operation));
        Ok(self)
    }

    /// The operations, as a program runner runs them.
    pub fn operations(&self) -> GeopResult<OperationRegistry> {
        let mut registry = OperationRegistry::default();
        for operation in self.0.values() {
            let operation: Rc<dyn ErasedOperation> = operation.clone();
            registry = registry.with_erased(operation)?;
        }
        Ok(registry)
    }

    /// What each operation is, in the order of their names.
    pub fn infos(&self) -> Vec<Listed> {
        (self.0.iter())
            .map(|(&kind, operation)| Listed {
                kind,
                info: operation.info(),
            })
            .collect()
    }

    fn get(&self, kind: &str) -> GeopResult<&dyn ErasedEditable> {
        self.0
            .get(kind)
            .map(Rc::as_ref)
            .ok_or_else(|| GeopError::new(format!("unknown operation `{kind}`")))
    }

    /// The step `step` of `program` and its operation, and what it is
    /// edited against in `part`, as `runner` last ran it: the part before
    /// it and after it, at its position in `program`. A program edited since
    /// the last run is read at its new positions until it runs again.
    fn step<'a>(
        &'a self,
        runner: &'a ProgramRunner,
        part: &'a Part,
        program: &'a Program,
        step: &'a str,
    ) -> GeopResult<Step<'a>> {
        let program_step = program.step(step)?;
        let position = position(program, step)?;
        let context = Context {
            step,
            runner,
            before: part.reader(position),
            built: Some(part.reader(position + 1)),
            scene: None,
        };
        Ok(Step {
            args: &program_step.args,
            operation: self.get(&program_step.operation)?,
            context,
        })
    }

    /// A new step of the operation `kind`, `id`, to insert into `program`
    /// before its step `before`, or after its last.
    pub fn new_step(
        &self,
        runner: &ProgramRunner,
        part: &Part,
        program: &Program,
        kind: &str,
        id: &str,
        before: Option<&str>,
    ) -> GeopResult<ProgramStep> {
        let operation = self.get(kind)?;
        let before = match before {
            Some(step) => part.reader(position(program, step)?),
            None => part.current(),
        };
        let context = Context {
            step: id,
            runner,
            before,
            built: None,
            scene: None,
        };
        Ok(ProgramStep {
            id: id.to_string(),
            operation: operation.kind().to_string(),
            args: operation.new_args(&context)?,
        })
    }

    /// The editing state the step `step` starts with.
    pub fn initial_editing(&self, program: &Program, step: &str) -> GeopResult<Box<dyn Any>> {
        Ok(self.get(&program.step(step)?.operation)?.initial_editing())
    }

    /// What the editor shows while the step `step` is edited.
    pub fn presentation(
        &self,
        runner: &ProgramRunner,
        part: &Part,
        program: &Program,
        step: &str,
        editing: &dyn Any,
    ) -> GeopResult<Form<Design>> {
        let step = self.step(runner, part, program, step)?;
        (step.operation).presentation(&step.context, step.args, editing)
    }

    /// What a list of steps shows of the step `step`: its form, as a step
    /// just opened shows it, against the part before it — and never what
    /// it built, so that a list of every step asks nothing of what each
    /// built.
    pub fn listed(
        &self,
        runner: &ProgramRunner,
        part: &Part,
        program: &Program,
        step: &str,
    ) -> GeopResult<Form<Design>> {
        let mut step = self.step(runner, part, program, step)?;
        step.context.built = None;
        let editing = step.operation.initial_editing();
        (step.operation).presentation(&step.context, step.args, &*editing)
    }

    /// The arguments and editing state of the step `step` after `event`,
    /// with what the pointer is over found in `scene`, if one is given.
    #[allow(clippy::too_many_arguments)]
    pub fn process_user_event(
        &self,
        runner: &ProgramRunner,
        part: &Part,
        program: &Program,
        step: &str,
        editing: &dyn Any,
        event: Event<Design>,
        scene: Option<&dyn Scene<Design>>,
    ) -> GeopResult<Edited> {
        let mut step = self.step(runner, part, program, step)?;
        step.context.scene = scene;
        (step.operation).process_user_event(&step.context, step.args, editing, event)
    }
}
