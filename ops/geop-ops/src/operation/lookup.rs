//! [`Lookup`]: what an entity reference is resolved against — the model a
//! step builds on, and the targets it reads besides, sketches, 3-D sketches
//! and parameters — and the places it is resolved in: a step's generator,
//! which records every read so the step is rebuilt when one changes
//! ([`Scope`]), an editor showing a step, which builds nothing
//! ([`Seen`]), and a model on its own.

use std::{borrow::Cow, rc::Rc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Datum, Pose},
    scalars::Scalar,
};
use geop_core_program::part::{Reader, Retrieve, View};

use geop_core_sketch::space::Sketch3d;

use crate::{
    Design,
    model::{
        NamedModel, ParamValue, PlacedSketch, PlacedSketch3d, datum, datum_name, model_or_empty,
        sketch, sketch_name, sketch3d, sketch3d_name,
    },
    operation::EntityRef,
    placed::{About, Library, locate, within},
};

/// What a run is given from outside: the values of the program's parameters
/// by their names, the [`Library`] it places parts from (see
/// [`crate::placed::LIBRARY`]), and what the program says of its part (see
/// [`crate::placed::ABOUT`]).
#[derive(Clone)]
pub enum Input {
    Value(ParamValue),
    Library(Rc<dyn Library>),
    About(Rc<About>),
}

/// Values by value, a library by identity: a library given anew counts as
/// changed — what it builds may differ.
impl PartialEq for Input {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Input::Value(a), Input::Value(b)) => a == b,
            (Input::Library(a), Input::Library(b)) => Rc::ptr_eq(a, b),
            (Input::About(a), Input::About(b)) => a == b,
            _ => false,
        }
    }
}

impl std::fmt::Debug for Input {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Input::Value(value) => write!(f, "{value:?}"),
            Input::Library(_) => write!(f, "Library"),
            Input::About(about) => write!(f, "{about:?}"),
        }
    }
}

/// The part a program builds, given [`Input`]s, with a state of
/// [`ParamValue`]s.
pub type Build = geop_core_program::part::Part<Input, ParamValue>;

/// The part as an editor sees it before or after a step.
pub type BuildView<'a> = View<'a, Input, ParamValue>;

/// What a step's generator reads the part through.
pub type BuildReader<'a> = Reader<'a, Input, ParamValue>;

/// The value of the parameter `input`, if it is one.
fn value(input: Option<Input>) -> Option<ParamValue> {
    match input? {
        Input::Value(value) => Some(value),
        Input::Library(_) | Input::About(_) => None,
    }
}

/// What entity references are resolved against.
pub trait Lookup<S: Scalar> {
    /// The model as the step builds on it.
    fn model(&self) -> &NamedModel<S>;

    /// The sketch `name`.
    fn sketch(&mut self, name: &str) -> GeopResult<&PlacedSketch<S>>;

    /// The 3-D sketch `name` (see [`sketch3d`]).
    fn sketch3d(&mut self, name: &str) -> GeopResult<&Sketch3d<Design>>;

    /// The datum `name` a step added — not [`crate::ORIGIN`], which no
    /// step adds.
    fn datum(&mut self, name: &str) -> GeopResult<&Datum<S>>;

    /// The value of the parameter `name`, if the program has one.
    fn input(&mut self, name: &str) -> Option<ParamValue>;

    /// The part placed in the part looked up that `entity` is an entity
    /// of, however deep (see [`crate::placed::within`]): what looks up in
    /// it, as its own program left it, where it is in the part looked up,
    /// and the entity as it names it. None for an entity of the part itself
    /// — and for every entity, where no part is placed: a model on its own.
    fn in_placed(&mut self, entity: &EntityRef) -> Option<GeopResult<Placed<'_, S>>> {
        let _ = entity;
        None
    }

    /// The value of the formula `expression` — a plain number, or one
    /// reading the program's parameters by name (see
    /// [`crate::parameters::evaluate`]).
    fn evaluate(&mut self, expression: &str) -> GeopResult<f64> {
        crate::parameters::evaluate(expression, |name| match self.input(name) {
            Some(ParamValue::Number(v)) => Some(v.to_f64()),
            _ => None,
        })
    }
}

/// A part placed in the part looked up, as [`Lookup::in_placed`] finds it:
/// what looks up in it, where it is, and an entity of it as it names it.
pub type Placed<'a, S> = (Box<dyn Lookup<S> + 'a>, Pose<S>, EntityRef);

/// [`Lookup::in_placed`] for a lookup reading the part through `r`.
fn in_placed<'a, 'r>(
    r: &'r mut impl Retrieve<'a, Input, ParamValue>,
    entity: &EntityRef,
) -> Option<GeopResult<Placed<'a, Design>>> {
    let (path, _, local) = within(r, entity)?;
    Some(locate(r, &path).map(|located| {
        let inside: Box<dyn Lookup<Design> + 'a> =
            Box::new(Seen::new(located.instance.component.view()));
        (inside, located.world, local)
    }))
}

/// A step's generator resolving what it builds on: the model it revises,
/// and everything else through its reader, so the step depends on what it
/// read.
pub struct Scope<'r, 'a> {
    pub model: &'r NamedModel<Design>,
    pub reader: &'r mut BuildReader<'a>,
}

impl Lookup<Design> for Scope<'_, '_> {
    fn model(&self) -> &NamedModel<Design> {
        self.model
    }

    fn sketch(&mut self, name: &str) -> GeopResult<&PlacedSketch<Design>> {
        self.reader.retrieve_target(&sketch(name))
    }

    fn sketch3d(&mut self, name: &str) -> GeopResult<&Sketch3d<Design>> {
        Ok(&self.reader.retrieve_target(&sketch3d(name))?.sketch)
    }

    fn datum(&mut self, name: &str) -> GeopResult<&Datum<Design>> {
        self.reader.retrieve_target(&datum(name))
    }

    fn input(&mut self, name: &str) -> Option<ParamValue> {
        value(self.reader.retrieve_input(name))
    }

    fn in_placed(&mut self, entity: &EntityRef) -> Option<GeopResult<Placed<'_, Design>>> {
        in_placed(self.reader, entity)
    }
}

/// An editor resolving against what the last run built: a step's view
/// before or after it. Builds nothing, and records nothing.
pub struct Seen<'v> {
    model: Cow<'v, NamedModel<Design>>,
    view: BuildView<'v>,
}

impl<'v> Seen<'v> {
    /// What `view` shows: the model as it was there — empty, before any
    /// step built one — and the sketches.
    pub fn new(view: BuildView<'v>) -> Self {
        Self {
            model: model_or_empty(&mut view.clone()),
            view,
        }
    }

    /// The view it sees through.
    pub fn view(&self) -> BuildView<'v> {
        self.view
    }

    /// Every sketch the view sees, by name, the newest last.
    pub fn sketches(&self) -> Vec<(&'v str, &'v PlacedSketch<Design>)> {
        (self.view.targets_of::<PlacedSketch<Design>>().into_iter())
            .filter_map(|(target, sketch)| Some((sketch_name(target)?, sketch)))
            .collect()
    }

    /// Every datum a step added that the view sees, by name, the newest
    /// last.
    pub fn datums(&self) -> Vec<(&'v str, &'v Datum<Design>)> {
        (self.view.targets_of::<Datum<Design>>().into_iter())
            .filter_map(|(target, datum)| Some((datum_name(target)?, datum)))
            .collect()
    }

    /// The name of every sketch the view sees, the newest last.
    pub fn sketch_names(&self) -> Vec<String> {
        (self.sketches().into_iter())
            .map(|(name, _)| name.to_string())
            .collect()
    }

    /// Every 3-D sketch the view sees, by name, the newest last.
    pub fn sketches3d(&self) -> Vec<(&'v str, &'v PlacedSketch3d)> {
        (self.view.targets_of::<PlacedSketch3d>().into_iter())
            .filter_map(|(target, sketch)| Some((sketch3d_name(target)?, sketch)))
            .collect()
    }

    /// The name of every 3-D sketch the view sees, the newest last.
    pub fn sketch3d_names(&self) -> Vec<String> {
        (self.sketches3d().into_iter())
            .map(|(name, _)| name.to_string())
            .collect()
    }
}

/// What an editor sees, held or borrowed: reading records nothing, so a
/// form's setters can all hold the one view. One body for both, as a
/// borrowed view's references cannot outlive a call through the held one.
macro_rules! lookup_seen {
    ($seen:ty) => {
        impl Lookup<Design> for $seen {
            fn model(&self) -> &NamedModel<Design> {
                &self.model
            }

            fn sketch(&mut self, name: &str) -> GeopResult<&PlacedSketch<Design>> {
                self.view.retrieve_target(&sketch(name))
            }

            fn sketch3d(&mut self, name: &str) -> GeopResult<&Sketch3d<Design>> {
                Ok(&self.view.retrieve_target(&sketch3d(name))?.sketch)
            }

            fn datum(&mut self, name: &str) -> GeopResult<&Datum<Design>> {
                self.view.retrieve_target(&datum(name))
            }

            fn input(&mut self, name: &str) -> Option<ParamValue> {
                value(self.view.retrieve_input(name))
            }

            fn in_placed(&mut self, entity: &EntityRef) -> Option<GeopResult<Placed<'_, Design>>> {
                in_placed(&mut self.view.clone(), entity)
            }
        }
    };
}

lookup_seen!(Seen<'_>);
lookup_seen!(&Seen<'_>);

/// A model on its own: what it holds resolves, and it has no sketches and
/// no parameters — what a step resolves bodies and faces against.
impl<S: Scalar> Lookup<S> for &NamedModel<S> {
    fn model(&self) -> &NamedModel<S> {
        self
    }

    fn sketch(&mut self, name: &str) -> GeopResult<&PlacedSketch<S>> {
        Err(GeopError::new(format!(
            "there is no sketch {name:?}: a model on its own has none"
        )))
    }

    fn sketch3d(&mut self, name: &str) -> GeopResult<&Sketch3d<Design>> {
        Err(GeopError::new(format!(
            "there is no 3-D sketch {name:?}: a model on its own has none"
        )))
    }

    fn datum(&mut self, name: &str) -> GeopResult<&Datum<S>> {
        Err(GeopError::new(format!(
            "there is no datum {name:?}: a model on its own has none"
        )))
    }

    fn input(&mut self, _: &str) -> Option<ParamValue> {
        None
    }
}
