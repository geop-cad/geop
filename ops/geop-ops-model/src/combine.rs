//! How a step meets the solid it builds a tool for ([`Combine`]): kept as a
//! new body, or joined to, cut from or intersected with a solid of the part
//! ([`SolidRef`]) — the vocabulary every feature that combines shares. The
//! boolean that applies it is `geop_lib_booleans::combine_tools`, which takes the
//! [`Tool`]s a step built.

use geop_core_editor::{
    edit_form::{Edit, EditForm},
    editing::Choice,
};
use geop_core_math::scalars::Scalar;
use geop_core_program::{element::ElementReference, reader::Reader, values::Design};
use geop_core_topology::SolidId;
use serde::{Deserialize, Serialize};

use crate::{BooleanOp, NamedModel, Namer, Solid, ToolReach, model, read_model};

/// A solid of the part, by name.
pub type SolidRef = ElementReference<NamedModel<Design>, Solid>;

/// The solid named `name`.
pub fn solid(name: impl Into<String>) -> SolidRef {
    ElementReference::new(model(), name)
}

/// What an extrude or revolve does with the solid it builds: keep it as a
/// new body, or combine it by `op` with the solid named `target` — which
/// that consumes, as a boolean step does. A [`BooleanOp::Difference`] cuts
/// the new solid out of the target: an extruded pocket, a drilled hole.
///
/// A step may build its solid in several parts, each combined in turn — the
/// two sides of an extrude, one of which goes up to the next face of the
/// target (see [`Tool`]).
///
/// Either way the step's result is named after the step — `extrude(E)` —
/// so what comes after refers to "what step `E` left" however it was made.
/// Combined, the built solid is only a tool, gone once the step is done,
/// and what the combination creates is named `combine(E,...)`, as a
/// boolean step names its own.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum Combine {
    #[default]
    NewBody,
    With {
        op: BooleanOp,
        target: SolidRef,
    },
}

impl Combine {
    /// Cutting the tools out of the solid named `target`.
    pub fn cut(target: impl Into<String>) -> Self {
        Combine::With {
            op: BooleanOp::Difference,
            target: solid(target),
        }
    }

    /// Joining the tools onto the solid named `target`.
    pub fn join(target: impl Into<String>) -> Self {
        Combine::With {
            op: BooleanOp::Union,
            target: solid(target),
        }
    }

    /// Keeping what the tools and the solid named `target` have in common.
    pub fn intersect(target: impl Into<String>) -> Self {
        Combine::With {
            op: BooleanOp::Intersection,
            target: solid(target),
        }
    }

    /// The solid it combines with, if any.
    pub fn target(&self) -> Option<&str> {
        match self {
            Combine::NewBody => None,
            Combine::With { target, .. } => Some(target.name()),
        }
    }

    /// The default for a step after `before`: joining the newest solid, if
    /// there is one — building onto what is there is the common case.
    pub fn new_for<S: Scalar>(before: &NamedModel<S>) -> Self {
        match before.newest_solid() {
            Some(target) => Combine::join(target),
            None => Combine::NewBody,
        }
    }

    /// A join for a positive `value`, a cut for a negative one — what an
    /// extrude up out of a face, or down into it, means — when `value`
    /// crosses to the other side of zero from `from`. On the same side, a
    /// join or a cut stays what was chosen: an extrude may cut upwards into
    /// a solid above its sketch. A new body and an intersection stay what
    /// they are.
    pub fn follow_sign(&mut self, from: f64, value: f64) {
        let crossed = (from > 0.0) != (value > 0.0) || (from < 0.0) != (value < 0.0);
        if let Combine::With { op, .. } = self
            && crossed
            && *op != BooleanOp::Intersection
        {
            if value > 0.0 {
                *op = BooleanOp::Union;
            } else if value < 0.0 {
                *op = BooleanOp::Difference;
            }
        }
    }

    /// The field that picks the solid to combine with — or to go as far
    /// as — labelled `label`, holding `target`, in the part `reader` reads;
    /// `set` gets the solid picked.
    pub fn target_field<'a, S: Scalar, A, T>(
        f: &mut EditForm<'a, S, A, T>,
        reader: &Reader<'a>,
        label: &str,
        target: Option<&SolidRef>,
        set: impl Fn(Edit<'_, A, T>, SolidRef) + 'a,
    ) {
        f.pick_one(TARGET, label, reader, target, set);
    }

    /// Its fields in `f`: the mode — chosen without a target, it takes the
    /// newest solid of the part `reader` reads — and the target to pick.
    /// `set` puts the combination set where the arguments keep it.
    pub fn fields<'a, S: Scalar, A, T>(
        &self,
        f: &mut EditForm<'a, S, A, T>,
        reader: &Reader<'a>,
        set: impl Fn(&mut A, Combine) + Clone + 'a,
    ) {
        let mode = match self {
            Combine::NewBody => NEW_BODY,
            Combine::With { op, .. } => op.value(),
        };
        let modes =
            std::iter::once(Choice::new(NEW_BODY, "New body")).chain(BooleanOp::combine_choices());
        let newest = read_model(reader)
            .ok()
            .and_then(|model| model.newest_solid());
        let target = (self.target().map(str::to_string))
            .or(newest)
            .unwrap_or_default();
        let set_mode = set.clone();
        f.select(
            "combine",
            "combine",
            mode,
            modes.collect(),
            false,
            move |edit, mode| {
                if mode == NEW_BODY {
                    set_mode(edit.args, Combine::NewBody);
                } else if let Some(op) = BooleanOp::from_value(mode) {
                    let target = solid(&target);
                    set_mode(edit.args, Combine::With { op, target });
                }
            },
        );
        if let Combine::With { op, target } = self {
            let op = *op;
            Self::target_field(f, reader, "target", Some(target), move |edit, target| {
                set(edit.args, Combine::With { op, target })
            });
        }
    }

    /// The name the built solid gets: the step's own as a new body, or a
    /// name within the step for a tool the combination consumes.
    pub fn built_name(&self, namer: &Namer) -> String {
        match self {
            Combine::NewBody => namer.root(),
            Combine::With { .. } => namer.name(&["tool"]),
        }
    }
}

/// The key of the field that picks the solid a step combines with (see
/// [`Combine::target_field`]).
const TARGET: &str = "combine_target";

/// How a new body is offered, among the ways to combine.
const NEW_BODY: &str = "new_body";

impl BooleanOp {
    /// Every boolean operation, in the order a dialog offers them.
    const ALL: [BooleanOp; 3] = [
        BooleanOp::Union,
        BooleanOp::Difference,
        BooleanOp::Intersection,
    ];

    /// How it is offered, as a boolean or as a way to combine.
    pub fn value(self) -> &'static str {
        match self {
            BooleanOp::Union => "union",
            BooleanOp::Difference => "difference",
            BooleanOp::Intersection => "intersection",
        }
    }

    /// The op offered as `value`.
    pub fn from_value(value: &str) -> Option<BooleanOp> {
        Self::ALL.into_iter().find(|op| op.value() == value)
    }

    /// The ops as a boolean offers them: union, difference, intersection.
    pub fn choices() -> Vec<Choice> {
        let label = |op| match op {
            BooleanOp::Union => "Union",
            BooleanOp::Difference => "Difference",
            BooleanOp::Intersection => "Intersection",
        };
        (Self::ALL.into_iter())
            .map(|op| Choice::new(op.value(), label(op)))
            .collect()
    }

    /// The ops as a way to combine a tool: join, cut, intersect.
    fn combine_choices() -> impl Iterator<Item = Choice> {
        let label = |op| match op {
            BooleanOp::Union => "Join",
            BooleanOp::Difference => "Cut",
            BooleanOp::Intersection => "Intersect",
        };
        (Self::ALL.into_iter()).map(move |op| Choice::new(op.value(), label(op)))
    }
}

/// One solid an extrude or revolve built, for `geop_lib_booleans::combine_tools`
/// to combine: the whole of it, or, with an `up_to_next` reach, only up to
/// the next face of the target.
#[derive(Clone, Debug)]
pub struct Tool {
    pub solid: SolidId,
    pub reach: ToolReach,
}

impl Tool {
    /// All of `solid`, the one tool of its step.
    pub fn whole(solid: SolidId) -> Self {
        Tool {
            solid,
            reach: ToolReach::default(),
        }
    }
}
