//! [`Boolean`]: combine two solids — and [`Combine`], the same done by an
//! extrude or revolve with the solid it builds.

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_topology::SolidId;
use geop_ops::{
    Namer, Part,
    operation::{EntityRef, Operation},
    ui::{Choice, Dialog, Form, Target, Value},
};
use serde::{Deserialize, Serialize};

use crate::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};

/// Combines the solids named `a` and `b` into one solid named `boolean(B)`
/// for the operation `B`, consuming both.
///
/// Every face, edge and vertex that survives keeps its name. What the
/// boolean creates is named after what it was made from, see
/// [`crate::naming`] — for example `boolean(B,E1,E2,i,n)` for the
/// `i`-th of the `n` points where edges `E1` and `E2` cross.
///
/// An empty result — intersecting solids that don't overlap, say — is an
/// answer, not an error: both operands are consumed and no solid is named
/// `boolean(B)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Boolean;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BooleanArgs {
    /// The first solid; for a difference, the one cut from.
    pub a: String,
    /// The second solid; for a difference, the one cut away.
    pub b: String,
    /// How to combine them.
    pub op: BooleanOp,
}

/// The boolean operations, as a dialog offers them: `(op, value, label)`.
const OPS: [(BooleanOp, &str, &str); 3] = [
    (BooleanOp::Union, "union", "Union"),
    (BooleanOp::Intersection, "intersection", "Intersection"),
    (BooleanOp::Difference, "difference", "Difference"),
];

/// A solid by name, as a pick field holds it: nothing, if unnamed.
fn solid(name: &str) -> Vec<EntityRef> {
    if name.is_empty() {
        Vec::new()
    } else {
        vec![EntityRef::Solid { name: name.into() }]
    }
}

impl Operation for Boolean {
    type Args = BooleanArgs;
    type Session = ();

    /// The difference of the two newest solids: the older cut by the one
    /// just built as a tool.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> BooleanArgs {
        let solids = before.solid_names();
        let newest = |back: usize| {
            solids
                .len()
                .checked_sub(back)
                .map(|i| solids[i].clone())
                .unwrap_or_default()
        };
        BooleanArgs {
            a: newest(2),
            b: newest(1),
            op: BooleanOp::Difference,
        }
    }

    /// The two solids, picked, and how to combine them.
    fn form<S: Scalar>(&self, _before: &Part<S>, args: &BooleanArgs, _: &()) -> Form<S> {
        let mut d = Dialog::new();
        d.pick("a", "a", solid(&args.a), &[Target::Solid], false);
        d.pick("b", "b", solid(&args.b), &[Target::Solid], false);
        d.select(
            "op",
            "op",
            OPS.iter().find(|o| o.0 == args.op).map_or("", |o| o.1),
            OPS.iter()
                .map(|&(_, value, label)| Choice::new(value, label))
                .collect(),
        );
        Form::dialog(d)
    }

    fn set<S: Scalar>(
        &self,
        _before: &Part<S>,
        args: &mut BooleanArgs,
        _: &mut (),
        key: &str,
        value: Value,
    ) {
        match (key, value) {
            ("a", Value::Entity(EntityRef::Solid { name })) => args.a = name,
            ("b", Value::Entity(EntityRef::Solid { name })) => args.b = name,
            ("op", Value::Choice(value)) => {
                if let Some(&(op, ..)) = OPS.iter().find(|o| o.1 == value) {
                    args.op = op;
                }
            }
            _ => {}
        }
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &BooleanArgs,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("boolean({operation_id}, {args:?})");
        let namer = Namer::new("boolean", operation_id)?;
        let a = part.solid_id(&args.a).with_context(ctx)?;
        let b = part.solid_id(&args.b).with_context(ctx)?;
        boolean(&mut part, &namer, a, b, args.op, RemeshParams::default()).with_context(ctx)?;
        Ok(part)
    }
}

/// What an extrude or revolve does with the solid it builds: keep it as a
/// new body, or combine it with the solid named `target` — which that
/// consumes, like a [`Boolean`] does. `Difference` cuts the new solid out
/// of the target: an extruded pocket, a drilled hole.
///
/// Either way the step's result is named after the step — `extrude(E)` —
/// so what comes after refers to "what step `E` left" however it was made.
/// Combined, the built solid is only a tool, gone once the step is done,
/// and what the combination creates is named `combine(E,...)`, as
/// [`Boolean`] names its own.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum Combine {
    #[default]
    NewBody,
    Union {
        target: String,
    },
    Intersection {
        target: String,
    },
    Difference {
        target: String,
    },
}

/// How each combine mode is offered: `(value, label)`.
const MODES: [(&str, &str); 4] = [
    ("new_body", "New body"),
    ("union", "Join"),
    ("difference", "Cut"),
    ("intersection", "Intersect"),
];

/// The dialog key of a [`Combine`]'s target.
const TARGET: &str = "combine_target";

impl Combine {
    /// The mode, as it serializes: `new_body`, `union`.
    pub fn mode(&self) -> &'static str {
        match self {
            Combine::NewBody => "new_body",
            Combine::Union { .. } => "union",
            Combine::Intersection { .. } => "intersection",
            Combine::Difference { .. } => "difference",
        }
    }

    /// The solid it combines with, if any.
    pub fn target(&self) -> Option<&str> {
        match self {
            Combine::NewBody => None,
            Combine::Union { target }
            | Combine::Intersection { target }
            | Combine::Difference { target } => Some(target),
        }
    }

    /// The mode `mode` (as it serializes) with `target`.
    fn with_mode(mode: &str, target: String) -> Option<Self> {
        Some(match mode {
            "new_body" => Combine::NewBody,
            "union" => Combine::Union { target },
            "intersection" => Combine::Intersection { target },
            "difference" => Combine::Difference { target },
            _ => return None,
        })
    }

    /// The default for a step after `before`: joining the newest solid, if
    /// there is one — building onto what is there is the common case.
    pub fn new_for<S: Scalar>(before: &Part<S>) -> Self {
        match before.solid_names().pop() {
            Some(target) => Combine::Union { target },
            None => Combine::NewBody,
        }
    }

    /// A join for a positive `value`, a cut for a negative one: what an
    /// extrude up out of a face, or down into it, means. A new body and an
    /// intersection stay what they are.
    pub fn follow_sign(&mut self, value: f64) {
        *self = match std::mem::take(self) {
            Combine::Union { target } | Combine::Difference { target } if value > 0.0 => {
                Combine::Union { target }
            }
            Combine::Union { target } | Combine::Difference { target } if value < 0.0 => {
                Combine::Difference { target }
            }
            other => other,
        };
    }

    /// Its fields in a dialog: the mode, and the target to pick.
    pub fn show(&self, d: &mut Dialog) {
        d.select(
            "combine",
            "combine",
            self.mode(),
            MODES
                .iter()
                .map(|&(value, label)| Choice::new(value, label))
                .collect(),
        );
        if let Some(target) = self.target() {
            d.pick(TARGET, "target", solid(target), &[Target::Solid], false);
        }
    }

    /// Sets its field `key` (see [`Combine::show`]) to `value`: a mode —
    /// the target, if it had none, the newest solid of `before` — or the
    /// target. Whether `key` is one of its fields.
    pub fn set<S: Scalar>(&mut self, before: &Part<S>, key: &str, value: &Value) -> bool {
        match (key, value) {
            ("combine", Value::Choice(mode)) => {
                let target = self
                    .target()
                    .map(str::to_string)
                    .or_else(|| before.solid_names().pop())
                    .unwrap_or_default();
                if let Some(next) = Combine::with_mode(mode, target) {
                    *self = next;
                }
                true
            }
            (TARGET, Value::Entity(EntityRef::Solid { name })) => {
                if let Some(next) = Combine::with_mode(self.mode(), name.clone()) {
                    *self = next;
                }
                true
            }
            _ => false,
        }
    }

    /// The name the built solid gets: the step's own as a new body, or a
    /// name within the step for a tool the combination consumes.
    pub fn built_name(&self, namer: &Namer) -> String {
        match self {
            Combine::NewBody => namer.root(),
            _ => namer.name(&["tool"]),
        }
    }

    /// Combines `built` as the step `operation_id`, whose names `namer`
    /// builds: the result is named `namer`'s root. An empty result — an
    /// intersection with a solid the new one does not touch — leaves no
    /// solid.
    pub fn apply<S: Scalar>(
        &self,
        part: &mut Part<S>,
        namer: &Namer,
        operation_id: &str,
        built: SolidId,
    ) -> GeopResult<()> {
        let (op, target) = match self {
            Combine::NewBody => return Ok(()),
            Combine::Union { target } => (BooleanOp::Union, target),
            Combine::Intersection { target } => (BooleanOp::Intersection, target),
            Combine::Difference { target } => (BooleanOp::Difference, target),
        };
        let ctx = with_context!("combining with {target:?} ({op:?})");
        let target = part.solid_id(target).with_context(ctx)?;
        let combine = Namer::new("combine", operation_id)?;
        let result = boolean(part, &combine, target, built, op, RemeshParams::default())
            .with_context(ctx)?;
        if let Some(result) = result {
            part.rename(result, namer.root())?;
        }
        Ok(())
    }
}
