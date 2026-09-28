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
    operation::{Operation, OperationArgs},
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

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, OperationArgs)]
pub struct BooleanArgs {
    /// The first solid; for a difference, the one cut from.
    #[arg(Solid)]
    pub a: String,
    /// The second solid; for a difference, the one cut away.
    #[arg(Solid)]
    pub b: String,
    /// How to combine them.
    #[arg(Choice { options: &["union", "intersection", "difference"], default: "difference" })]
    pub op: BooleanOp,
}

impl<S: Scalar> Operation<S> for Boolean {
    type Args = BooleanArgs;

    fn apply(
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

impl Combine {
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
