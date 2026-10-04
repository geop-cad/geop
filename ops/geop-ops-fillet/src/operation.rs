//! [`Fillet`]: round a solid's edges. [`Chamfer`]: bevel them. Both blend
//! the edges picked (see [`crate::blend`]).

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Form, Number, Unit},
};
use serde::{Deserialize, Serialize};

use crate::blend::{BlendShape, blend};

/// Edges by name, as a reference field holds them.
fn edge_refs(names: &[String]) -> Vec<EntityRef> {
    names
        .iter()
        .map(|name| EntityRef::Edge { name: name.clone() })
        .collect()
}

/// The names of the edges a reference field holds.
fn edge_names(picked: &[EntityRef]) -> Vec<String> {
    picked
        .iter()
        .filter_map(|e| match e {
            EntityRef::Edge { name } => Some(name.clone()),
            _ => None,
        })
        .collect()
}

/// The edges field of both operations.
fn edges_field<'a, S: Scalar, A: 'a>(
    form: &mut Form<'a, S, A>,
    edges: &[String],
    set: fn(&mut A) -> &mut Vec<String>,
) {
    form.reference(
        "edges",
        "edges",
        edge_refs(edges),
        &[Role::Edge],
        None,
        true,
        move |e, picked| *set(e.args) = edge_names(&picked),
    );
}

/// Rounds the edges named `edges` of a solid with a fillet of `radius`, for
/// the operation `F`: the solid is consumed and the result named
/// `fillet(F)`. Every face, edge and vertex that survives keeps its name;
/// the round face of edge `E` is `fillet(F,E,fillet)` — by quarter turn,
/// `fillet(F,E,fillet,q0)`, ..., for a circle — and what applying it
/// creates is named as a boolean names it, `fillet(F,E,...)` (see
/// [`crate::blend::blend`]). A convex edge is cut round, a concave one
/// filled in.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Fillet;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FilletArgs {
    /// The edges to round, all of one solid.
    pub edges: Vec<String>,
    pub radius: f64,
}

impl Operation for Fillet {
    type Args = FilletArgs;
    type Session = ();

    /// No edges yet, and a small radius.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> FilletArgs {
        FilletArgs {
            edges: Vec::new(),
            radius: 0.1,
        }
    }

    /// The edges, picked, and the radius.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &FilletArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, FilletArgs> {
        let mut f = Form::<S, FilletArgs>::new();
        edges_field(&mut f, &args.edges, |a| &mut a.edges);
        f.number(
            "radius",
            Number::new("radius", args.radius, Unit::Length).range(0.0, 1.0),
            |args, radius| args.radius = radius,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &FilletArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("fillet({operation_id}, {args:?})");
        let namer = Namer::new("fillet", operation_id)?;
        blend(
            &mut part,
            &namer,
            &args.edges,
            BlendShape::Fillet {
                radius: args.radius,
            },
        )
        .with_context(ctx)?;
        Ok(part)
    }
}

/// Bevels the edges named `edges` of a solid with a chamfer `distance` into
/// both faces — or, with `distance2`, `distance` into the face on the left
/// of each edge as it runs (seen from outside the solid) and `distance2`
/// into the one on its right — for the operation `C`: the solid is consumed
/// and the result named `chamfer(C)`, the bevel face of edge `E`
/// `chamfer(C,E,chamfer)`, named like a [`Fillet`]'s otherwise.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Chamfer;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChamferArgs {
    /// The edges to bevel, all of one solid.
    pub edges: Vec<String>,
    pub distance: f64,
    /// The distance into the face on each edge's right, if it differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance2: Option<f64>,
}

impl ChamferArgs {
    /// Into the face on the left and on the right of each edge.
    fn distances(&self) -> [f64; 2] {
        [self.distance, self.distance2.unwrap_or(self.distance)]
    }
}

impl Operation for Chamfer {
    type Args = ChamferArgs;
    type Session = ();

    /// No edges yet, and a small distance, the same into both faces.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> ChamferArgs {
        ChamferArgs {
            edges: Vec::new(),
            distance: 0.1,
            distance2: None,
        }
    }

    /// The edges, picked, the distance, and whether the second face gets a
    /// distance of its own.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &ChamferArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ChamferArgs> {
        let mut f = Form::<S, ChamferArgs>::new();
        edges_field(&mut f, &args.edges, |a| &mut a.edges);
        f.number(
            "distance",
            Number::new("distance", args.distance, Unit::Length).range(0.0, 1.0),
            |args, distance| args.distance = distance,
        );
        f.checkbox(
            "two_distances",
            "two distances",
            args.distance2.is_some(),
            |args, on| args.distance2 = on.then_some(args.distance),
        );
        if let Some(distance2) = args.distance2 {
            f.number(
                "distance2",
                Number::new("distance 2", distance2, Unit::Length).range(0.0, 1.0),
                |args, distance| args.distance2 = Some(distance),
            );
        }
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &ChamferArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("chamfer({operation_id}, {args:?})");
        let namer = Namer::new("chamfer", operation_id)?;
        blend(
            &mut part,
            &namer,
            &args.edges,
            BlendShape::Chamfer {
                distances: args.distances(),
            },
        )
        .with_context(ctx)?;
        Ok(part)
    }
}
