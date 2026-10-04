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
    parameters::Formula,
    ui::{Form, Number, Unit},
};
use serde::{Deserialize, Serialize};

use crate::{
    blend::{BlendShape, blend},
    rolling::Radii,
};

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
/// `fillet(F,E,fillet,q0)`, ..., for a circle, and by edge of its tangent
/// chain, `fillet(F,E,fillet,s0)`, ..., for one rolled along several (see
/// [`crate::rolling`]) — and what applying it creates is named as a
/// boolean names it, `fillet(F,E,...)` (see [`crate::blend::blend`]). A
/// convex edge is cut round, a concave one filled in.
///
/// Any edge is rounded: a straight one between two planes and a circle
/// around its faces' axis exactly, every other one by a ball rolled along
/// it, which also takes its tangent chain along. With `end_radius` the
/// radius changes linearly along each chain, from `radius` where its first
/// picked edge starts to `end_radius` where the chain ends; `vertex_radii`
/// set it at vertices along a chain, linearly in between.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Fillet;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FilletArgs {
    /// The edges to round, all of one solid.
    pub edges: Vec<String>,
    /// The radius: a number, or a formula of the part's parameters.
    pub radius: Formula,
    /// The radius at the end of every chain, if it changes along it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end_radius: Option<Formula>,
    /// Radii at vertices along the chains.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub vertex_radii: Vec<VertexRadius>,
}

/// The radius a fillet has at a vertex, by name.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VertexRadius {
    pub vertex: String,
    pub radius: f64,
}

impl FilletArgs {
    /// Round `edges` with `radius` — a number, or a formula's text — the
    /// same all along.
    pub fn constant(edges: Vec<String>, radius: impl Into<Formula>) -> Self {
        FilletArgs {
            edges,
            radius: radius.into(),
            end_radius: None,
            vertex_radii: Vec::new(),
        }
    }

    /// Whether the radius changes along the edges.
    fn variable(&self) -> bool {
        self.end_radius.is_some() || !self.vertex_radii.is_empty()
    }

    /// The radii, as the step building `part` reads them.
    fn radii<S: Scalar>(&self, part: &mut Part<S>) -> GeopResult<Radii> {
        Ok(Radii {
            radius: self.radius.evaluate(part)?,
            end_radius: match &self.end_radius {
                Some(r) => Some(r.evaluate(part)?),
                None => None,
            },
            at_vertices: self
                .vertex_radii
                .iter()
                .map(|v| (v.vertex.clone(), v.radius))
                .collect(),
        })
    }
}

impl Operation for Fillet {
    type Args = FilletArgs;
    type Session = ();

    /// No edges yet, and a small radius.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> FilletArgs {
        FilletArgs::constant(Vec::new(), 0.1)
    }

    /// The edges, picked, and the radius — and, varying, the radius at the
    /// end and at vertices picked, each a number of its own.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &FilletArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, FilletArgs> {
        let inputs = context.before.inputs();
        let mut f = Form::<S, FilletArgs>::new();
        edges_field(&mut f, &args.edges, |a| &mut a.edges);
        f.formula(
            "radius",
            Number::formula("radius", &args.radius, inputs, Unit::Length).range(0.0, 1.0),
            |args, radius| args.radius = radius,
        );
        f.checkbox(
            "variable",
            "variable radius",
            args.variable(),
            |args, on| {
                args.end_radius = on.then(|| args.radius.clone());
                if !on {
                    args.vertex_radii.clear();
                }
            },
        );
        if args.variable() {
            let end = args.end_radius.as_ref().unwrap_or(&args.radius);
            f.formula(
                "end_radius",
                Number::formula("end radius", end, inputs, Unit::Length).range(0.0, 1.0),
                |args, radius| args.end_radius = Some(radius),
            );
            let vertices = args
                .vertex_radii
                .iter()
                .map(|v| EntityRef::Vertex {
                    name: v.vertex.clone(),
                })
                .collect();
            f.reference(
                "radius_vertices",
                "radii at vertices",
                vertices,
                &[Role::Point],
                None,
                true,
                move |e, picked| {
                    // A new vertex starts at the radius as it is now.
                    let radius = e.args.radius.peek(inputs).unwrap_or(0.1);
                    let old = std::mem::take(&mut e.args.vertex_radii);
                    e.args.vertex_radii = picked
                        .iter()
                        .filter_map(|p| match p {
                            EntityRef::Vertex { name } => Some(name.clone()),
                            _ => None,
                        })
                        .map(|vertex| VertexRadius {
                            radius: old
                                .iter()
                                .find(|v| v.vertex == vertex)
                                .map_or(radius, |v| v.radius),
                            vertex,
                        })
                        .collect();
                },
            );
            f.optional("radius_vertices");
            for (i, v) in args.vertex_radii.iter().enumerate() {
                f.number(
                    &format!("vertex_radius_{i}"),
                    Number::new(format!("radius at {}", v.vertex), v.radius, Unit::Length)
                        .range(0.0, 1.0),
                    move |args, radius| {
                        if let Some(v) = args.vertex_radii.get_mut(i) {
                            v.radius = radius;
                        }
                    },
                );
            }
        }
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
        let shape = BlendShape::Fillet {
            radii: args.radii(&mut part).with_context(ctx)?,
        };
        blend(&mut part, &namer, &args.edges, &shape).with_context(ctx)?;
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
    /// How far into the faces: a number, or a formula of the part's
    /// parameters.
    pub distance: Formula,
    /// The distance into the face on each edge's right, if it differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance2: Option<Formula>,
}

impl ChamferArgs {
    /// Into the face on the left and on the right of each edge, as the
    /// step building `part` reads them.
    fn distances<S: Scalar>(&self, part: &mut Part<S>) -> GeopResult<[f64; 2]> {
        let left = self.distance.evaluate(part)?;
        let right = match &self.distance2 {
            Some(d) => d.evaluate(part)?,
            None => left,
        };
        Ok([left, right])
    }
}

impl Operation for Chamfer {
    type Args = ChamferArgs;
    type Session = ();

    /// No edges yet, and a small distance, the same into both faces.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> ChamferArgs {
        ChamferArgs {
            edges: Vec::new(),
            distance: Formula::Plain(0.1),
            distance2: None,
        }
    }

    /// The edges, picked, the distance, and whether the second face gets a
    /// distance of its own.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &ChamferArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ChamferArgs> {
        let inputs = context.before.inputs();
        let mut f = Form::<S, ChamferArgs>::new();
        edges_field(&mut f, &args.edges, |a| &mut a.edges);
        f.formula(
            "distance",
            Number::formula("distance", &args.distance, inputs, Unit::Length).range(0.0, 1.0),
            |args, distance| args.distance = distance,
        );
        f.checkbox(
            "two_distances",
            "two distances",
            args.distance2.is_some(),
            |args, on| args.distance2 = on.then(|| args.distance.clone()),
        );
        if let Some(distance2) = &args.distance2 {
            f.formula(
                "distance2",
                Number::formula("distance 2", distance2, inputs, Unit::Length).range(0.0, 1.0),
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
        let shape = BlendShape::Chamfer {
            distances: args.distances(&mut part).with_context(ctx)?,
        };
        blend(&mut part, &namer, &args.edges, &shape).with_context(ctx)?;
        Ok(part)
    }
}
