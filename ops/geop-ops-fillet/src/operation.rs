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
    pub radius: f64,
    /// The radius at the end of every chain, if it changes along it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end_radius: Option<f64>,
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
    /// Round `edges` with `radius`, the same all along.
    pub fn constant(edges: Vec<String>, radius: f64) -> Self {
        FilletArgs {
            edges,
            radius,
            end_radius: None,
            vertex_radii: Vec::new(),
        }
    }

    /// Whether the radius changes along the edges.
    fn variable(&self) -> bool {
        self.end_radius.is_some() || !self.vertex_radii.is_empty()
    }

    fn radii(&self) -> Radii {
        Radii {
            radius: self.radius,
            end_radius: self.end_radius,
            at_vertices: self
                .vertex_radii
                .iter()
                .map(|v| (v.vertex.clone(), v.radius))
                .collect(),
        }
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
        f.checkbox(
            "variable",
            "variable radius",
            args.variable(),
            |args, on| {
                args.end_radius = on.then_some(args.radius);
                if !on {
                    args.vertex_radii.clear();
                }
            },
        );
        if args.variable() {
            f.number(
                "end_radius",
                Number::new(
                    "end radius",
                    args.end_radius.unwrap_or(args.radius),
                    Unit::Length,
                )
                .range(0.0, 1.0),
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
                |e, picked| {
                    let radius = e.args.radius;
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
            radii: args.radii(),
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
            &BlendShape::Chamfer {
                distances: args.distances(),
            },
        )
        .with_context(ctx)?;
        Ok(part)
    }
}
