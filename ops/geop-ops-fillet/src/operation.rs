//! [`Fillet`]: round a solid's edges. [`Chamfer`]: bevel them. Both blend
//! the edges picked (see [`crate::blend`]).

use geop_core_editor::view::icons;
use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::ElementReference, numeric::NumericInput, part::StepContext, reader::Reader,
    values::Design,
};
use geop_ops_model::{Edge, ModelOperation, NamedModel, Namer, Vertex, build_model, model};
use serde::{Deserialize, Serialize};

use crate::{
    blend::{BlendShape, blend},
    rolling::Radii,
};

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
/// set it at vertices along a chain, linearly in between. A corner every
/// edge of which is rounded, with one radius, is rounded by the ball
/// touching its three faces, `fillet(F,V,corner)` for its vertex `V` (see
/// [`crate::corner`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Fillet;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FilletArgs {
    /// The edges to round, all of one solid.
    pub edges: Vec<ElementReference<NamedModel<Design>, Edge>>,
    /// The radius: a number, or a formula of the part's parameters.
    pub radius: NumericInput,
    /// The radius at the end of every chain, if it changes along it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end_radius: Option<NumericInput>,
    /// Radii at vertices along the chains.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub vertex_radii: Vec<VertexRadius>,
}

/// The radius a fillet has at a vertex, by name.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VertexRadius {
    pub vertex: ElementReference<NamedModel<Design>, Vertex>,
    pub radius: f64,
}

impl FilletArgs {
    /// Round the edges named `edges` with `radius` — a number, or a
    /// formula's text — the same all along.
    pub fn constant(edges: Vec<String>, radius: impl Into<NumericInput>) -> Self {
        FilletArgs {
            edges: edge_refs(edges),
            radius: radius.into(),
            end_radius: None,
            vertex_radii: Vec::new(),
        }
    }

    /// Whether the radius changes along the edges.
    fn variable(&self) -> bool {
        self.end_radius.is_some() || !self.vertex_radii.is_empty()
    }

    /// The radii, as `reader` reads the parameters.
    fn radii(&self, reader: &Reader) -> GeopResult<Radii> {
        Ok(Radii {
            radius: self.radius.resolve(reader)?,
            end_radius: match &self.end_radius {
                Some(r) => Some(r.resolve(reader)?),
                None => None,
            },
            at_vertices: self
                .vertex_radii
                .iter()
                .map(|v| (v.vertex.name().to_string(), v.radius))
                .collect(),
        })
    }
}

/// The edges named `names`.
fn edge_refs(names: Vec<String>) -> Vec<ElementReference<NamedModel<Design>, Edge>> {
    (names.into_iter())
        .map(|name| ElementReference::new(model(), name))
        .collect()
}

/// The names of `edges`, as [`blend`] takes them.
fn edge_names(edges: &[ElementReference<NamedModel<Design>, Edge>]) -> Vec<String> {
    edges.iter().map(|e| e.name().to_string()).collect()
}

part_operation!(Fillet(FilletArgs), Chamfer(ChamferArgs));

impl PartOperation for Fillet {
    const KIND: &'static str = "fillet";
    const INFO: OperationInfo = OperationInfo::new(
        "Fillet",
        "Round a solid's edges.",
        OperationGroup::Features,
        OperationTier::Big,
    )
    .icon(icons::FILLET);
    type Args = FilletArgs;
    type Session = ();

    /// No edges yet, and a small radius.
    fn new_args(_: &Context) -> FilletArgs {
        FilletArgs::constant(Vec::new(), 0.1)
    }

    fn run(ctx: &mut StepContext, args: &FilletArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The edges, picked, and the radius — and, varying, the radius at the
    /// end and at vertices picked, each a number of its own.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &FilletArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, FilletArgs> {
        let before = &context.before;
        let mut f = EditForm::<Design, FilletArgs>::default();
        f.pick(
            "edges",
            "edges",
            before,
            &args.edges,
            true,
            |edit, edges| edit.args.edges = edges,
        );
        let radius = Number::numeric(&args.radius, "radius", before, Unit::Length);
        f.numeric("radius", radius.range(0.0, 1.0), |edit, radius| {
            edit.args.radius = radius
        })
        .checkbox(
            "variable",
            "variable radius",
            args.variable(),
            |edit, on| {
                let args = edit.args;
                args.end_radius = on.then(|| args.radius.clone());
                if !on {
                    args.vertex_radii.clear();
                }
            },
        );
        if !args.variable() {
            return f;
        }
        let end = args.end_radius.as_ref().unwrap_or(&args.radius);
        let end = Number::numeric(end, "end radius", before, Unit::Length);
        f.numeric("end_radius", end.range(0.0, 1.0), |edit, radius| {
            edit.args.end_radius = Some(radius)
        });
        let vertices: Vec<_> = (args.vertex_radii.iter())
            .map(|v| v.vertex.clone())
            .collect();
        f.pick(
            "radius_vertices",
            "radii at vertices",
            before,
            &vertices,
            true,
            |edit, picked| {
                // A new vertex starts at the radius as it is now.
                let args = edit.args;
                let radius = args.radius.resolve(before).unwrap_or(0.1);
                let old = std::mem::take(&mut args.vertex_radii);
                args.vertex_radii = (picked.into_iter())
                    .map(|vertex| VertexRadius {
                        radius: (old.iter().find(|v| v.vertex == vertex))
                            .map_or(radius, |v| v.radius),
                        vertex,
                    })
                    .collect();
            },
        );
        f.form.dialog.optional("radius_vertices");
        for (i, v) in args.vertex_radii.iter().enumerate() {
            let label = format!("radius at {}", v.vertex.name());
            let number = Number::new(label, v.radius, Unit::Length).range(0.0, 1.0);
            f.number(
                &format!("vertex_radius_{i}"),
                number,
                move |edit, radius| {
                    if let Some(v) = edit.args.vertex_radii.get_mut(i) {
                        v.radius = radius;
                    }
                },
            );
        }
        f
    }
}

impl ModelOperation for Fillet {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        args: &FilletArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("fillet", step)?;
        let radii = args.radii(reader)?;
        let shape = BlendShape::Fillet { radii };
        blend(model, &namer, &edge_names(&args.edges), &shape)?;
        Ok(())
    }
}

/// Bevels the edges named `edges` of a solid with a chamfer `distance` into
/// both faces — or, with `distance2`, `distance` into the face on the left
/// of each edge as it runs (seen from outside the solid) and `distance2`
/// into the one on its right — for the operation `C`: the solid is consumed
/// and the result named `chamfer(C)`, the bevel face of edge `E`
/// `chamfer(C,E,chamfer)`, named like a [`Fillet`]'s otherwise. Any edge is
/// bevelled: a straight one between two planes and a circle around its
/// faces' axis exactly, every other one along its tangent chain by chords
/// the distances into the faces, square to the edge.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Chamfer;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChamferArgs {
    /// The edges to bevel, all of one solid.
    pub edges: Vec<ElementReference<NamedModel<Design>, Edge>>,
    /// How far into the faces: a number, or a formula of the part's
    /// parameters.
    pub distance: NumericInput,
    /// The distance into the face on each edge's right, if it differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance2: Option<NumericInput>,
}

impl ChamferArgs {
    /// Bevel the edges named `edges` by `distance` — a number, or a
    /// formula's text — into both faces.
    pub fn even(edges: Vec<String>, distance: impl Into<NumericInput>) -> Self {
        ChamferArgs {
            edges: edge_refs(edges),
            distance: distance.into(),
            distance2: None,
        }
    }

    /// Into the face on the left and on the right of each edge, as
    /// `reader` reads the parameters.
    fn distances(&self, reader: &Reader) -> GeopResult<[f64; 2]> {
        let left = self.distance.resolve(reader)?;
        let right = match &self.distance2 {
            Some(d) => d.resolve(reader)?,
            None => left,
        };
        Ok([left, right])
    }
}

impl PartOperation for Chamfer {
    const KIND: &'static str = "chamfer";
    const INFO: OperationInfo = OperationInfo::new(
        "Chamfer",
        "Bevel a solid's edges.",
        OperationGroup::Features,
        OperationTier::Small,
    )
    .icon(icons::CHAMFER);
    type Args = ChamferArgs;
    type Session = ();

    /// No edges yet, and a small distance, the same into both faces.
    fn new_args(_: &Context) -> ChamferArgs {
        ChamferArgs::even(Vec::new(), 0.1)
    }

    fn run(ctx: &mut StepContext, args: &ChamferArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The edges, picked, the distance, and whether the second face gets a
    /// distance of its own.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ChamferArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, ChamferArgs> {
        let before = &context.before;
        let mut f = EditForm::<Design, ChamferArgs>::default();
        f.pick(
            "edges",
            "edges",
            before,
            &args.edges,
            true,
            |edit, edges| edit.args.edges = edges,
        );
        let distance = Number::numeric(&args.distance, "distance", before, Unit::Length);
        f.numeric("distance", distance.range(0.0, 1.0), |edit, distance| {
            edit.args.distance = distance
        });
        let two = args.distance2.is_some();
        f.checkbox("two_distances", "two distances", two, |edit, on| {
            edit.args.distance2 = on.then(|| edit.args.distance.clone());
        });
        if let Some(distance2) = &args.distance2 {
            let distance2 = Number::numeric(distance2, "distance 2", before, Unit::Length);
            f.numeric("distance2", distance2.range(0.0, 1.0), |edit, distance| {
                edit.args.distance2 = Some(distance)
            });
        }
        f
    }
}

impl ModelOperation for Chamfer {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        args: &ChamferArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("chamfer", step)?;
        let distances = args.distances(reader)?;
        let shape = BlendShape::Chamfer { distances };
        blend(model, &namer, &edge_names(&args.edges), &shape)?;
        Ok(())
    }
}
