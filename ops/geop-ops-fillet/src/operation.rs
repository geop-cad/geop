//! [`Fillet`]: round a solid's edges. [`Chamfer`]: bevel them. Both blend
//! the edges picked (see [`crate::blend`]).

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    editing::{Event, Form, Number, Unit, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::operation::Input;
use geop_ops::parameters::expressions;
use geop_ops::{
    Design, NamedModel, Namer, ParamValue,
    operation::{Build, BuildReader, EntityRef, Lookup, LegacyModelStep, Role, Scope, Seen, legacy_build_model},
    parameters::Formula,
    ui::{EditForm, formula_number},
};
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

    /// The radii, as `lookup` reads the parameters.
    fn radii<S: Scalar>(&self, lookup: &mut (impl Lookup<S> + ?Sized)) -> GeopResult<Radii> {
        Ok(Radii {
            radius: self.radius.evaluate(lookup)?,
            end_radius: match &self.end_radius {
                Some(r) => Some(r.evaluate(lookup)?),
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

impl LegacyModelStep for Fillet {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &FilletArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("fillet({step}, {args:?})");
        let namer = Namer::new("fillet", step)?;
        let radii = (args.radii(&mut Scope { model, reader })).with_context(ctx)?;
        let shape = BlendShape::Fillet { radii };
        blend(model, &namer, &args.edges, &shape).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for Fillet {
    const INFO: OperationInfo = OperationInfo::new(
        "fillet",
        "Fillet",
        "Round a solid's edges.",
        OperationGroup::Features,
        OperationTier::Big,
    )
    .icon(geop_ops::ui::icons::FILLET);
    type Args = FilletArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    fn formulas(args: &mut FilletArgs) -> Vec<&mut String> {
        expressions(std::iter::once(&mut args.radius).chain(args.end_radius.as_mut()))
    }

    /// No edges yet, and a small radius.
    fn new_args(_: &Context<Input, ParamValue>) -> FilletArgs {
        FilletArgs::constant(Vec::new(), 0.1)
    }

    fn run(step: &str, part: &mut Build, args: &FilletArgs) -> GeopResult<()> {
        legacy_build_model::<Self>(part, step, args)
    }

    fn presentation(
        context: &Context<Input, ParamValue>,
        args: &FilletArgs,
        _: &(),
    ) -> Form<Design> {
        fillet_form(context, args).into_form()
    }

    fn process_user_event(
        context: &Context<Input, ParamValue>,
        args: &FilletArgs,
        _: &(),
        event: Event<Design>,
    ) -> (FilletArgs, ()) {
        (fillet_form(context, args).apply(args, event), ())
    }
}

/// The form of a fillet: the edges, picked, and the radius — and, varying,
/// the radius at the end and at vertices picked, each a number of its own.
fn fillet_form<'a>(
    context: &'a Context<Input, ParamValue>,
    args: &FilletArgs,
) -> EditForm<'a, Design, FilletArgs> {
    let mut seen = Seen::new(context.before);
    let mut f = EditForm::<Design, FilletArgs>::default();
    let edges = EntityRef::of_names(EntityRef::edge, &args.edges);
    f.reference(
        "edges",
        "edges",
        edges,
        &[Role::Edge],
        None,
        true,
        |edit, picked| {
            edit.args.edges = EntityRef::names_of(EntityRef::edge, &picked);
        },
    );
    let radius = formula_number("radius", &args.radius, &mut seen, Unit::Length);
    f.formula("radius", radius.range(0.0, 1.0), |edit, radius| {
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
    let end = formula_number("end radius", end, &mut seen, Unit::Length);
    f.formula("end_radius", end.range(0.0, 1.0), |edit, radius| {
        edit.args.end_radius = Some(radius)
    });
    let names: Vec<String> = (args.vertex_radii.iter())
        .map(|v| v.vertex.clone())
        .collect();
    let vertices = EntityRef::of_names(EntityRef::vertex, &names);
    let label = "radii at vertices";
    f.reference(
        "radius_vertices",
        label,
        vertices,
        &[Role::Point],
        None,
        true,
        |edit, picked| {
            // A new vertex starts at the radius as it is now.
            let args = edit.args;
            let radius = (args.radius.evaluate(&mut Seen::new(context.before))).unwrap_or(0.1);
            let old = std::mem::take(&mut args.vertex_radii);
            args.vertex_radii = (EntityRef::names_of(EntityRef::vertex, &picked).into_iter())
                .map(|vertex| VertexRadius {
                    radius: (old.iter().find(|v| v.vertex == vertex)).map_or(radius, |v| v.radius),
                    vertex,
                })
                .collect();
        },
    );
    f.form.dialog.optional("radius_vertices");
    for (i, v) in args.vertex_radii.iter().enumerate() {
        let label = format!("radius at {}", v.vertex);
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
    pub edges: Vec<String>,
    /// How far into the faces: a number, or a formula of the part's
    /// parameters.
    pub distance: Formula,
    /// The distance into the face on each edge's right, if it differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance2: Option<Formula>,
}

impl ChamferArgs {
    /// Into the face on the left and on the right of each edge, as
    /// `lookup` reads the parameters.
    fn distances<S: Scalar>(&self, lookup: &mut (impl Lookup<S> + ?Sized)) -> GeopResult<[f64; 2]> {
        let left = self.distance.evaluate(lookup)?;
        let right = match &self.distance2 {
            Some(d) => d.evaluate(lookup)?,
            None => left,
        };
        Ok([left, right])
    }
}

impl LegacyModelStep for Chamfer {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &ChamferArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("chamfer({step}, {args:?})");
        let namer = Namer::new("chamfer", step)?;
        let distances = (args.distances(&mut Scope { model, reader })).with_context(ctx)?;
        let shape = BlendShape::Chamfer { distances };
        blend(model, &namer, &args.edges, &shape).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for Chamfer {
    const INFO: OperationInfo = OperationInfo::new(
        "chamfer",
        "Chamfer",
        "Bevel a solid's edges.",
        OperationGroup::Features,
        OperationTier::Small,
    )
    .icon(geop_ops::ui::icons::CHAMFER);
    type Args = ChamferArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    fn formulas(args: &mut ChamferArgs) -> Vec<&mut String> {
        expressions(std::iter::once(&mut args.distance).chain(args.distance2.as_mut()))
    }

    /// No edges yet, and a small distance, the same into both faces.
    fn new_args(_: &Context<Input, ParamValue>) -> ChamferArgs {
        ChamferArgs {
            edges: Vec::new(),
            distance: Formula::Plain(0.1),
            distance2: None,
        }
    }

    fn run(step: &str, part: &mut Build, args: &ChamferArgs) -> GeopResult<()> {
        legacy_build_model::<Self>(part, step, args)
    }

    /// The edges, picked, the distance, and whether the second face gets a
    /// distance of its own.
    fn presentation(
        context: &Context<Input, ParamValue>,
        args: &ChamferArgs,
        _: &(),
    ) -> Form<Design> {
        let mut seen = Seen::new(context.before);
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        let edges = EntityRef::of_names(EntityRef::edge, &args.edges);
        dialog.reference("edges", "edges", edges, &[Role::Edge], None, true);
        let distance = formula_number("distance", &args.distance, &mut seen, Unit::Length);
        (dialog.number("distance", distance.range(0.0, 1.0))).checkbox(
            "two_distances",
            "two distances",
            args.distance2.is_some(),
        );
        if let Some(distance2) = &args.distance2 {
            let distance2 = formula_number("distance 2", distance2, &mut seen, Unit::Length);
            dialog.number("distance2", distance2.range(0.0, 1.0));
        }
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &ChamferArgs,
        _: &(),
        event: Event<Design>,
    ) -> (ChamferArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), value) {
            ("edges", Value::Entities(picked)) => {
                args.edges = EntityRef::names_of(EntityRef::edge, &picked);
            }
            ("two_distances", Value::Bool(on)) => {
                args.distance2 = on.then(|| args.distance.clone());
            }
            (key, value) => match (key, Formula::from_value(&value)) {
                ("distance", Some(distance)) => args.distance = distance,
                ("distance2", Some(distance)) => args.distance2 = Some(distance),
                _ => {}
            },
        }
        (args, ())
    }
}
