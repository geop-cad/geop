//! [`AddSketch3d`]: draw a sketch in space.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::CanvasEvent,
    one_of, part_operation,
    part_operation::PartOperation,
};
use geop_core_program::{
    element::{ElementReference, PickedElement},
    part::StepContext,
    reader::Reader,
    values::Design,
};
use std::collections::{BTreeMap, BTreeSet};

use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve3D},
    shape::Axis,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_sketch::{
    ConstraintId, CurveId, PointId,
    space::{Constraint3d, CurveKind3d},
};
use geop_core_topology::build::{BodySpec, EdgeSpec};
use geop_ops_model::datums::Datums;
use geop_ops_model::sketch::PlacedSketch;
use geop_ops_model::{
    BodyNames, Chain, Curve, Line, ModelOperation, NamedModel, Namer, Point, build_model,
};
use serde::{Deserialize, Serialize};

use crate::{
    Sketch3d,
    editor::{self, Sketch3dSession},
    placed::{PlacedSketch3d, sketch3d},
};

one_of! {
    /// A point of the part a 3-D sketch's fixed point is at: a vertex, a
    /// datum point or a frame's origin, a sketch's point.
    pub enum PointSource -> Vector3<Design> {
        Vertex(ElementReference<NamedModel<Design>, Point>),
        Datum(ElementReference<Datums, Point>),
        Sketch(ElementReference<PlacedSketch<Design>, Point>),
    }
}

one_of! {
    /// A curve of the part a 3-D sketch's reference curve is: an edge, a
    /// sketch's curve.
    pub enum CurveSource -> Chain<NurbCurve3D<Design>> {
        Edge(ElementReference<NamedModel<Design>, Curve>),
        Sketch(ElementReference<PlacedSketch<Design>, Curve>),
    }
}

one_of! {
    /// A line of the part a 3-D sketch's direction runs along: a straight
    /// edge, a datum axis or a frame's, a sketch's line.
    pub enum LineSource -> Axis<Design> {
        Edge(ElementReference<NamedModel<Design>, Line>),
        Datum(ElementReference<Datums, Line>),
        Sketch(ElementReference<PlacedSketch<Design>, Line>),
    }
}

/// Adds a sketch in space to the part, named by the operation's id: points,
/// lines, arcs through three points and splines through points, with the
/// constraints of a 3-D sketch (see [`geop_core_sketch::space`]). The step
/// `K` defines the part's 3-D sketch `K` (see [`sketch3d`]),
/// a path as a whole, and builds it as a wire: an edge for every curve
/// drawn, construction curves aside, and a vertex for every point a curve
/// of it ends at or that stands on its own. Points made one by a
/// coincidence are one vertex, named after the lowest of their ids: the
/// edge of curve `c3` is named `sketch3d(K,c3)`, the vertex of point `p1`
/// `sketch3d(K,p1)`.
///
/// What it is given from the part — a point at a vertex or a datum point, a
/// point on an edge, a line parallel to an axis or a straight edge — is
/// brought up to date every time it is built (see [`Reference3d`]), so the
/// sketch follows the part.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddSketch3d;

/// What a [`Reference3d`] gives the sketch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "target", rename_all = "snake_case")]
pub enum Target {
    /// Where the fixed point `point` is: the entity's point — a vertex, a
    /// datum point, a frame's origin, a sketch's point.
    Point { point: PointId },
    /// The geometry of the reference curve `curve`: the entity's — an edge.
    Curve { curve: CurveId },
    /// The direction of the constraint `constraint` — a
    /// [`Constraint3d::ParallelTo`] or [`Constraint3d::TangentTo`]: the
    /// entity's line — an axis, a straight edge, a sketch's line.
    Direction { constraint: ConstraintId },
}

/// Something of the sketch given by an element of the part: where a fixed
/// point is, what a reference curve is, which way a direction runs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "target", rename_all = "snake_case")]
pub enum Reference3d {
    Point {
        point: PointId,
        from: PointSource,
    },
    Curve {
        curve: CurveId,
        from: CurveSource,
    },
    Direction {
        constraint: ConstraintId,
        from: LineSource,
    },
}

impl Reference3d {
    /// What of the sketch it gives.
    pub fn target(&self) -> Target {
        match *self {
            Reference3d::Point { point, .. } => Target::Point { point },
            Reference3d::Curve { curve, .. } => Target::Curve { curve },
            Reference3d::Direction { constraint, .. } => Target::Direction { constraint },
        }
    }

    /// What of the part gives it, as a pick names it.
    pub fn source(&self) -> PickedElement {
        use geop_core_editor::selectable::Selectable;
        match self {
            Reference3d::Point { from, .. } => from.picked(),
            Reference3d::Curve { from, .. } => from.picked(),
            Reference3d::Direction { from, .. } => from.picked(),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddSketch3dArgs {
    /// The sketch as drawn.
    pub sketch: Sketch3d,
    /// What of it the part gives.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub references: Vec<Reference3d>,
}

/// `curve` in the program's scalar.
fn to_design<S: Scalar>(curve: &NurbCurve3D<S>) -> GeopResult<NurbCurve3D<Design>> {
    NurbCurve::try_new(
        curve.degree,
        curve
            .control_points
            .iter()
            .map(|p| p.map(|c| c.cast()))
            .collect(),
        curve.knot_vector.iter().map(|k| k.cast()).collect(),
    )
}

impl AddSketch3dArgs {
    /// The reference that gives `target`, if any.
    pub fn reference_of(&self, target: Target) -> Option<&Reference3d> {
        self.references.iter().find(|r| r.target() == target)
    }

    /// Forgets the references whose target the sketch no longer has.
    fn forget_dead_references(&mut self) {
        let sketch = &self.sketch;
        self.references.retain(|r| match r.target() {
            Target::Point { point } => sketch.points.contains_key(&point),
            Target::Curve { curve } => sketch.curves.contains_key(&curve),
            Target::Direction { constraint } => sketch.constraints.contains_key(&constraint),
        });
    }

    /// Brings what the part gives the sketch up to date with the part
    /// `reader` reads: moves its fixed points, hands in its reference
    /// curves' geometry, turns its directions. Forgets references whose
    /// target is gone.
    pub fn resolve(&mut self, reader: &Reader) -> GeopResult<()> {
        self.forget_dead_references();
        for r in &self.references {
            let ctx = with_context!("the 3-D sketch's reference to {}", r.source().label());
            match r {
                Reference3d::Point { point, from } => {
                    let at = from.resolve(reader).with_context(ctx)?;
                    let p = (self.sketch.points.get_mut(point)).ok_or_else(|| {
                        GeopError::new(format!("the sketch has no point {point}"))
                    })?;
                    p.at = at;
                    p.fixed = true;
                }
                Reference3d::Curve { curve, from } => {
                    let chain = from.resolve(reader).with_context(ctx)?;
                    let [edge] = chain.curves.as_slice() else {
                        return Err(GeopError::new("it is more than one curve")).with_context(ctx);
                    };
                    self.sketch
                        .references
                        .insert(*curve, to_design(edge).with_context(ctx)?);
                }
                &Reference3d::Direction {
                    constraint,
                    ref from,
                } => {
                    let d = from.resolve(reader).with_context(ctx)?.direction;
                    match self.sketch.constraints.get_mut(&constraint) {
                        Some(
                            Constraint3d::ParallelTo { direction, .. }
                            | Constraint3d::TangentTo { direction, .. },
                        ) => *direction = d,
                        other => {
                            return Err(GeopError::new(format!(
                                "a direction is given to constraint {constraint} = {other:?}, which has none"
                            )))
                            .with_context(ctx);
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Forgets the reference curves no constraint uses any more, and the
    /// references to what is gone.
    pub fn tidy(&mut self) {
        let unused: Vec<CurveId> = self
            .sketch
            .curves
            .iter()
            .filter(|(id, c)| {
                c.kind == CurveKind3d::Reference
                    && !self
                        .sketch
                        .constraints
                        .values()
                        .any(|k| k.curves().contains(id))
            })
            .map(|(&id, _)| id)
            .collect();
        self.sketch.remove(&[], &unused, &[]);
        self.forget_dead_references();
    }

    /// The sketch built on the part `reader` reads: what the part gives it
    /// brought up to date, solved anew where its constraints no longer hold
    /// as drawn — a sketch as it was drawn is as it was solved — and every
    /// curve of it checked to build.
    pub fn build(&self, reader: &Reader) -> GeopResult<Sketch3d> {
        let mut args = self.clone();
        args.resolve(reader)?;
        let mut sketch = args.sketch;
        if !sketch.check()?.converged {
            sketch.solve()?;
        }
        let geometry = sketch.enclose::<Design>()?;
        for (&id, curve) in &sketch.curves {
            if curve.is_drawn() {
                sketch.curve_nurbs(id, &geometry)?;
            }
        }
        Ok(sketch)
    }
}

/// The 3-D sketch the step `step` draws with `args`, on the part before it,
/// and the names of the edges it is built as.
fn draw(reader: &Reader, (step, args): &(String, AddSketch3dArgs)) -> GeopResult<PlacedSketch3d> {
    let ctx = with_context!("add_sketch3d({step})");
    let sketch = args.build(reader).with_context(ctx)?;
    let namer = Namer::new("sketch3d", step).with_context(ctx)?;
    let edges = (drawn(&sketch).with_context(ctx)?.iter())
        .map(|(id, _)| namer.name(&[&id.to_string()]))
        .collect();
    Ok(PlacedSketch3d {
        name: step.clone(),
        sketch,
        edges,
    })
}

/// The curves of `sketch` its wire has an edge for — those drawn, but for
/// construction curves — and their ends.
fn drawn(sketch: &Sketch3d) -> GeopResult<Vec<(CurveId, (PointId, PointId))>> {
    (sketch.drawn_curves())
        .map(|(&id, c)| {
            let ends = (c.endpoints())
                .ok_or_else(|| GeopError::new(format!("the drawn curve {id} has no ends")))?;
            Ok((id, ends))
        })
        .collect()
}

/// The wire `sketch` is built as, and the names of its vertices and edges
/// (see [`AddSketch3d`]): `None` for a sketch with no point to build.
fn wire_of<S: Scalar>(
    sketch: &Sketch3d,
    namer: &Namer,
) -> GeopResult<Option<(BodySpec<S>, BodyNames)>> {
    let class = sketch.point_classes();
    let geometry = sketch.enclose::<S>()?;
    let drawn = drawn(sketch)?;
    let used = |p: &PointId| {
        (sketch.curves.values()).any(|c| c.points().iter().any(|q| class[q] == class[p]))
    };
    let ends = drawn.iter().flat_map(|&(_, (start, end))| [start, end]);
    let alone = sketch.points.keys().copied().filter(|p| !used(p));
    let points: BTreeSet<PointId> = ends.chain(alone).map(|p| class[&p]).collect();
    if points.is_empty() {
        return Ok(None);
    }
    // A vertex encloses every point of its class.
    let vertices: BTreeMap<PointId, Vector3<S>> = (points.into_iter())
        .map(|rep| {
            let at = (sketch.points.keys())
                .filter(|p| class[*p] == rep)
                .map(|p| geometry.points[p])
                .reduce(|a, b| a.union(&b))
                .ok_or_else(|| GeopError::new(format!("the point class of {rep} is empty")))?;
            Ok((rep, at))
        })
        .collect::<GeopResult<_>>()?;
    let index: BTreeMap<PointId, usize> =
        vertices.keys().enumerate().map(|(i, &p)| (p, i)).collect();
    let edges = (drawn.iter())
        .map(|&(id, (start, end))| {
            Ok(EdgeSpec {
                curve: NurbCurve::join(&sketch.curve_nurbs(id, &geometry)?)?,
                start: index[&class[&start]],
                end: index[&class[&end]],
            })
        })
        .collect::<GeopResult<_>>()?;
    let names = BodyNames {
        vertices: (vertices.keys())
            .map(|p| namer.name(&[&p.to_string()]))
            .collect(),
        edges: (drawn.iter())
            .map(|(id, _)| namer.name(&[&id.to_string()]))
            .collect(),
        faces: Vec::new(),
        solid: None,
    };
    let spec = BodySpec {
        vertices: vertices.into_values().collect(),
        edges,
        faces: Vec::new(),
        shells: Vec::new(),
        solid: false,
    };
    Ok(Some((spec, names)))
}

/// What a 3-D sketch step keeps while it is edited: the drawing's session,
/// and the keys of the visuals selected.
pub type Sketch3dEditing = FormState<Sketch3dSession>;

part_operation!(AddSketch3d(AddSketch3dArgs));

impl PartOperation for AddSketch3d {
    const KIND: &'static str = "add_sketch3d";
    const INFO: OperationInfo = OperationInfo::new(
        "3-D sketch",
        "Draw points, lines, arcs and splines in space: paths to sweep along.",
        OperationGroup::Sketch,
        OperationTier::Small,
    )
    .icon(crate::icons::ADD_SKETCH3D);
    type Args = AddSketch3dArgs;
    type Session = Sketch3dSession;

    /// An empty sketch: the first click starts drawing.
    fn new_args(_: &Context) -> AddSketch3dArgs {
        AddSketch3dArgs::default()
    }

    /// Defines the 3-D sketch named after the step, and builds it as a
    /// wire.
    fn run(ctx: &mut StepContext, args: &AddSketch3dArgs) -> GeopResult<()> {
        let step = ctx.step();
        ctx.produce(&sketch3d(step), &(step.to_string(), args.clone()), draw)?;
        build_model::<Self>(ctx, args)
    }

    /// The tools, the coordinates of a point, the constraints: see
    /// [`crate::editor`].
    fn form<'a>(
        context: &'a Context<'a>,
        args: &AddSketch3dArgs,
        editing: &Sketch3dEditing,
    ) -> EditForm<'a, Design, AddSketch3dArgs, Sketch3dSession> {
        editor::form(&context.before, args, &editing.session, &editing.selection)
    }

    fn canvas(
        context: &Context,
        edit: Edit<'_, AddSketch3dArgs, Sketch3dSession>,
        event: &CanvasEvent<Design>,
    ) {
        editor::event(&context.before, context.scene, edit, event)
    }
}

impl ModelOperation for AddSketch3d {
    /// Builds the 3-D sketch the step defined as a wire (see
    /// [`AddSketch3d`]).
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        _: &AddSketch3dArgs,
    ) -> GeopResult<()> {
        let whole = ElementReference::<PlacedSketch3d, PlacedSketch3d>::whole(sketch3d(step));
        let placed = reader.retrieve_element(&whole)?;
        let namer = Namer::new("sketch3d", step)?;
        if let Some((spec, names)) = wire_of(&placed.sketch, &namer)? {
            model.build_body(spec, names)?;
        }
        Ok(())
    }
}
