//! [`AddSketch3d`]: draw a sketch in space.

use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_sketch::{
    ConstraintId, CurveId, PointId,
    space::{Constraint3d, CurveKind3d},
};
use geop_ops::{
    Context, Design, Library, Part,
    operation::{Aspects, EntityRef, Operation},
    ui::{CanvasEvent, Edit, Form},
};
use serde::{Deserialize, Serialize};

use crate::{
    Sketch3d,
    editor::{self, Sketch3dSession},
};

/// Adds a sketch in space to the part, named by the operation's id: points,
/// lines, arcs through three points and splines through points, with the
/// constraints of a 3-D sketch (see [`geop_core_sketch::space`]).
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

/// Something of the sketch given by an entity of the part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Reference3d {
    pub entity: EntityRef,
    #[serde(flatten)]
    pub target: Target,
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
        self.references.iter().find(|r| r.target == target)
    }

    /// Brings what the part gives the sketch up to date with `part`:
    /// moves its fixed points, hands in its reference curves' geometry,
    /// turns its directions. Forgets references whose target is gone.
    pub fn resolve<S: Scalar>(&mut self, part: &Part<S>) -> GeopResult<()> {
        let sketch = &self.sketch;
        self.references.retain(|r| match r.target {
            Target::Point { point } => sketch.points.contains_key(&point),
            Target::Curve { curve } => sketch.curves.contains_key(&curve),
            Target::Direction { constraint } => sketch.constraints.contains_key(&constraint),
        });
        for r in &self.references {
            let ctx = with_context!("the 3-D sketch's reference to {}", r.entity);
            let aspects = Aspects::of(&r.entity, part).with_context(ctx)?;
            let missing = |what: &str| GeopError::new(format!("{} is not {what}", r.entity));
            match r.target {
                Target::Point { point } => {
                    let at = aspects.point.ok_or_else(|| missing("a point"))?;
                    let p = self.sketch.points.get_mut(&point).expect("retained");
                    p.at = at.map(|c| c.cast());
                    p.fixed = true;
                }
                Target::Curve { curve } => {
                    let edge = aspects.curve.ok_or_else(|| missing("an edge"))?;
                    self.sketch
                        .references
                        .insert(curve, to_design(&edge).with_context(ctx)?);
                }
                Target::Direction { constraint } => {
                    let line = aspects.line.ok_or_else(|| missing("a line"))?;
                    let d = line.direction.map(|c| c.cast());
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
        let sketch = &self.sketch;
        self.references.retain(|r| match r.target {
            Target::Point { point } => sketch.points.contains_key(&point),
            Target::Curve { curve } => sketch.curves.contains_key(&curve),
            Target::Direction { constraint } => sketch.constraints.contains_key(&constraint),
        });
    }

    /// The sketch built on `part`: what the part gives it brought up to
    /// date, solved anew where its constraints no longer hold as drawn —
    /// a sketch as it was drawn is as it was solved — and every curve of it
    /// checked to build.
    pub fn build<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Sketch3d> {
        let mut args = self.clone();
        args.resolve(part)?;
        let mut sketch = args.sketch;
        if !sketch.check()?.converged {
            sketch.solve()?;
        }
        let geometry = sketch.enclose::<S>()?;
        for (&id, curve) in &sketch.curves {
            if curve.is_drawn() {
                sketch.curve_nurbs(id, &geometry)?;
            }
        }
        Ok(sketch)
    }
}

impl Operation for AddSketch3d {
    type Args = AddSketch3dArgs;
    type Session = Sketch3dSession;

    /// An empty sketch: the first click starts drawing.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> AddSketch3dArgs {
        AddSketch3dArgs::default()
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &AddSketch3dArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("add_sketch3d({operation_id})");
        let sketch = args.build(&part).with_context(ctx)?;
        part.add_sketch3d(sketch, operation_id).with_context(ctx)?;
        Ok(part)
    }

    /// The tools, the coordinates of a point, the constraints: see
    /// [`crate::editor`].
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &AddSketch3dArgs,
        session: &Sketch3dSession,
        selection: &[String],
    ) -> Form<'a, S, AddSketch3dArgs, Sketch3dSession> {
        editor::form(context.before, args, session, selection)
    }

    fn event<S: Scalar>(
        &self,
        context: Context<'_, S>,
        edit: Edit<'_, AddSketch3dArgs, Sketch3dSession>,
        event: &CanvasEvent<S>,
    ) {
        editor::event(context, edit, event);
    }
}
