//! What instances are laid out along, round or at, from whichever element
//! gives it — of the part itself, or of a part placed in it, where it is
//! placed: [`TurnAxis`], a line or what a round element turns around;
//! [`PatternDirection`], a line or a plane's normal; [`PointsRef`], a point
//! or a sketch's points of its own.

use std::collections::BTreeSet;

use geop_core_editor::one_of;
use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Motion,
    vector::Vector3,
    with_context,
};
use geop_core_program::{element::ElementReference, placed::InInstance, reader::Reader};
use geop_core_sketch::CurveKind;

use crate::{
    Line, NamedModel, Plane, Point, Round,
    datums::Datums,
    parameters::located,
    sketch::{PlacedSketch, SketchRef},
};

/// The motion placing what is in the instance `placed` is of where the
/// part `reader` reads has it: none for the part's own.
fn motion_of<T>(reader: &Reader, placed: &InInstance<T>) -> GeopResult<Motion> {
    Ok(located(reader, &placed.instance)?.motion())
}

one_of! {
    /// An axis to turn about: a line — a straight edge, a datum axis or a
    /// frame's, a sketch's line — or what a round element turns around: a
    /// circular edge, a cylindrical or conical face, a sketch's circle or
    /// arc.
    pub enum TurnAxis -> Axis {
        Edge(ElementReference<NamedModel, Line>),
        Round(ElementReference<NamedModel, Round>),
        Datum(ElementReference<Datums, Line>),
        Sketch(ElementReference<PlacedSketch, Line>),
        SketchRound(ElementReference<PlacedSketch, Round>),
    }
}

impl TurnAxis {
    /// The axis, where the part `reader` reads has it: of the part itself,
    /// or of a part placed in it, moved where that is.
    pub fn placed(&self, reader: &Reader) -> GeopResult<Axis> {
        let axis = self.resolve_placed(reader)?;
        Ok(axis.element.transform(&motion_of(reader, &axis)?))
    }
}

one_of! {
    /// A direction to go along: a line — a straight edge, a datum axis or
    /// a frame's, a sketch's line — or a plane, along its normal: a flat
    /// face, a datum plane, a sketch's plane.
    pub enum PatternDirection {
        Edge(ElementReference<NamedModel, Line>),
        Face(ElementReference<NamedModel, Plane>),
        Datum(ElementReference<Datums, Line>),
        DatumPlane(ElementReference<Datums, Plane>),
        Sketch(ElementReference<PlacedSketch, Line>),
        SketchPlane(ElementReference<PlacedSketch, Plane>),
    }
}

impl PatternDirection {
    /// The unit vector it goes along, where the part `reader` reads has
    /// it: a line's direction, a plane's normal — of the part itself, or of
    /// a part placed in it, turned as that is.
    pub fn direction(&self, reader: &Reader) -> GeopResult<Vector3> {
        let line = |placed: InInstance<Axis>| -> GeopResult<Vector3> {
            Ok(motion_of(reader, &placed)?.rotate(&placed.element.direction))
        };
        let plane = |placed: InInstance<_>| -> GeopResult<Vector3> {
            let motion = motion_of(reader, &placed)?;
            let frame: geop_core_math::primitives::CoordinateSystem = placed.element;
            Ok(motion.rotate(frame.w()))
        };
        match self {
            PatternDirection::Edge(r) => line(reader.retrieve_placed(r)?),
            PatternDirection::Datum(r) => line(reader.retrieve_placed(r)?),
            PatternDirection::Sketch(r) => line(reader.retrieve_placed(r)?),
            PatternDirection::Face(r) => plane(reader.retrieve_placed(r)?),
            PatternDirection::DatumPlane(r) => plane(reader.retrieve_placed(r)?),
            PatternDirection::SketchPlane(r) => plane(reader.retrieve_placed(r)?),
        }
    }
}

one_of! {
    /// Where something goes: a vertex, a datum point or a frame's origin,
    /// a sketch's point — or a sketch, standing for its points that no
    /// line, arc or spline runs through.
    pub enum PointsRef {
        Vertex(ElementReference<NamedModel, Point>),
        Datum(ElementReference<Datums, Point>),
        SketchPoint(ElementReference<PlacedSketch, Point>),
        Sketch(SketchRef),
    }
}

/// The points `refs` stand for, where the part `reader` reads has them —
/// of the part itself, or of a part placed in it, moved where that is —
/// each by a name of its own: a sketch's point `K,p3` for the point `p3`
/// of the sketch `K`, else the point's name. A sketch stands for its points
/// no line, arc or spline runs through: points placed on their own, and
/// circles' centres. One picked twice is refused.
pub fn resolve_points(reader: &Reader, refs: &[PointsRef]) -> GeopResult<Vec<(String, Vector3)>> {
    let mut found = Vec::new();
    for point in refs {
        let ctx = with_context!("the point {point:?}");
        match point {
            PointsRef::Sketch(sketch) => {
                let placed = reader.retrieve_placed(sketch).with_context(ctx)?;
                let motion = motion_of(reader, &placed).with_context(ctx)?;
                let (name, sketch) = (&placed.element.name, &placed.element.sketch);
                let on_curves: BTreeSet<_> = sketch
                    .curves
                    .values()
                    .filter(|c| !matches!(c.kind, CurveKind::Circle { .. }))
                    .flat_map(|c| c.points())
                    .collect();
                let geometry = sketch.enclose().with_context(ctx)?;
                let before = found.len();
                for (&id, p) in &sketch.points {
                    if !p.fixed && !on_curves.contains(&id) {
                        let at = placed.element.plane.uv_to_xyz(&geometry.points[&id]);
                        found.push((format!("{name},{id}"), motion.apply(&at)));
                    }
                }
                if found.len() == before {
                    return Err(GeopError::new(format!(
                        "sketch {name:?} has no points of its own: place points in it, or pick them one by one"
                    )));
                }
            }
            PointsRef::SketchPoint(at) => {
                let whole = SketchRef::whole(at.target().clone());
                let sketch = &reader.retrieve_placed(&whole).with_context(ctx)?.element.name;
                let p = reader.retrieve_placed(at).with_context(ctx)?;
                let motion = motion_of(reader, &p).with_context(ctx)?;
                found.push((format!("{sketch},{}", at.name()), motion.apply(&p.element)));
            }
            PointsRef::Vertex(at) => {
                let p = reader.retrieve_placed(at).with_context(ctx)?;
                let motion = motion_of(reader, &p).with_context(ctx)?;
                found.push((at.name().to_string(), motion.apply(&p.element)));
            }
            PointsRef::Datum(at) => {
                let p = reader.retrieve_placed(at).with_context(ctx)?;
                let motion = motion_of(reader, &p).with_context(ctx)?;
                found.push((at.name().to_string(), motion.apply(&p.element)));
            }
        }
    }
    let mut seen = BTreeSet::new();
    for (scope, _) in &found {
        if !seen.insert(scope) {
            return Err(GeopError::new(format!("the point {scope} is picked twice")));
        }
    }
    Ok(found)
}
