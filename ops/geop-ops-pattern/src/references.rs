//! What the operations of this crate move along or turn about, from
//! whichever element of the part gives it: [`TurnAxis`], a line or what a
//! round element turns around, and [`PatternDirection`], a line or a
//! plane's normal. A mirror's plane is a [`geop_ops_sketch::SketchPlane`]:
//! a flat face, a datum plane, a sketch's plane.

use geop_core_editor::one_of;
use geop_core_geometry::shape::Axis;
use geop_core_math::{geop_error::GeopResult, vector::Vector3};
use geop_core_program::{element::ElementReference, reader::Reader, values::Design};
use geop_ops_datums::Datums;
use geop_ops_model::{Line, NamedModel, Plane, Round};
use geop_ops_sketch::PlacedSketch;

one_of! {
    /// An axis to turn about: a line — a straight edge, a datum axis or a
    /// frame's, a sketch's line — or what a round element turns around: a
    /// circular edge, a cylindrical or conical face, a sketch's circle or
    /// arc.
    pub enum TurnAxis -> Axis<Design> {
        Edge(ElementReference<NamedModel<Design>, Line>),
        Round(ElementReference<NamedModel<Design>, Round>),
        Datum(ElementReference<Datums, Line>),
        Sketch(ElementReference<PlacedSketch<Design>, Line>),
        SketchRound(ElementReference<PlacedSketch<Design>, Round>),
    }
}

one_of! {
    /// A direction to go along: a line — a straight edge, a datum axis or
    /// a frame's, a sketch's line — or a plane, along its normal: a flat
    /// face, a datum plane, a sketch's plane.
    pub enum PatternDirection {
        Edge(ElementReference<NamedModel<Design>, Line>),
        Face(ElementReference<NamedModel<Design>, Plane>),
        Datum(ElementReference<Datums, Line>),
        DatumPlane(ElementReference<Datums, Plane>),
        Sketch(ElementReference<PlacedSketch<Design>, Line>),
        SketchPlane(ElementReference<PlacedSketch<Design>, Plane>),
    }
}

impl PatternDirection {
    /// The unit vector it goes along, as `reader` reads the part: a line's
    /// direction, a plane's normal.
    pub fn direction(&self, reader: &Reader) -> GeopResult<Vector3<Design>> {
        Ok(match self {
            PatternDirection::Edge(line) => reader.retrieve_element(line)?.direction,
            PatternDirection::Datum(line) => reader.retrieve_element(line)?.direction,
            PatternDirection::Sketch(line) => reader.retrieve_element(line)?.direction,
            PatternDirection::Face(plane) => *reader.retrieve_element(plane)?.w(),
            PatternDirection::DatumPlane(plane) => *reader.retrieve_element(plane)?.w(),
            PatternDirection::SketchPlane(plane) => *reader.retrieve_element(plane)?.w(),
        })
    }
}
