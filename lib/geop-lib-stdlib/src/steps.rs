//! The steps the standard parts are built of, alike for every family:
//! profiles revolved around the `z` axis, outlines extruded by formulas
//! of the size — an extrusion's length, a socket's depth from a datum
//! plane at its floor — hexagons cut to a revolved envelope, and the named
//! datums parts are mated by.
//!
//! What only needs a height is extruded that far, a formula, with no
//! boolean. What is chamfered is cut to an envelope instead: a nut's
//! corners are where its hexagon, extruded through all of it, leaves a
//! double cone.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{element::ElementReference, program::Program};
use geop_core_sketch::CurveId;
use geop_ops_model::datums::{Datums, datums, origin_axis, origin_plane};
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, Output, RevolveArgs, RevolveAxis};
use geop_ops_model::Combine;
use geop_ops_parameters::{
    Material, Parameter, ParameterKind, Parameters, parameters::Row, set_parameters,
};
use geop_ops_reference_geometry::{AddDatumArgs, Construction, Selected};
use geop_ops_sketch::{SketchPlane, sketch_element, whole_sketch};

use super::{drawing::Drawing, tables::Table};

/// The name of the table parameter every family's size is chosen by.
pub const SIZE: &str = "size";

/// `table` as the parameter [`SIZE`].
pub fn size(table: Table) -> Parameter {
    Parameter {
        name: SIZE.into(),
        kind: ParameterKind::Table {
            columns: table.columns.iter().map(|c| c.to_string()).collect(),
            rows: table
                .rows
                .into_iter()
                .map(|(name, values)| Row { name, values })
                .collect(),
            selected: table.selected.into(),
        },
    }
}

/// A family's program, of `material`, in `color` — none for the viewer's
/// own — and designed by the parameters `values`, the size among them.
pub fn family(material: &str, color: Option<&str>, values: Vec<Parameter>) -> Program {
    let mut program = Program::new();
    set_parameters(
        &mut program,
        Parameters {
            material: Material::named(material),
            color: color.map(Into::into),
            values,
            ..Parameters::default()
        },
    );
    program
}

/// The value of `column` of the size chosen, as a formula reads it.
pub fn col(column: &str) -> String {
    format!("{SIZE}.{column}")
}

/// The plane profiles are drawn in: the origin's `yz` plane, its `x` the
/// radius out from the `z` axis — along `y` — and its `y` along `z`.
pub fn profile_plane() -> SketchPlane {
    SketchPlane::Datum(origin_plane(FrameAxis::X))
}

/// Where an outline extruded up through a solid is drawn: below it.
pub const BELOW: &str = "below";

/// The plane outlines are drawn in, around the `z` axis: the origin's `xy`
/// plane.
pub fn outline_plane() -> SketchPlane {
    SketchPlane::Datum(origin_plane(FrameAxis::Z))
}

/// The datum plane `name`, as a sketch is drawn on it.
pub fn datum_plane(name: &str) -> SketchPlane {
    SketchPlane::Datum(ElementReference::new(datums(), name))
}

/// The datum `name`, as a datum is built from it.
pub fn datum(name: &str) -> Selected {
    Selected::Datum(ElementReference::<Datums, _>::new(datums(), name))
}

/// The face a full turn of the profile line `curve` of the sketch
/// `sketch`, revolved by step `revolve`, sweeps: one per quarter.
pub fn swept(revolve: &str, sketch: &str, curve: CurveId) -> Vec<String> {
    (0..4)
        .map(|q| format!("revolve({revolve},{sketch},{curve},q{q})"))
        .collect()
}

/// What a profile turns around: its own line on the axis, where it
/// touches it, or else the `z` axis.
pub enum Around {
    Line(CurveId),
    ZAxis,
}

/// Draws `drawing` in the profile plane as the sketch `sketch`, and turns
/// it a full turn around the `z` axis into the solid `revolve(<id>)` —
/// combined as `combine` says.
pub fn revolve(
    program: &mut Program,
    id: &str,
    sketch: &str,
    drawing: Drawing,
    around: Around,
    combine: Combine,
) {
    program.push(sketch, drawing.on(profile_plane()));
    let axis = match around {
        Around::Line(curve) => RevolveAxis::Sketch(sketch_element(sketch, curve)),
        Around::ZAxis => RevolveAxis::Datum(origin_axis(FrameAxis::Z)),
    };
    program.push(
        id,
        RevolveArgs {
            sketch: whole_sketch(sketch),
            axis: Some(axis),
            extent: Extents::blind(360.0),
            output: Output::Solid(combine),
        },
    );
}

/// Which way through a solid an outline is extruded from the `xy` plane:
/// up, for a solid wholly above it, or both ways, for one it crosses.
#[derive(Clone, Copy, PartialEq)]
pub enum Through {
    Up,
    Both,
}

/// Draws `drawing` on `plane` as the sketch `sketch`, and extrudes it as
/// far as `extent` says — its lengths formulas of the size — combined as
/// `combine` says: `extrude(<id>)`.
pub fn extrude(
    program: &mut Program,
    id: &str,
    sketch: &str,
    (drawing, plane): (Drawing, SketchPlane),
    extent: Extents,
    combine: Combine,
) {
    program.push(sketch, drawing.on(plane));
    program.push(
        id,
        ExtrudeArgs {
            sketch: whole_sketch(sketch),
            extent,
            output: Output::Solid(combine),
        },
    );
}

/// Draws `drawing` in the outline plane as the sketch `sketch`, and
/// extrudes it through all of the solid `target`, `through` it, keeping
/// where they overlap: `extrude(<id>)`.
pub fn cut_to(
    program: &mut Program,
    id: &str,
    sketch: &str,
    drawing: Drawing,
    target: &str,
    through: Through,
) {
    let plane = match through {
        Through::Both => outline_plane(),
        Through::Up => {
            if program.index_of(BELOW).is_err() {
                program.push(
                    BELOW,
                    AddDatumArgs {
                        selection: vec![datum(origin_plane(FrameAxis::Z).name())],
                        construction: Construction::Offset {
                            distance: (-1.0).into(),
                        },
                    },
                );
            }
            datum_plane(BELOW)
        }
    };
    let extent = Extents {
        side1: Extent::ThroughAll,
        symmetric: through == Through::Both,
        side2: None,
        reversed: false,
    };
    let combine = Combine::intersect(target);
    extrude(program, id, sketch, (drawing, plane), extent, combine)
}

/// A regular hexagon around the origin, `across` its flats — which face
/// `±x` — a formula.
pub fn hexagon(drawing: &mut Drawing, across: &str) -> GeopResult<Vec<CurveId>> {
    let flat = format!("({across}) / 2");
    let half = format!("({across}) / (2 * sqrt(3))");
    let corner = format!("({across}) / sqrt(3)");
    let neg = |f: &str| format!("-{f}");
    drawing.polygon(&[
        [flat.clone(), neg(&half)],
        [flat.clone(), half.clone()],
        ["0".into(), corner.clone()],
        [neg(&flat), half.clone()],
        [neg(&flat), neg(&half)],
        ["0".into(), neg(&corner)],
    ])
}

/// The datum `axis`: the `z` axis every standard part turns around or
/// runs along — what a concentric mate picks.
pub fn axis_datum(program: &mut Program) {
    program.push(
        "axis",
        AddDatumArgs {
            selection: vec![datum(origin_axis(FrameAxis::Z).name())],
            construction: Construction::AlongLine {},
        },
    );
}

/// The datum plane `id` normal to `z`, `height` — a formula of the size —
/// up it: through the origin, the face a part sits on, put there — what a
/// coincident mate picks — or a plane to draw an outline on.
pub fn plane_datum(program: &mut Program, id: &str, height: &str) {
    offset_datum(program, id, origin_plane(FrameAxis::Z).name(), height);
}

/// The datum plane `id` parallel to the datum plane `plane`, `distance` —
/// a formula of the size — along its normal: for a part that runs along
/// `z`, a face it sits on beside the axis.
pub fn offset_datum(program: &mut Program, id: &str, plane: &str, distance: &str) {
    program.push(
        id,
        AddDatumArgs {
            selection: vec![datum(plane)],
            construction: Construction::Offset {
                distance: distance.into(),
            },
        },
    );
}
