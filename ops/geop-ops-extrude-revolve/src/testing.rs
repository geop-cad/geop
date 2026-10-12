//! What the tests of programs share that start from sketches: the
//! operations they are written in, drawing outlines, and the steps
//! sketching and extruding them. Only built for tests: with `testing`, or
//! in this crate's own.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar};
use geop_core_sketch::{CurveId, PointId};
use geop_ops::{
    Design, EntityRef, origin_plane,
    program::{ProgramFile, Registry},
};
use geop_ops_booleans::Combine;
use geop_ops_sketch::{AddSketch, AddSketchArgs, Sketch};

use crate::{Extent, Extents, Extrude, ExtrudeArgs, Loft, Output, Revolve, Sweep};

/// Sketching, and the solids and sheets made from sketches: the operations
/// a test's programs are written in, besides those it tests.
pub fn registry() -> GeopResult<Registry> {
    (Registry::default()
        .with(AddSketch)?
        .with(Extrude)?
        .with(Revolve)?)
    .with(Sweep)?
    .with(Loft)
}

/// A number of a test, as design data.
pub fn n(x: f64) -> Design {
    Design::from_f64(x)
}

/// A closed polygon through `corners`, one line per side: its points and
/// lines.
pub fn polygon(sketch: &mut Sketch, corners: &[[f64; 2]]) -> (Vec<PointId>, Vec<CurveId>) {
    let points: Vec<PointId> = corners
        .iter()
        .map(|c| sketch.add_point(n(c[0]), n(c[1])))
        .collect();
    let lines = (0..points.len())
        .map(|i| sketch.add_line(points[i], points[(i + 1) % points.len()]))
        .collect();
    (points, lines)
}

/// A sketch of nothing but the closed polygon through `corners` (see
/// [`polygon`]).
pub fn outline(corners: &[[f64; 2]]) -> Sketch {
    let mut sketch = Sketch::new();
    polygon(&mut sketch, corners);
    sketch
}

/// The rectangle from `(x0, y0)` to `(x1, y1)`.
pub fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Sketch {
    outline(&[[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
}

/// A sketch of nothing but the circle of `radius` around `center`.
pub fn disc(center: [f64; 2], radius: f64) -> Sketch {
    let mut sketch = Sketch::new();
    let c = sketch.add_point(n(center[0]), n(center[1]));
    sketch.add_circle(c, n(radius));
    sketch
}

/// The step drawing `sketch` on `plane`.
pub fn sketch(plane: EntityRef, sketch: Sketch) -> AddSketchArgs {
    AddSketchArgs {
        plane: Some(plane),
        sketch,
        ..Default::default()
    }
}

/// One side only, ending as `side1` says.
pub fn one_way(side1: Extent) -> Extents {
    Extents {
        side1,
        symmetric: false,
        side2: None,
        reversed: false,
    }
}

/// Both ways alike, `length` in all.
pub fn both_ways(length: f64) -> Extents {
    Extents {
        symmetric: true,
        ..Extents::blind(length)
    }
}

/// `first` one way and `second` the other.
pub fn two_ways(first: f64, second: f64) -> Extents {
    Extents {
        side2: Some(Extent::blind(second)),
        ..Extents::blind(first)
    }
}

/// The step extruding the sketch `sketch` as far as `extent` says into a
/// solid, combined as `combine` says.
pub fn extrusion(sketch: &str, extent: Extents, combine: Combine) -> ExtrudeArgs {
    ExtrudeArgs {
        sketch: sketch.into(),
        extent,
        output: Output::Solid(combine),
    }
}

/// Pushes `drawn` on `on` as `{name}_sketch`, and its extrude as far as
/// `extent` says as `name`, combined as `combine` says.
pub fn extrude_sketch(
    program: &mut ProgramFile,
    name: &str,
    on: EntityRef,
    drawn: Sketch,
    extent: Extents,
    combine: Combine,
) {
    let sketch_name = format!("{name}_sketch");
    program.push(&sketch_name, sketch(on, drawn));
    program.push(name, extrusion(&sketch_name, extent, combine));
}

/// A 2 x 2 x 1 box with a blind hole drilled into its top, in the steps the
/// example `box_with_drill_hole` has: a square sketched on the Z plane
/// (`outline`) and extruded up (`box`), and a circle of radius 0.4 around
/// its middle sketched on the box's end cap (`hole_sketch`) and cut half
/// way back into it (`hole`) — the drilled box is `extrude(hole)`.
pub fn drilled_box() -> ProgramFile {
    let mut program = ProgramFile::new();
    program.push(
        "outline",
        sketch(origin_plane(FrameAxis::Z), rectangle(0.0, 0.0, 2.0, 2.0)),
    );
    program.push(
        "box",
        extrusion("outline", Extents::blind(1.0), Combine::NewBody),
    );
    program.push(
        "hole_sketch",
        sketch(
            EntityRef::Face {
                name: "extrude(box,end)".into(),
            },
            disc([1.0, 1.0], 0.4),
        ),
    );
    program.push(
        "hole",
        extrusion(
            "hole_sketch",
            Extents::blind(-0.5),
            Combine::cut("extrude(box)"),
        ),
    );
    program
}
