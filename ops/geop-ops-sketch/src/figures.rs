//! Figures drawn into a sketch the way the examples and tests write them
//! down: closed polygons, rectangles and circles, fully constrained where
//! the figure says so, and a sketch solved once it is drawn; sketches of
//! one figure, unconstrained, as a test draws a body's outline; and the
//! planes a test sketches on. Every number is design data ([`n`]).

use geop_core_math::{primitives::FrameAxis, scalars::Scalar};
use geop_core_program::{element::ElementReference, values::Design};
use geop_core_sketch::{CurveId, PointId};
use geop_ops_model::datums::origin_plane;
use geop_ops_model::model;

use crate::{Constraint, Sketch, SketchPlane};

/// `[x, y]`, as a figure is written down.
pub type P2 = [f64; 2];

/// A number of a figure, as design data.
pub fn n(x: f64) -> Design {
    Design::from_f64(x)
}

// ── Planes to sketch on ─────────────────────────────────────────────────────

/// The plane of the origin normal to `normal`, to sketch on.
pub fn on_origin(normal: FrameAxis) -> SketchPlane {
    SketchPlane::Datum(origin_plane(normal))
}

/// The flat face named `face`, to sketch on.
pub fn on_face(face: &str) -> SketchPlane {
    SketchPlane::Face(ElementReference::new(model(), face))
}

// ── Sketches of one figure ──────────────────────────────────────────────────

/// A sketch of nothing but the closed polygon through `corners` (see
/// [`polygon`]).
pub fn outline(corners: &[P2]) -> Sketch {
    let mut sketch = Sketch::new();
    polygon(&mut sketch, corners);
    sketch
}

/// A sketch of nothing but the rectangle from `(x0, y0)` to `(x1, y1)`.
pub fn rectangle_outline(x0: f64, y0: f64, x1: f64, y1: f64) -> Sketch {
    outline(&[[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
}

/// A sketch of nothing but the circle of `radius` around `center`.
pub fn disc(center: P2, radius: f64) -> Sketch {
    let mut sketch = Sketch::new();
    let c = sketch.add_point(n(center[0]), n(center[1]));
    sketch.add_circle(c, n(radius));
    sketch
}

// ── Constrained figures ─────────────────────────────────────────────────────

/// A closed polygon through `corners`, one line per side: its points and
/// lines.
pub fn polygon(sketch: &mut Sketch, corners: &[P2]) -> (Vec<PointId>, Vec<CurveId>) {
    let points: Vec<PointId> = corners
        .iter()
        .map(|c| sketch.add_point(n(c[0]), n(c[1])))
        .collect();
    let lines = (0..points.len())
        .map(|i| sketch.add_line(points[i], points[(i + 1) % points.len()]))
        .collect();
    (points, lines)
}

/// Solves `sketch`, which has to be fully constrained: a figure that does
/// not solve is a mistake in how it was written down.
#[allow(
    clippy::expect_used,
    clippy::panic,
    reason = "a figure that does not solve is a bug"
)]
pub fn solved(mut sketch: Sketch) -> Sketch {
    let report = sketch.solve().expect("example sketches are valid");
    assert!(
        report.converged,
        "example sketch does not solve: {report:?}"
    );
    sketch
}

/// A `width` x `depth` rectangle with its first corner at `origin`, drawn
/// roughly and fully constrained: its lines, from the one along `x` at
/// `origin` round.
pub fn rectangle(sketch: &mut Sketch, origin: P2, width: f64, depth: f64) -> Vec<CurveId> {
    let [x, y] = origin;
    // Deliberately a little off: the constraints decide the shape.
    let (p, l) = polygon(
        sketch,
        &[
            [x + 0.05, y - 0.02],
            [x + width, y + 0.03],
            [x + width - 0.04, y + depth],
            [x, y + depth + 0.01],
        ],
    );
    sketch.constrain(Constraint::Fix {
        point: p[0],
        x: n(x),
        y: n(y),
    });
    sketch.constrain(Constraint::Horizontal { line: l[0] });
    sketch.constrain(Constraint::Horizontal { line: l[2] });
    sketch.constrain(Constraint::Vertical { line: l[1] });
    sketch.constrain(Constraint::Vertical { line: l[3] });
    sketch.constrain(Constraint::Length {
        curve: l[0],
        value: n(width),
    });
    sketch.constrain(Constraint::Length {
        curve: l[1],
        value: n(depth),
    });
    l
}

/// A circle of `radius` around `center`, fully constrained.
pub fn circle(sketch: &mut Sketch, center: P2, radius: f64) -> CurveId {
    let c = sketch.add_point(n(center[0]), n(center[1]));
    let circle = sketch.add_circle(c, n(radius * 1.1));
    sketch.constrain(Constraint::Fix {
        point: c,
        x: n(center[0]),
        y: n(center[1]),
    });
    sketch.constrain(Constraint::Radius {
        curve: circle,
        value: n(radius),
    });
    circle
}
