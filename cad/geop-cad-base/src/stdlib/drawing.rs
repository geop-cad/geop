//! [`Drawing`]: a sketch whose every point is placed by formulas of the
//! program's parameters — what makes one file draw every size of a part.

use geop_core_program::{
    numeric::{NumericInput, evaluate},
    program::Program,
    values::Design,
};
use geop_ops_parameters::parameter_numbers;
use std::collections::BTreeMap;

use geop_core_geometry::nurb_curve::bezier_knots;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_sketch::{ConstraintId, CurveId, CurveKind, PointId, SplineShape};
use geop_ops_sketch::{AddSketchArgs, Constraint, Sketch, SketchPlane};

/// A sketch drawn point by point, each point placed by two formulas — its
/// `x` and `y` from a fixed origin — so that every point follows the
/// parameters on its own, and solving the sketch is solving a linear
/// system with one answer. Curves only join points; arcs take a radius.
///
/// It is drawn where the parameters as defined put it, so a program built
/// as it is needs no solving.
pub struct Drawing {
    sketch: Sketch,
    origin: PointId,
    formulas: BTreeMap<ConstraintId, NumericInput>,
    /// The numbers the parameters come to as defined: where to draw.
    numbers: BTreeMap<String, f64>,
}

impl Drawing {
    /// A drawing for a step of `program`, drawn with its parameters as it
    /// defines them.
    pub fn new(program: &Program) -> GeopResult<Self> {
        let numbers = parameter_numbers(program)?;
        let mut sketch = Sketch::new();
        let origin = sketch.add_fixed_point(Design::ZERO, Design::ZERO);
        Ok(Self {
            sketch,
            origin,
            formulas: BTreeMap::new(),
            numbers,
        })
    }

    /// The value of `formula` with the parameters as defined.
    fn value(&self, formula: &str) -> GeopResult<f64> {
        evaluate(formula, |name| self.numbers.get(name).copied())
    }

    /// Adds the dimension `constraint`, given by `formula` — a plain
    /// number is a value, not a formula.
    fn dimension(&mut self, constraint: Constraint, formula: &str) {
        let id = self.sketch.constrain(constraint);
        let input = NumericInput::from(formula);
        if input.plain().is_none() {
            self.formulas.insert(id, input);
        }
    }

    /// The point at `(x, y)`, each a formula.
    pub fn point(&mut self, x: impl AsRef<str>, y: impl AsRef<str>) -> GeopResult<PointId> {
        let (x, y) = (x.as_ref(), y.as_ref());
        let at = [self.value(x)?, self.value(y)?].map(Design::from_f64);
        let p = self.sketch.add_point(at[0], at[1]);
        let origin = self.origin;
        self.dimension(
            Constraint::DistanceX {
                a: origin,
                b: p,
                value: at[0],
            },
            x,
        );
        self.dimension(
            Constraint::DistanceY {
                a: origin,
                b: p,
                value: at[1],
            },
            y,
        );
        Ok(p)
    }

    pub fn line(&mut self, a: PointId, b: PointId) -> CurveId {
        self.sketch.add_line(a, b)
    }

    /// The Bézier curve of the control points `control`, of degree one
    /// less than there are: a spline on one span, through its first and
    /// last point.
    pub fn bezier(&mut self, control: Vec<PointId>) -> CurveId {
        let n = control.len();
        let knots = bezier_knots(n - 1);
        self.sketch.add_curve(CurveKind::Spline {
            control_points: control,
            shape: Some(SplineShape {
                degree: n - 1,
                knots,
                weights: vec![Design::ONE; n],
            }),
        })
    }

    /// A closed polygon through `corners`, each `[x, y]` formulas: its
    /// lines, the first from the first corner to the second.
    pub fn polygon(&mut self, corners: &[[String; 2]]) -> GeopResult<Vec<CurveId>> {
        let points = corners
            .iter()
            .map(|[x, y]| self.point(x, y))
            .collect::<GeopResult<Vec<_>>>()?;
        Ok((0..points.len())
            .map(|i| self.line(points[i], points[(i + 1) % points.len()]))
            .collect())
    }

    /// The shorter arc of `radius` from `start` to `end`, turning
    /// counter-clockwise if `ccw`.
    pub fn arc(
        &mut self,
        start: PointId,
        end: PointId,
        radius: &str,
        ccw: bool,
    ) -> GeopResult<CurveId> {
        let r = self.value(radius)?;
        let at = |p: PointId| {
            let point = &self.sketch.points[&p];
            [point.x.to_f64(), point.y.to_f64()]
        };
        let ([x0, y0], [x1, y1]) = (at(start), at(end));
        let chord = (x1 - x0).hypot(y1 - y0);
        if chord > 2.0 * r {
            return Err(GeopError::new(format!(
                "no arc of radius {radius:?} = {r} joins points {chord} apart"
            )));
        }
        let sweep = 2.0 * (chord / (2.0 * r)).asin();
        let arc = self.sketch.add_arc(
            start,
            end,
            Design::from_f64(if ccw { sweep } else { -sweep }),
        );
        self.dimension(
            Constraint::Radius {
                curve: arc,
                value: Design::from_f64(r),
            },
            radius,
        );
        Ok(arc)
    }

    /// A circle around `center` of `diameter`.
    pub fn circle(&mut self, center: PointId, diameter: &str) -> GeopResult<CurveId> {
        let d = self.value(diameter)?;
        let circle = self.sketch.add_circle(center, Design::from_f64(d / 2.0));
        self.dimension(
            Constraint::Diameter {
                curve: circle,
                value: Design::from_f64(d),
            },
            diameter,
        );
        Ok(circle)
    }

    /// The fixed point every other is placed from, at `(0, 0)`.
    pub fn origin(&self) -> PointId {
        self.origin
    }

    /// The sketch, on `plane`: drawn where its dimensions put it, so it
    /// needs no solving — and a build with the parameters as defined then
    /// solves none (see `AddSketch`), which is what keeps a large profile, a
    /// rack's teeth, cheap to place.
    pub fn on(self, plane: SketchPlane) -> AddSketchArgs {
        AddSketchArgs {
            formulas: self.formulas,
            ..AddSketchArgs::on(plane, self.sketch)
        }
    }
}
