//! A drawing sheet: plain 2-D strokes and labels on paper, in millimetres
//! with `y` up, each on a layer — what the SVG and DXF writers write.
//!
//! The kernel's curves become lines, arcs and circles where they are those
//! exactly (see [`NurbCurve3D::as_line`], [`NurbCurve3D::as_arc`]), and
//! polylines through points of them otherwise.

use geop_core_geometry::nurb_curve::{NurbCurve2D, NurbCurve3D};
use geop_core_math::{
    geop_error::GeopResult,
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

/// A point on the paper.
pub type P = [f64; 2];

/// What a stroke or label is, which decides how it is drawn: the layers
/// of a DXF file, the classes of an SVG one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Layer {
    Visible,
    Hidden,
    Center,
    Dimension,
    Hatch,
    Border,
    /// Thin continuous lines: a cosmetic thread seen.
    Thread,
}

impl Layer {
    pub const ALL: [Layer; 7] = [
        Layer::Visible,
        Layer::Hidden,
        Layer::Center,
        Layer::Dimension,
        Layer::Hatch,
        Layer::Border,
        Layer::Thread,
    ];

    /// Its name, as a DXF layer and an SVG class.
    pub fn name(self) -> &'static str {
        match self {
            Layer::Visible => "VISIBLE",
            Layer::Hidden => "HIDDEN",
            Layer::Center => "CENTER",
            Layer::Dimension => "DIMENSIONS",
            Layer::Hatch => "HATCH",
            Layer::Border => "BORDER",
            Layer::Thread => "THREAD",
        }
    }
}

/// A shape on the paper.
#[derive(Clone, Debug, PartialEq)]
pub enum Shape {
    Line(P, P),
    /// Counter-clockwise around `center`, from the angle `start` to `end`,
    /// in radians.
    Arc {
        center: P,
        radius: f64,
        start: f64,
        end: f64,
    },
    Circle {
        center: P,
        radius: f64,
    },
    Polyline(Vec<P>),
    /// A filled polygon: an arrowhead.
    Filled(Vec<P>),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Stroke {
    pub layer: Layer,
    pub shape: Shape,
}

/// Which point of a label its position is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Anchor {
    Start,
    Middle,
    End,
}

/// A line of text, its baseline at `at`, turned `angle` degrees
/// counter-clockwise.
#[derive(Clone, Debug, PartialEq)]
pub struct Label {
    pub layer: Layer,
    pub at: P,
    pub height: f64,
    pub angle: f64,
    pub anchor: Anchor,
    pub text: String,
}

/// A sheet of paper, `width` x `height` millimetres, and what is on it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Sheet {
    pub width: f64,
    pub height: f64,
    pub strokes: Vec<Stroke>,
    pub labels: Vec<Label>,
}

impl Sheet {
    pub fn stroke(&mut self, layer: Layer, shape: Shape) {
        self.strokes.push(Stroke { layer, shape });
    }

    pub fn label(
        &mut self,
        layer: Layer,
        at: P,
        height: f64,
        anchor: Anchor,
        text: impl Into<String>,
    ) {
        self.labels.push(Label {
            layer,
            at,
            height,
            angle: 0.0,
            anchor,
            text: text.into(),
        });
    }
}

/// Points each polyline standing for a curve that is neither a line nor an
/// arc has, per polynomial piece of the curve.
const POINTS_PER_PIECE: usize = 16;

/// A curve recognised for what it is, in its own coordinates (the plane
/// `z = 0`).
enum Piece<S: Scalar> {
    Line(Vector3<S>, Vector3<S>),
    /// Counter-clockwise around `center` from `start` to `end` — all of it
    /// if they coincide.
    Arc {
        center: Vector3<S>,
        radius: S,
        start: Vector3<S>,
        end: Vector3<S>,
    },
    Other(Vec<P>),
}

/// A 2-D curve as a 3-D one in the plane `z = 0`.
pub(crate) fn lift<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<NurbCurve3D<S>> {
    NurbCurve3D::try_new(
        curve.degree,
        curve
            .control_points
            .iter()
            .map(|q| Vector4::from_array([q[0], q[1], S::ZERO, q[2]]))
            .collect(),
        curve.knot_vector.clone(),
    )
}

/// `curve` recognised: a line or an arc where it is one exactly (see
/// [`NurbCurve3D::as_line`], [`NurbCurve3D::as_arc`]), else points of it.
fn piece<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<Piece<S>> {
    let lifted = lift(curve)?;
    let (t0, t1) = lifted.domain();
    if lifted.as_line()?.is_some() {
        return Ok(Piece::Line(lifted.evaluate(t0)?, lifted.evaluate(t1)?));
    }
    if let Some(arc) = lifted.as_arc()? {
        let (start, end) = if arc.circle.normal[2].to_f64() > 0.0 {
            (arc.start, arc.end)
        } else {
            (arc.end, arc.start)
        };
        return Ok(Piece::Arc {
            center: arc.circle.center,
            radius: arc.circle.radius,
            start,
            end,
        });
    }
    let mut points = Vec::new();
    for piece in curve.bezier_pieces()? {
        let (a, b) = piece.domain();
        let first = if points.is_empty() { 0 } else { 1 };
        for k in first..=POINTS_PER_PIECE {
            let t = match k {
                k if k == POINTS_PER_PIECE => b,
                k => {
                    let f = S::from_f64(k as f64 / POINTS_PER_PIECE as f64);
                    a.add(b.sub(a).mul(f)).sharpen()
                }
            };
            let p = piece.evaluate(t)?;
            points.push([p[0].to_f64(), p[1].to_f64()]);
        }
    }
    Ok(Piece::Other(points))
}

/// `a` and `b` as one, if they continue each other: two lines along one
/// line meeting at an end, or two arcs of one circle, one starting where
/// the other ends.
fn join<S: Scalar>(a: &Piece<S>, b: &Piece<S>) -> Option<Piece<S>> {
    match (a, b) {
        (Piece::Line(a0, a1), Piece::Line(b0, b1)) => {
            let (da, db) = (a1.sub(a0), b1.sub(b0));
            if !da.prod_cross(&db)[2].could_be_equal(S::ZERO) {
                return None;
            }
            let ends = [
                (a0, a1, b0, b1),
                (a0, a1, b1, b0),
                (a1, a0, b0, b1),
                (a1, a0, b1, b0),
            ];
            ends.into_iter().find_map(|(far_a, near_a, near_b, far_b)| {
                // Meeting end to end, not overlapping: the far ends lie on
                // either side of the meeting point.
                (near_a.could_be_equal(near_b)
                    && far_a
                        .sub(near_a)
                        .prod_dot(&far_b.sub(near_b))
                        .definitely_less(S::ZERO))
                .then(|| Piece::Line(*far_a, *far_b))
            })
        }
        (
            Piece::Arc {
                center: ca,
                radius: ra,
                start: sa,
                end: ea,
            },
            Piece::Arc {
                center: cb,
                radius: rb,
                start: sb,
                end: eb,
            },
        ) if ca.could_be_equal(cb) && ra.could_be_equal(*rb) => {
            let arc = |start: &Vector3<S>, end: &Vector3<S>| Piece::Arc {
                center: ca.union(cb),
                radius: ra.union(*rb),
                start: *start,
                end: *end,
            };
            if ea.could_be_equal(sb) {
                Some(arc(sa, eb))
            } else if eb.could_be_equal(sa) {
                Some(arc(sb, ea))
            } else {
                None
            }
        }
        _ => None,
    }
}

/// The curves `curves` on the paper, through `place` from their own
/// coordinates at `scale`: each a line, an arc or a circle where it is one
/// exactly, those continuing each other on one layer joined, and every
/// other curve a polyline through points of it.
pub fn strokes_of<S: Scalar>(
    curves: &[(Layer, &NurbCurve2D<S>)],
    place: &impl Fn(P) -> P,
    scale: f64,
) -> GeopResult<Vec<Stroke>> {
    let mut pieces: Vec<(Layer, Piece<S>)> = curves
        .iter()
        .map(|(layer, curve)| Ok((*layer, piece(curve)?)))
        .collect::<GeopResult<_>>()?;
    let mut i = 0;
    while i < pieces.len() {
        let joined = (0..pieces.len()).find_map(|j| {
            if j == i || pieces[j].0 != pieces[i].0 {
                return None;
            }
            join(&pieces[i].1, &pieces[j].1).map(|p| (j, p))
        });
        match joined {
            Some((j, p)) => {
                pieces[i].1 = p;
                pieces.remove(j);
                if j < i {
                    i -= 1;
                }
            }
            None => i += 1,
        }
    }
    let at = |p: &Vector3<S>| place([p[0].to_f64(), p[1].to_f64()]);
    Ok(pieces
        .into_iter()
        .map(|(layer, piece)| {
            let shape = match piece {
                Piece::Line(a, b) => Shape::Line(at(&a), at(&b)),
                Piece::Arc {
                    center,
                    radius,
                    start,
                    end,
                } => {
                    let c = at(&center);
                    let radius = radius.to_f64() * scale;
                    if start.could_be_equal(&end) {
                        Shape::Circle { center: c, radius }
                    } else {
                        let angle = |p: P| (p[1] - c[1]).atan2(p[0] - c[0]);
                        Shape::Arc {
                            center: c,
                            radius,
                            start: angle(at(&start)),
                            end: angle(at(&end)),
                        }
                    }
                }
                Piece::Other(points) => Shape::Polyline(points.into_iter().map(place).collect()),
            };
            Stroke { layer, shape }
        })
        .collect())
}
