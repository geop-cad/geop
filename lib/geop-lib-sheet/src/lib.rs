//! A [`Sheet`]: plain 2-D strokes and labels on paper, in millimetres with
//! `y` up, each on a layer — what a drawing and a flat pattern are laid out
//! as, and written in a [`Format`]: SVG ([`to_svg`]) or DXF ([`to_dxf`]).
//!
//! The kernel's curves become lines, arcs and circles where they are those
//! exactly (see [`NurbCurve3D::as_line`], [`NurbCurve3D::as_arc`]), and
//! polylines through points of them otherwise.
//!
//! Text in an SVG sheet, or in any other XML a writer makes, is escaped
//! by [`xml_escape`].

// Library code reports what goes wrong as an error, never by panicking;
// tests may unwrap.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::todo,
        clippy::unimplemented,
        clippy::unreachable
    )
)]

mod dxf;
mod svg;

pub use dxf::to_dxf;
pub use svg::to_svg;

use geop_core_geometry::nurb_curve::NurbCurve2D;
use geop_core_math::{
    geop_error::GeopResult,
    scalars::Scalar,
    solvers::repeat::{Step, repeat},
    vector::Vector3,
};

/// A point on the paper.
pub type P = [f64; 2];

/// `text` fit for XML: for an element's content, or an attribute's value
/// between either kind of quotes.
pub fn xml_escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            _ => out.push(c),
        }
    }
    out
}

/// What a stroke or label is, which decides how it is drawn: a layer of a
/// DXF file, a class of an SVG one. The sheet knows no layer of its own:
/// whoever draws on it defines the layers it draws on (a drawing its
/// visible and hidden lines, a flat pattern what is cut and where it is
/// bent).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Layer {
    /// Its name, as a DXF layer and an SVG class.
    pub name: &'static str,
    /// Its strokes' width, in millimetres.
    pub width: f64,
    pub line_type: LineType,
    /// Its colour, by AutoCAD colour index.
    pub color: u8,
}

/// How a layer's strokes are dashed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineType {
    Continuous,
    /// Short dashes: what is hidden.
    Hidden,
    /// Long and short dashes: an axis, a centre, a bend.
    Center,
}

impl LineType {
    pub const ALL: [LineType; 3] = [LineType::Continuous, LineType::Hidden, LineType::Center];

    /// Its name, as a DXF line type.
    pub fn name(self) -> &'static str {
        match self {
            LineType::Continuous => "CONTINUOUS",
            LineType::Hidden => "HIDDEN",
            LineType::Center => "CENTER",
        }
    }

    /// How a DXF file describes it.
    pub fn description(self) -> &'static str {
        match self {
            LineType::Continuous => "Solid line",
            LineType::Hidden => "Hidden __ __ __",
            LineType::Center => "Center ____ _ ____",
        }
    }

    /// Its dash pattern: lengths in millimetres, negative for gaps.
    pub fn dashes(self) -> &'static [f64] {
        match self {
            LineType::Continuous => &[],
            LineType::Hidden => &[3.0, -1.5],
            LineType::Center => &[8.0, -1.5, 1.5, -1.5],
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

/// A file format a sheet is written in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Format {
    Svg,
    Dxf,
}

impl Format {
    /// The format named by its files' extension, `svg` or `dxf`, if it is
    /// one.
    pub fn of_extension(extension: &str) -> Option<Format> {
        match extension.to_ascii_lowercase().as_str() {
            "svg" => Some(Format::Svg),
            "dxf" => Some(Format::Dxf),
            _ => None,
        }
    }

    pub fn extension(self) -> &'static str {
        match self {
            Format::Svg => "svg",
            Format::Dxf => "dxf",
        }
    }

    /// Its media type, to save it as.
    pub fn media_type(self) -> &'static str {
        match self {
            Format::Svg => "image/svg+xml",
            Format::Dxf => "image/vnd.dxf",
        }
    }
}

/// A sheet of paper, `width` x `height` millimetres, and what is on it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Sheet {
    pub width: f64,
    pub height: f64,
    /// The layers it is drawn on, in the order they are written, whether
    /// anything is on them or not. A layer drawn on but not listed here is
    /// written after them (see [`Sheet::layers`]).
    pub layers: Vec<Layer>,
    pub strokes: Vec<Stroke>,
    pub labels: Vec<Label>,
}

impl Sheet {
    /// Its [`Sheet::layers`], followed by every other layer a stroke or a
    /// label is on, in the order first drawn on: each layer once.
    pub fn layers(&self) -> Vec<Layer> {
        let mut layers = self.layers.clone();
        let drawn =
            (self.strokes.iter().map(|s| s.layer)).chain(self.labels.iter().map(|l| l.layer));
        for layer in drawn {
            if !layers.contains(&layer) {
                layers.push(layer);
            }
        }
        layers
    }

    /// It written as `format`.
    pub fn write(&self, format: Format) -> String {
        match format {
            Format::Svg => to_svg(self),
            Format::Dxf => to_dxf(self),
        }
    }

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

/// `curve` recognised: a line or an arc where it is one exactly (see
/// [`NurbCurve3D::as_line`], [`NurbCurve3D::as_arc`]), else points of it.
fn piece<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<Piece<S>> {
    let lifted = curve.lifted()?;
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
                    S::fraction(a, b, f)
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
                .then_some(Piece::Line(*far_a, *far_b))
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

/// `x` as written on a sheet: to `places` decimals, without trailing zeros.
pub fn decimal(x: f64, places: usize) -> String {
    let s = format!("{x:.places$}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.into() }
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
    // Each piece joined with every one that continues it, until none does:
    // a step either joins two pieces or moves on to the next, and every
    // join takes one piece away, so there are at most twice as many steps
    // as pieces, and one more that finds the last piece passed.
    let mut i = 0;
    let most = 2 * pieces.len() + 1;
    repeat(most, || -> GeopResult<_> {
        if i >= pieces.len() {
            return Ok(Step::Done(()));
        }
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
        Ok(Step::Again)
    })?
    .done("joining the strokes that continue each other")?;
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
