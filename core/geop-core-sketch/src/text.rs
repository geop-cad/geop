//! Text in a sketch: the outlines of a string's glyphs, closed contours
//! of exact NURBS (see [`crate::CurveKind::Text`]).
//!
//! The font is Noto Sans Regular, by the Noto Project Authors, under the
//! SIL Open Font License 1.1 (`fonts/OFL.txt`; it reserves no font name).
//! It is embedded as a subset (`fonts/NotoSans-Regular-subset.ttf`, made
//! with `fontTools.subset`): Basic Latin, Latin-1, dashes, quotes, the
//! bullet, the ellipsis, the minus, the euro and the trade mark sign;
//! hinting and layout tables dropped, overlapping contours united
//! (`fontTools.ttLib.removeOverlaps`), so that no two contours of a glyph
//! cross and each is a loop of the sketch's profile. A character the font
//! has no glyph of is drawn as its missing glyph, a box.
//!
//! A TrueType outline is lines and quadratic Béziers between points of
//! whole font units, so it is a NURBS exactly. A run of quadratics each
//! starting halfway between the control points either side of it — a
//! TrueType "implied" point — is one quadratic B-spline on uniform knots,
//! the off-curve points its control points: so it is one piece, which
//! the extrusion makes one face, not a face per Bézier.

use geop_core_geometry::nurb_curve::NurbCurve2D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector2,
};
use ttf_parser::{Face, GlyphId, OutlineBuilder};

use crate::profile::spline_knots;

/// The font every sketch text is set in.
static FONT: &[u8] = include_bytes!("../fonts/NotoSans-Regular-subset.ttf");

/// A point of an outline, in font units.
pub type FontPoint = [f64; 2];

/// One piece of a contour: a polynomial B-spline of `degree` through
/// `points`, its control points, on clamped uniform knots — a line, a
/// quadratic or a cubic Bézier, or a run of quadratics joined smoothly.
#[derive(Clone, Debug, PartialEq)]
pub struct OutlinePiece {
    pub degree: usize,
    pub points: Vec<FontPoint>,
}

/// A closed contour of a glyph: pieces each starting where the one before
/// ends, the last ending where the first starts — at least two of them,
/// so every joint between them is one of the contour's own.
#[derive(Clone, Debug, PartialEq)]
pub struct Contour {
    pub pieces: Vec<OutlinePiece>,
}

fn font() -> GeopResult<Face<'static>> {
    Face::parse(FONT, 0).map_err(|e| GeopError::new(format!("the sketch font: {e}")))
}

/// The contours of `text` set in the sketch font, in font units: its first
/// baseline along the `x` axis from the origin, each line below the one
/// before by the font's line spacing.
pub fn outlines(text: &str) -> GeopResult<Vec<Contour>> {
    let face = font()?;
    let line =
        f64::from(face.ascender()) - f64::from(face.descender()) + f64::from(face.line_gap());
    let mut contours = Vec::new();
    let mut pen = [0.0, 0.0];
    for c in text.chars() {
        if c == '\n' {
            pen = [0.0, pen[1] - line];
            continue;
        }
        let glyph = face.glyph_index(c).unwrap_or(GlyphId(0));
        let mut outline = Outline {
            at: pen,
            contours: Vec::new(),
            current: Vec::new(),
            start: [0.0, 0.0],
            last: [0.0, 0.0],
        };
        face.outline_glyph(glyph, &mut outline);
        outline.close();
        contours.extend(outline.contours);
        pen[0] += f64::from(face.glyph_hor_advance(glyph).unwrap_or(0));
    }
    Ok(contours)
}

/// How tall the font's capitals are, in font units: what a text's height
/// measures.
fn capital_height() -> GeopResult<f64> {
    let face = font()?;
    let height = face
        .capital_height()
        .ok_or_else(|| GeopError::new("the sketch font gives no capital height"))?;
    Ok(f64::from(height))
}

/// `contours` placed in the sketch: their origin at `anchor`, scaled so a
/// capital is `height` tall, turned counter-clockwise by `angle` radians —
/// each a loop of NURBS pieces on `[0, 1]`.
pub fn place(
    contours: &[Contour],
    anchor: Vector2,
    height: Scalar,
    angle: Scalar,
) -> GeopResult<Vec<Vec<NurbCurve2D>>> {
    let scale = height.div(Scalar::from_f64(capital_height()?))?;
    let (cos, sin) = (angle.cos(), angle.sin());
    let at = |p: &FontPoint| {
        let local = Vector2::from_array([
            Scalar::from_f64(p[0]).mul(scale),
            Scalar::from_f64(p[1]).mul(scale),
        ]);
        anchor.add(&local.rotate(cos, sin))
    };
    (contours.iter())
        .map(|contour| {
            (contour.pieces.iter())
                .map(|piece| {
                    let n = piece.points.len();
                    NurbCurve2D::try_new(
                        piece.degree,
                        piece
                            .points
                            .iter()
                            .map(|p| at(p).homogeneous(Scalar::ONE))
                            .collect(),
                        spline_knots(n, piece.degree)?,
                    )
                })
                .collect()
        })
        .collect()
}

/// A glyph's outline as `ttf_parser` traces it, gathered into contours
/// (see [`Contour`]), moved by `at`.
struct Outline {
    at: FontPoint,
    contours: Vec<Contour>,
    /// The pieces of the contour being traced.
    current: Vec<OutlinePiece>,
    /// Where it started.
    start: FontPoint,
    /// Where it is.
    last: FontPoint,
}

impl Outline {
    fn point(&self, x: f32, y: f32) -> FontPoint {
        [self.at[0] + f64::from(x), self.at[1] + f64::from(y)]
    }

    /// Adds the piece of `degree` from where the outline is through
    /// `points`: a quadratic continuing a quadratic smoothly — starting
    /// halfway between the control points either side — joined to it, one
    /// whose control point is an end of it as the line it is, and nothing
    /// for a piece of no length.
    fn push(&mut self, degree: usize, points: &[FontPoint]) {
        let from = self.last;
        let to = *points.last().expect("a segment ends somewhere");
        if points.iter().all(|&p| p == from) {
            return;
        }
        if degree == 2 && (points[0] == from || points[0] == to) {
            return self.push(1, &[to]);
        }
        self.last = to;
        if degree == 2
            && let Some(previous) = self.current.last_mut()
            && previous.degree == 2
        {
            let control = previous.points[previous.points.len() - 2];
            let halfway = [
                (control[0] + points[0][0]) / 2.0,
                (control[1] + points[0][1]) / 2.0,
            ];
            if halfway == from {
                previous.points.pop();
                previous.points.extend_from_slice(points);
                return;
            }
        }
        let mut all = vec![from];
        all.extend_from_slice(points);
        self.current.push(OutlinePiece {
            degree,
            points: all,
        });
    }

    /// Ends the contour being traced: closed by a line back to its start,
    /// if it does not end there, and kept if it is a loop of at least two
    /// pieces — one closed piece split in two.
    fn close(&mut self) {
        if self.last != self.start {
            self.push(1, &[self.start]);
        }
        let mut pieces = std::mem::take(&mut self.current);
        if let [only] = &pieces[..] {
            pieces = halves(only);
        }
        if pieces.len() >= 2 {
            self.contours.push(Contour { pieces });
        }
    }
}

/// A piece cut in two, exactly: a run of quadratics at the joint in its
/// middle, a single Bézier at its middle by de Casteljau — in halves and
/// quarters of whole font units, which the format holds exactly.
fn halves(piece: &OutlinePiece) -> Vec<OutlinePiece> {
    let p = &piece.points;
    let mid = |a: FontPoint, b: FontPoint| [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
    let segments = p.len() - piece.degree;
    let (first, second) = if piece.degree == 2 && segments >= 2 {
        let m = segments / 2;
        let joint = mid(p[m], p[m + 1]);
        let mut first = p[..=m].to_vec();
        first.push(joint);
        let mut second = vec![joint];
        second.extend_from_slice(&p[m + 1..]);
        (first, second)
    } else {
        // De Casteljau at one half, for a single Bézier of any degree.
        let mut rows = vec![p.clone()];
        while rows.last().expect("a first row").len() > 1 {
            let row = rows.last().expect("a row");
            let next = row.windows(2).map(|w| mid(w[0], w[1])).collect();
            rows.push(next);
        }
        let first = rows.iter().map(|row| row[0]).collect();
        let second = rows
            .iter()
            .rev()
            .map(|row| *row.last().expect("a point"))
            .collect();
        (first, second)
    };
    [first, second]
        .into_iter()
        .map(|points| OutlinePiece {
            degree: piece.degree,
            points,
        })
        .collect()
}

impl OutlineBuilder for Outline {
    fn move_to(&mut self, x: f32, y: f32) {
        self.close();
        self.start = self.point(x, y);
        self.last = self.start;
    }

    fn line_to(&mut self, x: f32, y: f32) {
        let p = self.point(x, y);
        self.push(1, &[p]);
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        let points = [self.point(x1, y1), self.point(x, y)];
        self.push(2, &points);
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        let points = [self.point(x1, y1), self.point(x2, y2), self.point(x, y)];
        self.push(3, &points);
    }

    fn close(&mut self) {
        Outline::close(self);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_geometry::nurb_curve::loop_signed_area;

    fn closes(contour: &Contour) -> bool {
        let n = contour.pieces.len();
        let p = &contour.pieces;
        (0..n).all(|i| p[i].points.last() == p[(i + 1) % n].points.first())
    }

    /// An `o` is an outer contour and a hole, `B` one and two, an `i` two
    /// apart, a space nothing: each closed, of whole pieces.
    #[test]
    fn letters_have_their_contours() {
        for (text, count) in [("o", 2), ("B", 3), ("i", 2), (" ", 0), ("8", 3), ("A", 2)] {
            let contours = outlines(text).unwrap();
            assert_eq!(contours.len(), count, "{text:?}");
            assert!(contours.iter().all(closes), "{text:?}");
        }
    }

    /// The joined runs of quadratics make few pieces: an `o` is not a
    /// piece per Bézier.
    #[test]
    fn smooth_runs_are_one_piece() {
        let contours = outlines("o").unwrap();
        let pieces: usize = contours.iter().map(|c| c.pieces.len()).sum();
        assert!(pieces <= 8, "{contours:?}");
    }

    /// Placed, an outer contour and its hole wind opposite ways, by the
    /// exact area of the curves.
    #[test]
    fn a_hole_winds_against_its_outline() {
        let contours = outlines("o").unwrap();
        let origin = Vector2::from_array([Scalar::from_f64(10.0), Scalar::from_f64(-3.0)]);
        let placed = place(
            &contours,
            origin,
            Scalar::from_f64(5.0),
            Scalar::from_f64(0.3),
        )
        .unwrap();
        let areas: Vec<f64> = placed
            .iter()
            .map(|c| loop_signed_area(c).unwrap().to_f64())
            .collect();
        assert!(areas[0] * areas[1] < 0.0, "{areas:?}");
    }

    /// A second line starts below the first.
    #[test]
    fn a_new_line_goes_down() {
        let one = outlines("I").unwrap();
        let two = outlines("I\nI").unwrap();
        assert_eq!(two.len(), 2 * one.len());
        let lowest = |c: &[Contour]| {
            c.iter()
                .flat_map(|c| &c.pieces)
                .flat_map(|p| &p.points)
                .map(|p| p[1])
                .fold(f64::INFINITY, f64::min)
        };
        assert!(lowest(&two) < lowest(&one) - 500.0);
    }
}
