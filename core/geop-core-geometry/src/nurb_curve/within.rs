//! Where a plane curve lies with respect to a convex polygon: wholly in
//! it, touching it, or clear of it — what a box drawn over a sketch
//! selects. Decided on the curve itself by subdivision, never from
//! samples: a piece whose control points all lie in the polygon lies in
//! it, and one whose control points all lie past one side lies clear of it
//! (the convex hull property, the weights being positive).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    polygon::Convex,
    scalars::Scalar,
    solvers::subdivide::{Order, Searched, Split, subdivide},
    vector::Vector2,
};

use super::{NurbCurve, dehomogenize};

/// Where a curve lies with respect to a convex polygon (see
/// [`NurbCurve::within`]). Where a piece of the curve runs along the
/// polygon's boundary closer than the search resolves, it counts as
/// touching, and as no point outside.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Within {
    /// No point of the curve lies definitely outside the polygon.
    pub wholly: bool,
    /// Some point of the curve could lie in the polygon.
    pub touches: bool,
}

impl NurbCurve<3> {
    /// Where the curve lies with respect to `region`: split until each
    /// piece lies in it, lies past one of its sides, or has a point on
    /// either side — or is smaller than `min_size`, along the boundary.
    /// Visits at most `max_nodes` pieces: exhausting them is an error, the
    /// answer being unknown.
    pub fn within(
        &self,
        region: &Convex,
        min_size: Scalar,
        max_nodes: usize,
    ) -> GeopResult<Within> {
        let (mut inside, mut outside) = (false, false);
        let pieces = self.bezier_pieces()?;
        let searched = subdivide(pieces, Order::DepthFirst, max_nodes, |piece, _| {
            if inside && outside {
                return Ok(Split::Done);
            }
            let points: Vec<Vector2> = dehomogenize(&piece.control_points);
            // Past one side with every control point: clear of the polygon.
            let clear = (region.sides.iter()).any(|(n, c)| {
                (points.iter()).all(|p| n.prod_dot(p).sub(*c).definitely_greater(Scalar::ZERO))
            });
            if clear {
                outside = true;
                return Ok(Split::Done);
            }
            if points.iter().all(|p| region.could_contain(p)) {
                inside = true;
                return Ok(Split::Done);
            }
            // A Bézier piece passes through its end control points.
            for end in [points.first(), points.last()].into_iter().flatten() {
                match region.could_contain(end) {
                    true => inside = true,
                    false => outside = true,
                }
            }
            let [x, y, _] = *piece.aabb();
            if !x.width().max(y.width()).definitely_greater(min_size) {
                // Along the boundary, closer than resolved.
                inside = true;
                return Ok(Split::Done);
            }
            let (left, right) = piece.split_mid()?;
            Ok::<_, GeopError>(Split::Halves(left, right))
        })?;
        if let Searched::Exhausted { pending } = searched {
            return Err(GeopError::new(format!(
                "NurbCurve::within: exhausted max_nodes={max_nodes} with {pending} pieces left, \
                 min_size={min_size:?}, region={region:?}"
            )));
        }
        Ok(Within {
            wholly: !outside,
            touches: inside,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurb_curve::NurbCurve2D;

    fn v(x: f64, y: f64) -> Vector2 {
        Vector2::from_array([Scalar::from_f64(x), Scalar::from_f64(y)])
    }

    fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> Convex {
        Convex::new(&[v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y1)]).unwrap()
    }

    fn within(curve: &NurbCurve2D, region: &Convex) -> Within {
        curve
            .within(region, Scalar::from_f64(1e-9), 10_000)
            .unwrap()
    }

    /// The quarter arc of the unit circle from `(1, 0)` to `(0, 1)`.
    fn quarter() -> NurbCurve2D {
        let w = Scalar::ONE.div(Scalar::TWO).unwrap().sqrt().unwrap();
        NurbCurve2D::arc(v(1.0, 0.0), v(1.0, 1.0), v(0.0, 1.0), w).unwrap()
    }

    #[test]
    fn a_line_inside_across_and_clear() {
        let line = NurbCurve2D::line(v(1.0, 1.0), v(2.0, 2.0)).unwrap();
        let both = Within {
            wholly: true,
            touches: true,
        };
        assert_eq!(within(&line, &square(0.0, 0.0, 3.0, 3.0)), both);
        let across = within(&line, &square(1.5, 0.0, 3.0, 3.0));
        assert!(!across.wholly && across.touches);
        // Through the box with both ends outside it.
        let through = NurbCurve2D::line(v(-1.0, 0.5), v(4.0, 0.5)).unwrap();
        let crossed = within(&through, &square(0.0, 0.0, 3.0, 3.0));
        assert!(!crossed.wholly && crossed.touches);
        let clear = within(&line, &square(5.0, 5.0, 6.0, 6.0));
        assert!(!clear.wholly && !clear.touches);
    }

    /// An arc's control polygon reaches its corner `(1, 1)`, the arc only
    /// `√½` of the way: a box over the corner alone is clear of the arc,
    /// and one the arc bulges into, but not its control point, holds it.
    #[test]
    fn an_arc_is_judged_by_itself_not_its_control_points() {
        let arc = quarter();
        let corner = within(&arc, &square(0.8, 0.8, 1.2, 1.2));
        assert!(!corner.touches, "{corner:?}");
        let holds = within(&arc, &square(-0.01, -0.01, 1.01, 1.01));
        assert!(holds.wholly && holds.touches, "{holds:?}");
        // The arc's middle is at `0.7071`: a box to `0.72` takes it whole,
        // one to `0.70` only touches it.
        let bulge = within(&arc, &square(0.6, 0.6, 0.72, 0.72));
        assert!(bulge.touches && !bulge.wholly, "{bulge:?}");
        let near = within(&arc, &square(0.72, 0.72, 0.9, 0.9));
        assert!(!near.touches, "{near:?}");
    }
}
