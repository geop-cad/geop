//! Two surfaces, one running on from where the other ends, as one surface.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    vector::Vector,
};

use super::NurbSurface3D;
use crate::knot_insertion::is_clamped_at;

impl NurbSurface3D {
    /// This surface and `next`, which starts along `u` — along `v`, if not
    /// `along_u` — where this one ends, as one surface: `next`'s knots
    /// carried on from where this one's end, its homogeneous control points
    /// scaled so that its first row's weights are this one's last row's —
    /// the same rational surface — and the two joined by a knot of full
    /// multiplicity, so the joined surface is exactly the two pieces,
    /// however they meet. The row where they meet is what both had there:
    /// two enclosures of one row of points, so what they share. What the
    /// pieces of a wall swept along a profile are, one face per piece of
    /// the profile, joined into the wall.
    ///
    /// Fails, saying why, for surfaces of other degrees or other knots
    /// across the join, knot vectors not clamped where they meet, and rows
    /// that do not meet.
    pub fn joined(&self, next: &Self, along_u: bool) -> GeopResult<Self> {
        if !along_u {
            return Ok(self
                .transposed()
                .joined(&next.transposed(), true)?
                .transposed());
        }
        let (a, b) = (self, next);
        let fail = |why: String| Err(GeopError::new(format!("NurbSurface::joined: {why}")));
        if a.degree_u != b.degree_u || a.degree_v != b.degree_v {
            return fail(format!(
                "degrees {}x{} and {}x{} differ",
                a.degree_u, a.degree_v, b.degree_u, b.degree_v
            ));
        }
        let across = a.knot_vector_v.len() == b.knot_vector_v.len()
            && (a.knot_vector_v.iter().zip(&b.knot_vector_v)).all(|(x, y)| x.could_be_equal(*y));
        if !across {
            return fail(format!(
                "the knots across the join differ: {:?} and {:?}",
                a.knot_vector_v, b.knot_vector_v
            ));
        }
        let p = a.degree_u;
        if !is_clamped_at(&a.knot_vector_u, p, false) || !is_clamped_at(&b.knot_vector_u, p, true) {
            return fail(format!(
                "the knot vectors {:?} and {:?} are not clamped where the surfaces meet",
                a.knot_vector_u, b.knot_vector_u
            ));
        }
        let nv = a.num_v;
        let last = &a.control_points[(a.num_u - 1) * nv..];
        let first = &b.control_points[..nv];
        let scale = last[0][3].div(first[0][3])?;
        let scaled: Vec<Vector<4>> = (b.control_points.iter())
            .map(|cp| cp.prod_scalar(scale))
            .collect();
        if !(last.iter().zip(&scaled[..nv])).all(|(p, q)| p.could_be_equal(q)) {
            return fail(format!(
                "the surfaces do not meet: one ends at {last:?}, the other starts at {:?}",
                &scaled[..nv]
            ));
        }
        let meet: Vec<Vector<4>> = (last.iter().zip(&scaled[..nv]))
            .map(|(p, q)| Vector::from_array(std::array::from_fn(|k| p[k].intersect(q[k]))))
            .collect();
        let (_, a1) = a.domain_u();
        let (b0, _) = b.domain_u();
        let shift = a1.sub(b0);
        let mut knots: Vec<_> = a.knot_vector_u[..a.knot_vector_u.len() - 1].to_vec();
        knots.extend(b.knot_vector_u[p + 1..].iter().map(|k| k.add(shift)));
        let mut points = a.control_points[..(a.num_u - 1) * nv].to_vec();
        points.extend(meet);
        points.extend_from_slice(&scaled[nv..]);
        let across: Vec<_> = (a.knot_vector_v.iter().zip(&b.knot_vector_v))
            .map(|(x, y)| x.union(*y))
            .collect();
        NurbSurface3D::try_new(p, a.degree_v, points, knots, across)
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use crate::nurb_surface::NurbSurface3D;

    fn p(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    /// Two squares side by side, joined along `u` and along `v`: one
    /// surface through both, each half where its piece was.
    #[test]
    fn joins_two_squares_along_either_direction() {
        let square = |x: f64| {
            NurbSurface3D::bilinear(
                p(x, 0., 0.),
                p(x + 1., 0., 0.),
                p(x + 1., 1., 0.),
                p(x, 1., 0.),
            )
            .unwrap()
        };
        let joined = square(0.0).joined(&square(1.0), true).unwrap();
        let (u0, u1) = joined.domain_u();
        assert_eq!(u1.sub(u0).to_f64(), 2.0);
        let at = joined
            .evaluate(Scalar::from_f64(1.5), Scalar::from_f64(0.5))
            .unwrap();
        assert!((at[0].to_f64() - 1.5).abs() < 1e-12 && (at[1].to_f64() - 0.5).abs() < 1e-12);
        let turned = |s: &NurbSurface3D| s.transposed();
        let joined = turned(&square(0.0))
            .joined(&turned(&square(1.0)), false)
            .unwrap();
        let at = joined
            .evaluate(Scalar::from_f64(0.5), Scalar::from_f64(1.5))
            .unwrap();
        assert!((at[0].to_f64() - 1.5).abs() < 1e-12, "{at:?}");
        assert!(square(0.0).joined(&square(2.0), true).is_err(), "apart");
    }
}
