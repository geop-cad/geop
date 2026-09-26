pub mod gjk;

use crate::{scalars::Scalar, vector::Vector};

/// A convex hull represented by its defining point set (e.g. the control
/// points of a NURBS curve or surface), in `N`-dimensional space.
///
/// The hull is never computed explicitly — overlap and containment queries
/// operate directly on the point set via [`gjk`].  By the convex-hull
/// property of B-spline / NURBS bases, every point on a curve or surface
/// patch lies within the convex hull of its (dehomogenized) control points.
#[derive(Debug, Clone)]
pub struct ConvexHull<S: Scalar, const N: usize> {
    pub points: Vec<Vector<S, N>>,
}

impl<S: Scalar, const N: usize> ConvexHull<S, N> {
    pub fn new(points: Vec<Vector<S, N>>) -> Self {
        Self { points }
    }

    /// The average of this hull's defining points — a representative
    /// position for the region it bounds, used e.g. to detect near-duplicate
    /// solutions at a caller-chosen distance tolerance (see
    /// `curve_surface_intersect`'s dedup check, which can't rely solely on
    /// `could_overlap` since that bottoms out in each `Scalar`'s own
    /// built-in equality tolerance rather than the search's own epsilon).
    pub fn centroid(&self) -> Vector<S, N> {
        let mut sum = Vector::<S, N>::zero();
        for p in &self.points {
            sum = sum.add(p);
        }
        sum.prod_scalar(S::ONE.div(S::from_i64(self.points.len() as i64)).unwrap())
    }

    /// True if `self` and `other` could overlap (intersect or touch).
    pub fn could_overlap(&self, other: &Self) -> bool {
        gjk::could_overlap(&self.points, &other.points)
    }

    /// Negation of [`Self::could_overlap`].
    pub fn definitely_no_overlap(&self, other: &Self) -> bool {
        gjk::definitely_no_overlap(&self.points, &other.points)
    }

    /// True if `point` could lie within this convex hull.
    pub fn could_contain(&self, point: &Vector<S, N>) -> bool {
        gjk::could_overlap(&self.points, std::slice::from_ref(point))
    }

    /// Negation of [`Self::could_contain`].
    pub fn definitely_not_contains(&self, point: &Vector<S, N>) -> bool {
        !self.could_contain(point)
    }
}

#[cfg(test)]
mod tests {
    use super::ConvexHull;
    use crate::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector2, Vector3},
    };

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(x), S::from_f64(y)])
    }

    fn unit_square<S: Scalar>() -> ConvexHull<S, 3> {
        ConvexHull::new(vec![
            v3(0., 0., 0.),
            v3(1., 0., 0.),
            v3(0., 1., 0.),
            v3(1., 1., 0.),
        ])
    }

    fn check_contains_interior_point<S: Scalar>() {
        let hull = unit_square::<S>();
        assert!(hull.could_contain(&v3(0.5, 0.5, 0.)));
    }
    #[test]
    fn contains_interior_point() {
        for_all_scalars!(check_contains_interior_point);
    }

    fn check_excludes_exterior_point<S: Scalar>() {
        let hull = unit_square::<S>();
        assert!(hull.definitely_not_contains(&v3(2., 2., 0.)));
    }
    #[test]
    fn excludes_exterior_point() {
        for_all_scalars!(check_excludes_exterior_point);
    }

    fn check_overlapping_hulls<S: Scalar>() {
        let a = unit_square::<S>();
        let b = ConvexHull::new(vec![
            v3(0.5, 0.5, 0.),
            v3(1.5, 0.5, 0.),
            v3(0.5, 1.5, 0.),
            v3(1.5, 1.5, 0.),
        ]);
        assert!(a.could_overlap(&b));
        assert!(!a.definitely_no_overlap(&b));
    }
    #[test]
    fn overlapping_hulls() {
        for_all_scalars!(check_overlapping_hulls);
    }

    fn check_separated_hulls<S: Scalar>() {
        let a = unit_square::<S>();
        let b = ConvexHull::new(vec![
            v3(10., 10., 0.),
            v3(11., 10., 0.),
            v3(10., 11., 0.),
            v3(11., 11., 0.),
        ]);
        assert!(a.definitely_no_overlap(&b));
        assert!(!a.could_overlap(&b));
    }
    #[test]
    fn separated_hulls() {
        for_all_scalars!(check_separated_hulls);
    }

    /// The same containment/overlap checks, natively in 2-D (no `z=0`
    /// embedding needed) — confirms `ConvexHull` genuinely works at `N=2`.
    fn unit_square_2d<S: Scalar>() -> ConvexHull<S, 2> {
        ConvexHull::new(vec![v2(0., 0.), v2(1., 0.), v2(0., 1.), v2(1., 1.)])
    }

    fn check_contains_interior_point_2d<S: Scalar>() {
        let hull = unit_square_2d::<S>();
        assert!(hull.could_contain(&v2(0.5, 0.5)));
    }
    #[test]
    fn contains_interior_point_2d() {
        for_all_scalars!(check_contains_interior_point_2d);
    }

    fn check_excludes_exterior_point_2d<S: Scalar>() {
        let hull = unit_square_2d::<S>();
        assert!(hull.definitely_not_contains(&v2(2., 2.)));
    }
    #[test]
    fn excludes_exterior_point_2d() {
        for_all_scalars!(check_excludes_exterior_point_2d);
    }
}
