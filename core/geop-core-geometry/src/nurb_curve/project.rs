use geop_core_math::{
    disjoint_set::DisjointSet, geop_error::GeopResult, scalars::Scalar, vector::Vector3,
};

use super::NurbCurve;

impl<S: Scalar> NurbCurve<S, 4> {
    /// Every parameter `t` at which this curve passes through `target`
    /// (plural since a self-intersecting curve can pass through the same
    /// point at more than one, genuinely distinct, parameter).
    ///
    /// Recursively subdivides the curve, discarding any segment whose
    /// convex hull could not contain `target`. A surviving segment
    /// converges once its hull's chord length is no longer definitely
    /// greater than `min_subdivision_size`, contributing the
    /// [`Scalar::union`] of its own `[t0, t1]` domain as a candidate,
    /// folded into a [`DisjointSet`] so no two returned solutions ever
    /// describe the same physical parameter — for a genuine interval
    /// scalar each is a real "the true parameter is provably within this
    /// span" guarantee, not an arbitrarily narrowed single point.
    pub fn project(
        &self,
        target: Vector3<S>,
        max_nodes: usize,
        min_subdivision_size: S,
    ) -> GeopResult<Vec<S>> {
        let mut stack: Vec<NurbCurve<S, 4>> = vec![self.clone()];
        let mut explored = 0usize;
        let mut solutions: DisjointSet<S> = DisjointSet::new();

        while let Some(seg) = stack.pop() {
            if explored >= max_nodes {
                break;
            }
            explored += 1;

            let hull = seg.convex_hull();
            if hull.definitely_not_contains(&target) {
                continue;
            }

            let (t0, t1) = seg.domain();
            let chord_len = hull.points[hull.points.len() - 1]
                .sub(&hull.points[0])
                .norm();
            if !chord_len.definitely_greater(min_subdivision_size) {
                solutions.insert(t0.union(t1));
                continue;
            }

            if let Ok((left, right)) = seg.split_mid() {
                stack.push(left);
                stack.push(right);
            }
            // Cannot split (e.g. midpoint already at multiplicity p+1) and
            // hasn't converged — nothing more to do with this segment.
        }

        Ok(solutions.into_vec())
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use super::super::NurbCurve3D;

    const MAX: usize = 500;

    fn line<S: Scalar>() -> NurbCurve3D<S> {
        let p = |x: f64, y: f64, z: f64| {
            geop_core_math::vector::Vector4::from_array([
                S::from_f64(x),
                S::from_f64(y),
                S::from_f64(z),
                S::ONE,
            ])
        };
        NurbCurve3D::try_new(
            1,
            vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap()
    }

    // `ScalInFPA64` is fixed-point at 2^-32 (~2.3e-10) resolution — once
    // subdivision reaches that granularity, `t` can't be refined any
    // further, so the tolerance here has to be a bit looser than
    // `min_subdivision_size` itself to accommodate that scalar's precision
    // floor.
    const TOL: f64 = 1e-4;

    fn check_project_point_on_line<S: Scalar>() {
        let curve = line::<S>();
        let target = Vector3::from_array([S::from_f64(0.42), S::from_f64(0.0), S::from_f64(0.0)]);
        let solutions = curve.project(target, MAX, S::from_f64(1e-6)).unwrap();
        assert_eq!(solutions.len(), 1);
        assert!(
            solutions[0]
                .sub(S::from_f64(0.42))
                .abs()
                .could_be_less(S::from_f64(TOL))
        );
    }
    #[test]
    fn project_point_on_line() {
        for_all_scalars!(check_project_point_on_line);
    }

    fn check_project_point_off_line_finds_nothing<S: Scalar>() {
        let curve = line::<S>();
        // Off the line entirely (y=1) — no segment's hull could ever
        // contain it.
        let target = Vector3::from_array([S::from_f64(0.3), S::from_f64(1.0), S::ZERO]);
        assert!(
            curve
                .project(target, MAX, S::from_f64(1e-6))
                .unwrap()
                .is_empty()
        );
    }
    #[test]
    fn project_point_off_line_finds_nothing() {
        for_all_scalars!(check_project_point_off_line_finds_nothing);
    }

    fn check_zero_budget_finds_nothing<S: Scalar>() {
        let curve = line::<S>();
        let target = Vector3::from_array([S::from_f64(0.5), S::ZERO, S::ZERO]);
        assert!(
            curve
                .project(target, 0, S::from_f64(1e-6))
                .unwrap()
                .is_empty()
        );
    }
    #[test]
    fn zero_budget_finds_nothing() {
        for_all_scalars!(check_zero_budget_finds_nothing);
    }
}
