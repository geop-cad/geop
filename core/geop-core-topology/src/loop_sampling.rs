//! Sample a coedge loop's pcurves into a closed `(u, v)` polygon.
//!
//! Lives in `geop-core-topology` rather than `geop-ops-rasterize` (which
//! also uses it, for triangulation) because `geop-core-topology`'s own edit
//! code (`splice_edge_into_face`'s loop-orientation check) needs it too,
//! and topology sits below rasterize in the dependency order.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector2};

use crate::{CoedgeId, Model};

/// Sample the pcurves of the loop anchored at `first` (in traversal order)
/// into a closed `(u, v)` polygon, `n` samples per coedge (the last sample of
/// each coedge is dropped, since it coincides with the next coedge's first
/// sample).
pub fn sample_loop_to_polygon<S: Scalar>(
    model: &Model<S>,
    first: CoedgeId,
    n: usize,
) -> GeopResult<Vec<Vector2<S>>> {
    let mut poly = Vec::new();
    for current in model.iterate_loop_coedges(first) {
        let pcurve = &model.coedges[&current].pcurve;
        let (t0, t1) = pcurve.domain();
        for i in 0..n - 1 {
            let frac = S::from_ratio(i as i64, (n - 1) as i64)?;
            let t = t0.add(t1.sub(t0).mul(frac));
            poly.push(pcurve.evaluate(t)?);
        }
    }
    Ok(poly)
}
