//! Sample a loop's pcurves into a closed `(u, v)` polygon.
//!
//! Lives in `geop-core-topology` rather than `geop-core-rasterize` (which
//! also uses it, for triangulation) because `geop-core-topology`'s own edit
//! code (`splice_edge_into_face`'s loop-orientation check) needs it too,
//! and topology sits below rasterize in the dependency order.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector2};

use crate::{CoedgeId, Curve2, Model};

/// Sample the pcurves of the loop anchored at `first` (in traversal order)
/// into a closed `(u, v)` polygon, `n` samples per coedge (the last sample of
/// each coedge is dropped, since it coincides with the next coedge's first
/// sample).
pub fn sample_loop_to_polygon<S: Scalar>(
    model: &Model<S>,
    first: CoedgeId,
    n: usize,
) -> GeopResult<Vec<Vector2<S>>> {
    sample_pcurves_to_polygon(
        model
            .iterate_loop_coedges(first)
            .map(|coedge| &model.coedges[&coedge].pcurve),
        n - 1,
    )
}

/// Sample `pcurves`, a loop's in order, into a closed `(u, v)` polygon: `n`
/// points evenly along each one's domain, its end left out, as the next
/// one starts there.
pub fn sample_pcurves_to_polygon<'a, S: Scalar>(
    pcurves: impl IntoIterator<Item = &'a Curve2<S>>,
    n: usize,
) -> GeopResult<Vec<Vector2<S>>> {
    let mut poly = Vec::new();
    for pcurve in pcurves {
        let (t0, t1) = pcurve.domain();
        for i in 0..n {
            let frac = S::from_ratio(i as i64, n as i64)?;
            let t = t0.add(t1.sub(t0).mul(frac));
            poly.push(pcurve.evaluate(t)?);
        }
    }
    Ok(poly)
}
