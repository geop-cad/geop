//! A face's loop in its surface's parameter plane: the signed area it
//! encloses, integrated exactly from its pcurves ([`loop_signed_area`]),
//! and a polygon sampled along them, for the rasterizer to triangulate
//! ([`sample_loop_to_polygon`]).
//!
//! Ask the area which way a loop runs, never a sampled polygon: the gap
//! between a polygon and the pcurves is part of no enclosure (see
//! [`geop_core_geometry::nurb_curve::loop_signed_area`]).

use geop_core_geometry::nurb_curve;
use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector2};

use crate::{CoedgeId, Curve2, Model};

/// The signed area, in `(u, v)`, of the loop anchored at `first`: positive
/// counter-clockwise, negative clockwise.
pub fn loop_signed_area(model: &Model, first: CoedgeId) -> GeopResult<Scalar> {
    let pcurves = model
        .iterate_loop_coedges(first)
        .map(|coedge| Ok(&model.get_coedge(coedge)?.pcurve))
        .collect::<GeopResult<Vec<_>>>()?;
    nurb_curve::loop_signed_area(pcurves)
}

/// Sample the pcurves of the loop anchored at `first` (in traversal order)
/// into a closed `(u, v)` polygon, `n` samples per coedge (the last sample of
/// each coedge is dropped, since it coincides with the next coedge's first
/// sample).
pub fn sample_loop_to_polygon(
    model: &Model,
    first: CoedgeId,
    n: usize,
) -> GeopResult<Vec<Vector2>> {
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
pub fn sample_pcurves_to_polygon<'a>(
    pcurves: impl IntoIterator<Item = &'a Curve2>,
    n: usize,
) -> GeopResult<Vec<Vector2>> {
    let mut poly = Vec::new();
    for pcurve in pcurves {
        let (t0, t1) = pcurve.domain();
        for i in 0..n {
            let frac = Scalar::from_ratio(i as i64, n as i64)?;
            let t = t0.add(t1.sub(t0).mul(frac));
            poly.push(pcurve.evaluate(t)?);
        }
    }
    Ok(poly)
}
