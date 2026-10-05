use geop_core_geometry::contains::{curve::curve_could_contain, surface::surface_could_contain};
use geop_core_math::{geop_error::GeopError, scalars::Scalar, vector::Vector3};

use crate::{CoedgeGeometry, Model, validation::ValidationParameters};

/// Checks that every coedge's pcurve actually traces its own backing
/// geometry, not just at the two endpoints (see
/// `check_curves_and_surfaces_vertices`) but along its whole length.
///
/// For an `Edge`-backed coedge: samples `params.sample_count` points along
/// the pcurve (mapped through the coedge's face surface into 3-D) and along
/// the edge's own curve, at matching fractional positions of each one's own
/// domain, and checks the two points `could_be_equal`.
///
/// For a `Vertex`-backed coedge (see `CoedgeGeometry::Vertex`): there's no
/// edge curve to pace against — instead, every pcurve sample must map
/// (through the surface) to that one constant vertex position, the whole
/// way. A sample that doesn't means the pcurve strayed off the degenerate
/// row it's supposed to be tracing.
///
/// Pushes at most one error per coedge onto `errors` (its first bad sample)
/// and moves on to the next coedge rather than stopping altogether.
pub fn check_curve_and_surface_sampling<S: Scalar>(
    params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    for (&coedge_id, coedge) in &model.coedges {
        let Some(face) = model.faces.get(&coedge.face) else {
            continue;
        };
        let surface = &face.surface;
        let (t0, t1) = coedge.pcurve.domain();

        match coedge.geometry {
            CoedgeGeometry::Edge(edge_id) => {
                let Some(edge) = model.edges.get(&edge_id) else {
                    continue;
                };
                let (u0, u1) = edge.curve.domain();

                // Each pcurve sample, mapped through the surface, must land
                // *somewhere* on the edge's curve — but not at any particular
                // parameter of it.
                //
                // This used to compare against the curve at the same fraction
                // (mirrored for a `Reversed` coedge), which assumes the pcurve
                // and the 3-D curve share a parametrization. They do not, and
                // the kernel knows it: `Model::split_edge_at_vertex` relocates
                // a split point geometrically precisely because "the pcurve's
                // own parameter range doesn't share the edge curve's `t`
                // values". A pcurve fitted by `NurbSurface::fit_pcurve` — a
                // cubic interpolated through projected samples — agrees at the
                // endpoints and drifts in between, so the fraction-matched
                // comparison reported traced edges as broken purely for being
                // parametrized differently.
                //
                // Asking "is this point on the curve at all" is both the
                // invariant that actually holds and a stronger statement about
                // the geometry, since it is parametrization-free. It still
                // catches an edge curve that bends away from its pcurve, and
                // the degenerate pcurves that once got a face spliced onto a
                // surface it did not lie on.
                for i in 0..params.sample_count {
                    let frac = S::from_f64(i as f64 / (params.sample_count - 1) as f64);
                    let t = t0.add(t1.sub(t0).mul(frac));
                    let _ = (u0, u1);

                    let sample_result = (|| -> Result<Option<(Vector3<S>, _)>, GeopError> {
                        let uv = coedge.pcurve.evaluate(t)?;
                        let point = surface.evaluate(uv[0], uv[1])?;
                        let on_edge = curve_could_contain(
                            &edge.curve,
                            &point,
                            params.max_nodes,
                            params.min_subdivision_size,
                        )?
                        .is_some();
                        Ok((!on_edge).then_some((point, uv)))
                    })();

                    match sample_result {
                        Ok(None) => {}
                        Ok(Some((point, uv))) => {
                            // Whether the stray point lies on the surfaces of
                            // the edge's other faces tells a pcurve that
                            // follows the right intersection but the wrong
                            // stretch of it from one on a curve the other
                            // face never meets.
                            let partners: Vec<String> = model
                                .coedges
                                .iter()
                                .filter(|(id, c)| {
                                    **id != coedge_id && c.geometry == CoedgeGeometry::Edge(edge_id)
                                })
                                .map(|(_, c)| {
                                    let on = model.faces.get(&c.face).map(|f| {
                                        surface_could_contain(
                                            &f.surface,
                                            &point,
                                            params.max_nodes,
                                            params.min_subdivision_size,
                                        )
                                        .map(|hit| hit.is_some())
                                    });
                                    format!("face {} (point on its surface: {on:?})", c.face.0)
                                })
                                .collect();
                            let ends = (edge.curve.evaluate(u0).ok(), edge.curve.evaluate(u1).ok());
                            // A pcurve that drifted from its edge, and an
                            // edge curve that is not on this face's surface
                            // to begin with, fail alike here; the edge's own
                            // samples tell them apart.
                            let off_surface: Vec<usize> = (0..params.sample_count)
                                .filter(|&k| {
                                    let frac =
                                        S::from_f64(k as f64 / (params.sample_count - 1) as f64);
                                    edge.curve
                                        .evaluate(u0.add(u1.sub(u0).mul(frac)))
                                        .and_then(|q| {
                                            surface_could_contain(
                                                surface,
                                                &q,
                                                params.max_nodes,
                                                params.min_subdivision_size,
                                            )
                                        })
                                        .is_ok_and(|hit| hit.is_none())
                                })
                                .collect();
                            errors.push(GeopError::new(format!(
                                "coedge {}'s pcurve sample {} (t={}) maps through its face's surface to a point that is not on edge {}'s curve at all \
                                 (coedge on face {}, (u, v) {uv:?}, point {point:?}, edge from {:?} to {:?}; edge curve samples off this face's surface: {off_surface:?} of {}; the edge's other coedges: {}; pcurve {:?}; edge curve {:?})",
                                coedge_id.0,
                                i,
                                t,
                                edge_id.0,
                                coedge.face.0,
                                ends.0,
                                ends.1,
                                params.sample_count,
                                partners.join(", "),
                                coedge.pcurve,
                                edge.curve
                            )));
                            break;
                        }
                        Err(e) => {
                            errors.push(e.with_context(format!(
                                "coedge {} sample {} (t={})",
                                coedge_id.0, i, t
                            )));
                            break;
                        }
                    }
                }
            }
            CoedgeGeometry::Vertex(vertex_id) => {
                let Some(vertex) = model.vertices.get(&vertex_id) else {
                    continue;
                };

                for i in 0..params.sample_count {
                    let frac = S::from_f64(i as f64 / (params.sample_count - 1) as f64);
                    let t = t0.add(t1.sub(t0).mul(frac));

                    let sample_result = (|| -> Result<bool, GeopError> {
                        let uv = coedge.pcurve.evaluate(t)?;
                        let point = surface.evaluate(uv[0], uv[1])?;
                        Ok(point.could_be_equal(&vertex.point))
                    })();

                    match sample_result {
                        Ok(true) => {}
                        Ok(false) => {
                            errors.push(GeopError::new(format!(
                                "coedge {}'s pcurve sample {} (t={}) maps to a point not equal to its own vertex {}",
                                coedge_id.0, i, t, vertex_id.0
                            )));
                            break;
                        }
                        Err(e) => {
                            errors.push(e.with_context(format!(
                                "coedge {} sample {} (t={})",
                                coedge_id.0, i, t
                            )));
                            break;
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::check_curve_and_surface_sampling;
    use crate::test_fixtures::test_cube_solid;
    use crate::{Model, validation::ValidationParameters};
    use geop_core_geometry::nurb_curve::NurbCurve;
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector4};

    fn run<S: Scalar>(model: &Model<S>) -> Vec<geop_core_math::geop_error::GeopError> {
        let mut errors = Vec::new();
        check_curve_and_surface_sampling(&ValidationParameters::default(), &mut errors, model);
        errors
    }

    fn check_valid_cube_passes<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        assert!(run(&model).is_empty());
    }
    #[test]
    fn valid_cube_passes() {
        for_all_scalars!(check_valid_cube_passes);
    }

    fn bend_edge<S: Scalar>(model: &mut Model<S>, edge_id: crate::EdgeId) {
        let (start_v, end_v) = {
            let edge = &model.edges[&edge_id];
            (edge.start_vertex, edge.end_vertex)
        };
        let p0 = model.vertices[&start_v].point;
        let p1 = model.vertices[&end_v].point;
        let far = Vector4::from_array([
            S::from_f64(50.0),
            S::from_f64(50.0),
            S::from_f64(50.0),
            S::ONE,
        ]);
        let bent = NurbCurve::try_new(
            2,
            vec![
                Vector4::from_array([p0[0], p0[1], p0[2], S::ONE]),
                far,
                Vector4::from_array([p1[0], p1[1], p1[2], S::ONE]),
            ],
            vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE],
        )
        .unwrap();
        model.edges.get_mut(&edge_id).unwrap().curve = bent;
    }

    fn check_bent_edge_curve_fails<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);

        // Bend one edge's curve through a far-off interior control point,
        // keeping its endpoints exactly where they were — the endpoint-only
        // check can't catch this, only sampling along the curve can.
        let edge_id = *model.edges.keys().next().unwrap();
        bend_edge(&mut model, edge_id);

        assert!(!run(&model).is_empty());
    }
    #[test]
    fn bent_edge_curve_fails() {
        for_all_scalars!(check_bent_edge_curve_fails);
    }

    fn check_bending_two_edges_reports_both<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        let mut ids = model.edges.keys().copied();
        let e1 = ids.next().unwrap();
        let e2 = ids.next().unwrap();
        bend_edge(&mut model, e1);
        bend_edge(&mut model, e2);
        assert!(run(&model).len() >= 2);
    }
    #[test]
    fn bending_two_edges_reports_both() {
        for_all_scalars!(check_bending_two_edges_reports_both);
    }
}
