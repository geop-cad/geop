use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

use crate::{Curve2, FaceId, Model, Sense};

impl<S: Scalar> Model<S> {
    /// Turn `face_id`'s material side around, so its normal points the other
    /// way.
    ///
    /// Orientation in this kernel lives in the surface's parametrization —
    /// there is no flag on a face — so flipping it means mirroring the
    /// surface's `u` (see [`geop_core_geometry::nurb_surface::NurbSurface::reverse_u`])
    /// and applying the identical mirror to every pcurve drawn on it. The
    /// domain is unchanged by the mirror, so the trim loops still describe
    /// the same region of the same surface; only `Su x Sv` reverses.
    ///
    /// Deliberately touches nothing but this face's surface and its own
    /// pcurves. The 3-D curves, the edges, the vertices and every coedge on
    /// the *other* side of those edges are untouched, which is what lets a
    /// boolean flip one operand's faces while they stay glued to the other's
    /// along shared edges.
    pub fn reverse_face(&mut self, face_id: FaceId) -> GeopResult<()> {
        let ctx = |e: GeopError| e.with_context(format!("Model::reverse_face(face={face_id})"));

        let face = self.get_face(face_id).with_context(&ctx)?;
        let (u_lo, u_hi) = face.surface.domain_u();
        let span = u_lo.add(u_hi);
        let reversed = face.surface.reverse_u();
        self.get_face_mut(face_id).with_context(&ctx)?.surface = reversed;

        for coedge_id in self.iterate_face_coedges(face_id).collect::<Vec<_>>() {
            let coedge = self.get_coedge_mut(coedge_id).with_context(&ctx)?;
            mirror_u(&mut coedge.pcurve, span);

            // Then reverse the loop: swap `next`/`prev`, run the pcurve the
            // other way, and flip the sense.
            //
            // Mirroring `u` flips every loop's winding on its own — a
            // counter-clockwise outer loop comes back clockwise — which puts
            // the material on the *right* of each coedge instead of its left.
            // Nothing structural notices (the loop still closes and its
            // pcurves still join), but `splice_edge_into_face` reads winding
            // to tell a new face from a new hole, and the debug renderer reads
            // it to inset a trim curve inward, so a reversed face renders with
            // its coedges outside it. Reversing the traversal flips the
            // winding back, leaving only the normal reversed — which is the
            // whole point of the operation.
            //
            // The three go together: reversing traversal without reversing the
            // pcurves would break `pcurve_loop_continuity`, and without
            // flipping the sense a coedge's start vertex would no longer be
            // the vertex its pcurve now starts at.
            std::mem::swap(&mut coedge.next, &mut coedge.prev);
            coedge.pcurve = coedge.pcurve.reverse();
            coedge.sense = match coedge.sense {
                Sense::Forward => Sense::Reversed,
                Sense::Reversed => Sense::Forward,
            };
        }
        Ok(())
    }
}

/// Mirrors `pcurve` in `u`, `u -> span - u`: what a face's pcurves go
/// through when its surface is mirrored by
/// [`geop_core_geometry::nurb_surface::NurbSurface::reverse_u`].
///
/// A pcurve's control points are homogeneous `[u*w, v*w, w]`, so mirroring
/// `u -> span - u` is `x -> span*w - x`. Applying it to the control points
/// rather than to evaluated points keeps the curve exact: a mirror is
/// affine, and an affine map of a NURBS curve is the same map applied to its
/// control net.
pub(crate) fn mirror_u<S: Scalar>(pcurve: &mut Curve2<S>, span: S) {
    for point in &mut pcurve.control_points {
        point[0] = span.mul(point[2]).sub(point[0]);
    }
    // The mutation above bypasses every constructor that would otherwise
    // keep the pcurve's cached bounding box (used by the intersection
    // search's `aabb_could_overlap` prefilter) in sync — refresh it
    // explicitly or it goes stale and starts pruning real overlaps
    // involving this pcurve.
    pcurve.recompute_aabb();
}

#[cfg(test)]
mod tests {
    use geop_core_geometry::{
        nurb_curve::{NurbCurve, NurbCurve2D},
        nurb_surface::NurbSurface3D,
    };
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    use crate::{
        Coedge, CoedgeGeometry, CoedgeId, Edge, Face, Model, Sense, ShellId, Vertex, VertexId,
        boundary::BoundaryType,
    };

    /// A face whose surface is a saddle (so the normal genuinely varies) with
    /// one triangular trim loop.
    fn saddle_face<S: Scalar>(model: &mut Model<S>) -> crate::FaceId {
        let p = |x: f64, y: f64, z: f64| {
            Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z), S::ONE])
        };
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![p(0., 0., 0.), p(0., 2., 1.), p(2., 0., 1.), p(2., 2., 0.)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap();
        let face_id = model.insert_face(Face {
            surface,
            outer: BoundaryType::Vertex(VertexId(0)),
            holes: Vec::new(),
            shell: ShellId(999),
        });

        let corners = [(0.2, 0.2), (0.8, 0.2), (0.5, 0.8)];
        let line2 = |a: (f64, f64), b: (f64, f64)| -> NurbCurve2D<S> {
            NurbCurve::try_new(
                1,
                vec![
                    Vector3::from_array([S::from_f64(a.0), S::from_f64(a.1), S::ONE]),
                    Vector3::from_array([S::from_f64(b.0), S::from_f64(b.1), S::ONE]),
                ],
                vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            )
            .unwrap()
        };
        let coedges: Vec<CoedgeId> = (0..3)
            .map(|i| {
                let a = corners[i];
                let b = corners[(i + 1) % 3];
                let v0 = model.insert_vertex(Vertex {
                    point: Vector3::from_array([S::from_f64(a.0), S::from_f64(a.1), S::ZERO]),
                });
                let v1 = model.insert_vertex(Vertex {
                    point: Vector3::from_array([S::from_f64(b.0), S::from_f64(b.1), S::ZERO]),
                });
                let edge = model.insert_edge(Edge {
                    curve: NurbCurve::try_new(
                        1,
                        vec![
                            Vector4::from_array([
                                S::from_f64(a.0),
                                S::from_f64(a.1),
                                S::ZERO,
                                S::ONE,
                            ]),
                            Vector4::from_array([
                                S::from_f64(b.0),
                                S::from_f64(b.1),
                                S::ZERO,
                                S::ONE,
                            ]),
                        ],
                        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
                    )
                    .unwrap(),
                    start_vertex: v0,
                    end_vertex: v1,
                });
                model.insert_coedge(Coedge {
                    geometry: CoedgeGeometry::Edge(edge),
                    sense: Sense::Forward,
                    pcurve: line2(a, b),
                    next: CoedgeId(0),
                    prev: CoedgeId(0),
                    face: face_id,
                })
            })
            .collect();
        for i in 0..3 {
            model.coedges.get_mut(&coedges[i]).unwrap().next = coedges[(i + 1) % 3];
            model.coedges.get_mut(&coedges[i]).unwrap().prev = coedges[(i + 2) % 3];
        }
        model.faces.get_mut(&face_id).unwrap().outer = BoundaryType::Loop(coedges[0]);
        face_id
    }

    /// Reversing flips the normal while every trim loop still lands on the
    /// same 3-D points — the face covers the same patch, facing the other way.
    fn check_reverse_face_flips_normal_and_keeps_the_patch<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = saddle_face(&mut model);

        let anchor = match model.get_face(face_id).unwrap().outer {
            BoundaryType::Loop(a) => a,
            BoundaryType::Vertex(_) => unreachable!(),
        };
        let sample_t = S::from_f64(0.3);
        let before_uv = model
            .get_coedge(anchor)
            .unwrap()
            .pcurve
            .evaluate(sample_t)
            .unwrap();
        let surface_before = model.get_face(face_id).unwrap().surface.clone();
        let before_point = surface_before.evaluate(before_uv[0], before_uv[1]).unwrap();
        let before_normal = surface_before.normal(before_uv[0], before_uv[1]).unwrap();

        model.reverse_face(face_id).unwrap();

        // The pcurve now runs the other way (the loop's traversal is
        // reversed), so the same point sits at the mirrored parameter.
        let (t0, t1) = model.get_coedge(anchor).unwrap().pcurve.domain();
        let after_uv = model
            .get_coedge(anchor)
            .unwrap()
            .pcurve
            .evaluate(t0.add(t1).sub(sample_t))
            .unwrap();
        let surface_after = &model.get_face(face_id).unwrap().surface;
        let after_point = surface_after.evaluate(after_uv[0], after_uv[1]).unwrap();
        let after_normal = surface_after.normal(after_uv[0], after_uv[1]).unwrap();

        for c in 0..3 {
            assert!(
                before_point[c].could_be_equal(after_point[c]),
                "coord {c}: the trim curve moved: {before_point:?} vs {after_point:?}"
            );
            assert!(
                before_normal[c].could_be_equal(after_normal[c].neg()),
                "coord {c}: the normal did not flip: {before_normal:?} vs {after_normal:?}"
            );
        }
    }
    #[test]
    fn reverse_face_flips_normal_and_keeps_the_patch() {
        for_all_scalars!(check_reverse_face_flips_normal_and_keeps_the_patch);
    }

    /// Reversing a face must not change which side of its loops the material
    /// is on. The kernel's convention is that an outer loop runs
    /// counter-clockwise in `(u, v)` and a hole runs clockwise — that is what
    /// `splice_edge_into_face` uses to tell a new face from a new hole, and
    /// what the debug renderer uses to inset a coedge's trim curve *inward*.
    ///
    /// Mirroring `u` flips the winding on its own, so reversing has to undo
    /// that by also reversing each loop's traversal. Without it the outer loop
    /// comes back clockwise: still continuous, still structurally valid, but
    /// with every coedge now running with the material on its right.
    fn check_reverse_face_preserves_loop_winding<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = saddle_face(&mut model);

        let area = |model: &Model<S>| {
            let BoundaryType::Loop(anchor) = model.get_face(face_id).unwrap().outer else {
                unreachable!()
            };
            let polygon = crate::loop_sampling::sample_loop_to_polygon(model, anchor, 8).unwrap();
            geop_core_math::polygon::polygon_signed_area(&polygon)
        };

        let before = area(&model);
        model.reverse_face(face_id).unwrap();
        let after = area(&model);

        assert!(
            before.definitely_greater(S::ZERO) == after.definitely_greater(S::ZERO)
                && before.definitely_less(S::ZERO) == after.definitely_less(S::ZERO),
            "winding flipped: signed area went from {before:?} to {after:?}"
        );
    }
    #[test]
    fn reverse_face_preserves_loop_winding() {
        for_all_scalars!(check_reverse_face_preserves_loop_winding);
    }

    /// Reversing twice is the identity.
    fn check_reverse_face_twice_is_identity<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = saddle_face(&mut model);
        let anchor = match model.get_face(face_id).unwrap().outer {
            BoundaryType::Loop(a) => a,
            BoundaryType::Vertex(_) => unreachable!(),
        };
        let t = S::from_f64(0.4);
        let before = model
            .get_coedge(anchor)
            .unwrap()
            .pcurve
            .evaluate(t)
            .unwrap();
        let before_sense = model.get_coedge(anchor).unwrap().sense;

        model.reverse_face(face_id).unwrap();
        model.reverse_face(face_id).unwrap();

        let after = model
            .get_coedge(anchor)
            .unwrap()
            .pcurve
            .evaluate(t)
            .unwrap();
        for c in 0..2 {
            assert!(before[c].could_be_equal(after[c]), "coord {c} drifted");
        }
        assert_eq!(
            before_sense,
            model.get_coedge(anchor).unwrap().sense,
            "sense must come back to where it started"
        );
    }
    #[test]
    fn reverse_face_twice_is_identity() {
        for_all_scalars!(check_reverse_face_twice_is_identity);
    }
}
