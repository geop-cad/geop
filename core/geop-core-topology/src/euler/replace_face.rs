use crate::{
    FaceId, Model, argument_validation::validate_pcurve_start_and_end, boundary::BoundaryType,
};
use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::{geop_error::GeopResult, scalars::Scalar};

impl<S: Scalar> Model<S> {
    // Swap `face_id`'s surface for `surface` — e.g. dropping in a real
    // parameterization once a face's boundary rings are complete, in place
    // of the placeholder `NurbSurface3D::everything()` it was built with.
    // Every existing coedge's pcurve must still land on its edge's actual
    // 3-D endpoints under the new surface, checked before anything is
    // mutated, so a rejected swap leaves the model untouched.
    pub fn replace_face(
        self: &mut Model<S>,
        face_id: FaceId,
        surface: NurbSurface3D<S>,
    ) -> GeopResult<()> {
        let face = self.get_face(face_id)?.clone();
        for boundary in face.boundaries() {
            let BoundaryType::Loop(anchor) = boundary else {
                continue;
            };
            let mut cursor = anchor;
            loop {
                let coedge = self.get_coedge(cursor)?.clone();
                let start = self.coedge_start_vertex(cursor)?.clone();
                let end = self.coedge_end_vertex(cursor)?.clone();
                validate_pcurve_start_and_end(&surface, &coedge.pcurve, &start.point, &end.point)?;

                cursor = coedge.next;
                if cursor == anchor {
                    break;
                }
            }
        }

        self.get_face_mut(face_id)?.surface = surface;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        Coedge, CoedgeGeometry, CoedgeId, Edge, Face, Model, Sense, ShellId, Vertex,
        boundary::BoundaryType,
    };
    use geop_core_geometry::{
        nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D},
        nurb_surface::NurbSurface3D,
    };
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    fn line3<S: Scalar>(a: (f64, f64, f64), b: (f64, f64, f64)) -> NurbCurve3D<S> {
        let p = |x: f64, y: f64, z: f64| {
            Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z), S::ONE])
        };
        NurbCurve3D::try_new(
            1,
            vec![p(a.0, a.1, a.2), p(b.0, b.1, b.2)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap()
    }

    /// A pcurve matching the unit-square bilinear surface built below:
    /// `pcurve.evaluate(t) == (a + (b-a)*t, ...)` in `(u, v)`, exactly
    /// tracing the 3-D straight line `a -> b` since the surface is planar
    /// and bilinear.
    fn line2<S: Scalar>(a: (f64, f64), b: (f64, f64)) -> NurbCurve2D<S> {
        let p = |x: f64, y: f64| Vector3::from_array([S::from_f64(x), S::from_f64(y), S::ONE]);
        NurbCurve::try_new(
            1,
            vec![p(a.0, a.1), p(b.0, b.1)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap()
    }

    fn unit_square_surface<S: Scalar>() -> NurbSurface3D<S> {
        let p =
            |x: f64, y: f64| Vector4::from_array([S::from_f64(x), S::from_f64(y), S::ZERO, S::ONE]);
        NurbSurface3D::try_new(
            1,
            1,
            vec![p(0.0, 0.0), p(0.0, 1.0), p(1.0, 0.0), p(1.0, 1.0)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap()
    }

    /// A triangular face on `NurbSurface3D::everything()` with corners
    /// `(0,0,0) -> (1,0,0) -> (0,1,0)`, whose pcurves are consistent with
    /// `unit_square_surface` (so swapping to it should succeed).
    fn triangle_face_on_everything<S: Scalar>(model: &mut Model<S>) -> crate::FaceId {
        let face_id = model.insert_face(Face {
            surface: NurbSurface3D::everything(),
            outer: BoundaryType::Vertex(crate::VertexId(0)),
            holes: Vec::new(),
            shell: ShellId(999),
        });

        let points_3d = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)];
        let points_2d = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)];
        let n = points_3d.len();
        let verts: Vec<_> = points_3d
            .iter()
            .map(|&(x, y, z)| {
                model.insert_vertex(Vertex {
                    point: Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)]),
                })
            })
            .collect();
        let edges: Vec<_> = (0..n)
            .map(|i| {
                model.insert_edge(Edge {
                    curve: line3(points_3d[i], points_3d[(i + 1) % n]),
                    start_vertex: verts[i],
                    end_vertex: verts[(i + 1) % n],
                })
            })
            .collect();
        let coedges: Vec<_> = (0..n)
            .map(|i| {
                model.insert_coedge(Coedge {
                    geometry: CoedgeGeometry::Edge(edges[i]),
                    sense: Sense::Forward,
                    pcurve: line2(points_2d[i], points_2d[(i + 1) % n]),
                    next: CoedgeId(0),
                    prev: CoedgeId(0),
                    face: face_id,
                })
            })
            .collect();
        for i in 0..n {
            model.coedges.get_mut(&coedges[i]).unwrap().next = coedges[(i + 1) % n];
            model.coedges.get_mut(&coedges[i]).unwrap().prev = coedges[(i + n - 1) % n];
        }
        model.faces.get_mut(&face_id).unwrap().outer = BoundaryType::Loop(coedges[0]);

        face_id
    }

    fn check_replace_face_with_matching_surface_succeeds<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = triangle_face_on_everything(&mut model);
        model.replace_face(face_id, unit_square_surface()).unwrap();
        // Sharpness check: evaluating a pcurve endpoint now goes through the
        // real surface, not `everything()`, and must still land exactly on
        // its edge's vertex.
        let p = model
            .get_face(face_id)
            .unwrap()
            .surface
            .evaluate(S::ONE, S::ZERO)
            .unwrap();
        assert!(p.could_be_equal(&Vector3::from_array([S::ONE, S::ZERO, S::ZERO])));
    }
    #[test]
    fn replace_face_with_matching_surface_succeeds() {
        for_all_scalars!(check_replace_face_with_matching_surface_succeeds);
    }

    fn check_replace_face_with_mismatched_surface_fails_and_does_not_mutate<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = triangle_face_on_everything(&mut model);

        let p = |x: f64, y: f64| {
            Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(5.0), S::ONE])
        };
        let shifted_surface = NurbSurface3D::try_new(
            1,
            1,
            vec![p(0.0, 0.0), p(0.0, 1.0), p(1.0, 0.0), p(1.0, 1.0)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap();

        assert!(model.replace_face(face_id, shifted_surface).is_err());
        // Rejected swap must leave the original `everything()` surface in
        // place, which evaluates to `ENTIRE` (equal to any point) anywhere.
        let p_check = model
            .get_face(face_id)
            .unwrap()
            .surface
            .evaluate(S::ZERO, S::ZERO)
            .unwrap();
        assert!(p_check.could_be_equal(&Vector3::from_array([
            S::from_f64(123.0),
            S::from_f64(456.0),
            S::from_f64(789.0)
        ])));
    }
    #[test]
    fn replace_face_with_mismatched_surface_fails_and_does_not_mutate() {
        for_all_scalars!(check_replace_face_with_mismatched_surface_fails_and_does_not_mutate);
    }
}
