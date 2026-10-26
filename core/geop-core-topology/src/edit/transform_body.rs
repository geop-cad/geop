use geop_core_geometry::{aabb::union_aabb, deform::Deformation};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    vector::Vector3,
};

use crate::{
    Body, Model, boundary::BoundaryType, contains::face::face_interior_point,
    validation::ValidationParameters,
};

/// Seed of the ray casting that finds a point of a face for a map made of
/// pieces, whose answer does not depend on it.
const SEED: u64 = 0x7A5F_0D1E_2B3C_4D5E;

impl Model {
    /// Moves the whole of `body` by `map` in place — a rigid motion, a
    /// translation, a mirror, a scaling
    /// ([`geop_core_math::primitives::Motion`]) or a deformation about an
    /// axis ([`geop_core_geometry::deform::AxialMap`]): every vertex, every
    /// edge's curve and every face's surface. The pcurves stay as they are:
    /// every map keeps the surfaces' parametrization, so each pcurve still
    /// traces the same patch, now moved. Nothing is created or deleted, and
    /// the body stays connected exactly as it was.
    ///
    /// A map made of pieces moves each edge and face by the piece it lies
    /// in, asking for a point of it (the curve's middle, a point inside the
    /// face) only where its control points straddle two pieces: the body
    /// has to be divided first where it crosses from one to the next.
    ///
    /// A mirror turns every surface's normal into the body (see
    /// [`geop_core_geometry::nurb_surface::NurbSurface::transform`]), so
    /// it also turns every face around ([`Model::reverse_face`]): a mirrored
    /// solid is a valid solid again, its normals pointing out. Each edge is
    /// shared by two faces of the body, both turned, so their coedges stay
    /// opposite.
    ///
    /// A translation moves every point by one addition per coordinate;
    /// a rotation or a mirror encloses its result honestly, widening every
    /// point by what its rounded matrix leaves open.
    pub fn transform_body(
        &mut self,
        body: impl Into<Body>,
        map: &dyn Deformation,
    ) -> GeopResult<()> {
        let body = body.into();
        let ctx = |e: GeopError| e.with_context(format!("Model::transform_body(body={body})"));
        let vertices: Vec<_> = self.iter_body_vertices(body).with_context(&ctx)?.collect();
        let edges: Vec<_> = self.iter_body_edges(body).with_context(&ctx)?.collect();
        let faces = self.body_faces(body).with_context(&ctx)?;
        // Everything moved from the body as it is, then written.
        let points = (vertices.iter())
            .map(|v| map.point(&self.vertices[v].point))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(&ctx)?;
        let curves = (edges.iter())
            .map(|&edge| {
                let curve = &self.edges[&edge].curve;
                let at = || curve.evaluate(curve.mid_parameter()?);
                map.curve(curve, &at)
                    .map_err(|e| e.with_context(format!("moving edge {edge}")))
            })
            .collect::<GeopResult<Vec<_>>>()
            .with_context(&ctx)?;
        let surfaces = (faces.iter())
            .map(|&face| {
                let whole = &self.faces[&face].surface;
                // Cut down to the box in `(u, v)` around the face's outer
                // loop, which the face lies in: the parametrization stays.
                let outer = match self.faces[&face].outer {
                    BoundaryType::Loop(anchor) => (self.iterate_loop_coedges(anchor))
                        .map(|c| Ok(*self.get_coedge(c)?.pcurve.aabb()))
                        .collect::<GeopResult<Vec<_>>>()?,
                    BoundaryType::Vertex(_) => Vec::new(),
                };
                let restricted = match union_aabb(&outer) {
                    Some(uv) if !map.affine() => Some(whole.sub_surface(
                        (uv[0].lower(), uv[0].upper()),
                        (uv[1].lower(), uv[1].upper()),
                    )?),
                    _ => None,
                };
                let surface = restricted.as_ref().unwrap_or(whole);
                let at = || -> GeopResult<Vector3> {
                    let params = ValidationParameters::default();
                    let (u, v) = face_interior_point(
                        self,
                        face,
                        params.max_nodes,
                        params.min_subdivision_size,
                        SEED,
                    )?;
                    surface.evaluate(u.sharpen(), v.sharpen())
                };
                map.surface(surface, &at)
                    .map_err(|e| e.with_context(format!("moving face {face}")))
            })
            .collect::<GeopResult<Vec<_>>>()
            .with_context(&ctx)?;
        for (vertex, point) in vertices.into_iter().zip(points) {
            self.get_vertex_mut(vertex).with_context(&ctx)?.point = point;
        }
        for (edge, curve) in edges.into_iter().zip(curves) {
            self.get_edge_mut(edge).with_context(&ctx)?.curve = curve;
        }
        for (&face, surface) in faces.iter().zip(surfaces) {
            self.get_face_mut(face).with_context(&ctx)?.surface = surface;
        }
        if map.mirrors() {
            for face in faces {
                self.reverse_face(face).with_context(&ctx)?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        primitives::{Motion, Pose},
        scalars::Scalar,
        vector::Vector3,
    };

    use crate::{
        Model,
        test_fixtures::test_cube_solid,
        validation::{ValidationParameters, validate},
    };

    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    fn assert_valid(model: &Model) {
        if let Err(errors) = validate(&ValidationParameters::default(), model) {
            let messages: Vec<String> = errors.iter().map(|e| format!("{e:?}")).collect();
            panic!("{}", messages.join("\n"));
        }
    }

    /// A translation moves every corner by exactly the offset: one
    /// addition, nothing multiplied.
    #[test]
    fn translation_is_exact() {
        let mut model = Model::new();
        let cube = test_cube_solid(&mut model);
        let before = model.clone();
        let offset = v(2.0, -3.0, 0.5);
        model
            .transform_body(cube, &Motion::translation(offset))
            .unwrap();
        assert_valid(&model);
        for (id, vertex) in &model.vertices {
            let expected = before.vertices[id].point.add(&offset);
            assert_eq!(
                format!("{:?}", vertex.point),
                format!("{expected:?}"),
                "moved by one addition"
            );
        }
    }

    /// A rotation at an awkward angle about a slanted axis leaves a valid
    /// solid, its corners where the pose puts them.
    #[test]
    fn rotated_cube_stays_valid() {
        let mut model = Model::new();
        let cube = test_cube_solid(&mut model);
        let before = model.clone();
        let axis = v(1.0, 2.0, 3.0).normalize().unwrap();
        let pose =
            Pose::rotation_about(&v(0.5, -1.0, 2.0), &axis, Scalar::from_f64(1.234)).unwrap();
        model.transform_body(cube, &pose.motion()).unwrap();
        assert_valid(&model);
        for (id, vertex) in &model.vertices {
            let expected = pose.apply(&before.vertices[id].point);
            assert!(vertex.point.could_be_equal(&expected));
        }
    }

    /// Mirrored, the cube is turned inside out unless its faces are turned
    /// around: validation checks that every normal points out.
    #[test]
    fn mirrored_cube_stays_valid() {
        let mut model = Model::new();
        let cube = test_cube_solid(&mut model);
        let before = model.clone();
        let mirror = Motion::mirror(&v(2.0, 0.0, 0.0), &v(1.0, 0.0, 0.0)).unwrap();
        model.transform_body(cube, &mirror).unwrap();
        assert_valid(&model);
        for (id, vertex) in &model.vertices {
            let p = before.vertices[id].point;
            let expected = v(4.0, 0.0, 0.0).add(&Vector3::from_array([p[0].neg(), p[1], p[2]]));
            assert!(vertex.point.could_be_equal(&expected), "{vertex:?}");
        }
    }
}
