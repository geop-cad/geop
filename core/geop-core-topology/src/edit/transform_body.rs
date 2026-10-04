use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Motion,
    scalars::Scalar,
};

use crate::{Body, Model};

impl<S: Scalar> Model<S> {
    /// Moves the whole of `body` by `motion` — a rigid motion, a
    /// translation, or a mirror — in place: every vertex, every edge's
    /// curve and every face's surface. The pcurves stay as they are: the
    /// surfaces keep their parametrization, so each pcurve still traces
    /// the same patch, now moved. Nothing is created or deleted, and the
    /// body stays connected exactly as it was.
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
    pub fn transform_body(&mut self, body: impl Into<Body>, motion: &Motion<S>) -> GeopResult<()> {
        let body = body.into();
        let ctx = |e: GeopError| e.with_context(format!("Model::transform_body(body={body})"));
        let vertices: Vec<_> = self.iter_body_vertices(body).with_context(&ctx)?.collect();
        let edges: Vec<_> = self.iter_body_edges(body).with_context(&ctx)?.collect();
        let faces = self.body_faces(body).with_context(&ctx)?;
        for vertex in vertices {
            let v = self.get_vertex_mut(vertex).with_context(&ctx)?;
            v.point = motion.apply(&v.point);
        }
        for edge in edges {
            let e = self.get_edge_mut(edge).with_context(&ctx)?;
            e.curve = e.curve.transform(motion);
        }
        for &face in &faces {
            let f = self.get_face_mut(face).with_context(&ctx)?;
            f.surface = f.surface.transform(motion);
        }
        if motion.mirrors() {
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
        for_all_scalars,
        primitives::{Motion, Pose},
        scalars::Scalar,
        vector::Vector3,
    };

    use crate::{
        Model,
        test_fixtures::test_cube_solid,
        validation::{ValidationParameters, validate},
    };

    fn v<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([x, y, z].map(S::from_f64))
    }

    fn assert_valid<S: Scalar>(model: &Model<S>) {
        if let Err(errors) = validate(&ValidationParameters::default(), model) {
            let messages: Vec<String> = errors.iter().map(|e| format!("{e:?}")).collect();
            panic!("{}", messages.join("\n"));
        }
    }

    /// A translation moves every corner by exactly the offset: one
    /// addition, nothing multiplied.
    fn check_translation_is_exact<S: Scalar>() {
        let mut model = Model::<S>::new();
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
    #[test]
    fn translation_is_exact() {
        for_all_scalars!(check_translation_is_exact);
    }

    /// A rotation at an awkward angle about a slanted axis leaves a valid
    /// solid, its corners where the pose puts them.
    fn check_rotated_cube_stays_valid<S: Scalar>() {
        let mut model = Model::<S>::new();
        let cube = test_cube_solid(&mut model);
        let before = model.clone();
        let axis = v::<S>(1.0, 2.0, 3.0).normalize().unwrap();
        let pose =
            Pose::rotation_about(&v(0.5, -1.0, 2.0), &axis, S::from_f64(1.234)).unwrap();
        model.transform_body(cube, &pose.motion()).unwrap();
        assert_valid(&model);
        for (id, vertex) in &model.vertices {
            let expected = pose.apply(&before.vertices[id].point);
            assert!(vertex.point.could_be_equal(&expected));
        }
    }
    #[test]
    fn rotated_cube_stays_valid() {
        for_all_scalars!(check_rotated_cube_stays_valid);
    }

    /// Mirrored, the cube is turned inside out unless its faces are turned
    /// around: validation checks that every normal points out.
    fn check_mirrored_cube_stays_valid<S: Scalar>() {
        let mut model = Model::<S>::new();
        let cube = test_cube_solid(&mut model);
        let before = model.clone();
        let mirror = Motion::mirror(&v(2.0, 0.0, 0.0), &v(1.0, 0.0, 0.0)).unwrap();
        model.transform_body(cube, &mirror).unwrap();
        assert_valid(&model);
        for (id, vertex) in &model.vertices {
            let p = before.vertices[id].point;
            let expected = v::<S>(4.0, 0.0, 0.0).add(&Vector3::from_array([p[0].neg(), p[1], p[2]]));
            assert!(vertex.point.could_be_equal(&expected), "{vertex:?}");
        }
    }
    #[test]
    fn mirrored_cube_stays_valid() {
        for_all_scalars!(check_mirrored_cube_stays_valid);
    }
}
