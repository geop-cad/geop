//! Test fixture — this crate's tests, and others' with the `testing`
//! feature: a plain axis-aligned unit-cube-shaped solid, described whole
//! and built at once (see [`crate::build`]).
//!
//! This crate sits *below* `geop-lib-sweep` in the workspace's
//! dependency order (that crate is what actually builds shapes like a real
//! `cube_solid` via `extrude`), so validation/edit tests here that
//! just need "some realistic populated `Model`" to run structural or
//! numerical checks against can't reach for it — a dev-dependency back on
//! `geop-lib-sweep` would make Cargo compile this crate twice
//! (once under `#[cfg(test)]`, once as that crate's plain library
//! dependency), and the two copies' `Model` types don't unify, which
//! surfaces as `expected Model, found Model` at every call site. This is a
//! small, self-contained stand-in scoped to exactly what these tests need:
//! six planar faces, twelve shared edges, eight shared vertices — no
//! `extrude` generality (arbitrary footprints, holes) required.

use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::GeopError,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

use crate::{
    Model, Sense, SolidId,
    build::{BodySpec, CoedgeOn, EdgeSpec},
    validation::ValidationParameters,
};

/// Build `model`'s unit cube (corners at `0`/`1` on every axis) and return
/// its `SolidId`. Every face is a flat, degree-`1` bilinear patch; every
/// edge a straight degree-`1` line — plenty for the structural/numerical
/// checks these tests actually exercise.
pub fn test_cube_solid(model: &mut Model) -> SolidId {
    let mut spec = BodySpec::new(true);
    // 0..3 the bottom (z=0) ring, 4..7 the matching top (z=1) ring.
    for [x, y, z] in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ] {
        spec.vertices
            .push(Vector3::from_array([x, y, z].map(Scalar::from_f64)));
    }
    // Each ring, and the edges rising from the bottom one to the top.
    let pairs = (0..4)
        .flat_map(|k| [(k, (k + 1) % 4), (4 + k, 4 + (k + 1) % 4), (k, 4 + k)])
        .collect::<Vec<_>>();
    for &(start, end) in &pairs {
        spec.edges.push(EdgeSpec {
            curve: NurbCurve3D::line(spec.vertices[start], spec.vertices[end]).unwrap(),
            start,
            end,
        });
    }
    let edge = |a: usize, b: usize| {
        let e = pairs
            .iter()
            .position(|&p| p == (a, b) || p == (b, a))
            .expect("an edge of the cube");
        CoedgeOn::Edge(e, Sense::Forward)
    };
    // Each face's corners in `(0,0) -> (1,0) -> (1,1) -> (0,1)` order, so
    // that `Su x Sv` points away from the solid's interior.
    for corners in [
        [0, 3, 2, 1], // bottom
        [4, 5, 6, 7], // top
        [0, 1, 5, 4], // front (y=0)
        [3, 7, 6, 2], // back (y=1)
        [0, 4, 7, 3], // left (x=0)
        [1, 2, 6, 5], // right (x=1)
    ] {
        let at = |k: usize| spec.vertices[corners[k]];
        let surface = NurbSurface3D::bilinear(at(0), at(1), at(2), at(3)).unwrap();
        let sides = std::array::from_fn(|k| edge(corners[k], corners[(k + 1) % 4]));
        let face = spec.square_face(surface, corners, sides).unwrap();
        spec.faces.push(face);
    }
    spec.shells = vec![(0..6).collect()];
    model
        .build_body(spec)
        .unwrap()
        .solid
        .expect("built as a solid")
}

/// A new model holding only [`test_cube_solid`].
pub fn test_cube_model() -> Model {
    let mut model = Model::new();
    test_cube_solid(&mut model);
    model
}

/// One check of [`crate::validation`]: it adds what it finds wrong with a
/// model to the errors.
pub type Check = fn(&ValidationParameters, &mut Vec<GeopError>, &Model);

/// The errors `check` finds in `model`, under the default parameters.
pub fn check_errors(check: Check, model: &Model) -> Vec<GeopError> {
    let mut errors = Vec::new();
    check(&ValidationParameters::default(), &mut errors, model);
    errors
}

/// The straight line from `a` to `b`, over `[0, 1]`.
pub fn line3(a: (f64, f64, f64), b: (f64, f64, f64)) -> NurbCurve3D {
    let p = |(x, y, z): (f64, f64, f64)| Vector3::from_array([x, y, z].map(Scalar::from_f64));
    NurbCurve3D::line(p(a), p(b)).unwrap()
}

/// The straight pcurve from `(u, v)` `a` to `b`, over `[0, 1]`.
pub fn line2(a: (f64, f64), b: (f64, f64)) -> NurbCurve2D {
    let p = |(u, v): (f64, f64)| Vector2::from_array([u, v].map(Scalar::from_f64));
    NurbCurve2D::line(p(a), p(b)).unwrap()
}
