//! What the features here share: search budgets, and reading a solid
//! described as a [`BodySpec`].

use geop_core_geometry::{contains::surface::surface_could_contain, nurb_surface::NurbSurface3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::{
    Curve2, Curve3, Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, FaceSpec},
};

/// Bounds how hard a containment search or a pcurve fit tries: effort, not
/// what an answer means.
pub const MAX_NODES: usize = 20_000;

/// Where a containment search hands over to Newton (see `AGENTS.md`).
pub fn min_subdivision_size<S: Scalar>() -> S {
    S::from_f64(1e-7)
}

/// Newton steps projecting a point onto a surface.
pub const PROJECT_ITERATIONS: usize = 20;

/// The loops of `face`, outer first.
pub fn loops_of<S: Scalar>(face: &FaceSpec<S>) -> Vec<&Vec<CoedgeSpec<S>>> {
    std::iter::once(&face.outer).chain(&face.holes).collect()
}

/// The vertices a coedge runs from and to.
pub fn ends<S: Scalar>(spec: &BodySpec<S>, on: CoedgeOn) -> (usize, usize) {
    match on {
        CoedgeOn::Edge(e, Sense::Forward) => (spec.edges[e].start, spec.edges[e].end),
        CoedgeOn::Edge(e, Sense::Reversed) => (spec.edges[e].end, spec.edges[e].start),
        CoedgeOn::Vertex(v) => (v, v),
    }
}

/// Where `pcurve` starts and ends.
pub fn pcurve_ends<S: Scalar>(pcurve: &Curve2<S>) -> GeopResult<(Vector2<S>, Vector2<S>)> {
    let (t0, t1) = pcurve.domain();
    Ok((pcurve.evaluate(t0)?, pcurve.evaluate(t1)?))
}

/// `curve`, run the way a coedge of `sense` runs it.
pub fn oriented<S: Scalar>(curve: &Curve3<S>, sense: Sense) -> Curve3<S> {
    match sense {
        Sense::Forward => curve.clone(),
        Sense::Reversed => curve.reverse(),
    }
}

/// Halfway along `curve`: the point, and the direction it runs there.
pub fn halfway<S: Scalar>(curve: &Curve3<S>) -> GeopResult<(Vector3<S>, Vector3<S>)> {
    let (t0, t1) = curve.domain();
    let t = S::interpolate(t0, t1, S::from_f64(0.5));
    Ok((curve.evaluate(t)?, curve.tangent(t)?))
}

/// The normal of `surface` at `point`, which lies on it — pointing out of
/// the solid, for a face's surface.
pub fn normal_at<S: Scalar>(
    surface: &NurbSurface3D<S>,
    point: &Vector3<S>,
) -> GeopResult<Vector3<S>> {
    let (u, v) = surface_could_contain(surface, point, MAX_NODES, min_subdivision_size())?
        .ok_or_else(|| GeopError::new(format!("{point:?} is not on the surface")))?;
    let (u, v) = surface.project(*point, u.sharpen(), v.sharpen(), PROJECT_ITERATIONS)?;
    surface.normal(u, v)
}
