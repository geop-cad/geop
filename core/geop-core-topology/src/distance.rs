//! [`closest_points`]: the least distance between two points, curves or
//! faces of models — each where a pose puts it — and the two points that
//! attain it.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
};

use crate::{
    Curve3, FaceId, Model,
    contains::face::{PointClassification, face_contains},
};

/// Samples per polynomial piece of a surface taken as seeds — and, those
/// inside the face, as the face's samples. Bounds effort only.
const FACE_SAMPLES: usize = 4;

/// Samples per polynomial piece of a curve, for the initial search.
const CURVE_SAMPLES: usize = 8;

/// How often the nearest points are moved, each onto the other feature, at
/// most. Bounds effort only: each move can only shorten the distance, and
/// what is returned is attained whenever the moves stop.
const MAX_ALTERNATIONS: usize = 64;

/// The search budget and tolerance of the containment test that decides
/// whether a foot point lies inside a face's trim (see [`face_contains`]).
const CONTAINS_MAX_NODES: usize = 20_000;
const CONTAINS_EPSILON: f64 = 1e-7;
const SEED: u64 = 0xD157_A1CE_0000_0001;

/// A face of a model, where `pose` puts it — or where it is, without one —
/// with what a distance search starts from: samples of it, and its edges.
pub struct PlacedFace<'m, S: Scalar> {
    model: &'m Model<S>,
    face: FaceId,
    pose: Option<Pose<S>>,
    /// Parameters of the surface's sample grid, all of them: seeds for a
    /// projection.
    seeds: Vec<(S, S)>,
    /// Where those of them inside the face are, placed.
    inside: Vec<Vector3<S>>,
    /// The curves of its edges, in the model's own frame.
    edges: Vec<Curve3<S>>,
}

impl<'m, S: Scalar> PlacedFace<'m, S> {
    pub fn new(model: &'m Model<S>, face: FaceId, pose: Option<Pose<S>>) -> GeopResult<Self> {
        let ctx = |e: GeopError| e.with_context(format!("PlacedFace::new(face={face})"));
        let surface = &model.get_face(face).with_context(&ctx)?.surface;
        let seeds = surface.sample_parameters(FACE_SAMPLES).with_context(&ctx)?;
        let mut inside = Vec::new();
        for &(u, v) in &seeds {
            if contains(model, face, u, v).with_context(&ctx)? {
                inside.push(place(&pose, surface.evaluate(u, v)?));
            }
        }
        let mut edges = Vec::new();
        for coedge in model.iterate_face_coedges(face) {
            if let Ok(edge) = model.get_coedge(coedge)?.edge() {
                edges.push(model.get_edge(edge)?.curve.clone());
            }
        }
        Ok(Self {
            model,
            face,
            pose,
            seeds,
            inside,
            edges,
        })
    }
}

/// Whether `(u, v)` lies on `face`: inside its trim, or on its boundary.
fn contains<S: Scalar>(model: &Model<S>, face: FaceId, u: S, v: S) -> GeopResult<bool> {
    let class = face_contains(
        model,
        face,
        u,
        v,
        CONTAINS_MAX_NODES,
        S::from_f64(CONTAINS_EPSILON),
        SEED,
    )?;
    Ok(class != PointClassification::Outside)
}

/// `p` where `pose` puts it.
fn place<S: Scalar>(pose: &Option<Pose<S>>, p: Vector3<S>) -> Vector3<S> {
    match pose {
        Some(pose) => pose.apply(&p),
        None => p,
    }
}

/// Something a distance is measured to.
pub enum Feature<'m, S: Scalar> {
    Point(Vector3<S>),
    /// A curve, already where it is.
    Curve(Curve3<S>),
    Face(PlacedFace<'m, S>),
}

/// The least distance found between two features, and where on each it is
/// attained.
#[derive(Clone, Copy, Debug)]
pub struct Closest<S: Scalar> {
    pub distance: S,
    pub a: Vector3<S>,
    pub b: Vector3<S>,
}

impl<S: Scalar> Feature<'_, S> {
    /// Points of it the search starts from.
    fn samples(&self) -> GeopResult<Vec<Vector3<S>>> {
        Ok(match self {
            Feature::Point(p) => vec![*p],
            Feature::Curve(curve) => curve_samples(curve)?,
            Feature::Face(face) => {
                let mut points = face.inside.clone();
                for edge in &face.edges {
                    points.extend(
                        curve_samples(edge)?
                            .into_iter()
                            .map(|p| place(&face.pose, p)),
                    );
                }
                points
            }
        })
    }

    /// Its point nearest `target`. Of a face, the nearest of the foot point
    /// on its surface — if that lies inside the trim — and the nearest
    /// points of its edges: where the nearest point of a face is not a foot
    /// point inside it, it is on its boundary.
    pub fn closest_to(&self, target: &Vector3<S>) -> GeopResult<Vector3<S>> {
        match self {
            Feature::Point(p) => Ok(*p),
            Feature::Curve(curve) => Ok(curve.closest_point(target)?.1),
            Feature::Face(face) => {
                let local = match &face.pose {
                    Some(pose) => pose.inverse().apply(target),
                    None => *target,
                };
                let surface = &face.model.get_face(face.face)?.surface;
                let mut candidates = Vec::new();
                let (u, v, foot) = surface.closest_point_from(&local, &face.seeds)?;
                if contains(face.model, face.face, u, v)? {
                    candidates.push(foot);
                }
                for edge in &face.edges {
                    candidates.push(edge.closest_point(&local)?.1);
                }
                let nearest = candidates
                    .into_iter()
                    .min_by(|a, b| distance(a, &local).total_cmp(&distance(b, &local)))
                    .ok_or_else(|| {
                        GeopError::new(format!(
                            "face {}: the foot point is outside it, and it has no edges",
                            face.face
                        ))
                    })?;
                Ok(place(&face.pose, nearest))
            }
        }
    }
}

/// Samples of `curve`: [`CURVE_SAMPLES`] per polynomial piece, and its end.
fn curve_samples<S: Scalar>(curve: &Curve3<S>) -> GeopResult<Vec<Vector3<S>>> {
    let breaks = curve.breakpoints();
    let mut points = Vec::new();
    for w in breaks.windows(2) {
        for i in 0..CURVE_SAMPLES {
            let alpha = S::from_ratio(i as i64, CURVE_SAMPLES as i64)?;
            points.push(curve.evaluate(S::interpolate(w[0], w[1], alpha).sharpen())?);
        }
    }
    points.push(curve.evaluate(curve.domain().1)?);
    Ok(points)
}

/// The squared distance's midpoint: what the search compares candidates by
/// — which of two points to keep is a free choice, never an answer.
fn distance<S: Scalar>(a: &Vector3<S>, b: &Vector3<S>) -> f64 {
    a.sub(b).norm_sq().to_f64()
}

/// The least distance between `a` and `b`, and the points attaining it.
///
/// The nearest pair of their samples is moved in turns, each point onto the
/// other feature's point nearest it ([`Feature::closest_to`]), until a move
/// no longer shortens the distance. Every point the search holds lies on
/// its feature, so the distance returned is attained by the two points
/// returned: it is never less than the least distance, and it is the least
/// one wherever the samples found the right neighbourhood. The distance is
/// the enclosure of `|a - b|` for those two points.
pub fn closest_points<S: Scalar>(a: &Feature<S>, b: &Feature<S>) -> GeopResult<Closest<S>> {
    let (sa, sb) = (a.samples()?, b.samples()?);
    let mut best: Option<(f64, Vector3<S>, Vector3<S>)> = None;
    for pa in &sa {
        for pb in &sb {
            let d = distance(pa, pb);
            if best.as_ref().is_none_or(|(e, ..)| d < *e) {
                best = Some((d, *pa, *pb));
            }
        }
    }
    let (mut d, mut pa, mut pb) =
        best.ok_or_else(|| GeopError::new("closest_points: a feature has no points"))?;
    for _ in 0..MAX_ALTERNATIONS {
        let qb = b.closest_to(&pa)?;
        let qa = a.closest_to(&qb)?;
        let e = distance(&qa, &qb);
        if e >= d {
            break;
        }
        (d, pa, pb) = (e, qa, qb);
    }
    Ok(Closest {
        distance: pa.sub(&pb).norm(),
        a: pa,
        b: pb,
    })
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        primitives::Pose,
        scalars::{ScalInF64 as S, Scalar},
        vector::Vector3,
    };

    use super::{Feature, PlacedFace, closest_points};
    use crate::{Model, test_fixtures::test_cube_solid};

    fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([x, y, z].map(S::from_f64))
    }

    /// A point off the unit cube is nearest its face in front of it, a
    /// point off a corner nearest the corner — over every face, the least
    /// distance is the true one, attained by points on both.
    #[test]
    fn a_point_and_the_faces_of_a_cube() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        let least = |target: Vector3<S>, pose: Option<Pose<S>>| {
            model
                .faces
                .keys()
                .map(|&face| {
                    let face = Feature::Face(PlacedFace::new(&model, face, pose).unwrap());
                    closest_points(&Feature::Point(target), &face).unwrap()
                })
                .min_by(|a, b| a.distance.to_f64().total_cmp(&b.distance.to_f64()))
                .unwrap()
        };
        let front = least(v(2.0, 0.3, 0.6), None);
        assert!(front.distance.could_be_equal(S::ONE), "{front:?}");
        assert!(front.b.could_be_equal(&v(1.0, 0.3, 0.6)), "{front:?}");
        let corner = least(v(2.0, 2.0, 2.0), None);
        assert!(
            corner.distance.could_be_equal(S::from_f64(3f64.sqrt())),
            "{corner:?}"
        );
        // The cube moved up by 5: the same point is now 4.4 below its bottom.
        let up = Pose::identity().with_position(v(0.0, 0.0, 5.0));
        let below = least(v(0.5, 0.5, 0.6), Some(up));
        assert!(below.distance.could_be_equal(S::from_f64(4.4)), "{below:?}");
    }
}
