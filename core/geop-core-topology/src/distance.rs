//! [`closest_points`]: the least distance between two points, curves or
//! faces of models — each where a pose puts it — and two points that
//! attain it.
//!
//! A branch and bound over pieces of the two ([`least_distance`]): spans
//! of curves, patches of a face's surface and spans of its edges. A patch
//! the face's trim definitely leaves out is dropped, and a patch only
//! known to meet the trim attains a point where its middle is on the face.
//! The least distance of a face is either at a point inside it or on its
//! boundary, which its edges' spans attain.

use geop_core_geometry::{
    nearest::{CurveSpan, Piece, SurfacePatch, least_distance},
    search::{MAX_NODES, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
};

use crate::{
    Curve3, EdgeId, FaceId, Model,
    contains::face::{PointClassification, face_contains},
};

const SEED: u64 = 0xD157_A1CE_0000_0001;

/// How often the two points are refined, each by Newton towards the other,
/// at most. Bounds effort only: each refinement keeps the point on its
/// feature and is kept only if it brings the two nearer.
const MAX_ALTERNATIONS: usize = 64;

/// A face of a model, where `pose` puts it — or where it is, without one.
#[derive(Clone)]
pub struct PlacedFace<'m, S: Scalar> {
    model: &'m Model<S>,
    face: FaceId,
    pose: Option<Pose<S>>,
}

impl<'m, S: Scalar> PlacedFace<'m, S> {
    pub fn new(model: &'m Model<S>, face: FaceId, pose: Option<Pose<S>>) -> GeopResult<Self> {
        model.get_face(face).with_context(&|e: GeopError| {
            e.with_context(format!("PlacedFace::new(face={face})"))
        })?;
        Ok(Self { model, face, pose })
    }

    /// Where `(u, v)` — a point, or a box of them — is against the face's
    /// trim.
    fn classify(&self, u: S, v: S) -> GeopResult<PointClassification> {
        face_contains(
            self.model,
            self.face,
            u,
            v,
            MAX_NODES,
            min_subdivision_size(),
            SEED,
        )
    }

    /// `p` where the pose puts it.
    fn place(&self, p: Vector3<S>) -> Vector3<S> {
        place(&self.pose, p)
    }

    /// `p` in the face's own frame, where it is without the pose.
    fn local(&self, p: &Vector3<S>) -> Vector3<S> {
        match &self.pose {
            Some(pose) => pose.inverse().apply(p),
            None => *p,
        }
    }
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

/// The least distance between two features, and two points of them
/// attaining as little as the search could tell apart from it.
#[derive(Clone, Copy, Debug)]
pub struct Closest<S: Scalar> {
    /// An enclosure of the least distance: from what the search proved no
    /// two points are nearer than, to the distance between `a` and `b`.
    pub distance: S,
    pub a: Vector3<S>,
    pub b: Vector3<S>,
}

/// Where on a feature a point of it is.
#[derive(Clone, Copy, Debug)]
enum Spot<S: Scalar> {
    Point,
    /// On a [`Feature::Curve`], at this parameter.
    Curve(S),
    /// On an edge of a face, at this parameter of its curve.
    Edge(EdgeId, S),
    /// On a face, at these parameters of its surface.
    Face(S, S),
}

impl<S: Scalar> Feature<'_, S> {
    /// From `spot`, Newton's point of the feature nearest `target` around
    /// it — or `spot` itself where the step would leave the face — and
    /// where it is. Which point of the feature is a free choice; it stays
    /// one of its points.
    fn refine(&self, spot: Spot<S>, target: &Vector3<S>) -> GeopResult<(Spot<S>, Vector3<S>)> {
        match (self, spot) {
            (Feature::Point(p), _) => Ok((Spot::Point, *p)),
            (Feature::Curve(curve), Spot::Curve(t)) => {
                let t = curve.closest_parameter_from(target, t)?;
                Ok((Spot::Curve(t), curve.evaluate(t)?))
            }
            (Feature::Face(face), Spot::Edge(edge, t)) => {
                let curve = &face.model.get_edge(edge)?.curve;
                let t = curve.closest_parameter_from(&face.local(target), t)?;
                Ok((Spot::Edge(edge, t), face.place(curve.evaluate(t)?)))
            }
            (Feature::Face(face), Spot::Face(u, v)) => {
                let surface = &face.model.get_face(face.face)?.surface;
                let (ru, rv) = surface.refine_closest(&face.local(target), u, v)?;
                let (u, v) = match face.classify(ru, rv)? {
                    PointClassification::Outside => (u, v),
                    _ => (ru, rv),
                };
                Ok((Spot::Face(u, v), face.place(surface.evaluate(u, v)?)))
            }
            (_, spot) => Err(GeopError::new(format!(
                "Feature::refine: {spot:?} is not on this feature"
            ))),
        }
    }
}

/// A piece of a [`Feature`], for [`least_distance`].
#[derive(Clone)]
enum FeaturePiece<'m, S: Scalar> {
    Point(Vector3<S>),
    /// A span of a curve — of `edge`, if it is a face's — where `pose`
    /// puts it.
    Curve {
        span: CurveSpan<S>,
        edge: Option<EdgeId>,
        pose: Option<Pose<S>>,
    },
    /// A patch of a face's surface; `inside` once it is known to lie
    /// inside the trim.
    Patch {
        face: PlacedFace<'m, S>,
        patch: SurfacePatch<S>,
        inside: bool,
    },
}

impl<'m, S: Scalar> FeaturePiece<'m, S> {
    /// The pieces `feature` starts as.
    fn roots(feature: &Feature<'m, S>) -> GeopResult<Vec<Self>> {
        Ok(match feature {
            Feature::Point(p) => vec![FeaturePiece::Point(*p)],
            Feature::Curve(curve) => vec![FeaturePiece::Curve {
                span: CurveSpan(curve.clone()),
                edge: None,
                pose: None,
            }],
            Feature::Face(face) => {
                let surface = &face.model.get_face(face.face)?.surface;
                let mut pieces: Vec<Self> =
                    Self::patch(face, SurfacePatch(surface.clone()), false)?
                        .into_iter()
                        .collect();
                for edge in face.model.face_edges(face.face) {
                    pieces.push(FeaturePiece::Curve {
                        span: CurveSpan(face.model.get_edge(edge)?.curve.clone()),
                        edge: Some(edge),
                        pose: face.pose,
                    });
                }
                pieces
            }
        })
    }

    /// `patch` of `face`, unless the trim definitely leaves all of it out.
    /// A patch inside its parent is inside too.
    fn patch(
        face: &PlacedFace<'m, S>,
        patch: SurfacePatch<S>,
        inside: bool,
    ) -> GeopResult<Option<Self>> {
        let inside = inside || {
            let ((u0, u1), (v0, v1)) = (patch.0.domain_u(), patch.0.domain_v());
            match face.classify(u0.union(u1), v0.union(v1))? {
                PointClassification::Outside => return Ok(None),
                PointClassification::Inside => true,
                PointClassification::OnVertex | PointClassification::OnCoedge => false,
            }
        };
        Ok(Some(FeaturePiece::Patch {
            face: face.clone(),
            patch,
            inside,
        }))
    }
}

impl<S: Scalar> Piece<S> for FeaturePiece<'_, S> {
    type At = Spot<S>;

    fn hull(&self) -> GeopResult<Vec<Vector3<S>>> {
        Ok(match self {
            FeaturePiece::Point(p) => vec![*p],
            FeaturePiece::Curve { span, pose, .. } => {
                span.hull()?.into_iter().map(|p| place(pose, p)).collect()
            }
            FeaturePiece::Patch { face, patch, .. } => {
                patch.hull()?.into_iter().map(|p| face.place(p)).collect()
            }
        })
    }

    fn point(&self) -> GeopResult<Option<(Spot<S>, Vector3<S>)>> {
        Ok(match self {
            FeaturePiece::Point(p) => Some((Spot::Point, *p)),
            FeaturePiece::Curve { span, edge, pose } => span.point()?.map(|(t, p)| {
                let spot = match edge {
                    Some(edge) => Spot::Edge(*edge, t),
                    None => Spot::Curve(t),
                };
                (spot, place(pose, p))
            }),
            FeaturePiece::Patch {
                face,
                patch,
                inside,
            } => {
                let (u, v) = patch.middle()?;
                let on = *inside || face.classify(u, v)? != PointClassification::Outside;
                on.then(|| {
                    Ok::<_, GeopError>((Spot::Face(u, v), face.place(patch.0.evaluate(u, v)?)))
                })
                .transpose()?
            }
        })
    }

    fn split(&self, handoff: S) -> GeopResult<Option<Vec<Self>>> {
        Ok(match self {
            FeaturePiece::Point(_) => None,
            FeaturePiece::Curve { span, edge, pose } => span.split(handoff)?.map(|halves| {
                halves
                    .into_iter()
                    .map(|span| FeaturePiece::Curve {
                        span,
                        edge: *edge,
                        pose: *pose,
                    })
                    .collect()
            }),
            FeaturePiece::Patch {
                face,
                patch,
                inside,
            } => match patch.split(handoff)? {
                None => None,
                Some(halves) => {
                    let mut kept = Vec::new();
                    for half in halves {
                        kept.extend(Self::patch(face, half, *inside)?);
                    }
                    Some(kept)
                }
            },
        })
    }
}

/// The least distance between `a` and `b`, and two points attaining it
/// (see the module docs).
///
/// The search encloses the least distance, but the points attaining it
/// only as sharply as the square root of that: the distance is flat where
/// it is least. So the two points are then refined in turns, each by
/// Newton towards the other, as long as that brings them nearer.
pub fn closest_points<S: Scalar>(a: &Feature<S>, b: &Feature<S>) -> GeopResult<Closest<S>> {
    let nearest = least_distance(
        FeaturePiece::roots(a)?,
        FeaturePiece::roots(b)?,
        min_subdivision_size(),
        MAX_NODES,
    )?;
    let ((mut sa, mut pa), (mut sb, mut pb)) = (nearest.a, nearest.b);
    // Which of two attained pairs to keep is a free choice.
    let apart = |p: &Vector3<S>, q: &Vector3<S>| p.sub(q).norm_sq().to_f64();
    for _ in 0..MAX_ALTERNATIONS {
        let before = apart(&pa, &pb);
        let (nb, qb) = b.refine(sb, &pa)?;
        if apart(&pa, &qb) < apart(&pa, &pb) {
            (sb, pb) = (nb, qb);
        }
        let (na, qa) = a.refine(sa, &pb)?;
        if apart(&qa, &pb) < apart(&pa, &pb) {
            (sa, pa) = (na, qa);
        }
        if apart(&pa, &pb) >= before {
            break;
        }
    }
    Ok(Closest {
        distance: nearest.distance.lower().union(pa.sub(&pb).norm()),
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
