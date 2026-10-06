//! The reference frame an entity gives where it is put, in a mate or
//! anywhere else something is placed by one: where it sits, and which way
//! its `z` axis runs (see [`Aspects::frame_on`]).

use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_solve::mates::across;
use geop_core_topology::{CoedgeGeometry, FaceId};

use super::{Aspects, EntityRef};
use crate::Part;

/// How many points of each of a face's edges its middle is measured from.
const SAMPLES: i64 = 8;

impl<S: Scalar> Aspects<S> {
    /// The frame `on` gives, put at `at` — a vertex of it, or an edge of it,
    /// at whose middle — where that says anything (see [`EntityRef::Frame`]).
    /// In the part, as placed there.
    ///
    /// Its `z` axis is the entity's own, and what is turned about it, or slid
    /// along:
    ///
    /// - A planar face: its normal, out of its solid. `at` is where it sits
    ///   in the plane, else the middle of the face.
    /// - A round face — a cylinder, a cone: its axis. `at` is where along
    ///   it, as near as the axis is to it, else half way along the face.
    /// - A circular edge: the axis of its circle, at its center.
    /// - A straight edge: its direction. `at` is where along it — a vertex
    ///   at one end, say — else the middle.
    /// - A point: the world's axes, there.
    /// - A datum — a frame, an axis, a plane — or a sketch's plane: its own.
    ///
    /// Where the frame sits is a free choice, so it is sharp.
    pub fn frame_on(
        on: &EntityRef,
        at: Option<&EntityRef>,
        part: &Part<S>,
    ) -> GeopResult<CoordinateSystem<S>> {
        let ctx = with_context!("the frame on {on}");
        let aspects = Aspects::of(on, part).with_context(ctx)?;
        if let Some(frame) = aspects.frame {
            return Ok(frame);
        }
        if let Some(arc) = &aspects.arc {
            return axis_frame(arc.circle.center.sharpen(), &arc.circle.normal)
                .with_context(ctx);
        }
        let anchor = at
            .map(|at| middle_of(at, part))
            .transpose()
            .with_context(ctx)?;
        let middle = || middle_of(on, part).with_context(ctx);
        let along = |axis: &Axis<S>| -> GeopResult<CoordinateSystem<S>> {
            let near = match anchor {
                Some(anchor) => anchor,
                None => middle()?,
            };
            axis_frame(axis.project(&near).sharpen(), &axis.direction).with_context(ctx)
        };
        if let Some(plane) = &aspects.plane {
            let near = match anchor {
                Some(anchor) => anchor,
                None => match aspects.face {
                    true => middle()?,
                    false => *plane.origin(),
                },
            };
            let off = near.sub(plane.origin()).prod_dot(plane.w());
            let origin = near.sub(&plane.w().prod_scalar(off)).sharpen();
            return CoordinateSystem::try_new(origin, *plane.u(), *plane.v(), *plane.w())
                .with_context(ctx);
        }
        if let Some(axis) = &aspects.round {
            return along(axis);
        }
        if let Some(axis) = &aspects.line {
            return along(axis);
        }
        if let Some(point) = aspects.point {
            return Ok(CoordinateSystem::world_at(point));
        }
        Err(GeopError::new(format!(
            "{on} gives no frame: pick a face, an edge, a point or a datum"
        )))
        .with_context(ctx)
    }
}

/// The frame at `origin` with `z` along `axis`, and `x` across it the way
/// a joint measures its turns from (see [`across`]): the same for every
/// body drawn the same way round, so two mated on the same axis of each are
/// at a turn of zero.
fn axis_frame<S: Scalar>(
    origin: Vector3<S>,
    axis: &Vector3<S>,
) -> GeopResult<CoordinateSystem<S>> {
    let w = axis.normalize()?;
    let [u, v] = across(&w)?;
    CoordinateSystem::try_new(origin, u, v, w)
}

/// Where an entity is in the middle of itself, in `part`: a vertex's point,
/// the center of a circular edge, the middle of any other edge — of its
/// parameter, not its length — and of a face the middle of the box round
/// its edges, in its own surface. Where a frame sits when it sits nowhere
/// else: a free choice, so sharp.
fn middle_of<S: Scalar>(entity: &EntityRef, part: &Part<S>) -> GeopResult<Vector3<S>> {
    let ctx = with_context!("the middle of {entity}");
    if let Some((name, inner)) = entity.split_instance() {
        let instance = part
            .instance(part.instance_id(&name).with_context(ctx)?)
            .with_context(ctx)?;
        let local = middle_of(&inner, instance.part()).with_context(ctx)?;
        return Ok(instance.pose.motion().apply(&local));
    }
    if let EntityRef::Face { name } = entity {
        return face_middle(part, part.face_id(name).with_context(ctx)?).with_context(ctx);
    }
    let aspects = Aspects::of(entity, part).with_context(ctx)?;
    if let Some(arc) = &aspects.arc {
        return Ok(arc.circle.center.sharpen());
    }
    if let Some(curve) = &aspects.curve {
        let (t0, t1) = curve.domain();
        return Ok(curve
            .evaluate(t0.add(t1).div(S::TWO).with_context(ctx)?.sharpen())
            .with_context(ctx)?
            .sharpen());
    }
    aspects
        .point
        .map(|p| p.sharpen())
        .ok_or_else(|| GeopError::new(format!("{entity} has no middle")))
        .with_context(ctx)
}

/// The middle of the box round the edges of `face`, sampled: the point of a
/// bare vertex face. In the part's own frame.
fn face_middle<S: Scalar>(part: &Part<S>, face: FaceId) -> GeopResult<Vector3<S>> {
    let model = part.topology();
    let mut points = Vec::new();
    for coedge in model.iterate_face_coedges(face) {
        match model.get_coedge(coedge)?.geometry {
            CoedgeGeometry::Edge(edge) => {
                let curve = &model.get_edge(edge)?.curve;
                let (t0, t1) = curve.domain();
                for k in 0..=SAMPLES {
                    let t = S::interpolate(t0, t1, S::from_ratio(k, SAMPLES)?);
                    points.push(curve.evaluate(t)?);
                }
            }
            CoedgeGeometry::Vertex(vertex) => points.push(model.get_vertex(vertex)?.point),
        }
    }
    let [first, rest @ ..] = points.as_slice() else {
        return Err(GeopError::new(format!("face {face} has no edge to measure")));
    };
    let (lo, hi) = rest.iter().fold((*first, *first), |(lo, hi), p| {
        (
            Vector3::from_array([0, 1, 2].map(|k| lo[k].min(p[k]))),
            Vector3::from_array([0, 1, 2].map(|k| hi[k].max(p[k]))),
        )
    });
    Ok(lo.add(&hi).prod_scalar(S::ONE.div(S::TWO)?).sharpen())
}
