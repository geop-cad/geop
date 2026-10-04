//! [`lip`] and [`groove`]: the joint between the two halves of an
//! enclosure — a raised strip along one half's rim, and the recess in the
//! other's that takes it.
//!
//! Both are a band along a chain of the rim face's edges (see
//! [`crate::band`]), swept square off the rim: a lip `width` wide from the
//! edges into the rim and `height` high, joined to the solid; a groove as
//! wide as the lip and a `clearance` more, as deep as the lip is high and
//! a `clearance` more, cut from it. The lip stands flush with the wall the
//! edges are on, so on the edges of a shelled enclosure's rim towards its
//! inside it carries the inner wall on up, and the groove on the mating
//! half takes the inner part of its wall away. The cut reaches past the
//! rim and past the wall into the air, `width` both ways, so that none of
//! its faces lies on a face of the solid.
//!
//! The edges are straight or circular, as a fillet's are; corners between
//! straight edges are mitred, corners at an arc refused. The rim has to be
//! planar, and every edge of the chain convex: the wall falls away from the
//! rim along it, so the space beside the edge, over the wall, is air.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::{
    Body, EdgeId, FaceId,
    build::{BodySpec, CoedgeOn, CoedgeSpec},
};
use geop_ops::{Namer, Part, operation::frame_along};
use geop_ops_booleans::{Combine, Tool};
use geop_ops_extrude_revolve::{extrude::extrude, sweep::SweepLoop};

use crate::{
    band::{Chain, Piece},
    common::{halfway, loops_of, normal_at, oriented},
};

/// The chain of `face`'s edges `edges` — or, if none, its one hole — in
/// the plane of the face: its frame, `w` the face's outward normal, and
/// the chain in it, the face on its left. Fails for a face that is not
/// planar, edges that are not one chain along one of its loops, edges that
/// are neither straight nor circular, and edges the solid is not convex
/// along.
pub fn rim_chain<S: Scalar>(
    part: &Part<S>,
    face: FaceId,
    edges: &[EdgeId],
) -> GeopResult<(CoordinateSystem<S>, Chain<S>)> {
    let model = part.topology();
    let name = |id: geop_ops::RefId| part.name_of(id).unwrap_or("?").to_string();
    let Body::Solid(solid) = model.body_of_face(face)? else {
        return Err(GeopError::new(format!(
            "face {} stands on its own; a rim is a face of a solid",
            name(face.into())
        )));
    };
    let faces = model.solid_faces(solid)?;
    let (spec, sources) = model.body_spec(&faces, true)?;
    let f = sources
        .faces
        .iter()
        .position(|&x| x == face)
        .expect("a face of its solid");
    let plane = spec.faces[f].surface.as_plane()?.ok_or_else(|| {
        GeopError::new(format!(
            "face {} is not planar: a lip or groove runs along a flat rim",
            name(face.into())
        ))
    })?;
    let frame = frame_along(plane.point, &plane.normal)?;
    let loops = loops_of(&spec.faces[f]);
    let edge_index = |e: EdgeId| sources.edges.iter().position(|&x| x == e);

    // The loop, and which of its coedges the chain runs along.
    let (lp, picked): (&Vec<CoedgeSpec<S>>, Vec<bool>) = if edges.is_empty() {
        match spec.faces[f].holes.as_slice() {
            [hole] => (hole, vec![true; hole.len()]),
            holes => {
                return Err(GeopError::new(format!(
                    "face {} has {} holes: pick the edges to run along",
                    name(face.into()),
                    holes.len()
                )));
            }
        }
    } else {
        let wanted: Vec<usize> = edges
            .iter()
            .map(|&e| {
                edge_index(e).ok_or_else(|| {
                    GeopError::new(format!(
                        "edge {} is not an edge of face {}",
                        name(e.into()),
                        name(face.into())
                    ))
                })
            })
            .collect::<GeopResult<_>>()?;
        let on = |c: &CoedgeSpec<S>| matches!(c.on, CoedgeOn::Edge(e, _) if wanted.contains(&e));
        let lp = loops
            .iter()
            .find(|lp| lp.iter().any(on))
            .ok_or_else(|| {
                GeopError::new(format!(
                    "edge {} does not bound face {}",
                    name(edges[0].into()),
                    name(face.into())
                ))
            })?;
        let picked: Vec<bool> = lp.iter().map(on).collect();
        if picked.iter().filter(|&&p| p).count() != wanted.len() {
            return Err(GeopError::new(format!(
                "the edges picked do not all run along one loop of face {}",
                name(face.into())
            )));
        }
        (lp, picked)
    };
    let n = lp.len();
    let closed = picked.iter().all(|&p| p);
    // An open chain starts after a coedge not picked; and is one run.
    let first = if closed {
        0
    } else {
        (0..n)
            .find(|&k| picked[k] && !picked[(k + n - 1) % n])
            .expect("some picked, some not")
    };
    let run: Vec<usize> = (0..n)
        .map(|i| (first + i) % n)
        .take_while(|&k| picked[k])
        .collect();
    if run.len() != picked.iter().filter(|&&p| p).count() {
        return Err(GeopError::new(format!(
            "the edges picked along face {} are not one chain: they leave gaps",
            name(face.into())
        )));
    }

    let to_plane = |p: &Vector3<S>| {
        let uvw = frame.to_uvw(p);
        Vector2::from_array([uvw[0], uvw[1]])
    };
    let mut pieces = Vec::with_capacity(run.len());
    let mut joints = Vec::with_capacity(run.len() + 1);
    for &k in &run {
        let CoedgeOn::Edge(e, sense) = lp[k].on else {
            return Err(GeopError::new(format!(
                "face {} has a loop that is a single point",
                name(face.into())
            )));
        };
        let edge_name = name(sources.edges[e].into());
        let curve = oriented(&spec.edges[e].curve, sense);
        let center = if curve.as_line()?.is_some() {
            None
        } else if let Some(arc) = curve.as_arc()? {
            Some(to_plane(&arc.circle.center))
        } else {
            return Err(GeopError::new(format!(
                "edge {edge_name} is neither straight nor circular: a lip or groove runs along straight and circular edges"
            )));
        };
        convex_along(&spec, e, f, &curve, &plane.normal).map_err(|err| {
            err.with_context(format!("edge {edge_name} of the rim"))
        })?;
        // Into the plane's `(u, v)`, homogeneous: an affine map.
        let mut flat = Vec::with_capacity(curve.control_points.len());
        for cp in &curve.control_points {
            let w = cp[3];
            let p = Vector3::from_array([cp[0], cp[1], cp[2]]).sub(&frame.origin().prod_scalar(w));
            flat.push(Vector3::from_array([
                p.prod_dot(frame.u()),
                p.prod_dot(frame.v()),
                w,
            ]));
        }
        let curve2 = geop_core_geometry::nurb_curve::NurbCurve2D::try_new(
            curve.degree,
            flat,
            curve.knot_vector.clone(),
        )?;
        let (start, _) = crate::common::ends(&spec, lp[k].on);
        joints.push(name(sources.vertices[start].into()));
        pieces.push(Piece {
            curve: curve2,
            center,
            name: edge_name,
        });
    }
    if !closed {
        let (_, end) = crate::common::ends(&spec, lp[*run.last().unwrap()].on);
        joints.push(name(sources.vertices[end].into()));
        // An open chain ends square to its last edge. At a corner of the
        // loop that end lies in the plane of the next edge's wall, along
        // that edge: a face on an edge that already exists. Only where the
        // loop runs on smoothly does it cross the rim cleanly.
        let tangent = |k: usize, at_end: bool| -> GeopResult<Vector3<S>> {
            let CoedgeOn::Edge(e, sense) = lp[k].on else {
                unreachable!("checked above")
            };
            let curve = oriented(&spec.edges[e].curve, sense);
            let (t0, t1) = curve.domain();
            curve.tangent(if at_end { t1 } else { t0 })
        };
        let last = *run.last().unwrap();
        for (inside, outside, at_end, joint) in [
            (first, (first + n - 1) % n, false, &joints[0]),
            (last, (last + 1) % n, true, joints.last().unwrap()),
        ] {
            let (a, b) = if at_end {
                (tangent(inside, true)?, tangent(outside, false)?)
            } else {
                (tangent(outside, true)?, tangent(inside, false)?)
            };
            if !a.prod_cross(&b).norm_sq().could_be_equal(S::ZERO) {
                return Err(GeopError::new(format!(
                    "the edges picked end at vertex {joint}, a corner of face {}: a lip or groove along part of a loop has to end where the loop runs on smoothly — pick the whole loop, or end the chain at a round",
                    name(face.into())
                )));
            }
        }
    }
    // The pieces have to meet exactly: each starts where the one before
    // ended, as the vertex between them.
    let mut chain = Chain {
        pieces,
        joints,
        closed,
    };
    stitch(&mut chain)?;
    Ok((frame, chain))
}

/// Makes each piece of `chain` start exactly where the one before ends:
/// both ends were mapped from the same vertex, each by its own control
/// point; the two enclosures of the one point are intersected.
fn stitch<S: Scalar>(chain: &mut Chain<S>) -> GeopResult<()> {
    let n = chain.pieces.len();
    let count = if chain.closed { n } else { n - 1 };
    for i in 0..count {
        let j = (i + 1) % n;
        let end = chain.pieces[i].curve.control_points.last().unwrap().clone();
        let start = chain.pieces[j].curve.control_points[0];
        let p = |h: &Vector3<S>| -> GeopResult<Vector2<S>> {
            Ok(Vector2::from_array([h[0].div(h[2])?, h[1].div(h[2])?]))
        };
        let (a, b) = (p(&end)?, p(&start)?);
        if !a.could_be_equal(&b) {
            return Err(GeopError::new(format!(
                "{} does not end where {} starts",
                chain.pieces[i].name, chain.pieces[j].name
            )));
        }
        let at = Vector2::from_array([a[0].intersect(b[0]), a[1].intersect(b[1])]);
        let set = |h: &mut Vector3<S>| {
            let w = h[2];
            *h = Vector3::from_array([at[0].mul(w), at[1].mul(w), w]);
        };
        set(chain.pieces[i].curve.control_points.last_mut().unwrap());
        set(&mut chain.pieces[j].curve.control_points[0]);
    }
    Ok(())
}

/// Fails unless the solid is convex along edge `e` of the rim face `f`
/// (outward normal `up`), which `curve` runs along the way `f`'s loop
/// does: the other face's outward normal, halfway along, points away from
/// the rim — to the right of the edge, the rim lying on its left.
fn convex_along<S: Scalar>(
    spec: &BodySpec<S>,
    e: usize,
    f: usize,
    curve: &geop_core_topology::Curve3<S>,
    up: &Vector3<S>,
) -> GeopResult<()> {
    let other = spec
        .faces
        .iter()
        .enumerate()
        .filter(|&(g, _)| g != f)
        .find(|(_, face)| {
            loops_of(face)
                .iter()
                .any(|lp| lp.iter().any(|c| matches!(c.on, CoedgeOn::Edge(x, _) if x == e)))
        })
        .map(|(g, _)| g)
        .ok_or_else(|| GeopError::new("the edge bounds the rim alone"))?;
    let (point, along) = halfway(curve)?;
    let right = along.prod_cross(up);
    let normal = normal_at(&spec.faces[other].surface, &point)?;
    if !normal.prod_dot(&right).definitely_greater(S::ZERO) {
        return Err(GeopError::new(
            "the solid is not convex along it: a lip or groove runs along an edge where the wall falls away from the rim",
        ));
    }
    Ok(())
}

/// The sizes of a lip, or of the lip a groove takes.
#[derive(Clone, Copy, Debug)]
pub struct LipSize<S: Scalar> {
    pub width: S,
    pub height: S,
}

fn check_size<S: Scalar>(what: &str, value: S) -> GeopResult<()> {
    if !value.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "a lip's {what} is more than zero, not {:?}",
            value.to_f64()
        )));
    }
    Ok(())
}

/// Raises a lip along the edges `edges` of the rim face `face` (see the
/// module docs), joined to its solid, for the operation `operation_id`
/// whose names `namer` builds: the result is named `namer`'s root, the
/// lip's faces along an edge `E` `N(E)` (on the wall) and `N(E,far)`
/// (inside the rim), its top `N(end)`.
pub fn lip<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    operation_id: &str,
    face: FaceId,
    edges: &[EdgeId],
    size: LipSize<S>,
) -> GeopResult<()> {
    let ctx = with_context!("lip({}, face={face}, edges={edges:?}, {size:?})", namer.root());
    check_size("width", size.width).with_context(ctx)?;
    check_size("height", size.height).with_context(ctx)?;
    let target = target_name(part, face).with_context(ctx)?;
    let (frame, chain) = rim_chain(part, face, edges).with_context(ctx)?;
    let loops: Vec<SweepLoop<S>> = chain
        .band(S::ZERO, size.width)
        .with_context(ctx)?
        .into_iter()
        .map(SweepLoop::plain)
        .collect();
    let built = extrude(
        part,
        namer,
        Some(&namer.name(&["tool"])),
        &frame,
        S::ZERO,
        size.height,
        &loops,
    )
    .with_context(ctx)?;
    let tool = Tool {
        solid: built.solid.expect("extruded as a solid"),
        up_to_next: None,
        scope: None,
    };
    Combine::Union { target }
        .apply(part, namer, operation_id, &[tool])
        .with_context(ctx)
}

/// Cuts the groove that takes a lip of `size` along the edges `edges` of
/// the rim face `face` (see the module docs), `clearance` wider and deeper,
/// from its solid; named like a [`lip`].
pub fn groove<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    operation_id: &str,
    face: FaceId,
    edges: &[EdgeId],
    size: LipSize<S>,
    clearance: S,
) -> GeopResult<()> {
    let ctx = with_context!(
        "groove({}, face={face}, edges={edges:?}, {size:?}, clearance={clearance:?})",
        namer.root()
    );
    check_size("width", size.width).with_context(ctx)?;
    check_size("height", size.height).with_context(ctx)?;
    if clearance.definitely_less(S::ZERO) {
        return Err(GeopError::new(format!(
            "a groove's clearance is not less than zero, not {:?}",
            clearance.to_f64()
        )))
        .with_context(ctx);
    }
    let target = target_name(part, face).with_context(ctx)?;
    let (frame, chain) = rim_chain(part, face, edges).with_context(ctx)?;
    // Past the wall into the air, and above the rim, by the lip's width: a
    // free choice, only so no face of the cut lies on one of the solid.
    let air = size.width;
    let loops: Vec<SweepLoop<S>> = chain
        .band(air.neg(), size.width.add(clearance))
        .with_context(ctx)?
        .into_iter()
        .map(SweepLoop::plain)
        .collect();
    let built = extrude(
        part,
        namer,
        Some(&namer.name(&["tool"])),
        &frame,
        air,
        size.height.add(clearance).neg(),
        &loops,
    )
    .with_context(ctx)?;
    let tool = Tool {
        solid: built.solid.expect("extruded as a solid"),
        up_to_next: None,
        scope: None,
    };
    Combine::Difference { target }
        .apply(part, namer, operation_id, &[tool])
        .with_context(ctx)
}

/// The name of the solid `face` is a face of.
fn target_name<S: Scalar>(part: &Part<S>, face: FaceId) -> GeopResult<String> {
    let Body::Solid(solid) = part.topology().body_of_face(face)? else {
        return Err(GeopError::new("the rim stands on its own: it is no face of a solid"));
    };
    part.name_of(solid)
        .map(str::to_string)
        .ok_or_else(|| GeopError::new("the rim's solid has no name"))
}
