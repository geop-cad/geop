//! [`rib`]: a thin wall grown from an open sketch profile until it meets a
//! solid, and joined to it.
//!
//! **Its ends.** Each end of the profile runs on along its tangent, in the
//! sketch's plane, into the wall it points at: to the middle of that wall,
//! between where the line enters the solid and where it leaves it again —
//! a place the geometry defines, so the rib's end face lies on no face of
//! the solid. An end already inside the solid stays where it is; one that
//! points at nothing of the solid is refused, by name.
//!
//! **Its growth.** Normal to the sketch, the profile is thickened in its
//! plane — on one side, the other, or both alike (see [`crate::band`]) —
//! and swept along the plane's normal. Parallel to the sketch, the profile
//! is swept in its plane, square to the chord between its ends, and
//! thickened along the plane's normal. Either way the sweep reaches well
//! past the solid, and only what grows from the profile until it runs into
//! the solid is joined: the same "up to next" an extrude goes by
//! ([`geop_ops_booleans::Combine`] with a [`Tool`] going up to the next
//! face). A profile that is not stopped by the solid all along is refused.

use geop_core_geometry::{
    intersection::curve_surface_intersect,
    nurb_curve::{NurbCurve2D, NurbCurve3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    plain,
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::{
    Body, SolidId,
    contains::{
        face::{PointClassification as OnFace, face_contains},
        shell::{Classified, PointClassification, solid_contains},
    },
};
use geop_ops_booleans::{Combine, Tool, boolean::Reach, extent::face_hull};
use geop_ops_extrude_revolve::{common::Profile, extrude::extrude, sweep::SweepLoop};
use geop_ops_model::{NamedModel, Namer, ToolReach};

use geop_core_geometry::search::{MAX_NODES, min_subdivision_size};

use crate::band::{Chain, Piece};

/// Seeds the ray casts' containment queries: any fixed value keeps a rib
/// reproducible run to run.
const SEED: u64 = 0x7269_6273;

/// Which way a rib grows from its profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Growth {
    /// Along the sketch plane's normal — backwards if `flipped`.
    Normal { flipped: bool },
    /// In the sketch plane, square to the chord between the profile's ends,
    /// to its left — to its right if `flipped`.
    Parallel { flipped: bool },
}

/// Grows a rib `(near, far)` thick — from `near` to `far` to the left of the
/// profile in its plane, growing normal to it; from `near` to `far` along
/// the plane's normal, growing parallel to it — from the open profile
/// `profile` in `plane`, up to the solid `target`, and joins it, for the
/// operation `operation_id` whose names `namer` builds: the result is named
/// `namer`'s root. See the module docs.
#[allow(clippy::too_many_arguments)]
pub fn rib<S: Scalar>(
    part: &mut NamedModel<S>,
    namer: &Namer,
    operation_id: &str,
    plane: &CoordinateSystem<S>,
    profile: &Profile<S>,
    target: SolidId,
    (near, far): (S, S),
    growth: Growth,
) -> GeopResult<()> {
    let ctx = with_context!(
        "rib({}, target={target}, thickness=({near:?}, {far:?}), {growth:?})",
        namer.root()
    );
    if !far.sub(near).definitely_greater(S::ZERO) {
        return Err(GeopError::new("a rib is thicker than zero")).with_context(ctx);
    }
    if profile.is_closed() {
        return Err(GeopError::new(
            "a rib grows from an open profile: its sketch encloses an area",
        ))
        .with_context(ctx);
    }
    let target_name = part.name(target).with_context(ctx)?.to_string();
    let chain = extended(part, plane, profile, target).with_context(ctx)?;
    let reach = reach_past(part, plane, target).with_context(ctx)?;
    let tool_name = namer.name(&["tool"]);
    let (built, start, end) = match growth {
        Growth::Normal { flipped } => {
            let loops: Vec<SweepLoop<S>> = chain
                .band(near, far)
                .with_context(ctx)?
                .into_iter()
                .map(SweepLoop::plain)
                .collect();
            let to = if flipped { reach.neg() } else { reach };
            let built = extrude(part, namer, Some(&tool_name), plane, S::ZERO, to, &loops)
                .with_context(ctx)?;
            (built, namer.name(&["start"]), namer.name(&["end"]))
        }
        Growth::Parallel { flipped } => {
            let swept = swept_region(&chain, reach, flipped).with_context(ctx)?;
            let built = extrude(
                part,
                namer,
                Some(&tool_name),
                plane,
                near,
                far,
                &[SweepLoop::plain(swept)],
            )
            .with_context(ctx)?;
            // Grown from the profile's first piece: the profile lies in
            // one open space, so the piece it grows from is the one that
            // piece lies in.
            (
                built,
                namer.name(&[&profile.curve_names[0]]),
                namer.name(&["far"]),
            )
        }
    };
    let tool = Tool {
        solid: built.solid.expect("extruded as a solid"),
        reach: ToolReach {
            up_to_next: Some((start, end)),
            scope: None,
        },
    };
    let combine = Combine::join(target_name);
    match combine.apply(part, namer, operation_id, &[tool]).with_context(ctx)? {
        Reach::Stopped(()) => Ok(()),
        Reach::RunsOn(e) => Err(e.with_context(
            "the rib's profile grows past the solid without meeting it all along: it has to run between faces that close it in",
        ))
        .with_context(ctx),
    }
}

/// How far along a direction of `plane`, from its origin, reaches past all
/// of `target`: its distance to the furthest corner of the box around the
/// solid's control points, and that box's diagonal again.
fn reach_past<S: Scalar>(
    part: &NamedModel<S>,
    plane: &CoordinateSystem<S>,
    target: SolidId,
) -> GeopResult<S> {
    let model = part.topology();
    let hull = face_hull(model, &model.body_faces(Body::Solid(target))?)?;
    let (lo, hi) = plain::bounds(hull)
        .ok_or_else(|| GeopError::new(format!("solid {target:?} has no faces")))?;
    let origin = [0, 1, 2].map(|k| plane.origin()[k].to_f64());
    let diagonal = plain::diagonal(lo, hi);
    let furthest = (0..8)
        .map(|corner: usize| {
            (0..3)
                .map(|k| {
                    let x = if corner >> k & 1 == 1 { hi[k] } else { lo[k] };
                    (x - origin[k]).powi(2)
                })
                .sum::<f64>()
                .sqrt()
        })
        .fold(0.0, f64::max);
    Ok(S::from_f64(furthest + diagonal))
}

/// The profile as a [`Chain`], each end run on along its tangent into the
/// wall of `target` it points at (see the module docs): a straight piece
/// `ext0` before it, `ext1` after it — or none, for an end inside the
/// solid already.
fn extended<S: Scalar>(
    part: &NamedModel<S>,
    plane: &CoordinateSystem<S>,
    profile: &Profile<S>,
    target: SolidId,
) -> GeopResult<Chain<S>> {
    let lift = |p: &Vector2<S>| plane.uv_to_xyz(p);
    let mut pieces = Vec::with_capacity(profile.curves.len() + 2);
    for (curve, name) in profile.curves.iter().zip(&profile.curve_names) {
        pieces.push(Piece {
            curve: curve.clone(),
            center: arc_center(curve)?,
            name: name.clone(),
        });
    }
    let mut joints = profile.joint_names.clone();
    let reach = reach_past(part, plane, target)?;
    let first = &profile.curves[0];
    let last = profile.curves.last().expect("a profile has curves");
    let (t0, _) = first.domain();
    let (_, t1) = last.domain();
    let start = first.start()?;
    let end = last.end()?;
    let back = first.tangent(t0)?.normalize()?.neg();
    let on = last.tangent(t1)?.normalize()?;
    let into_wall = |from: Vector2<S>, along: Vector2<S>, joint: &str| {
        wall_middle(
            part,
            target,
            lift(&from),
            &plane.uv_direction(&along),
            reach,
        )
        .map_err(|e| e.with_context(format!("running the rib's end {joint} on")))
    };
    if let Some(at) = into_wall(start, back, &joints[0])? {
        let to = plane.to_uv(&at);
        pieces.insert(
            0,
            Piece {
                curve: NurbCurve2D::line(to, start)?,
                center: None,
                name: "ext0".into(),
            },
        );
        joints.insert(0, "end0".into());
    }
    let last_joint = joints.last().expect("joints").clone();
    if let Some(at) = into_wall(end, on, &last_joint)? {
        let to = plane.to_uv(&at);
        pieces.push(Piece {
            curve: NurbCurve2D::line(end, to)?,
            center: None,
            name: "ext1".into(),
        });
        joints.push("end1".into());
    }
    Ok(Chain {
        pieces,
        joints,
        closed: false,
    })
}

/// The centre of the circle `curve` runs along, if it is an arc.
fn arc_center<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<Option<Vector2<S>>> {
    if curve.degree == 1 {
        return Ok(None);
    }
    let lifted = curve.lifted()?;
    if lifted.as_line()?.is_some() {
        return Ok(None);
    }
    let arc = lifted.as_arc()?.ok_or_else(|| {
        GeopError::new("a rib's profile is made of lines and arcs, and this piece is neither")
    })?;
    Ok(Some(Vector2::from_array([
        arc.circle.center[0],
        arc.circle.center[1],
    ])))
}

/// Where a line from `from` along the unit vector `along` should end in the
/// wall of `target` it runs into: halfway between where it enters the
/// solid and where it leaves it again — or, from a point on the solid's
/// boundary, halfway to where it leaves. None for a point inside the solid:
/// it is in the wall already. `reach` is a length past all of the solid.
fn wall_middle<S: Scalar>(
    part: &NamedModel<S>,
    target: SolidId,
    from: Vector3<S>,
    along: &Vector3<S>,
    reach: S,
) -> GeopResult<Option<Vector3<S>>> {
    let model = part.topology();
    let start = match solid_contains(
        model,
        target,
        from,
        MAX_NODES,
        min_subdivision_size(),
        SEED,
        &mut Classified::default(),
    )? {
        PointClassification::Inside => return Ok(None),
        PointClassification::Outside => None,
        _ => Some(0.0),
    };
    let ray = NurbCurve3D::line(from, from.add(&along.prod_scalar(reach)))?;
    // Where the line crosses the solid's boundary, as fractions of `reach`
    // — a crossing on an edge once, though both its faces report it.
    let mut hits: Vec<S> = Vec::new();
    for face in model.body_faces(Body::Solid(target))? {
        let surface = &model.get_face(face)?.surface;
        let found =
            curve_surface_intersect(&ray, surface, MAX_NODES, MAX_NODES, min_subdivision_size())?;
        if found.is_coincident() {
            return Err(GeopError::new(format!(
                "it runs along face {} of the solid: draw the profile off the solid's faces",
                part.name_of(face).unwrap_or("?")
            )));
        }
        for (t, uv) in found.into_vec() {
            if !t.definitely_greater(S::ZERO) {
                continue;
            }
            let class = face_contains(
                model,
                face,
                uv[0].midpoint(),
                uv[1].midpoint(),
                MAX_NODES,
                min_subdivision_size(),
                SEED ^ face.0,
            )?;
            if class != OnFace::Outside && !hits.iter().any(|h| h.could_be_equal(t)) {
                hits.push(t);
            }
        }
    }
    hits.sort_by(|a, b| a.to_f64().total_cmp(&b.to_f64()));
    let (enter, leave) = match (start, hits.as_slice()) {
        (Some(on), [leave, ..]) => (on, leave.to_f64()),
        (None, [enter, leave, ..]) => (enter.to_f64(), leave.to_f64()),
        _ => {
            return Err(GeopError::new(
                "it points at nothing of the solid: a rib's profile has to point at the walls it runs between",
            ));
        }
    };
    // The middle of the wall is a free choice: sharp.
    let t = S::from_f64((enter + leave) / 2.0);
    Ok(Some(from.add(&along.prod_scalar(reach.mul(t)))))
}

/// The region `chain` sweeps in its plane, square to the chord between its
/// ends, `reach` far — to the chord's left, or its right if `flipped` — as
/// one counter-clockwise loop: the chain, the side from its end, the far
/// side `far`, the side back to its start. Every piece has to run forwards
/// along the chord, or the region would fold over itself.
fn swept_region<S: Scalar>(chain: &Chain<S>, reach: S, flipped: bool) -> GeopResult<Profile<S>> {
    let first = &chain.pieces[0].curve;
    let last = &chain.pieces.last().expect("pieces").curve;
    let start = first.start()?;
    let end = last.end()?;
    let chord = end.sub(&start).normalize()?;
    for piece in &chain.pieces {
        let (t0, t1) = piece.curve.domain();
        for t in [t0, t1] {
            if !piece
                .curve
                .tangent(t)?
                .prod_dot(&chord)
                .definitely_greater(S::ZERO)
            {
                return Err(GeopError::new(format!(
                    "{} turns back against the direction from the profile's start to its end: grown parallel to the sketch, the rib would fold over itself",
                    piece.name
                )));
            }
        }
    }
    let left = chord.perp();
    let sweep = if flipped { left.neg() } else { left }.prod_scalar(reach);
    let (end_far, start_far) = (end.add(&sweep), start.add(&sweep));
    let mut curves: Vec<NurbCurve2D<S>> = chain.pieces.iter().map(|p| p.curve.clone()).collect();
    curves.push(NurbCurve2D::line(end, end_far)?);
    curves.push(NurbCurve2D::line(end_far, start_far)?);
    curves.push(NurbCurve2D::line(start_far, start)?);
    let mut curve_names: Vec<String> = chain.pieces.iter().map(|p| p.name.clone()).collect();
    curve_names.extend(["side1".into(), "far".into(), "side0".into()]);
    let mut joint_names = chain.joints.clone();
    joint_names.extend(["far1".into(), "far0".into()]);
    let region = Profile {
        name: String::new(),
        curves,
        curve_names,
        joint_names,
    };
    // Swept to the chord's left, the region lies on the chain's left: the
    // loop runs counter-clockwise.
    Ok(if flipped { region.reversed() } else { region })
}
