//! Section views: the part cut by a plane, with what lies on the side its
//! normal points to taken away — by the kernel's own boolean, subtracting a
//! box standing on the plane from every solid.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::{FaceId, Model, SolidId};
use geop_ops::{Namer, Part, operation::frame_along};
use geop_ops_booleans::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};
use geop_ops_extrude_revolve::{
    common::{Profile, polygon},
    extrude::extrude,
    sweep::SweepLoop,
};

/// The box holding every face of `model`, `[lo, hi]` per axis.
fn bounds<S: Scalar>(model: &Model<S>) -> GeopResult<[[f64; 2]; 3]> {
    let mut b = [[f64::INFINITY, f64::NEG_INFINITY]; 3];
    for face in model.faces.values() {
        for q in &face.surface.control_points {
            for (k, b) in b.iter_mut().enumerate() {
                let x = q[k].div(q[3])?.to_f64();
                b[0] = b[0].min(x);
                b[1] = b[1].max(x);
            }
        }
    }
    Ok(b)
}

/// `part` with every solid cut by the plane through `origin` normal to the
/// unit vector `normal`, the side `normal` points to taken away. A solid
/// wholly on that side is gone; faces standing on their own are left whole.
pub fn section_part<S: Scalar>(
    part: &Part<S>,
    origin: &Vector3<S>,
    normal: &Vector3<S>,
) -> GeopResult<Part<S>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "section_part(origin={origin:?}, normal={normal:?})"
        ))
    };
    let mut cut = part.clone();
    let mut solids: Vec<SolidId> = cut.topology().solids.keys().copied().collect();
    solids.sort_by_key(|s| s.0);
    if solids.is_empty() {
        return Err(GeopError::new("the part has no solid to cut")).with_context(&ctx);
    }
    // A box reaching past the whole part from wherever on the plane.
    let b = bounds(cut.topology()).with_context(&ctx)?;
    let center: Vec<f64> = b.iter().map(|[lo, hi]| (lo + hi) / 2.0).collect();
    let diagonal: f64 = b
        .iter()
        .map(|[lo, hi]| (hi - lo).powi(2))
        .sum::<f64>()
        .sqrt();
    let offset: f64 = (0..3)
        .map(|k| (origin[k].to_f64() - center[k]).powi(2))
        .sum::<f64>()
        .sqrt();
    let reach = S::from_f64(2.0 * (diagonal + offset) + 1.0);
    let plane = frame_along(*origin, normal).with_context(&ctx)?;
    let square = [
        Vector2::from_array([reach.neg(), reach.neg()]),
        Vector2::from_array([reach, reach.neg()]),
        Vector2::from_array([reach, reach]),
        Vector2::from_array([reach.neg(), reach]),
    ];
    for (k, solid) in solids.into_iter().enumerate() {
        let tool_namer = Namer::new("section", &format!("tool{k}"))?;
        let built = extrude(
            &mut cut,
            &tool_namer,
            Some(&tool_namer.root()),
            &plane,
            S::ZERO,
            reach,
            &[SweepLoop::plain(Profile::closed(
                polygon(&square).with_context(&ctx)?,
            ))],
        )
        .with_context(&ctx)?;
        let tool = built
            .solid
            .ok_or_else(|| GeopError::new("the cutting box is no solid"))
            .with_context(&ctx)?;
        let namer = Namer::new("section", &format!("cut{k}"))?;
        boolean(
            &mut cut,
            &namer,
            solid,
            tool,
            BooleanOp::Difference,
            RemeshParams::default(),
        )
        .map_err(|e| ctx(e.with_context(format!("cutting solid {solid}"))))?;
    }
    Ok(cut)
}

/// The faces of `faces` lying in the plane through `origin` normal to
/// `normal`: a section's cut.
pub fn cut_faces<S: Scalar>(
    model: &Model<S>,
    faces: &[FaceId],
    origin: &Vector3<S>,
    normal: &Vector3<S>,
) -> GeopResult<Vec<FaceId>> {
    let mut cut = Vec::new();
    for &face in faces {
        let Some(plane) = model.get_face(face)?.surface.as_plane()? else {
            continue;
        };
        if plane
            .normal
            .prod_cross(normal)
            .could_be_equal(&Vector3::zero())
            && plane
                .point
                .sub(origin)
                .prod_dot(normal)
                .could_be_equal(S::ZERO)
        {
            cut.push(face);
        }
    }
    Ok(cut)
}
