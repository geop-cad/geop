//! Section views: the part cut by a plane, with what lies on the side its
//! normal points to taken away — by the kernel's own boolean, subtracting a
//! box standing on the plane from every solid.

use geop_core_geometry::aabb::union_aabb;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    plain,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::{FaceId, Model, SolidId};
use geop_lib_booleans::{RemeshParams, boolean};
use geop_lib_sweep::{SweepLoop, extrude, polygon};
use geop_ops_model::{BooleanOp, Profile};
use geop_ops_model::{NamedModel, Namer, frame_along};

/// The box holding every face of `model`, `[lo, hi]` per axis.
fn bounds(model: &Model) -> GeopResult<[[f64; 2]; 3]> {
    let b = union_aabb(model.faces.values().map(|f| f.surface.aabb()))
        .ok_or_else(|| GeopError::new("the part has no face"))?;
    Ok(b.map(|x| [x.lower().to_f64(), x.upper().to_f64()]))
}

/// `model` with every solid cut by the plane through `origin` normal to the
/// unit vector `normal`, the side `normal` points to taken away. A solid
/// wholly on that side is gone; faces standing on their own are left whole.
pub fn section_part(
    model: &NamedModel,
    origin: &Vector3,
    normal: &Vector3,
) -> GeopResult<NamedModel> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "section_part(origin={origin:?}, normal={normal:?})"
        ))
    };
    let mut cut = model.clone();
    // Ordered by id, as the model's arenas are.
    let solids: Vec<SolidId> = cut.topology().solids.keys().copied().collect();
    if solids.is_empty() {
        return Err(GeopError::new("the part has no solid to cut")).with_context(&ctx);
    }
    // A box reaching past the whole part from wherever on the plane.
    let b = bounds(cut.topology()).with_context(&ctx)?;
    let center: Vec<f64> = b.iter().map(|[lo, hi]| (lo + hi) / 2.0).collect();
    let diagonal = plain::diagonal(b.map(|[lo, _]| lo), b.map(|[_, hi]| hi));
    let offset: f64 = (0..3)
        .map(|k| (origin[k].to_f64() - center[k]).powi(2))
        .sum::<f64>()
        .sqrt();
    let reach = Scalar::from_f64(2.0 * (diagonal + offset) + 1.0);
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
            &[Scalar::ZERO, reach],
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
pub fn cut_faces(
    model: &Model,
    faces: &[FaceId],
    origin: &Vector3,
    normal: &Vector3,
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
                .could_be_equal(Scalar::ZERO)
        {
            cut.push(face);
        }
    }
    Ok(cut)
}
