use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};

use crate::{
    FaceId, Model, ShellId, SolidId,
    boundary::BoundaryType,
    contains::{
        face::face_interior_point_where,
        shell::{Beside, on_solid_boundary_besides, solid_contains_beside},
    },
    uv_loop::loop_signed_area,
    validation::ValidationParameters,
};

/// Fixed seed for the ray casting behind both checks. `solid_contains_beside`
/// and `face_interior_point` retry until they find a ray grazing nothing, so
/// their answers are seed-independent; a constant keeps validation
/// reproducible run to run.
const SEED: u64 = 0x0F1E_0D1E_0000_0001;

/// Checks that every face's outer boundary runs counter-clockwise in
/// `(u, v)` and every hole runs clockwise.
///
/// This is the convention the kernel *computes with*, not merely a tidiness
/// rule: `Model::splice_edge_into_face` uses a ring's signed area to decide
/// whether splitting a hole produced a new face or another hole — the two
/// rings pass through the same vertices and share the same edge, so nothing
/// weaker can tell them apart — and the debug renderer uses it to inset a
/// coedge's trim curve inward. A face whose winding is inverted is
/// structurally perfect and every other check accepts it, while every
/// operation that asks which side the material is on gets the opposite
/// answer.
pub fn check_loop_winding(
    _params: &ValidationParameters,
    errors: &mut Vec<GeopError>,
    model: &Model,
) {
    for (&face_id, face) in &model.faces {
        for (which, boundary, want_positive) in
            std::iter::once(("outer".to_string(), face.outer, true)).chain(
                face.holes
                    .iter()
                    .enumerate()
                    .map(|(i, &h)| (format!("hole {i}"), h, false)),
            )
        {
            // A bare-vertex boundary encloses nothing and has no winding.
            let BoundaryType::Loop(anchor) = boundary else {
                continue;
            };
            // A ring that walks every one of its edges twice — out along
            // each and back — encloses no area by construction, so it has no
            // winding to check. `Model::splice_edge_into_face` builds exactly
            // these: an imprinted edge whose endpoints are not yet on the
            // face becomes a two-coedge dangling loop, and a spur inserts the
            // same pair into an existing ring. Both are "wire" topology
            // rather than a trim boundary, and demanding a sign from them
            // reports every imprint as broken.
            if traverses_each_edge_twice(model, anchor) {
                continue;
            }
            let area = match loop_signed_area(model, anchor) {
                Ok(area) => area,
                Err(e) => {
                    errors.push(e.with_context(format!(
                        "face {face_id}'s {which} boundary, measuring its winding"
                    )));
                    continue;
                }
            };
            let ok = if want_positive {
                area.definitely_greater(Scalar::ZERO)
            } else {
                area.definitely_less(Scalar::ZERO)
            };
            if !ok {
                let expected = if want_positive {
                    "counter-clockwise (positive)"
                } else {
                    "clockwise (negative)"
                };
                errors.push(GeopError::new(format!(
                    "face {face_id}'s {which} boundary has signed area {area:?}, but must be {expected} — an outer loop bounds material and a hole removes it, and that is what the sign records"
                )));
            }
        }
    }
}

/// Whether every edge in the ring anchored at `anchor` is traversed twice —
/// the signature of a dangling loop or a spur, which encloses no area.
fn traverses_each_edge_twice(model: &Model, anchor: crate::CoedgeId) -> bool {
    let mut counts: std::collections::HashMap<crate::EdgeId, usize> =
        std::collections::HashMap::new();
    let cap = model.coedges.len() + 1;
    let mut seen = 0usize;
    for coedge_id in model.iterate_loop_coedges(anchor).take(cap) {
        seen += 1;
        let Some(coedge) = model.coedges.get(&coedge_id) else {
            return false;
        };
        match coedge.geometry {
            crate::CoedgeGeometry::Edge(edge_id) => {
                *counts.entry(edge_id).or_default() += 1;
            }
            // A degenerate vertex coedge bounds nothing either way.
            crate::CoedgeGeometry::Vertex(_) => {}
        }
    }
    seen < cap && !counts.is_empty() && counts.values().all(|&n| n == 2)
}

/// Checks that every face's normal points *out* of the solid it bounds.
///
/// A face's normal comes from its surface's parametrization (`Su x Sv`), and
/// nothing about a well-formed loop constrains which way it ends up facing.
/// An inverted face still validates structurally, still renders, and still
/// has a perfectly good boundary — but every operation that asks which side
/// the material is on gets the wrong answer, which for a boolean means the
/// face is kept when it should be dropped, or kept facing into the result.
///
/// Decided at an interior point of the face, by which side of the face the
/// solid is on there (see [`normal_points_outward`]).
pub fn check_normals_point_outward(
    params: &ValidationParameters,
    errors: &mut Vec<GeopError>,
    model: &Model,
) {
    // A sheet bounds nothing, so either side of it may face anywhere.
    for (&shell_id, shell) in &model.shells {
        // Measured against the whole solid: a void's faces point into the
        // void, out of the material around it.
        let Some(solid_id) = shell.solid else {
            continue;
        };
        for &face_id in &shell.faces {
            match normal_points_outward(params, model, solid_id, shell_id, face_id) {
                Ok(Some(Outward::No(inward))) => {
                    let Inward {
                        point,
                        uv,
                        derivatives,
                        normal,
                        ray,
                    } = *inward;
                    errors.push(GeopError::new(format!(
                        "face {face_id}'s normal points into solid {solid_id} (from its shell {shell_id}) rather than out of it: at its interior point {point:?}, (u, v) = {uv:?} where (S_u, S_v) = {derivatives:?}, the normal is {normal:?}, and the solid is on the normal's side of the face there: the ray along {:?} from it crosses {} face(s) past it, {:?}",
                        ray.direction,
                        ray.crossings.len(),
                        ray.crossings
                    )));
                }
                Ok(_) => {}
                Err(e) => errors.push(e.with_context(format!(
                    "face {face_id} of shell {shell_id}, deciding which side of it solid {solid_id} is on"
                ))),
            }
        }
    }
}

/// Whether a face's normal points out of its solid, as
/// [`normal_points_outward`] found it.
#[derive(Clone, Debug)]
pub enum Outward {
    Yes,
    /// It points in: at `point`, inside the face, the normal is `normal`,
    /// and `ray`, leaving the face to that side, found the solid.
    No(Box<Inward>),
}

/// Where and how a normal was found pointing in (see [`Outward::No`]).
#[derive(Clone, Debug)]
pub struct Inward {
    pub point: Vector3,
    pub uv: (Scalar, Scalar),
    /// `(S_u, S_v)` there: the parametrization the normal comes from.
    pub derivatives: (Vector3, Vector3),
    pub normal: Vector3,
    pub ray: Beside,
}

/// Whether the normal of face `face_id` of shell `shell_id` points out of
/// the solid `solid_id` — `None` for a face without an interior point, which
/// `check_faces_have_interior` reports, or without one clear of the rest of
/// the solid's boundary.
///
/// Decided at an interior point of the face: the normal points out exactly
/// when the solid is not on its side of the face there
/// ([`solid_contains_beside`]). That is a local fact, the limit as the
/// distance off the face goes to zero, and it is decided by a ray leaving
/// the face, not by a point some distance off it: on a non-convex solid a
/// point any fixed distance off the face may lie in another part of it.
///
/// The interior point is a free choice, and one on the solid's boundary
/// elsewhere too is a bad one: a ray from the pole of an inscribed sphere,
/// lying in the cube's face, passes through that vertex at its start. The
/// first point offered that is on nothing else is taken
/// ([`on_solid_boundary_besides`]).
pub fn normal_points_outward(
    params: &ValidationParameters,
    model: &Model,
    solid_id: SolidId,
    shell_id: ShellId,
    face_id: FaceId,
) -> GeopResult<Option<Outward>> {
    let Some(face) = model.faces.get(&face_id) else {
        return Ok(None);
    };
    let clear = |u: Scalar, v: Scalar| -> GeopResult<bool> {
        Ok(!on_solid_boundary_besides(
            model,
            solid_id,
            face_id,
            &face.surface.evaluate(u, v)?,
            params.max_nodes,
            params.min_subdivision_size,
            SEED,
        )?)
    };
    let Ok(Some((u, v))) = face_interior_point_where(
        model,
        face_id,
        params.max_nodes,
        params.min_subdivision_size,
        SEED,
        clear,
    ) else {
        return Ok(None);
    };
    let (point, normal) = (face.surface.evaluate(u, v)?, face.surface.normal(u, v)?);
    let beside = solid_contains_beside(
        model,
        solid_id,
        shell_id,
        face_id,
        point,
        normal,
        params.max_nodes,
        params.min_subdivision_size,
        SEED,
    )
    .with_context(&|e: GeopError| {
        e.with_context(format!(
            "normal_points_outward(face={face_id}, at (u, v) = ({u:?}, {v:?}) of a surface over {:?} x {:?})",
            face.surface.domain_u(),
            face.surface.domain_v()
        ))
    })?;
    Ok(Some(if beside.inside {
        Outward::No(Box::new(Inward {
            point,
            uv: (u, v),
            derivatives: face.surface.derivatives(u, v)?,
            normal,
            ray: beside,
        }))
    } else {
        Outward::Yes
    }))
}
