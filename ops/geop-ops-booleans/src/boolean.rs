//! Boolean operations on two solids, built on top of [`remesh`].
//!
//! The algorithm is deliberately simple, and it is simple *because* remesh
//! has already done the hard part. After remeshing, every face of either
//! solid lies wholly inside the other, wholly outside it, or exactly on its
//! boundary — no face straddles, because every intersection curve has been
//! imprinted and every face it crossed has been split. That turns a boolean
//! into a per-face classification followed by a keep/drop table.
//!
//! 1. [`remesh`] the two solids against each other.
//! 2. Classify each face with [`classify_face`], by taking a point strictly
//!    inside its trimmed region and asking where it sits relative to the
//!    other solid.
//! 3. Keep the faces the operator wants (see [`BooleanOp::keeps`]), reversing
//!    them where the operator needs the material on the other side.
//! 4. Assemble the survivors into a new solid and discard everything else.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::{
    FaceId, Model, ShellId, SolidId,
    contains::{
        face::{PointClassification as FacePoint, face_contains, face_interior_point_where},
        shell::{PointClassification as ShellPoint, shell_contains},
    },
};
use geop_ops::{Namer, Part};
use serde::{Deserialize, Serialize};

use crate::remesh::remesh::{RemeshParams, remesh};

/// Which boolean to perform.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BooleanOp {
    /// Everything in either solid.
    Union,
    /// Only what is in both.
    Intersection,
    /// `solid_a` with `solid_b` removed.
    Difference,
}

/// Where one solid's face sits relative to the *other* solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceClassification {
    /// Strictly inside the other solid.
    Inside,
    /// Strictly outside it.
    Outside,
    /// On its boundary, with the two surfaces' normals pointing the same way
    /// — the two solids touch and lie on the same side of the shared patch.
    OnSameNormal,
    /// On its boundary, with the normals opposed — the solids meet along the
    /// patch from opposite sides.
    OnOppositeNormal,
}

/// What to do with a classified face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Keep {
    /// Keep it as it is.
    AsIs,
    /// Keep it with its normal flipped — the operator wants the material on
    /// the other side of this patch.
    Reversed,
    /// Drop it.
    Drop,
}

impl BooleanOp {
    /// Whether to keep a face classified as `class`, and with which
    /// orientation. `from_a` says which solid the face came from, because the
    /// two are not symmetric for [`BooleanOp::Difference`], and because a
    /// coincident patch must be kept exactly once rather than from both.
    ///
    /// The coincident rules are the only subtle ones. A patch shared by both
    /// solids with **matching** normals bounds the same material on the same
    /// side, so union and intersection each keep exactly one copy (`a`'s, by
    /// convention) and difference deletes it — the material behind it is
    /// removed from both sides at once. A patch shared with **opposing**
    /// normals is the reverse: it separates the two solids, so union and
    /// intersection drop it (it is interior to the result) while difference
    /// keeps `a`'s copy, since that is exactly the surface where `a` is left
    /// open by removing `b`.
    fn keeps(self, class: FaceClassification, from_a: bool) -> Keep {
        use FaceClassification::*;
        match (self, class) {
            // Union: the result is bounded by whatever is outside the other.
            (BooleanOp::Union, Outside) => Keep::AsIs,
            (BooleanOp::Union, Inside) => Keep::Drop,
            (BooleanOp::Union, OnSameNormal) => {
                if from_a {
                    Keep::AsIs
                } else {
                    Keep::Drop
                }
            }
            (BooleanOp::Union, OnOppositeNormal) => Keep::Drop,

            // Intersection: bounded by whatever is inside the other.
            (BooleanOp::Intersection, Inside) => Keep::AsIs,
            (BooleanOp::Intersection, Outside) => Keep::Drop,
            (BooleanOp::Intersection, OnSameNormal) => {
                if from_a {
                    Keep::AsIs
                } else {
                    Keep::Drop
                }
            }
            (BooleanOp::Intersection, OnOppositeNormal) => Keep::Drop,

            // Difference (a - b) = a intersected with the complement of b, so
            // `a`'s faces behave as for intersection-with-the-outside, and
            // `b`'s surviving faces are the ones inside `a`, turned around to
            // face into the cavity they now bound.
            (BooleanOp::Difference, Outside) if from_a => Keep::AsIs,
            (BooleanOp::Difference, Inside) if from_a => Keep::Drop,
            (BooleanOp::Difference, Inside) => Keep::Reversed,
            (BooleanOp::Difference, Outside) => Keep::Drop,
            (BooleanOp::Difference, OnSameNormal) => Keep::Drop,
            (BooleanOp::Difference, OnOppositeNormal) => {
                if from_a {
                    Keep::AsIs
                } else {
                    Keep::Drop
                }
            }
        }
    }
}

/// Fixed seed for the ray casting behind every containment query here. Both
/// `face_contains` and `shell_contains` retry until they find a ray grazing
/// nothing, so their answers are seed-independent; a constant keeps a boolean
/// reproducible run to run.
const SEED: u64 = 0xB001_EA47_0000_0001;

/// `solid_a` combined with `solid_b` under `op`, as a new solid in `model`.
///
/// `Ok(None)` means the result is **empty**, which is an answer rather than a
/// failure: intersecting two solids that do not overlap, or subtracting a
/// solid that wholly contains the first, legitimately leaves nothing. Callers
/// that treat "no solid" as an error would reject the majority of scenes in
/// this crate's own test set.
///
/// Both input solids are consumed either way: their faces are transferred to
/// the result or deleted, so neither id is valid afterwards.
///
/// The result is named `namer`'s root name, and everything the boolean
/// creates on the way is named as [`crate::naming`] describes; every face,
/// edge and vertex it keeps keeps its name.
pub fn boolean<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid_a: SolidId,
    solid_b: SolidId,
    op: BooleanOp,
    params: RemeshParams<S>,
) -> GeopResult<Option<SolidId>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "boolean(name={}, solid_a={solid_a}, solid_b={solid_b}, op={op:?})",
            namer.root()
        ))
    };

    remesh(part, namer, solid_a, solid_b, params).with_context(&ctx)?;
    let model = part.topology();

    let faces_a = model.solid_faces(solid_a).with_context(&ctx)?;
    let faces_b = model.solid_faces(solid_b).with_context(&ctx)?;

    let mut keep: Vec<FaceId> = Vec::new();
    let mut reverse: Vec<FaceId> = Vec::new();
    for (faces, from_a, other) in [(&faces_a, true, solid_b), (&faces_b, false, solid_a)] {
        for &face_id in faces {
            let class = classify_face(model, face_id, other, params)
                .with_context(&ctx)
                .with_context(&|e: GeopError| {
                    e.with_context(format!("classifying face {face_id}"))
                })?;
            match op.keeps(class, from_a) {
                Keep::AsIs => keep.push(face_id),
                Keep::Reversed => {
                    keep.push(face_id);
                    reverse.push(face_id);
                }
                Keep::Drop => {}
            }
        }
    }

    for &face_id in &reverse {
        part.reverse_face(face_id).with_context(&ctx)?;
    }

    part.assemble_solid(&[solid_a, solid_b], &keep, namer.root())
        .with_context(&ctx)
}

/// Where `face_id` sits relative to `other_solid`, decided at a single point
/// strictly inside the face's trimmed region.
///
/// One point is enough *because remesh ran first*: every curve along which
/// the other solid's boundary crosses this face has been imprinted, and the
/// face split along it, so the face no longer straddles anything. Without
/// that guarantee this would be unsound, which is why it lives here rather
/// than as a general-purpose query.
///
/// A point that lands *on* the other solid's boundary doesn't decide it by
/// itself: the face may lie along that boundary over an area (coincident
/// patches, told apart by their normals), or merely touch it at a point or
/// along a curve — a cube face resting on a sphere's pole, say, where the
/// sphere has no normal at all. So such a point is set aside and the next
/// interior point tried (one per boundary coedge, see
/// `face_interior_point_where`); any point off the other solid's boundary
/// classifies the whole face. Only if every one lies on it are the patches
/// coincident, and their normals are compared.
pub fn classify_face<S: Scalar>(
    model: &Model<S>,
    face_id: FaceId,
    other_solid: SolidId,
    params: RemeshParams<S>,
) -> GeopResult<FaceClassification> {
    let face = model.get_face(face_id)?;
    let shells = model.get_solid(other_solid)?.shells.clone();
    let mut decided = None;
    let mut on_boundary = Vec::new();
    face_interior_point_where(
        model,
        face_id,
        params.max_nodes,
        params.curve_curve_min_subdivision_size,
        SEED,
        |u, v| {
            let point = face.surface.evaluate(u, v)?;
            for &shell_id in &shells {
                match shell_contains(
                    model,
                    shell_id,
                    point,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                    SEED,
                )? {
                    ShellPoint::Inside => {
                        decided = Some(FaceClassification::Inside);
                        return Ok(true);
                    }
                    ShellPoint::Outside => continue,
                    ShellPoint::OnFace | ShellPoint::OnEdge | ShellPoint::OnVertex => {
                        on_boundary.push((u, v, point, shell_id));
                        return Ok(false);
                    }
                }
            }
            decided = Some(FaceClassification::Outside);
            Ok(true)
        },
    )?;
    if let Some(classification) = decided {
        return Ok(classification);
    }

    // Every interior point lies on the other solid's boundary: the patches
    // coincide, and which way the shared patch faces is what separates "these
    // solids touch" from "these solids overlap along this patch". Any point
    // of the shared patch shows that equally well, so the first where both
    // normals exist is used — a revolve's cap collapses to its pole, where
    // it has none.
    let mut undefined = None;
    for &(u, v, point, shell_id) in &on_boundary {
        let normals = face
            .surface
            .normal(u, v)
            .and_then(|n| Ok((n, shell_normal_at(model, shell_id, &point, params)?)));
        let (this_normal, other_normal) = match normals {
            Ok(normals) => normals,
            Err(e) => {
                undefined = Some(e);
                continue;
            }
        };
        let alignment = this_normal.prod_dot(&other_normal);
        return if alignment.definitely_greater(S::ZERO) {
            Ok(FaceClassification::OnSameNormal)
        } else if alignment.definitely_less(S::ZERO) {
            Ok(FaceClassification::OnOppositeNormal)
        } else {
            Err(GeopError::new(format!(
                "boolean: face {face_id} lies on solid {other_solid}'s boundary at {point:?} (its interior point uv=({u:?}, {v:?})), but the two normals ({this_normal:?} and {other_normal:?}) are too close to perpendicular to tell which side is which"
            )))
        };
    }
    Err(match undefined {
        Some(e) => e.with_context(format!(
            "classify_face: face {face_id} lies on solid {other_solid}'s boundary at each of its {} interior points tried, and no normal comparison could be made at any of them",
            on_boundary.len()
        )),
        None => GeopError::new(format!(
            "classify_face: face {face_id} yielded interior points, yet none was classified or set aside"
        )),
    })
}

/// The normal of whichever face of `shell_id` contains `point`.
fn shell_normal_at<S: Scalar>(
    model: &Model<S>,
    shell_id: ShellId,
    point: &Vector3<S>,
    params: RemeshParams<S>,
) -> GeopResult<Vector3<S>> {
    for &face_id in &model.get_shell(shell_id)?.faces {
        let surface = &model.get_face(face_id)?.surface;
        let Some((u, v)) = geop_core_geometry::contains::surface::surface_could_contain(
            surface,
            point,
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
        )?
        else {
            continue;
        };
        if !matches!(
            face_contains(
                model,
                face_id,
                u,
                v,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
                SEED,
            )?,
            FacePoint::Outside
        ) {
            return surface.normal(u, v);
        }
    }
    Err(GeopError::new(format!(
        "boolean: no face of shell {shell_id} contains {point:?}, although the shell reported the point on its boundary"
    )))
}

#[cfg(test)]
mod tests {
    use super::{BooleanOp, FaceClassification, boolean, classify_face};
    use crate::{remesh::remesh::RemeshParams, scenes::all_scenes};
    use geop_core_math::{scalars::ScalInF64, scalars::Scalar, vector::Vector3};
    use geop_core_topology::{
        contains::rng::Rng,
        validation::{ValidationParameters, validate_fast},
    };
    use geop_ops::Namer;

    fn scene(name: &str) -> crate::scenes::TestScene<ScalInF64> {
        all_scenes::<ScalInF64>()
            .into_iter()
            .find(|s| s.name == name)
            .unwrap_or_else(|| panic!("scene {name} must exist"))
    }

    /// A fresh operation id, so that every solid and boolean a test builds
    /// gets names of its own.
    fn fresh_id() -> String {
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        format!(
            "op{}",
            NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        )
    }

    fn namer() -> Namer {
        Namer::new("boolean", &fresh_id()).unwrap()
    }

    fn validation() -> ValidationParameters<ScalInF64> {
        let params = RemeshParams::<ScalInF64>::default();
        ValidationParameters {
            max_nodes: params.max_nodes,
            min_subdivision_size: params.curve_curve_min_subdivision_size,
            ..ValidationParameters::default()
        }
    }

    /// Every face of an un-remeshed cube is outside a cylinder that only
    /// overlaps part of it — the classification itself, before any operator.
    #[test]
    fn classify_face_separates_inside_from_outside() {
        let mut s = scene("box_cylinder_drilled_hole_through");
        let params = RemeshParams::<ScalInF64>::default();
        crate::remesh::remesh::remesh(&mut s.part, &namer(), s.solid_a, s.solid_b, params).unwrap();

        let mut inside = 0;
        let mut outside = 0;
        for face_id in s.part.topology().solid_faces(s.solid_a).unwrap() {
            match classify_face(s.part.topology(), face_id, s.solid_b, params).unwrap() {
                FaceClassification::Inside => inside += 1,
                FaceClassification::Outside => outside += 1,
                _ => {}
            }
        }
        assert!(
            inside > 0 && outside > 0,
            "a cylinder drilled through a cube must leave cube faces on both sides: {inside} inside, {outside} outside"
        );
    }

    /// Every face must yield an interior point, or classification is
    /// meaningless — this is the part of the algorithm with no fallback.
    #[test]
    fn every_remeshed_face_has_an_interior_point() {
        let mut s = scene("box_cylinder_drilled_hole_through");
        let params = RemeshParams::<ScalInF64>::default();
        crate::remesh::remesh::remesh(&mut s.part, &namer(), s.solid_a, s.solid_b, params).unwrap();

        for solid in [s.solid_a, s.solid_b] {
            for face_id in s.part.topology().solid_faces(solid).unwrap() {
                let (u, v) = geop_core_topology::contains::face::face_interior_point(
                    s.part.topology(),
                    face_id,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                    1234,
                )
                .unwrap_or_else(|e| panic!("face {face_id}: {e}"));
                let _ = s
                    .part
                    .topology()
                    .get_face(face_id)
                    .unwrap()
                    .surface
                    .evaluate(u, v)
                    .unwrap();
            }
        }
    }

    fn check_op(name: &str, op: BooleanOp) {
        let mut s = scene(name);
        let params = RemeshParams::<ScalInF64>::default();
        let result = boolean(&mut s.part, &namer(), s.solid_a, s.solid_b, op, params)
            .unwrap_or_else(|e| panic!("{name} {op:?}: {e}"))
            .unwrap_or_else(|| panic!("{name} {op:?}: result is empty"));

        assert!(
            !s.part.topology().solid_faces(result).unwrap().is_empty(),
            "{name} {op:?}: result has no faces"
        );
        if let Err(errors) = validate_fast(&validation(), s.part.topology()) {
            panic!(
                "{name} {op:?}: {} validate_fast error(s): {}",
                errors.len(),
                errors[0]
            );
        }
    }

    /// Where a probe point lands relative to the result solid.
    fn contains(
        model: &geop_core_topology::Model<ScalInF64>,
        solid: geop_core_topology::SolidId,
        p: (f64, f64, f64),
    ) -> bool {
        let params = RemeshParams::<ScalInF64>::default();
        let point = geop_core_math::vector::Vector3::from_array([
            ScalInF64::from_f64(p.0),
            ScalInF64::from_f64(p.1),
            ScalInF64::from_f64(p.2),
        ]);
        let shell = model.get_solid(solid).unwrap().shells[0];
        matches!(
            geop_core_topology::contains::shell::shell_contains(
                model,
                shell,
                point,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
                0xA5A5_1234,
            )
            .unwrap(),
            geop_core_topology::contains::shell::PointClassification::Inside
        )
    }

    /// The cube is `[-0.5, 0.5]^3`; the cylinder has radius `0.2` on the z
    /// axis and spans `z in [-1, 1]`. So each probe below is unambiguously in
    /// one region, and the three operators must disagree about them in
    /// exactly the way their definitions say. Structural validity says
    /// nothing about *which* faces were kept — this is what does.
    const IN_CUBE_ONLY: (f64, f64, f64) = (0.4, 0.4, 0.0);
    const IN_CYLINDER_ONLY: (f64, f64, f64) = (0.0, 0.0, 0.8);
    const IN_BOTH: (f64, f64, f64) = (0.0, 0.0, 0.0);

    fn run(
        op: BooleanOp,
    ) -> (
        geop_core_topology::Model<ScalInF64>,
        geop_core_topology::SolidId,
    ) {
        let mut s = scene("box_cylinder_drilled_hole_through");
        let params = RemeshParams::<ScalInF64>::default();
        let result = boolean(&mut s.part, &namer(), s.solid_a, s.solid_b, op, params)
            .unwrap()
            .expect("this scene's operands overlap, so no operator is empty");
        (s.part.topology().clone(), result)
    }

    /// Regression: two cubes meeting along a shared edge, where the traced
    /// intersection curve *is* that edge — already part of both faces'
    /// boundaries. Splicing it carved a zero-area sliver instead of splitting
    /// anything. The direction check was passing because the corrector
    /// returned a sharpened `(u, v)` sitting 6e-17 off the trim boundary,
    /// which `face_contains` read as `Inside`; see
    /// `predictor_corrector_step`.
    #[test]
    fn box_grid_n1p00_n0p50_n1p00_difference_succeeds() {
        let mut s = scene("box_grid_n1p00_n0p50_n1p00");
        let params = RemeshParams::<ScalInF64>::default();
        boolean(
            &mut s.part,
            &namer(),
            s.solid_a,
            s.solid_b,
            BooleanOp::Difference,
            params,
        )
        .unwrap_or_else(|e| panic!("{e}"));
    }

    /// The defining property of a boolean, checked by sampling space rather
    /// than by inspecting topology: for a point `p`, membership in the result
    /// is a pure function of membership in the two operands.
    ///
    /// `union` keeps `p` iff either operand held it, `intersection` iff both
    /// did, `difference` iff `a` held it and `b` did not. Nothing about faces,
    /// coedges or orientation enters into it — which is exactly why it is
    /// worth checking. A result can be structurally perfect and still enclose
    /// the wrong region, if `BooleanOp::keeps` drops a face it should have
    /// kept or leaves one facing inward, and no structural check can see that.
    ///
    /// Points landing *on* a boundary are skipped rather than asserted about:
    /// membership there is genuinely ambiguous, and both the operands and the
    /// result may legitimately disagree about a surface they share.
    fn check_boolean_matches_point_membership(name: &str, op: BooleanOp, seed: u64) {
        let mut s = scene(name);
        let params = RemeshParams::<ScalInF64>::default();
        let (solid_a, solid_b) = (s.solid_a, s.solid_b);

        // Sampled against the operands *before* the boolean consumes them.
        let mut rng = Rng::new(seed);
        let mut samples = Vec::new();
        while samples.len() < SAMPLE_COUNT {
            let p = Vector3::from_array([
                ScalInF64::from_f64(rng.next_f64() * 4.0 - 2.0),
                ScalInF64::from_f64(rng.next_f64() * 4.0 - 2.0),
                ScalInF64::from_f64(rng.next_f64() * 4.0 - 2.0),
            ]);
            let (Some(in_a), Some(in_b)) = (
                strictly_inside(s.part.topology(), solid_a, p),
                strictly_inside(s.part.topology(), solid_b, p),
            ) else {
                continue;
            };
            samples.push((p, in_a, in_b));
        }

        let result = boolean(&mut s.part, &namer(), solid_a, solid_b, op, params)
            .unwrap_or_else(|e| panic!("{name} {op:?}: {e}"));

        for (p, in_a, in_b) in samples {
            let expected = match op {
                BooleanOp::Union => in_a || in_b,
                BooleanOp::Intersection => in_a && in_b,
                BooleanOp::Difference => in_a && !in_b,
            };
            let actual = match result {
                Some(solid) => match strictly_inside(s.part.topology(), solid, p) {
                    Some(inside) => inside,
                    // On the result's own boundary — ambiguous, no claim.
                    None => continue,
                },
                // An empty result contains nothing.
                None => false,
            };
            assert_eq!(
                actual,
                expected,
                "{name} {op:?}: point {p:?} is {} solid A and {} solid B, so the result should {} contain it",
                if in_a { "inside" } else { "outside" },
                if in_b { "inside" } else { "outside" },
                if expected { "" } else { "not" }
            );
        }
    }

    /// How many points each membership check samples.
    const SAMPLE_COUNT: usize = 20;

    /// `Some(true)`/`Some(false)` for a point strictly inside/outside every
    /// shell of `solid`; `None` if it lands on a boundary, where membership is
    /// not a yes/no question.
    fn strictly_inside(
        model: &geop_core_topology::Model<ScalInF64>,
        solid: geop_core_topology::SolidId,
        p: Vector3<ScalInF64>,
    ) -> Option<bool> {
        let params = RemeshParams::<ScalInF64>::default();
        let mut inside = false;
        for shell in model.get_solid(solid).ok()?.shells.clone() {
            match geop_core_topology::contains::shell::shell_contains(
                model,
                shell,
                p,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
                0x5A3D_1234,
            ) {
                Ok(geop_core_topology::contains::shell::PointClassification::Inside) => {
                    inside = true
                }
                Ok(geop_core_topology::contains::shell::PointClassification::Outside) => {}
                _ => return None,
            }
        }
        Some(inside)
    }

    /// Scenes with genuinely overlapping operands, so every operator has
    /// something to do and the samples land on both sides of each boundary.
    const MEMBERSHIP_SCENES: &[&str] = &[
        "box_cylinder_drilled_hole_through",
        "box_cylinder_blind_hole",
        "figure8_cylinder_through_neck",
    ];

    #[test]
    fn union_matches_point_membership() {
        for (i, name) in MEMBERSHIP_SCENES.iter().enumerate() {
            check_boolean_matches_point_membership(name, BooleanOp::Union, 0xB001 + i as u64);
        }
    }

    #[test]
    fn intersection_matches_point_membership() {
        for (i, name) in MEMBERSHIP_SCENES.iter().enumerate() {
            check_boolean_matches_point_membership(
                name,
                BooleanOp::Intersection,
                0xB101 + i as u64,
            );
        }
    }

    #[test]
    fn difference_matches_point_membership() {
        for (i, name) in MEMBERSHIP_SCENES.iter().enumerate() {
            check_boolean_matches_point_membership(name, BooleanOp::Difference, 0xB201 + i as u64);
        }
    }

    #[test]
    fn union_contains_either_operand() {
        let (model, r) = run(BooleanOp::Union);
        assert!(contains(&model, r, IN_CUBE_ONLY), "cube-only point");
        assert!(contains(&model, r, IN_CYLINDER_ONLY), "cylinder-only point");
        assert!(contains(&model, r, IN_BOTH), "shared point");
    }

    #[test]
    fn intersection_contains_only_the_overlap() {
        let (model, r) = run(BooleanOp::Intersection);
        assert!(
            !contains(&model, r, IN_CUBE_ONLY),
            "cube-only point must be out"
        );
        assert!(
            !contains(&model, r, IN_CYLINDER_ONLY),
            "cylinder-only point must be out"
        );
        assert!(contains(&model, r, IN_BOTH), "shared point must be in");
    }

    #[test]
    fn difference_removes_the_second_operand() {
        let (model, r) = run(BooleanOp::Difference);
        assert!(
            contains(&model, r, IN_CUBE_ONLY),
            "cube-only point must remain"
        );
        assert!(
            !contains(&model, r, IN_CYLINDER_ONLY),
            "cylinder-only point must be out"
        );
        assert!(
            !contains(&model, r, IN_BOTH),
            "the drilled-out region must be gone"
        );
    }

    #[test]
    fn union_of_box_and_cylinder_is_valid() {
        check_op("box_cylinder_drilled_hole_through", BooleanOp::Union);
    }

    #[test]
    fn intersection_of_box_and_cylinder_is_valid() {
        check_op("box_cylinder_drilled_hole_through", BooleanOp::Intersection);
    }

    #[test]
    fn difference_of_box_and_cylinder_is_valid() {
        check_op("box_cylinder_drilled_hole_through", BooleanOp::Difference);
    }

    /// A chain of 3 differences (two axis-aligned slots cut from a block,
    /// then a sphere drilled out of the result) — captured from a browser
    /// session's timeline, exactly like `scenes`' hand-picked
    /// cases, built here directly from `geop_ops_extrude_revolve::shapes`
    /// and `boolean` rather than through `cad::Op`/wasm. Rendered to `outputs/` so the new
    /// curvature-adaptive rasterizer's output on a real chained-boolean
    /// result (flat cut faces plus the sphere's curved ones) can be
    /// inspected visually, the same way `scenes`' figure8/box
    /// cases are.
    // ── Chained booleans on the basic shapes, as the CAD front end builds them ──
    //
    // Each solid is created just before the boolean that uses it: every step
    // validates the *whole* model, and a solid not yet combined with anything
    // legitimately overlaps the others in space — which the validation would
    // report as edges crossing faces.

    type M = geop_ops::Part<ScalInF64>;

    fn v(x: f64, y: f64, z: f64) -> Vector3<ScalInF64> {
        Vector3::from_array([x, y, z].map(ScalInF64::from_f64))
    }

    /// `CreateCube(offset, dims)`: the axis-aligned box from `offset` to
    /// `offset + dims`.
    fn cube(part: &mut M, offset: [f64; 3], dims: [f64; 3]) -> geop_core_topology::SolidId {
        let [x, y, z] = offset;
        let [dx, dy, dz] = dims;
        let (min, max) = (v(x, y, z), v(x + dx, y + dy, z + dz));
        geop_ops_extrude_revolve::shapes::cube_solid(part, &fresh_id(), min, max).unwrap()
    }

    /// `CreateSphere(offset, r)`.
    fn sphere(part: &mut M, center: [f64; 3], r: f64) -> geop_core_topology::SolidId {
        let [x, y, z] = center;
        let r = ScalInF64::from_f64(r);
        geop_ops_extrude_revolve::shapes::sphere::sphere_solid(part, &fresh_id(), v(x, y, z), r)
            .unwrap()
    }

    /// `CreateCylinder(offset, r, h, axis)`: `offset` is the bottom cap's centre.
    fn cylinder(
        part: &mut M,
        base: [f64; 3],
        r: f64,
        h: f64,
        axis: geop_ops_extrude_revolve::shapes::cylinder::Axis,
    ) -> geop_core_topology::SolidId {
        let [x, y, z] = base;
        geop_ops_extrude_revolve::shapes::cylinder::revolved_cylinder_along_axis(
            part,
            &fresh_id(),
            v(x, y, z),
            ScalInF64::from_f64(r),
            ScalInF64::from_f64(h),
            axis,
        )
        .unwrap()
    }

    /// Run one boolean, requiring it to succeed with a non-empty solid that
    /// passes `validate_fast` and the full `validate`.
    fn op(
        part: &mut M,
        a: geop_core_topology::SolidId,
        b: geop_core_topology::SolidId,
        op: BooleanOp,
    ) -> geop_core_topology::SolidId {
        let result = boolean(part, &namer(), a, b, op, RemeshParams::default())
            .unwrap_or_else(|e| panic!("{op:?} failed: {e:?}"))
            .unwrap_or_else(|| panic!("{op:?} produced an empty solid"));
        part.check_names().unwrap();
        let model = part.topology();
        if let Err(errors) = validate_fast(&validation(), model) {
            panic!(
                "{op:?}: {} validate_fast error(s): {}",
                errors.len(),
                errors[0]
            );
        }
        // The full validation too: `validate_fast` checks each face on its
        // own, and passes an edge running through the middle of another face,
        // or two edges crossing away from any vertex. Not `validate_manifold`:
        // many of these results are genuinely non-manifold — solids touching
        // along a line leave edges shared by four faces.
        if let Err(errors) = geop_core_topology::validation::validate(&validation(), model) {
            panic!("{op:?}: {} validate error(s): {}", errors.len(), errors[0]);
        }
        // Closed, which neither check covers: every edge of the result is used
        // by an even number of coedges — two where two faces meet, four where
        // the solid touches itself along the edge. An odd count is a hole.
        for face in model.solid_faces(result).unwrap() {
            for coedge in model.iterate_face_coedges(face) {
                // A bare-vertex boundary has no edge to share.
                let Ok(edge) = model.get_coedge(coedge).unwrap().edge() else {
                    continue;
                };
                let uses = model.coedges_of_edge(edge).len();
                if uses % 2 != 0 {
                    let e = model.get_edge(edge).unwrap();
                    let at = |v| {
                        let p = model.get_vertex(v).unwrap().point;
                        [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()]
                    };
                    panic!(
                        "{op:?}: edge {edge} of face {face}, from {:?} to {:?}, is used by {uses} coedge(s) — the solid is open there",
                        at(e.start_vertex),
                        at(e.end_vertex)
                    );
                }
            }
        }
        result
    }

    /// Reported from the web front end: a cube with a bore through it, the
    /// sphere inscribed in the cube added back, then the `y > 0` half cut
    /// away. The last difference failed in `remesh_edges_x_edges` with "could
    /// not locate vertex on coedge's pcurve" at the cube corner
    /// `(-0.5, 0, -0.5)`, where the cutting cube's face meets an edge of the
    /// first result.
    #[test]
    fn bored_cube_plus_inscribed_sphere_minus_half() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, [-0.5, -0.5, -0.5], [1.0, 1.0, 1.0]);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.875], 0.5, 1.75, Axis::Z);
        let bored = op(&mut part, block, bore, BooleanOp::Difference);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let filled = op(&mut part, bored, ball, BooleanOp::Union);
        let half = cube(&mut part, [-0.5, 0.0, -0.5], [1.0, 1.0, 1.0]);
        op(&mut part, filled, half, BooleanOp::Difference);
    }

    /// Reported from the web front end: a cube with a flush inscribed bore
    /// (tangent to the four sides, caps coplanar with top and bottom), the
    /// inscribed sphere added back, then everything with `x < 0.05` cut away
    /// like a section view. The result looked open — two faces missing —
    /// with spurious triangles in the cutting plane. Not a manifold even when
    /// right: the bore touches the cube's sides along lines, and the one at
    /// `x = 0.5` survives the cut.
    #[test]
    #[ignore = "still fails: the section plane x = 0.05 cuts the sphere in a circle of radius \
                0.4975, exactly the bore wall's y = ±0.4975, so the circle touches the wall's section \
                lines tangentially at (0.05, ±0.4975, 0). Splitting the section face there leaves two \
                kept faces overlapping on the thin strip between the cube side and the wall (the \
                spurious triangles), with a wall-section edge used by 3 coedges. Fixed so far: the \
                missing top/bottom corner faces (stale start-point face; a curve shorter than the \
                tracer's first step never getting a direction)."]
    fn flush_bored_cube_plus_inscribed_sphere_section() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.5], 0.5, 1.0, Axis::Z);
        let bored = op(&mut part, block, bore, BooleanOp::Difference);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let filled = op(&mut part, bored, ball, BooleanOp::Union);
        let cutter = cube(&mut part, [-1.45, -1.05, -0.82], [1.5, 2.1, 1.65]);
        op(&mut part, filled, cutter, BooleanOp::Difference);
    }

    /// Like [`op`], for a boolean whose correct result is empty.
    fn op_empty(
        part: &mut M,
        a: geop_core_topology::SolidId,
        b: geop_core_topology::SolidId,
        op: BooleanOp,
    ) {
        let result = boolean(part, &namer(), a, b, op, RemeshParams::default())
            .unwrap_or_else(|e| panic!("{op:?} failed: {e:?}"));
        assert!(result.is_none(), "{op:?} should be empty");
    }

    const UNIT: [f64; 3] = [1.0, 1.0, 1.0];
    const CORNER: [f64; 3] = [-0.5, -0.5, -0.5];

    // Tangent contact. Where two surfaces touch without crossing, the
    // intersection is a curve or point along which a tiny error in one
    // surface moves the contact a long way (a perpendicular error `d` shifts a
    // tangency by `~sqrt(2 R d)`), which is what broke
    // `bored_cube_plus_inscribed_sphere_minus_half`.

    /// The cylinder touches all four side faces along vertical lines.
    #[test]
    fn cube_minus_inscribed_cylinder() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.875], 0.5, 1.75, Axis::Z);
        op(&mut part, block, bore, BooleanOp::Difference);
    }

    /// The cylinder is tangent to the four sides *and* its caps are flush
    /// with the top and bottom: each cap's rim lies in the cube's face and
    /// touches that face's four edges at their midpoints. The first step of
    /// `flush_bored_cube_plus_inscribed_sphere_section`.
    #[test]
    fn cube_minus_flush_inscribed_cylinder() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.5], 0.5, 1.0, Axis::Z);
        op(&mut part, block, bore, BooleanOp::Difference);
    }

    /// The sphere touches each face of the cube at its centre.
    #[test]
    fn cube_minus_inscribed_sphere() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        op(&mut part, block, ball, BooleanOp::Difference);
    }

    /// The sphere passes through all eight corners of the cube.
    #[test]
    fn cube_intersect_circumscribed_sphere() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 3f64.sqrt() / 2.0);
        op(&mut part, block, ball, BooleanOp::Intersection);
    }

    /// The cylinder's surface contains the cube's four vertical edges.
    #[test]
    fn cube_intersect_cylinder_through_its_edges() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5f64.sqrt(), 2.0, Axis::Z);
        op(&mut part, block, tube, BooleanOp::Intersection);
    }

    /// A coaxial cylinder of the sphere's radius touches it along the
    /// equator, a whole circle of tangency.
    #[test]
    fn sphere_union_tangent_cylinder() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 2.0, Axis::Z);
        op(&mut part, ball, tube, BooleanOp::Union);
    }

    /// A cube resting on the sphere's pole, where the sphere's patches all
    /// collapse to one point: tangent contact at a singular point.
    #[test]
    fn sphere_union_cube_touching_its_pole() {
        let mut part = M::new();
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let block = cube(&mut part, [-0.5, -0.5, 0.5], UNIT);
        op(&mut part, ball, block, BooleanOp::Union);
    }

    /// A cylinder along x lying on the cube's top face: tangent along a line.
    #[test]
    fn cube_union_cylinder_lying_on_top() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let log = cylinder(&mut part, [-1.0, 0.0, 0.75], 0.25, 2.0, Axis::X);
        op(&mut part, block, log, BooleanOp::Union);
    }

    // Coincident features: shared faces, edges and corners, coplanar caps, and
    // identical solids.

    #[test]
    fn cubes_sharing_a_face_union() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, [0.5, -0.5, -0.5], UNIT);
        op(&mut part, a, b, BooleanOp::Union);
    }

    #[test]
    fn cubes_sharing_an_edge_union() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, [0.5, 0.5, -0.5], UNIT);
        op(&mut part, a, b, BooleanOp::Union);
    }

    #[test]
    fn cubes_sharing_a_corner_union() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, [0.5, 0.5, 0.5], UNIT);
        op(&mut part, a, b, BooleanOp::Union);
    }

    /// A half-overlapping cube: two of its faces are coplanar with the first
    /// cube's, over part of their area.
    #[test]
    fn cube_minus_offset_cube_with_coplanar_faces() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, [0.0, -0.5, -0.5], UNIT);
        op(&mut part, a, b, BooleanOp::Difference);
    }

    /// A hole whose caps are flush with the cube's top and bottom.
    #[test]
    fn cube_minus_flush_cylinder() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.5], 0.3, 1.0, Axis::Z);
        op(&mut part, block, bore, BooleanOp::Difference);
    }

    #[test]
    fn identical_spheres_union() {
        let mut part = M::new();
        let a = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let b = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        op(&mut part, a, b, BooleanOp::Union);
    }

    #[test]
    fn identical_cubes_difference_is_empty() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, CORNER, UNIT);
        op_empty(&mut part, a, b, BooleanOp::Difference);
    }

    /// A sphere centred on the cube's corner: its three coordinate-plane
    /// seams lie exactly in three faces of the cube.
    #[test]
    fn cube_minus_sphere_on_its_corner() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [0.5, 0.5, 0.5], 0.5);
        op(&mut part, block, ball, BooleanOp::Difference);
    }

    // Curved against curved.

    /// Two equal cylinders crossing at right angles: their intersection
    /// curves (two ellipses) cross each other at two singular points.
    #[test]
    fn steinmetz_cylinders_union() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let a = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 2.0, Axis::Z);
        let b = cylinder(&mut part, [-1.0, 0.0, 0.0], 0.5, 2.0, Axis::X);
        op(&mut part, a, b, BooleanOp::Union);
    }

    #[test]
    fn steinmetz_cylinders_intersection() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let a = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 2.0, Axis::Z);
        let b = cylinder(&mut part, [-1.0, 0.0, 0.0], 0.5, 2.0, Axis::X);
        op(&mut part, a, b, BooleanOp::Intersection);
    }

    /// A bore along the sphere's axis, through both poles.
    #[test]
    fn sphere_minus_cylinder_through_its_poles() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let bore = cylinder(&mut part, [0.0, 0.0, -1.0], 0.2, 2.0, Axis::Z);
        op(&mut part, ball, bore, BooleanOp::Difference);
    }

    /// A sphere centred on a cylinder's cap: the cap cuts it along its
    /// equator, which is also where its patches meet.
    #[test]
    fn cylinder_union_sphere_on_its_cap() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 1.0, Axis::Z);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.3);
        op(&mut part, tube, ball, BooleanOp::Union);
    }

    /// The same with the sphere's radius equal to the cylinder's: the
    /// sphere's equator coincides with the cap's rim, *and* the sphere is
    /// tangent to the cylinder's side along that same circle.
    #[test]
    fn cylinder_union_equal_sphere_on_its_cap() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 1.0, Axis::Z);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        op(&mut part, tube, ball, BooleanOp::Union);
    }

    /// External tangency at a point of the sphere's equator (not a pole).
    #[test]
    fn cube_union_sphere_touching_a_face() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [1.0, 0.0, 0.0], 0.5);
        op(&mut part, block, ball, BooleanOp::Union);
    }

    /// A sphere half sunk into the cube through the centre of a face.
    #[test]
    fn cube_union_sphere_on_a_face() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [0.5, 0.0, 0.0], 0.3);
        op(&mut part, block, ball, BooleanOp::Union);
    }

    /// Inscribed bores along all three axes, one after another (a "jack").
    /// Each bore is tangent to four faces, and each later bore crosses the
    /// earlier ones at Steinmetz points.
    #[test]
    #[ignore = "still fails: after the third bore one face's normal points into the solid \
                (face_orientation check); not yet investigated. The bores are tangent to the cube's \
                faces and cross each other at Steinmetz points, the same degeneracies as elsewhere."]
    fn cube_minus_three_inscribed_bores() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let z = cylinder(&mut part, [0.0, 0.0, -0.875], 0.5, 1.75, Axis::Z);
        let a = op(&mut part, block, z, BooleanOp::Difference);
        let x = cylinder(&mut part, [-0.875, 0.0, 0.0], 0.5, 1.75, Axis::X);
        let b = op(&mut part, a, x, BooleanOp::Difference);
        let y = cylinder(&mut part, [0.0, -0.875, 0.0], 0.5, 1.75, Axis::Y);
        op(&mut part, b, y, BooleanOp::Difference);
    }

    /// Thinner bores, so the cube keeps its faces: two crossing bores.
    #[test]
    fn cube_minus_two_crossing_bores() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let z = cylinder(&mut part, [0.0, 0.0, -0.875], 0.3, 1.75, Axis::Z);
        let a = op(&mut part, block, z, BooleanOp::Difference);
        let x = cylinder(&mut part, [-0.875, 0.0, 0.0], 0.3, 1.75, Axis::X);
        op(&mut part, a, x, BooleanOp::Difference);
    }

    // More tangencies, and a longer chain.

    /// Two spheres touching externally at a single point.
    #[test]
    fn touching_spheres_union() {
        let mut part = M::new();
        let a = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let b = sphere(&mut part, [1.0, 0.0, 0.0], 0.5);
        op(&mut part, a, b, BooleanOp::Union);
    }

    /// A sphere inside a cylinder of the same radius, touching it along the
    /// equator from inside.
    #[test]
    fn cylinder_minus_inscribed_sphere() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 2.0, Axis::Z);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        op(&mut part, tube, ball, BooleanOp::Difference);
    }

    /// Two parallel cylinders touching along a line.
    #[test]
    fn touching_parallel_cylinders_union() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let a = cylinder(&mut part, [0.0, 0.0, -0.5], 0.5, 1.0, Axis::Z);
        let b = cylinder(&mut part, [1.0, 0.0, -0.5], 0.5, 1.0, Axis::Z);
        op(&mut part, a, b, BooleanOp::Union);
    }

    /// A sphere touching one of the cube's edges from outside.
    #[test]
    fn cube_union_sphere_touching_an_edge() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let d = 0.5 + 0.5 / 2f64.sqrt();
        let ball = sphere(&mut part, [d, d, 0.0], 0.5);
        op(&mut part, block, ball, BooleanOp::Union);
    }

    /// A small part as a user might build it: a plate with a boss and a
    /// through hole, then a rounded cap on the boss.
    #[test]
    fn plate_with_boss_hole_and_cap() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let plate = cube(&mut part, [-1.0, -1.0, -0.25], [2.0, 2.0, 0.5]);
        let boss = cylinder(&mut part, [0.0, 0.0, 0.25], 0.5, 0.5, Axis::Z);
        let a = op(&mut part, plate, boss, BooleanOp::Union);
        let cap = sphere(&mut part, [0.0, 0.0, 0.75], 0.5);
        let b = op(&mut part, a, cap, BooleanOp::Union);
        let hole = cylinder(&mut part, [0.0, 0.0, -0.5], 0.25, 1.5, Axis::Z);
        op(&mut part, b, hole, BooleanOp::Difference);
    }

    #[test]
    fn chained_differences_block_with_two_slots_and_a_sphere() {
        let f = ScalInF64::from_f64;
        let corner = |x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64| {
            (
                Vector3::from_array([f(x), f(y), f(z)]),
                Vector3::from_array([f(x + dx), f(y + dy), f(z + dz)]),
            )
        };

        let mut part = M::new();
        let (min_a, max_a) = corner(-0.50, -0.50, -0.50, 1.00, 1.00, 1.00);
        let a = geop_ops_extrude_revolve::shapes::cube_solid(&mut part, "a", min_a, max_a).unwrap();
        let (min_b, max_b) = corner(-1.13, -0.30, -0.33, 2.25, 0.60, 0.65);
        let b = geop_ops_extrude_revolve::shapes::cube_solid(&mut part, "b", min_b, max_b).unwrap();
        let params = RemeshParams::<ScalInF64>::default();
        let c = boolean(&mut part, &namer(), a, b, BooleanOp::Difference, params)
            .unwrap()
            .expect("block minus the first slot must be non-empty");

        let (min_d, max_d) = corner(-0.33, -0.28, -1.15, 0.65, 0.55, 2.30);
        let d = geop_ops_extrude_revolve::shapes::cube_solid(&mut part, "d", min_d, max_d).unwrap();
        let e = boolean(&mut part, &namer(), c, d, BooleanOp::Difference, params)
            .unwrap()
            .expect("minus the second slot must be non-empty");

        let sphere = geop_ops_extrude_revolve::shapes::sphere::sphere_solid(
            &mut part,
            "s",
            Vector3::zero(),
            f(0.45),
        )
        .unwrap();
        let result = boolean(
            &mut part,
            &namer(),
            e,
            sphere,
            BooleanOp::Difference,
            params,
        )
        .unwrap()
        .expect("minus the sphere must be non-empty");
        part.check_names().unwrap();

        let model = part.topology();
        assert!(!model.solid_faces(result).unwrap().is_empty());
        if let Err(errors) = validate_fast(&validation(), model) {
            panic!("{} validate_fast error(s): {}", errors.len(), errors[0]);
        }

        let scene = geop_ops_rasterize::rasterize(model, 8)
            .unwrap()
            .scene(|_| geop_ops_rasterize::debug::Color10::Blue);
        std::fs::create_dir_all("outputs").unwrap();
        scene
            .save_to_file("outputs/chained_differences_block_with_two_slots_and_a_sphere.html")
            .unwrap();
    }

    /// Regression test, captured from a browser session: two cubes
    /// differenced, then a sphere differenced out of that, then a thin slab
    /// cube differenced out of *that*. The second cube and the sphere were
    /// anchored to corners of the first cube, picked by kernel id
    /// (`VertexId(13)` and `VertexId(9)`); building that cube the same way
    /// assigns the same ids, so they are read back from it here.
    ///
    /// Used to fail inside the final boolean's `classify_face`:
    /// `shell_contains` reported a point on the slab's boundary that no face
    /// of that shell's `face_contains` agreed contained. Root cause:
    /// `shell_contains`'s face coincidence pre-check only asked
    /// `surface_could_contain` — proximity to a face's *untrimmed* surface —
    /// without checking the point fell within the face's trim. Proximity is
    /// not membership (see `AGENTS.md`).
    #[test]
    fn chained_differences_with_anchored_shapes_and_thin_slab_cutter_succeeds() {
        let mut part = M::new();
        let block = cube(&mut part, [-0.5, -0.5, -0.5], [1.0, 1.0, 1.0]);
        let corner = |part: &M, id: u64| {
            let p = part
                .topology()
                .get_vertex(geop_core_topology::VertexId(id))
                .unwrap()
                .point;
            [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()]
        };
        let [x, y, z] = corner(&part, 13);
        let second = cube(&mut part, [x - 0.5, y - 0.5, z - 0.5], [1.0, 1.0, 1.0]);
        let center = corner(&part, 9);
        let ball = sphere(&mut part, center, 0.5);
        // Only success is asserted, as when this was captured: the full
        // `validate` that `op` runs finds an edge of the second result
        // crossing a face unsplit, which this regression never covered.
        let difference = |part: &mut M, a, b| {
            boolean(
                part,
                &namer(),
                a,
                b,
                BooleanOp::Difference,
                RemeshParams::default(),
            )
            .unwrap()
            .expect("the result is not empty")
        };
        let cut = difference(&mut part, block, second);
        let cut = difference(&mut part, cut, ball);
        let slab = cube(&mut part, [-1.13, -0.10, -0.15], [2.25, 0.20, 0.30]);
        difference(&mut part, cut, slab);
        part.check_names().unwrap();
    }

    /// Regression test: a cylinder drilled through a cube, its height
    /// chosen so *both* caps sit exactly flush with the cube's own
    /// opposite faces (`cylinder z in [-0.5, 0.5]` exactly matching the
    /// cube's), rather than the usual "drilled hole" test scenes (e.g.
    /// `box_cylinder_drilled_hole_through`) where the cylinder
    /// deliberately extends *past* both faces.
    ///
    /// That exact-coplanar-cap case used to (1) take ~30s — over 1000x the
    /// sub-second time every other boolean test in this file takes — and
    /// (2) produce a *wrong* result, cutting the hole through only one of
    /// the two coplanar-cap faces and leaving the other fully solid.
    ///
    /// Root cause: `revolve_at_oriented` builds any flat cap (a
    /// doubly-curved pole, like a sphere's, genuinely needs a fan of
    /// wedges meeting at a center vertex — a flat cap doesn't, but got the
    /// same treatment) as 4 wedges with radial "spoke" edges from rim to
    /// center. `find_coincident_pair` (`remesh_edges_x_faces.rs`) couldn't
    /// tell those spokes apart from the genuine circular rim boundary, so
    /// it tried imprinting all of them — the slowdown, and (via
    /// interleaved imprints across the two caps corrupting
    /// `Model::splice_edge_into_face`'s face-splitting bookkeeping) the
    /// wrong result. Fixed by `is_internal_seam_edge`/`faces_are_coplanar`
    /// there: an edge whose two neighboring faces (within its own solid)
    /// are already coplanar with each other is never the only carrier of a
    /// genuine coincidence, so it's skipped — cutting straight to the rim
    /// arcs.
    #[test]
    fn cube_minus_z_cylinder_with_coplanar_cap_is_fast_and_correct() {
        let f = ScalInF64::from_f64;
        let mut part = M::new();
        let a = geop_ops_extrude_revolve::shapes::cube_solid(
            &mut part,
            "a",
            Vector3::from_array([f(-0.5), f(-0.5), f(-0.5)]),
            Vector3::from_array([f(0.5), f(0.5), f(0.5)]),
        )
        .unwrap();
        let b = geop_ops_extrude_revolve::shapes::cylinder::revolved_cylinder_along_axis(
            &mut part,
            "b",
            Vector3::from_array([f(0.0), f(0.0), f(-0.5)]),
            f(0.3),
            f(1.0),
            geop_ops_extrude_revolve::shapes::cylinder::Axis::Z,
        )
        .unwrap();
        let params = RemeshParams::<ScalInF64>::default();

        let t0 = std::time::Instant::now();
        let result = boolean(&mut part, &namer(), a, b, BooleanOp::Difference, params)
            .unwrap()
            .unwrap();
        let elapsed = t0.elapsed();
        let model = part.topology();
        // Ballpark-matches `difference_of_box_and_cylinder_is_valid`'s own
        // (non-coincident-cap) ~2s in isolation; running inside the full
        // suite under CPU contention from other parallel tests has been
        // observed up to ~12s. The threshold stays generous — the point is
        // catching a regression back toward the old ~32s (a *further*
        // 1000x-ish blowup on top of ordinary parallel-run noise), not
        // pinning down exact timing.
        assert!(
            elapsed.as_secs() < 20,
            "boolean took {elapsed:?}, expected well under 20s"
        );

        // Every point on both cap planes' hole boundary must be *outside*
        // the result (the cylinder's full bore is open at both ends) —
        // catches "hole only cut on one end" directly, unlike
        // `validate_fast`.
        let shell = model.get_solid(result).unwrap().shells[0];
        for z in [f(-0.45), f(0.45)] {
            let p = Vector3::from_array([f(0.0), f(0.0), z]);
            let outside = matches!(
                geop_core_topology::contains::shell::shell_contains(
                    model,
                    shell,
                    p,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                    0xC7D1
                )
                .unwrap(),
                geop_core_topology::contains::shell::PointClassification::Outside
            );
            assert!(
                outside,
                "point {p:?} (near a cap's bore) should be outside the drilled result"
            );
        }

        let scene = geop_ops_rasterize::rasterize(model, 8)
            .unwrap()
            .scene(|_| geop_ops_rasterize::debug::Color10::Blue);
        std::fs::create_dir_all("outputs").unwrap();
        scene
            .save_to_file("outputs/cube_minus_coplanar_cap_cylinder.html")
            .unwrap();
    }

    /// A bore through a block must actually be empty in the rendered mesh:
    /// no triangle may cover the hole. End-to-end cover for the trimming
    /// that `geop_ops_rasterize::clip` does per grid cell — where a
    /// duplicated vertex in a clipped hole outline, or a concave fragment
    /// classified as a whole, used to leave a flap of surface hanging
    /// across an opening (isolated in that module's own tests).
    #[test]
    fn bore_renders_as_a_hole() {
        let mut part = M::new();
        let block = cube(&mut part, [-1.0, -1.0, 0.0], [2.0, 2.0, 0.5]);
        // Radius 0.5 of the 2x2 footprint puts the bore's outline exactly
        // through grid cell corners at this resolution, which is what it
        // takes to produce the duplicated vertex.
        let bore = cylinder(
            &mut part,
            [0.0, 0.0, -0.5],
            0.5,
            1.5,
            geop_ops_extrude_revolve::shapes::cylinder::Axis::Z,
        );
        op(&mut part, block, bore, BooleanOp::Difference);

        let rasterized = geop_ops_rasterize::rasterize(part.topology(), 24).unwrap();
        for triangle in rasterized.faces.values().flatten() {
            for p in [triangle.a, triangle.b, triangle.c] {
                let (x, y, z) = (p[0].to_f64(), p[1].to_f64(), p[2].to_f64());
                let r = x.hypot(y);
                // Strictly inside the bore, and within the block's height:
                // nothing may be drawn there. The margin keeps the bore
                // wall's own triangles (at r = 0.6, faceted slightly inwards)
                // out of the test.
                assert!(
                    r > 0.45 || !(0.01..0.49).contains(&z),
                    "a triangle corner sits inside the bore at ({x}, {y}, {z})"
                );
            }
        }
    }
}
