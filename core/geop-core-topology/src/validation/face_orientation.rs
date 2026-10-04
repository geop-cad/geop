use geop_core_math::{geop_error::GeopError, polygon::polygon_signed_area, scalars::Scalar};

use crate::{
    Model,
    boundary::BoundaryType,
    contains::{
        face::face_interior_point,
        shell::{PointClassification, solid_contains},
    },
    loop_sampling::sample_loop_to_polygon,
    validation::ValidationParameters,
};

/// Fixed seed for the ray casting behind both checks. `shell_contains` and
/// `face_interior_point` retry until they find a ray grazing nothing, so
/// their answers are seed-independent; a constant keeps validation
/// reproducible run to run.
const SEED: u64 = 0x0F1E_0D1E_0000_0001;

/// Samples per coedge when measuring a loop's signed area. Only the *sign*
/// is used, and a curved trim loop needs a few samples per coedge for that
/// sign to be right; this bounds effort, not correctness.
const LOOP_AREA_SAMPLES: usize = 8;

/// How many times [`check_normals_point_outward`] may halve its probe
/// distance. Bounds effort only: every halving is more local than the last,
/// so the smallest that resolves is the answer. Twelve takes the probe to
/// about a four-thousandth of the shell's own extent, well below any feature
/// it is asked about. Twelve takes the probe to about a four-thousandth of the
/// shell's own extent, well below any feature it is asked about.
const MAX_HALVINGS: usize = 12;

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
pub fn check_loop_winding<S: Scalar>(
    _params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
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
            let polygon = match sample_loop_to_polygon(model, anchor, LOOP_AREA_SAMPLES) {
                Ok(p) => p,
                Err(e) => {
                    errors.push(e.with_context(format!(
                        "face {face_id}'s {which} boundary, sampling it to measure its winding"
                    )));
                    continue;
                }
            };
            let area = polygon_signed_area(&polygon);
            let ok = if want_positive {
                area.definitely_greater(S::ZERO)
            } else {
                area.definitely_less(S::ZERO)
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
fn traverses_each_edge_twice<S: Scalar>(model: &Model<S>, anchor: crate::CoedgeId) -> bool {
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
/// Tested by probing just off the surface at an interior point: a step
/// *against* the normal must land inside the shell, and a step *along* it
/// outside. The probe distance is a free choice — any distance small enough
/// to stay within the material works — so it starts from the shell's own
/// extent and halves until the two probes disagree decisively, exactly as
/// [`face_interior_point`] halves its way inward. A face too thin to
/// resolve either way is skipped rather than guessed at.
pub fn check_normals_point_outward<S: Scalar>(
    params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    // A sheet bounds nothing, so either side of it may face anywhere.
    for (&shell_id, shell) in &model.shells {
        // Measured against the whole solid: a void's faces point into the
        // void, out of the material around it.
        let Some(solid_id) = shell.solid else {
            continue;
        };
        for &face_id in &shell.faces {
            let Some(face) = model.faces.get(&face_id) else {
                continue;
            };
            let Ok((u, v)) = face_interior_point(
                model,
                face_id,
                params.max_nodes,
                params.min_subdivision_size,
                SEED,
            ) else {
                // Reported by `check_faces_have_interior`; nothing to add.
                continue;
            };
            let (Ok(point), Ok(normal)) = (face.surface.evaluate(u, v), face.surface.normal(u, v))
            else {
                continue;
            };

            // Probe both sides, halving all the way down and keeping the
            // *last* decisive answer rather than the first.
            //
            // Which side of a face is solid is a local fact — the limit as the
            // probe distance goes to zero — and a long probe answers a
            // different question. On a non-convex solid it reaches right past
            // the local material into some other part: probing outward from
            // the figure-8's neck wall by half the model's extent lands inside
            // the *opposite lobe*, which reads as an inverted normal on a face
            // that is perfectly correct. Every halving that still resolves is
            // more local than the one before, so the smallest one that
            // resolves is the answer.
            let mut step = shell_extent(model, shell_id);
            let mut verdict = None;
            for _ in 0..MAX_HALVINGS {
                let outward = point.add(&normal.prod_scalar(step));
                let inward = point.sub(&normal.prod_scalar(step));
                if let (Ok(out_class), Ok(in_class)) = (
                    solid_contains(
                        model,
                        solid_id,
                        outward,
                        params.max_nodes,
                        params.min_subdivision_size,
                        SEED,
                    ),
                    solid_contains(
                        model,
                        solid_id,
                        inward,
                        params.max_nodes,
                        params.min_subdivision_size,
                        SEED,
                    ),
                ) {
                    let resolved = match (out_class, in_class) {
                        (PointClassification::Outside, PointClassification::Inside) => Some(true),
                        (PointClassification::Inside, PointClassification::Outside) => Some(false),
                        // Both probes on the same side, or on a boundary:
                        // this distance resolves nothing. Halve and retry.
                        _ => None,
                    };
                    // Report only if *every* probe that resolved says the
                    // normal points inward. Any single probe finding solid on
                    // the far side is proof the face is oriented correctly,
                    // and no probe distance is trustworthy on its own: a long
                    // one reaches past the local material into another part of
                    // a non-convex solid (the figure-8's neck sees the
                    // opposite lobe), while one near `min_subdivision_size`
                    // cannot tell "just off the face" from "on it". Requiring
                    // unanimity means a false positive needs *every* scale to
                    // agree wrongly, and keeps the check one-sided: it never
                    // fails a face it has any evidence for.
                    match resolved {
                        Some(true) => {
                            verdict = Some(true);
                            break;
                        }
                        Some(false) => verdict = Some(verdict != Some(true) && false),
                        None => {}
                    }
                }
                // Stop once the probe would be closer to the surface than the
                // containment search can resolve. Below `min_subdivision_size`
                // the query cannot tell "just off the face" from "on it", so a
                // verdict there is noise — and since the last decisive verdict
                // wins, letting the loop run past this floor would hand the
                // answer to exactly the least reliable probe.
                let next = match step.div(S::TWO) {
                    Ok(s) => s,
                    Err(_) => break,
                };
                if !next.definitely_greater(params.min_subdivision_size) {
                    break;
                }
                step = next;
            }
            if verdict == Some(false) {
                errors.push(GeopError::new(format!(
                    "face {face_id}'s normal points into solid {solid_id} (from its shell {shell_id}) rather than out of it: at its interior point {point:?} the normal is {normal:?}, and the closest probe that resolved put the solid on the normal's side"
                )));
            }
        }
    }
}

/// A length comparable to the shell's own size, as a starting probe
/// distance: the largest coordinate extent of its faces' interior points and
/// the model's vertices, or 1 if there is nothing to measure.
fn shell_extent<S: Scalar>(model: &Model<S>, shell_id: crate::ShellId) -> S {
    let mut lo = None;
    let mut hi = None;
    let Some(shell) = model.shells.get(&shell_id) else {
        return S::ONE;
    };
    for &face_id in &shell.faces {
        for coedge_id in model.iterate_face_coedges(face_id) {
            let Some(coedge) = model.coedges.get(&coedge_id) else {
                continue;
            };
            let Ok(vertex) = model.coedge_start_vertex(coedge_id) else {
                let _ = coedge;
                continue;
            };
            for c in 0..3 {
                lo = Some(match lo {
                    None => vertex.point[c],
                    Some(l) => {
                        if vertex.point[c].definitely_less(l) {
                            vertex.point[c]
                        } else {
                            l
                        }
                    }
                });
                hi = Some(match hi {
                    None => vertex.point[c],
                    Some(h) => {
                        if vertex.point[c].definitely_greater(h) {
                            vertex.point[c]
                        } else {
                            h
                        }
                    }
                });
            }
        }
    }
    match (lo, hi) {
        (Some(l), Some(h)) => {
            let extent = h.sub(l);
            if extent.definitely_greater(S::ZERO) {
                extent
            } else {
                S::ONE
            }
        }
        _ => S::ONE,
    }
}
