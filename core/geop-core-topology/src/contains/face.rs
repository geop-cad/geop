use geop_core_geometry::{
    contains::curve::curve_could_contain,
    intersection::{curve_curve_intersect, refine_curve_curve_crossing},
    nurb_curve::{NurbCurve, NurbCurve2D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    solvers::probe::{Probe, attempts, halve},
    vector::{Vector2, Vector3},
};

use crate::{CoedgeId, FaceId, Model, boundary::BoundaryType, contains::rng::Rng};

/// Result of classifying a query point against a face's trimmed boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointClassification {
    /// The query point coincides with a vertex (shared by two coedges).
    OnVertex,
    /// The query point lies on a coedge's pcurve, away from its endpoints.
    OnCoedge,
    /// The query point is strictly inside the trimmed boundary.
    Inside,
    /// The query point is strictly outside the trimmed boundary (or inside
    /// a hole).
    Outside,
}

const MAX_RAY_ATTEMPTS: usize = 64;

/// Classify `(u, v)` against `face_id`'s trimmed boundary (outer loop minus
/// holes): [`PointClassification::OnVertex`] / [`PointClassification::OnCoedge`]
/// if the query point itself coincides with a vertex or lies on a coedge's
/// pcurve, else [`PointClassification::Inside`]/[`PointClassification::Outside`]
/// via ray casting in parameter space.
///
/// The ray direction is drawn from a seeded PRNG (see [`Rng`]) and retried
/// (up to a bounded number of attempts) until every crossing it finds lands
/// strictly inside a coedge's pcurve, away from any vertex — vertex grazes
/// are ambiguous to count (shared by two coedges) so they're avoided rather
/// than specially classified. Once such a direction is found, the parity
/// (even/odd) of its crossing count determines inside/outside; this needs no
/// normal or winding-direction information, so it works regardless of a
/// face's loop orientation.
///
/// `max_nodes` bounds both the `curve_could_contain` BFS subdivision search
/// (on-vertex/on-coedge tests) and the `curve_curve_intersect` DFS search
/// (edge-interior hits). `seed` seeds the direction PRNG.
pub fn face_contains(
    model: &Model,
    face_id: FaceId,
    u: Scalar,
    v: Scalar,
    max_nodes: usize,
    epsilon: Scalar,
    seed: u64,
) -> GeopResult<PointClassification> {
    let face = &model.faces[&face_id];
    // A hole that is a bare vertex (another body touching the face at a
    // point) has no pcurve for the loops to find it by: it is a point of
    // the boundary all the same, and a query that could be it is on it.
    for boundary in face.boundaries() {
        if let BoundaryType::Vertex(vertex_id) = boundary {
            let at = face.surface.evaluate(u, v)?;
            if at.could_be_equal(&model.get_vertex(vertex_id)?.point) {
                return Ok(PointClassification::OnVertex);
            }
        }
    }
    let coedges: Vec<CoedgeId> = model.iterate_face_coedges(face_id).collect();
    loops_contain(
        model,
        &face.surface,
        &coedges,
        Vector2::from_array([u, v]),
        max_nodes,
        epsilon,
        seed,
    )
}

/// The same classification as [`face_contains`], but against an explicit set
/// of loops rather than all of a face's.
///
/// Exists because "inside this face" and "inside this face's *outer* loop"
/// are different questions, and validation needs the second: a hole has to
/// lie within the outer boundary, and asking [`face_contains`] would only
/// ever answer `OnCoedge` for a point taken from the hole itself. Splitting
/// the ray casting out here keeps one implementation of it rather than a
/// second copy that could drift.
pub fn loops_contain(
    model: &Model,
    surface: &NurbSurface3D,
    coedges: &[CoedgeId],
    query: Vector2,
    max_nodes: usize,
    epsilon: Scalar,
    seed: u64,
) -> GeopResult<PointClassification> {
    let pcurves: Vec<&NurbCurve2D> = coedges
        .iter()
        .map(|c| Ok(&model.get_coedge(*c)?.pcurve))
        .collect::<GeopResult<_>>()?;
    pcurves_contain(surface, &pcurves, query, max_nodes, epsilon, seed).with_context(
        &|e: GeopError| {
            e.with_context(format!(
                "loops_contain(coedges={coedges:?}, pcurve k is coedge k)"
            ))
        },
    )
}

/// The classification of [`loops_contain`], against the closed loops that
/// `pcurves` run around on `surface` — for loops not in a model yet, being
/// built. `pcurve k` in an error is `pcurves[k]`.
pub fn pcurves_contain(
    surface: &NurbSurface3D,
    pcurves: &[&NurbCurve2D],
    query: Vector2,
    max_nodes: usize,
    epsilon: Scalar,
    seed: u64,
) -> GeopResult<PointClassification> {
    // Is the query point itself a vertex, or on some coedge's pcurve?
    // Checked as two full passes (all vertices, then all curves) so
    // `OnVertex` always takes priority over `OnCoedge` regardless of
    // coedge iteration order.
    for pcurve in pcurves {
        let vertex_pt = pcurve.evaluate(pcurve.domain().0)?;
        if vertex_pt.could_be_equal(&query) {
            return Ok(PointClassification::OnVertex);
        }
    }
    for pcurve in pcurves {
        if curve_could_contain(pcurve, &query, max_nodes, epsilon)?.is_some() {
            return Ok(PointClassification::OnCoedge);
        }
    }

    // The query touches no vertex and no pcurve, so — connected as it is —
    // all of it lies on one side of the boundary, and any one point of it
    // decides which: the ray is cast from a sharp one. Cast from a wide
    // query, the ray would be a strip, and a strip merely grazing a pcurve
    // overlaps it as a crossing does — one hit, flipping the parity — where
    // a ray from any single point of the query misses it or crosses twice.
    let origin = Vector2::from_array([query[0].midpoint(), query[1].midpoint()]);

    let (u_lo, u_hi) = surface.domain_u();
    let (v_lo, v_hi) = surface.domain_v();
    let du = u_hi.sub(u_lo);
    let dv = v_hi.sub(v_lo);
    let diag = du.mul(du).add(dv.mul(dv)).sqrt()?;
    let ray_length = diag.mul(Scalar::from_f64(3.0)).add(Scalar::ONE);

    let mut rng = Rng::new(seed);
    // Why the most recent direction was given up on — reported if every one
    // is, since "no clear direction" alone says nothing about the cause.
    let mut last_rejection = String::new();
    // Directions tried in turn until one is clear.
    let found = attempts(MAX_RAY_ATTEMPTS, |_| -> GeopResult<_> {
        let dir = rng.next_direction2();
        let far = origin.add(&dir.prod_scalar(ray_length));
        let ray: NurbCurve2D = NurbCurve::try_new(
            1,
            vec![
                Vector3::from_array([origin[0], origin[1], Scalar::ONE]),
                Vector3::from_array([far[0], far[1], Scalar::ONE]),
            ],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
        )?;

        for (k, pcurve) in pcurves.iter().enumerate() {
            let vertex_pt = pcurve.evaluate(pcurve.domain().0)?;
            if curve_could_contain(&ray, &vertex_pt, max_nodes, epsilon)?.is_some() {
                last_rejection =
                    format!("ray {ray:?} could pass through pcurve {k}'s start {vertex_pt:?}");
                return Ok(Probe::Next);
            }
        }

        let mut count = 0usize;
        for (k, pcurve) in pcurves.iter().enumerate() {
            let (d0, d1) = pcurve.domain();
            // An `Err` here means the search exhausted its node budget
            // before converging — the same ambiguous signal as a vertex/edge
            // graze (see `curve_curve_intersect`'s own doc comment: budget
            // exhaustion is itself evidence of a near-tangential or
            // coincident ray, not a reliable crossing count), so it's
            // handled the same way: retry with a fresh direction rather than
            // propagating a hard failure.
            let hits = match curve_curve_intersect(&ray, pcurve, max_nodes, max_nodes, epsilon) {
                Ok(hits) => hits.into_vec(),
                Err(e) => {
                    last_rejection = format!("ray {ray:?} x pcurve {k} pcurve {pcurve:?}: {e:?}");
                    return Ok(Probe::Next);
                }
            };
            for (t, mid) in hits {
                // A crossing the search cannot place beyond the query point
                // is refined until it can: a crossing right next to the query
                // is a real one — the boundary is just there — and has to be
                // counted. Dropping every hit within `epsilon` of the query,
                // as this once did, turned a point 6e-5 outside a face into
                // one inside it.
                let (t, mid, refined) = if t.definitely_greater(Scalar::ZERO) {
                    (t, mid, false)
                } else {
                    let (rt, rmid) = refine_curve_curve_crossing(&ray, pcurve, t, mid);
                    // Refinement returns the box unchanged where it fails;
                    // where it converges, the box is far below the search's
                    // resolution.
                    let same = rt.is_subset_of(t) && t.is_subset_of(rt);
                    (rt, rmid, !same && rt.width().definitely_less(epsilon))
                };
                if !t.definitely_greater(Scalar::ZERO) {
                    // Still not beyond it. Either the query is on this
                    // pcurve to within what the numbers can tell — the
                    // check above asks about the sharp query, and a point
                    // 6e-16 off the boundary passes it — or the search met
                    // the pcurve only at its own resolution, near the query
                    // but not on it, and this ray is as ambiguous as a
                    // vertex graze. A crossing Newton has isolated is the
                    // first: the ray meets the pcurve where it starts, as
                    // far as rounding can tell. A point 4e-16 inside a face
                    // met its boundary so along every ray, and was never
                    // classified.
                    if refined || pcurve.evaluate(mid)?.could_be_equal(&query) {
                        return Ok(Probe::Found(PointClassification::OnCoedge));
                    }
                    last_rejection = format!(
                        "ray {ray:?} meets pcurve {k} at t={t:?}, which cannot be told from the query point {query:?}; the pcurve there is {:?}, the pcurve {pcurve:?}",
                        pcurve.evaluate(mid)?
                    );
                    return Ok(Probe::Next);
                }
                // `curve_curve_intersect` honestly returns the whole
                // surviving span of its converged leaf, not an arbitrarily
                // narrowed midpoint — sharpen before comparing against the
                // pcurve's own endpoints.
                let mid = mid.midpoint();
                if !mid.sub(d0).abs().definitely_greater(epsilon)
                    || !mid.sub(d1).abs().definitely_greater(epsilon)
                {
                    // Grazes a vertex despite the check above (numerical
                    // slop right at the boundary) — retry with a fresh
                    // direction rather than risk mis-counting it.
                    last_rejection = format!(
                        "ray {ray:?} grazes pcurve {k}'s end at t={mid:?} (domain {d0:?}..{d1:?})"
                    );
                    return Ok(Probe::Next);
                }
                count += 1;
            }
        }
        Ok(Probe::Found(if count % 2 == 1 {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        }))
    })?;
    if let Some(classification) = found {
        return Ok(classification);
    }
    Err(GeopError::new(format!(
        "loops_contain: could not find a ray direction clear of every vertex after many attempts; \
         the last one was rejected because {last_rejection}"
    )))
}

/// How many interior points [`face_interior_point_where`] offers from one
/// boundary base point before moving to the next: enough for two genuinely
/// different points, few enough that rejecting everything stays cheap.
const POINTS_PER_BASE: usize = 2;

/// How many times [`face_interior_point`] may halve its step before giving
/// up. Bounds effort only: each halving is another attempt to land inside the
/// trim, and exhausting them is reported as an error rather than accepted.
const MAX_HALVINGS: usize = 40;

/// How many times [`face_interior_point_where`] makes its margin from the
/// boundary finer, by [`MARGIN_RATIO`] each time, before it gives up: from
/// the usual `1e-4` to `1.5e-9` of the surface's parameters.
const FINER_MARGINS: usize = 4;
/// How much finer each margin is than the last.
const MARGIN_RATIO: f64 = 16.0;

/// A `(u, v)` strictly inside `face_id`'s trimmed region.
///
/// Found the way the operation itself suggests: start on the boundary and
/// step inward. The step is taken along the inward normal of the outer loop
/// at a boundary point — inward in `(u, v)`, obtained by rotating the loop's
/// own tangent — and halved whenever the point it lands on is not
/// `Inside`. Halving converges on any non-degenerate face, since a
/// sufficiently short inward step from a boundary point is always interior,
/// and it needs no guess about the face's size.
///
/// The domain midpoint is not usable for this: a trimmed face need not
/// contain it, and for a face carved out by a boolean's remesh it very often
/// does not.
///
/// `max_nodes`/`seed` are passed straight to [`face_contains`], and `epsilon` as
/// its first tolerance: how far clear of the boundary, in `(u, v)`, a point
/// has to be — made finer where none is that far (see
/// [`face_interior_point_where`]).
pub fn face_interior_point(
    model: &Model,
    face_id: FaceId,
    max_nodes: usize,
    epsilon: Scalar,
    seed: u64,
) -> GeopResult<(Scalar, Scalar)> {
    let found =
        face_interior_point_where(model, face_id, max_nodes, epsilon, seed, |_, _| Ok(true))?;
    Ok(found.expect("the first interior point found is always accepted"))
}

/// Like [`face_interior_point`], but for a caller that needs a point with
/// some further property: up to one interior point per outer coedge (stepped
/// in from it, in loop order) is handed to `accept`, and the first it accepts
/// is returned. `Ok(None)` means interior points were found but none was
/// accepted; an error, as for `face_interior_point`, that none was found.
pub fn face_interior_point_where(
    model: &Model,
    face_id: FaceId,
    max_nodes: usize,
    epsilon: Scalar,
    seed: u64,
    mut accept: impl FnMut(Scalar, Scalar) -> GeopResult<bool>,
) -> GeopResult<Option<(Scalar, Scalar)>> {
    let face = &model.faces[&face_id];
    let BoundaryType::Loop(anchor) = face.outer else {
        return Err(GeopError::new(format!(
            "face_interior_point: face {face_id} is bounded by a bare vertex, so it has no interior to sample"
        )));
    };

    let (u_lo, u_hi) = face.surface.domain_u();
    let (v_lo, v_hi) = face.surface.domain_v();
    let du = u_hi.sub(u_lo);
    let dv = v_hi.sub(v_lo);
    let diagonal = if dv.definitely_greater(du) { dv } else { du };

    // Every coedge of the outer loop is a candidate base point, not just the
    // anchor's. One base point is not enough in practice: a loop can pass
    // through a degenerate stretch (a revolve pole, a sliver left by a face
    // split) where the tangent is unusable or where the face is locally
    // thinner than `face_contains`' own tolerance band, and there the search
    // fails however finely it steps — while a different side of the very same
    // face offers an easy interior point.
    let coedges: Vec<CoedgeId> = model
        .iterate_loop_coedges(anchor)
        .take(model.coedges.len() + 1)
        .collect();

    // How far clear of the boundary a point has to be, in `(u, v)`: the
    // search's resolution `epsilon` first, then finer. A face thinner in its
    // surface's parameters than that resolution — a fillet's corner piece
    // on a face a hundred times its size — has interior points all the
    // same; the margin only says how hard the search tries to find one, and
    // a finer one takes `face_contains` more subdivision.
    let mut found_any = false;
    let mut margin = epsilon;
    for _ in 0..=FINER_MARGINS {
        for &coedge_id in &coedges {
            let pcurve = &model.get_coedge(coedge_id)?.pcurve;
            let t = pcurve.mid_parameter()?;
            let Ok(base) = pcurve.evaluate(t) else {
                continue;
            };
            let Ok(tangent) = pcurve.tangent(t).and_then(|d| d.normalize()) else {
                continue;
            };

            // Rotate the tangent a quarter turn in `(u, v)`. Which of the two
            // perpendiculars points *into* the face depends on the loop's
            // winding, so both are tried and whichever lands inside wins —
            // cheaper and more robust than deriving the winding.
            let normals = [
                tangent.perp(),
                Vector2::from_array([tangent[1], tangent[0].neg()]),
            ];

            let mut offered = 0;
            let found = halve(
                diagonal.div(Scalar::TWO)?,
                MAX_HALVINGS,
                |&step| -> GeopResult<_> {
                    for inward in normals {
                        // Any point inside the face will do — a free choice — so the
                        // candidate is sharp: it doesn't inherit the width of the
                        // pcurve it was stepped from, and every later computation
                        // from it (a classification ray, say) starts from a point.
                        let u = base[0].add(inward[0].mul(step)).sharpen();
                        let v = base[1].add(inward[1].mul(step)).sharpen();
                        if u.definitely_less(u_lo)
                            || u.definitely_greater(u_hi)
                            || v.definitely_less(v_lo)
                            || v.definitely_greater(v_hi)
                        {
                            continue;
                        }
                        // Accepted only if the whole box of radius `margin` around it
                        // is inside, not just the point: the point is a free choice,
                        // and every later use of it (a classification ray, a surface
                        // evaluation compared against other solids) works at that
                        // resolution. A point merely strictly inside can sit 5e-16
                        // off the boundary — one inward step exactly the face's width
                        // lands there — and at `margin` it *is* on the boundary.
                        let neighbourhood = |t: Scalar| t.sub(margin).union(t.add(margin));
                        if matches!(
                            face_contains(
                                model,
                                face_id,
                                neighbourhood(u),
                                neighbourhood(v),
                                max_nodes,
                                margin,
                                seed
                            )?,
                            PointClassification::Inside
                        ) {
                            found_any = true;
                            if accept(u, v)? {
                                return Ok(Probe::Found((u, v)));
                            }
                            // Rejected: the next point comes from half this step, so
                            // it differs from this one — stepping in from each side
                            // of a symmetric face lands every base on the same
                            // centre, so bases alone don't give distinct points. Two
                            // points per base keep a caller that rejects everything
                            // (every point it tries turns out alike) cheap.
                            offered += 1;
                            if offered == POINTS_PER_BASE {
                                return Ok(Probe::Stop);
                            }
                            // Leave the direction loop, reaching the halving below.
                            break;
                        }
                    }
                    // Halving walks toward the boundary, and `face_contains` reports
                    // `OnCoedge` for anything within its own tolerance of it — so
                    // below that tolerance every step is `OnCoedge` and no amount of
                    // further halving can succeed. Stop and let the next base point
                    // try instead of burning the remaining budget here.
                    Ok(if step.definitely_greater(margin) {
                        Probe::Next
                    } else {
                        Probe::Stop
                    })
                },
            )?;
            if found.is_some() {
                return Ok(found);
            }
        }
        if found_any {
            break;
        }
        margin = margin.div(Scalar::from_f64(MARGIN_RATIO))?;
    }

    if found_any {
        return Ok(None);
    }

    // Report the outer loop's own `(u, v)` extent. A loop that encloses no
    // area has nothing inside it and the failure is correct — a degenerate
    // face, to be fixed wherever it was created. A loop with real extent
    // means the search failed on a face that does have an interior, which is
    // this function's bug. The two need opposite fixes and are otherwise
    // indistinguishable from the message.
    let mut u_extent = None;
    let mut v_extent = None;
    for &coedge_id in &coedges {
        let Ok(coedge) = model.get_coedge(coedge_id) else {
            continue;
        };
        let (t0, t1) = coedge.pcurve.domain();
        for i in 0..=4 {
            let Ok(frac) = Scalar::from_ratio(i, 4) else {
                continue;
            };
            let Ok(uv) = coedge.pcurve.evaluate(t0.add(t1.sub(t0).mul(frac))) else {
                continue;
            };
            u_extent = Some(match u_extent {
                None => uv[0],
                Some(e) => Scalar::union(e, uv[0]),
            });
            v_extent = Some(match v_extent {
                None => uv[1],
                Some(e) => Scalar::union(e, uv[1]),
            });
        }
    }
    // Each coedge's edge, start vertex and pcurve start: enough to recognise
    // the face in the model, to see a loop that doubles back on itself (an
    // edge appearing twice, as a spur), and to see a pcurve whose `(u, v)`
    // disagrees with where its 3-D vertex actually is on the surface.
    let loop_description: Vec<String> = coedges
        .iter()
        .map(|&coedge_id| {
            let start = model.coedge_start_vertex(coedge_id).map(|v| v.point);
            let coedge = model.get_coedge(coedge_id);
            let geometry = coedge.as_ref().ok().map(|c| c.geometry);
            let start_uv = coedge.and_then(|c| c.pcurve.evaluate(c.pcurve.domain().0));
            format!("{coedge_id} ({geometry:?}): starts at {start:?}, (u, v) = {start_uv:?}")
        })
        .collect();
    Err(GeopError::new(format!(
        "face_interior_point: no point strictly inside face {face_id} was found, stepping inward from the midpoint of each of its {} outer coedges; that loop spans u={u_extent:?}, v={v_extent:?} (a loop spanning nothing encloses no area, so the face is degenerate) within the surface's domain u={:?}, v={:?}; the loop: [{}]",
        coedges.len(),
        (u_lo, u_hi),
        (v_lo, v_hi),
        loop_description.join("; ")
    )))
}

#[cfg(test)]
mod interior_point_tests {
    use super::face_interior_point;
    use crate::{Model, test_fixtures::test_cube_solid};
    use geop_core_math::scalars::Scalar;

    const MAX: usize = 20000;
    const SEED: u64 = 99;

    fn eps() -> Scalar {
        Scalar::from_f64(1e-4)
    }

    /// Every face of a plain cube must yield an interior point.
    #[test]
    fn cube_faces_all_have_interior_points() {
        let mut model = Model::new();
        let solid = test_cube_solid(&mut model);
        for face_id in model.solid_faces(solid).unwrap() {
            face_interior_point(&model, face_id, MAX, eps(), SEED)
                .unwrap_or_else(|e| panic!("face {face_id}: {e}"));
        }
    }

    // `sphere_faces_all_have_interior_points` (exercising seam curves and
    // degenerate poles, unlike the plain cube above) needs a real
    // `sphere_solid` — that lives in `geop-lib-sweep`, which
    // depends on this crate, so it can't be reached from here without
    // Cargo compiling this crate twice (see `test_fixtures`'s doc
    // comment). Covered instead by `geop-lib-sweep`'s `sphere_solid`'s
    // own tests, which build the same solid `face_interior_point` runs on
    // here.
}

#[cfg(test)]
mod tests {
    use super::{PointClassification, face_contains};
    use crate::test_fixtures::line2;
    use crate::{
        Coedge, CoedgeGeometry, CoedgeId, Edge, Face, FaceId, Model, Sense, ShellId, Vertex,
        VertexId, boundary::BoundaryType, model::Curve3,
    };
    use geop_core_geometry::nurb_surface::NurbSurface3D;
    use geop_core_math::{
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    const MAX: usize = 200;
    const MIN_SUBDIVISION_SIZE: f64 = 1e-3;
    const SEED: u64 = 12345;

    /// A polygon face on the unit-square `[0,1]^2` parameter surface, built
    /// from `points` (CCW, `(u, v) == (x, y)`).
    fn polygon_face(model: &mut Model, points: &[(f64, f64)]) -> FaceId {
        let p = |x: f64, y: f64| {
            Vector4::from_array([
                Scalar::from_f64(x),
                Scalar::from_f64(y),
                Scalar::ZERO,
                Scalar::ONE,
            ])
        };
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![p(0.0, 0.0), p(0.0, 1.0), p(1.0, 0.0), p(1.0, 1.0)],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
        )
        .unwrap();

        let face_id = model.insert_face(Face {
            surface,
            outer: BoundaryType::Vertex(VertexId(0)),
            holes: Vec::new(),
            shell: ShellId(999),
        });

        let n = points.len();
        let verts: Vec<VertexId> = points
            .iter()
            .map(|&(x, y)| {
                model.insert_vertex(Vertex {
                    point: Vector3::from_array([
                        Scalar::from_f64(x),
                        Scalar::from_f64(y),
                        Scalar::ZERO,
                    ]),
                })
            })
            .collect();
        let edges = (0..n)
            .map(|i| {
                model.insert_edge(Edge {
                    curve: Curve3::try_new(
                        1,
                        vec![
                            Vector4::from_array([
                                Scalar::from_f64(points[i].0),
                                Scalar::from_f64(points[i].1),
                                Scalar::ZERO,
                                Scalar::ONE,
                            ]),
                            Vector4::from_array([
                                Scalar::from_f64(points[(i + 1) % n].0),
                                Scalar::from_f64(points[(i + 1) % n].1),
                                Scalar::ZERO,
                                Scalar::ONE,
                            ]),
                        ],
                        vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
                    )
                    .unwrap(),
                    start_vertex: verts[i],
                    end_vertex: verts[(i + 1) % n],
                })
            })
            .collect::<Vec<_>>();
        let coedges: Vec<CoedgeId> = (0..n)
            .map(|i| {
                model.insert_coedge(Coedge {
                    geometry: CoedgeGeometry::Edge(edges[i]),
                    sense: Sense::Forward,
                    pcurve: line2(points[i], points[(i + 1) % n]),
                    next: CoedgeId(0),
                    prev: CoedgeId(0),
                    face: face_id,
                })
            })
            .collect();
        for i in 0..n {
            model.coedges.get_mut(&coedges[i]).unwrap().next = coedges[(i + 1) % n];
            model.coedges.get_mut(&coedges[i]).unwrap().prev = coedges[(i + n - 1) % n];
        }
        model.faces.get_mut(&face_id).unwrap().outer = BoundaryType::Loop(coedges[0]);

        face_id
    }

    /// Diamond with corners at (1,.5) right, (.5,1) top, (0,.5) left,
    /// (.5,0) bottom, traversed CCW.
    fn diamond_face(model: &mut Model) -> FaceId {
        polygon_face(model, &[(1., 0.5), (0.5, 1.), (0., 0.5), (0.5, 0.)])
    }

    /// A block's cap is a patch whose corner is the box of its profile
    /// rounded outwards: a side at `x = 0` lay at `u = MIN_POSITIVE / 20`
    /// in it, not at the domain's edge. A point a trace's corrector clamped
    /// onto the domain's edge, `u = 0`, is outside the face by that much:
    /// every ray from it met the side at a parameter the search could not
    /// tell from zero (a block turned a quarter round its edge and joined).
    #[test]
    fn a_point_a_subnormal_outside_a_side_is_classified() {
        let side = f64::MIN_POSITIVE / 20.0;
        let mut model = Model::new();
        let face_id = polygon_face(
            &mut model,
            &[(side, side), (1., side), (1., 1.), (side, 1.)],
        );
        let found = face_contains(
            &model,
            face_id,
            Scalar::ZERO,
            Scalar::from_f64(0.0698771242968679),
            MAX,
            Scalar::from_f64(1e-4),
            SEED,
        )
        .unwrap();
        assert_ne!(found, PointClassification::Inside);
    }

    #[test]
    fn diamond_interior_point_is_contained() {
        let mut model = Model::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                Scalar::from_f64(0.5),
                Scalar::from_f64(0.3),
                MAX,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
                SEED
            )
            .unwrap(),
            PointClassification::Inside
        );
    }

    #[test]
    fn diamond_exterior_point_is_not_contained() {
        let mut model = Model::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                Scalar::from_f64(0.1),
                Scalar::from_f64(0.3),
                MAX,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
                SEED
            )
            .unwrap(),
            PointClassification::Outside
        );
    }

    #[test]
    fn diamond_center_hits_convex_vertex_from_inside() {
        let mut model = Model::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                Scalar::from_f64(0.5),
                Scalar::from_f64(0.5),
                MAX,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
                SEED
            )
            .unwrap(),
            PointClassification::Inside
        );
    }

    #[test]
    fn diamond_vertex_query_is_on_vertex() {
        let mut model = Model::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                Scalar::ONE,
                Scalar::from_f64(0.5),
                MAX,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
                SEED
            )
            .unwrap(),
            PointClassification::OnVertex
        );
    }

    #[test]
    fn diamond_edge_query_is_on_coedge() {
        let mut model = Model::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                Scalar::from_f64(0.75),
                Scalar::from_f64(0.75),
                MAX,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
                SEED
            )
            .unwrap(),
            PointClassification::OnCoedge
        );
    }
}
