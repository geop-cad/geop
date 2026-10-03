use geop_core_geometry::{
    contains::curve::curve_could_contain,
    intersection::{curve_curve_intersect, refine_curve_curve_crossing},
    nurb_curve::{NurbCurve, NurbCurve2D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
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
pub fn face_contains<S: Scalar>(
    model: &Model<S>,
    face_id: FaceId,
    u: S,
    v: S,
    max_nodes: usize,
    epsilon: S,
    seed: u64,
) -> GeopResult<PointClassification> {
    let face = &model.faces[&face_id];
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
pub fn loops_contain<S: Scalar>(
    model: &Model<S>,
    surface: &NurbSurface3D<S>,
    coedges: &[CoedgeId],
    query: Vector2<S>,
    max_nodes: usize,
    epsilon: S,
    seed: u64,
) -> GeopResult<PointClassification> {
    // Is the query point itself a vertex, or on some coedge's pcurve?
    // Checked as two full passes (all vertices, then all curves) so
    // `OnVertex` always takes priority over `OnCoedge` regardless of
    // coedge iteration order.
    for &coedge_id in coedges {
        let pcurve = &model.coedges[&coedge_id].pcurve;
        let vertex_pt = pcurve.evaluate(pcurve.domain().0)?;
        if vertex_pt.could_be_equal(&query) {
            return Ok(PointClassification::OnVertex);
        }
    }
    for &coedge_id in coedges {
        let pcurve = &model.coedges[&coedge_id].pcurve;
        if curve_could_contain(pcurve, &query, max_nodes, epsilon)?.is_some() {
            return Ok(PointClassification::OnCoedge);
        }
    }

    let (u_lo, u_hi) = surface.domain_u();
    let (v_lo, v_hi) = surface.domain_v();
    let du = u_hi.sub(u_lo);
    let dv = v_hi.sub(v_lo);
    let diag = du.mul(du).add(dv.mul(dv)).sqrt()?;
    let ray_length = diag.mul(S::from_f64(3.0)).add(S::ONE);

    let mut rng = Rng::new(seed);
    // Why the most recent direction was given up on — reported if every one
    // is, since "no clear direction" alone says nothing about the cause.
    let mut last_rejection = String::new();
    'attempt: for _ in 0..MAX_RAY_ATTEMPTS {
        let dir = rng.next_direction2::<S>();
        let far = query.add(&dir.prod_scalar(ray_length));
        let ray: NurbCurve2D<S> = NurbCurve::try_new(
            1,
            vec![
                Vector3::from_array([query[0], query[1], S::ONE]),
                Vector3::from_array([far[0], far[1], S::ONE]),
            ],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )?;

        for &coedge_id in coedges {
            let pcurve = &model.coedges[&coedge_id].pcurve;
            let vertex_pt = pcurve.evaluate(pcurve.domain().0)?;
            if curve_could_contain(&ray, &vertex_pt, max_nodes, epsilon)?.is_some() {
                last_rejection = format!(
                    "ray {ray:?} could pass through coedge {coedge_id}'s start {vertex_pt:?}"
                );
                continue 'attempt;
            }
        }

        let mut count = 0usize;
        for &coedge_id in coedges {
            let pcurve = &model.coedges[&coedge_id].pcurve;
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
                    last_rejection =
                        format!("ray {ray:?} x coedge {coedge_id} pcurve {pcurve:?}: {e:?}");
                    continue 'attempt;
                }
            };
            for (t, mid) in hits {
                // A crossing the search cannot place beyond the query point
                // is refined until it can: a crossing right next to the query
                // is a real one — the boundary is just there — and has to be
                // counted. Dropping every hit within `epsilon` of the query,
                // as this once did, turned a point 6e-5 outside a face into
                // one inside it.
                let (t, mid) = if t.definitely_greater(S::ZERO) {
                    (t, mid)
                } else {
                    refine_curve_curve_crossing(&ray, pcurve, t, mid)
                };
                if !t.definitely_greater(S::ZERO) {
                    // Still not beyond it. Either the query is on this
                    // pcurve to within what the numbers can tell — the
                    // check above asks about the sharp query, and a point
                    // 6e-16 off the boundary passes it — or the search met
                    // the pcurve only at its own resolution, near the query
                    // but not on it, and this ray is as ambiguous as a
                    // vertex graze.
                    if pcurve.evaluate(mid)?.could_be_equal(&query) {
                        return Ok(PointClassification::OnCoedge);
                    }
                    last_rejection = format!(
                        "ray {ray:?} meets coedge {coedge_id} at t={t:?}, which cannot be told from the query point"
                    );
                    continue 'attempt;
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
                        "ray {ray:?} grazes coedge {coedge_id}'s end at t={mid:?} (domain {d0:?}..{d1:?})"
                    );
                    continue 'attempt;
                }
                count += 1;
            }
        }
        return Ok(if count % 2 == 1 {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        });
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
/// `max_nodes`/`epsilon`/`seed` are passed straight to [`face_contains`].
pub fn face_interior_point<S: Scalar>(
    model: &Model<S>,
    face_id: FaceId,
    max_nodes: usize,
    epsilon: S,
    seed: u64,
) -> GeopResult<(S, S)> {
    let found =
        face_interior_point_where(model, face_id, max_nodes, epsilon, seed, |_, _| Ok(true))?;
    Ok(found.expect("the first interior point found is always accepted"))
}

/// Like [`face_interior_point`], but for a caller that needs a point with
/// some further property: up to one interior point per outer coedge (stepped
/// in from it, in loop order) is handed to `accept`, and the first it accepts
/// is returned. `Ok(None)` means interior points were found but none was
/// accepted; an error, as for `face_interior_point`, that none was found.
pub fn face_interior_point_where<S: Scalar>(
    model: &Model<S>,
    face_id: FaceId,
    max_nodes: usize,
    epsilon: S,
    seed: u64,
    mut accept: impl FnMut(S, S) -> GeopResult<bool>,
) -> GeopResult<Option<(S, S)>> {
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

    let mut found_any = false;
    'base: for &coedge_id in &coedges {
        let pcurve = &model.get_coedge(coedge_id)?.pcurve;
        let (t0, t1) = pcurve.domain();
        let t = t0.add(t1).div(S::TWO)?.sharpen();
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
            Vector2::from_array([tangent[1].neg(), tangent[0]]),
            Vector2::from_array([tangent[1], tangent[0].neg()]),
        ];

        let mut step = diagonal.div(S::TWO)?;
        let mut offered = 0;
        for _ in 0..MAX_HALVINGS {
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
                // Accepted only if the whole box of radius `epsilon` around it
                // is inside, not just the point: the point is a free choice,
                // and every later use of it (a classification ray, a surface
                // evaluation compared against other solids) works at that
                // resolution. A point merely strictly inside can sit 5e-16
                // off the boundary — one inward step exactly the face's width
                // lands there — and at `epsilon` it *is* on the boundary.
                let neighbourhood = |t: S| t.sub(epsilon).union(t.add(epsilon));
                if matches!(
                    face_contains(
                        model,
                        face_id,
                        neighbourhood(u),
                        neighbourhood(v),
                        max_nodes,
                        epsilon,
                        seed
                    )?,
                    PointClassification::Inside
                ) {
                    found_any = true;
                    if accept(u, v)? {
                        return Ok(Some((u, v)));
                    }
                    // Rejected: the next point comes from half this step, so
                    // it differs from this one — stepping in from each side
                    // of a symmetric face lands every base on the same
                    // centre, so bases alone don't give distinct points. Two
                    // points per base keep a caller that rejects everything
                    // (every point it tries turns out alike) cheap.
                    offered += 1;
                    if offered == POINTS_PER_BASE {
                        continue 'base;
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
            if !step.definitely_greater(epsilon) {
                break;
            }
            step = step.div(S::TWO)?;
        }
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
            let Ok(frac) = S::from_ratio(i, 4) else {
                continue;
            };
            let Ok(uv) = coedge.pcurve.evaluate(t0.add(t1.sub(t0).mul(frac))) else {
                continue;
            };
            u_extent = Some(match u_extent {
                None => uv[0],
                Some(e) => S::union(e, uv[0]),
            });
            v_extent = Some(match v_extent {
                None => uv[1],
                Some(e) => S::union(e, uv[1]),
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
    use geop_core_math::scalars::{ScalInF64, Scalar};

    const MAX: usize = 20000;
    const SEED: u64 = 99;

    fn eps() -> ScalInF64 {
        <ScalInF64 as Scalar>::from_f64(1e-4)
    }

    /// Every face of a plain cube must yield an interior point.
    #[test]
    fn cube_faces_all_have_interior_points() {
        let mut model = Model::<ScalInF64>::new();
        let solid = test_cube_solid(&mut model);
        for face_id in model.solid_faces(solid).unwrap() {
            face_interior_point(&model, face_id, MAX, eps(), SEED)
                .unwrap_or_else(|e| panic!("face {face_id}: {e}"));
        }
    }

    // `sphere_faces_all_have_interior_points` (exercising seam curves and
    // degenerate poles, unlike the plain cube above) needs a real
    // `sphere_solid` — that lives in `geop-ops-extrude-revolve`, which
    // depends on this crate, so it can't be reached from here without
    // Cargo compiling this crate twice (see `test_fixtures`'s doc
    // comment). Covered instead by `geop-ops-extrude-revolve::sphere`'s
    // own tests, which build the same solid `face_interior_point` runs on
    // here.
}

#[cfg(test)]
mod tests {
    use super::{PointClassification, face_contains};
    use crate::{
        Coedge, CoedgeGeometry, CoedgeId, Edge, Face, FaceId, Model, Sense, ShellId, Vertex,
        VertexId, boundary::BoundaryType, model::Curve3,
    };
    use geop_core_geometry::{
        nurb_curve::{NurbCurve, NurbCurve2D},
        nurb_surface::NurbSurface3D,
    };
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    const MAX: usize = 200;
    const EPS: f64 = 1e-3;
    const SEED: u64 = 12345;

    fn p2<S: Scalar>(x: f64, y: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::ONE])
    }

    fn line2<S: Scalar>(a: (f64, f64), b: (f64, f64)) -> NurbCurve2D<S> {
        NurbCurve::try_new(
            1,
            vec![p2(a.0, a.1), p2(b.0, b.1)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap()
    }

    /// A polygon face on the unit-square `[0,1]^2` parameter surface, built
    /// from `points` (CCW, `(u, v) == (x, y)`).
    fn polygon_face<S: Scalar>(model: &mut Model<S>, points: &[(f64, f64)]) -> FaceId {
        let p =
            |x: f64, y: f64| Vector4::from_array([S::from_f64(x), S::from_f64(y), S::ZERO, S::ONE]);
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![p(0.0, 0.0), p(0.0, 1.0), p(1.0, 0.0), p(1.0, 1.0)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
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
                    point: Vector3::from_array([S::from_f64(x), S::from_f64(y), S::ZERO]),
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
                                S::from_f64(points[i].0),
                                S::from_f64(points[i].1),
                                S::ZERO,
                                S::ONE,
                            ]),
                            Vector4::from_array([
                                S::from_f64(points[(i + 1) % n].0),
                                S::from_f64(points[(i + 1) % n].1),
                                S::ZERO,
                                S::ONE,
                            ]),
                        ],
                        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
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
    fn diamond_face<S: Scalar>(model: &mut Model<S>) -> FaceId {
        polygon_face(model, &[(1., 0.5), (0.5, 1.), (0., 0.5), (0.5, 0.)])
    }

    fn check_diamond_interior_point_is_contained<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                S::from_f64(0.5),
                S::from_f64(0.3),
                MAX,
                S::from_f64(EPS),
                SEED
            )
            .unwrap(),
            PointClassification::Inside
        );
    }
    #[test]
    fn diamond_interior_point_is_contained() {
        for_all_scalars!(check_diamond_interior_point_is_contained);
    }

    fn check_diamond_exterior_point_is_not_contained<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                S::from_f64(0.1),
                S::from_f64(0.3),
                MAX,
                S::from_f64(EPS),
                SEED
            )
            .unwrap(),
            PointClassification::Outside
        );
    }
    #[test]
    fn diamond_exterior_point_is_not_contained() {
        for_all_scalars!(check_diamond_exterior_point_is_not_contained);
    }

    fn check_diamond_center_hits_convex_vertex_from_inside<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                S::from_f64(0.5),
                S::from_f64(0.5),
                MAX,
                S::from_f64(EPS),
                SEED
            )
            .unwrap(),
            PointClassification::Inside
        );
    }
    #[test]
    fn diamond_center_hits_convex_vertex_from_inside() {
        for_all_scalars!(check_diamond_center_hits_convex_vertex_from_inside);
    }

    fn check_diamond_vertex_query_is_on_vertex<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                S::ONE,
                S::from_f64(0.5),
                MAX,
                S::from_f64(EPS),
                SEED
            )
            .unwrap(),
            PointClassification::OnVertex
        );
    }
    #[test]
    fn diamond_vertex_query_is_on_vertex() {
        for_all_scalars!(check_diamond_vertex_query_is_on_vertex);
    }

    fn check_diamond_edge_query_is_on_coedge<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face_id = diamond_face(&mut model);
        assert_eq!(
            face_contains(
                &model,
                face_id,
                S::from_f64(0.75),
                S::from_f64(0.75),
                MAX,
                S::from_f64(EPS),
                SEED
            )
            .unwrap(),
            PointClassification::OnCoedge
        );
    }
    #[test]
    fn diamond_edge_query_is_on_coedge() {
        for_all_scalars!(check_diamond_edge_query_is_on_coedge);
    }
}
