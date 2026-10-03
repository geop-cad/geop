//! Split every edge at the points where an intersection branch leaves it
//! along a tangency.
//!
//! An edge `E` of one solid can lie on a face `F` of the other while the
//! face `G` it bounds touches `F` tangentially all along it — the profile
//! a revolve and an extrude of the same sketch share, say: the revolved
//! surface at its seam and the extruded wall both contain the profile and
//! the direction perpendicular to the sketch. `G ∩ F` then contains `E`,
//! and may also contain other branches that leave `E` transversally.
//! Where such a branch meets `E` is a place an intersection curve ends, so
//! it needs a vertex — but nothing else puts one there: `E` lies on `F`
//! throughout, so no piercing search can single the point out.
//!
//! Locally, with `s` along `E` and `h` across it in the common tangent
//! plane, `F` stands off `G` by `f(s, h) = h² g(s, h)` — tangency along `E`
//! is exactly what makes both `f` and `∂f/∂h` vanish at `h = 0`. So
//! `G ∩ F = {h = 0} ∪ {g = 0}`, and the other branches meet `E` where
//! `g(s, 0)` — half the difference of the two surfaces' normal curvatures
//! across `E` — vanishes. Along `E` the difference of their second
//! fundamental forms is zero in `E`'s own direction (both contain `E`) and
//! couples nothing to it (both share `E`'s normal), leaving that one
//! curvature as its trace: the points are where the two surfaces' *mean
//! curvatures* agree.

use geop_core_geometry::{
    contains::surface::surface_could_contain,
    intersection::{Intersections, curve_surface_intersect},
    nurb_curve::NurbCurve,
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::{EdgeId, FaceId, Model, SolidId, VertexId};
use geop_ops::Part;

use crate::naming::BooleanNaming;

/// How finely the edge's parameter is subdivided before each remaining
/// stretch is handed to bisection: `2^-ISOLATION_LEVELS` of its domain.
/// Like `max_nodes` this bounds effort — it only has to separate the branch
/// points, and bisection then refines each to what the data determines —
/// and it is also the worst case: two surfaces that agree in curvature all
/// along the edge (coplanar faces) never exclude anything.
const ISOLATION_LEVELS: usize = 8;

/// Newton iterations for each foot-point projection onto the two surfaces.
const NEWTON_ITERATIONS: usize = 20;

/// Sharp foot points `(u, v)` on the two surfaces, each projection's seed.
type Seeds<S> = [(S, S); 2];

/// The difference in mean curvature between `surf_g` and `surf_f` at
/// `curve(t)`, `F`'s taken against `G`'s normal — or `None` where the two
/// surfaces cannot be tangent there, and no branch can leave. `seeds` are
/// sharp foot points on each to project from (see [`tangent_branch_points`]),
/// and come back as this projection's own, sharpened.
fn curvature_gap<S: Scalar>(
    curve: &NurbCurve<S, 4>,
    surf_g: &NurbSurface3D<S>,
    surf_f: &NurbSurface3D<S>,
    t: S,
    seeds: &mut Seeds<S>,
) -> GeopResult<Option<S>> {
    let point = curve.evaluate(t)?;
    let mut feet = [(S::ZERO, S::ZERO); 2];
    for (k, surface) in [surf_g, surf_f].into_iter().enumerate() {
        let (u, v) = surface.project(point, seeds[k].0, seeds[k].1, NEWTON_ITERATIONS)?;
        feet[k] = (u, v);
        seeds[k] = (u.sharpen(), v.sharpen());
    }
    let [(ug, vg), (uf, vf)] = feet;
    let (ng, nf) = (surf_g.normal(ug, vg)?, surf_f.normal(uf, vf)?);
    let cross = ng.prod_cross(&nf);
    if !(0..3).all(|k| cross[k].could_be_equal(S::ZERO)) {
        return Ok(None);
    }
    let alignment = ng.prod_dot(&nf);
    let h_f = surf_f.mean_curvature(uf, vf)?;
    let h_f = if alignment.definitely_greater(S::ZERO) {
        h_f
    } else if alignment.definitely_less(S::ZERO) {
        h_f.neg()
    } else {
        return Err(GeopError::new(format!(
            "parallel normals {ng:?} and {nf:?} with an undecided orientation"
        )));
    };
    Ok(Some(surf_g.mean_curvature(ug, vg)?.sub(h_f)))
}

/// The sign of `gap`, if it has one.
fn sign<S: Scalar>(gap: &GeopResult<Option<S>>) -> Option<bool> {
    match gap {
        Ok(Some(c)) if c.definitely_greater(S::ZERO) => Some(true),
        Ok(Some(c)) if c.definitely_less(S::ZERO) => Some(false),
        _ => None,
    }
}

/// Every parameter of `curve` — lying on `surf_f`, and bounding a face on
/// `surf_g` — where an intersection branch of the two surfaces leaves it
/// (see the module doc): each a bracket `[a, b]` whose ends have
/// [`curvature_gap`]s of definitely opposite signs, so it holds a branch
/// point by continuity.
///
/// Subdivide to isolate, then bisect to refine. A stretch is dropped where
/// the surfaces cannot be tangent or their gap cannot vanish; what is left
/// at `ISOLATION_LEVELS` is grouped into contiguous runs, and each run whose
/// two ends have opposite signs is bisected, at sharp parameters, until the
/// sign of its midpoint is no longer decided. A run whose ends agree, or
/// cannot be told apart — the surfaces agreeing in curvature along a whole
/// stretch, or touching without crossing — has no branch leaving it.
///
/// The foot points are found once, globally ([`surface_could_contain`] at
/// the curve's midpoint), and every later projection seeds from its parent
/// stretch's, so each stays on the sheet the subdivision started on. A curve
/// whose midpoint is not on `surf_f` has no branch points found.
fn tangent_branch_points<S: Scalar>(
    curve: &NurbCurve<S, 4>,
    surf_g: &NurbSurface3D<S>,
    surf_f: &NurbSurface3D<S>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Vec<S>> {
    let (t0, t1) = curve.domain();
    let mid = curve.evaluate(t0.add(t1).div(S::TWO)?.sharpen())?;
    let (Some(seed_g), Some(seed_f)) = (
        surface_could_contain(surf_g, &mid, max_nodes, min_subdivision_size)?,
        surface_could_contain(surf_f, &mid, max_nodes, min_subdivision_size)?,
    ) else {
        return Ok(vec![]);
    };
    let sharp = |(u, v): (S, S)| (u.sharpen(), v.sharpen());

    let mut live = vec![(t0, t1, [sharp(seed_g), sharp(seed_f)])];
    for level in 0..=ISOLATION_LEVELS {
        let mut next = Vec::new();
        for (lo, hi, mut seeds) in live {
            match curvature_gap(curve, surf_g, surf_f, lo.union(hi), &mut seeds) {
                Ok(None) => continue,
                Ok(Some(c)) if !c.could_be_equal(S::ZERO) => continue,
                _ => {}
            }
            if level == ISOLATION_LEVELS {
                next.push((lo, hi, seeds));
            } else {
                let m = lo.add(hi).div(S::TWO)?.sharpen();
                next.extend([(lo, m, seeds), (m, hi, seeds)]);
            }
        }
        live = next;
    }

    let mut runs: Vec<(S, S, Seeds<S>)> = Vec::new();
    for (lo, hi, seeds) in live {
        match runs.last_mut() {
            Some(run) if run.1.could_be_equal(lo) => run.1 = hi,
            _ => runs.push((lo, hi, seeds)),
        }
    }

    let mut found = Vec::new();
    for (mut a, mut b, mut seeds) in runs {
        let (Some(sign_a), Some(sign_b)) = (
            sign(&curvature_gap(curve, surf_g, surf_f, a, &mut seeds)),
            sign(&curvature_gap(curve, surf_g, surf_f, b, &mut seeds)),
        ) else {
            continue;
        };
        if sign_a == sign_b {
            continue;
        }
        loop {
            let m = a.add(b).div(S::TWO)?.sharpen();
            if !(a.definitely_less(m) && m.definitely_less(b)) {
                break;
            }
            match sign(&curvature_gap(curve, surf_g, surf_f, m, &mut seeds)) {
                Some(s) if s == sign_a => a = m,
                Some(_) => b = m,
                None => break,
            }
        }
        found.push(a.union(b));
    }
    Ok(found)
}

/// The first branch point (see the module doc) on an edge of `edge_solid`
/// lying on a face of `face_solid`, strictly inside the edge: the edge, the
/// parameter to split it at, an existing vertex there if there is one, the
/// point, and the face.
#[allow(clippy::type_complexity)]
fn find_tangent_branch<S: Scalar>(
    model: &Model<S>,
    edge_solid: SolidId,
    face_solid: SolidId,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<(EdgeId, S, Option<VertexId>, Vector3<S>, FaceId)>> {
    for edge_id in model.iter_solid_edges(edge_solid)? {
        let edge = model.get_edge(edge_id)?;
        let bounded: Vec<FaceId> = model
            .coedges_of_edge(edge_id)
            .into_iter()
            .map(|coedge| model.get_coedge(coedge).map(|c| c.face))
            .collect::<GeopResult<_>>()?;
        for face_id in model.solid_faces(face_solid)? {
            let pair_ctx = |e: GeopError| e.with_context(format!("edge={edge_id}, face={face_id}"));
            let surf_f = &model.get_face(face_id)?.surface;
            let on_face = curve_surface_intersect(
                &edge.curve,
                surf_f,
                max_solutions,
                max_nodes,
                min_subdivision_size,
            )
            .with_context(&pair_ctx)?;
            if !matches!(on_face, Intersections::Coincident(_)) {
                continue;
            }
            for &g in &bounded {
                let surf_g = &model.get_face(g)?.surface;
                for t in tangent_branch_points(
                    &edge.curve,
                    surf_g,
                    surf_f,
                    max_nodes,
                    min_subdivision_size,
                )
                .with_context(&|e: GeopError| {
                    pair_ctx(e.with_context(format!("bounding face={g}")))
                })? {
                    let (lo, hi) = edge.curve.domain();
                    if !t.definitely_greater(lo) || !t.definitely_less(hi) {
                        continue;
                    }
                    let point = edge.curve.evaluate(t)?;
                    let ends = [edge.start_vertex, edge.end_vertex];
                    if ends.iter().any(|&v| {
                        model
                            .get_vertex(v)
                            .is_ok_and(|vertex| vertex.point.could_be_equal(&point))
                    }) {
                        continue;
                    }
                    let vertex = model
                        .vertices
                        .iter()
                        .find(|(_, v)| v.point.could_be_equal(&point))
                        .map(|(&id, _)| id);
                    return Ok(Some((edge_id, t, vertex, point, face_id)));
                }
            }
        }
    }
    Ok(None)
}

/// Split every edge of `edge_solid` at every point where an intersection
/// branch of a face it bounds and a face of `face_solid` it lies on leaves
/// it — see the module doc. Runs before vertices are matched against edges,
/// so an edge of `face_solid` running through the new vertex — the other
/// solid's copy of a shared profile — is split there too.
pub fn remesh_tangent_branches<S: Scalar>(
    part: &mut Part<S>,
    naming: &mut BooleanNaming<S>,
    edge_solid: SolidId,
    face_solid: SolidId,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<()> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "remesh_tangent_branches(edge_solid={edge_solid}, face_solid={face_solid}, max_solutions={max_solutions}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size:?})"
        ))
    };
    while let Some((edge_id, t, vertex, point, face_id)) = find_tangent_branch(
        part.topology(),
        edge_solid,
        face_solid,
        max_solutions,
        max_nodes,
        min_subdivision_size,
    )
    .with_context(&ctx)?
    {
        let vertex_id = match vertex {
            Some(vertex) => vertex,
            None => {
                let vertex = part.insert_vertex(point, naming.provisional())?;
                naming.piercing(vertex, edge_id, t, face_id)?;
                vertex
            }
        };
        let new_edge = part
            .split_edge_at_vertex(
                edge_id,
                t,
                vertex_id,
                max_nodes,
                min_subdivision_size,
                naming.provisional(),
            )
            .with_context(&|e: GeopError| {
                e.with_context(format!("edge={edge_id}, t={t:?}, vertex={vertex_id}"))
            })
            .with_context(&ctx)?;
        naming.edge_split(edge_id, new_edge, vertex_id)?;
    }
    Ok(())
}
