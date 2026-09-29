//! A verbose, id-labelled debug rasterization of a [`Model`]'s raw topology
//! — as opposed to [`crate::rasterize`], which only cares about the final
//! geometric shape. Draws every vertex, edge (with `0.1..0.9`
//! direction-arrow markers, plus one at its midpoint), coedge (as a trim
//! curve pulled slightly inward, mitered back at each end so neighboring
//! coedges' trim curves don't overlap at the shared corner, with its own
//! `0.1..0.9` direction-arrow markers and one at its midpoint) and face
//! (semi-transparent, so everything behind it stays visible, plus a normal
//! arrow), each labelled with its id — useful for debugging the euler
//! operators themselves, where *which* coedge/edge is which matters.
//!
//! Coedges are drawn in two colours: orange (with cyan direction markers) for
//! those on a face's outer loop, purple (with olive markers) for those on a
//! hole. Which of the two a loop is drives every restructuring in
//! `splice_edge_into_face` and the Euler operators, and is invisible from
//! geometry alone — an outer loop and a hole are both just rings.

use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::TriangleFace,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::{Coedge, CoedgeId, Model, boundary::BoundaryType};

use super::{Color10, Line, PrimitiveScene};

/// How far inward (in 3-D world units) a coedge's trim curve and its
/// `0.1..0.9` markers are pulled off of the true boundary curve.
const COEDGE_INSET: f64 = 0.02;

/// Fraction of an edge's/coedge's own chord length used for its direction
/// arrow's "wingspan".
const ARROW_SIZE_FRACTION: f64 = 0.15 / 12.0;

/// World-space length of each face's normal arrow.
const FACE_NORMAL_LENGTH: f64 = 0.1;

/// Never miter-trim more than this fraction of a coedge's own parameter
/// range off of *each* end — so a very sharp corner shortens the visible
/// trim curve a lot without ever inverting it.
const MAX_TRIM_FRACTION: f64 = 0.45;

const VERTEX_COLOR: Color10 = Color10::Red;
const EDGE_COLOR: Color10 = Color10::Gray;
const EDGE_MARKER_COLOR: Color10 = Color10::Pink;
/// Coedges are coloured by which kind of boundary they belong to, since that
/// is the distinction the face topology turns on and the one that is
/// otherwise invisible in a render: a loop that bounds the material looks
/// exactly like one that removes from it.
const COEDGE_OUTER_COLOR: Color10 = Color10::Orange;
const COEDGE_OUTER_MARKER_COLOR: Color10 = Color10::Cyan;
const COEDGE_HOLE_COLOR: Color10 = Color10::Purple;
const COEDGE_HOLE_MARKER_COLOR: Color10 = Color10::Olive;
const FACE_COLOR: Color10 = Color10::Blue;
const FACE_OPACITY: f64 = 0.35;
const FACE_NORMAL_COLOR: Color10 = Color10::Green;

/// Radius of the world coordinate system's axis cylinders.
const AXIS_RADIUS: f64 = 0.01 / 3.0;
/// Spacing, along each axis, of the small perpendicular step ticks.
const AXIS_STEP: f64 = 0.1;
/// Length of each step tick (centered on the axis).
const AXIS_STEP_TICK_LENGTH: f64 = 0.03;
const AXIS_X_COLOR: Color10 = Color10::Red;
const AXIS_Y_COLOR: Color10 = Color10::Green;
const AXIS_Z_COLOR: Color10 = Color10::Blue;

/// `coedge`'s raw (non-normalized — its length is the local parameter
/// speed `|dP/dt|`) 3-D tangent at `t`, via the chain rule through the
/// pcurve's own 2-D tangent and the surface's partial derivatives.
fn coedge_tangent_3d<S: Scalar>(
    coedge: &Coedge<S>,
    surface: &NurbSurface3D<S>,
    t: S,
) -> GeopResult<Vector3<S>> {
    let ctx = |e: GeopError| {
        let (t0, t1) = coedge.pcurve.domain();
        e.with_context(format!(
            "coedge_tangent_3d(t={t:?}): pcurve domain=({t0:?}, {t1:?}), pcurve degree={}, pcurve knot_vector={:?}, pcurve control_points={:?}",
            coedge.pcurve.degree, coedge.pcurve.knot_vector, coedge.pcurve.control_points
        ))
    };

    let uv = coedge.pcurve.evaluate(t).with_context(&ctx)?;
    let (ds_du, ds_dv) = surface.derivatives(uv[0], uv[1]).with_context(&ctx)?;
    let d_uv = coedge.pcurve.tangent(t).with_context(&ctx)?;
    Ok(ds_du.prod_scalar(d_uv[0]).add(&ds_dv.prod_scalar(d_uv[1])))
}

/// The 3-D point `coedge`'s pcurve reaches at `t`, pulled `inset` inward
/// (toward the face's interior) along `face_normal × tangent` — or, for a
/// degenerate coedge (e.g. a revolve pole's own zero-length self-loop,
/// whose tangent has no well-defined direction to inset along), just the
/// raw, un-inset point.
fn coedge_inset_point<S: Scalar>(
    coedge: &Coedge<S>,
    surface: &NurbSurface3D<S>,
    t: S,
    inset: S,
) -> GeopResult<Vector3<S>> {
    let uv = coedge.pcurve.evaluate(t)?;
    let point = surface.evaluate(uv[0], uv[1])?;
    let Ok(tangent) = coedge_tangent_3d(coedge, surface, t)?.normalize() else {
        return Ok(point);
    };
    let Ok(normal) = surface.normal(uv[0], uv[1]) else {
        return Ok(point);
    };
    let Ok(offset) = normal.prod_cross(&tangent).normalize() else {
        return Ok(point);
    };
    Ok(point.add(&offset.prod_scalar(inset)))
}

/// `coedge`'s unit tangent at `t`, or `None` for a degenerate coedge whose
/// tangent has no well-defined direction (e.g. a revolve pole's own
/// zero-length self-loop).
fn try_tangent<S: Scalar>(
    coedge: &Coedge<S>,
    surface: &NurbSurface3D<S>,
    t: S,
) -> GeopResult<Option<Vector3<S>>> {
    Ok(coedge_tangent_3d(coedge, surface, t)?.normalize().ok())
}

/// How far to pull `t_this` (one end of `coedge`'s own parameter range) back
/// from the shared corner vertex, so its inset trim curve meets `neighbor`'s
/// (the coedge on the other side of that corner, sharing `coedge`'s loop via
/// `next`/`prev`) roughly at the corner's angle bisector instead of
/// overlapping it — the same "miter join" setback used for offset strokes:
/// `inset / tan(interior_angle / 2)` along the curve, converted from arc
/// length to a parameter delta via `coedge`'s own local speed at `t_this`.
/// `away_this`/`away_neighbor` are each curve's unit tangent pointing *away*
/// from the shared vertex (so `interior_angle` is the angle between them).
/// Returns `0` (no trim) if any of the geometry needed is degenerate.
#[allow(clippy::too_many_arguments)]
fn miter_trim_delta_t<S: Scalar>(
    coedge: &Coedge<S>,
    surface: &NurbSurface3D<S>,
    t_this: S,
    away_this: Vector3<S>,
    neighbor: &Coedge<S>,
    neighbor_surface: &NurbSurface3D<S>,
    t_neighbor: S,
    away_neighbor: Vector3<S>,
    inset: S,
) -> GeopResult<S> {
    let speed = coedge_tangent_3d(coedge, surface, t_this)?.norm();
    if speed.could_be_equal(S::ZERO) {
        return Ok(S::ZERO);
    }
    // Only used to keep `neighbor`/`neighbor_surface` meaningfully paired
    // with `t_neighbor`/`away_neighbor` in the caller — the angle itself
    // only needs the two (already-computed) "away" directions.
    let _ = (neighbor, neighbor_surface, t_neighbor);

    let cos_theta = away_this.prod_dot(&away_neighbor).to_f64().clamp(-1.0, 1.0);
    let theta = cos_theta.acos();
    let half_tan = (theta / 2.0).tan();
    if half_tan.abs() < 1e-6 {
        // Degenerate (near-0 interior angle, i.e. a hairpin turn) — trim
        // heavily; the caller's `MAX_TRIM_FRACTION` clamp keeps this sane.
        return Ok(S::from_f64(f64::MAX));
    }
    let setback = S::from_f64(inset.to_f64() / half_tan);
    setback.div(speed)
}

/// Draw a small 2-segment chevron at `tip`, pointing along (unit) `dir`,
/// with `size` "wingspan" — an arrow marking a curve's traversal direction.
fn add_direction_arrow<S: Scalar>(
    scene: &mut PrimitiveScene<S>,
    tip: Vector3<S>,
    dir: Vector3<S>,
    size: S,
    color: Color10,
) -> GeopResult<()> {
    let up = Vector3::from_array([S::ZERO, S::ZERO, S::ONE]);
    let raw = dir.prod_cross(&up);
    let perp = if raw.norm_sq().could_be_equal(S::ZERO) {
        dir.prod_cross(&Vector3::from_array([S::ONE, S::ZERO, S::ZERO]))
            .normalize()?
    } else {
        raw.normalize()?
    };

    let half = S::from_f64(0.5);
    let back = tip.sub(&dir.prod_scalar(size));
    let left = back.add(&perp.prod_scalar(size.mul(half)));
    let right = back.sub(&perp.prod_scalar(size.mul(half)));

    if let Ok(l) = Line::try_new(left, tip) {
        scene.add_line(l, color);
    }
    if let Ok(l) = Line::try_new(right, tip) {
        scene.add_line(l, color);
    }
    Ok(())
}

/// Draw a full arrow (shaft + head) from `base` along (unit) `dir`, `length`
/// long — unlike [`add_direction_arrow`], which only draws the chevron head
/// (meant to sit on top of an already-drawn curve serving as its shaft).
fn add_arrow<S: Scalar>(
    scene: &mut PrimitiveScene<S>,
    base: Vector3<S>,
    dir: Vector3<S>,
    length: S,
    color: Color10,
) -> GeopResult<()> {
    let tip = base.add(&dir.prod_scalar(length));
    if let Ok(l) = Line::try_new(base, tip) {
        scene.add_line(l, color);
    }
    add_direction_arrow(
        scene,
        tip,
        dir,
        length.mul(S::from_f64(ARROW_SIZE_FRACTION)),
        color,
    )
}

/// Draw a world coordinate system at the origin — X/Y/Z axes as solid
/// `AXIS_RADIUS`-wide cylinders (red/green/blue), long enough to cover
/// `model`'s own extent, each with a small perpendicular tick every
/// `AXIS_STEP` units so distances are easy to read off at a glance.
fn add_coordinate_system<S: Scalar>(
    scene: &mut PrimitiveScene<S>,
    model: &Model<S>,
) -> GeopResult<()> {
    let axis_length = model
        .vertices
        .values()
        .flat_map(|v| {
            [
                v.point[0].to_f64().abs(),
                v.point[1].to_f64().abs(),
                v.point[2].to_f64().abs(),
            ]
        })
        .fold(1.0_f64, f64::max);

    let origin = Vector3::from_array([S::ZERO, S::ZERO, S::ZERO]);
    // Each axis, paired with the direction its own step ticks point along
    // (one of the other two axes, picked arbitrarily but consistently).
    let axes: [(Vector3<S>, Vector3<S>, Color10); 3] = [
        (
            Vector3::from_array([S::from_f64(axis_length), S::ZERO, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
            AXIS_X_COLOR,
        ),
        (
            Vector3::from_array([S::ZERO, S::from_f64(axis_length), S::ZERO]),
            Vector3::from_array([S::ONE, S::ZERO, S::ZERO]),
            AXIS_Y_COLOR,
        ),
        (
            Vector3::from_array([S::ZERO, S::ZERO, S::from_f64(axis_length)]),
            Vector3::from_array([S::ZERO, S::ONE, S::ZERO]),
            AXIS_Z_COLOR,
        ),
    ];

    for (end, tick_dir, color) in axes {
        scene.add_cylinder(origin, end, AXIS_RADIUS, color);

        let dir = end.normalize()?;
        let half_tick = S::from_f64(AXIS_STEP_TICK_LENGTH / 2.0);
        let steps = (axis_length / AXIS_STEP).floor() as usize;
        for step in 1..=steps {
            let center = dir.prod_scalar(S::from_f64(step as f64 * AXIS_STEP));
            let tick_start = center.sub(&tick_dir.prod_scalar(half_tick));
            let tick_end = center.add(&tick_dir.prod_scalar(half_tick));
            scene.add_cylinder(tick_start, tick_end, AXIS_RADIUS, color);
        }
    }
    Ok(())
}

/// Rasterize `model`'s raw topology (as opposed to just its final shape —
/// see [`crate::rasterize`]): every vertex, edge, coedge and face,
/// each labelled with its id, `n` samples per curve/coedge.
pub fn rasterize_topology<S: Scalar>(model: &Model<S>, n: usize) -> GeopResult<PrimitiveScene<S>> {
    rasterize_topology_inner(model, n)
        .with_context(&|e: GeopError| e.with_context(format!("rasterize_topology(n={n})")))
}

fn rasterize_topology_inner<S: Scalar>(
    model: &Model<S>,
    n: usize,
) -> GeopResult<PrimitiveScene<S>> {
    let mut scene = PrimitiveScene::new();

    add_coordinate_system(&mut scene, model)?;

    // ── Vertices ──────────────────────────────────────────────────────────
    for (&id, vertex) in &model.vertices {
        scene.add_point(vertex.point, VERTEX_COLOR);
        scene.add_label(vertex.point, format!("V{}", id.0), VERTEX_COLOR);
    }

    // ── Edges (curve + 0.1..0.9 markers + direction arrow + label) ───────
    for (&id, edge) in &model.edges {
        let edge_ctx = |e: GeopError| {
            let (t0, t1) = edge.curve.domain();
            e.with_context(format!(
                "rasterize_topology: edge {id}, domain=({t0:?}, {t1:?}), degree={}, knot_vector={:?}",
                edge.curve.degree, edge.curve.knot_vector
            ))
        };

        let (t0, t1) = edge.curve.domain();
        scene
            .add_curve(&edge.curve, t0, t1, EDGE_COLOR, n)
            .with_context(&edge_ctx)?;

        let length = edge
            .curve
            .evaluate(t1)
            .with_context(&edge_ctx)?
            .sub(&edge.curve.evaluate(t0).with_context(&edge_ctx)?)
            .norm();
        let marker_size = length.mul(S::from_f64(ARROW_SIZE_FRACTION));
        for tenth in 1..10 {
            let frac = S::from_f64(tenth as f64 / 10.0);
            let t = t0.add(t1.sub(t0).mul(frac));
            let tenth_ctx =
                |e: GeopError| e.with_context(format!("tenth={tenth}, frac={frac:?}, t={t:?}"));
            let p = edge
                .curve
                .evaluate(t)
                .with_context(&edge_ctx)
                .with_context(&tenth_ctx)?;
            if let Ok(dir) = edge
                .curve
                .tangent(t)
                .with_context(&edge_ctx)
                .with_context(&tenth_ctx)?
                .normalize()
            {
                add_direction_arrow(&mut scene, p, dir, marker_size, EDGE_MARKER_COLOR)?;
            }
        }

        let mid = t0.add(t1.sub(t0).mul(S::from_f64(0.5)));
        let mid_point = edge.curve.evaluate(mid).with_context(&edge_ctx)?;
        if let Ok(dir) = edge.curve.tangent(mid).with_context(&edge_ctx)?.normalize() {
            add_direction_arrow(&mut scene, mid_point, dir, marker_size, EDGE_COLOR)?;
        }
        scene.add_label(mid_point, format!("E{}", id.0), EDGE_COLOR);
    }

    // Which coedges sit on a *hole* rather than on a face's outer loop,
    // gathered up front: asking per coedge would re-walk its whole ring each
    // time. Each ring walk is capped, because this renderer is deliberately
    // pointed at models that may be broken — the sweep renders scenes whose
    // remesh failed — and a corrupted `next` chain must not hang the render.
    let mut hole_coedges: std::collections::HashSet<CoedgeId> = std::collections::HashSet::new();
    let cap = model.coedges.len() + 1;
    for face in model.faces.values() {
        for hole in &face.holes {
            if let BoundaryType::Loop(anchor) = hole {
                hole_coedges.extend(model.iterate_loop_coedges(*anchor).take(cap));
            }
        }
    }

    // ── Coedges (mitered inset trim curve + 0.1..0.9 markers + direction
    // arrow + label) ──────────────────────────────────────────────────────
    for (&id, coedge) in &model.coedges {
        let Some(face) = model.faces.get(&coedge.face) else {
            continue;
        };
        let surface = &face.surface;

        let (coedge_color, marker_color) = if hole_coedges.contains(&id) {
            (COEDGE_HOLE_COLOR, COEDGE_HOLE_MARKER_COLOR)
        } else {
            (COEDGE_OUTER_COLOR, COEDGE_OUTER_MARKER_COLOR)
        };

        (|| -> GeopResult<()> {
            let (t0, t1) = coedge.pcurve.domain();
            let inset = S::from_f64(COEDGE_INSET);
            let max_trim = t1.sub(t0).mul(S::from_f64(MAX_TRIM_FRACTION));

            // Trim each end back toward the shared corner's angle bisector so
            // this coedge's trim curve doesn't run past it and overlap the
            // neighboring coedge's own trim curve (see `miter_trim_delta_t`).
            let (t0_trim, t1_trim) = {
                let neg_one = S::from_f64(-1.0);
                let prev = model.get_coedge(coedge.prev)?;
                let away_this_start = try_tangent(coedge, surface, t0)?;
                let away_prev = try_tangent(prev, surface, prev.pcurve.domain().1)?
                    .map(|t| t.prod_scalar(neg_one));
                let delta_start = match (away_this_start, away_prev) {
                    (Some(away_this_start), Some(away_prev)) => miter_trim_delta_t(
                        coedge,
                        surface,
                        t0,
                        away_this_start,
                        prev,
                        surface,
                        prev.pcurve.domain().1,
                        away_prev,
                        inset,
                    )?,
                    // Degenerate coedge on one side of the corner (no
                    // well-defined tangent) — nothing to miter against.
                    _ => S::ZERO,
                };

                let next = model.get_coedge(coedge.next)?;
                let away_this_end =
                    try_tangent(coedge, surface, t1)?.map(|t| t.prod_scalar(neg_one));
                let away_next = try_tangent(next, surface, next.pcurve.domain().0)?;
                let delta_end = match (away_this_end, away_next) {
                    (Some(away_this_end), Some(away_next)) => miter_trim_delta_t(
                        coedge,
                        surface,
                        t1,
                        away_this_end,
                        next,
                        surface,
                        next.pcurve.domain().0,
                        away_next,
                        inset,
                    )?,
                    _ => S::ZERO,
                };

                let delta_start = if delta_start.definitely_greater(max_trim) {
                    max_trim
                } else {
                    delta_start
                };
                let delta_end = if delta_end.definitely_greater(max_trim) {
                    max_trim
                } else {
                    delta_end
                };
                (t0.add(delta_start), t1.sub(delta_end))
            };

            let mut trim_points = Vec::with_capacity(n);
            for i in 0..n {
                let frac = S::from_ratio(i as i64, (n - 1) as i64)?;
                let t = t0_trim.add(t1_trim.sub(t0_trim).mul(frac));
                trim_points.push(coedge_inset_point(coedge, surface, t, inset)?);
            }
            scene.add_polyline(&trim_points, coedge_color);

            let length = trim_points
                .last()
                .unwrap()
                .sub(trim_points.first().unwrap())
                .norm();
            let marker_size = length.mul(S::from_f64(ARROW_SIZE_FRACTION));
            for tenth in 1..10 {
                let frac = S::from_f64(tenth as f64 / 10.0);
                let t = t0.add(t1.sub(t0).mul(frac));
                let p = coedge_inset_point(coedge, surface, t, inset)?;
                if let Ok(dir) = coedge_tangent_3d(coedge, surface, t)?.normalize() {
                    add_direction_arrow(&mut scene, p, dir, marker_size, marker_color)?;
                }
            }

            let mid_t = t0_trim.add(t1_trim.sub(t0_trim).mul(S::from_f64(0.5)));
            let mid_point = coedge_inset_point(coedge, surface, mid_t, inset)?;
            if let Ok(dir) = coedge_tangent_3d(coedge, surface, mid_t)?.normalize() {
                add_direction_arrow(&mut scene, mid_point, dir, marker_size, coedge_color)?;
            }
            scene.add_label(mid_point, format!("C{}", id.0), coedge_color);
            Ok(())
        })()
        .with_context(&|e: GeopError| {
            let (t0, t1) = coedge.pcurve.domain();
            e.with_context(format!(
                "rasterize_topology: coedge {id}, pcurve domain=({t0:?}, {t1:?}), pcurve degree={}, pcurve knot_vector={:?}, pcurve control_points={:?}",
                coedge.pcurve.degree, coedge.pcurve.knot_vector, coedge.pcurve.control_points
            ))
        })?;
    }

    // ── Faces (transparent fill + label) ─────────────────────────────────
    for (&id, face) in &model.faces {
        let has_loop = face
            .boundaries()
            .any(|b| matches!(b, geop_core_topology::boundary::BoundaryType::Loop(_)));
        if !has_loop {
            continue;
        }

        for (uv_a, uv_b, uv_c) in crate::face_triangles_uv(model, face, n)? {
            let a = face.surface.evaluate(uv_a[0], uv_a[1])?;
            let b = face.surface.evaluate(uv_b[0], uv_b[1])?;
            let c = face.surface.evaluate(uv_c[0], uv_c[1])?;
            if let Ok(t) = TriangleFace::try_new(a, b, c) {
                scene.add_triangle_transparent(t, FACE_COLOR, FACE_OPACITY);
            }
        }

        let (u0, u1) = face.surface.domain_u();
        let (v0, v1) = face.surface.domain_v();
        let mid_u = u0.add(u1.sub(u0).mul(S::from_f64(0.5)));
        let mid_v = v0.add(v1.sub(v0).mul(S::from_f64(0.5)));
        if let Ok(label_point) = face.surface.evaluate(mid_u, mid_v) {
            scene.add_label(label_point, format!("F{}", id.0), FACE_COLOR);
            if let Ok(normal) = face.surface.normal(mid_u, mid_v) {
                add_arrow(
                    &mut scene,
                    label_point,
                    normal,
                    S::from_f64(FACE_NORMAL_LENGTH),
                    FACE_NORMAL_COLOR,
                )?;
            }
        }
    }

    Ok(scene)
}
