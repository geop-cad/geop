//! [`ExtendSurface`]: a face standing on its own carried on past one of its
//! edges.
//!
//! The face's surface is continued past the edge as the polynomial it
//! already is (see [`NurbSurface3D::extended`]) — the natural extension: a
//! plane stays a plane, a cylinder a cylinder, a circle carries on round,
//! a free-form surface goes on curving as it curved. The face is rebuilt
//! on the longer surface, bounded by its four sides.
//!
//! How far: `distance` along the surface, measured across the middle of
//! the edge. Where the surface's parametrization runs evenly across the
//! edge — a plane, a cylinder along its axis, any straight extrusion — the
//! whole edge moves that far; on a free-form surface its ends move as far
//! as the parametrization carries them, more or less.
//!
//! Supported: a face that is its surface's whole patch — bounded by the
//! patch's four sides, as a boundary surface or an extruded face is — and
//! stands alone in its sheet. Anything else is refused, by name.

use geop_core_geometry::{contains::surface::boundary_curve, nurb_surface::NurbSurface3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_topology::{
    Body, CoedgeGeometry, FaceId, Sense,
    boundary::BoundaryType,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use geop_ops::{
    BodyNames, Context, Library, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Form, Number, Unit},
};
use geop_ops_extrude_revolve::common::line2;
use serde::{Deserialize, Serialize};

use crate::{name_of, sheet_of};

/// Carries the face standing on its own that the edge named `edge` bounds
/// on past that edge by `distance` (see the module docs). The face, its
/// edges and its vertices keep their names, moved and stretched as the
/// face grows.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ExtendSurface;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExtendSurfaceArgs {
    /// The edge of a face standing on its own to extend the face past.
    pub edge: String,
    /// How far, along the surface across the middle of the edge.
    pub distance: f64,
}

impl Operation for ExtendSurface {
    type Args = ExtendSurfaceArgs;
    type Session = ();

    /// Nothing picked yet, a tenth on.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> ExtendSurfaceArgs {
        ExtendSurfaceArgs {
            edge: String::new(),
            distance: 0.1,
        }
    }

    /// The edge, picked, and the distance.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &ExtendSurfaceArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ExtendSurfaceArgs> {
        let mut f = Form::<S, ExtendSurfaceArgs>::new();
        let picked: Vec<String> = (!args.edge.is_empty())
            .then(|| args.edge.clone())
            .into_iter()
            .collect();
        f.reference(
            "edge",
            "edge",
            EntityRef::of_names(EntityRef::edge, &picked),
            &[Role::Edge],
            None,
            false,
            |e, picked: Vec<EntityRef>| {
                e.args.edge = EntityRef::names_of(EntityRef::edge, &picked)
                    .pop()
                    .unwrap_or_default()
            },
        );
        f.number(
            "distance",
            Number::new("distance", args.distance, Unit::Length).range(0.0, 1.0),
            |args, d| args.distance = d,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &ExtendSurfaceArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("extend_surface({operation_id}, {args:?})");
        geop_ops::validate_operation_id(operation_id)?;
        if !args.distance.is_finite() {
            return Err(GeopError::new("the distance is not a number")).with_context(ctx);
        }
        extend(&mut part, &args.edge, S::from_f64(args.distance)).with_context(ctx)?;
        Ok(part)
    }
}

/// Which side of its patch a face's coedge runs along, numbered as
/// [`NurbSurface3D::coons`] numbers them: `v` lowest, `u` highest, `v`
/// highest, `u` lowest — the way an outer loop runs round its patch.
fn side_of<S: Scalar>(
    surface: &NurbSurface3D<S>,
    pcurve: &geop_core_topology::Curve2<S>,
) -> GeopResult<Option<usize>> {
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    let (t0, t1) = pcurve.domain();
    let (a, b) = (pcurve.evaluate(t0)?, pcurve.evaluate(t1)?);
    let both = |k: usize, x: S| a[k].could_be_equal(x) && b[k].could_be_equal(x);
    // Straight along the side, not merely ending on it.
    let straight = pcurve.degree == 1 && pcurve.control_points.len() == 2;
    Ok(match () {
        _ if !straight => None,
        _ if both(1, v0) && a[0].could_be_equal(u0) && b[0].could_be_equal(u1) => Some(0),
        _ if both(0, u1) && a[1].could_be_equal(v0) && b[1].could_be_equal(v1) => Some(1),
        _ if both(1, v1) && a[0].could_be_equal(u1) && b[0].could_be_equal(u0) => Some(2),
        _ if both(0, u0) && a[1].could_be_equal(v1) && b[1].could_be_equal(v0) => Some(3),
        _ => None,
    })
}

/// Extends the face standing on its own that the edge `edge` bounds past
/// it by `distance` (see the module docs).
pub fn extend<S: Scalar>(part: &mut Part<S>, edge: &str, distance: S) -> GeopResult<FaceId> {
    let ctx = with_context!("extend({edge}, {distance:?})");
    if !distance.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "an extension needs a distance greater than zero, not {distance:?}"
        )))
        .with_context(ctx);
    }
    let model = part.topology();
    let edge_id = part.edge_id(edge).with_context(ctx)?;
    let coedges = model.coedges_of_edge(edge_id);
    let [coedge] = coedges.as_slice() else {
        return Err(GeopError::new(format!(
            "edge {edge} bounds {} faces: extending is past the free edge of a face standing on its own",
            coedges.len()
        )))
        .with_context(ctx);
    };
    let face_id = model.get_coedge(*coedge)?.face;
    let face_name = name_of(part, face_id)?;
    let (_, sheet) = sheet_of(part, &face_name).with_context(ctx)?;
    if model.body_faces(Body::Sheet(sheet))?.len() != 1 {
        return Err(GeopError::new(format!(
            "face {face_name} shares edges with other faces of its sheet: extending a face standing alone is supported"
        )))
        .with_context(ctx);
    }
    let face = model.get_face(face_id)?;
    let not_whole = || {
        GeopError::new(format!(
            "face {face_name} is not bounded by its surface's four sides: extending such a face is not supported"
        ))
    };
    let BoundaryType::Loop(anchor) = face.outer else {
        return Err(not_whole()).with_context(ctx);
    };
    if !face.holes.is_empty() {
        return Err(not_whole()).with_context(ctx);
    }
    // Per side of the patch, its coedge's edge, and per corner — where a
    // side starts — its vertex.
    let mut edges = [None; 4];
    let mut corners = [None; 4];
    for c in model.iterate_loop_coedges(anchor) {
        let coedge = model.get_coedge(c)?;
        let CoedgeGeometry::Edge(e) = coedge.geometry else {
            return Err(not_whole()).with_context(ctx);
        };
        let Some(k) = side_of(&face.surface, &coedge.pcurve)? else {
            let (t0, t1) = coedge.pcurve.domain();
            return Err(not_whole().with_context(format!(
                "edge {}: its pcurve of degree {} runs from {:?} to {:?}, on none of the sides of the domain {:?} x {:?}",
                name_of(part, e)?,
                coedge.pcurve.degree,
                coedge.pcurve.evaluate(t0)?,
                coedge.pcurve.evaluate(t1)?,
                face.surface.domain_u(),
                face.surface.domain_v()
            )))
            .with_context(ctx);
        };
        if edges[k].is_some() {
            return Err(not_whole()).with_context(ctx);
        }
        edges[k] = Some(e);
        corners[k] = Some(model.coedge_start_vertex_id(c)?);
    }
    let (Some(edges), Some(corners)) = (
        edges.into_iter().collect::<Option<Vec<_>>>(),
        corners.into_iter().collect::<Option<Vec<_>>>(),
    ) else {
        return Err(not_whole()).with_context(ctx);
    };
    let side = edges
        .iter()
        .position(|&e| e == edge_id)
        .expect("the edge bounds the face");

    // How far in the parameter across the edge: the distance over how fast
    // the surface runs across it at its middle — sharpened, a free choice
    // of how much to add.
    let surface = &face.surface;
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    let half = |a: S, b: S| a.add(b).div(S::TWO);
    let (along_u, at_end, (u, v)) = match side {
        0 => (false, false, (half(u0, u1)?, v0)),
        1 => (true, true, (u1, half(v0, v1)?)),
        2 => (false, true, (half(u0, u1)?, v1)),
        _ => (true, false, (u0, half(v0, v1)?)),
    };
    let (du, dv) = surface.derivatives(u.sharpen(), v.sharpen())?;
    let speed = if along_u { du } else { dv }.norm();
    let by = distance.div(speed)?.sharpen();
    let longer = surface.extended(along_u, at_end, by).map_err(|e| {
        e.with_context(format!(
            "carrying the surface of face {face_name} on past edge {edge}"
        ))
    })?;

    // The face anew: the whole longer patch, its sides its border curves,
    // each keeping the name of the edge it continues.
    let ((u0, u1), (v0, v1)) = (longer.domain_u(), longer.domain_v());
    let border = |u_fixed: bool, first: bool| {
        boundary_curve(&longer, u_fixed, first)
            .ok_or_else(|| GeopError::new("the extended surface is not clamped at its border"))
    };
    // Sides 0..4, as the loop runs: from corner `k` to corner `k + 1`. The
    // curves along `u` run with sides 0 and 2's `u`, along `v` with 1 and
    // 3's `v`: 0 and 1 forward, 2 and 3 backward.
    let curves = [
        border(false, true)?,
        border(true, false)?,
        border(false, false)?,
        border(true, true)?,
    ];
    let corner_points = [
        longer.evaluate(u0, v0)?,
        longer.evaluate(u1, v0)?,
        longer.evaluate(u1, v1)?,
        longer.evaluate(u0, v1)?,
    ];
    let uv = |u: S, v: S| Vector2::from_array([u, v]);
    let corner_uv = [uv(u0, v0), uv(u1, v0), uv(u1, v1), uv(u0, v1)];
    let spec = BodySpec {
        vertices: corner_points.to_vec(),
        edges: (0..4)
            .map(|k| {
                let forward = k < 2;
                EdgeSpec {
                    curve: curves[k].clone(),
                    start: if forward { k } else { (k + 1) % 4 },
                    end: if forward { (k + 1) % 4 } else { k },
                }
            })
            .collect(),
        faces: vec![FaceSpec {
            surface: longer.clone(),
            outer: (0..4)
                .map(|k| {
                    Ok(CoedgeSpec {
                        on: CoedgeOn::Edge(
                            k,
                            if k < 2 {
                                Sense::Forward
                            } else {
                                Sense::Reversed
                            },
                        ),
                        pcurve: line2(corner_uv[k], corner_uv[(k + 1) % 4])?,
                    })
                })
                .collect::<GeopResult<_>>()?,
            holes: Vec::new(),
        }],
        shells: vec![vec![0]],
        solid: false,
    };
    let names = BodyNames {
        vertices: corners
            .iter()
            .map(|&v| name_of(part, v))
            .collect::<GeopResult<_>>()?,
        edges: edges
            .iter()
            .map(|&e| name_of(part, e))
            .collect::<GeopResult<_>>()?,
        faces: vec![face_name],
        solid: None,
    };
    part.assemble_sheet(&[Body::Sheet(sheet)], &[])
        .with_context(ctx)?;
    let built = part.build_body(spec, names).with_context(ctx)?;
    Ok(built.faces[0])
}

#[cfg(test)]
mod tests;
