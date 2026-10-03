//! [`Revolve`]: sweep a sketch's regions a full turn around a line in its
//! plane.

use std::collections::BTreeSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_sketch::{CurveKind, Enclosure, PointId, ProfileLoop, profile::curve_polyline};
use geop_ops::{
    Context, Library, Namer, Part, PlacedSketch,
    operation::{Aspects, EntityRef, Operation, Role},
    ui::Form,
};
use geop_ops_booleans::Combine;
use serde::{Deserialize, Serialize};

use super::{extrude::sketch_profile, sketch_field};
use crate::revolve::revolve_at_oriented;

/// Revolves every region of a sketch a full turn around a line in its
/// plane, into one solid named `revolve(R)` for the operation `R` — or,
/// with [`RevolveArgs::combine`], combines that with another solid (see
/// [`Combine`]), the result named `revolve(R)` all the same.
///
/// The axis is any line (see [`Role::Line`]) in the sketch's plane: a line
/// of the sketch, of another one, a datum axis, a frame's axis, a straight
/// edge. Every region lies entirely on one side of it, the same side for
/// every region, and either
///
/// - touches it along an edge — the constraints put the edge's endpoints on
///   the axis, which must then be a line of the sketch itself (see
///   `Sketch::on_line`), typically the axis line itself — and the rest of
///   its boundary is an open profile from the axis back to it, sweeping a
///   solid around it; or
/// - stays clear of it, and its whole boundary sweeps a ring.
///
/// The profile names what it sweeps (see [`revolve_at_oriented`]): with `X`
/// a piece of a sketch curve and `P` a joint of the sketch `K`, as for
/// [`super::Extrude`],
///
/// - `revolve(R,K,X,q0)` .. `q3`: the face `X` sweeps through each quarter
///   turn, starting from the sketch plane;
/// - `revolve(R,K,X,a0)` .. `a3`: `X` itself at each quarter angle (`a0` is
///   the profile where it was drawn);
/// - `revolve(R,K,P,q0)` .. / `revolve(R,K,P,a0)` ..: the circular edges and
///   vertices `P` sweeps, and `revolve(R,K,P)` for a `P` on the axis.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Revolve;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RevolveArgs {
    /// The sketch to revolve.
    pub sketch: String,
    /// The line to revolve around, in the sketch's plane; none yet, for a
    /// step that has not picked one.
    pub axis: Option<EntityRef>,
    /// Keep the solid as a new body, or combine it with another solid.
    #[serde(default)]
    pub combine: Combine,
}

/// The line of `sketch` to revolve around by default: its first
/// construction line — what an axis is usually drawn as — else its first
/// line; none, for a sketch with no line.
fn default_axis<S: Scalar>(part: &Part<S>, sketch: &str) -> Option<EntityRef> {
    let placed = part.sketch_id(sketch).and_then(|id| part.sketch(id)).ok()?;
    let lines: Vec<_> = placed
        .sketch
        .curves
        .iter()
        .filter(|(_, c)| matches!(c.kind, CurveKind::Line { .. }))
        .collect();
    let (curve, _) = lines
        .iter()
        .find(|(_, c)| c.construction)
        .or(lines.first())?;
    Some(EntityRef::SketchCurve {
        sketch: sketch.into(),
        curve: **curve,
    })
}

/// The axis in the sketch's own coordinates.
struct SketchAxis<S: Scalar> {
    /// A point on it, and its unit direction, enclosed.
    point: Vector2<S>,
    direction: Vector2<S>,
    /// A point on it and its direction (of any length), as drawn: which side
    /// of it a region lies on is a question about the sketch as drawn.
    drawn: (Vector2<S>, Vector2<S>),
    /// The points of the sketch the constraints put on it — none, unless it
    /// is a line of the sketch.
    on_axis: BTreeSet<PointId>,
}

impl<S: Scalar> SketchAxis<S> {
    /// `axis` in the coordinates of `placed`, the sketch `name` of `part`,
    /// whose solution is `geometry`. A line of the sketch itself is its own
    /// line; any other line must lie in the sketch's plane, and is used as
    /// it lies there.
    fn of(
        axis: &EntityRef,
        part: &Part<S>,
        name: &str,
        placed: &PlacedSketch<S>,
        geometry: &Enclosure<S>,
    ) -> GeopResult<Self> {
        let ctx = with_context!("revolving around {axis}");
        let unit = |d: Vector2<S>| d.normalize().with_context(ctx);
        if let EntityRef::SketchCurve { sketch, curve } = axis
            && sketch == name
        {
            let sketch = &placed.sketch;
            let CurveKind::Line { start, end } = sketch.curve(*curve).with_context(ctx)?.kind
            else {
                return Err(GeopError::new(format!("{axis} is not a line"))).with_context(ctx);
            };
            let drawn = |p: PointId| sketch.points[&p].xy().map(|c| c.cast::<S>());
            let point = geometry.points[&start];
            return Ok(SketchAxis {
                point,
                direction: unit(geometry.points[&end].sub(&point))?,
                drawn: (drawn(start), drawn(end).sub(&drawn(start))),
                on_axis: sketch.on_line(*curve).with_context(ctx)?,
            });
        }
        let line = Aspects::of(axis, part)
            .with_context(ctx)?
            .line
            .ok_or_else(|| GeopError::new(format!("{axis} is not a line")))
            .with_context(ctx)?;
        let plane = &placed.plane;
        let point = plane.to_uvw(&line.point);
        let along = |axis: &geop_core_math::vector::Vector3<S>| line.direction.prod_dot(axis);
        let direction = [along(plane.u()), along(plane.v()), along(plane.w())];
        if !point[2].could_be_equal(S::ZERO) || !direction[2].could_be_equal(S::ZERO) {
            return Err(GeopError::new(format!(
                "{axis} does not lie in the plane of sketch {name:?}"
            )))
            .with_context(ctx);
        }
        let point = Vector2::from_array([point[0], point[1]]);
        let direction = unit(Vector2::from_array([direction[0], direction[1]]))?;
        Ok(SketchAxis {
            drawn: (point, direction),
            point,
            direction,
            on_axis: BTreeSet::new(),
        })
    }
}

impl Operation for Revolve {
    type Args = RevolveArgs;
    type Session = ();

    /// The newest sketch around its axis line, joined to the newest solid
    /// if there is one.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> RevolveArgs {
        let sketch = before.sketch_names().pop().unwrap_or_default();
        RevolveArgs {
            axis: default_axis(before, &sketch),
            sketch,
            combine: Combine::new_for(before),
        }
    }

    /// The sketch and the axis, picked. Another sketch picked brings an
    /// axis that was a line of the old one back to the new one's default.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &RevolveArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, RevolveArgs> {
        let before = context.before;
        let mut f = Form::<S, RevolveArgs>::new();
        sketch_field(&mut f, before, &args.sketch, move |args, sketch| {
            let old = std::mem::replace(&mut args.sketch, sketch);
            let of_old = match &args.axis {
                None => true,
                Some(EntityRef::SketchCurve { sketch, .. }) => *sketch == old,
                Some(_) => false,
            };
            if of_old {
                args.axis = default_axis(before, &args.sketch);
            }
        });
        f.reference(
            "axis",
            "axis",
            args.axis.iter().cloned().collect(),
            &[Role::Line],
            None,
            false,
            |edit, picked| edit.args.axis = picked.into_iter().next(),
        );
        args.combine.show(&mut f, before, |args| &mut args.combine);
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &RevolveArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("revolve({operation_id}, {args:?})");
        let namer = Namer::new("revolve", operation_id)?;
        let placed = part
            .sketch(part.sketch_id(&args.sketch).with_context(ctx)?)?
            .clone();
        let sketch = &placed.sketch;
        let Some(axis) = &args.axis else {
            return Err(GeopError::new("pick a line to revolve around")).with_context(ctx);
        };
        let geometry = sketch.enclose::<S>().with_context(ctx)?;
        let axis = SketchAxis::of(axis, &part, &args.sketch, &placed, &geometry)?;
        let (a, dir) = axis.drawn;
        let axis_left = Vector2::from_array([axis.direction[1].neg(), axis.direction[0]]);
        let on_axis = &axis.on_axis;

        let regions = sketch.regions().with_context(ctx)?;
        let mut sides = Vec::new();
        let mut solid = None;
        for (index, region) in regions.iter().enumerate() {
            let ctx = with_context!("revolving sketch region {index}");
            if !region.holes.is_empty() {
                return Err(GeopError::new(
                    "revolving a region with holes is not supported yet",
                ))
                .with_context(ctx);
            }
            // Which side of the axis the region lies on, from its outline as
            // drawn; it must not cross. The points the constraints put on the
            // axis are on it — known, not measured — so they and the edges
            // between two of them say nothing; every other point of the
            // outline must lie definitely on one side, and all on the same.
            let mut left = false;
            let mut right = false;
            let mut touches = false;
            for edge in &region.outer.edges {
                let ends = sketch.curves[&edge.curve].endpoints();
                let on = |p: Option<PointId>| p.is_some_and(|p| on_axis.contains(&p));
                let (start_on, end_on) = (on(ends.map(|e| e.0)), on(ends.map(|e| e.1)));
                touches |= start_on || end_on;
                if start_on
                    && end_on
                    && matches!(sketch.curves[&edge.curve].kind, CurveKind::Line { .. })
                {
                    continue;
                }
                let samples = curve_polyline(sketch, edge.curve).with_context(ctx)?;
                let last = samples.len().saturating_sub(1);
                for (k, q) in samples.iter().enumerate() {
                    if (k == 0 && start_on) || (k == last && end_on) {
                        continue;
                    }
                    let side = dir.prod_cross(&q.map(|c| c.cast::<S>()).sub(&a));
                    left |= side.definitely_greater(S::ZERO);
                    right |= side.definitely_less(S::ZERO);
                    touches |= side.could_be_equal(S::ZERO);
                }
            }
            if left && right {
                return Err(GeopError::new("the profile crosses the revolve axis"))
                    .with_context(ctx);
            }
            if !left && !right {
                return Err(GeopError::new("the profile lies on the revolve axis"))
                    .with_context(ctx);
            }
            sides.push(left);
            if sides.iter().any(|&s| s != left) {
                return Err(GeopError::new(
                    "the sketch has regions on both sides of the revolve axis, which would overlap \
                     once revolved; revolve them in separate operations",
                ))
                .with_context(ctx);
            }
            let clear = !touches;

            // `(r, z)` coordinates: `r` towards the region, `z` along the axis,
            // oriented like the sketch (`z` is `r` turned counter-clockwise), so
            // loops keep their winding — a rigid motion, which leaves sweeps
            // and radii as they are. Points the constraints put on the axis
            // get `r = 0` exactly: that is what the constraints say.
            let r_dir = axis_left.prod_scalar(if left { S::ONE } else { S::ONE.neg() });
            let z_dir = Vector2::from_array([r_dir[1].neg(), r_dir[0]]);
            let rz = Enclosure {
                points: geometry
                    .points
                    .iter()
                    .map(|(&p, xy)| {
                        let d = xy.sub(&axis.point);
                        let r = if on_axis.contains(&p) {
                            S::ZERO
                        } else {
                            d.prod_dot(&r_dir)
                        };
                        (p, Vector2::from_array([r, d.prod_dot(&z_dir)]))
                    })
                    .collect(),
                params: geometry.params.clone(),
            };

            // The profile is the outer loop minus its run of edges on the
            // axis — or, clear of the axis, all of it — walked top-down (see
            // `revolve_at_oriented`): the loop is counter-clockwise, so the
            // region lies left of it, and reversing puts it on the right.
            let edges = &region.outer.edges;
            let flags: Vec<bool> = edges
                .iter()
                .map(|e| match sketch.curves[&e.curve].kind {
                    CurveKind::Line { start, end } => {
                        on_axis.contains(&start) && on_axis.contains(&end)
                    }
                    _ => false,
                })
                .collect();
            let n = edges.len();
            let starts_off_axis = |k: usize| !flags[k] && flags[(k + n - 1) % n];
            let (chain, closed) = match (0..n).filter(|&k| starts_off_axis(k)).count() {
                0 if clear => (region.outer.clone(), true),
                0 => {
                    return Err(GeopError::new(
                        "the profile touches the revolve axis, but none of its edges lies on it: \
                         constrain an edge onto a line of the sketch to revolve around, or keep \
                         the profile clear of the axis",
                    ))
                    .with_context(ctx);
                }
                1 => {
                    let first_off = (0..n).find(|&k| starts_off_axis(k)).unwrap();
                    let chain = ProfileLoop {
                        edges: (0..n)
                            .map(|k| (first_off + k) % n)
                            .take_while(|&k| !flags[k])
                            .map(|k| edges[k])
                            .collect(),
                    };
                    (chain, false)
                }
                _ => {
                    return Err(GeopError::new(
                        "the profile must touch the revolve axis along exactly one run of edges",
                    ))
                    .with_context(ctx);
                }
            };
            let profile = sketch_profile(
                &args.sketch,
                chain.reversed().to_nurbs(sketch, &rz).with_context(ctx)?,
                closed,
            );

            let plane = &placed.plane;
            let dir3 = |d: Vector2<S>| {
                plane
                    .u()
                    .prod_scalar(d[0])
                    .add(&plane.v().prod_scalar(d[1]))
            };
            let (u, w) = (dir3(r_dir), dir3(z_dir));
            let origin = plane.uv_to_xyz(&axis.point);
            let cs = CoordinateSystem::try_new(origin, u, w.prod_cross(&u), w)?;
            // Every region after the first is merged into the first, so its
            // own solid name only exists until then.
            let solid_name = match solid {
                None => args.combine.built_name(&namer),
                Some(_) => namer.name(&["solid", &profile.curve_names[0]]),
            };
            let built = revolve_at_oriented(&mut part, &namer, &solid_name, &profile, &cs)
                .with_context(ctx)?;
            match solid {
                None => solid = Some(built),
                Some(first) => part.merge_solids(first, built)?,
            }
        }
        if let Some(built) = solid {
            args.combine
                .apply(&mut part, &namer, operation_id, built)
                .with_context(ctx)?;
        }
        Ok(part)
    }
}
