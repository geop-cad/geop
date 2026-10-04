//! [`Revolve`]: sweep a sketch's area around a line in its plane.

use std::collections::BTreeSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_sketch::{
    CurveKind, Enclosure, PointId, ProfileJoint, ProfileLoop, Shape, profile::curve_polyline,
};
use geop_ops::{
    Context, Library, Namer, Part, PlacedSketch,
    operation::{Aspects, EntityRef, Operation, Role},
    ui::{Form, Number, Unit},
};
use geop_ops_booleans::{Combine, Tool};
use serde::{Deserialize, Serialize};

use super::{
    Extent, Extents, Plan, combine_sides, extrude::sketch_profile, face_target_field, hull,
    no_target, side_namer, side_tool, sketch_field, stops, trim_side,
};
use crate::{
    common::start_point,
    revolve::{revolution, revolve},
    sweep::SweepLoop,
};

/// Revolves the one area of a sketch around a line in its plane, into a
/// solid named `revolve(R)` for the operation `R` — or, with
/// [`RevolveArgs::combine`], combines that with another solid (see
/// [`Combine`]), the result named `revolve(R)` all the same. Or, as a face
/// ([`RevolveArgs::face`]), sweeps the sketch's curves into faces standing
/// on their own — the area's outline, or a chain of curves enclosing
/// nothing.
///
/// How far it turns is an [`Extents`] in degrees: an angle, up to a full
/// turn, or up to the next face of the solid it is joined to or cut from —
/// on one side of the sketch's plane, both alike, or each its own way.
///
/// The axis is any line (see [`Role::Line`]) in the sketch's plane: a line
/// of the sketch, of another one, a datum axis, a frame's axis, a straight
/// edge. The area, holes and all, lies on one side of it, and touches it
/// only where the constraints put it on the axis — which must then be a line
/// of the sketch itself (see `Sketch::on_line`), typically the axis line
/// itself. A point there is a pole, swept into nothing but itself, and an
/// edge there sweeps nothing: a full turn closes the solid over it, a
/// partial one leaves it the edge between its two caps.
///
/// The profile names what it sweeps (see [`revolve`]): with `X` a piece of
/// a sketch curve and `P` a joint of the sketch `K`, as for
/// [`super::Extrude`],
///
/// - `revolve(R,K,X,q0)`, `q1`, ...: the face `X` sweeps through each span
///   of at most a quarter turn — four for a full turn — starting from the
///   sketch plane;
/// - `revolve(R,K,X,a0)`, `a1`, ...: `X` itself at each angle between the
///   spans (`a0` is the profile where it was drawn, for a turn starting at
///   the sketch plane);
/// - `revolve(R,K,P,q0)` .. / `revolve(R,K,P,a0)` ..: the circular edges and
///   vertices `P` sweeps, and `revolve(R,K,P)` for a `P` on the axis;
/// - `revolve(R,start)` / `revolve(R,end)`: the caps of a partial turn.
///
/// A side going up to the next face is built on its own, as for
/// [`super::Extrude`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Revolve;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RevolveArgs {
    /// The sketch to revolve.
    pub sketch: String,
    /// The line to revolve around, in the sketch's plane; none yet, for a
    /// step that has not picked one.
    pub axis: Option<EntityRef>,
    /// How far, in degrees: turning from the area's side of the axis
    /// counter-clockwise, as seen looking down the axis — negative to turn
    /// the other way.
    pub extent: Extents,
    /// Sweep the sketch's curves into faces standing on their own, rather
    /// than its area into a solid.
    #[serde(default)]
    pub face: bool,
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

    fn formulas<'a>(&self, args: &'a mut RevolveArgs) -> Vec<&'a mut String> {
        args.extent.formulas()
    }

    /// The newest sketch a full turn around its axis line, joined to the
    /// newest solid if there is one.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> RevolveArgs {
        let sketch = before.sketch_names().pop().unwrap_or_default();
        RevolveArgs {
            axis: default_axis(before, &sketch),
            sketch,
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::new_for(before),
        }
    }

    /// The sketch and the axis, picked; how far; whether a face; and how to
    /// combine. Another sketch picked brings an axis that was a line of the
    /// old one back to the new one's default.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &RevolveArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, RevolveArgs> {
        let before = context.before;
        let mut f = Form::<S, RevolveArgs>::new();
        sketch_field(
            &mut f,
            before,
            "sketch",
            &args.sketch,
            move |args, sketch| {
                let old = std::mem::replace(&mut args.sketch, sketch);
                let of_old = match &args.axis {
                    None => true,
                    Some(EntityRef::SketchCurve { sketch, .. }) => *sketch == old,
                    Some(_) => false,
                };
                if of_old {
                    args.axis = default_axis(before, &args.sketch);
                }
            },
        );
        f.reference(
            "axis",
            "axis",
            args.axis.iter().cloned().collect(),
            &[Role::Line],
            None,
            false,
            |edit, picked| edit.args.axis = picked.into_iter().next(),
        );
        args.extent.show(
            &mut f,
            "angle",
            |angle, second| {
                let label = if second { "angle 2" } else { "angle" };
                Number::formula(label, angle, before.inputs(), Unit::Angle).range(-360.0, 360.0)
            },
            360.0,
            |args| &mut args.extent,
            |args, angle| args.extent.side1 = Extent::Blind(angle),
        );
        f.checkbox("face", "face", args.face, |args, b| args.face = b);
        if !args.face {
            args.combine.show(&mut f, before, |args| &mut args.combine);
        } else if args.extent.reaches_target() {
            face_target_field(&mut f, &args.combine, |args| &mut args.combine);
        }
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

        // What to sweep, each loop as the sketch runs it: an area's outer
        // loop counter-clockwise, its holes clockwise.
        let (loops, closed): (Vec<ProfileLoop>, bool) = match if args.face {
            sketch.shape()
        } else {
            sketch.region().map(Shape::Region)
        }
        .with_context(ctx)?
        {
            Shape::Region(region) => (
                std::iter::once(region.outer).chain(region.holes).collect(),
                true,
            ),
            Shape::Chain(chain) => (vec![chain], false),
        };

        // Which side of the axis the profile lies on, from its curves as
        // drawn; it must not cross. The points the constraints put on the
        // axis are on it — known, not measured — so they and the edges
        // between two of them say nothing; every other point of a curve must
        // lie definitely on one side, and all on the same.
        let mut left = false;
        let mut right = false;
        for edge in loops.iter().flat_map(|l| &l.edges) {
            let ends = sketch.curves[&edge.curve].endpoints();
            let on = |p: Option<PointId>| p.is_some_and(|p| on_axis.contains(&p));
            let (start_on, end_on) = (on(ends.map(|e| e.0)), on(ends.map(|e| e.1)));
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
            }
        }
        if left && right {
            return Err(GeopError::new("the profile crosses the revolve axis")).with_context(ctx);
        }
        if !left && !right {
            return Err(GeopError::new("the profile lies on the revolve axis")).with_context(ctx);
        }

        // `(r, z)` coordinates: `r` towards the profile, `z` along the axis,
        // oriented like the sketch (`z` is `r` turned counter-clockwise), so
        // loops keep their winding — a rigid motion, which leaves sweeps and
        // radii as they are. Points the constraints put on the axis get `r =
        // 0` exactly: that is what the constraints say.
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
        let pole =
            |joint: &ProfileJoint| matches!(joint, ProfileJoint::Point(p) if on_axis.contains(p));
        let sweep_loops = loops
            .iter()
            .map(|lp| {
                let pieces = lp.to_nurbs(sketch, &rz)?;
                let mut poles: Vec<bool> = pieces.iter().map(|p| pole(&p.start)).collect();
                if !closed && let Some(last) = pieces.last() {
                    poles.push(pole(&last.end));
                }
                let along_axis: Vec<bool> = pieces
                    .iter()
                    .map(|p| match sketch.curves[&p.source].kind {
                        CurveKind::Line { start, end } => {
                            on_axis.contains(&start) && on_axis.contains(&end)
                        }
                        _ => false,
                    })
                    .collect();
                let profile = sketch_profile(&args.sketch, pieces, closed);
                for (i, &is_pole) in poles.iter().enumerate() {
                    let at = if i < profile.curves.len() {
                        start_point(&profile.curves[i])?
                    } else {
                        crate::common::end_point(&profile.curves[i - 1])?
                    };
                    if !is_pole && at[0].could_be_equal(S::ZERO) {
                        return Err(GeopError::new(format!(
                            "the profile touches the revolve axis at {}, which the constraints do not put on it: constrain it onto a line of the sketch to revolve around, or keep the profile clear of the axis",
                            profile.joint_names[i]
                        )));
                    }
                }
                Ok(SweepLoop {
                    profile,
                    poles,
                    on_axis: along_axis,
                })
            })
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;

        let plane = &placed.plane;
        let dir3 = |d: Vector2<S>| {
            plane
                .u()
                .prod_scalar(d[0])
                .add(&plane.v().prod_scalar(d[1]))
        };
        let (u, w) = (dir3(r_dir), dir3(z_dir));
        let origin = plane.uv_to_xyz(&axis.point);
        let axes = CoordinateSystem::try_new(origin, u, w.prod_cross(&u), w)?;

        let stops = stops(&part, &args.combine);
        let hull = if args.extent.reaches_target() {
            hull(&part, &stops).with_context(ctx)?
        } else {
            None
        };
        let plan = args
            .extent
            .plan(&mut part, |sign| match &hull {
                Some(hull) => turn_past(hull, &axes, sign),
                None => Err(no_target()),
            })
            .with_context(ctx)?;

        match (args.face, plan) {
            (true, Plan::Whole(from, to)) => {
                revolve(&mut part, &namer, None, &axes, from, to, &sweep_loops)
                    .with_context(ctx)?;
            }
            (true, Plan::Sides(sides)) => {
                for (k, side) in sides.into_iter().enumerate() {
                    let named = side_namer(&namer, k);
                    let built = revolve(&mut part, &named, None, &axes, 0.0, side.to, &sweep_loops)
                        .with_context(ctx)?;
                    if side.up_to_next {
                        let stations = revolution::<S>(&axes, 0.0, side.to)?.station_names;
                        let (first, last) = (&stations[0], &stations[stations.len() - 1]);
                        trim_side(
                            &mut part,
                            operation_id,
                            k,
                            &built,
                            &stops,
                            (&named, &sweep_loops),
                            (first, last),
                        )
                        .with_context(ctx)?;
                    }
                }
            }
            (false, Plan::Whole(from, to)) => {
                let name = args.combine.built_name(&namer);
                let built = revolve(
                    &mut part,
                    &namer,
                    Some(&name),
                    &axes,
                    from,
                    to,
                    &sweep_loops,
                )
                .with_context(ctx)?;
                let tool = Tool {
                    solid: built.solid.expect("revolved as a solid"),
                    up_to_next: None,
                    scope: None,
                };
                args.combine
                    .apply(&mut part, &namer, operation_id, &[tool])
                    .with_context(ctx)?;
            }
            (false, Plan::Sides(sides)) => {
                let mut tools = Vec::new();
                for (k, &side) in sides.iter().enumerate() {
                    let named = side_namer(&namer, k);
                    let name = args.combine.built_name(&named);
                    let solid = revolve(
                        &mut part,
                        &named,
                        Some(&name),
                        &axes,
                        0.0,
                        side.to,
                        &sweep_loops,
                    )
                    .with_context(ctx)?
                    .solid
                    .expect("revolved as a solid");
                    tools.push(side_tool(&named, k, solid, side));
                }
                let far: Vec<f64> = sides.iter().map(|side| side.to.abs()).collect();
                combine_sides(
                    &mut part,
                    &namer,
                    operation_id,
                    &args.combine,
                    tools,
                    &stops,
                    &far,
                    &[360.0; 2][..sides.len()],
                    |part, k, from, to| {
                        let named = side_namer(&namer, k);
                        let name = args.combine.built_name(&named);
                        let sign = sides[k].to.signum();
                        let built = revolve(
                            part,
                            &named,
                            Some(&name),
                            &axes,
                            from * sign,
                            to * sign,
                            &sweep_loops,
                        )?;
                        Ok(built.solid.expect("revolved as a solid"))
                    },
                    |k, p| turned_angle(&axes, p, sides[k].to.signum()),
                )
                .with_context(ctx)?;
            }
        }
        Ok(part)
    }
}

/// How far `p` lies turned around `axes.w()` from where the turn starts,
/// at `axes.u()` — forwards for `sign = 1`, backwards for `-1` — in
/// degrees in `(0, 360)`; none for a point that could lie on the half plane
/// the turn starts from, or on the axis.
fn turned_angle<S: Scalar>(axes: &CoordinateSystem<S>, p: &Vector3<S>, sign: f64) -> Option<f64> {
    let q = axes.to_uvw(p);
    let on_axis = q[0].could_be_equal(S::ZERO) && q[1].could_be_equal(S::ZERO);
    let at_start = q[1].could_be_equal(S::ZERO) && q[0].definitely_greater(S::ZERO);
    if on_axis || at_start {
        return None;
    }
    let angle = (sign * q[1].to_f64().atan2(q[0].to_f64()).to_degrees()).rem_euclid(360.0);
    (angle > 0.0 && angle < 360.0).then_some(angle)
}

/// How far to turn around `axes.w()` — forwards for `sign = 1`, backwards
/// for `-1`, in degrees from `axes.u()` — for a side going up to its next
/// face to meet whichever face that is, short of a full turn, which would
/// run into itself.
///
/// Seen down the axis, the solid whose convex hull `hull` spans fills the
/// arc between the hull's points that leaves out the widest gap: from `b`
/// to `e`, turning the way the side does. A turn starting inside that arc
/// passes its end and stops in the middle of the gap after it. One starting
/// before it — in the gap — passes all of it and stops between its end and a
/// full turn; but if the arc reaches round to where the turn starts, there is
/// nothing beyond it short of a full turn, and the turn stops in the middle
/// of the arc instead: whatever the side meets on the way in is there
/// already. If the axis runs through the hull, there is no gap at all; one
/// merely touching it still leaves the half turn on its other side.
fn turn_past<S: Scalar>(
    hull: &[[f64; 3]],
    axes: &CoordinateSystem<S>,
    sign: f64,
) -> GeopResult<f64> {
    let f = |v: &geop_core_math::vector::Vector3<S>| [0, 1, 2].map(|k| v[k].to_f64());
    let (o, u, v) = (f(axes.origin()), f(axes.u()), f(axes.v()));
    // In `[0, 360)`: `rem_euclid` of the smallest negative angle rounds to
    // 360 itself, which is where the turn starts — 0.
    let turned = |degrees: f64| match degrees.rem_euclid(360.0) {
        a if a >= 360.0 => 0.0,
        a => a,
    };
    let mut angles = Vec::with_capacity(hull.len());
    for p in hull {
        let d: Vec<f64> = (0..3).map(|k| p[k] - o[k]).collect();
        let (x, y) = (
            (0..3).map(|k| d[k] * u[k]).sum::<f64>(),
            (0..3).map(|k| d[k] * v[k]).sum::<f64>(),
        );
        if x == 0.0 && y == 0.0 {
            return Err(surrounds());
        }
        angles.push(turned(sign * y.atan2(x).to_degrees()));
    }
    angles.sort_by(f64::total_cmp);
    let n = angles.len();
    let (gap, after) = (0..n)
        .map(|i| {
            let next = if i + 1 < n {
                angles[i + 1]
            } else {
                angles[0] + 360.0
            };
            (next - angles[i], angles[i])
        })
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .expect("a hull has points");
    // A convex hull clear of the axis fills less than a half turn of
    // directions, and one merely touching it — a solid with a face in a
    // plane through the axis — exactly a half turn: the other half is free
    // either way. Only a hull the axis runs through leaves less.
    if gap < 180.0 {
        return Err(surrounds());
    }
    // The arc the solid fills, from `b` to `e`; an end at the start is a
    // full turn round.
    let b = turned(after + gap);
    let e = if after == 0.0 { 360.0 } else { after };
    Ok(if b > e {
        // The arc runs over the start: past its end, into the gap.
        e + gap / 2.0
    } else if e < 360.0 {
        (e + 360.0) / 2.0
    } else {
        (b + 360.0) / 2.0
    })
}

/// Why [`turn_past`] finds no angle to turn to.
fn surrounds() -> GeopError {
    GeopError::new(
        "up to next: the target lies all around the revolve axis, so no turn short of a full one comes past it; give the angle instead",
    )
}

#[cfg(test)]
mod tests {
    use super::turn_past;
    use geop_core_math::{primitives::CoordinateSystem, scalars::ScalInF64 as S, vector::Vector3};

    /// A solid with a face in the plane the turn starts from — a revolve
    /// sketched on that face — lies against the start from behind: one of
    /// its points, at an angle a hair below zero, rounds to a whole turn.
    /// The turn must still stop short of one, wherever it goes.
    #[test]
    fn a_target_against_the_start_plane_stops_the_turn_short_of_a_full_one() {
        let axes = CoordinateSystem::<S>::world_at(Vector3::zero());
        // The half space below `y = 0`, touching the start plane at `x > 0`
        // — one point a hair below it.
        let hull = [
            [2.0, -1e-17, 0.0],
            [3.0, 0.0, 1.0],
            [-3.0, 0.0, 1.0],
            [0.0, -3.0, 0.0],
            [2.0, -2.0, 1.0],
        ];
        for sign in [1.0, -1.0] {
            let far = turn_past(&hull, &axes, sign).unwrap();
            assert!(far > 0.0 && far < 360.0, "{sign}: {far}");
        }
        // Turning forwards, through the free half first, it must come past
        // the half turn where the solid begins.
        assert!(turn_past(&hull, &axes, 1.0).unwrap() > 180.0);
    }
}
