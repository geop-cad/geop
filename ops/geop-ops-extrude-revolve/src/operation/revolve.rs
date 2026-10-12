//! [`Revolve`]: sweep a sketch's area around a line in its plane.

use std::collections::BTreeSet;

use geop_core_math::scalars::Ring;
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
use geop_core_topology::build::BuiltBody;
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, NamedModel, Namer, PlacedSketch,
    operation::{
        Aspects, BuildReader, EntityRef, Lookup, PartOperation, Role, Scope, Seen, StepContext,
    },
    parameters::Formula,
    part_operation,
    ui::{EditForm, FormState, Unit, Value, formula_number, sketch_field},
};
use geop_ops_booleans::Extent;
use serde::{Deserialize, Serialize};

use super::{Extents, Output, Sweeping, build_extents};
use crate::{
    common::Profile,
    revolve::{revolution, revolve},
    sweep::SweepLoop,
};

/// Revolves the one area of a sketch around a line in its plane, into a
/// solid named `revolve(R)` for the operation `R` — or combines that with
/// another solid (see [`Combine`](geop_ops_booleans::Combine)), the result named `revolve(R)` all the
/// same. Or, as faces ([`Output::Faces`]), sweeps the sketch's curves into
/// faces standing on their own — the area's outline, or a chain of curves
/// enclosing nothing.
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
    /// A solid, kept as a new body or combined with another, or faces.
    #[serde(default)]
    pub output: Output,
}

/// The line of `sketch` to revolve around by default: its first
/// construction line — what an axis is usually drawn as — else its first
/// line; none, for a sketch with no line.
fn default_axis<S: Scalar>(
    lookup: &mut (impl Lookup<S> + ?Sized),
    sketch: &str,
) -> Option<EntityRef> {
    let placed = lookup.sketch(sketch).ok()?;
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
    /// `axis` in the coordinates of `placed`, the sketch `name`, whose
    /// solution is `geometry`, other entities resolved by `lookup`. A line of the sketch itself is its own
    /// line; any other line must lie in the sketch's plane, and is used as
    /// it lies there.
    fn of(
        axis: &EntityRef,
        lookup: &mut (impl Lookup<S> + ?Sized),
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
        let line = Aspects::of(axis, lookup)
            .with_context(ctx)?
            .line
            .ok_or_else(|| GeopError::new(format!("{axis} is not a line")))
            .with_context(ctx)?;
        let plane = &placed.plane;
        let point = plane.to_uvw(&line.point);
        let along = |axis: &Vector3<S>| line.direction.prod_dot(axis);
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

part_operation!(Revolve);

impl PartOperation for Revolve {
    const INFO: OperationInfo = OperationInfo::new(
        "revolve",
        "Revolve",
        "Sweep a sketch's area around one of its lines into a solid, or its curves into faces.",
        OperationGroup::Solid,
        OperationTier::Big,
    )
    .icon(crate::icons::REVOLVE);
    type Args = RevolveArgs;
    type Session = ();

    fn formulas(args: &mut RevolveArgs) -> Vec<&mut String> {
        args.extent.formulas()
    }

    /// The newest sketch a full turn around its axis line, joined to the
    /// newest solid if there is one.
    fn new_args(context: &StepContext) -> RevolveArgs {
        let mut seen = Seen::new(context.before);
        let sketch = seen.sketch_names().pop().unwrap_or_default();
        RevolveArgs {
            axis: default_axis(&mut seen, &sketch),
            sketch,
            extent: Extents::blind(360.0),
            output: Output::new_for(seen.model()),
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &RevolveArgs,
    ) -> GeopResult<()> {
        type S = Design;
        let namer = Namer::new("revolve", step)?;
        let mut scope = Scope {
            model: &mut *model,
            reader: &mut *reader,
        };
        let placed = scope.sketch(&args.sketch)?.clone();
        let sketch = &placed.sketch;
        let Some(axis) = &args.axis else {
            return Err(GeopError::new("pick a line to revolve around"));
        };
        let geometry = sketch.enclose::<S>()?;
        let axis = SketchAxis::of(axis, &mut scope, &args.sketch, &placed, &geometry)?;
        let (a, dir) = axis.drawn;
        let axis_left = axis.direction.perp();
        let on_axis = &axis.on_axis;

        // What to sweep, each loop as the sketch runs it: an area's outer
        // loop counter-clockwise, its holes clockwise.
        let (loops, closed): (Vec<ProfileLoop>, bool) = match args.output.shape(sketch)? {
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
            let samples = curve_polyline(sketch, edge.curve)?;
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
            return Err(GeopError::new("the profile crosses the revolve axis"));
        }
        if !left && !right {
            return Err(GeopError::new("the profile lies on the revolve axis"));
        }

        // `(r, z)` coordinates: `r` towards the profile, `z` along the axis,
        // oriented like the sketch (`z` is `r` turned counter-clockwise), so
        // loops keep their winding — a rigid motion, which leaves sweeps and
        // radii as they are. Points the constraints put on the axis get `r =
        // 0` exactly: that is what the constraints say.
        let r_dir = axis_left.prod_scalar(if left { S::ONE } else { S::ONE.neg() });
        let z_dir = r_dir.perp();
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
                let profile = Profile::of_pieces(&args.sketch, pieces, closed);
                for (i, &is_pole) in poles.iter().enumerate() {
                    let at = profile.joint(i)?;
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
            ?;

        let plane = &placed.plane;
        let (u, w) = (plane.uv_direction(&r_dir), plane.uv_direction(&z_dir));
        let origin = plane.uv_to_xyz(&axis.point);
        let axes = CoordinateSystem::try_from_xz(origin, u, w)?;

        let revolving = Revolving {
            axes: &axes,
            loops: &sweep_loops,
        };
        let how = (&args.extent, &args.output);
        build_extents(model, reader, &namer, step, how, &revolving)
    }
    /// The sketch and the axis, picked; how far; whether a face; and how to
    /// combine. Another sketch picked brings an axis that was a line of the
    /// old one back to the new one's default.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &RevolveArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, RevolveArgs> {
        let mut seen = Seen::new(context.before);
        let mut f = EditForm::<Design, RevolveArgs>::default();
        let sketch = std::slice::from_ref(&args.sketch);
        sketch_field(&mut f.form.dialog, &context.before, "sketch", sketch, false);
        f.on("sketch", move |edit, value| {
            let Value::Entities(picked) = value else {
                return;
            };
            let args = edit.args;
            let picked = EntityRef::last_name(EntityRef::sketch, &picked);
            let old = std::mem::replace(&mut args.sketch, picked.unwrap_or_default());
            let of_old = match &args.axis {
                None => true,
                Some(EntityRef::SketchCurve { sketch, .. }) => *sketch == old,
                Some(_) => false,
            };
            if of_old {
                args.axis = default_axis(&mut Seen::new(context.before), &args.sketch);
            }
        });
        let axis = args.axis.iter().cloned().collect();
        f.reference(
            "axis",
            "axis",
            axis,
            &[Role::Line],
            None,
            false,
            |edit, picked| edit.args.axis = picked.first().cloned(),
        );
        let amount = |angle: &Formula, second| {
            let label = if second { "angle 2" } else { "angle" };
            formula_number(label, angle, &mut seen, Unit::Angle).range(-360.0, 360.0)
        };
        args.extent.fields(
            &mut f,
            ("angle", 360.0),
            amount,
            |args| &mut args.extent,
            |edit, angle| edit.args.extent.side1 = Extent::Blind(angle),
        );
        let reaches = args.extent.reaches_target();
        let output = |args: &mut RevolveArgs, output| args.output = output;
        (args.output).fields(&mut f, Seen::new(context.before).model(), reaches, output);
        f
    }
}

/// A revolve's loops, swept around the axis `axes.w()` from `axes.u()`
/// (see [`Sweeping`]): its measure is the angle turned, in degrees.
struct Revolving<'a> {
    axes: &'a CoordinateSystem<Design>,
    loops: &'a [SweepLoop<Design>],
}

impl Sweeping for Revolving<'_> {
    fn loops(&self) -> &[SweepLoop<Design>] {
        self.loops
    }

    fn sweep(
        &self,
        model: &mut NamedModel<Design>,
        namer: &Namer,
        name: Option<&str>,
        from: f64,
        to: f64,
    ) -> GeopResult<BuiltBody> {
        revolve(model, namer, name, self.axes, from, to, self.loops)
    }

    fn past(&self, hull: &[[f64; 3]], sign: f64) -> GeopResult<f64> {
        turn_past(hull, self.axes, sign)
    }

    fn ends(&self, to: f64) -> GeopResult<(String, String)> {
        let stations = revolution::<Design>(self.axes, 0.0, to)?.station_names;
        Ok((stations[0].clone(), stations[stations.len() - 1].clone()))
    }

    fn all_the_way(&self, _: f64) -> f64 {
        360.0
    }

    fn measure(&self, sign: f64, p: &Vector3<Design>) -> Option<f64> {
        turned_angle(self.axes, p, sign)
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
    let f = |v: &Vector3<S>| v.to_f64_array();
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
