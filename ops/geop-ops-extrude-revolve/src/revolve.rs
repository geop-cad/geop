//! [`Revolve`]: sweep a sketch's area around a line in its plane.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Unit},
    one_of, part_operation,
    part_operation::PartOperation,
};
use geop_core_program::{
    element::ElementReference, numeric::NumericInput, part::StepContext, reader::Reader,
};
use std::collections::BTreeSet;

use geop_core_geometry::{
    search::{MAX_NODES, min_subdivision_size},
    shape::Axis,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_sketch::{
    CurveKind, Enclosure, PointId, ProfileJoint, ProfileLoop, Shape, profile::curve_nurbs,
};
use geop_core_topology::build::BuiltBody;
use geop_ops_model::datums::Datums;
use geop_ops_model::sketch::{PlacedSketch, SketchRef, chain_of_pieces, sketch_element};
use geop_ops_model::{Extent, Line, ModelOperation, NamedModel, Namer, build_model, read_model};
use serde::{Deserialize, Serialize};

use geop_lib_sweep::{SweepLoop, revolution, revolve};

use crate::sweeping::{Extents, Output, Sweeping, build_extents};

one_of! {
    /// A line a revolve turns around: a line of a sketch — its own, or
    /// another's — a datum axis or a frame's, a straight edge.
    pub enum RevolveAxis -> Axis {
        Sketch(ElementReference<PlacedSketch, Line>),
        Datum(ElementReference<Datums, Line>),
        Edge(ElementReference<NamedModel, Line>),
    }
}

/// Revolves the one area of a sketch around a line in its plane, into a
/// solid named `revolve(R)` for the operation `R` — or combines that with
/// another solid (see [`Combine`](geop_ops_model::Combine)), the result named `revolve(R)` all the
/// same. Or, as faces ([`Output::Faces`]), sweeps the sketch's curves into
/// faces standing on their own — the area's outline, or a chain of curves
/// enclosing nothing.
///
/// How far it turns is an [`Extents`] in degrees: an angle, up to a full
/// turn, or up to the next face of the solid it is joined to or cut from —
/// on one side of the sketch's plane, both alike, or each its own way.
///
/// The axis is any line (see [`RevolveAxis`]) in the sketch's plane: a
/// line of the sketch, of another one, a datum axis, a frame's axis, a
/// straight edge. The area, holes and all, lies on one side of it, and touches it
/// only where the constraints put it on the axis — which must then be a line
/// of the sketch itself (see `Sketch::on_line`), typically the axis line
/// itself. A point there is a pole, swept into nothing but itself, and an
/// edge there sweeps nothing: a full turn closes the solid over it, a
/// partial one leaves it the edge between its two caps.
///
/// The profile names what it sweeps (see [`revolve`]): with `X` a piece of
/// a sketch curve and `P` a joint of the sketch `K`, as for
/// [`crate::Extrude`],
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
/// [`crate::Extrude`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Revolve;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RevolveArgs {
    /// The sketch to revolve.
    pub sketch: SketchRef,
    /// The line to revolve around, in the sketch's plane; none yet, for a
    /// step that has not picked one.
    pub axis: Option<RevolveAxis>,
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
fn default_axis(reader: &Reader, sketch: &SketchRef) -> Option<RevolveAxis> {
    let placed = reader.retrieve_element(sketch).ok()?;
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
    Some(RevolveAxis::Sketch(sketch_element(&placed.name, curve)))
}

/// The axis in the sketch's own coordinates.
struct SketchAxis {
    /// A point on it, and its unit direction, enclosed.
    point: Vector2,
    direction: Vector2,
    /// A point on it and its direction (of any length), as drawn: which side
    /// of it a region lies on is a question about the sketch as drawn.
    drawn: (Vector2, Vector2),
    /// The points of the sketch the constraints put on it — none, unless it
    /// is a line of the sketch.
    on_axis: BTreeSet<PointId>,
}

impl SketchAxis {
    /// `axis` in the coordinates of `placed`, the sketch `sketch`, whose
    /// solution is `geometry`, other elements read through `reader`. A line
    /// of the sketch itself is its own line; any other line must lie in the
    /// sketch's plane, and is used as it lies there.
    fn of(
        axis: &RevolveAxis,
        reader: &Reader,
        sketch: &SketchRef,
        placed: &PlacedSketch,
        geometry: &Enclosure,
    ) -> GeopResult<Self> {
        let ctx = with_context!("revolving around {axis:?}");
        let name = &placed.name;
        let unit = |d: Vector2| d.normalize().with_context(ctx);
        if let RevolveAxis::Sketch(line) = axis
            && line.target() == sketch.target()
        {
            let curve = line.name().parse().with_context(ctx)?;
            let sketch = &placed.sketch;
            let CurveKind::Line { start, end } = sketch.curve(curve).with_context(ctx)?.kind else {
                return Err(GeopError::new(format!("{curve} is not a line"))).with_context(ctx);
            };
            let drawn = |p: PointId| sketch.points[&p].xy();
            let point = geometry.points[&start];
            return Ok(SketchAxis {
                point,
                direction: unit(geometry.points[&end].sub(&point))?,
                drawn: (drawn(start), drawn(end).sub(&drawn(start))),
                on_axis: sketch.on_line(curve).with_context(ctx)?,
            });
        }
        let line = axis.resolve(reader).with_context(ctx)?;
        let plane = &placed.plane;
        let point = plane.to_uvw(&line.point);
        let along = |axis: &Vector3| line.direction.prod_dot(axis);
        let direction = [along(plane.u()), along(plane.v()), along(plane.w())];
        if !point[2].could_be_equal(Scalar::ZERO) || !direction[2].could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "it does not lie in the plane of sketch {name:?}"
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

part_operation!(Revolve(RevolveArgs));

impl PartOperation for Revolve {
    const KIND: &'static str = "revolve";
    const INFO: OperationInfo = OperationInfo::new(
        "Revolve",
        "Sweep a sketch's area around one of its lines into a solid, or its curves into faces.",
        OperationGroup::Solid,
        OperationTier::Big,
    )
    .shortcut("R")
    .icon(crate::icons::REVOLVE);
    type Args = RevolveArgs;
    type Session = ();

    /// The newest sketch a full turn around its axis line, joined to the
    /// newest solid if there is one.
    fn new_args(context: &Context) -> RevolveArgs {
        let before = &context.before;
        let newest = before.targets_of::<PlacedSketch>().pop();
        let sketch = SketchRef::whole(newest.unwrap_or_else(|| geop_ops_model::sketch::sketch("")));
        RevolveArgs {
            axis: default_axis(before, &sketch),
            sketch,
            extent: Extents::blind(360.0),
            output: Output::new_for(&read_model(before).unwrap_or_default()),
        }
    }

    fn run(ctx: &mut StepContext, args: &RevolveArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The sketch and the axis, picked; how far; whether a face; and how to
    /// combine. Another sketch picked brings an axis that was a line of the
    /// old one back to the new one's default.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &RevolveArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, RevolveArgs> {
        let before = &context.before;
        let mut f = EditForm::<RevolveArgs>::default();
        f.pick_one(
            "sketch",
            "sketch",
            before,
            Some(&args.sketch),
            move |edit, picked| {
                let args = edit.args;
                let old = std::mem::replace(&mut args.sketch, picked);
                let of_old = match &args.axis {
                    None => true,
                    Some(RevolveAxis::Sketch(line)) => line.target() == old.target(),
                    Some(_) => false,
                };
                if of_old {
                    args.axis = default_axis(before, &args.sketch);
                }
            },
        );
        f.pick_one(
            "axis",
            "axis",
            before,
            args.axis.as_ref(),
            |edit, picked| edit.args.axis = Some(picked),
        );
        let amount = |angle: &NumericInput, second| {
            let label = if second { "angle 2" } else { "angle" };
            Number::numeric(angle, label, before, Unit::Angle).range(-360.0, 360.0)
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
        (args.output).fields(&mut f, before, reaches, output);
        f
    }
}

impl ModelOperation for Revolve {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &RevolveArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("revolve", step)?;
        let placed = reader.retrieve_element(&args.sketch)?;
        let sketch = &placed.sketch;
        let Some(axis) = &args.axis else {
            return Err(GeopError::new("pick a line to revolve around"));
        };
        let geometry = sketch.enclose()?;
        let axis = SketchAxis::of(axis, reader, &args.sketch, placed, &geometry)?;
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

        // Which side of the axis the profile lies on; it must not cross. The
        // points the constraints put on the axis are on it — known, not
        // measured — so they and the edges between two of them say nothing;
        // every other point of a curve must lie definitely on one side, and
        // all on the same. A curve is furthest to either side at its ends or
        // where it runs parallel to the axis
        // ([`NurbCurve::stationary_parameters`] across it): there are its
        // extremes, so they say on which sides it lies.
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
            let pieces = curve_nurbs(sketch, &geometry, edge.curve)?;
            let last = pieces.len().saturating_sub(1);
            for (k, piece) in pieces.iter().enumerate() {
                let (t0, t1) = piece.domain();
                let side_at = |t: Scalar| -> GeopResult<Scalar> {
                    Ok(dir.prod_cross(&piece.evaluate(t)?.sub(&a)))
                };
                let mut ends = Vec::new();
                if !(k == 0 && start_on) {
                    ends.push(t0);
                }
                if !(k == last && end_on) {
                    ends.push(t1);
                }
                // A stationary point inside the curve that could lie on the
                // axis is a touch no joint stands for: revolved, the wall
                // would come to a pole there that no vertex of the profile
                // is. One at a constrained end is that end's own pole.
                let at_constrained_end = |t: &Scalar| {
                    (k == 0 && start_on && t.could_be_equal(t0))
                        || (k == last && end_on && t.could_be_equal(t1))
                };
                let interior = piece
                    .stationary_parameters(&dir.perp(), MAX_NODES, min_subdivision_size())?
                    .into_iter()
                    .filter(|t| !at_constrained_end(t));
                for t in interior {
                    let side = side_at(t)?;
                    if side.could_be_equal(Scalar::ZERO) {
                        return Err(GeopError::new(format!(
                            "the curve {},{} touches the revolve axis inside it, at parameter {t:?}: split it there at a point constrained onto the axis, or keep it clear of the axis",
                            placed.name, edge.curve
                        )));
                    }
                    left |= side.definitely_greater(Scalar::ZERO);
                    right |= side.definitely_less(Scalar::ZERO);
                }
                for t in ends {
                    let side = side_at(t)?;
                    left |= side.definitely_greater(Scalar::ZERO);
                    right |= side.definitely_less(Scalar::ZERO);
                }
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
        let r_dir = axis_left.prod_scalar(if left { Scalar::ONE } else { Scalar::ONE.neg() });
        let z_dir = r_dir.perp();
        let rz = Enclosure {
            points: geometry
                .points
                .iter()
                .map(|(&p, xy)| {
                    let d = xy.sub(&axis.point);
                    let r = if on_axis.contains(&p) {
                        Scalar::ZERO
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
                let profile = chain_of_pieces(&placed.name, pieces, closed);
                for (i, &is_pole) in poles.iter().enumerate() {
                    let at = profile.joint(i)?;
                    if !is_pole && at[0].could_be_equal(Scalar::ZERO) {
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
                    apex: false,
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
}

/// A revolve's loops, swept around the axis `axes.w()` from `axes.u()`
/// (see [`Sweeping`]): its measure is the angle turned, in degrees.
struct Revolving<'a> {
    axes: &'a CoordinateSystem,
    loops: &'a [SweepLoop],
}

impl Sweeping for Revolving<'_> {
    fn loops(&self) -> &[SweepLoop] {
        self.loops
    }

    fn sweep(
        &self,
        model: &mut NamedModel,
        namer: &Namer,
        name: Option<&str>,
        from: f64,
        to: f64,
    ) -> GeopResult<BuiltBody> {
        revolve(model, namer, name, self.axes, from, to, self.loops)
    }

    fn past(&self, hull: &[Vector3], sign: f64) -> GeopResult<f64> {
        turn_past(hull, self.axes, sign)
    }

    fn ends(&self, to: f64) -> GeopResult<(String, String)> {
        let stations = revolution(self.axes, 0.0, to)?.station_names;
        Ok((stations[0].clone(), stations[stations.len() - 1].clone()))
    }

    fn all_the_way(&self, _: f64) -> f64 {
        360.0
    }

    fn full_turn(&self) -> Option<f64> {
        Some(360.0)
    }

    fn measure(&self, sign: f64, p: &Vector3) -> Option<f64> {
        turned_angle(self.axes, p, sign)
    }
}

/// How far `p` lies turned around `axes.w()` from where the turn starts,
/// at `axes.u()` — forwards for `sign = 1`, backwards for `-1` — in
/// degrees in `(0, 360)`; none for a point that could lie on the half plane
/// the turn starts from, or on the axis.
fn turned_angle(axes: &CoordinateSystem, p: &Vector3, sign: f64) -> Option<f64> {
    let q = axes.to_uvw(p);
    let on_axis = q[0].could_be_equal(Scalar::ZERO) && q[1].could_be_equal(Scalar::ZERO);
    let at_start = q[1].could_be_equal(Scalar::ZERO) && q[0].definitely_greater(Scalar::ZERO);
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
///
/// Touching is the usual case of a revolve around an edge of the target —
/// a cup's seam, the edge of a plate — and the hull's points then lie on
/// the axis and on a plane through it, exactly where rounding decides
/// nothing. So they are measured as enclosed: a point that could lie on the
/// axis says nothing of which way the target lies, and the target only
/// surrounds the axis if the two points bounding the widest gap definitely
/// leave less than a half turn between them.
fn turn_past(hull: &[Vector3], axes: &CoordinateSystem, sign: f64) -> GeopResult<f64> {
    let surrounds = |detail: String| {
        surrounds(format!(
            "{detail}; the axis through {:?} along {:?}",
            axes.origin(),
            axes.w()
        ))
    };
    // In `[0, 360)`: `rem_euclid` of the smallest negative angle rounds to
    // 360 itself, which is where the turn starts — 0.
    let turned = |degrees: f64| match degrees.rem_euclid(360.0) {
        a if a >= 360.0 => 0.0,
        a => a,
    };
    // Each point seen down the axis, `y` measured the way the turn goes, and
    // its angle there.
    let mut seen = Vec::with_capacity(hull.len());
    for p in hull {
        let q = axes.to_uvw(p);
        let (x, y) = (q[0], q[1].mul(Scalar::from_f64(sign)));
        // A point that could lie on the axis could lie on the edge of every
        // half turn: where a hull touching the axis touches it — the axis an
        // edge of the target, or through a corner of it.
        if x.could_be_equal(Scalar::ZERO) && y.could_be_equal(Scalar::ZERO) {
            continue;
        }
        let angle = turned(y.to_f64().atan2(x.to_f64()).to_degrees());
        seen.push((angle, x, y, p));
    }
    if seen.is_empty() {
        return Err(surrounds(format!(
            "all of the hull's {} points could lie on the axis",
            hull.len()
        )));
    }
    seen.sort_by(|a, b| a.0.total_cmp(&b.0));
    let n = seen.len();
    let (gap, i) = (0..n)
        .map(|i| {
            let next = if i + 1 < n {
                seen[i + 1].0
            } else {
                seen[0].0 + 360.0
            };
            (next - seen[i].0, i)
        })
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .expect("a hull has points");
    // A convex hull clear of the axis fills less than a half turn of
    // directions, and one merely touching it — a solid with a face in a
    // plane through the axis — exactly a half turn: the other half is free
    // either way. Only a hull the axis runs through leaves less: the gap
    // from `a` round to `b` is less than a half turn where `a × b` is
    // definitely positive.
    let (a, b) = (&seen[i], &seen[(i + 1) % n]);
    let across = a.1.mul(b.2).sub(a.2.mul(b.1));
    if gap < 180.0 && across.definitely_greater(Scalar::ZERO) {
        return Err(surrounds(format!(
            "the widest gap between the hull's {n} points seen down the axis, from {:?} at {:?} degrees to {:?} at {:?}, is only {gap:?} degrees",
            a.3, a.0, b.3, b.0,
        )));
    }
    // The arc the solid fills, from `b` to `e`; an end at the start is a
    // full turn round.
    let after = a.0;
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
fn surrounds(detail: String) -> GeopError {
    GeopError::new(format!(
        "up to next: the target lies all around the revolve axis, so no turn short of a full one comes past it; give the angle instead ({detail})",
    ))
}

#[cfg(test)]
mod tests {
    use super::turn_past;
    use geop_core_math::{primitives::CoordinateSystem, scalars::Scalar, vector::Vector3};

    /// A solid with a face in the plane the turn starts from — a revolve
    /// sketched on that face — lies against the start from behind: one of
    /// its points, at an angle a hair below zero, rounds to a whole turn.
    /// The turn must still stop short of one, wherever it goes.
    #[test]
    fn a_target_against_the_start_plane_stops_the_turn_short_of_a_full_one() {
        let axes = CoordinateSystem::world_at(Vector3::zero());
        // The half space below `y = 0`, touching the start plane at `x > 0`
        // — one point a hair below it.
        let hull = [
            [2.0, -1e-17, 0.0],
            [3.0, 0.0, 1.0],
            [-3.0, 0.0, 1.0],
            [0.0, -3.0, 0.0],
            [2.0, -2.0, 1.0],
        ]
        .map(|p| Vector3::from_array(p.map(Scalar::from_f64)));
        for sign in [1.0, -1.0] {
            let far = turn_past(&hull, &axes, sign).unwrap();
            assert!(far > 0.0 && far < 360.0, "{sign}: {far}");
        }
        // Turning forwards, through the free half first, it must come past
        // the half turn where the solid begins.
        assert!(turn_past(&hull, &axes, 1.0).unwrap() > 180.0);
    }
}
