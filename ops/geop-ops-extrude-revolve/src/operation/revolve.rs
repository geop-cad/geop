//! [`Revolve`]: sweep a sketch's regions a full turn around a sketch line.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_sketch::{
    CurveId, CurveKind, Positions, ProfileLoop,
    point::{P2, dot, sub},
};
use geop_ops::{
    Namer, Part,
    operation::{EntityRef, Operation},
    ui::{Choice, Dialog, Form, Tone, Value},
};
use geop_ops_booleans::Combine;
use serde::{Deserialize, Serialize};

use super::{extrude::sketch_profile, sketch_field};
use crate::revolve::revolve_at_oriented;

/// Revolves every region of a sketch a full turn around one of its lines,
/// into one solid named `revolve(R)` for the operation `R` — or, with
/// [`RevolveArgs::combine`], combines that with another solid (see
/// [`Combine`]), the result named `revolve(R)` all the same.
///
/// Each region must touch the axis along an edge — a line whose endpoints the
/// constraints put on the axis (see `Sketch::on_line`), typically the axis
/// line itself — and lie entirely on one side of it, the same side for every
/// region. The rest of its boundary is the profile that sweeps out the
/// solid, and names it (see
/// [`revolve_at_oriented`]): with `X` a
/// piece of a sketch curve and `P` a joint of the sketch `K`, as for
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
    /// The line of that sketch to revolve around. The profile must touch it
    /// along an edge.
    pub axis: CurveId,
    /// Keep the solid as a new body, or combine it with another solid.
    #[serde(default)]
    pub combine: Combine,
}

/// The lines of the sketch named `sketch` in `part`: `(id, construction)`.
fn lines<S: Scalar>(part: &Part<S>, sketch: &str) -> Vec<(CurveId, bool)> {
    let Some(placed) = part.sketch_id(sketch).and_then(|id| part.sketch(id)).ok() else {
        return Vec::new();
    };
    placed
        .sketch
        .curves
        .iter()
        .filter(|(_, c)| matches!(c.kind, CurveKind::Line { .. }))
        .map(|(&id, c)| (id, c.construction))
        .collect()
}

/// The line of `sketch` to revolve around by default: its first
/// construction line — what an axis is usually drawn as — else its first
/// line.
fn default_axis<S: Scalar>(part: &Part<S>, sketch: &str) -> CurveId {
    let lines = lines(part, sketch);
    lines
        .iter()
        .find(|(_, construction)| *construction)
        .or(lines.first())
        .map_or(CurveId(0), |&(id, _)| id)
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

    /// The sketch, picked, and the axis among its lines.
    fn form<S: Scalar>(&self, before: &Part<S>, args: &RevolveArgs, _: &()) -> Form<S> {
        let mut d = Dialog::new();
        sketch_field(&mut d, before, &args.sketch);
        let lines = lines(before, &args.sketch);
        if lines.is_empty() {
            d.text(
                "axis",
                "The sketch has no line to revolve around.",
                Tone::Hint,
            );
        } else {
            d.select(
                "axis",
                "axis",
                args.axis.0.to_string(),
                lines
                    .iter()
                    .map(|(id, construction)| {
                        let label = if *construction {
                            format!("Line {id} (construction)")
                        } else {
                            format!("Line {id}")
                        };
                        Choice::new(id.0.to_string(), label)
                    })
                    .collect(),
            );
        }
        args.combine.show(&mut d);
        Form::dialog(d)
    }

    /// A sketch picked brings its axis back to its default.
    fn set<S: Scalar>(
        &self,
        before: &Part<S>,
        args: &mut RevolveArgs,
        _: &mut (),
        key: &str,
        value: Value,
    ) {
        if args.combine.set(before, key, &value) {
            return;
        }
        match (key, value) {
            ("sketch", Value::Entity(EntityRef::Sketch { name })) => {
                args.axis = default_axis(before, &name);
                args.sketch = name;
            }
            ("axis", Value::Choice(id)) => {
                if let Ok(id) = id.parse() {
                    args.axis = CurveId(id);
                }
            }
            _ => {}
        }
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &RevolveArgs,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("revolve({operation_id}, {args:?})");
        let namer = Namer::new("revolve", operation_id)?;
        let placed = part
            .sketch(part.sketch_id(&args.sketch).with_context(ctx)?)?
            .clone();
        let sketch = &placed.sketch;
        let CurveKind::Line { start, end } = sketch.curve(args.axis).with_context(ctx)?.kind else {
            return Err(GeopError::new(format!(
                "revolve axis {} is not a line",
                args.axis
            )))
            .with_context(ctx);
        };
        let positions = sketch.positions();
        let a = positions[&start];
        let b = positions[&end];
        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
        let dir = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
        let left = [-dir[1], dir[0]];
        let on_axis = sketch.on_line(args.axis)?;

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
            // Which side of the axis the region lies on, from its outline; it
            // must not cross. Vertices on the axis sit within rounding of it on
            // either side, so crossing means reaching measurably across,
            // relative to the region's extent — a classification of design
            // data, like the nesting test in `Sketch::regions`.
            let outline = region.outer.polyline(sketch, &positions);
            let side = |p: &P2| dot(sub(*p, a), left);
            let (lo, hi) = outline
                .iter()
                .map(side)
                .fold((0.0f64, 0.0f64), |(lo, hi), s| (lo.min(s), hi.max(s)));
            let scale = hi - lo;
            if lo < -1e-9 * scale && hi > 1e-9 * scale {
                return Err(GeopError::new("the profile crosses the revolve axis"))
                    .with_context(ctx);
            }
            let sign = if hi > -lo { 1.0 } else { -1.0 };
            sides.push(sign);
            if sides.iter().any(|&s| s != sign) {
                return Err(GeopError::new(
                    "the sketch has regions on both sides of the revolve axis, which would overlap \
                     once revolved; revolve them in separate operations",
                ))
                .with_context(ctx);
            }

            // `(r, z)` coordinates: `r` towards the region, `z` along the axis,
            // oriented like the sketch (`z` is `r` turned counter-clockwise), so
            // loops keep their winding. Points the constraints put on the axis
            // get `r = 0` exactly — the solver only approaches it.
            let r_dir = [sign * left[0], sign * left[1]];
            let z_dir = [-r_dir[1], r_dir[0]];
            let rz: Positions = positions
                .iter()
                .map(|(&p, xy)| {
                    let d = [xy[0] - a[0], xy[1] - a[1]];
                    let r = if on_axis.contains(&p) {
                        0.0
                    } else {
                        d[0] * r_dir[0] + d[1] * r_dir[1]
                    };
                    (p, [r, d[0] * z_dir[0] + d[1] * z_dir[1]])
                })
                .collect();

            // The profile is the outer loop minus its run of edges on the
            // axis, walked top-down (see `revolve_at_oriented`): the loop is
            // counter-clockwise, so the region lies left of it, and reversing
            // puts it on the right.
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
            if (0..n).filter(|&k| starts_off_axis(k)).count() != 1 {
                return Err(GeopError::new(
                    "the profile must touch the revolve axis along exactly one run of edges \
                     (constrain its edge onto the axis line)",
                ))
                .with_context(ctx);
            }
            let first_off = (0..n).find(|&k| starts_off_axis(k)).unwrap();
            let chain = ProfileLoop {
                edges: (0..n)
                    .map(|k| (first_off + k) % n)
                    .take_while(|&k| !flags[k])
                    .map(|k| edges[k])
                    .collect(),
            }
            .reversed();
            let profile = sketch_profile(
                &args.sketch,
                chain.to_nurbs(sketch, &rz).with_context(ctx)?,
                false,
            );

            let plane = &placed.plane;
            let dir3 = |d: P2| {
                plane
                    .u()
                    .prod_scalar(S::from_f64(d[0]))
                    .add(&plane.v().prod_scalar(S::from_f64(d[1])))
            };
            let (u, w) = (dir3(r_dir), dir3(z_dir));
            let origin =
                plane.uv_to_xyz(&Vector2::from_array([S::from_f64(a[0]), S::from_f64(a[1])]));
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
