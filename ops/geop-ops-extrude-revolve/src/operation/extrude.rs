//! [`Extrude`]: sweep a sketch's area along its plane's normal, and how a
//! sketch becomes the named [`Profile`]s extrude and revolve sweep.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_sketch::{
    Enclosure, ProfileLoop, ProfilePiece, Region, Shape, Sketch, profile::curve_polyline,
};
use geop_ops::Design;
use geop_ops::{
    Context, Library, Namer, Part,
    operation::Operation,
    ui::{Form, Number, Track, Unit},
};
use geop_ops_booleans::{Combine, Tool};
use serde::{Deserialize, Serialize};

use super::{
    Extent, Extents, Plan, combine_sides, face_target_field, hull, no_target, side_namer,
    side_tool, sketch_field, stops, trim_side,
};
use crate::{common::Profile, extrude::extrude, sweep::SweepLoop};

/// Extrudes the one area of a sketch along the sketch plane's normal, into
/// a solid named `extrude(E)` for the operation `E` — kept as a new body,
/// or combined with another solid, see [`Combine`]; the result is
/// `extrude(E)` either way. Or, as a face ([`ExtrudeArgs::face`]), sweeps
/// the sketch's curves into faces standing on their own — the area's
/// outline, or a chain of curves enclosing nothing.
///
/// How far it goes is an [`Extents`]: a length, or up to the next face of
/// the solid it is joined to or cut from, on one side of the plane, both
/// alike, or each its own way.
///
/// The faces, edges and vertices are named after the sketch elements they
/// are swept from (see [`crate::extrude::extrude`]), with `X` a piece of a
/// sketch curve (`c3`, or `c3#1` for the second piece of an arc or circle
/// split into several) and `P` a joint (`p2` for a sketch point, `c3@1`
/// where a curve was split) of the sketch `K`:
///
/// - `extrude(E,start)` / `extrude(E,end)`: the caps, at either end;
/// - `extrude(E,K,X)`: the side face swept by `X`; `extrude(E,K,X,start)` /
///   `extrude(E,K,X,end)` its edges on the two caps;
/// - `extrude(E,K,P)`: the edge swept by `P`; `extrude(E,K,P,start)` /
///   `extrude(E,K,P,end)` its vertices.
///
/// A side going up to the next face is built on its own, from the sketch's
/// plane — its caps `start` on the plane — and the second of two sides
/// built so is named within `side2`: `extrude(E,side2,K,X)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Extrude;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExtrudeArgs {
    /// The sketch to extrude.
    pub sketch: String,
    /// How far, along the sketch plane's normal — backwards for a negative
    /// length.
    pub extent: Extents,
    /// Sweep the sketch's curves into faces standing on their own, rather
    /// than its area into a solid.
    #[serde(default)]
    pub face: bool,
    /// Keep the solid as a new body, or combine it with another solid.
    #[serde(default)]
    pub combine: Combine,
}

/// Where a side's length is dragged: at the centre of the sketch, that far
/// along the sketch plane's normal — backwards for the second side. None
/// while the sketch cannot be found.
fn distance_handle<S: Scalar>(
    before: &Part<S>,
    args: &ExtrudeArgs,
    length: f64,
    second: bool,
) -> Option<Track<S>> {
    let placed = before.sketch(before.sketch_id(&args.sketch).ok()?).ok()?;
    let plane = &placed.plane;
    let center = plane.uv_to_xyz(&sketch_center(&placed.sketch)?);
    // A symmetric extrude goes half its length either way; the second side
    // the other way.
    let scale = match (args.extent.symmetric, second) {
        (true, _) => 0.5,
        (false, false) => 1.0,
        (false, true) => -1.0,
    };
    let normal = *plane.w();
    Some(Track {
        at: center.add(&normal.prod_scalar(S::from_f64(length * scale))),
        direction: normal.prod_scalar(S::from_f64(scale)),
    })
}

impl Operation for Extrude {
    type Args = ExtrudeArgs;
    type Session = ();

    /// The newest sketch, a unit up, joined to the newest solid if there is
    /// one.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> ExtrudeArgs {
        ExtrudeArgs {
            sketch: before.sketch_names().pop().unwrap_or_default(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::new_for(before),
        }
    }

    /// The sketch, picked; how far, each length typed or dragged as a
    /// handle — the first turning a join into a cut when it crosses down
    /// through the sketch plane, and back when it crosses up (see
    /// [`Combine::follow_sign`]); whether a face; and how to combine.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &ExtrudeArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ExtrudeArgs> {
        let before = context.before;
        let mut f = Form::<S, ExtrudeArgs>::new();
        sketch_field(&mut f, before, "sketch", &args.sketch, |args, sketch| {
            args.sketch = sketch
        });
        args.extent.show(
            &mut f,
            "distance",
            |length, second| {
                let label = if second { "distance 2" } else { "distance" };
                Number::new(label, length, Unit::Length)
                    .range(-10.0, 10.0)
                    .handle(distance_handle(before, args, length, second))
            },
            1.0,
            |args| &mut args.extent,
            |args, length| {
                let from = match std::mem::replace(&mut args.extent.side1, Extent::Blind(length)) {
                    Extent::Blind(from) => from,
                    Extent::UpToNext | Extent::ThroughAll => length,
                };
                args.combine.follow_sign(from, length);
            },
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
        args: &ExtrudeArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("extrude({operation_id}, {args:?})");
        let namer = Namer::new("extrude", operation_id)?;
        let placed = part
            .sketch(part.sketch_id(&args.sketch).with_context(ctx)?)?
            .clone();
        let sketch = &placed.sketch;
        let plane = &placed.plane;
        let geometry = sketch.enclose::<S>().with_context(ctx)?;
        let loops = if args.face {
            shape_loops(
                &args.sketch,
                sketch,
                &geometry,
                sketch.shape().with_context(ctx)?,
            )
        } else {
            region_loops(
                &args.sketch,
                sketch,
                &geometry,
                &sketch.region().with_context(ctx)?,
            )
        }
        .with_context(ctx)?;
        let s = S::from_f64;
        let stops = stops(&part, &args.combine);
        let hull = if args.extent.reaches_target() {
            hull(&part, &stops).with_context(ctx)?
        } else {
            None
        };
        let plan = args
            .extent
            .plan(|sign| match &hull {
                Some(hull) => reach_past(hull, plane, sign),
                None => Err(no_target()),
            })
            .with_context(ctx)?;

        match (args.face, plan) {
            (true, Plan::Whole(from, to)) => {
                extrude(&mut part, &namer, None, plane, s(from), s(to), &loops)
                    .with_context(ctx)?;
            }
            (true, Plan::Sides(sides)) => {
                for (k, side) in sides.into_iter().enumerate() {
                    let named = side_namer(&namer, k);
                    let built =
                        extrude(&mut part, &named, None, plane, S::ZERO, s(side.to), &loops)
                            .with_context(ctx)?;
                    if side.up_to_next {
                        trim_side(
                            &mut part,
                            operation_id,
                            k,
                            &built,
                            &stops,
                            (&named, &loops),
                            ("start", "end"),
                        )
                        .with_context(ctx)?;
                    }
                }
            }
            (false, Plan::Whole(from, to)) => {
                let name = args.combine.built_name(&namer);
                let built = extrude(
                    &mut part,
                    &namer,
                    Some(&name),
                    plane,
                    s(from),
                    s(to),
                    &loops,
                )
                .with_context(ctx)?;
                let tool = Tool {
                    solid: built.solid.expect("extruded as a solid"),
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
                    let built = extrude(
                        &mut part,
                        &named,
                        Some(&name),
                        plane,
                        S::ZERO,
                        s(side.to),
                        &loops,
                    )
                    .with_context(ctx)?;
                    let solid = built.solid.expect("extruded as a solid");
                    tools.push(side_tool(&named, k, solid, side));
                }
                let normal = *plane.w();
                let origin = *plane.origin();
                let far: Vec<f64> = sides.iter().map(|side| side.to.abs()).collect();
                combine_sides(
                    &mut part,
                    &namer,
                    operation_id,
                    &args.combine,
                    tools,
                    &stops,
                    &far,
                    &far,
                    |part, k, from, to| {
                        let named = side_namer(&namer, k);
                        let name = args.combine.built_name(&named);
                        let sign = sides[k].to.signum();
                        let built = extrude(
                            part,
                            &named,
                            Some(&name),
                            plane,
                            s(from * sign),
                            s(to * sign),
                            &loops,
                        )?;
                        Ok(built.solid.expect("extruded as a solid"))
                    },
                    |k, p| {
                        let sign = s(sides[k].to.signum());
                        let along = p.sub(&origin).prod_dot(&normal).mul(sign);
                        along.definitely_greater(S::ZERO).then(|| along.to_f64())
                    },
                )
                .with_context(ctx)?;
            }
        }
        Ok(part)
    }
}

/// How far along `plane`'s normal — forwards for `sign = 1`, backwards for
/// `-1` — reaches past everything of the solid whose convex hull `hull`
/// spans: a side going up to its next face has to be built at least that
/// long to meet whichever face that is.
fn reach_past<S: Scalar>(
    hull: &[[f64; 3]],
    plane: &CoordinateSystem<S>,
    sign: f64,
) -> GeopResult<f64> {
    let o = [0, 1, 2].map(|k| plane.origin()[k].to_f64());
    let w = [0, 1, 2].map(|k| plane.w()[k].to_f64());
    let along = |p: &[f64; 3]| sign * (0..3).map(|k| (p[k] - o[k]) * w[k]).sum::<f64>();
    let ahead = hull.iter().map(along).fold(f64::NEG_INFINITY, f64::max);
    if ahead <= 0.0 {
        return Err(GeopError::new(format!(
            "up to next: nothing of the target lies {} the sketch, so there is no next face",
            if sign > 0.0 { "in front of" } else { "behind" }
        )));
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in hull {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let diagonal = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
    // Well past the target, so the tool's end is clear of all of it.
    Ok(ahead + diagonal)
}

/// The loops of the sketch `name`'s `region` to sweep, outer loop first,
/// with the sketch's points at `geometry`.
pub(crate) fn region_loops<S: Scalar>(
    name: &str,
    sketch: &Sketch<Design>,
    geometry: &Enclosure<S>,
    region: &Region,
) -> GeopResult<Vec<SweepLoop<S>>> {
    std::iter::once(&region.outer)
        .chain(&region.holes)
        .map(|lp| {
            Ok(SweepLoop::plain(sketch_profile(
                name,
                lp.to_nurbs(sketch, geometry)?,
                true,
            )))
        })
        .collect()
}

/// The loops of `shape` to sweep into a sheet: the region's, or the one
/// open chain.
pub fn shape_loops<S: Scalar>(
    name: &str,
    sketch: &Sketch<Design>,
    geometry: &Enclosure<S>,
    shape: Shape,
) -> GeopResult<Vec<SweepLoop<S>>> {
    match shape {
        Shape::Region(region) => region_loops(name, sketch, geometry, &region),
        Shape::Chain(chain) => Ok(vec![SweepLoop::plain(chain_profile(
            name, sketch, geometry, &chain,
        )?)]),
    }
}

/// The open `chain` of the sketch `name` as a profile.
fn chain_profile<S: Scalar>(
    name: &str,
    sketch: &Sketch<Design>,
    geometry: &Enclosure<S>,
    chain: &ProfileLoop,
) -> GeopResult<Profile<S>> {
    Ok(sketch_profile(
        name,
        chain.to_nurbs(sketch, geometry)?,
        false,
    ))
}

/// The pieces of a sketch loop or chain as a [`Profile`] named after the
/// sketch `sketch` and its elements: `sketch,c3` for a piece, `sketch,p2`
/// for a joint. An open chain also names its end joint.
pub(crate) fn sketch_profile<S: Scalar>(
    sketch: &str,
    pieces: Vec<ProfilePiece<S>>,
    closed: bool,
) -> Profile<S> {
    let curve_names = pieces
        .iter()
        .map(|p| format!("{sketch},{}", p.name()))
        .collect();
    let mut joint_names: Vec<String> = pieces
        .iter()
        .map(|p| format!("{sketch},{}", p.start))
        .collect();
    if !closed && let Some(last) = pieces.last() {
        joint_names.push(format!("{sketch},{}", last.end));
    }
    Profile {
        curves: pieces.into_iter().map(|p| p.curve).collect(),
        curve_names,
        joint_names,
    }
}

/// The centre of the box around what `sketch` draws — its profile curves,
/// or its points if it has none — in sketch coordinates.
fn sketch_center<S: Scalar>(sketch: &Sketch<Design>) -> Option<Vector2<S>> {
    let mut drawn: Vec<Vector2<Design>> = sketch
        .curves
        .iter()
        .filter(|(_, c)| !c.construction)
        .flat_map(|(&id, _)| curve_polyline(sketch, id).unwrap_or_default())
        .collect();
    if drawn.is_empty() {
        drawn = sketch.positions().into_values().collect();
    }
    let hull = drawn.into_iter().reduce(|a, b| a.union(&b))?;
    // The middle of the box is a free choice: sharp.
    Some(hull.map(|c| c.cast::<S>().midpoint()))
}
