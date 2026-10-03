//! [`Extrude`]: sweep a sketch's regions into a solid, and how sketch
//! profiles become the named [`Profile`]s extrude and revolve sweep.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_sketch::{ProfilePiece, Sketch, profile::curve_polyline};
use geop_ops::Design;
use geop_ops::{
    Context, Library, Namer, Part,
    operation::Operation,
    ui::{Form, Number, Track, Unit},
};
use geop_ops_booleans::Combine;
use serde::{Deserialize, Serialize};

use super::sketch_field;
use crate::{
    common::Profile,
    extrude::{ExtrudeNames, extrude_from_plane},
};

/// Extrudes every region of a sketch along the sketch plane's normal, into
/// one solid named `extrude(E)` for the operation `E`.
///
/// The faces, edges and vertices are named after the sketch elements they
/// are swept from (see [`ExtrudeNames`]),
/// with `X` a piece of a sketch curve (`c3`, or `c3#1` for the second piece
/// of an arc or circle split into several) and `P` a joint (`p2` for a
/// sketch point, `c3@1` where a curve was split) of the sketch `K`:
///
/// - `extrude(E,start)` / `extrude(E,end)`: the caps, on the sketch plane and
///   `distance` away from it.
/// - `extrude(E,K,X)`: the side face swept by `X`; `extrude(E,K,X,start)` /
///   `extrude(E,K,X,end)` its edges on the two caps.
/// - `extrude(E,K,P)`: the edge swept by `P`; `extrude(E,K,P,start)` /
///   `extrude(E,K,P,end)` its vertices.
///
/// With [`ExtrudeArgs::combine`], the solid can instead be combined with
/// another one, see [`Combine`] — the result is `extrude(E)` either way.
///
/// A sketch of several regions gets one pair of caps per region,
/// `extrude(E,start,K,c)` / `extrude(E,end,K,c)` with `c` the lowest curve id
/// on the region's outer boundary. The regions must not share a curve or a
/// point — every element names what is swept from it, so each can only be
/// swept once.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Extrude;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExtrudeArgs {
    /// The sketch to extrude.
    pub sketch: String,
    /// How far, along the sketch plane's normal; backwards if negative.
    pub distance: f64,
    /// Centre the solid on the sketch plane: extrude half the distance to
    /// either side.
    #[serde(default)]
    pub symmetric: bool,
    /// Keep the solid as a new body, or combine it with another solid.
    #[serde(default)]
    pub combine: Combine,
}

/// Where the distance is dragged: at the centre of the end cap, along the
/// sketch plane's normal. None while the sketch cannot be found.
fn distance_handle<S: Scalar>(before: &Part<S>, args: &ExtrudeArgs) -> Option<Track<S>> {
    let placed = before.sketch(before.sketch_id(&args.sketch).ok()?).ok()?;
    let plane = &placed.plane;
    let center = plane.uv_to_xyz(&sketch_center(&placed.sketch)?);
    // A symmetric extrude's end cap is half the distance off the plane.
    let scale = if args.symmetric { 0.5 } else { 1.0 };
    let normal = *plane.w();
    Some(Track {
        at: center.add(&normal.prod_scalar(S::from_f64(args.distance * scale))),
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
            distance: 1.0,
            symmetric: false,
            combine: Combine::new_for(before),
        }
    }

    /// The sketch, picked; the distance, typed or dragged as a handle —
    /// turning a join into a cut when it crosses down through the sketch
    /// plane, and back when it crosses up (see [`Combine::follow_sign`]);
    /// and how to combine.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &ExtrudeArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ExtrudeArgs> {
        let before = context.before;
        let mut f = Form::<S, ExtrudeArgs>::new();
        sketch_field(&mut f, before, &args.sketch, |args, sketch| {
            args.sketch = sketch
        });
        f.number(
            "distance",
            Number::new("distance", args.distance, Unit::Length)
                .range(-10.0, 10.0)
                .handle(distance_handle(before, args)),
            |args, distance| {
                let from = std::mem::replace(&mut args.distance, distance);
                args.combine.follow_sign(from, distance);
            },
        );
        f.checkbox("symmetric", "symmetric", args.symmetric, |args, b| {
            args.symmetric = b
        });
        args.combine.show(&mut f, before, |args| &mut args.combine);
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
        let distance = S::from_f64(args.distance);
        let plane = if args.symmetric {
            let half = distance.div(S::TWO)?;
            let origin = placed
                .plane
                .origin()
                .sub(&placed.plane.w().prod_scalar(half));
            CoordinateSystem::try_new(
                origin,
                *placed.plane.u(),
                *placed.plane.v(),
                *placed.plane.w(),
            )?
        } else {
            placed.plane.clone()
        };

        let geometry = sketch.enclose::<S>().with_context(ctx)?;
        let regions = sketch.regions().with_context(ctx)?;
        let mut solid = None;
        for region in &regions {
            let outer_pieces = region.outer.to_nurbs(sketch, &geometry)?;
            let region_name = (regions.len() > 1).then(|| {
                let lowest = outer_pieces.iter().map(|p| p.source).min();
                format!("{},{}", args.sketch, lowest.expect("a loop has pieces"))
            });
            let outer = sketch_profile(&args.sketch, outer_pieces, true);
            let holes = region
                .holes
                .iter()
                .map(|h| {
                    Ok(sketch_profile(
                        &args.sketch,
                        h.to_nurbs(sketch, &geometry)?,
                        true,
                    ))
                })
                .collect::<GeopResult<Vec<_>>>()?;
            let names = ExtrudeNames {
                namer: &namer,
                region: region_name.as_deref(),
                // Every region after the first is merged into the first, so
                // its own solid name only exists until then.
                solid: match (&solid, &region_name) {
                    (Some(_), Some(region)) => namer.name(&["solid", region]),
                    _ => args.combine.built_name(&namer),
                },
            };
            let built = extrude_from_plane(&mut part, &names, &plane, &outer, &holes, distance)
                .with_context(ctx)
                .with_context(with_context!(
                    "(a sketch's regions are extruded into one solid and must not share curves or points)"
                ))?;
            match solid {
                None => solid = Some(built),
                Some(first) => part.merge_solids(first, built)?,
            }
        }
        let Some(built) = solid else {
            return Err(GeopError::new("sketch has no region to extrude")).with_context(ctx);
        };
        args.combine
            .apply(&mut part, &namer, operation_id, built)
            .with_context(ctx)?;
        Ok(part)
    }
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
