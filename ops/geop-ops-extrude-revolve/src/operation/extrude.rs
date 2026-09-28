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
use geop_ops::{
    Namer, Part,
    operation::{Handle, HandleGroup, HandleMotion, Operation, OperationArgs, arg_path, to_f64},
};
use geop_ops_booleans::Combine;
use serde::{Deserialize, Serialize};

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

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, OperationArgs)]
pub struct ExtrudeArgs {
    /// The sketch to extrude.
    #[arg(Sketch)]
    pub sketch: String,
    /// How far, along the sketch plane's normal; backwards if negative.
    #[arg(Number { default: 1.0, min: -10.0, max: 10.0 })]
    pub distance: f64,
    /// Centre the solid on the sketch plane: extrude half the distance to
    /// either side.
    #[serde(default)]
    #[arg(Bool { default: false })]
    pub symmetric: bool,
    /// Keep the solid as a new body, or combine it with another solid.
    #[serde(default)]
    #[arg(Combine { sign: Some("distance") })]
    pub combine: Combine,
}

impl<S: Scalar> Operation<S> for Extrude {
    type Args = ExtrudeArgs;

    fn apply(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &ExtrudeArgs,
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

        let positions = sketch.positions();
        let regions = sketch.regions().with_context(ctx)?;
        let mut solid = None;
        for region in &regions {
            let outer_pieces = region.outer.to_nurbs::<S>(sketch, &positions)?;
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
                        h.to_nurbs(sketch, &positions)?,
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

    /// The distance, as a handle at the centre of the end cap that slides
    /// along the sketch plane's normal.
    fn handles(&self, before: &Part<S>, args: &ExtrudeArgs) -> GeopResult<Vec<Handle>> {
        let placed = before.sketch(before.sketch_id(&args.sketch)?)?;
        let Some(center) = sketch_center(&placed.sketch) else {
            return Ok(Vec::new());
        };
        let plane = &placed.plane;
        let at = plane.uv_to_xyz(&Vector2::from_array(center.map(S::from_f64)));
        let normal = to_f64(plane.w());
        // A symmetric extrude's end cap is half the distance off the plane.
        let scale = if args.symmetric { 0.5 } else { 1.0 };
        let offset = args.distance * scale;
        let at = to_f64(&at);
        Ok(vec![Handle {
            label: "distance".into(),
            group: HandleGroup::Feature,
            position: [0, 1, 2].map(|k| at[k] + normal[k] * offset),
            motion: HandleMotion::Linear {
                direction: normal,
                arg: arg_path(&["distance"]),
                value: args.distance,
                scale,
            },
        }])
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
fn sketch_center(sketch: &Sketch) -> Option<[f64; 2]> {
    let positions = sketch.positions();
    let mut drawn: Vec<[f64; 2]> = sketch
        .curves
        .iter()
        .filter(|(_, c)| !c.construction)
        .flat_map(|(&id, _)| curve_polyline(sketch, &positions, id))
        .collect();
    if drawn.is_empty() {
        drawn = positions.into_values().collect();
    }
    let (lo, hi) = drawn.iter().fold(
        ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]),
        |(lo, hi), p| {
            (
                [lo[0].min(p[0]), lo[1].min(p[1])],
                [hi[0].max(p[0]), hi[1].max(p[1])],
            )
        },
    );
    (!drawn.is_empty()).then(|| [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0])
}
