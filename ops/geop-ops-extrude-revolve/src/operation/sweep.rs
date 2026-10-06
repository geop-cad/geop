//! [`Sweep`]: carry a sketch's area along the curves of another sketch, or
//! of a 3-D sketch.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{Chain, EntityRef, Operation},
    ui::{Choice, Form, Number, Unit},
};
use geop_ops_booleans::{Combine, Tool};
use serde::{Deserialize, Serialize};

use super::{
    extrude::{region_loops, shape_loops, sketch_profile},
    path_field, paths_field, sketch_field,
};
use crate::{
    path_sweep::{Control, Orientation, sweep_along},
    sweep::SweepLoop,
};

/// Sweeps the one area of a sketch — the profile — along the curves of
/// another sketch or of a 3-D sketch — the path — into a solid named `sweep(W)` for the
/// operation `W`, kept as a new body or combined with another solid (see
/// [`Combine`]). Or, as a face ([`SweepArgs::face`]), sweeps the profile's
/// curves into faces standing on their own — the area's outline, or a chain
/// of curves enclosing nothing.
///
/// The path is the path sketch's one chain of curves, or its one loop, which
/// sweeps a ring — a 3-D sketch's chain runs through space, and the profile
/// turns with it as little as it can (see [`crate::path_sweep`]). The profile travels along it from where it is drawn,
/// square to the path as it was there (see [`crate::path_sweep`]): draw it
/// at the start of the path, its plane across it. An open path starts at
/// its end nearer the profile, a closed one at its joint nearest the
/// profile. The path is tangent-continuous, except where two lines meet at
/// an angle, which the sweep mitres.
///
/// Named after the profile's elements — `X` a piece of a curve and `P` a
/// joint of the profile sketch `K`, as for [`super::Extrude`] — and the
/// path's — `C` a piece of a curve and `J` a joint of the path sketch `L`:
///
/// - `sweep(W,K,X,L,C)`: the face `X` sweeps along `C`;
/// - `sweep(W,K,X,L,J)`: `X` where the path passes `J`;
/// - `sweep(W,K,P,L,C)` / `sweep(W,K,P,L,J)`: the edge `P` sweeps along `C`,
///   and its vertex at `J`;
/// - `sweep(W,start)` / `sweep(W,end)`: the caps of an open path.
///
/// Along an open path the profile can change as it goes (see
/// [`Control`]): twisted and scaled evenly along the path, or shaped by one
/// or two guide rails — sketches whose one chain of curves starts on the
/// profile's plane, at a point of the profile, and runs along beside the
/// path: that point of the profile follows the rail, the whole profile
/// turned and scaled with it. And it can keep facing the way it is drawn
/// rather than turn with the path ([`Orientation::FixedNormal`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Sweep;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SweepArgs {
    /// The sketch to sweep.
    pub profile: String,
    /// What it is swept along: a sketch's or a 3-D sketch's one chain of
    /// curves, or one loop, or an edge.
    pub path: Option<EntityRef>,
    /// Whether the profile turns with the path or keeps facing one way.
    #[serde(default)]
    pub orientation: Orientation,
    /// How far the profile turns about the path from start to end, in
    /// degrees, counter-clockwise in its sketch.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub twist: f64,
    /// The profile's size at the end, as a multiple of its size where drawn.
    #[serde(default = "one", skip_serializing_if = "is_one")]
    pub end_scale: f64,
    /// What guides the profile — one or two, each a path as the sweep's
    /// own is.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub rails: Vec<EntityRef>,
    /// Sweep the profile's curves into faces standing on their own, rather
    /// than its area into a solid.
    #[serde(default)]
    pub face: bool,
    /// Keep the solid as a new body, or combine it with another solid.
    #[serde(default)]
    pub combine: Combine,
}

fn is_zero(x: &f64) -> bool {
    *x == 0.0
}

fn one() -> f64 {
    1.0
}

fn is_one(x: &f64) -> bool {
    *x == 1.0
}

/// The orientations a sweep offers, by value and label.
const ORIENTATIONS: [(Orientation, &str, &str); 2] = [
    (Orientation::FollowPath, "follow_path", "follow path"),
    (
        Orientation::FixedNormal,
        "fixed_normal",
        "keep normal fixed",
    ),
];

impl Operation for Sweep {
    type Args = SweepArgs;
    type Session = ();

    /// The newest sketch along the newest path — a sketch or a 3-D sketch,
    /// whichever is newer — joined to the newest solid if there is one.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> SweepArgs {
        let mut sketches: Vec<(u64, String)> = before
            .sketches()
            .filter_map(|(id, _)| Some((id.0, before.name_of(id)?.to_string())))
            .collect();
        let newest_3d = before
            .sketches3d()
            .filter_map(|(id, _)| Some((id.0, before.name_of(id)?.to_string())))
            .last();
        let path = match newest_3d {
            Some((id, name)) if sketches.last().is_none_or(|(newest, _)| *newest < id) => {
                Some(EntityRef::Sketch3d { name })
            }
            _ => sketches.pop().map(|(_, name)| EntityRef::Sketch { name }),
        };
        SweepArgs {
            profile: sketches.pop().map(|(_, name)| name).unwrap_or_default(),
            path,
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: Vec::new(),
            face: false,
            combine: Combine::new_for(before),
        }
    }

    /// The profile and the path, picked; the rails, if any; how the profile
    /// turns, twists and scales; whether a face; and how to combine.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &SweepArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, SweepArgs> {
        let before = context.before;
        let mut f = Form::<S, SweepArgs>::new();
        sketch_field(&mut f, before, "profile", &args.profile, |args, sketch| {
            args.profile = sketch
        });
        path_field(&mut f, before, "path", &args.path, |args, path| {
            args.path = path
        });
        paths_field(
            &mut f,
            before,
            "rails",
            "guide rails",
            &args.rails,
            |args, rails| args.rails = rails,
        );
        f.select(
            "orientation",
            "orientation",
            ORIENTATIONS
                .iter()
                .find(|o| o.0 == args.orientation)
                .map_or("", |o| o.1),
            ORIENTATIONS
                .iter()
                .map(|&(_, value, label)| Choice::new(value, label))
                .collect(),
            false,
            |args, value| {
                if let Some(&(orientation, ..)) = ORIENTATIONS.iter().find(|o| o.1 == value) {
                    args.orientation = orientation;
                }
            },
        );
        if args.rails.is_empty() {
            f.number(
                "twist",
                Number::new("twist", args.twist, Unit::Angle),
                |args, v| args.twist = v,
            );
            f.number(
                "end_scale",
                Number::new("end scale", args.end_scale, Unit::Fraction).range(0.0, 4.0),
                |args, v| args.end_scale = v,
            );
        }
        f.checkbox("face", "face", args.face, |args, b| args.face = b);
        if !args.face {
            args.combine.show(&mut f, before, |args| &mut args.combine);
        }
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &SweepArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("sweep({operation_id}, {args:?})");
        let namer = Namer::new("sweep", operation_id)?;
        let profile = EntityRef::Sketch {
            name: args.profile.clone(),
        };
        let path = args
            .path
            .as_ref()
            .ok_or_else(|| GeopError::new("sweep: pick the path to sweep along"))
            .with_context(ctx)?;
        if *path == profile {
            return Err(GeopError::new(
                "sweep: the profile and the path are two different sketches",
            ))
            .with_context(ctx);
        }
        let placed = part
            .sketch(part.sketch_id(&args.profile).with_context(ctx)?)?
            .clone();
        let sketch = &placed.sketch;
        let geometry = sketch.enclose::<S>().with_context(ctx)?;
        let loops = if args.face {
            shape_loops(
                &args.profile,
                sketch,
                &geometry,
                sketch.shape().with_context(ctx)?,
            )
        } else {
            region_loops(
                &args.profile,
                sketch,
                &geometry,
                &sketch.region().with_context(ctx)?,
            )
        }
        .with_context(ctx)?;
        let plane = &placed.plane;
        let chain = path.resolve_chain(&part).with_context(ctx)?;
        let chain = starting_near(chain, &profile_centre(plane, &loops)?)?;
        let mut rails = Vec::new();
        for rail in &args.rails {
            if *rail == profile || rail == path {
                return Err(GeopError::new(format!(
                    "sweep: the rail {rail} is the profile or the path: a rail is a curve of its own"
                )))
                .with_context(ctx);
            }
            rails.push(rail.resolve_chain(&part).with_context(ctx)?);
        }
        let control = Control {
            orientation: args.orientation,
            twist: args.twist.to_radians(),
            end_scale: args.end_scale,
            rails,
        };

        if args.face {
            sweep_along(&mut part, &namer, None, &chain, plane, &loops, &control)
                .with_context(ctx)?;
            return Ok(part);
        }
        let name = args.combine.built_name(&namer);
        let built = sweep_along(
            &mut part,
            &namer,
            Some(&name),
            &chain,
            plane,
            &loops,
            &control,
        )
        .with_context(ctx)?;
        let tool = Tool {
            solid: built.solid.expect("swept as a solid"),
            up_to_next: None,
            scope: None,
        };
        args.combine
            .apply(&mut part, &namer, operation_id, &[tool])
            .with_context(ctx)?;
        Ok(part)
    }
}

/// Where the profile `loops`, drawn in `plane`, is: the centre of its outer
/// loop's joints — a point to find the nearest end of the path from.
fn profile_centre<S: Scalar>(
    plane: &geop_core_math::primitives::CoordinateSystem<S>,
    loops: &[SweepLoop<S>],
) -> GeopResult<[f64; 3]> {
    let outer = &loops
        .first()
        .ok_or_else(|| GeopError::new("sweep: the profile has no curves"))?
        .profile;
    let mut centre = [0.0; 3];
    for curve in &outer.curves {
        let p = plane.uv_to_xyz(&crate::common::start_point(curve)?);
        for (k, c) in centre.iter_mut().enumerate() {
            *c += p[k].to_f64() / outer.curves.len() as f64;
        }
    }
    Ok(centre)
}

/// `chain` starting where it comes nearest to `centre`: an open one at its
/// nearer end, a closed one at its nearest joint. Which is a free choice of
/// where to start; the nearest is what the profile is drawn at.
fn starting_near<S: Scalar>(chain: Chain<S>, centre: &[f64; 3]) -> GeopResult<Chain<S>> {
    let distance =
        |p: &Vector3<S>| -> f64 { (0..3).map(|k| (p[k].to_f64() - centre[k]).powi(2)).sum() };
    let joints = chain.joints()?;
    Ok(if chain.is_closed() {
        let nearest = (0..joints.len())
            .min_by(|&a, &b| distance(&joints[a]).total_cmp(&distance(&joints[b])))
            .expect("a chain has joints");
        chain.starting_at(nearest)
    } else if distance(&joints[joints.len() - 1]) < distance(&joints[0]) {
        chain.reversed()
    } else {
        chain
    })
}
