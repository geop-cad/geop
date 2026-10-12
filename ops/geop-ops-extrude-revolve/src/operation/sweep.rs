//! [`Sweep`]: carry a sketch's area along the curves of another sketch, or
//! of a 3-D sketch.

use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{
    editing::{Choice, Event, Form, Number, Unit, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::operation::Input;
use geop_ops::{
    Design, NamedModel, Namer, ParamValue,
    operation::{
        Build, BuildReader, Chain, EntityRef, Lookup, LegacyModelStep, ResolveEntity, Role, Scope, Seen,
        legacy_build_model,
    },
    ui::sketch_field,
};
use serde::{Deserialize, Serialize};

use super::{Output, extrude::shape_loops};
use crate::{
    loft::centre,
    path_sweep::{Control, Orientation, sweep_along},
    sweep::SweepLoop,
};

/// Sweeps the one area of a sketch — the profile — along the curves of
/// another sketch or of a 3-D sketch — the path — into a solid named `sweep(W)` for the
/// operation `W`, kept as a new body or combined with another solid (see
/// [`Combine`](geop_ops_booleans::Combine)). Or, as faces ([`Output::Faces`]), sweeps the profile's
/// curves into faces standing on their own — the area's outline, or a chain
/// of curves enclosing nothing.
///
/// The path is the path sketch's one chain of curves, or its one loop, which
/// sweeps a ring, or an edge of the part — a 3-D sketch's chain runs
/// through space, and the profile turns with it as little as it can. The
/// profile travels along it from where it is drawn, square to the path as
/// it was there (see [`crate::path_sweep`]): draw it
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
    /// A solid, kept as a new body or combined with another, or faces.
    #[serde(default)]
    pub output: Output,
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
    const INFO: OperationInfo = OperationInfo::new(
        "sweep",
        "Sweep",
        "Sweep a sketch's area along the curves of another sketch into a solid, or its curves into faces.",
        OperationGroup::Solid,
        OperationTier::Small,
    )
    .icon(crate::icons::SWEEP);
    type Args = SweepArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// The newest sketch along the sketch before it — joined to the newest
    /// solid if there is one.
    fn new_args(context: &Context<Input, ParamValue>) -> SweepArgs {
        let seen = Seen::new(context.before);
        let mut sketches = seen.sketch_names();
        // The newest 3-D sketch, which is drawn to be swept along; else
        // the newest sketch.
        let path = match seen.sketch3d_names().pop() {
            Some(name) => Some(EntityRef::sketch3d(name)),
            None => sketches.pop().map(EntityRef::sketch),
        };
        SweepArgs {
            profile: sketches.pop().unwrap_or_default(),
            path,
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: Vec::new(),
            output: Output::new_for(seen.model()),
        }
    }

    fn run(step: &str, part: &mut Build, args: &SweepArgs) -> GeopResult<()> {
        legacy_build_model::<Self>(part, step, args)
    }

    /// The profile and the path, picked; the rails, if any; how the profile
    /// turns, twists and scales; whether a face; and how to combine.
    fn presentation(
        context: &Context<Input, ParamValue>,
        args: &SweepArgs,
        _: &(),
    ) -> Form<Design> {
        let view = &context.before;
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        sketch_field(
            dialog,
            view,
            "profile",
            std::slice::from_ref(&args.profile),
            false,
        );
        let path = args.path.iter().cloned().collect();
        (dialog.reference("path", "path", path, &[Role::Path], None, false))
            .reference(
                "rails",
                "guide rails",
                args.rails.clone(),
                &[Role::Path],
                None,
                true,
            )
            .optional("rails");
        let orientation =
            (ORIENTATIONS.iter().find(|o| o.0 == args.orientation)).map_or("", |o| o.1);
        let orientations = ORIENTATIONS
            .iter()
            .map(|&(_, value, label)| Choice::new(value, label));
        dialog.select(
            "orientation",
            "orientation",
            orientation,
            orientations.collect(),
            false,
        );
        if args.rails.is_empty() {
            dialog.number("twist", Number::new("twist", args.twist, Unit::Angle));
            let end_scale =
                Number::new("end scale", args.end_scale, Unit::Fraction).range(0.0, 4.0);
            dialog.number("end_scale", end_scale);
        }
        args.output.show(dialog, false);
        form
    }

    fn process_user_event(
        context: &Context<Input, ParamValue>,
        args: &SweepArgs,
        _: &(),
        event: Event<Design>,
    ) -> (SweepArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), value) {
            ("profile", Value::Entities(picked)) => {
                args.profile = EntityRef::last_name(EntityRef::sketch, &picked).unwrap_or_default();
            }
            ("path", Value::Entities(picked)) => args.path = picked.into_iter().next(),
            ("rails", Value::Entities(picked)) => args.rails = picked,
            ("orientation", Value::Choice(choice)) => {
                if let Some(&(orientation, ..)) = ORIENTATIONS.iter().find(|o| o.1 == choice) {
                    args.orientation = orientation;
                }
            }
            ("twist", Value::Number(v)) => args.twist = v,
            ("end_scale", Value::Number(v)) => args.end_scale = v,
            (key, value) => {
                (args.output).set(key, &value, Seen::new(context.before).model());
            }
        }
        (args, ())
    }
}

impl LegacyModelStep for Sweep {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &SweepArgs,
    ) -> GeopResult<()> {
        type S = Design;
        let ctx = with_context!("sweep({step}, {args:?})");
        let namer = Namer::new("sweep", step)?;
        let profile = EntityRef::sketch(args.profile.clone());
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
        let mut scope = Scope {
            model: &mut *model,
            reader,
        };
        let placed = scope.sketch(&args.profile).with_context(ctx)?.clone();
        let sketch = &placed.sketch;
        let geometry = sketch.enclose::<S>().with_context(ctx)?;
        let shape = args.output.shape(sketch).with_context(ctx)?;
        let loops = shape_loops(&args.profile, sketch, &geometry, shape).with_context(ctx)?;
        let plane = &placed.plane;
        let chain = path.resolve_chain(&mut scope).with_context(ctx)?;
        let chain = starting_near(chain, &profile_centre(plane, &loops)?)?;
        let mut rails = Vec::new();
        for rail in &args.rails {
            if *rail == profile || rail == path {
                return Err(GeopError::new(format!(
                    "sweep: the rail {rail} is the profile or the path: a rail is a curve of its own"
                )))
                .with_context(ctx);
            }
            rails.push(rail.resolve_chain(&mut scope).with_context(ctx)?);
        }
        let control = Control {
            orientation: args.orientation,
            twist: args.twist.to_radians(),
            end_scale: args.end_scale,
            rails,
        };

        (args.output)
            .build_whole(model, &namer, step, |model, name| {
                sweep_along(model, &namer, name, &chain, plane, &loops, &control)
            })
            .with_context(ctx)
    }
}

/// Where the profile `loops`, drawn in `plane`, is: the centre of its outer
/// loop's joints — a point to find the nearest end of the path from.
fn profile_centre<S: Scalar>(
    plane: &CoordinateSystem<S>,
    loops: &[SweepLoop<S>],
) -> GeopResult<[f64; 3]> {
    let outer = &loops
        .first()
        .ok_or_else(|| GeopError::new("sweep: the profile has no curves"))?
        .profile;
    let starts = outer
        .curves
        .iter()
        .map(|curve| Ok(plane.uv_to_xyz(&curve.start()?).to_f64_array()))
        .collect::<GeopResult<Vec<_>>>()?;
    Ok(centre(&starts))
}

/// `chain` starting where it comes nearest to `centre`: an open one at its
/// nearer end, a closed one at its nearest joint. Which is a free choice of
/// where to start; the nearest is what the profile is drawn at.
fn starting_near<S: Scalar>(
    chain: Chain<NurbCurve3D<S>>,
    centre: &[f64; 3],
) -> GeopResult<Chain<NurbCurve3D<S>>> {
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
