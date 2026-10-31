//! [`Sweep`]: carry a sketch's area along the curves of another sketch, or
//! of a 3-D sketch.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Choice, Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::geop_error::{GeopError, GeopResult, WithContext};
use geop_core_program::{element::ElementReference, part::StepContext, reader::Reader};
use geop_ops_model::sketch::{PlacedSketch, SketchRef, shape_profiles};
use geop_ops_model::sketch3d::PlacedSketch3d;
use geop_ops_model::{ModelOperation, NamedModel, Namer, build_model, read_model};
use serde::{Deserialize, Serialize};

use geop_lib_sweep::{
    Control, Orientation, Rebuild, SweepLoop, profile_centre, starting_near, sweep_along,
};

use crate::sweeping::{Output, PathRef};

/// Sweeps the one area of a sketch — the profile — along the curves of
/// another sketch or of a 3-D sketch — the path — into a solid named `sweep(W)` for the
/// operation `W`, kept as a new body or combined with another solid (see
/// [`Combine`](geop_ops_model::Combine)). Or, as faces ([`Output::Faces`]), sweeps the profile's
/// curves into faces standing on their own — the area's outline, or a chain
/// of curves enclosing nothing.
///
/// The path is the path sketch's one chain of curves, or its one loop, which
/// sweeps a ring, or an edge of the part — a 3-D sketch's chain runs
/// through space, and the profile turns with it as little as it can. The
/// profile travels along it from where it is drawn, square to the path as
/// it was there (see [`geop_lib_sweep::sweep_along`]): draw it
/// at the start of the path, its plane across it. An open path starts at
/// its end nearer the profile, a closed one at its joint nearest the
/// profile. The path is tangent-continuous, except where two lines meet at
/// an angle, which the sweep mitres.
///
/// Named after the profile's elements — `X` a piece of a curve and `P` a
/// joint of the profile sketch `K`, as for [`crate::Extrude`] — and the
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
    pub profile: SketchRef,
    /// What it is swept along: a sketch's or a 3-D sketch's one chain of
    /// curves, or one loop, or an edge.
    pub path: Option<PathRef>,
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
    pub rails: Vec<PathRef>,
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

part_operation!(Sweep(SweepArgs));

impl PartOperation for Sweep {
    const KIND: &'static str = "sweep";
    const INFO: OperationInfo = OperationInfo::new(
        "Sweep",
        "Sweep a sketch's area along the curves of another sketch into a solid, or its curves into faces.",
        OperationGroup::Solid,
        OperationTier::Small,
    )
    .shortcut("W")
    .icon(crate::icons::SWEEP);
    type Args = SweepArgs;
    type Session = ();

    /// The newest sketch along the sketch before it — joined to the newest
    /// solid if there is one.
    fn new_args(context: &Context) -> SweepArgs {
        let before = &context.before;
        let mut sketches = before.targets_of::<PlacedSketch>();
        // The newest 3-D sketch, which is drawn to be swept along; else
        // the newest sketch.
        let path = match before.targets_of::<PlacedSketch3d>().pop() {
            Some(sketch) => Some(PathRef::Sketch3d(ElementReference::whole(sketch))),
            None => (sketches.pop()).map(|sketch| PathRef::Sketch(ElementReference::whole(sketch))),
        };
        let profile = sketches
            .pop()
            .unwrap_or_else(|| geop_ops_model::sketch::sketch(""));
        SweepArgs {
            profile: SketchRef::whole(profile),
            path,
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: Vec::new(),
            output: Output::new_for(&read_model(before).unwrap_or_default()),
        }
    }

    fn run(ctx: &mut StepContext, args: &SweepArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }
    /// The profile and the path, picked; the rails, if any; how the profile
    /// turns, twists and scales; whether a face; and how to combine.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &SweepArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, SweepArgs> {
        let before = &context.before;
        let mut f = EditForm::<SweepArgs>::default();
        f.pick_one(
            "profile",
            "profile",
            before,
            Some(&args.profile),
            |edit, profile| edit.args.profile = profile,
        )
        .pick_one("path", "path", before, args.path.as_ref(), |edit, path| {
            edit.args.path = Some(path)
        })
        .pick(
            "rails",
            "guide rails",
            before,
            &args.rails,
            true,
            |edit, rails| edit.args.rails = rails,
        );
        f.form.dialog.optional("rails");
        let orientation =
            (ORIENTATIONS.iter().find(|o| o.0 == args.orientation)).map_or("", |o| o.1);
        let orientations = ORIENTATIONS
            .iter()
            .map(|&(_, value, label)| Choice::new(value, label));
        let label = "orientation";
        f.select(
            label,
            label,
            orientation,
            orientations.collect(),
            false,
            |edit, choice| {
                if let Some(&(orientation, ..)) = ORIENTATIONS.iter().find(|o| o.1 == choice) {
                    edit.args.orientation = orientation;
                }
            },
        );
        if args.rails.is_empty() {
            let twist = Number::new("twist", args.twist, Unit::Angle);
            f.number("twist", twist, |edit, v| edit.args.twist = v);
            let end_scale =
                Number::new("end scale", args.end_scale, Unit::Fraction).range(0.0, 4.0);
            f.number("end_scale", end_scale, |edit, v| edit.args.end_scale = v);
        }
        let output = |args: &mut SweepArgs, output| args.output = output;
        (args.output).fields(&mut f, before, false, output);
        f
    }
}

impl ModelOperation for Sweep {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &SweepArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("sweep", step)?;
        let path = (args.path.as_ref())
            .ok_or_else(|| GeopError::new("sweep: pick the path to sweep along"))?;
        if path.is_sketch(&args.profile) {
            return Err(GeopError::new(
                "sweep: the profile and the path are two different sketches",
            ));
        }
        let placed = reader
            .retrieve_element(&args.profile)
            .concerning("profile")?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose()?;
        let shape = args.output.shape(sketch)?;
        let loops: Vec<_> = (shape_profiles(&placed.name, sketch, &geometry, shape)?.into_iter())
            .map(SweepLoop::plain)
            .collect();
        let plane = &placed.plane;
        let chain = path.resolve(reader)?;
        let chain = starting_near(chain, &profile_centre(plane, &loops)?)?;
        let mut rails = Vec::new();
        for rail in &args.rails {
            if rail.is_sketch(&args.profile) || rail == path {
                return Err(GeopError::new(format!(
                    "sweep: the rail {rail:?} is the profile or the path: a rail is a curve of its own"
                )));
            }
            rails.push(rail.resolve(reader)?);
        }
        let control = Control {
            orientation: args.orientation,
            twist: args.twist.to_radians(),
            end_scale: args.end_scale,
            rails,
        };

        (args.output).build_whole(model, &namer, step, |model, name| {
            sweep_along(
                model,
                &namer,
                name,
                &chain,
                plane,
                &loops,
                &control,
                Some(Rebuild {
                    room: geop_lib_shell::room,
                    rebuilt: geop_lib_shell::rebuilt,
                }),
            )
        })
    }
}
