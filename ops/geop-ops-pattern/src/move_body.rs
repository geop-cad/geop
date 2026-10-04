//! [`MoveBody`]: move bodies, or a copy of them, by a turn and a shift.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Motion, Pose},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Form, Number, Unit},
};
use geop_ops_booleans::Combine;
use serde::{Deserialize, Serialize};

use crate::common::{
    axis, bodies_field, center, combine_instances, copy_seeds, newest_solid, seed_instances, seeds,
    track,
};

/// Moves the bodies `bodies` — solids, and sheets by one of their faces —
/// turned by `angle` degrees about `axis`, if any (counter-clockwise
/// looking against it), then shifted by `translation`. Moved, every entity
/// keeps its name; with `copy`, the bodies stay and a moved copy of them is
/// made, the copy of each entity named `X` named `move(M,X)` for the
/// operation `M`.
///
/// What is moved — the copy, or the bodies themselves — is kept as it is,
/// or combined as [`Combine`] says, the result then named `move(M)`: a copy
/// joined to the bodies, or the bodies moved into place and cut from
/// another solid.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MoveBody;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MoveBodyArgs {
    /// The bodies to move: solids, or a face of each sheet.
    pub bodies: Vec<EntityRef>,
    /// The shift, along the world's `x`, `y` and `z`.
    #[serde(default)]
    pub translation: [f64; 3],
    /// What to turn about, before the shift: a line, or a round entity's
    /// axis; none, to shift only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub axis: Option<EntityRef>,
    /// How far to turn about `axis`, in degrees.
    #[serde(default)]
    pub angle: f64,
    /// Move a copy, keeping the bodies where they are.
    #[serde(default)]
    pub copy: bool,
    /// Keep what is moved as it is, or combine it with a solid.
    #[serde(default)]
    pub combine: Combine,
}

impl MoveBodyArgs {
    /// The motion it moves by in `part`: none, if it moves nothing.
    fn motion<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Option<Motion<S>>> {
        let values = self.translation.iter().chain([&self.angle]);
        if let Some(bad) = values.clone().find(|v| !v.is_finite()) {
            return Err(GeopError::new(format!("{bad} is not a number")));
        }
        let shift = Vector3::from_array(self.translation.map(S::from_f64));
        let shifts = self.translation.iter().any(|&t| t != 0.0);
        let turn = match &self.axis {
            Some(entity) if self.angle != 0.0 => {
                let axis = axis(part, entity)?;
                let radians = S::from_f64(self.angle).mul(S::PI).div(S::from_i64(180))?;
                Some(Pose::rotation_about(&axis.point, &axis.direction, radians)?)
            }
            _ => None,
        };
        Ok(match (turn, shifts) {
            (None, false) => None,
            (None, true) => Some(Motion::translation(shift)),
            (Some(turn), false) => Some(turn.motion()),
            (Some(turn), true) => Some(turn.with_position(turn.position().add(&shift)).motion()),
        })
    }
}

/// The keys and labels of the shift's three fields.
const AXES: [(&str, &str); 3] = [("x", "shift x"), ("y", "shift y"), ("z", "shift z")];

impl Operation for MoveBody {
    type Args = MoveBodyArgs;
    type Session = ();

    /// The newest solid, not moved yet.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> MoveBodyArgs {
        MoveBodyArgs {
            bodies: newest_solid(before),
            translation: [0.0; 3],
            axis: None,
            angle: 0.0,
            copy: false,
            combine: Combine::NewBody,
        }
    }

    /// The bodies, picked; the shift along each axis, each dragged by a
    /// handle; the axis to turn about and the angle; whether to copy; and
    /// how to combine.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &MoveBodyArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, MoveBodyArgs> {
        let before = context.before;
        let mut f = Form::<S, MoveBodyArgs>::new();
        bodies_field(&mut f, &args.bodies, |args| &mut args.bodies);
        // Each handle sits where the bodies' middle is moved to, and slides
        // along its own axis.
        let at = center(before, &args.bodies)
            .map(|c| c.add(&Vector3::from_array(args.translation.map(S::from_f64))));
        for (k, (key, label)) in AXES.into_iter().enumerate() {
            let handle = at.and_then(|at| track(at, Vector3::axis(k)));
            f.number(
                key,
                Number::new(label, args.translation[k], Unit::Length).handle(handle),
                move |args, value| args.translation[k] = value,
            );
        }
        f.reference(
            "axis",
            "turn about",
            args.axis.iter().cloned().collect(),
            &[Role::Line, Role::Round],
            None,
            false,
            |edit, picked| edit.args.axis = picked.into_iter().next(),
        );
        f.optional("axis");
        if args.axis.is_some() {
            f.number(
                "angle",
                Number::new("angle", args.angle, Unit::Angle).range(-360.0, 360.0),
                |args, value| args.angle = value,
            );
        }
        f.checkbox("copy", "copy", args.copy, |args, b| args.copy = b);
        args.combine.show(&mut f, before, |args| &mut args.combine);
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &MoveBodyArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("move_body({operation_id}, {args:?})");
        let namer = Namer::new("move", operation_id)?;
        let seeds = seeds(&part, &args.bodies).with_context(ctx)?;
        let motion = args.motion(&part).with_context(ctx)?;
        let instances = match (args.copy, motion) {
            (true, None) => {
                return Err(GeopError::new(
                    "a copy that is not moved lies on top of the bodies: shift or turn it",
                ))
                .with_context(ctx);
            }
            (true, Some(motion)) => copy_seeds(&mut part, &seeds, &motion, "copy", |name| {
                namer.name(&[name])
            })
            .with_context(ctx)?,
            (false, motion) => {
                if let Some(motion) = motion {
                    for &seed in &seeds {
                        part.transform_body(seed, &motion).with_context(ctx)?;
                    }
                }
                seed_instances(&seeds, "moved")
            }
        };
        combine_instances(&mut part, &namer, operation_id, &args.combine, &instances)
            .with_context(ctx)?;
        Ok(part)
    }
}
