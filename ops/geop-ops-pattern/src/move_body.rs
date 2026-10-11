//! [`MoveBody`]: move bodies, or a copy of them.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Motion, Pose},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{
    editing::{CanvasEvent, Change, Event, Form, Gizmo, Number, Unit, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::operation::Input;
use geop_ops::{
    Design, NamedModel, Namer, ParamValue,
    operation::{
        Aspects, Build, BuildReader, EntityRef, Lookup, ModelStep, Role, Scope, Seen, build_model,
        resolve_bodies,
    },
};
use geop_ops_booleans::Combine;
use serde::{Deserialize, Serialize};

use crate::common::{
    bodies_field, center, combine_instances, copy_seeds, newest_solid, seed_instances,
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
    /// The motion it moves by, as `lookup` has the part: none, if it moves
    /// nothing.
    fn motion<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<Option<Motion<S>>> {
        let values = self.translation.iter().chain([&self.angle]);
        if let Some(bad) = values.clone().find(|v| !v.is_finite()) {
            return Err(GeopError::new(format!("{bad} is not a number")));
        }
        let shift = Vector3::from_array(self.translation.map(S::from_f64));
        let shifts = self.translation.iter().any(|&t| t != 0.0);
        let turn = match &self.axis {
            Some(entity) if self.angle != 0.0 => {
                let axis = Aspects::axis_of(entity, lookup)?;
                let radians = S::from_f64(self.angle).degrees_to_radians()?;
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

impl ModelStep for MoveBody {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &MoveBodyArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("move_body({step}, {args:?})");
        let namer = Namer::new("move", step)?;
        let seeds = resolve_bodies(&args.bodies, &mut &*model).with_context(ctx)?;
        let motion = (args.motion(&mut Scope { model, reader })).with_context(ctx)?;
        let instances = match (args.copy, motion) {
            (true, None) => {
                return Err(GeopError::new(
                    "a copy that is not moved lies on top of the bodies: shift or turn it",
                ))
                .with_context(ctx);
            }
            (true, Some(motion)) => {
                copy_seeds(model, &seeds, &motion, "copy", |name| namer.name(&[name]))
                    .with_context(ctx)?
            }
            (false, motion) => {
                if let Some(motion) = motion {
                    for &seed in &seeds {
                        model.transform_body(seed, &motion).with_context(ctx)?;
                    }
                }
                seed_instances(&seeds, "moved")
            }
        };
        combine_instances(model, &namer, step, &args.combine, &instances).with_context(ctx)
    }
}

impl Operation for MoveBody {
    const INFO: OperationInfo = OperationInfo::new(
        "move_body",
        "Move body",
        "Move bodies, or a copy of them, turned and shifted.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    );
    type Args = MoveBodyArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// The newest solid, not moved yet.
    fn new_args(context: &Context<Input, ParamValue>) -> MoveBodyArgs {
        MoveBodyArgs {
            bodies: newest_solid(Seen::new(context.before).model()),
            translation: [0.0; 3],
            axis: None,
            angle: 0.0,
            copy: false,
            combine: Combine::NewBody,
        }
    }

    fn run(step: &str, part: &mut Build, args: &MoveBodyArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The bodies, picked; the shift along each axis, dragged by a gizmo
    /// where the bodies' middle is moved to; the axis to turn about and the
    /// angle; whether to copy; and how to combine.
    fn presentation(
        context: &Context<Input, ParamValue>,
        args: &MoveBodyArgs,
        _: &(),
    ) -> Form<Design> {
        let seen = Seen::new(context.before);
        let mut form = Form::default();
        let shift = Vector3::from_array(args.translation.map(Design::from_f64));
        form.gizmo =
            center(seen.model(), &args.bodies).map(|c| Gizmo::new(c.add(&shift)).translate());
        let dialog = &mut form.dialog;
        bodies_field(dialog, &args.bodies);
        for (k, (key, label)) in AXES.into_iter().enumerate() {
            dialog.number(key, Number::new(label, args.translation[k], Unit::Length));
        }
        let axis = args.axis.iter().cloned().collect();
        (dialog.reference(
            "axis",
            "turn about",
            axis,
            &[Role::Line, Role::Round],
            None,
            false,
        ))
        .optional("axis");
        if args.axis.is_some() {
            let angle = Number::new("angle", args.angle, Unit::Angle).range(-360.0, 360.0);
            dialog.number("angle", angle);
        }
        dialog.checkbox("copy", "copy", args.copy);
        args.combine.show(dialog);
        form
    }

    /// The gizmo dragged: the shift, as it was when the drag started, moved
    /// by as much.
    fn process_user_event(
        context: &Context<Input, ParamValue>,
        args: &MoveBodyArgs,
        _: &(),
        event: Event<Design>,
    ) -> (MoveBodyArgs, ()) {
        let mut args = args.clone();
        match event {
            Event::Canvas(CanvasEvent::Gizmo { drag, .. }) => {
                if let Change::Translate(by) = &drag.change {
                    for (k, shift) in args.translation.iter_mut().enumerate() {
                        *shift += by[k].to_f64();
                    }
                }
            }
            Event::Field { key, value } => match (key.as_str(), &value) {
                ("bodies", Value::Entities(picked)) => args.bodies = picked.clone(),
                ("axis", Value::Entities(picked)) => args.axis = picked.first().cloned(),
                ("angle", Value::Number(v)) => args.angle = *v,
                ("copy", Value::Bool(b)) => args.copy = *b,
                (key, value) => {
                    if let (Some(k), Value::Number(v)) =
                        (AXES.iter().position(|(axis, _)| *axis == key), value)
                    {
                        args.translation[k] = *v;
                    } else {
                        args.combine
                            .set(key, value, Seen::new(context.before).model());
                    }
                }
            },
            Event::Select(_) | Event::Canvas(_) => {}
        }
        (args, ())
    }
}
