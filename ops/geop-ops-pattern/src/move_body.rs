//! [`MoveBody`]: move bodies, or a copy of them.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Motion, Pose},
    scalars::Scalar,
    vector::Vector3,
};
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, NamedModel, Namer,
    operation::{
        Aspects, BuildReader, EntityRef, Lookup, PartOperation, Role, Scope, Seen, StepContext,
        resolve_bodies,
    },
    part_operation,
    ui::{CanvasEvent, Change, Edit, EditForm, FormState, Gizmo, Number, Unit},
};
use geop_ops_booleans::Combine;
use serde::{Deserialize, Serialize};

use crate::common::{bodies_field, center, combine_instances, copy_seeds, seed_instances};

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

part_operation!(MoveBody);

impl PartOperation for MoveBody {
    const INFO: OperationInfo = OperationInfo::new(
        "move_body",
        "Move body",
        "Move bodies, or a copy of them, turned and shifted.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::MOVE_BODY);
    type Args = MoveBodyArgs;
    type Session = ();

    /// The newest solid, not moved yet.
    fn new_args(context: &StepContext) -> MoveBodyArgs {
        MoveBodyArgs {
            bodies: (Seen::new(context.before).model().newest_solid())
                .map(EntityRef::solid)
                .into_iter()
                .collect(),
            translation: [0.0; 3],
            axis: None,
            angle: 0.0,
            copy: false,
            combine: Combine::NewBody,
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &MoveBodyArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("move", step)?;
        let seeds = resolve_bodies(&args.bodies, &mut &*model)?;
        let motion = args.motion(&mut Scope { model, reader })?;
        let instances = match (args.copy, motion) {
            (true, None) => {
                return Err(GeopError::new(
                    "a copy that is not moved lies on top of the bodies: shift or turn it",
                ));
            }
            (true, Some(motion)) => {
                copy_seeds(model, &seeds, &motion, "copy", |name| namer.name(&[name]))?
            }
            (false, motion) => {
                if let Some(motion) = motion {
                    for &seed in &seeds {
                        model.transform_body(seed, &motion)?;
                    }
                }
                seed_instances(&seeds, "moved")
            }
        };
        combine_instances(model, &namer, step, &args.combine, &instances)
    }

    /// The bodies, picked; the shift along each axis, dragged by a gizmo
    /// where the bodies' middle is moved to; the axis to turn about and the
    /// angle; whether to copy; and how to combine.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &MoveBodyArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, MoveBodyArgs> {
        let seen = Seen::new(context.before);
        let mut f = EditForm::<Design, MoveBodyArgs>::default();
        let shift = Vector3::from_array(args.translation.map(Design::from_f64));
        f.form.gizmo =
            center(seen.model(), &args.bodies).map(|c| Gizmo::new(c.add(&shift)).translate());
        bodies_field(&mut f, &args.bodies, |edit, picked| {
            edit.args.bodies = picked
        });
        for (k, (key, label)) in AXES.into_iter().enumerate() {
            let number = Number::new(label, args.translation[k], Unit::Length);
            f.number(key, number, move |edit, v| edit.args.translation[k] = v);
        }
        let axis = args.axis.iter().cloned().collect();
        let roles = [Role::Line, Role::Round];
        f.reference(
            "axis",
            "turn about",
            axis,
            &roles,
            None,
            false,
            |edit, picked| edit.args.axis = picked.first().cloned(),
        );
        f.form.dialog.optional("axis");
        if args.axis.is_some() {
            let angle = Number::new("angle", args.angle, Unit::Angle).range(-360.0, 360.0);
            f.number("angle", angle, |edit, v| edit.args.angle = v);
        }
        f.checkbox("copy", "copy", args.copy, |edit, b| edit.args.copy = b);
        let set_combine = |args: &mut MoveBodyArgs, combine| args.combine = combine;
        args.combine.fields(&mut f, seen.model(), set_combine);
        f
    }

    /// The gizmo dragged: the shift, as it was when the drag started, moved
    /// by as much.
    fn canvas(_: &StepContext, edit: Edit<'_, MoveBodyArgs, ()>, event: &CanvasEvent<Design>) {
        if let CanvasEvent::Gizmo { drag, .. } = event
            && let Change::Translate(by) = &drag.change
        {
            for (k, shift) in edit.args.translation.iter_mut().enumerate() {
                *shift += by[k].to_f64();
            }
        }
    }
}
