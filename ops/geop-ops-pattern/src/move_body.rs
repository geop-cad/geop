//! [`MoveBody`]: move bodies, or a copy of them.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{CanvasEvent, Change, Gizmo, Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Motion, Pose},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{part::StepContext, reader::Reader, values::Design};
use geop_ops_booleans::Combine;
use geop_ops_model::{ModelOperation, NamedModel, Namer, build_model, read_model, resolve_bodies};
use serde::{Deserialize, Serialize};

use crate::{
    common::{BodyRef, bodies_field, body, center, combine_instances, copy_seeds, seed_instances},
    references::TurnAxis,
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
    pub bodies: Vec<BodyRef>,
    /// The shift, along the world's `x`, `y` and `z`.
    #[serde(default)]
    pub translation: [f64; 3],
    /// What to turn about, before the shift: a line, or a round entity's
    /// axis; none, to shift only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub axis: Option<TurnAxis>,
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
    /// The motion it moves by, as `reader` reads the part: none, if it
    /// moves nothing.
    fn motion(&self, reader: &Reader) -> GeopResult<Option<Motion<Design>>> {
        type S = Design;
        let values = self.translation.iter().chain([&self.angle]);
        if let Some(bad) = values.clone().find(|v| !v.is_finite()) {
            return Err(GeopError::new(format!("{bad} is not a number")));
        }
        let shift = Vector3::from_array(self.translation.map(S::from_f64));
        let shifts = self.translation.iter().any(|&t| t != 0.0);
        let turn = match &self.axis {
            Some(axis) if self.angle != 0.0 => {
                let axis = axis.resolve(reader)?;
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

part_operation!(MoveBody(MoveBodyArgs));

impl PartOperation for MoveBody {
    const KIND: &'static str = "move_body";
    const INFO: OperationInfo = OperationInfo::new(
        "Move body",
        "Move bodies, or a copy of them, turned and shifted.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::MOVE_BODY);
    type Args = MoveBodyArgs;
    type Session = ();

    /// The newest solid, not moved yet.
    fn new_args(context: &Context) -> MoveBodyArgs {
        MoveBodyArgs {
            bodies: (read_model(&context.before).ok())
                .and_then(|model| model.newest_solid())
                .map(body)
                .into_iter()
                .collect(),
            translation: [0.0; 3],
            axis: None,
            angle: 0.0,
            copy: false,
            combine: Combine::NewBody,
        }
    }

    fn run(ctx: &mut StepContext, args: &MoveBodyArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The bodies, picked; the shift along each axis, dragged by a gizmo
    /// where the bodies' middle is moved to; the axis to turn about and the
    /// angle; whether to copy; and how to combine.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &MoveBodyArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, MoveBodyArgs> {
        let before = &context.before;
        let mut f = EditForm::<Design, MoveBodyArgs>::default();
        let shift = Vector3::from_array(args.translation.map(Design::from_f64));
        f.form.gizmo = center(before, &args.bodies).map(|c| Gizmo::new(c.add(&shift)).translate());
        bodies_field(&mut f, before, &args.bodies, |edit, picked| {
            edit.args.bodies = picked
        });
        for (k, (key, label)) in AXES.into_iter().enumerate() {
            let number = Number::new(label, args.translation[k], Unit::Length);
            f.number(key, number, move |edit, v| edit.args.translation[k] = v);
        }
        f.pick_one(
            "axis",
            "turn about",
            before,
            args.axis.as_ref(),
            |edit, picked| edit.args.axis = Some(picked),
        );
        f.form.dialog.optional("axis");
        if args.axis.is_some() {
            let angle = Number::new("angle", args.angle, Unit::Angle).range(-360.0, 360.0);
            f.number("angle", angle, |edit, v| edit.args.angle = v);
        }
        f.checkbox("copy", "copy", args.copy, |edit, b| edit.args.copy = b);
        let set_combine = |args: &mut MoveBodyArgs, combine| args.combine = combine;
        args.combine.fields(&mut f, before, set_combine);
        f
    }

    /// The gizmo dragged: the shift, as it was when the drag started, moved
    /// by as much.
    fn canvas(_: &Context, edit: Edit<'_, MoveBodyArgs, ()>, event: &CanvasEvent<Design>) {
        if let CanvasEvent::Gizmo { drag, .. } = event
            && let Change::Translate(by) = &drag.change
        {
            for (k, shift) in edit.args.translation.iter_mut().enumerate() {
                *shift += by[k].to_f64();
            }
        }
    }
}

impl ModelOperation for MoveBody {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        args: &MoveBodyArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("move", step)?;
        let seeds = resolve_bodies(reader, &args.bodies)?;
        let motion = args.motion(reader)?;
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
}
