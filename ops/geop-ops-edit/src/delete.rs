//! [`DeleteBody`]: delete whole bodies — solids, and sheets of faces
//! standing on their own — from the part.

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    with_context,
};
use geop_core_program::{
    editing::{Event, Form, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::{
    Design, EntityRef, NamedModel, ParamValue,
    operation::{Build, BuildReader, Input, LegacyModelStep, Role, legacy_build_model, resolve_bodies},
};
use serde::{Deserialize, Serialize};

/// Deletes the bodies `bodies` refers to, and everything of them: a solid
/// by its name, a sheet — faces standing on their own, which have no name
/// as a whole — by one of its faces. A face of a solid deletes the whole
/// solid.
///
/// Creates nothing, so names nothing; the names of everything deleted are
/// free again.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DeleteBody;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DeleteBodyArgs {
    /// The bodies to delete: solids, or a face of each sheet.
    pub bodies: Vec<EntityRef>,
}

impl LegacyModelStep for DeleteBody {
    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &DeleteBodyArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("delete_body({step}, {args:?})");
        let bodies = resolve_bodies(&args.bodies, &mut &*model).with_context(ctx)?;
        model.assemble_sheet(&bodies, &[]).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for DeleteBody {
    const INFO: OperationInfo = OperationInfo::new(
        "delete_body",
        "Delete body",
        "Delete solids, and faces standing on their own.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::DELETE_BODY);
    type Args = DeleteBodyArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// Nothing picked yet: what to delete is never a guess.
    fn new_args(_: &Context<Input, ParamValue>) -> DeleteBodyArgs {
        DeleteBodyArgs::default()
    }

    fn run(step: &str, part: &mut Build, args: &DeleteBodyArgs) -> GeopResult<()> {
        legacy_build_model::<Self>(part, step, args)
    }

    /// The bodies, picked.
    fn presentation(_: &Context<Input, ParamValue>, args: &DeleteBodyArgs, _: &()) -> Form<Design> {
        let mut form = Form::default();
        let roles = [Role::Solid, Role::Sheet];
        (form.dialog).reference("bodies", "bodies", args.bodies.clone(), &roles, None, true);
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &DeleteBodyArgs,
        _: &(),
        event: Event<Design>,
    ) -> (DeleteBodyArgs, ()) {
        let mut args = args.clone();
        if let Event::Field {
            key,
            value: Value::Entities(picked),
        } = event
            && key == "bodies"
        {
            args.bodies = picked;
        }
        (args, ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::{
        scalars::{ScalInF64 as S, Scalar},
        vector::Vector3,
    };
    use geop_ops::operation::apply_to_model;
    use geop_ops_extrude_revolve::shapes::cube_solid;

    fn v3(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    fn two_cubes() -> NamedModel<S> {
        let mut part = NamedModel::<S>::new();
        cube_solid(&mut part, "a", v3(0.0, 0.0, 0.0), v3(1.0, 1.0, 1.0)).unwrap();
        cube_solid(&mut part, "b", v3(2.0, 0.0, 0.0), v3(3.0, 1.0, 1.0)).unwrap();
        part
    }

    /// Deleting one of two solids leaves the other exactly as it was, and
    /// no name of the deleted one behind.
    #[test]
    fn deletes_one_solid_and_keeps_the_other() {
        let part = two_cubes();
        let b = part.solid_names()[1].clone();
        let kept_faces = part
            .topology()
            .solid_faces(part.solid_id(&part.solid_names()[0]).unwrap())
            .unwrap()
            .len();
        let args = DeleteBodyArgs {
            bodies: vec![EntityRef::Solid { name: b }],
        };
        let part = apply_to_model::<DeleteBody>(part, "del", &args).unwrap();
        assert_eq!(part.solid_names().len(), 1);
        assert_eq!(part.topology().faces.len(), kept_faces);
        assert_eq!(part.topology().vertices.len(), 8);
        part.check_names().unwrap();
    }

    /// A face of a solid stands for the whole solid, and two references
    /// to one body delete it once.
    #[test]
    fn a_face_deletes_its_whole_body() {
        let part = two_cubes();
        let a = part.solid_id(&part.solid_names()[0]).unwrap();
        let face = part.topology().solid_faces(a).unwrap()[0];
        let face = part.name_of(face).unwrap().to_string();
        let name = part.name_of(a).unwrap().to_string();
        let args = DeleteBodyArgs {
            bodies: vec![EntityRef::Face { name: face }, EntityRef::Solid { name }],
        };
        let part = apply_to_model::<DeleteBody>(part, "del", &args).unwrap();
        assert_eq!(part.solid_names().len(), 1);
        assert_eq!(part.topology().vertices.len(), 8);
        part.check_names().unwrap();
    }

    /// Nothing picked, or something that is no body, is refused.
    #[test]
    fn refuses_what_is_no_body() {
        let error = apply_to_model::<DeleteBody>(two_cubes(), "del", &DeleteBodyArgs::default())
            .expect_err("refused");
        assert!(error.root_message().contains("pick"), "{error:?}");
        let args = DeleteBodyArgs {
            bodies: vec![EntityRef::Sketch { name: "s".into() }],
        };
        assert!(apply_to_model::<DeleteBody>(two_cubes(), "del", &args).is_err());
    }
}
