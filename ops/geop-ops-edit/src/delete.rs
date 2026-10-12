//! [`DeleteBody`]: delete whole bodies — solids, and sheets of faces
//! standing on their own — from the part.

use geop_core_math::geop_error::GeopResult;
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, EntityRef, NamedModel,
    operation::{BuildReader, PartOperation, Role, StepContext, resolve_bodies},
    part_operation,
    ui::{EditForm, FormState},
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

part_operation!(DeleteBody);

impl PartOperation for DeleteBody {
    const INFO: OperationInfo = OperationInfo::new(
        "delete_body",
        "Delete body",
        "Delete solids, and faces standing on their own.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::DELETE_BODY);
    type Args = DeleteBodyArgs;
    type Session = ();

    /// Nothing picked yet: what to delete is never a guess.
    fn new_args(_: &StepContext) -> DeleteBodyArgs {
        DeleteBodyArgs::default()
    }

    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        _: &str,
        args: &DeleteBodyArgs,
    ) -> GeopResult<()> {
        let bodies = resolve_bodies(&args.bodies, &mut &*model)?;
        model.delete_bodies(&bodies)?;
        Ok(())
    }

    /// The bodies, picked.
    fn form<'a>(
        _: &'a StepContext<'a>,
        args: &DeleteBodyArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, DeleteBodyArgs> {
        let mut f = EditForm::<Design, DeleteBodyArgs>::default();
        let roles = [Role::Solid, Role::Sheet];
        let bodies = args.bodies.clone();
        f.reference(
            "bodies",
            "bodies",
            bodies,
            &roles,
            None,
            true,
            |edit, picked| edit.args.bodies = picked,
        );
        f
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::scalars::ScalInF64 as S;
    use geop_ops::operation::apply_to_model;
    use geop_ops_extrude_revolve::shapes::{block, block_part};

    fn two_cubes() -> NamedModel<S> {
        let mut part = block_part("a", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        block(&mut part, "b", [2.0, 0.0, 0.0], [3.0, 1.0, 1.0]);
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
