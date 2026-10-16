//! [`DeleteBody`]: delete whole bodies — solids, and sheets of faces
//! standing on their own — from the part.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::ElementReference, part::StepContext, reader::Reader, values::Design,
};
use geop_ops_model::{Body, ModelOperation, NamedModel, build_model, resolve_bodies};
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
    pub bodies: Vec<ElementReference<NamedModel<Design>, Body>>,
}

part_operation!(DeleteBody(DeleteBodyArgs));

impl PartOperation for DeleteBody {
    const KIND: &'static str = "delete_body";
    const INFO: OperationInfo = OperationInfo::new(
        "Delete body",
        "Delete solids, and faces standing on their own.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::DELETE_BODY);
    type Args = DeleteBodyArgs;
    type Session = ();

    /// Nothing picked yet: what to delete is never a guess.
    fn new_args(_: &Context) -> DeleteBodyArgs {
        DeleteBodyArgs::default()
    }

    fn run(ctx: &mut StepContext, args: &DeleteBodyArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The bodies, picked.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &DeleteBodyArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, DeleteBodyArgs> {
        let mut f = EditForm::<Design, DeleteBodyArgs>::default();
        f.pick(
            "bodies",
            "bodies",
            &context.before,
            &args.bodies,
            true,
            |edit, picked| edit.args.bodies = picked,
        );
        f
    }
}

impl ModelOperation for DeleteBody {
    /// The bodies are resolved in the model as the steps before left it,
    /// which `model` is a copy of.
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        _: &str,
        args: &DeleteBodyArgs,
    ) -> GeopResult<()> {
        let bodies = resolve_bodies(reader, &args.bodies)?;
        model.delete_bodies(&bodies)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_lib_sweep::shapes::{block, block_part};
    use geop_ops_model::{model, testing::apply_to_model};

    fn body(name: impl Into<String>) -> ElementReference<NamedModel<Design>, Body> {
        ElementReference::new(model(), name)
    }

    fn two_cubes() -> NamedModel<Design> {
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
            bodies: vec![body(b)],
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
            bodies: vec![body(face), body(name)],
        };
        let part = apply_to_model::<DeleteBody>(part, "del", &args).unwrap();
        assert_eq!(part.solid_names().len(), 1);
        assert_eq!(part.topology().vertices.len(), 8);
        part.check_names().unwrap();
    }

    /// Nothing picked, or a name of something that is no body — an edge —
    /// is refused.
    #[test]
    fn refuses_what_is_no_body() {
        let error = apply_to_model::<DeleteBody>(two_cubes(), "del", &DeleteBodyArgs::default())
            .expect_err("refused");
        assert!(error.root_message().contains("pick"), "{error:?}");
        let part = two_cubes();
        let edge = part
            .topology()
            .edges
            .keys()
            .copied()
            .min_by_key(|e| e.0)
            .unwrap();
        let args = DeleteBodyArgs {
            bodies: vec![body(part.name_of(edge).unwrap())],
        };
        let error = apply_to_model::<DeleteBody>(part, "del", &args).expect_err("refused");
        assert!(error.root_message().contains("no body"), "{error:?}");
    }
}
