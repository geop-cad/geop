//! [`DeleteBody`]: delete whole bodies — solids, and sheets of faces
//! standing on their own — from the part.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_topology::Body;
use geop_ops::{
    Context, EntityRef, Library, Part,
    operation::{Operation, Role},
    ui::Form,
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

/// The body `entity` refers to in `part`.
fn body_of<S: Scalar>(part: &Part<S>, entity: &EntityRef) -> GeopResult<Body> {
    if entity.split_instance().is_some() {
        return Err(GeopError::new(format!(
            "{entity} belongs to a placed part; delete it in the part's own program"
        )));
    }
    match entity {
        EntityRef::Solid { name } => Ok(Body::Solid(part.solid_id(name)?)),
        EntityRef::Face { name } => part.topology().body_of_face(part.face_id(name)?),
        other => Err(GeopError::new(format!(
            "{other} is no body: pick a solid or a face standing on its own"
        ))),
    }
}

impl Operation for DeleteBody {
    type Args = DeleteBodyArgs;
    type Session = ();

    /// Nothing picked yet: what to delete is never a guess.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> DeleteBodyArgs {
        DeleteBodyArgs::default()
    }

    /// The bodies, picked.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &DeleteBodyArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, DeleteBodyArgs> {
        let mut f = Form::<S, DeleteBodyArgs>::new();
        f.reference(
            "bodies",
            "bodies",
            args.bodies.clone(),
            &[Role::Solid, Role::Sheet],
            None,
            true,
            |e, picked| e.args.bodies = picked,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &DeleteBodyArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("delete_body({operation_id}, {args:?})");
        if args.bodies.is_empty() {
            return Err(GeopError::new("pick the bodies to delete")).with_context(ctx);
        }
        let mut bodies = Vec::new();
        for entity in &args.bodies {
            let body = body_of(&part, entity).with_context(ctx)?;
            if !bodies.contains(&body) {
                bodies.push(body);
            }
        }
        part.assemble_sheet(&bodies, &[]).with_context(ctx)?;
        Ok(part)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::{
        scalars::{ScalInF64 as S, Scalar},
        vector::Vector3,
    };
    use geop_ops::NoFiles;
    use geop_ops_extrude_revolve::shapes::cube_solid;

    fn v3(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    fn two_cubes() -> Part<S> {
        let mut part = Part::<S>::new();
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
        let part = DeleteBody.apply(part, "del", &args, &NoFiles).unwrap();
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
        let part = DeleteBody.apply(part, "del", &args, &NoFiles).unwrap();
        assert_eq!(part.solid_names().len(), 1);
        assert_eq!(part.topology().vertices.len(), 8);
        part.check_names().unwrap();
    }

    /// Nothing picked, or something that is no body, is refused.
    #[test]
    fn refuses_what_is_no_body() {
        let error = DeleteBody
            .apply(two_cubes(), "del", &DeleteBodyArgs::default(), &NoFiles)
            .err()
            .expect("refused");
        assert!(error.root_message().contains("pick"), "{error:?}");
        let args = DeleteBodyArgs {
            bodies: vec![EntityRef::Sketch { name: "s".into() }],
        };
        assert!(
            DeleteBody
                .apply(two_cubes(), "del", &args, &NoFiles)
                .is_err()
        );
    }
}
