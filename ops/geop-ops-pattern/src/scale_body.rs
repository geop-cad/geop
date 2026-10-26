//! [`ScaleBody`]: bodies scaled about a point, by one factor or by one
//! along each axis.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Motion,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{part::StepContext, reader::Reader};
use geop_ops_model::{ModelOperation, NamedModel, build_model, read_model, resolve_bodies};
use serde::{Deserialize, Serialize};

use crate::common::{BodyRef, bodies_field, body};

/// Scales the bodies `bodies` — solids, and sheets by one of their faces —
/// about the point `about` by `factors` along the world's `x`, `y` and
/// `z`: in place, every entity keeping its name. Exact: a NURBS curve or
/// surface scaled is its control points scaled, weights and parametrization
/// kept, so every pcurve stays as it is. Unequal factors make a circle an
/// ellipse — exactly, as a NURBS. A negative factor mirrors (an odd number
/// of them turns every face around, so the normals still point out); a
/// factor of zero is refused.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ScaleBody;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScaleBodyArgs {
    /// The bodies to scale: solids, or a face of each sheet.
    pub bodies: Vec<BodyRef>,
    /// The point that stays where it is.
    #[serde(default)]
    pub about: [f64; 3],
    /// The factor along the world's `x`, `y` and `z`.
    pub factors: [f64; 3],
}

part_operation!(ScaleBody(ScaleBodyArgs));

impl PartOperation for ScaleBody {
    const KIND: &'static str = "scale_body";
    const INFO: OperationInfo = OperationInfo::new(
        "Scale body",
        "Scale bodies about a point, by one factor or by one along each axis.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::SCALE_BODY);
    type Args = ScaleBodyArgs;
    type Session = ();

    /// The newest solid, about the origin, not scaled yet.
    fn new_args(context: &Context) -> ScaleBodyArgs {
        ScaleBodyArgs {
            bodies: (read_model(&context.before).ok())
                .and_then(|model| model.newest_solid())
                .map(body)
                .into_iter()
                .collect(),
            about: [0.0; 3],
            factors: [1.0; 3],
        }
    }

    fn run(ctx: &mut StepContext, args: &ScaleBodyArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The bodies, picked; the point scaled about and the factors.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ScaleBodyArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, ScaleBodyArgs> {
        let mut f = EditForm::<ScaleBodyArgs>::default();
        bodies_field(&mut f, &context.before, &args.bodies, |edit, picked| {
            edit.args.bodies = picked
        });
        for (k, axis) in ["x", "y", "z"].into_iter().enumerate() {
            let label = ["about x", "about y", "about z"][k];
            let number = Number::new(label, args.about[k], Unit::Length);
            f.number(
                ["about_x", "about_y", "about_z"][k],
                number,
                move |edit, v| edit.args.about[k] = v,
            );
            let label = ["factor x", "factor y", "factor z"][k];
            let number = Number::new(label, args.factors[k], Unit::Fraction);
            f.number(axis, number, move |edit, v| edit.args.factors[k] = v);
        }
        f
    }
}

impl ModelOperation for ScaleBody {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        _: &str,
        args: &ScaleBodyArgs,
    ) -> GeopResult<()> {
        let values = args.about.iter().chain(&args.factors);
        if let Some(bad) = values.clone().find(|v| !v.is_finite()) {
            return Err(GeopError::new(format!("{bad} is not a number")));
        }
        let about = Vector3::from_array(args.about.map(Scalar::from_f64));
        let motion = Motion::scaling(&about, args.factors.map(Scalar::from_f64))?;
        for body in resolve_bodies(reader, &args.bodies)? {
            model.transform_body(body, &motion)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_lib_sweep::shapes::{block_part, cylinder};
    use geop_ops_model::testing::{apply_to_model, assert_valid};

    /// The volume of the solid `solid` of `part`, integrated from its
    /// faces (see `geop_core_topology::mass`).
    fn solid_volume(part: &NamedModel, solid: &str) -> f64 {
        let solid = part.solid_id(solid).unwrap();
        (part.topology().mass_properties(solid).unwrap().volume).to_f64()
    }

    fn scaled(part: NamedModel, factors: [f64; 3]) -> GeopResult<NamedModel> {
        let name = part.solid_names()[0].clone();
        let args = ScaleBodyArgs {
            bodies: vec![body(name)],
            about: [1.0, 1.0, 1.0],
            factors,
        };
        apply_to_model::<ScaleBody>(part, "s", &args)
    }

    /// A box scaled by 2, 3 and 0.5: three times the volume,
    /// valid, the corner it was scaled about where it was.
    #[test]
    fn a_box_scaled_unevenly() {
        let part = block_part("b", [0.0; 3], [2.0, 2.0, 2.0]);
        let part = scaled(part, [2.0, 3.0, 0.5]).unwrap();
        assert_valid(&part);
        let solid = part.solid_names()[0].clone();
        assert!((solid_volume(&part, &solid) - 24.0).abs() < 1e-9);
    }

    /// A cylinder scaled along `x` only: an elliptic cylinder, exact.
    #[test]
    fn a_cylinder_scaled_into_an_ellipse() {
        let mut part = NamedModel::new();
        cylinder(&mut part, "c", [0.0; 3], 1.0, 1.0);
        let part = scaled(part, [2.0, 1.0, 1.0]).unwrap();
        assert_valid(&part);
        let solid = part.solid_names()[0].clone();
        let volume = 2.0 * std::f64::consts::PI;
        assert!((solid_volume(&part, &solid) - volume).abs() < 1e-9);
    }

    /// A negative factor mirrors: the box is still valid, its faces turned
    /// to point out.
    #[test]
    fn a_negative_factor_mirrors() {
        let part = block_part("b", [0.0; 3], [2.0, 2.0, 2.0]);
        let part = scaled(part, [-1.0, 1.0, 1.0]).unwrap();
        assert_valid(&part);
    }

    /// A factor of zero is refused.
    #[test]
    fn a_zero_factor_is_refused() {
        let part = block_part("b", [0.0; 3], [2.0, 2.0, 2.0]);
        let error = scaled(part, [1.0, 0.0, 1.0]).unwrap_err();
        assert!(format!("{error}").contains("flattens"), "{error}");
    }
}
