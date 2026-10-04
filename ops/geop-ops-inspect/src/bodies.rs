//! The solids a part shows, wherever they are placed — its own, and those
//! of the parts placed in it, however deep — and entities named as the part
//! names them, resolved to the part they belong to.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::Scalar,
};
use geop_core_topology::SolidId;
use geop_ops::{EntityRef, Part, operation::INSTANCE_SEPARATOR};

/// A solid of a part, or of a part placed in it: named as the part names
/// it (`pin/extrude(pin)` for one of the part placed as `pin`), where it is.
pub struct PlacedSolid<'p, S: Scalar> {
    pub name: String,
    /// The part it is a solid of — whose material it is made of.
    pub part: &'p Part<S>,
    pub solid: SolidId,
    /// Where its part is placed; `None` for the part's own solids.
    pub pose: Option<Pose<S>>,
}

/// Every solid of `part` and of the parts placed in it, oldest first, the
/// part's own before those placed.
pub fn placed_solids<S: Scalar>(part: &Part<S>) -> GeopResult<Vec<PlacedSolid<'_, S>>> {
    let mut out = Vec::new();
    collect(part, "", None, &mut out)?;
    Ok(out)
}

fn collect<'p, S: Scalar>(
    part: &'p Part<S>,
    prefix: &str,
    pose: Option<Pose<S>>,
    out: &mut Vec<PlacedSolid<'p, S>>,
) -> GeopResult<()> {
    for name in part.solid_names() {
        out.push(PlacedSolid {
            solid: part.solid_id(&name)?,
            name: format!("{prefix}{name}"),
            part,
            pose,
        });
    }
    for (id, instance) in part.instances() {
        let name = part
            .name_of(id)
            .ok_or_else(|| GeopError::new(format!("placed part {id} has no name")))?;
        let inner = match &pose {
            Some(outer) => outer.compose(&instance.pose),
            None => instance.pose,
        };
        let prefix = format!("{prefix}{name}{INSTANCE_SEPARATOR}");
        collect(instance.part(), &prefix, Some(inner), out)?;
    }
    Ok(())
}

/// `entity`, named as `part` names it, resolved: the part it belongs to —
/// `part` itself or one placed in it — the entity as that part names it,
/// and where that part is placed.
pub fn resolve<'p, S: Scalar>(
    part: &'p Part<S>,
    entity: &EntityRef,
) -> GeopResult<(&'p Part<S>, EntityRef, Option<Pose<S>>)> {
    let Some((name, inner)) = entity.split_instance() else {
        return Ok((part, entity.clone(), None));
    };
    let ctx = |e: GeopError| e.with_context(format!("resolving {entity}"));
    let instance = part
        .instance(part.instance_id(&name).with_context(&ctx)?)
        .with_context(&ctx)?;
    let (inner_part, local, pose) = resolve(instance.part(), &inner).with_context(&ctx)?;
    let pose = match pose {
        Some(pose) => instance.pose.compose(&pose),
        None => instance.pose,
    };
    Ok((inner_part, local, Some(pose)))
}
