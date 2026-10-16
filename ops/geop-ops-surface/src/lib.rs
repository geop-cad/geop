//! Surface modelling: faces standing on their own — sheets — built from
//! edges, offset, thickened, joined, trimmed and extended, as operations of
//! a program (see [`geop_core_editor::part_operation`]):
//!
//! - [`BoundarySurface`]: the ruled face between two edges, or a closed
//!   loop of edges filled — a flat face for a flat loop, a Coons patch for
//!   four edges, optionally tangent to flat faces along them, and a patch
//!   of quadrilaterals around a center for any other number.
//! - [`NetworkSurface`]: the face through a network of curves crossing
//!   each other, `u` curves and `v` curves, interpolating every one.
//! - [`OffsetSurface`]: faces copied a distance along their normals.
//! - [`Thicken`]: a sheet made a solid of one thickness, on either side of
//!   it or on both.
//! - [`Knit`]: sheets whose edges meet joined into one, and into a solid
//!   once they close up.
//! - [`TrimSurface`]: a sheet cut back to one side of a face.
//! - [`ExtendSurface`]: a face standing on its own carried on past one of
//!   its edges.
//!
//! What a sheet is offset and thickened with is the shell's machinery
//! ([`geop_lib_shell::shell()`]): a sheet thickened is a sheet shelled.

pub mod boundary;
pub mod extend;
mod icons;
pub mod knit;
pub mod network;
pub mod offset;
#[cfg(test)]
mod program_tests;
pub mod thicken;
pub mod trim;

pub use boundary::{BoundarySurface, BoundarySurfaceArgs};
pub use extend::{ExtendSurface, ExtendSurfaceArgs};
pub use knit::{Knit, KnitArgs};
pub use network::{NetworkSurface, NetworkSurfaceArgs};
pub use offset::{OffsetSurface, OffsetSurfaceArgs};
pub use thicken::{Thicken, ThickenArgs, ThickenSide};
pub use trim::{TrimKeep, TrimSurface, TrimSurfaceArgs};

use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_ops_model::Chain;

/// A sheet to build, and the names of what it is made of.
pub type SheetSpec<S> = (
    geop_core_topology::build::BodySpec<S>,
    geop_ops_model::BodyNames,
);

/// One curve picked to span a surface through: an edge — of a face or of a
/// wire — or a sketch's curve, with the names of its ends. An edge's name
/// is its own, its ends its vertices'; a sketch `K`'s curve `c3` is `K,c3`,
/// its ends `K,p1` (see [`geop_ops_sketch3d::CurveSource`]).
#[derive(Clone, Debug)]
struct NamedCurve<S: Scalar> {
    name: String,
    curve: NurbCurve3D<S>,
    start: (String, Vector3<S>),
    end: (String, Vector3<S>),
}

impl<S: Scalar> NamedCurve<S> {
    /// The one curve of `chain`, as a curve picked resolves; an error if it
    /// is more than one.
    fn of(chain: &Chain<NurbCurve3D<S>>) -> GeopResult<Self> {
        let [curve] = chain.curves.as_slice() else {
            return Err(GeopError::new(format!(
                "{} is {} curves, not one",
                chain.name,
                chain.curves.len()
            )));
        };
        let joints = chain.joints()?;
        let end = joints.len() - 1;
        Ok(Self {
            name: chain.curve_names[0].clone(),
            curve: curve.clone(),
            start: (chain.joint_names[0].clone(), joints[0]),
            end: (chain.joint_names[end].clone(), joints[end]),
        })
    }
}

use geop_core_editor::registry::Registry;

/// Adds what this crate offers an editor to `r`: surfaces: boundary,
/// offset, thickened, knit, trimmed, extended and network surfaces.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(BoundarySurface)?
        .operation(OffsetSurface)?
        .operation(Thicken)?
        .operation(Knit)?
        .operation(TrimSurface)?
        .operation(ExtendSurface)?
        .operation(NetworkSurface)
}

/// What the tests share: the model's elements by name.
#[cfg(test)]
pub(crate) mod test_refs {
    use geop_core_program::{element::ElementReference, values::Design};
    use geop_ops_model::{NamedModel, model};
    use geop_ops_sketch3d::CurveSource;

    /// The model's element named `name`.
    pub fn named<K>(name: impl Into<String>) -> ElementReference<NamedModel<Design>, K> {
        ElementReference::new(model(), name)
    }

    /// The edge named `name`, as a curve picked.
    pub fn edge_curve(name: impl Into<String>) -> CurveSource {
        CurveSource::Edge(named(name))
    }
}
