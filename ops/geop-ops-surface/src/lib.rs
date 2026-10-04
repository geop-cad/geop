//! Surface modelling: faces standing on their own — sheets — built from
//! edges, offset, thickened, joined, trimmed and extended, as operations of
//! a program (see [`geop_ops::operation`]):
//!
//! - [`BoundarySurface`]: the ruled face between two edges, or a closed
//!   loop of edges filled — a flat face for a flat loop, a Coons patch for
//!   four edges, optionally tangent to flat faces along them, and a patch
//!   of quadrilaterals around a center for any other number.
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
//! ([`geop_ops_shell::shell`]): a sheet thickened is a sheet shelled.

pub mod boundary;
pub mod extend;
pub mod knit;
pub mod offset;
pub mod thicken;
pub mod trim;

pub use boundary::{BoundarySurface, BoundarySurfaceArgs};
pub use extend::{ExtendSurface, ExtendSurfaceArgs};
pub use knit::{Knit, KnitArgs};
pub use offset::{OffsetSurface, OffsetSurfaceArgs};
pub use thicken::{Thicken, ThickenArgs, ThickenSide};
pub use trim::{TrimKeep, TrimSurface, TrimSurfaceArgs};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_topology::{Body, FaceId, ShellId};
use geop_ops::{EntityRef, Part, RefId};

/// Bounds how hard a containment search or a pcurve fit tries: effort, not
/// what an answer means.
const MAX_NODES: usize = 20_000;

/// Where a containment search hands over to Newton (see `AGENTS.md`).
fn min_subdivision_size<S: Scalar>() -> S {
    S::from_f64(1e-7)
}

/// The name of `id` in `part`.
fn name_of<S: Scalar>(part: &Part<S>, id: impl Into<RefId>) -> GeopResult<String> {
    let id = id.into();
    part.name_of(id)
        .map(str::to_string)
        .ok_or_else(|| GeopError::new(format!("{id} has no name")))
}

/// Entities by name, as a reference field holds them: faces or edges.
fn refs(names: &[String], make: fn(String) -> EntityRef) -> Vec<EntityRef> {
    names.iter().cloned().map(make).collect()
}

fn face(name: String) -> EntityRef {
    EntityRef::Face { name }
}

fn edge(name: String) -> EntityRef {
    EntityRef::Edge { name }
}

/// The names of the faces or edges a reference field holds.
fn picked_names(picked: &[EntityRef]) -> Vec<String> {
    picked
        .iter()
        .filter_map(|e| match e {
            EntityRef::Face { name } | EntityRef::Edge { name } => Some(name.clone()),
            _ => None,
        })
        .collect()
}

/// The sheet the face named `name` stands in — an error, naming it, if it
/// is a solid's face.
fn sheet_of<S: Scalar>(part: &Part<S>, name: &str) -> GeopResult<(FaceId, ShellId)> {
    let face = part.face_id(name)?;
    match part.topology().body_of_face(face)? {
        Body::Sheet(sheet) => Ok((face, sheet)),
        Body::Solid(_) => Err(GeopError::new(format!(
            "face {name} is a face of a solid, not one standing on its own"
        ))),
    }
}
