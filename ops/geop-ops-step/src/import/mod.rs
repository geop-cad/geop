//! Reading the bodies of a STEP file (AP203, AP214, AP242 — the B-rep
//! geometry they share) as descriptions the kernel builds.
//!
//! - [`structure`] finds the bodies — every `MANIFOLD_SOLID_BREP`,
//!   `BREP_WITH_VOIDS` and `SHELL_BASED_SURFACE_MODEL` of every product —
//!   with their units and, in an assembly, where each part is placed.
//! - [`geometry`] reads curves and surfaces and builds their NURBS.
//! - [`body`] reads each body's topology and turns it into a geop body.
//!
//! What it cannot read it refuses, naming the entity by type and `#id`
//! and saying why.

pub mod body;
pub mod geometry;
pub mod reader;
pub mod structure;

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};

pub use body::ImportedBody;

use crate::part21::Exchange;

/// The uncertainty of a file that states none, in millimetres.
const DEFAULT_UNCERTAINTY: f64 = 1e-6;

/// Every body of the STEP file `text`, in the order the file has them.
pub fn read_step<S: Scalar>(text: &str) -> GeopResult<Vec<ImportedBody<S>>> {
    let exchange = Exchange::parse(text)?;
    read_exchange(&exchange)
}

/// Every body of the exchange file `exchange`.
pub fn read_exchange<S: Scalar>(exchange: &Exchange) -> GeopResult<Vec<ImportedBody<S>>> {
    let reader = reader::Reader { exchange };
    let mut items = reader.bodies()?;
    for item in &mut items {
        if !(item.scope.uncertainty > 0.0) {
            item.scope.uncertainty = DEFAULT_UNCERTAINTY;
        }
    }
    items
        .iter()
        .map(|item| {
            body::read_body(reader, item)
                .map_err(|e| e.with_context(format!("reading the body #{} ({})", item.id, item.label)))
        })
        .collect()
}
