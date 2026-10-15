//! Assemblies: parts placed in a part ([`placed`]) — another program
//! file's part, loaded through the runner and placed by [`AddPart`] at a
//! pose its program's state holds — and mated to what is already there
//! ([`mates`]). Where every placed part is, is the program's state, which
//! the editor settles after every edit ([`Settler`], solving the mates:
//! [`mechanism`], [`solving`]); the step placing a part drags it as far as
//! its mates let it (see [`editor`]), and so does the drag tool
//! ([`DragTool`]) with no step edited. The joints panel ([`JointsPanel`])
//! sets a joint's coordinates, and says how free the placed parts are and
//! which constraints conflict. [`PartPattern`] places copies of a placed
//! part in a row or round an axis.
//!
//! Also what reads a part with the parts placed in it: their solids, each
//! of its own part's material ([`solids`]), and an STL file of them all
//! ([`Stl`]); and how the placed parts are drawn and listed
//! ([`AssemblyDrawer`]).

mod add_part;
mod drag_tool;
mod draw;
pub mod editor;
pub mod frame;
#[cfg(test)]
mod frame_tests;
mod icons;
pub mod joints;
pub mod mates;
pub mod mechanism;
mod panel;
mod part_pattern;
pub mod placed;
pub mod residuals;
mod settle;
pub mod solids;
pub mod solving;
mod stl;
#[cfg(any(test, feature = "testing"))]
#[allow(clippy::expect_used, reason = "tests' helpers fail where a test would")]
pub mod testing;

use std::rc::Rc;

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

pub use add_part::{AddPart, AddPartArgs};
pub use drag_tool::DragTool;
pub use draw::AssemblyDrawer;
pub use frame::MateEntity;
pub use mates::{
    Anchor, CouplingKind, JointKind, Kind, Mate, MateKind, Motion, On, Resolved, Transmission,
    joint_parameter, mate, mate_name,
};
pub use mechanism::mates_resolved;
pub use panel::{JointsCommand, JointsPanel, JointsState};
pub use part_pattern::{Layout, PartPattern, PartPatternArgs, PatternAxis};
pub use placed::{Drawn, define_placed, drawn, placed_parts, solid_meshes};
pub use settle::Settler;
pub use stl::{STL_QUALITY, Stl, StlOptions, stl_bytes};

/// Adds what assemblies offer an editor to `r`: placing parts and patterns
/// of them, the STL of a part with its placed parts, how placed parts are
/// drawn, the joints panel, the drag tool, and what settles the program's
/// state after every edit.
pub fn register(r: Registry) -> GeopResult<Registry> {
    let settled = Rc::new(settle::Settled::default());
    r.operation(AddPart)?
        .operation(PartPattern)?
        .exporter(Stl)?
        .drawer(AssemblyDrawer)?
        .panel(JointsPanel::new(settled.clone()))?
        .tool(DragTool::INFO, DragTool::default())?
        .settler(Settler { settled })
}
