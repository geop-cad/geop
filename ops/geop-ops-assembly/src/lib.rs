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
//! part in a row or round an axis. [`Explode`] moves placed parts off
//! where they are assembled, one move of an exploded view per step.
//!
//! Also what is exported of an assembly: an STL file of a part with the
//! parts placed in it ([`Stl`]), and its mechanism as a robot description
//! ([`urdf`]); and how the placed parts are drawn and listed
//! ([`AssemblyDrawer`]). What a placed part is to what reads it is
//! [`geop_ops_model::placed`]'s.

mod add_part;
mod drag_tool;
mod draw;
#[cfg(test)]
mod draw_tests;
pub mod editor;
#[cfg(test)]
mod editor_tests;
#[cfg(any(test, feature = "examples"))]
pub mod examples;
mod explode;
#[cfg(test)]
mod frame_tests;
mod icons;
#[cfg(test)]
mod joint_tests;
pub mod joints;
#[cfg(test)]
mod kept_tests;
pub mod mates;
pub mod mechanism;
mod panel;
mod part_pattern;
pub mod placed;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod regression_tests;
pub mod residuals;
#[cfg(test)]
mod scale_tests;
mod settle;
pub mod solving;
mod stl;
#[cfg(any(test, feature = "testing"))]
#[allow(clippy::expect_used, reason = "tests' helpers fail where a test would")]
pub mod testing;
pub mod urdf;

use std::rc::Rc;

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

pub use add_part::{AddPart, AddPartArgs};
pub use drag_tool::DragTool;
pub use draw::AssemblyDrawer;
pub use explode::{Direction, Explode, ExplodeArgs};
pub use mates::{
    Anchor, CouplingKind, Joint, JointKind, Kind, Limits, MOTIONS, Mate, MateKind, Motion, On,
    Resolved, Transmission, foot_parameter, joint_parameter, mate, mate_name,
};
pub use mechanism::mates_resolved;
pub use panel::{JointsCommand, JointsPanel, JointsState};
pub use part_pattern::{Layout, PartPattern, PartPatternArgs, PatternAxis};
pub use placed::define_placed;
pub use settle::Settler;
pub use stl::{Stl, StlOptions};
pub use urdf::Urdf;

/// Adds what assemblies offer an editor to `r`: placing parts and patterns
/// of them, the STL of a part with its placed parts and the URDF of its
/// mechanism, how placed parts are drawn, the joints panel, the drag tool,
/// what settles the program's state after every edit, and the examples of
/// assemblies.
pub fn register(r: Registry) -> GeopResult<Registry> {
    let settled = Rc::new(settle::Settled::default());
    let r = r
        .operation(AddPart)?
        .operation(PartPattern)?
        .operation(Explode)?
        .exporter(Stl)?
        .exporter(Urdf)?
        .drawer(AssemblyDrawer)?
        .panel(JointsPanel::new(settled.clone()))?
        .tool(DragTool::INFO, DragTool::default())?
        .settler(Settler { settled })?;
    #[cfg(any(test, feature = "examples"))]
    let r = r.examples(examples::set())?;
    Ok(r)
}
