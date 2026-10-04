use std::fmt;

use super::ids::{ShellId, SolidId};

/// A whole the operations act on: a solid, or a sheet — a shell belonging to
/// no solid (see [`crate::Shell`]).
///
/// A boolean's operands, what remesh imprints onto each other and what a
/// split cuts with are bodies: their faces, edges and vertices are all that
/// matters to imprinting, and a sheet has those just as a solid does.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Body {
    Solid(SolidId),
    Sheet(ShellId),
}

impl fmt::Display for Body {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Body::Solid(id) => write!(f, "{id}"),
            Body::Sheet(id) => write!(f, "sheet {id}"),
        }
    }
}

impl From<SolidId> for Body {
    fn from(id: SolidId) -> Self {
        Body::Solid(id)
    }
}
