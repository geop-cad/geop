pub mod argument_validation;
pub mod boundary;
pub mod coedge;
pub mod contains;
pub mod edge;
pub mod edit;
pub mod euler;
pub mod face;
pub mod ids;
pub mod loop_sampling;
pub mod model;
pub mod shell;
pub mod solid;
#[cfg(test)]
pub(crate) mod test_fixtures;
pub mod validation;
pub mod vertex;

pub use coedge::{Coedge, CoedgeGeometry, Sense};
pub use edge::Edge;
pub use face::Face;
pub use ids::{CoedgeId, EdgeId, FaceId, ShellId, SolidId, VertexId};
pub use model::{Curve2, Curve3, Model};
pub use shell::Shell;
pub use solid::Solid;
pub use vertex::Vertex;
