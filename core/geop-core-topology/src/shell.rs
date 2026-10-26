use super::ids::{FaceId, SolidId};

/// A connected, orientable 2-manifold made up of faces.
///
/// A shell of a solid is closed and bounds a region of space: a lump of
/// the solid, or a void in one (see [`crate::Solid`]). A shell of no solid
/// is a *sheet*, connected or not: faces standing on their own, not
/// bounding anything, whose free border edges are used by one face only
/// (what an extrude or revolve builds as a surface, and what a split cuts a
/// solid with).
#[derive(Clone, Debug, PartialEq)]
pub struct Shell {
    pub faces: Vec<FaceId>,
    pub solid: Option<SolidId>,
}
