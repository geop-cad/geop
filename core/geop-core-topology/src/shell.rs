use super::ids::{FaceId, SolidId};

/// A connected, orientable 2-manifold made up of faces.
///
/// A closed shell bounds a region of space; an open shell does not.  A solid
/// typically has exactly one outer shell and zero or more void shells.
#[derive(Clone, Debug)]
pub struct Shell {
    pub faces: Vec<FaceId>,
    pub solid: SolidId,
}
