use super::ids::ShellId;

/// A 3-dimensional topological entity: a region of space bounded by
/// closed shells, each connected, which never cross. A point is inside the
/// solid when it is inside an odd number of them, so a shell is a lump of
/// its own or a void in one by where it lies, not by its place in
/// `shells`: a boolean cutting a bar in two leaves two lumps, one leaving
/// a sphere's room inside a block a lump and a void. A void's faces point
/// into it, out of the material, so the volume it encloses is negative.
#[derive(Clone, Debug, PartialEq)]
pub struct Solid {
    pub shells: Vec<ShellId>,
}
