use super::ids::ShellId;

/// A 3-dimensional topological entity: a closed volume bounded by shells.
///
/// The first shell in `shells` is conventionally the outer bounding shell;
/// subsequent shells are void (cavity) shells.
#[derive(Clone, Debug)]
pub struct Solid {
    pub shells: Vec<ShellId>,
}
