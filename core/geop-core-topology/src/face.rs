use super::ids::ShellId;
use crate::boundary::{BoundaryIndex, BoundaryType};
use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::scalars::Scalar;

/// A trimmed patch of `surface`: everything inside `outer` and outside every
/// loop in `holes`.
///
/// The two are deliberately separate fields rather than one list with the
/// outer loop by convention at index 0. A face always has exactly one outer
/// boundary — that is what makes it a face — while holes are a genuinely
/// variable collection, and encoding that in the type means no code has to
/// re-establish it. It also removes a whole class of bug where a hole is
/// accidentally treated as the outer loop (or vice versa) after a
/// split/merge reorders the list.
#[derive(Clone, Debug)]
pub struct Face<S: Scalar> {
    pub surface: NurbSurface3D<S>,
    pub outer: BoundaryType,
    pub holes: Vec<BoundaryType>,
    pub shell: ShellId,
}

impl<S: Scalar> Face<S> {
    /// Every boundary, outer loop first.
    pub fn boundaries(&self) -> impl Iterator<Item = BoundaryType> + '_ {
        std::iter::once(self.outer).chain(self.holes.iter().copied())
    }

    /// Every boundary, outer loop first, for in-place rewriting — used by
    /// the operations that re-anchor or rename boundaries wholesale (a
    /// vertex being merged away, a loop's anchor coedge being deleted)
    /// without caring which kind each one is.
    pub fn boundaries_mut(&mut self) -> impl Iterator<Item = &mut BoundaryType> + '_ {
        std::iter::once(&mut self.outer).chain(self.holes.iter_mut())
    }

    /// The boundary `index` names, or `None` if it names a hole this face
    /// does not have.
    pub fn boundary(&self, index: BoundaryIndex) -> Option<BoundaryType> {
        match index {
            BoundaryIndex::Outer => Some(self.outer),
            BoundaryIndex::Hole(i) => self.holes.get(i).copied(),
        }
    }

    /// Re-anchor the boundary `index` names on a different loop. Needed after
    /// any restructuring that rewires `next`/`prev`, since the previous
    /// anchor may no longer sit on the ring the boundary now describes.
    pub fn set_boundary(&mut self, index: BoundaryIndex, boundary: BoundaryType) -> bool {
        match index {
            BoundaryIndex::Outer => {
                self.outer = boundary;
                true
            }
            BoundaryIndex::Hole(i) => match self.holes.get_mut(i) {
                Some(slot) => {
                    *slot = boundary;
                    true
                }
                None => false,
            },
        }
    }
}
