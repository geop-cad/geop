mod coincidence;
pub mod curve_curve;
pub mod curve_surface;

pub use coincidence::{Hit, Overlap};
pub use curve_curve::refine_crossing as refine_curve_curve_crossing;
pub use curve_curve::{curve_curve_intersect, curve_curve_overlaps_and_crossings};
pub use curve_surface::{
    curve_surface_crossings, curve_surface_intersect, curve_surface_overlaps, refine_crossing,
};

/// Result of a subdivision-based intersection search: either the finite set
/// of distinct solutions, or a signal that the two shapes coincide/overlap
/// over a shared region rather than meeting at finitely many isolated
/// points.
///
/// Overlaps are found directly (see [`curve_curve::curve_curve_intersect`]
/// and [`curve_surface::curve_surface_intersect`]), not inferred from a
/// search reaching `max_solutions`.
///
/// [`Intersections::Coincident`] carries, up to `max_solutions`, the
/// overlaps' ends, the isolated crossings and points spread evenly over the
/// overlaps — some callers detect coincidence some other way and just want
/// every point regardless (see
/// `geop-ops-booleans::remesh::remesh_edges_x_edges`); others want to reuse
/// them for a follow-up geometric test instead of re-deriving them (see
/// `remesh_edges_x_faces::find_coincident_pair`). Use [`Intersections::into_vec`]
/// to get the solutions either way, or match to tell the two cases apart.
#[derive(Debug, Clone)]
pub enum Intersections<T> {
    /// The distinct isolated solutions: the shapes overlap nowhere.
    Found(Vec<T>),
    /// The shapes overlap along a shared region: points of it, and the
    /// isolated solutions besides.
    Coincident(Vec<T>),
}

impl<T> Intersections<T> {
    /// `true` for [`Intersections::Coincident`].
    pub fn is_coincident(&self) -> bool {
        matches!(self, Intersections::Coincident(_))
    }

    /// The solutions found, regardless of variant.
    pub fn into_vec(self) -> Vec<T> {
        match self {
            Intersections::Found(v) | Intersections::Coincident(v) => v,
        }
    }

    /// The solutions found, regardless of variant.
    pub fn as_slice(&self) -> &[T] {
        match self {
            Intersections::Found(v) | Intersections::Coincident(v) => v,
        }
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.as_slice().is_empty()
    }
}
