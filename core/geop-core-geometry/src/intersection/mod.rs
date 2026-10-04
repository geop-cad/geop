mod coincidence;
pub mod curve_curve;
pub mod curve_surface;

pub use coincidence::{Hit, Overlap};
pub use curve_curve::{curve_curve_intersect, curve_curve_overlaps_and_crossings};
pub use curve_curve::refine_crossing as refine_curve_curve_crossing;
pub use curve_surface::{
    curve_surface_crossings, curve_surface_intersect, curve_surface_overlaps, refine_crossing,
};

/// Result of a subdivision-based intersection search: either the complete,
/// finite set of distinct solutions (fewer than the caller's requested
/// `max_solutions`), or a signal that the two shapes coincide/overlap over a
/// shared region rather than meeting at finitely many isolated points.
///
/// Reaching exactly `max_solutions` (`> 0`) is the established signal for
/// coincidence throughout this module — see [`curve_curve::curve_curve_intersect`]'s
/// own doc comment for why the search's breadth-first-by-level queue
/// ordering specifically makes that reliable (an evenly-spread set of
/// leaves across the *whole* shared region, not just a lucky cluster of
/// adjacent ones). This enum makes that convention explicit and
/// type-checked, in place of a `result.len() >= max_solutions` comparison
/// every caller previously had to remember to make.
///
/// [`Intersections::Coincident`] still carries the (capped, evenly-spread)
/// solutions found before the cap was hit — some callers detect coincidence
/// some other way and just want every point regardless (see
/// `geop-ops-booleans::remesh::remesh_edges_x_edges`); others want to reuse
/// them for a follow-up geometric test instead of re-deriving them (see
/// `remesh_edges_x_faces::find_coincident_pair`). Use [`Intersections::into_vec`]
/// to get the solutions either way, or match to tell the two cases apart.
#[derive(Debug, Clone)]
pub enum Intersections<T> {
    /// The complete set of distinct solutions — strictly fewer than the
    /// requested `max_solutions` (or `max_solutions` was `0`).
    Found(Vec<T>),
    /// `max_solutions` (`> 0`) distinct solutions were found before the
    /// search exhausted the domain: read as coincidence (or, for a partial
    /// overlap, an extended shared region), not a finite crossing count.
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
