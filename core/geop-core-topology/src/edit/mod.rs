//! Incremental mutation of an existing [`crate::Model`]: unlike
//! `Model`'s own `insert_*` methods (which only ever add a brand-new,
//! unconnected entity), everything here restructures or replaces what's
//! already there.

mod assemble_solid;
mod merge_edge;
mod merge_vertex;
pub(crate) mod reverse_face;
pub mod splice_edge_into_face;
mod split_edge_at_vertex;
