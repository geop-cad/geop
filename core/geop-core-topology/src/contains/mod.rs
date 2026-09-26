//! Topological containment queries: "is this point inside this face/shell?"
//! — distinct from `crate::contains`, which tests raw-geometry ("does this
//! 3-D point lie on this NURBS surface/curve") without any awareness of
//! trimming loops or topology.

pub mod face;
pub mod rng;
pub mod shell;
