//! What steps build on: [`EntityRef`]s, entities of a part by name, and
//! what they are in it ([`ResolveEntity`], [`Aspects`]) as a [`Lookup`]
//! has it — a step's generator's, an editor's, a test's.

mod aspects;
mod build;
mod chain;
mod entity;
mod frame;
mod lookup;

pub use aspects::{Aspects, Role, describe_roles};
pub use build::{ModelStep, apply, apply_to_model, build_model};
pub use chain::{Chain, ChainCurve, Profile, piece_names};
pub use entity::{
    EntityRef, INSTANCE_SEPARATOR, NamedCurve, ORIGIN, ResolveEntity, frame_along, instance_path,
    origin_axis, origin_plane, resolve_bodies,
};
pub use frame::across;
pub use lookup::{Build, BuildReader, BuildView, Input, Lookup, Scope, Seen};
