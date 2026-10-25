pub mod aabb;
pub mod contains;
mod exact;
mod fat_line;
pub mod intersection;
mod knot_insertion;
pub mod meet;
pub mod nearest;
pub mod nurb_curve;
pub mod nurb_surface;
pub mod search;
pub mod shape;
mod spline;
#[cfg(test)]
pub(crate) mod test_fixtures;
