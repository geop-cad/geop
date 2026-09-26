//! Small, standalone checks for the euler operators' own arguments —
//! e.g. "does this pcurve actually start/end where the caller claims" —
//! as opposed to [`super::validation`], which checks a whole [`super::Model`]
//! for internal consistency. Each check lives in its own file.

mod curve_start_and_end;
mod different_loop;
mod pcurve_start_and_end;
mod same_loop;

pub use curve_start_and_end::validate_curve_start_and_end;
pub use different_loop::validate_different_loop;
pub use pcurve_start_and_end::validate_pcurve_start_and_end;
pub use same_loop::validate_same_loop;
